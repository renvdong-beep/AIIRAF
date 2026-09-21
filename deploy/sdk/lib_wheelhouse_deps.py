#!/usr/bin/env python3
"""wheelhouse 传递依赖闭包解析（候选清单模式）。

================================ 设计约束（先读） ================================
1. 本模块**只读**：不写 `config/sdk/package_matrix.yaml`，不写正式 wheelhouse
   （`build/wheelhouse/<target-id>/`），不安装任何东西。
   候选 wheel 只落到 `--work-dir` 指定的候选暂存目录（默认 build/iraf-24h/ 下）。
   回填声明必须由人工确认后执行，之后重跑 `fetch_wheelhouse.sh` 才是"真抓取"。
2. **fail-closed**：解析不出 METADATA、Requires-Dist 无法解析、marker 变量未知、
   索引不可达、无匹配 wheel，都显式失败（退出码 3）或显式标注 `unresolved` 原因，
   绝不静默丢弃依赖、绝不猜版本。
3. 目标环境（python_version / sys_platform / platform_machine）全部来自声明矩阵，
   不在代码里写死。`python_full_version` 无法在不假设补丁号的前提下求值 —— 若某个
   依赖的 marker 真的用到它，显式失败而不是替使用者假设。

用法（一般经 deploy/sdk/resolve_wheelhouse_deps.sh 调用）：
  /usr/bin/python3 deploy/sdk/lib_wheelhouse_deps.py \\
      --matrix config/sdk/package_matrix.yaml \\
      --target aarch64-manylinux_2_28-cp310 \\
      --output build/iraf-24h/21-dep-closure \\
      [--offline] [--max-depth 5] [--include-extra <name>]...
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from email.parser import Parser
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lib_wheel_spec as wheelspec  # noqa: E402  共享：标签语义与版本键（唯一实现）

try:  # packaging 是硬依赖：marker 求值与 wheel 标签匹配都不能靠手写正则
    from packaging.markers import UndefinedEnvironmentName
    from packaging.requirements import InvalidRequirement, Requirement
    from packaging.tags import Tag, compatible_tags, cpython_tags
    from packaging.utils import InvalidWheelFilename, parse_wheel_filename
except ImportError as exc:  # pragma: no cover - 由 sh 入口的预检同样覆盖
    raise SystemExit(f"[预检失败] 缺少 packaging 模块：{exc}；请安装 python3-packaging 后重试")

SCHEMA = "iraf.wheelhouse-deps/v1"
DEFAULT_MAX_DEPTH = 5


class ResolveError(Exception):
    """显式失败：调用方必须转换为非零退出码，不得吞掉。"""


# --------------------------------------------------------------------------- 矩阵

def load_matrix(path: Path) -> dict:
    if not path.is_file():
        raise ResolveError(f"矩阵不存在：{path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ResolveError(f"矩阵必须是对象：{path}")
    return data


def target_config(matrix: dict, target_id: str) -> dict:
    for item in matrix.get("targets") or []:
        if str(item.get("id")) == target_id:
            return item
    raise ResolveError(
        f"矩阵未声明目标 {target_id}；已声明：{[t.get('id') for t in matrix.get('targets') or []]}"
    )


def build_environment(target: dict) -> dict:
    """由声明的 python_tag / 平台标签推导 marker 求值环境；不可推导即显式失败。"""
    python_tag = str(target.get("python_tag") or "")
    if not python_tag.startswith("cp"):
        raise ResolveError(
            f"python_tag={python_tag!r} 无法推导 python_version（只支持 cpXY 形式）；"
            "marker 求值不能靠猜，请修正矩阵声明"
        )
    major, minor = python_tag[2], python_tag[3:]
    if not minor.isdigit():
        raise ResolveError(f"python_tag={python_tag!r} 的次版本号无法解析（只支持 cpXY）")
    arch = str(target.get("arch") or "")
    machine = {"aarch64": "aarch64", "arm64": "aarch64", "x86_64": "x86_64"}.get(arch)
    if machine is None:
        raise ResolveError(f"arch={arch!r} 无法推导 platform_machine；请修正矩阵或扩展映射")
    return {
        "python_version": f"{major}.{minor}",
        "sys_platform": "linux",
        "platform_system": "Linux",
        "platform_machine": machine,
        "os_name": "posix",
        "implementation_name": "cpython",
        "platform_python_implementation": "CPython",
        "extra": "",
    }


# --------------------------------------------------------------------- wheel 解析

def compatible_platform_tags(platform_tag: str) -> list:
    """声明平台的兼容阶梯（**委托 `lib_wheel_spec`，抓取侧与解析侧共用一份实现**）。"""
    return wheelspec.compatible_platform_tags(platform_tag)


def acceptable_tags(target: dict) -> frozenset:
    """目标可接受标签集（委托 `lib_wheel_spec.acceptable_tags`，`packaging` 同源）。"""
    return wheelspec.acceptable_tags(
        str(target.get("python_tag") or ""),
        str(target.get("platform_tag") or ""),
    )


def wheel_metadata_text(path: Path) -> str:
    try:
        with zipfile.ZipFile(path) as zf:
            entries = [n for n in zf.namelist() if n.endswith(".dist-info/METADATA")]
            if not entries:
                raise ResolveError(f"{path.name} 内没有 <dist>-<ver>.dist-info/METADATA（不是合法 wheel？）")
            if len(entries) > 1:
                raise ResolveError(f"{path.name} 内有多个 METADATA：{entries}（无法判定哪份为准）")
            return zf.read(entries[0]).decode("utf-8", errors="replace")
    except zipfile.BadZipFile as exc:
        raise ResolveError(f"{path.name} 不是合法 zip/wheel：{exc}") from exc


MARKER_VARIABLES = (
    "python_version", "python_full_version", "os_name", "sys_platform", "platform_release",
    "platform_system", "platform_version", "platform_machine", "platform_python_implementation",
    "implementation_name", "implementation_version", "extra",
)


def _referenced_marker_variables(marker_text: str) -> set:
    import re

    return {name for name in MARKER_VARIABLES if re.search(rf"\b{name}\b", marker_text)}


def _assert_marker_is_evaluable(marker_text: str, env: dict, source: str) -> None:
    """packaging 的 Marker.evaluate 会并入宿主默认环境，未声明的变量会被宿主值悄悄填上。

    跨架构解析不能接受这种"宿主兜底"：凡 marker 用到的变量必须由我们按目标声明提供，
    否则显式失败，交由人工决定（例如 python_full_version 无法在不假设补丁号时求值）。
    """
    referenced = _referenced_marker_variables(marker_text)
    missing = sorted(name for name in referenced if name not in env)
    if missing:
        raise ResolveError(
            f"{source} 的 marker 需要未声明的目标环境变量 {missing}：{marker_text!r}；"
            "不得用宿主环境兜底，请在声明中提供或改由人工裁决"
        )


def _extra_matches(marker_text: str, extra: str) -> bool:
    """marker 中是否显式要求了某个 extra（兼容单/双引号）。"""
    return f'extra == "{extra}"' in marker_text or f"extra == '{extra}'" in marker_text


def parse_requirements(metadata_text: str, source: str, env: dict, include_extras: set) -> tuple:
    """返回 (生效依赖 Requirement 列表, 因 marker/extras 被排除的原文列表)。"""
    msg = Parser().parsestr(metadata_text)
    raw = msg.get_all("Requires-Dist") or []
    active, skipped = [], []
    for item in raw:
        try:
            req = Requirement(item)
        except InvalidRequirement as exc:
            raise ResolveError(f"{source} 的 Requires-Dist 无法解析：{item!r}（{exc}）") from exc
        if req.marker is not None:
            marker_text = str(req.marker)
            # 先做"变量是否都由我们声明"的守卫：packaging 会并入宿主默认环境，
            # 未声明的变量（如 python_full_version）会被宿主值悄悄填上，跨架构解析不能接受。
            _assert_marker_is_evaluable(marker_text, env, source)
            if "extra" in marker_text:
                # extras 默认不生效：只有调用方显式 --include-extra 时才纳入
                if not any(_extra_matches(marker_text, e) for e in include_extras):
                    skipped.append(item)
                    continue
                eval_env = dict(env, extra=next(
                    (e for e in include_extras if _extra_matches(marker_text, e)), ""))
            else:
                eval_env = env
            try:
                if not req.marker.evaluate(eval_env):
                    skipped.append(item)
                    continue
            except UndefinedEnvironmentName as exc:
                raise ResolveError(
                    f"{source} 的 marker 需要本模块无法声明式求值的变量：{item!r}（{exc}）"
                ) from exc
        active.append(req)
    return active, skipped


def canonical(name: str) -> str:
    return str(name).lower().replace("_", "-").replace(".", "-")


# ------------------------------------------------------------------- 索引与下载

class IndexFetcher:
    """PEP 503 索引读取 + 候选 wheel 下载（标准库，不调用 pip）。"""

    def __init__(self, index_url: str, tags: frozenset, work_dir: Path):
        self.index_url = index_url.rstrip("/") + "/"
        self.tags = tags
        self.work_dir = work_dir
        self.work_dir.mkdir(parents=True, exist_ok=True)

    def _get(self, url: str) -> bytes:
        try:
            with urllib.request.urlopen(url, timeout=30) as resp:
                return resp.read()
        except (urllib.error.URLError, OSError) as exc:
            raise ResolveError(f"索引请求失败：{url}（{exc}）") from exc

    def candidates(self, package: str) -> list:
        """返回按版本降序排列、标签兼容的候选 [(version, filename, url)]。

        注：预发布版本**在此处仍会返回**，择优时由 select_candidate() 按
        "版本约束是否显式允许预发布"决定是否排除——与 pip 的默认语义一致。
        """
        page = urllib.parse.urljoin(self.index_url, f"{package}/")
        html = self._get(page).decode("utf-8", errors="replace")
        found = []
        for href in _iter_hrefs(html):
            filename = href.split("#")[0].rsplit("/", 1)[-1]
            if not filename.endswith(".whl"):
                continue
            try:
                name, version, _build, tags = parse_wheel_filename(filename)
            except InvalidWheelFilename:
                continue
            if canonical(str(name)) != canonical(package):
                continue
            if not (tags & self.tags):
                continue
            found.append((str(version), filename, urllib.parse.urljoin(page, href.split("#")[0])))
        found.sort(key=lambda item: _version_key(item[0]), reverse=True)
        return found

    def download(self, package: str, url: str, filename: str) -> Path:
        dest = self.work_dir / f"{canonical(package)}-{filename}"
        if dest.is_file():
            return dest
        dest.write_bytes(self._get(url))
        return dest


def _iter_hrefs(html: str):
    import re

    return re.findall(r'href="([^"]+)"', html)


def _version_key(version: str):
    """按 PEP 440 排序（**委托 `lib_wheel_spec.version_key`**）。"""
    try:
        return wheelspec.version_key(version)
    except wheelspec.WheelSpecError as exc:
        raise ResolveError(str(exc)) from exc


def select_candidate(candidates: list, specifiers: list, target: dict | None = None) -> tuple:
    """从候选中择优：版本约束过滤 → 正式版优先 → 版本降序 → **同版本内 kind 偏好** → 文件名兜底。

    返回 (chosen, note)；chosen 为 None 表示无可用候选。kind 由 `lib_wheel_spec.classify`
    判定（exact > compatible > pure_python），因此本函数给出的候选与 `fetch_wheelhouse.sh`
    实际抓取保持同一套语义与同一套偏好 —— 这是"候选清单 == 可安装 wheelhouse"的前提。
    """
    from packaging.specifiers import SpecifierSet

    if not candidates:
        return None, "索引中无标签可用的候选 wheel"
    active_specs = [s for s in specifiers if s]
    pool = candidates
    if active_specs:
        spec = SpecifierSet(",".join(active_specs))
        pool = [c for c in pool if spec.contains(c[0], prereleases=True)]
        if not pool:
            return None, f"候选均不满足版本约束 {active_specs}"

    stable = [c for c in pool if not wheelspec.is_prerelease(c[0])]
    if stable:
        note = f"取满足约束的最高稳定版（已排除 {len(pool) - len(stable)} 个预发布）"
        pool = stable
    else:
        note = "⚠ 仅有预发布版本可用（pip 默认不会选它，需人工确认后再回填）"

    def kind_rank(entry):
        filename = entry[1]
        if not target:
            return 9
        verdict = wheelspec.classify(filename, str(target.get("python_tag") or ""),
                                     str(target.get("platform_tag") or ""),
                                     tuple(target.get("reject_platform_tags") or ()))
        return wheelspec.KIND_RANK.get(verdict["kind"], 9) if verdict["accepted"] else 9

    pool = sorted(pool, key=lambda c: c[1])                                  # 文件名：确定性
    pool = sorted(pool, key=lambda c: kind_rank(c))                          # kind 偏好
    pool = sorted(pool, key=lambda c: wheelspec.version_key(c[0]), reverse=True)  # 版本降序
    return pool[0], note


def _spec_mentions_prerelease(specifier: str) -> bool:
    import re

    return bool(re.search(r"[0-9](a|b|rc|dev)[0-9]", specifier or ""))


# ------------------------------------------------------------------------ 闭包

def resolve_closure(wheelhouse: Path, target: dict, env: dict, fetcher, max_depth: int,
                    include_extras: set) -> dict:
    wheels = sorted(p for p in wheelhouse.glob("*.whl"))
    if not wheels:
        raise ResolveError(f"wheelhouse 目录内没有 wheel：{wheelhouse}")

    provided = {}          # canonical name -> {version, filename}
    first_level = {}       # canonical name -> {"requires": [...], "source": filename}
    for whl in wheels:
        text = wheel_metadata_text(whl)
        msg = Parser().parsestr(text)
        name = canonical(msg.get("Name") or "")
        if not name:
            raise ResolveError(f"{whl.name} 的 METADATA 缺 Name 字段")
        provided[name] = {"version": msg.get("Version"), "filename": whl.name}
        active, skipped = parse_requirements(text, whl.name, env, include_extras)
        first_level[name] = {
            "requires": [{"name": canonical(r.name), "specifier": str(r.specifier)} for r in active],
            "skipped": skipped,
        }

    # 广度优先递归：缺的包从索引取候选 → 读它的 METADATA → 继续展开
    missing = {}
    queue = []
    for name, info in first_level.items():
        for req in info["requires"]:
            if req["name"] not in provided:
                queue.append((req["name"], req["specifier"], name, 1))

    seen_depth = {}
    while queue:
        name, specifier, required_by, depth = queue.pop(0)
        if name in provided:
            continue
        if depth > max_depth:
            missing.setdefault(name, {"required_by": [], "specifier": [], "status": "depth_exceeded"})
            missing[name]["required_by"].append(required_by)
            continue
        record = missing.setdefault(name, {
            "required_by": [], "specifier": [], "status": "unresolved", "reason": None,
            "candidate": None, "requires": [],
        })
        if required_by not in record["required_by"]:
            record["required_by"].append(required_by)
        if specifier and specifier not in record["specifier"]:
            record["specifier"].append(specifier)
        if record["candidate"] is not None or record["status"] != "unresolved":
            continue
        if fetcher is None:
            record["reason"] = "offline：未探测索引（--offline 模式，只报告一级缺失）"
            continue
        candidates = fetcher.candidates(name)
        # 择优必须与抓取侧同规则（lib_wheel_spec.classify + 同版本内 kind 偏好），
        # 否则"人工确认的候选清单"与实际抓取仍可能不是同一个 wheel。
        chosen, note = select_candidate(candidates, record["specifier"], target=target)
        if chosen is None:
            record["reason"] = note
            continue
        version, filename, url = chosen
        record["candidate"] = {"version": version, "filename": filename, "url": url,
                               "candidates_total": len(candidates), "selection_note": note}
        path = fetcher.download(name, url, filename)
        text = wheel_metadata_text(path)
        active, skipped = parse_requirements(text, filename, env, include_extras)
        record["requires"] = [{"name": canonical(r.name), "specifier": str(r.specifier)} for r in active]
        record["status"] = "resolved"
        record["metadata_sha256"] = hashlib.sha256(text.encode("utf-8")).hexdigest()
        seen_depth[name] = depth
        for req in record["requires"]:
            if req["name"] not in provided and req["name"] not in missing:
                queue.append((req["name"], req["specifier"], name, depth + 1))

    for name, record in missing.items():
        if record["status"] == "unresolved" and record["reason"] is None:
            record["reason"] = "未解析到候选"

    return {
        "schema_version": SCHEMA,
        "candidate_only": True,
        "matrix_modified": False,
        "wheelhouse_written": False,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S+08:00"),
        "target": {k: target.get(k) for k in ("id", "arch", "python_tag", "platform_tag")},
        "environment": env,
        "wheelhouse_dir": str(wheelhouse),
        "provided": provided,
        "first_level": first_level,
        "missing": missing,
        "summary": {
            "provided_count": len(provided),
            "missing_count": len(missing),
            "missing_resolved": sorted(n for n, r in missing.items() if r["status"] == "resolved"),
            "missing_unresolved": sorted(n for n, r in missing.items() if r["status"] != "resolved"),
            "requires_dist_total": sum(len(v["requires"]) for v in first_level.values()),
            "requires_dist_active": sum(
                1 for v in first_level.values() for _ in v["requires"]
            ),
        },
    }


def write_report(report: dict, outdir: Path) -> tuple:
    outdir.mkdir(parents=True, exist_ok=True)
    json_path = outdir / "candidates.json"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# wheelhouse 传递依赖候选清单（只读产物，**未回填矩阵**）",
        "",
        f"- 目标：`{report['target']['id']}`（python_tag `{report['target']['python_tag']}` / "
        f"platform_tag `{report['target']['platform_tag']}`）",
        f"- 生成时间：{report['generated_at']}",
        f"- 现有 wheelhouse：`{report['wheelhouse_dir']}` —— 已覆盖包 {report['summary']['provided_count']} 个",
        f"- 生效 Requires-Dist 条目：{report['summary']['requires_dist_active']} 条",
        f"- 缺失（未覆盖）包：{report['summary']['missing_count']} 个"
        f"（已解析出候选 {len(report['summary']['missing_resolved'])} 个）",
        "",
        "## 缺失包与候选（需人工确认后才回填矩阵）",
        "",
    ]
    if report["missing"]:
        lines += ["| 包 | 被谁要求 | 版本约束 | 候选版本 | 候选 wheel | 递归依赖 |", "|---|---|---|---|---|---|"]
        for name in sorted(report["missing"]):
            rec = report["missing"][name]
            cand = rec.get("candidate") or {}
            deps = ", ".join(f"{d['name']}{d['specifier']}" for d in rec.get("requires") or []) or "-"
            lines.append(
                f"| `{name}` | {', '.join('`'+r+'`' for r in rec['required_by'])} | "
                f"{', '.join(rec['specifier']) or '-'} | {cand.get('version') or '-'} | "
                f"`{cand.get('filename') or '-'}` | {deps} |"
            )
        unresolved = [n for n in sorted(report["missing"]) if report["missing"][n]["status"] != "resolved"]
        if unresolved:
            lines += ["", "### 未解析的包（必须先解决，不允许静默跳过）", ""]
            for name in unresolved:
                lines.append(f"- `{name}`：{report['missing'][name].get('reason')}")
    else:
        lines += ["（无缺失：现有 wheelhouse 已覆盖全部生效依赖）"]
    lines += [
        "",
        "## 回填流程（人工确认后执行）",
        "",
        "1. 人工确认上表候选（版本、来源、递归依赖）；",
        "2. 把确认后的包名写入 `config/sdk/package_matrix.yaml` 的 `targets[].wheels`；",
        "3. 重跑 `bash deploy/sdk/fetch_wheelhouse.sh`，再跑目录级标签校验；",
        "4. 重新组装 bundle 并重跑安装 dry-run 验收。",
        "",
        f"本文件由 `deploy/sdk/lib_wheelhouse_deps.py` 生成；`candidate_only={report['candidate_only']}`、"
        f"`matrix_modified={report['matrix_modified']}`、`wheelhouse_written={report['wheelhouse_written']}`。",
    ]
    md_path = outdir / "summary.md"
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, md_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="wheelhouse 传递依赖闭包（候选清单模式，只读）")
    parser.add_argument("--matrix", type=Path, default=Path("config/sdk/package_matrix.yaml"))
    parser.add_argument("--target", required=True)
    parser.add_argument("--wheelhouse", type=Path, default=None,
                        help="缺省按 build/wheelhouse/<target-id> 推导")
    parser.add_argument("--output", type=Path, default=Path("build/iraf-24h/dep-closure"))
    parser.add_argument("--work-dir", type=Path, default=None, help="候选 wheel 暂存目录（不写正式 wheelhouse）")
    parser.add_argument("--max-depth", type=int, default=DEFAULT_MAX_DEPTH)
    parser.add_argument("--offline", action="store_true", help="不探测索引，只报告一级缺失")
    parser.add_argument("--include-extra", action="append", default=[])
    args = parser.parse_args(argv)

    try:
        matrix = load_matrix(args.matrix)
        target = target_config(matrix, args.target)
        env = build_environment(target)
        wheelhouse = args.wheelhouse or Path("build/wheelhouse") / args.target
        if not wheelhouse.is_dir():
            raise ResolveError(f"wheelhouse 目录不存在：{wheelhouse}")
        # 索引地址只在目标声明里（设计 §7）；缺声明即失败，不从别处"兜底"
        index_url = target.get("index_url")
        if not args.offline and not index_url:
            raise ResolveError("矩阵未声明 index_url，无法探测索引（可用 --offline 只报告一级缺失）")
        work_dir = args.work_dir or (args.output / "candidate-wheels")
        fetcher = None if args.offline else IndexFetcher(str(index_url), acceptable_tags(target), work_dir)
        report = resolve_closure(wheelhouse, target, env, fetcher, args.max_depth, set(args.include_extra))
        json_path, md_path = write_report(report, args.output)
    except ResolveError as exc:
        print(f"[依赖闭包解析失败] {exc}", file=sys.stderr)
        return 3

    summary = report["summary"]
    print(f"[wheelhouse 依赖闭包] 目标 {report['target']['id']}")
    print(f"  已覆盖包 {summary['provided_count']} 个；生效 Requires-Dist {summary['requires_dist_active']} 条")
    print(f"  缺失包 {summary['missing_count']} 个：{summary['missing_resolved'] or '（无）'}")
    if summary["missing_unresolved"]:
        print(f"  未解析：{summary['missing_unresolved']}（原因见报告）")
    print(f"  候选清单：{json_path}")
    print(f"  中文摘要：{md_path}")
    print("  注意：本产物只是候选；矩阵未改、正式 wheelhouse 未写。回填需人工确认。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
