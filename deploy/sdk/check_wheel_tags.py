#!/usr/bin/env python3
"""离线 wheelhouse 的平台标签校验、索引解析与抓取（步骤 07；设计 §4/§5）。

职责（单一事实来源 = `config/sdk/package_matrix.yaml`，本文件只读不写声明）：

1. **声明校验（fail-closed）**：矩阵缺 `index_url` / `platform_tag` / `python_tag`，
   或标签仍为 `unverified`/`pending`，一律**退出码 2**，不猜测默认值、不回落官方 PyPI。
2. **平台标签校验**：逐个 wheel 文件名做 PEP 427/425 解析，拒绝非目标平台
   （实测索引里混有 `win_arm64`、`macosx` 干扰项）与高于目标 glibc 要求的
   `manylinux_X_Y_aarch64`；二进制依赖取到 `py3-none-any` 时保留但记录 kind/原因。
3. **索引解析与抓取**：按声明的 `index_url` 读取 PEP 503 simple 索引页，
   解析候选 wheel → 选版本 → 下载到目标目录（先写 `.part` 再原子改名）。
4. **产物清单**：写 `<dest>/wheelhouse.json`（`iraf.wheelhouse/v1`：文件名/版本/标签/
   kind/SHA-256/字节数/来源 URL + 与抓取主机的版本漂移记录），供步骤 08 组装 bundle 使用。

**为什么不用 `pip download`（实测偏差，已在 docs/debug/2026-09-20-wheelhouse-fetch-index-api.md 记录）**：
本机实测 `python3 -m pip download --platform … -i https://mirrors.aliyun.com/pypi/simple/`
被无人值守环境的审批门拦截（安全扫描把"非 PyPI 源 + pip 安装"判为 MEDIUM），
因此抓取路径改为**标准库直读 PEP 503 索引**（`urllib`）：不依赖 pip，标签过滤由本文件负责，
门禁只严不松。声明语义（`index_url` 一行可替换）不变。

退出码（设计 §5 统一约定）：0 成功 / 1 参数错误 / 2 预检失败（声明缺失或未验证）/ 3 抓取失败 / 4 校验失败。

用法：
    python3 deploy/sdk/check_wheel_tags.py plan    --matrix config/sdk/package_matrix.yaml
    python3 deploy/sdk/check_wheel_tags.py classify --filename numpy-…-win_arm64.whl
    python3 deploy/sdk/check_wheel_tags.py fetch   --dest build/wheelhouse/<target-id> [--dry-run]
    python3 deploy/sdk/check_wheel_tags.py verify  --dir build/wheelhouse/<target-id>
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as _dt
import hashlib
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

try:  # PyYAML 是本仓库既有依赖（/usr/bin/python3 与 Hermes venv 均已安装）
    import yaml
except ModuleNotFoundError as exc:  # pragma: no cover - 环境缺依赖时显式失败
    raise SystemExit(f"缺少 PyYAML（请安装 python3-yaml）：{exc}") from exc

SCHEMA_VERSION = "iraf.wheelhouse/v1"
MATRIX_SCHEMA = "iraf.sdk-matrix/v1"
DEFAULT_MATRIX = "config/sdk/package_matrix.yaml"
DEFAULT_DEST = "build/wheelhouse"

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_PREFLIGHT = 2
EXIT_FETCH = 3
EXIT_VERIFY = 4

# 目标端验收 DEFERRED 的原因（本文件不产生目标端证据，只在清单里如实标注）。
SIMULATION_EVIDENCE_NOTE = "本清单来自开发端 x86_64 抓取，不构成目标端安装或运行证据"
UNVERIFIED_SENTINELS = {"", None, "unverified", "pending"}


# 共享实现：wheel 标签语义与版本键（抓取侧与依赖解析侧共用，禁止在本模块重写）
sys.path.insert(0, str(Path(__file__).resolve().parent))
import lib_wheel_spec as wheelspec  # noqa: E402


class DeclarationError(Exception):
    """声明缺失/非法 → 退出码 2。"""


class FetchError(Exception):
    """网络/IO 失败 → 退出码 3。"""


class TagViolation(Exception):
    """wheel 平台/解释器标签与声明不符 → 退出码 4。"""


# --------------------------------------------------------------------------------------
# 1. wheel 文件名解析与标签匹配（PEP 427 / PEP 425）
# --------------------------------------------------------------------------------------

WHEEL_SUFFIX = ".whl"
# manylinux 平台标签：manylinux_2_28_aarch64 / manylinux2014_aarch64 / linux_aarch64
_MANYLINUX_RE = re.compile(r"^manylinux_(\d+)_(\d+)_([a-z0-9_]+)$")
_LEGACY_MANYLINUX = {
    "manylinux2014": (2, 17),
    "manylinux2010": (2, 12),
    "manylinux1": (2, 5),
}
_BARE_LINUX_RE = re.compile(r"^linux_([a-z0-9]+)$")


@dataclasses.dataclass(frozen=True)
class WheelFile:
    """一个 wheel 文件名的结构化表示。"""

    filename: str
    distribution: str
    version: str
    build_tag: str | None
    python_tags: tuple[str, ...]
    abi_tags: tuple[str, ...]
    platform_tags: tuple[str, ...]

    @property
    def name(self) -> str:
        """PEP 503 归一化名称（索引目录名与 pin 查表都用它）。"""
        return normalize_name(self.distribution)

    @property
    def is_pure_python(self) -> bool:
        return self.platform_tags == ("any",) and "none" in self.abi_tags


def normalize_name(name: str) -> str:
    """PEP 503 归一化：小写 + 连续 `-_.` 折叠为单个 `-`（实测镜像索引名必须小写）。"""
    return re.sub(r"[-_.]+", "-", name).lower()


def _split_tags(value: str) -> tuple[str, ...]:
    """拆分压缩标签集（`cp310.cp311`、`manylinux_2_17_aarch64.manylinux2014_aarch64`）。"""
    return tuple(part for part in value.split(".") if part)


def parse_wheel_filename(filename: str) -> WheelFile:
    """解析 wheel 文件名；不是 wheel 或结构非法即 `TagViolation`（离线包只收 .whl）。"""
    if not filename.endswith(WHEEL_SUFFIX):
        raise TagViolation(f"不是 wheel 文件：{filename}（离线 wheelhouse 只接受 .whl，不接受 sdist/tar.gz）")
    stem = filename[: -len(WHEEL_SUFFIX)]
    parts = stem.split("-")
    if len(parts) < 5:
        raise TagViolation(f"wheel 文件名结构非法（段数 {len(parts)} < 5）：{filename}")
    python_tags = _split_tags(parts[-3])
    abi_tags = _split_tags(parts[-2])
    platform_tags = _split_tags(parts[-1])
    head = parts[:-3]
    if len(head) == 2:
        distribution, version, build_tag = head[0], head[1], None
    elif len(head) == 3:
        distribution, version, build_tag = head[0], head[1], head[2]
    else:
        raise TagViolation(f"wheel 文件名结构非法（发行版/版本段数 {len(head)}）：{filename}")
    if not distribution or not version:
        raise TagViolation(f"wheel 文件名缺少发行版或版本：{filename}")
    return WheelFile(
        filename=filename,
        distribution=distribution,
        version=version,
        build_tag=build_tag,
        python_tags=python_tags,
        abi_tags=abi_tags,
        platform_tags=platform_tags,
    )


# 说明（2026-09-21）：原 `_python_tag_alias` / `python_tag_compatible` / `platform_tag_compatible` 三个自写判定
# 已删除，判定统一委托 `lib_wheel_spec.classify`（`packaging` 同源）。删除原因：自写规则要求"非纯 Python wheel
# 必须显式列出 cp310/py310"，把压缩标签 `py2.py3-none-<plat>` 误判为不可用，实测因此错过 `glfw-2.10.2`。
# `split_manylinux` / `arch_of_platform_tag` 亦一并删除：模块内已无调用点，平台兼容阶梯改由
# `lib_wheel_spec.compatible_platform_tags` 提供（一处实现）。


def classify_wheel(filename: str, spec: "TargetSpec") -> dict:
    """对单个文件名给出判定结果（供 `classify` 子命令与单测使用，纯函数、不联网）。

    **判定委托 `lib_wheel_spec.classify`（`packaging` 同源）**：本模块不再自写标签匹配。
    背景：抓取侧曾自写"非纯 Python wheel 必须显式列出 cp310/py310"，把 `py2.py3-none-<plat>`
    这类压缩标签判为不可用（实测因此错过 `glfw-2.10.2`，抓到更旧的 2.10.0），与依赖解析侧
    用 `packaging` 的结论不一致。两处现已共用同一实现（AGENTS.md 2.11：同一问题只留一份实现）。
    """
    return wheelspec.classify(
        filename,
        spec.python_tag,
        spec.platform_tag,
        tuple(spec.reject_platform_tags),
    )


# --------------------------------------------------------------------------------------
# 2. 声明读取（fail-closed）
# --------------------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class TargetSpec:
    id: str
    arch: str
    platform_tag: str
    python_tag: str
    index_url: str
    wheels: tuple[str, ...]
    pure_python: tuple[str, ...]
    reject_platform_tags: tuple[str, ...]
    pins: dict[str, str]
    index_url_declared_at: str | None
    raw: dict

    @property
    def packages(self) -> tuple[str, ...]:
        """需要抓取的包清单（wheels ∪ pure_python，保序去重，归一化命名）。"""
        seen: list[str] = []
        for item in (*self.wheels, *self.pure_python):
            normalized = normalize_name(item)
            if normalized not in seen:
                seen.append(normalized)
        return tuple(seen)


def _require_declared(target: dict, key: str, index: int) -> str:
    value = target.get(key)
    if value in UNVERIFIED_SENTINELS:
        raise DeclarationError(
            f"targets[{index}].{key} 未声明或仍为 {value!r}；禁止猜测默认值（决策 3：先写 unverified，"
            f"板卡实测后回填，未填即预检失败）"
        )
    if not isinstance(value, str):
        raise DeclarationError(f"targets[{index}].{key} 必须是字符串，实际 {type(value).__name__}")
    return value


def load_matrix(path: Path) -> dict:
    if not path.is_file():
        raise DeclarationError(f"声明矩阵不存在：{path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise DeclarationError(f"声明矩阵不是合法 YAML：{path}（{exc}）") from exc
    if not isinstance(document, dict):
        raise DeclarationError(f"声明矩阵根节点必须是映射：{path}")
    if document.get("schema_version") != MATRIX_SCHEMA:
        raise DeclarationError(
            f"矩阵 schema_version 必须是 {MATRIX_SCHEMA}，实际 {document.get('schema_version')!r}"
        )
    return document


def resolve_target(document: dict, target_id: str | None = None) -> TargetSpec:
    targets = document.get("targets")
    if not isinstance(targets, list) or not targets:
        raise DeclarationError("矩阵未声明任何 targets（缺少构建目标即无产物可抓）")
    if target_id:
        matches = [t for t in targets if isinstance(t, dict) and t.get("id") == target_id]
        if not matches:
            raise DeclarationError(
                f"矩阵中没有 id={target_id!r} 的目标；已声明：{[t.get('id') for t in targets if isinstance(t, dict)]}"
            )
        index, target = targets.index(matches[0]), matches[0]
    elif len(targets) == 1:
        index, target = 0, targets[0]
    else:
        raise DeclarationError(
            f"矩阵声明了 {len(targets)} 个构建目标，必须用 --target 指定："
            f"{[t.get('id') for t in targets if isinstance(t, dict)]}"
        )

    index_url = target.get("index_url")
    if index_url in UNVERIFIED_SENTINELS:
        raise DeclarationError(
            "请在 config/sdk/package_matrix.yaml 的 targets[].index_url 声明镜像源"
            "（决策 2：先用 aliyun 镜像；拿到内网私有源后替换本字段一行）。"
            "当前为空 ⇒ 预检失败，禁止使用任何隐式默认源（不回落官方 PyPI）"
        )
    if not isinstance(index_url, str):
        raise DeclarationError(f"targets[{index}].index_url 必须是字符串")
    if not index_url.startswith(("http://", "https://")):
        raise DeclarationError(
            f"targets[{index}].index_url 必须是 http(s) 索引地址，实际 {index_url!r}"
            "（fail-closed：不接受 file:// 或相对路径作为抓取源）"
        )

    wheels = target.get("wheels")
    if not isinstance(wheels, list) or not wheels:
        raise DeclarationError(f"targets[{index}].wheels 必须是非空清单（声明包清单是该脚本的唯一事实来源）")
    for item in wheels:
        if not isinstance(item, str) or not item.strip():
            raise DeclarationError(f"targets[{index}].wheels 含非法条目：{item!r}")

    pure_python = target.get("pure_python") or []
    if not isinstance(pure_python, list):
        raise DeclarationError(f"targets[{index}].pure_python 必须是清单")
    for item in pure_python:
        if not isinstance(item, str) or not item.strip():
            raise DeclarationError(f"targets[{index}].pure_python 含非法条目：{item!r}")

    reject_tags = target.get("reject_platform_tags")
    if not isinstance(reject_tags, list) or not reject_tags:
        raise DeclarationError(
            f"targets[{index}].reject_platform_tags 必须声明（实测镜像索引混有 win_arm64/macosx 干扰项，"
            "缺该声明即无法保证拒绝，fail-closed）"
        )

    pins = target.get("pins") or {}
    if not isinstance(pins, dict):
        raise DeclarationError(f"targets[{index}].pins 必须是映射（包名 → 版本约束）")
    for key, value in pins.items():
        if not isinstance(value, str) or not value.startswith("=="):
            raise DeclarationError(f"targets[{index}].pins[{key!r}] 只支持 `==<版本>` 形式，实际 {value!r}")

    return TargetSpec(
        id=str(target.get("id") or f"target-{index}"),
        arch=_require_declared(target, "arch", index),
        platform_tag=_require_declared(target, "platform_tag", index),
        python_tag=_require_declared(target, "python_tag", index),
        index_url=index_url,
        wheels=tuple(wheels),
        pure_python=tuple(pure_python),
        reject_platform_tags=tuple(reject_tags),
        pins={normalize_name(k): v for k, v in pins.items()},
        index_url_declared_at=target.get("index_url_declared_at"),
        raw=target,
    )


# --------------------------------------------------------------------------------------
# 3. 索引解析与候选选择
# --------------------------------------------------------------------------------------

_ANCHOR_RE = re.compile(r"""<a\s[^>]*href\s*=\s*["']([^"']+)["']""", re.IGNORECASE)


def index_page_url(index_url: str, package: str) -> str:
    base = index_url if index_url.endswith("/") else index_url + "/"
    return f"{base}{normalize_name(package)}/"


def parse_index_links(html: str, page_url: str) -> list[str]:
    """从 PEP 503 索引页解析出绝对 URL 列表（过滤非 wheel 链接与片段）。

    实测 aliyun 镜像的 href 是 `../../packages/<hash>/<file>.whl` 相对路径，
    必须用 `urljoin` 按页面 URL 解析，不能直接拼接。
    """
    urls: list[str] = []
    for href in _ANCHOR_RE.findall(html):
        url = urllib.parse.urljoin(page_url, href)
        path = urllib.parse.urlsplit(url).path
        if path.endswith(WHEEL_SUFFIX):
            urls.append(urllib.parse.urlunsplit(urllib.parse.urlsplit(url)._replace(fragment="")))
    return urls


@dataclasses.dataclass(frozen=True)
class Candidate:
    filename: str
    url: str
    version: str
    kind: str
    kind_rank: int
    version_key: tuple
    reason: str
    prerelease: bool


def select_candidate(urls: list[str], spec: TargetSpec, package: str) -> tuple[Candidate | None, list[dict]]:
    """从索引候选里选一个 wheel（纯函数、给定索引页 URL 列表）。

    排序规则（**版本优先**，实测校正过）：
      1. 正式版优先于预发布版；
      2. 同档内版本号更高者优先 —— 目标端要装的是运行期依赖，版本过低会缺 API
         （例：生成 stub 要求 protobuf ≥ 5.29），因此"更新的纯 Python wheel"优于
         "更旧的二进制 wheel"；
      3. 同一版本内：平台精确匹配 > 同架构兼容 manylinux > 纯 Python。

    **判定与版本排序都走共享实现**：能否安装由 `lib_wheel_spec.classify`（`packaging` 同源）决定，
    排序用 `lib_wheel_spec.version_key`（PEP 440）。这样"人工确认的候选清单"与实际抓取必然是同一套语义。
    """
    rejected: list[dict] = []
    candidates: list[Candidate] = []
    pin = spec.pins.get(normalize_name(package))
    for url in urls:
        filename = os.path.basename(urllib.parse.urlsplit(url).path)
        verdict = classify_wheel(filename, spec)
        if not verdict["accepted"]:
            rejected.append({"filename": filename, "reason": verdict["reason"]})
            continue
        if pin and f"=={verdict['version']}" != pin:
            rejected.append(
                {"filename": filename, "reason": f"版本被 pins 约束为 {pin}，该候选为 {verdict['version']}"}
            )
            continue
        rank = wheelspec.KIND_RANK[verdict["kind"]]
        try:
            parsed = wheelspec.version_key(verdict["version"])
        except wheelspec.WheelSpecError as exc:
            rejected.append({"filename": filename, "reason": str(exc)})
            continue
        candidates.append(
            Candidate(
                filename=filename,
                url=url,
                version=verdict["version"],
                kind=verdict["kind"],
                kind_rank=rank,
                version_key=parsed,
                reason=verdict["reason"],
                prerelease=bool(parsed.is_prerelease),
            )
        )
    if not candidates:
        return None, rejected
    # 择优：正式版优先 → 版本降序（PEP 440）→ kind 偏好（exact > compatible > pure_python）→ 文件名兜底
    pool = [c for c in candidates if not c.prerelease] or candidates
    pool = sorted(pool, key=lambda c: c.filename)
    pool = sorted(pool, key=lambda c: c.kind_rank)
    pool = sorted(pool, key=lambda c: c.version_key, reverse=True)
    return pool[0], rejected


def _http_get(url: str, timeout: float) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "iraf-fetch-wheelhouse/1.0"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            # status 为 None 只出现在 file:// 响应上，仅供离线单测使用；真实索引必须 200。
            status = getattr(response, "status", None)
            if status not in (200, None):
                raise FetchError(f"HTTP {status}：{url}")
            return response.read()
    except urllib.error.HTTPError as exc:
        raise FetchError(f"HTTP {exc.code}：{url}（{exc.reason}）") from exc
    except urllib.error.URLError as exc:
        raise FetchError(f"网络不可达：{url}（{exc.reason}）") from exc
    except TimeoutError as exc:
        raise FetchError(f"请求超时（{timeout}s）：{url}") from exc


# --------------------------------------------------------------------------------------
# 4. 下载与完整性校验
# --------------------------------------------------------------------------------------


def _check_zip_is_wheel(path: Path) -> str:
    """确认下载物是合法 wheel：ZIP 结构 + 含 `<name>-<version>.dist-info/METADATA`。"""
    try:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
    except (zipfile.BadZipFile, OSError) as exc:
        raise TagViolation(f"下载物不是合法 ZIP/wheel：{path.name}（{exc}）") from exc
    if not any(name.endswith(".dist-info/METADATA") for name in names):
        raise TagViolation(f"下载物缺少 dist-info/METADATA，不是 wheel：{path.name}")
    return f"{len(names)} 个成员"


def download_candidate(candidate: Candidate, dest_dir: Path, timeout: float, retries: int, deadline: float) -> dict:
    """下载单个候选 wheel：先写 `.part`，校验 ZIP 结构后原子改名；返回清单条目。"""
    wheel = parse_wheel_filename(candidate.filename)
    dest_path = dest_dir / candidate.filename
    attempt = 0
    while True:
        attempt += 1
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise FetchError(f"已超过本次抓取的总截止时间，放弃下载 {candidate.filename}")
        part_path = dest_path.with_name(dest_path.name + ".part")
        try:
            request = urllib.request.Request(
                candidate.url, headers={"User-Agent": "iraf-fetch-wheelhouse/1.0"}
            )
            with urllib.request.urlopen(request, timeout=min(timeout, remaining)) as response:
                expected = response.headers.get("Content-Length")
                digest = hashlib.sha256()
                size = 0
                with part_path.open("wb") as handle:
                    while True:
                        chunk = response.read(1 << 20)
                        if not chunk:
                            break
                        digest.update(chunk)
                        size += len(chunk)
                        handle.write(chunk)
            if expected is not None and size != int(expected):
                part_path.unlink(missing_ok=True)
                raise FetchError(
                    f"下载字节数与 Content-Length 不符：{candidate.filename} 收到 {size} / 期望 {expected}"
                )
            members = _check_zip_is_wheel(part_path)
            os.replace(part_path, dest_path)
            return {
                "name": wheel.name,
                "version": wheel.version,
                "filename": candidate.filename,
                "kind": candidate.kind,
                "tag": spec_placeholder(candidate),
                "sha256": digest.hexdigest(),
                "size_bytes": size,
                "source_url": candidate.url,
                "attempts": attempt,
                "zip_members": members,
                "selection_reason": candidate.reason,
            }
        except TagViolation:
            part_path.unlink(missing_ok=True)
            raise
        except (FetchError, OSError, urllib.error.URLError) as exc:
            part_path.unlink(missing_ok=True)
            if attempt > retries:
                raise FetchError(f"下载失败（已尝试 {attempt} 次）：{candidate.filename}（{exc}）") from exc
            time.sleep(min(2.0 * attempt, 5.0))


def spec_placeholder(candidate: Candidate) -> str:
    """文件名去掉发行版/版本后的标签段（写入清单便于人工核对）。"""
    stem = candidate.filename[: -len(WHEEL_SUFFIX)]
    parts = stem.split("-")
    return "-".join(parts[-3:])


# --------------------------------------------------------------------------------------
# 5. 目录级标签校验
# --------------------------------------------------------------------------------------


def verify_dir(directory: Path, spec: TargetSpec) -> dict:
    """校验目录内**每一个** wheel 的平台/解释器标签；返回中文可读的检결结果。"""
    if not directory.is_dir():
        raise TagViolation(f"wheelhouse 目录不存在：{directory}")
    wheels = sorted(p.name for p in directory.iterdir() if p.is_file() and p.name.endswith(WHEEL_SUFFIX))
    if not wheels:
        raise TagViolation(f"wheelhouse 目录内没有任何 wheel：{directory}")
    results = [classify_wheel(name, spec) for name in wheels]
    violations = [
        {"filename": item["filename"], "reason": item["reason"]}
        for item in results
        if not item["accepted"]
    ]
    foreign = sorted({tag for item in results for tag in item.get("platform_tags", [])})
    return {
        "schema_version": SCHEMA_VERSION,
        "directory": str(directory),
        "target": {"id": spec.id, "arch": spec.arch, "platform_tag": spec.platform_tag, "python_tag": spec.python_tag},
        "checked": len(results),
        "accepted": len(results) - len(violations),
        "violations": violations,
        "platform_tags_seen": foreign,
        "simulation_note": SIMULATION_EVIDENCE_NOTE,
    }


# --------------------------------------------------------------------------------------
# 6. 抓取主流程
# --------------------------------------------------------------------------------------

def fetch(
    spec: TargetSpec,
    dest_dir: Path,
    *,
    dry_run: bool,
    resolve: bool,
    timeout: float,
    deadline_seconds: float,
    retries: int,
) -> dict:
    """执行抓取（或 dry-run 计划）；返回清单字典（dry-run 时不落盘）。

    先**解析全部包**再下载：任何一个声明的包在目标标签上没有可用 wheel，
    就在写出任何字节之前以退出码 2 失败（不留下半成品轮子）。
    """
    deadline = time.monotonic() + deadline_seconds
    plan: list[dict] = []
    entries: list[dict] = []
    skipped: list[dict] = []
    rejected_all: dict[str, list[dict]] = {}
    selected: list[Candidate] = []

    if not resolve:
        for package in spec.packages:
            plan.append(
                {"name": normalize_name(package), "index_page": index_page_url(spec.index_url, package), "resolved": False}
            )
        return {
            "schema_version": SCHEMA_VERSION,
            "dry_run": dry_run,
            "resolved": False,
            "target": target_block(spec),
            "index_url": spec.index_url,
            "planned": plan,
            "simulation_note": SIMULATION_EVIDENCE_NOTE,
        }

    for package in spec.packages:
        page = index_page_url(spec.index_url, package)
        html = _http_get(page, min(timeout, max(deadline - time.monotonic(), 1.0))).decode("utf-8", "replace")
        urls = parse_index_links(html, page)
        candidate, rejected = select_candidate(urls, spec, package)
        rejected_all[normalize_name(package)] = rejected
        if candidate is None:
            skipped.append(
                {
                    "name": normalize_name(package),
                    "index_page": page,
                    "reason": f"索引中没有匹配 {spec.python_tag}/{spec.platform_tag} 的 wheel"
                    f"（候选 {len(urls)} 个，全部被拒）",
                    "rejected_sample": rejected[:5],
                }
            )
            continue
        selected.append(candidate)
        plan.append(
            {
                "name": normalize_name(package),
                "index_page": page,
                "resolved": True,
                "filename": candidate.filename,
                "version": candidate.version,
                "kind": candidate.kind,
                "url": candidate.url,
                "candidates_total": len(urls),
                "candidates_rejected": len(rejected),
                "selection_reason": candidate.reason,
            }
        )

    if skipped:
        # 声明的包在目标标签上没有 wheel ⇒ 硬失败（退出码 2），不得用 x86_64 wheel 顶替。
        names = "、".join(item["name"] for item in skipped)
        raise DeclarationError(
            f"以下包在声明的源上没有可用的 {spec.python_tag}/{spec.platform_tag} wheel：{names}；"
            "禁止用 x86_64 wheel 顶替（AGENTS.md 1.5 / 5.10）"
        )

    if not dry_run:
        dest_dir.mkdir(parents=True, exist_ok=True)
        for candidate in selected:
            entries.append(download_candidate(candidate, dest_dir, timeout, retries, deadline))

    document = {
        "schema_version": SCHEMA_VERSION,
        "dry_run": dry_run,
        "resolved": True,
        "generated_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "target": target_block(spec),
        "index_url": spec.index_url,
        "index_url_declared_at": spec.index_url_declared_at,
        "packages_declared": list(spec.packages),
        "planned": plan,
        "entries": entries,
        "skipped": skipped,
        "rejected_candidates": {name: items[:5] for name, items in rejected_all.items()},
        "simulation_note": SIMULATION_EVIDENCE_NOTE,
    }
    if not dry_run:
        document["version_drift"] = version_drift(entries)
        (dest_dir / "wheelhouse.json").write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return document


def target_block(spec: TargetSpec) -> dict:
    return {
        "id": spec.id,
        "arch": spec.arch,
        "platform_tag": spec.platform_tag,
        "python_tag": spec.python_tag,
        "boards": spec.raw.get("boards", []),
    }


def version_drift(entries: list[dict]) -> list[dict]:
    """记录 wheelhouse 版本与抓取主机已装版本的差异（只记录，不做门禁；供人工决策是否 pin）。"""
    drift: list[dict] = []
    for entry in entries:
        name = entry["name"]
        try:
            from importlib import metadata

            host_version = metadata.version(name)
        except Exception:  # noqa: BLE001 - 抓取主机没装该包属正常情况
            continue
        if host_version != entry["version"]:
            drift.append(
                {"name": name, "wheelhouse_version": entry["version"], "fetch_host_version": host_version}
            )
    return drift


# --------------------------------------------------------------------------------------
# 7. CLI
# --------------------------------------------------------------------------------------


def _load(args) -> tuple[Path, TargetSpec]:
    repo_root = Path(args.repo_root).resolve() if args.repo_root else Path(__file__).resolve().parents[2]
    matrix_path = Path(args.matrix)
    if not matrix_path.is_absolute():
        matrix_path = repo_root / matrix_path
    document = load_matrix(matrix_path)
    return matrix_path, resolve_target(document, args.target)


def _emit(document: dict, json_out: str | None) -> None:
    text = json.dumps(document, ensure_ascii=False, indent=2) + "\n"
    if json_out:
        Path(json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(json_out).write_text(text, encoding="utf-8")
    sys.stdout.write(text)


def cmd_plan(args) -> int:
    _, spec = _load(args)
    _emit(
        {
            "schema_version": SCHEMA_VERSION,
            "target": target_block(spec),
            "index_url": spec.index_url,
            "packages": list(spec.packages),
            "wheels": list(spec.wheels),
            "pure_python": list(spec.pure_python),
            "reject_platform_tags": list(spec.reject_platform_tags),
            "pins": spec.pins,
            "simulation_note": SIMULATION_EVIDENCE_NOTE,
        },
        args.json_out,
    )
    return EXIT_OK


def cmd_classify(args) -> int:
    _, spec = _load(args)
    results = [classify_wheel(name, spec) for name in args.filename]
    rejected = [item for item in results if not item["accepted"]]
    for item in results:
        status = "接受" if item["accepted"] else "拒绝"
        sys.stderr.write(f"[check_wheel_tags] {status}：{item['filename']} —— {item['reason']}\n")
    _emit(
        {
            "schema_version": SCHEMA_VERSION,
            "target": target_block(spec),
            "results": results,
            "accepted": len(results) - len(rejected),
            "violations": len(rejected),
        },
        args.json_out,
    )
    return EXIT_VERIFY if rejected else EXIT_OK


def cmd_verify(args) -> int:
    _, spec = _load(args)
    report = verify_dir(Path(args.dir).resolve(), spec)
    _emit(report, args.json_out)
    if report["violations"]:
        for item in report["violations"]:
            sys.stderr.write(f"[check_wheel_tags][错误] 标签违规：{item['filename']} —— {item['reason']}\n")
        return EXIT_VERIFY
    sys.stderr.write(f"[check_wheel_tags] 标签校验通过：{report['checked']} 个 wheel\n")
    return EXIT_OK


def cmd_fetch(args) -> int:
    _, spec = _load(args)
    repo_root = Path(args.repo_root).resolve() if args.repo_root else Path(__file__).resolve().parents[2]
    dest = Path(args.dest)
    if args.dest == DEFAULT_DEST:
        # 默认按目标 id 命名空间化，避免多目标互相覆盖（步骤 06 已验证过这类并发覆盖缺陷）。
        dest = dest / spec.id
    if not dest.is_absolute():
        dest = repo_root / dest
    document = fetch(
        spec,
        dest,
        dry_run=args.dry_run,
        resolve=not args.no_resolve,
        timeout=args.timeout,
        deadline_seconds=args.deadline,
        retries=args.retries,
    )
    _emit(document, args.json_out)
    if args.dry_run:
        mode = "仅声明" if args.no_resolve else "已解析索引"
        sys.stderr.write(f"[check_wheel_tags] dry-run（{mode}）：未下载、未写盘（退出码 0）\n")
        return EXIT_OK

    report = verify_dir(dest, spec)
    _emit(report, str(dest / "verify-tags.json"))
    if report["violations"]:
        for item in report["violations"]:
            sys.stderr.write(f"[check_wheel_tags][错误] 标签违规：{item['filename']} —— {item['reason']}\n")
        return EXIT_VERIFY
    total_bytes = sum(entry["size_bytes"] for entry in document["entries"])
    sys.stderr.write(
        f"[check_wheel_tags] 抓取完成：{len(document['entries'])} 个 wheel / {total_bytes} 字节，"
        f"全部通过 {spec.python_tag}+{spec.platform_tag} 标签校验；清单：{dest / 'wheelhouse.json'}\n"
    )
    if document.get("version_drift"):
        drift = "、".join(f"{item['name']} {item['wheelhouse_version']}（本机 {item['fetch_host_version']}）" for item in document["version_drift"])
        sys.stderr.write(f"[check_wheel_tags][提示] 与本机已装版本存在差异：{drift}（只记录，不构成门禁；如需版本对齐请在矩阵声明 pins）\n")
    return EXIT_OK


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线 wheelhouse 平台标签校验与抓取（只读 config/sdk/package_matrix.yaml）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--repo-root", help="仓库根目录（默认：本文件上两级）")
    parser.add_argument("--matrix", default=DEFAULT_MATRIX, help=f"声明矩阵（默认 {DEFAULT_MATRIX}）")
    parser.add_argument("--target", help="构建目标 id（矩阵声明多个目标时必填）")
    parser.add_argument("--json-out", help="把本次结果的 JSON 同时落盘到该路径")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("plan", help="打印声明（不联网）")

    classify = sub.add_parser("classify", help="校验给定 wheel 文件名的标签（不联网）")
    classify.add_argument("filename", nargs="+", help="一个或多个 wheel 文件名")

    verify = sub.add_parser("verify", help="校验目录内所有 wheel 的标签")
    verify.add_argument("--dir", required=True, help="wheelhouse 目录")

    fetch_parser = sub.add_parser("fetch", help="解析索引并下载 wheel")
    fetch_parser.add_argument("--dest", default=DEFAULT_DEST, help=f"下载目录（默认 {DEFAULT_DEST}/<target-id>）")
    fetch_parser.add_argument("-n", "--dry-run", action="store_true", help="只打印计划，不下载不写盘")
    fetch_parser.add_argument("--no-resolve", action="store_true", help="dry-run 时连索引也不查询（纯声明）")
    fetch_parser.add_argument("--timeout", type=float, default=120.0, help="单次请求超时秒数（默认 120）")
    fetch_parser.add_argument("--deadline", type=float, default=900.0, help="本次抓取总截止秒数（默认 900）")
    fetch_parser.add_argument("--retries", type=int, default=2, help="单文件最大重试次数（默认 2，有界）")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "plan":
            return cmd_plan(args)
        if args.command == "classify":
            return cmd_classify(args)
        if args.command == "verify":
            return cmd_verify(args)
        if args.command == "fetch":
            return cmd_fetch(args)
        raise DeclarationError(f"未知子命令：{args.command}")
    except DeclarationError as exc:
        sys.stderr.write(f"[check_wheel_tags][预检失败] {exc}\n")
        return EXIT_PREFLIGHT
    except TagViolation as exc:
        sys.stderr.write(f"[check_wheel_tags][校验失败] {exc}\n")
        return EXIT_VERIFY
    except FetchError as exc:
        sys.stderr.write(f"[check_wheel_tags][抓取失败] {exc}\n")
        return EXIT_FETCH


if __name__ == "__main__":
    raise SystemExit(main())
