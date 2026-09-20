#!/usr/bin/env python3
"""SDK 部署编排实现层（IRAF iraf-24h 步骤 10）：双通道（media / ssh）传输。

设计约束：
  * **只用标准库**（argparse 之外：json/tarfile/hashlib/socket/subprocess/shutil/pathlib），
    因此不依赖 PyYAML / jsonschema，本机任意解释器（/usr/bin/python3 与 PATH 中的 python3）都能跑；
    与 08/09 的实现层不同：那两者需要在目标端读 YAML 声明，本工具不读声明文件，
    只读 **bundle 内清单成员**（bundle 是唯一事实来源，版本/板卡/架构不写死在代码里）。
  * 主机地址与凭据**只能**来自参数或环境变量；本文件不得出现任何 IP 常量（步骤 10 门禁）。
  * 无人值守硬边界：ssh/scp 一律 `BatchMode=yes`（不做密码交互、不落密码），仅在密钥就绪时可用。
  * 诚实边界（x86-first 战役）：本工具**不产生任何目标端证据**。media 通道只产出人工拷贝交接物；
    ssh 演练只产出计划。目标端真实安装、`/health`、事件库检查一律 DEFERRED（板卡不在场），
    禁止把演练结果表述为"已安装/板卡可用"。ssh apply 分支在本机**未实测**（无板卡），已显式登记。
"""

from __future__ import annotations

import hashlib
import json
import os
import shlex
import shutil
import socket
import subprocess
import sys
import tarfile
from pathlib import Path

DEPLOY_REPORT_SCHEMA_VERSION = "iraf.sdk-deploy-report/v1"

# 退出码（与设计 §5 / package_board_bundle.sh 统一约定）
EXIT_OK = 0
EXIT_USAGE = 1
EXIT_PRECHECK = 2
EXIT_TRANSFER = 3
EXIT_VERIFY = 4
EXIT_POSTCHECK = 5

REASON_UNREACHABLE = (
    "目标不可达：{target} 的 {port} 端口在 {timeout}s 内无响应（{detail}）；"
    "板卡不在场时这是 DEFERRED 语义（延后，不是失败），"
    "禁止用 dry-run 结果冒充目标端安装或健康检查证据"
)
REASON_NO_BOARD_EVIDENCE = (
    "目标端安装/健康检查 DEFERRED：板卡不在场（x86-first 战役）；"
    "本报告只描述开发端产出的交接物或计划，不构成目标端证据"
)

USAGE = """用法：python3 lib_deploy.py <media|ssh|plan> [选项]

子命令：
  media  产出媒体通道交接物：bundle 副本 + sha256sum.txt + README-安装.md + deploy-report.json
  ssh    ssh 直连部署：可达性探测 → scp 传输 → 远端校验和复核 → 远端 install.sh → 取回报告
  plan   只打印将要执行的动作（等价于 --dry-run）

选项：
  --bundle <tar.gz>       板级 bundle（默认：--bundle-dir 下**唯一**带伴随 *.sha256 的 tar.gz
                          —— 0 个或多个都显式失败，不自动挑第一个）
  --bundle-dir <dir>      bundle 所在目录（默认 build/sdk）
  --output <dir>          media 通道产物目录（media 必填）；ssh 通道为证据落盘目录（默认 build/deploy）
  --target <user@host>    ssh 目标（也可用环境变量 IRAF_DEPLOY_TARGET）；必须含 user@host
  --port <n>              ssh 端口（也可用环境变量 IRAF_DEPLOY_PORT；默认 22）
  --identity <key>        ssh 私钥（也可用环境变量 IRAF_DEPLOY_IDENTITY）
  --remote-dir <dir>      远端暂存目录（默认 ~/iraf-deploy）
  --allow-unverified      转发给远端 install.sh（默认**不转发**，fail-closed）
  --python <exe>          远端解释器（转发给 install.sh，默认 python3）
  --timeout-s <n>         可达性探测超时秒数（默认 4）
  --json-out <path>       额外落一份报告 JSON
  --repo-root <dir>       仓库根目录（默认：本文件上两级）
  --dry-run               只打印计划，不做任何传输

退出码：0 成功 / 1 参数错误 / 2 预检失败（bundle 多义或缺校验和、目标不可达、缺 ssh/scp）
        3 传输或远端执行失败 / 4 校验失败（SHA-256 不符）/ 5 远端后置校验失败
"""


class DeployError(Exception):
    """带退出码的部署错误（中文原因）。"""

    def __init__(self, message: str, exit_code: int, details: list[str] | None = None):
        super().__init__(message)
        self.exit_code = exit_code
        self.details = list(details or [])


def info(message: str) -> None:
    print(f"[deploy] {message}")


def warn(message: str) -> None:
    print(f"[deploy][警告] {message}", file=sys.stderr)


def err(message: str) -> None:
    print(f"[deploy][错误] {message}", file=sys.stderr)


# --- 基础工具 -----------------------------------------------------------------


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def display_path(path: Path, repo_root: Path) -> str:
    """日志用路径：仓库内用相对路径，仓库外允许绝对（仅日志）。"""
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)


def record_path(path: Path, repo_root: Path) -> tuple[str, str]:
    """入库路径：仓库内用仓库相对路径；仓库外显式标注（媒体通道允许拷到仓库外）。

    返回 (路径文本, 作用域)。
    """
    if path.is_absolute():
        candidate = path
    else:
        candidate = repo_root / path
    try:
        relative = candidate.resolve().relative_to(repo_root.resolve())
        return relative.as_posix(), "repo_relative"
    except ValueError:
        return str(candidate), "absolute_outside_repo"


def _abspath(repo_root: Path, value) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (repo_root / path)


# --- 参数解析 -----------------------------------------------------------------


def parse_options(argv: list[str]) -> dict:
    options: dict = {"repo_root": str(Path(__file__).resolve().parents[2])}
    value_flags = {
        "--repo-root": "repo_root",
        "--bundle": "bundle",
        "--bundle-dir": "bundle_dir",
        "--output": "output",
        "--target": "target",
        "--port": "port",
        "--identity": "identity",
        "--remote-dir": "remote_dir",
        "--python": "python",
        "--timeout-s": "timeout_s",
        "--json-out": "json_out",
    }
    bool_flags = {"--dry-run": "dry_run", "--allow-unverified": "allow_unverified"}
    index = 0
    while index < len(argv):
        item = argv[index]
        if item in ("-h", "--help"):
            options["help"] = True
        elif item in value_flags:
            if index + 1 >= len(argv):
                raise DeployError(f"{item} 缺少取值", EXIT_USAGE)
            options[value_flags[item]] = argv[index + 1]
            index += 1
        elif item in bool_flags:
            options[bool_flags[item]] = True
        else:
            raise DeployError(f"无法识别的参数：{item}（用 --help 查看用法）", EXIT_USAGE)
        index += 1
    return options


def env_option(options: dict, key: str, env_name: str):
    if options.get(key):
        return options[key]
    value = os.environ.get(env_name)
    return value if value else None


# --- bundle 解析与校验 --------------------------------------------------------


def discover_bundle(bundle_dir: Path) -> Path:
    if not bundle_dir.is_dir():
        raise DeployError(
            f"bundle 目录不存在：{bundle_dir}（用 --bundle 或 --bundle-dir 指定）", EXIT_PRECHECK
        )
    candidates = sorted(p for p in bundle_dir.glob("*.tar.gz") if Path(f"{p}.sha256").is_file())
    if not candidates:
        raise DeployError(
            f"{bundle_dir} 下没有带伴随校验和（*.sha256）的 tar.gz；"
            "bundle 由 deploy/sdk/package_board_bundle.sh 产出，请先组装或显式 --bundle",
            EXIT_PRECHECK,
        )
    if len(candidates) > 1:
        raise DeployError(
            f"{bundle_dir} 下存在 {len(candidates)} 个带校验和的候选 bundle：不自动挑第一个，"
            "请用 --bundle 显式指定",
            EXIT_PRECHECK,
            [p.name for p in candidates],
        )
    return candidates[0]


def resolve_bundle(options: dict, repo_root: Path) -> Path:
    value = options.get("bundle")
    if value:
        path = _abspath(repo_root, value)
        if not path.is_file():
            raise DeployError(f"bundle 不存在：{path}", EXIT_PRECHECK)
        return path
    bundle_dir = _abspath(repo_root, options.get("bundle_dir") or "build/sdk")
    return discover_bundle(bundle_dir)


def verify_bundle(bundle: Path) -> dict:
    sidecar = Path(f"{bundle}.sha256")
    if not sidecar.is_file():
        raise DeployError(
            f"缺少伴随校验和文件 {sidecar.name}（不信任无校验和的 bundle；"
            "由 package_board_bundle.sh 产出）",
            EXIT_PRECHECK,
        )
    fields = sidecar.read_text(encoding="utf-8").strip().split()
    expected = fields[0] if fields else ""
    if len(expected) != 64 or any(ch not in "0123456789abcdefABCDEF" for ch in expected):
        raise DeployError(
            f"伴随校验和文件内容非法：{sidecar.name}（期望 64 位十六进制 SHA-256）", EXIT_PRECHECK
        )
    named = fields[1] if len(fields) > 1 else ""
    if named and Path(named).name != bundle.name:
        raise DeployError(
            f"伴随校验和文件登记的条目名与 bundle 不符：{named} != {bundle.name}"
            "（疑似校验错文件，拒绝继续）",
            EXIT_VERIFY,
        )
    digest = sha256_file(bundle)
    if digest != expected.lower():
        raise DeployError(
            f"bundle SHA-256 与伴随校验和不符：清单 {expected[:16]}… / 实际 {digest[:16]}…",
            EXIT_VERIFY,
        )
    return {
        "name": bundle.name,
        "sha256": digest,
        "expected_sha256": expected.lower(),
        "size_bytes": bundle.stat().st_size,
        "checksum_file": sidecar.name,
    }


def read_bundle_meta(bundle: Path) -> dict:
    """从 bundle 顶层清单 JSON 读取声明取值（版本/板卡/架构不写死在代码里）。"""
    with tarfile.open(bundle, "r:gz") as tar:
        members = [
            member
            for member in tar.getmembers()
            if member.isfile() and member.name.endswith(".json") and "/" not in member.name
        ]
        if len(members) != 1:
            raise DeployError(
                f"bundle 顶层应有且仅有 1 个清单 JSON，实际 {len(members)} 个："
                f"{[m.name for m in members]}",
                EXIT_PRECHECK,
            )
        handle = tar.extractfile(members[0])
        if handle is None:  # pragma: no cover - tarfile 对普通文件不会返回 None
            raise DeployError(f"bundle 清单成员不可读：{members[0].name}", EXIT_PRECHECK)
        try:
            manifest = json.loads(handle.read().decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise DeployError(f"bundle 清单不是合法 JSON：{exc}", EXIT_PRECHECK) from exc
    section = manifest.get("bundle") or {}
    required = ("board", "version", "arch", "target", "python_tag", "platform_tag")
    missing = [key for key in required if not section.get(key)]
    if missing:
        raise DeployError(
            f"bundle 清单缺少声明字段：{','.join(missing)}（声明缺失必须显式失败，不猜默认值）",
            EXIT_PRECHECK,
        )
    return {
        "member": members[0].name,
        "schema_version": manifest.get("schema_version"),
        "declared": {key: section[key] for key in required},
    }


# --- 报告 ---------------------------------------------------------------------


def base_report(
    transport: str, mode: str, bundle_path: Path, bundle_info: dict, meta: dict, repo_root: Path
) -> dict:
    scope, path_scope = record_path(bundle_path, repo_root)
    return {
        "schema_version": DEPLOY_REPORT_SCHEMA_VERSION,
        "transport": transport,
        "mode": mode,
        "repo_root": repo_root.as_posix(),
        "bundle": {
            "path": scope,
            "path_scope": path_scope,
            **bundle_info,
            "declared": meta["declared"],
            "manifest_member": meta["member"],
        },
        "deployed": False,
        "install_performed": False,
        "board_evidence": {"available": False, "reason": REASON_NO_BOARD_EVIDENCE},
        "steps": [],
        "blockers": [],
    }


def finish_report(report: dict, options: dict, repo_root: Path) -> int:
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if options.get("json_out"):
        path = _abspath(repo_root, options["json_out"])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        report["report_file"] = display_path(path, repo_root)
        text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    sys.stdout.write(text)
    return EXIT_OK


def step(report: dict, name: str, status: str, detail: str, **extra) -> None:
    entry = {"step": name, "status": status, "detail": detail}
    entry.update(extra)
    report["steps"].append(entry)
    info(f"[{status}] {name}：{detail}")


# --- media 通道 ---------------------------------------------------------------


def readme_text(report: dict, artifacts: list[dict]) -> str:
    declared = report["bundle"]["declared"]
    bundle_name = report["bundle"]["name"]
    digest = report["bundle"]["sha256"]
    size_mb = report["bundle"]["size_bytes"] / (1024 * 1024)
    lines = [
        "# 板级 SDK 安装交接说明（媒体通道）",
        "",
        "本目录是 IRAF 板级 SDK 的**人工拷贝交接物**，由 `deploy/sdk/deploy.sh --transport media` 产出。",
        "隔离网环境请按下面的顺序操作；每一步的失败都必须停下来看中文原因，不要跳步。",
        "",
        "## 一、本包内容（取值来自 bundle 内清单，非人工填写）",
        "",
        f"- bundle 文件：`{bundle_name}`（{size_mb:.1f} MiB）",
        f"- 目标板卡声明：`{declared['board']}` / 目标 id `{declared['target']}` / 架构 `{declared['arch']}`",
        f"- SDK 版本：`{declared['version']}`；Python 标签 `{declared['python_tag']}`；"
        f"平台标签 `{declared['platform_tag']}`",
        f"- SHA-256：`{digest}`",
        "",
        "产物清单：",
        "",
    ]
    for item in artifacts:
        lines.append(f"- `{item['path']}`（{item['size_bytes']} 字节，SHA-256 `{item['sha256'][:16]}…`）")
    lines += [
        "",
        "## 二、安装前（目标端）",
        "",
        "1. 校验完整性：`sha256sum -c sha256sum.txt`，**必须**输出 `OK`；不符即停止，重新拷贝。",
        "2. 确认目标端有 Python 与 PyYAML + jsonschema（bundle 内 `wheelhouse/` 提供离线 wheel）：",
        "   `python3 -c 'import yaml, jsonschema'`；缺失时先用 `pip --no-index --find-links <bundle 解包目录>/wheelhouse/<target-id>` 安装。",
        "3. 确认板卡声明状态：`bash scripts/install.sh --bundle <bundle> --help` 查看约定；",
        "   若 BoardProfile 仍是 `unverified`（未实测），真实安装需要**人工确认后**显式追加 `--allow-unverified`，",
        "   这不是「绕过门禁」，而是把「未实测」这件事记录到安装报告里。",
        "",
        "## 三、安装与自检",
        "",
        "4. 解包脚本：`tar -xzf "
        + bundle_name
        + " scripts`（只需要 `scripts/`，目录可指定）。",
        "5. 安装（真实写入系统目录，需要 sudo）：",
        "",
        "   ```bash",
        "   sudo bash scripts/install.sh --bundle " + bundle_name + " --root / --allow-unverified \\",
        "        --json-out ./install-report.json",
        "   ```",
        "",
        "6. 自检（安装后，服务已注册）：",
        "",
        "   ```bash",
        "   sudo bash scripts/verify.sh --bundle " + bundle_name + " --root / --json-out ./verify-evidence.json",
        "   ```",
        "   退出码 0 才表示自检通过；`service_state=SERVICE_NOT_RUNNING` 属**失败**（退出 5），不是通过。",
        "",
        "## 四、回滚与卸载",
        "",
        "7. 回滚/卸载（按安装记录逐文件删除，保留日志与事件库）：",
        "",
        "   ```bash",
        "   sudo bash scripts/uninstall.sh --root /            # 先 --dry-run 比对范围",
        "   ```",
        "   安装前缀、配置目录、日志目录、安装记录名以 `config/sdk/package_matrix.yaml` 的",
        "   `delivery.install` 声明为准（单一来源，不在脚本里写死）。",
        "",
        "## 五、边界声明（必须照抄进验收记录，不得省）",
        "",
        "- 本交接物**只**证明开发端产出了可安装形态并登记了 SHA-256；**不**构成目标端安装或健康检查证据。",
        "- 目标端安装、`/health`、gRPC、事件库写入检查一律 **DEFERRED**（板卡不在场，x86-first 战役）。",
        "- 禁止把演练（`--dry-run`）或本说明书的写操作当成「已部署成功」。",
        "",
    ]
    return "\n".join(lines)


def cmd_media(options: dict) -> int:
    repo_root = Path(str(options["repo_root"])).resolve()
    output = options.get("output")
    if not output:
        raise DeployError("media 通道必须显式给 --output <dir>（交接物输出目录）", EXIT_USAGE)
    out_dir = _abspath(repo_root, output)
    bundle = resolve_bundle(options, repo_root)
    bundle_info = verify_bundle(bundle)
    meta = read_bundle_meta(bundle)
    report = base_report(
        "media",
        "rehearsal" if options.get("dry_run") else "apply",
        bundle,
        bundle_info,
        meta,
        repo_root,
    )
    report["evidence_scope"] = "media_handoff_artifact"

    if options.get("dry_run"):
        report["output_dir"], report["output_dir_scope"] = record_path(out_dir, repo_root)
        step(report, "M1 计划", "PLAN", "只打印计划：不创建目录、不写任何文件")
        step(report, "M2 校验和", "PASS", f"bundle sha256={bundle_info['sha256'][:16]}…（与伴随文件一致）")
        step(
            report,
            "M3 交接物",
            "PLAN",
            "将产出 bundle 副本 + sha256sum.txt + README-安装.md + deploy-report.json",
        )
        return finish_report(report, options, repo_root)

    out_dir.mkdir(parents=True, exist_ok=True)
    target_bundle = out_dir / bundle.name
    if target_bundle.resolve() == bundle.resolve():
        step(report, "M1 复制", "SKIP", "输出目录即 bundle 所在目录，无需复制")
    else:
        shutil.copy2(bundle, target_bundle)
        step(report, "M1 复制", "PASS", f"{bundle.name} → {display_path(out_dir, repo_root)}")

    checksum_file = out_dir / "sha256sum.txt"
    checksum_file.write_text(f"{bundle_info['sha256']}  {bundle.name}\n", encoding="utf-8")
    step(report, "M2 校验和清单", "PASS", "sha256sum.txt（sha256sum -c 可直接复核）")

    artifacts: list[dict] = []
    for path in (target_bundle, checksum_file):
        artifacts.append(
            {
                "path": record_path(path, repo_root)[0],
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    readme = out_dir / "README-安装.md"
    readme.write_text(readme_text(report, artifacts), encoding="utf-8")
    artifacts.append(
        {
            "path": record_path(readme, repo_root)[0],
            "sha256": sha256_file(readme),
            "size_bytes": readme.stat().st_size,
        }
    )
    step(report, "M3 交接物", "PASS", "bundle 副本 / sha256sum.txt / README-安装.md")

    report["output_dir"], report["output_dir_scope"] = record_path(out_dir, repo_root)
    report["artifacts"] = artifacts
    report["report_path"] = record_path(out_dir / "deploy-report.json", repo_root)[0]
    step(report, "M4 交接说明", "PASS", "README-安装.md（中文：校验 → 安装 → 自检 → 回滚 → 边界声明）")
    step(
        report,
        "M5 边界",
        "DEFERRED",
        "媒体通道不构成目标端证据；真实安装/健康检查需板卡到位后按 README 执行",
    )
    report_ptr = out_dir / "deploy-report.json"
    report_ptr.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return finish_report(report, options, repo_root)


# --- ssh 通道 -----------------------------------------------------------------


def parse_target(options: dict) -> tuple[str, str]:
    raw = env_option(options, "target", "IRAF_DEPLOY_TARGET")
    if not raw:
        raise DeployError(
            "ssh 通道必须给 --target <user@host>（或用环境变量 IRAF_DEPLOY_TARGET）；"
            "脚本内不内置任何主机地址",
            EXIT_USAGE,
        )
    user, _, host = str(raw).partition("@")
    if not user or not host:
        raise DeployError(f"--target 必须是 <user@host> 形式，实际：{raw}", EXIT_USAGE)
    if any(ch.isspace() for ch in host):
        raise DeployError(f"--target 主机名含空白字符：{raw}", EXIT_USAGE)
    return user, host


def parse_port(options: dict) -> int:
    raw = env_option(options, "port", "IRAF_DEPLOY_PORT") or "22"
    try:
        port = int(str(raw))
    except ValueError as exc:
        raise DeployError(f"--port 必须是整数，实际：{raw}", EXIT_USAGE) from exc
    if not (1 <= port <= 65535):
        raise DeployError(f"--port 超出范围（1-65535）：{port}", EXIT_USAGE)
    return port


def require_tools(names: tuple[str, ...]) -> dict:
    found = {name: shutil.which(name) for name in names}
    missing = [name for name, path in found.items() if not path]
    if missing:
        raise DeployError(
            f"缺少必需工具：{','.join(missing)}（ssh 通道需要 OpenSSH client；"
            "无网/精简环境请改用 --transport media）",
            EXIT_PRECHECK,
        )
    return found


def probe(host: str, port: int, timeout: float) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, ""
    except OSError as exc:
        return False, f"{type(exc).__name__}: {exc}"


def _ssh_options(port: int, identity: str | None, timeout: float) -> list[str]:
    args = [
        "-p",
        str(port),
        "-o",
        "BatchMode=yes",  # 无人值守：禁止密码交互
        "-o",
        f"ConnectTimeout={int(timeout)}",
        "-o",
        "StrictHostKeyChecking=accept-new",
    ]
    if identity:
        args += ["-i", identity]
    return args


def _run(argv: list[str], *, capture: bool = True) -> subprocess.CompletedProcess:
    info("执行：" + " ".join(shlex.quote(item) for item in argv))
    return subprocess.run(argv, capture_output=capture, text=True, timeout=600)


def cmd_ssh(options: dict) -> int:
    repo_root = Path(str(options["repo_root"])).resolve()
    user, host = parse_target(options)
    port = parse_port(options)
    target = f"{user}@{host}"
    identity = env_option(options, "identity", "IRAF_DEPLOY_IDENTITY")
    remote_dir = str(options.get("remote_dir") or "~/iraf-deploy")
    timeout = float(options.get("timeout_s") or 4)
    out_dir = _abspath(repo_root, options.get("output") or "build/deploy")

    bundle = resolve_bundle(options, repo_root)
    bundle_info = verify_bundle(bundle)
    meta = read_bundle_meta(bundle)
    report = base_report(
        "ssh", "rehearsal" if options.get("dry_run") else "apply", bundle, bundle_info, meta, repo_root
    )
    report["evidence_scope"] = "plan_only" if options.get("dry_run") else "target_end_install"
    report["target"] = {"user": user, "host": host, "port": port, "remote_dir": remote_dir}

    reachable, detail = probe(host, port, timeout)
    report["reachability"] = {"host": host, "port": port, "timeout_s": timeout, "reachable": reachable}
    if not reachable:
        report["blockers"].append({"kind": "target_unreachable", "message": detail})
        report["deployed"] = False
        report["exit_code"] = EXIT_PRECHECK
        step(report, "S1 可达性探测", "FAIL", REASON_UNREACHABLE.format(target=target, port=port, timeout=timeout, detail=detail))
        # 失败也要落证据：报告先落盘/打印，再抛错（禁止只留一句中文就结束）。
        finish_report(report, options, repo_root)
        raise DeployError(
            REASON_UNREACHABLE.format(target=target, port=port, timeout=timeout, detail=detail),
            EXIT_PRECHECK,
        )
    step(report, "S1 可达性探测", "PASS", f"{host}:{port} 可建立 TCP 连接（{timeout}s 超时）")

    tools = require_tools(("ssh", "scp"))
    install_args = ["--bundle", f"{remote_dir}/{bundle.name}", "--root", "/"]
    if options.get("allow_unverified"):
        install_args.append("--allow-unverified")
    if options.get("python"):
        install_args += ["--python", str(options["python"])]
    install_args += ["--json-out", f"{remote_dir}/install-report.json"]
    ssh_opts = _ssh_options(port, identity, timeout)
    q = shlex.quote
    plan = [
        ["ssh", *ssh_opts, target, f"mkdir -p {q(remote_dir)}"],
        ["scp", "-P", str(port), *(["-i", identity] if identity else []), str(bundle), f"{bundle}.sha256", f"{target}:{remote_dir}/"],
        ["ssh", *ssh_opts, target, f"cd {q(remote_dir)} && sha256sum -c {q(bundle.name + '.sha256')}"],
        ["ssh", *ssh_opts, target, f"tar -xzf {q(remote_dir + '/' + bundle.name)} -C {q(remote_dir)} scripts"],
        ["ssh", *ssh_opts, target, f"bash {q(remote_dir + '/scripts/install.sh')} " + " ".join(q(a) for a in install_args)],
        ["scp", "-P", str(port), *(["-i", identity] if identity else []), f"{target}:{remote_dir}/install-report.json", str(out_dir / "install-report.json")],
    ]
    report["plan"] = [[tools.get(argv[0], argv[0]) or argv[0], *argv[1:]] for argv in plan]

    if options.get("dry_run"):
        report["output_dir"], report["output_dir_scope"] = record_path(out_dir, repo_root)
        step(report, "S2 计划", "PLAN", f"演练（plan_only）：将执行 {len(plan)} 条远端/传输命令，不做任何传输")
        step(report, "S3 边界", "DEFERRED", "演练 ≠ 已部署；目标端安装与 /health 验收 DEFERRED（板卡不在场）")
        return finish_report(report, options, repo_root)

    rc = _run(plan[0]).returncode
    if rc != 0:
        raise DeployError(f"远端暂存目录创建失败（退出码 {rc}）：{remote_dir}", EXIT_TRANSFER)
    step(report, "S2 远端暂存目录", "PASS", remote_dir)

    proc = _run(plan[1])
    if proc.returncode != 0:
        raise DeployError(
            f"scp 传输失败（退出码 {proc.returncode}）：{proc.stderr.strip()[:300]}", EXIT_TRANSFER
        )
    step(report, "S3 传输", "PASS", f"{bundle.name} + 伴随校验和 → {target}:{remote_dir}/")

    proc = _run(plan[2])
    if proc.returncode != 0:
        raise DeployError(
            f"远端 SHA-256 复核失败（退出码 {proc.returncode}）：{proc.stdout.strip()[:300]}"
            "（传输损坏或对端文件被替换，禁止继续安装）",
            EXIT_VERIFY,
        )
    step(report, "S4 远端校验和复核", "PASS", "sha256sum -c 通过")

    proc = _run(plan[3])
    if proc.returncode != 0:
        raise DeployError(
            f"远端解包 scripts/ 失败（退出码 {proc.returncode}）：{proc.stderr.strip()[:300]}",
            EXIT_TRANSFER,
        )
    step(report, "S5 解包脚本", "PASS", f"{remote_dir}/scripts/（install.sh + 实现层）")

    proc = _run(plan[4])
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "install-stdout.json").write_text(proc.stdout or "", encoding="utf-8")
    if proc.returncode != 0:
        report["remote_exit_code"] = proc.returncode
        code = proc.returncode if proc.returncode in (EXIT_PRECHECK, EXIT_TRANSFER, EXIT_VERIFY, EXIT_POSTCHECK) else EXIT_TRANSFER
        raise DeployError(
            f"远端 install.sh 未通过（退出码 {proc.returncode}，本工具按 {code} 透传）："
            f"{(proc.stderr or proc.stdout).strip()[:400]}",
            code,
        )
    report["install_performed"] = True
    report["deployed"] = True
    step(report, "S6 远端安装", "PASS", "install.sh 退出码 0（安装报告见 install-report.json）")

    proc = _run(plan[5])
    if proc.returncode != 0:
        raise DeployError(
            f"取回 install-report.json 失败（退出码 {proc.returncode}）：{proc.stderr.strip()[:300]}",
            EXIT_TRANSFER,
        )
    step(report, "S7 取回证据", "PASS", display_path(out_dir / "install-report.json", repo_root))
    report["output_dir"], report["output_dir_scope"] = record_path(out_dir, repo_root)
    step(
        report,
        "S8 边界",
        "DEFERRED",
        "目标端 /health、gRPC 与事件库检查需在板卡上再跑 verify.sh，本工具不代跑、不当通过",
    )
    return finish_report(report, options, repo_root)


# --- 入口 ---------------------------------------------------------------------


def cmd_plan(options: dict) -> int:
    options = {**options, "dry_run": True}
    transport = options.get("transport")
    if transport == "media":
        return cmd_media(options)
    if transport == "ssh":
        return cmd_ssh(options)
    raise DeployError("plan 需要 --transport <media|ssh>", EXIT_USAGE)


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return EXIT_OK if argv else EXIT_USAGE
    command, rest = argv[0], argv[1:]
    try:
        options = parse_options(rest)
        if options.get("help"):
            print(USAGE)
            return EXIT_OK
        if command == "media":
            return cmd_media(options)
        if command == "ssh":
            return cmd_ssh(options)
        if command == "plan":
            return cmd_plan(options)
        print(f"[deploy] 无法识别的子命令：{command}\n", file=sys.stderr)
        return EXIT_USAGE
    except DeployError as exc:
        err(str(exc))
        for detail in exc.details:
            err(f"  - {detail}")
        return exc.exit_code
    except subprocess.SubprocessError as exc:  # pragma: no cover - 依赖系统工具行为
        err(f"子进程执行失败：{exc}")
        return EXIT_TRANSFER


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
