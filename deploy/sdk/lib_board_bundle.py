#!/usr/bin/env python3
"""板级 bundle 组装与目标端安装的实现层（步骤 08）。

被两个 shell 入口调用（入口只做参数解析与退出码传递，逻辑全在这里）：

    bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml [--allow-unverified]
    bash deploy/sdk/install.sh --bundle build/sdk/iraf-board-e300-0.2.0-aarch64.tar.gz --dry-run --root <dir>

也可以直接当 CLI 用：

    python3 deploy/sdk/lib_board_bundle.py plan    --board profiles/boards/e300.yaml
    python3 deploy/sdk/lib_board_bundle.py build   --board profiles/boards/e300.yaml --allow-unverified
    python3 deploy/sdk/lib_board_bundle.py inspect --bundle build/sdk/iraf-board-e300-0.2.0-aarch64.tar.gz
    python3 deploy/sdk/lib_board_bundle.py install --bundle <tar.gz> --dry-run --root <dir>

分层与契约（铁律 5.3、设计 §4/§5/§6）：

- **声明只读**：版本、Python/平台标签、包清单、镜像源、板卡映射、交付形态（bundle 命名模板与
  目标端安装布局）全部来自 `config/sdk/package_matrix.yaml`；本文件不复制这些字面量。
  目标端安装时改读 **bundle 内**的同一份矩阵（目标端没有仓库，只有 bundle）。
- **板卡门禁复用既有入口**：调用 `scripts/profile_check.py --board`（步骤 03 的实现），不重新
  实现一遍 unverified 判定；未加 `--allow-unverified` 且声明含 unverified/pending 即退出 2。
- **产物只含 bundle 相对路径**：bundle 内清单 `iraf-board-bundle.json` 里不得出现本机绝对路径
  （复用 `lib_manifest.absolute_path_violations` 同一份门禁）。
- **可复算**：bundle 用 gzip `mtime=0` + 成员 `mtime=0`/`uid=gid=0` + 路径排序，且不含时间戳字段，
  同一棵源码树连续两次组装 SHA-256 逐位相同。
- **演练 ≠ 安装**：`--dry-run`（rehearsal）只在 `--root` 沙箱内执行，退出码 0 只表示"演练流程
  完成"：报告里 `evidence_scope=rehearsal_only`、`real_install_allowed` 由 blockers 决定。
  禁止把演练结果表述为"已安装"（x86-first 战役范围，板卡不在场）。
- **stdout 契约**：`install` 子命令的 stdout 只有一份安装报告 JSON（日志一律走 stderr），便于
  `install.sh ... | jq .exit_code` 之类的调用。

退出码（设计 §5 全脚本统一约定）：

    0  成功 / 演练完成
    1  参数/用法错误
    2  预检失败（bundle 契约、声明未实测、架构/Python/glibc/磁盘/RAM/systemd 不符）
    3  安装失败（IO/权限/依赖安装失败）：已回滚
    4  校验失败（SHA-256 不符、bundle 成员被篡改、路径穿越、已装目录与 bundle 不一致）
    5  后置校验失败（profile-check / verify.sh / 服务启动失败）：不激活新版本，回滚激活链接
"""

from __future__ import annotations

import ctypes
import gzip
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from lib_manifest import (  # noqa: E402 - 同一目录的实现层，复用同一份契约与工具
    ExitCode,
    ManifestError,
    absolute_path_violations,
    display_path,
    git_state,
    load_matrix,
    read_declared_version,
    relpath_posix,
    select_target,
    sha256_bytes,
    sha256_file,
)

USAGE = __doc__

# --- 契约常量：只在这里声明一次 ------------------------------------------------

BUNDLE_SCHEMA_VERSION = "iraf.board-bundle/v1"
BUILD_SUMMARY_SCHEMA_VERSION = "iraf.board-bundle-build/v1"
INSTALL_REPORT_SCHEMA_VERSION = "iraf.board-install-report/v1"
MATRIX_REL = "config/sdk/package_matrix.yaml"
MATRIX_SCHEMA_REL = "config/sdk/package_matrix.schema.json"
DEFAULT_OUT = "build/sdk"
DEFAULT_WHEELHOUSE_ROOT = "build/wheelhouse"
RUNTIME_BUNDLE_TEMPLATE = "iraf-runtime-{version}-{arch}.tar.gz"
SDK_WHEEL_TEMPLATE = "{dist}-{version}-py3-none-any.whl"
PROFILE_CHECK_SCRIPT = "scripts/profile_check.py"
#: 随 bundle 交付的脚本（相对仓库根）。缺失即**组装失败**（不得交付跑不起来的安装包）。
BUNDLE_SCRIPT_FILES = (
    "deploy/sdk/install.sh",
    "deploy/sdk/package_board_bundle.sh",
    "deploy/sdk/build_sdk.sh",
    "deploy/sdk/fetch_wheelhouse.sh",
    "deploy/sdk/check_wheel_tags.py",
    "deploy/sdk/lib_manifest.py",
    "deploy/sdk/lib_board_bundle.py",
    # 步骤 09 交付：目标端自检与卸载的实现层（verify.sh/uninstall.sh 同目录依赖它，必须随包交付）
    "deploy/sdk/lib_target_verify.py",
    PROFILE_CHECK_SCRIPT,
)
#: 后续步骤交付的脚本：缺失只登记为 pending（步骤 09 verify/uninstall、步骤 10 deploy）；
#: 其中被声明为后置校验的那个（POST_VERIFY_SCRIPT）在**真实安装**时缺失即安装失败。
BUNDLE_PENDING_SCRIPTS = {
    "deploy/sdk/verify.sh": "步骤 09（verify.sh 与 uninstall.sh）",
    "deploy/sdk/uninstall.sh": "步骤 09（verify.sh 与 uninstall.sh）",
    "deploy/sdk/deploy.sh": "步骤 10（deploy.sh 双通道传输）",
}
POST_VERIFY_SCRIPT = "deploy/sdk/verify.sh"
#: 模板占位符（env 模板与 systemd 单元共用；渲染后残留任何 @NAME@ 即组装/安装失败：
#: 未知占位符必须显式失败，不得留给目标端一个半渲染的配置）。
PLACEHOLDERS = (
    "@SDK_NAME@",
    "@SDK_VERSION@",
    "@BOARD@",
    "@TARGET_ID@",
    "@PREFIX@",
    "@CURRENT_LINK@",
    "@CONFIG_DIR@",
    "@LOG_DIR@",
    "@SERVICE_UNIT@",
    "@ENV_FILE@",
    "@PYTHON_BIN@",
)
#: 磁盘门禁余量系数：解包后所需空间 × 该系数（覆盖安装期临时文件与回滚空间）。
DISK_HEADROOM_FACTOR = 2
#: runtime bundle 是压缩包，解包后体积按该倍数估算（实测 161 KB → 约 0.5 MB）。
RUNTIME_EXPANSION_FACTOR = 3
#: 目标端二进制 wheel 的安装状态：x86 开发端**不能**安装 aarch64 wheel，演练只登记为延后。
BINARY_WHEELS_DEFERRED = "deferred_target_end"
INSTALL_REPORT_FALLBACK_NAME = "install-report"

__all__ = [
    "BoardBundleError",
    "Installer",
    "build_board_bundle",
    "bundle_name",
    "collect_host_facts",
    "detect_glibc_version",
    "parse_version",
    "platform_tag_glibc",
    "precheck_violations",
    "python_tag_version",
    "read_bundle_members",
    "render_env_template",
    "required_disk_mb",
    "run_install",
    "verify_bundle_contract",
    "main",
]


class BoardBundleError(ManifestError):
    """带退出码的中文可操作错误（沿用构建侧同一套退出码语义）。"""


# --- 基础工具 ------------------------------------------------------------------


def info(message: str) -> None:
    """日志一律走 stderr：stdout 留给 JSON 报告（install 子命令的输出契约）。"""
    print(f"[board_bundle] {message}", file=sys.stderr, flush=True)


def warn(message: str) -> None:
    print(f"[board_bundle][警告] {message}", file=sys.stderr, flush=True)


def _abspath(base: Path, value) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else base / path


def _within(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def detect_glibc_version() -> str | None:
    """实测 glibc 版本。

    坑（本步实测）：`ctypes.CDLL("libc.so.6").gnu_get_libc_version()` 的默认返回类型是 `c_int`，
    直接取用会得到一个无意义整数（本机实测 -44196217）。必须显式声明 `restype = c_char_p`；
    两条路径都拿不到时返回 None（调用方按"无法判定"处理，**不猜**）。
    """
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.gnu_get_libc_version.restype = ctypes.c_char_p
        raw = libc.gnu_get_libc_version()
        if raw:
            text = raw.decode("ascii", "replace") if isinstance(raw, bytes) else str(raw)
            if re.match(r"^[0-9]+\.[0-9]+", text):
                return text
    except (OSError, AttributeError):
        pass
    _, libc_version = platform.libc_ver()
    return libc_version or None


def parse_version(text) -> tuple[int, ...] | None:
    if not isinstance(text, str):
        return None
    match = re.match(r"^\s*([0-9]+(?:\.[0-9]+)*)", text)
    if not match:
        return None
    return tuple(int(part) for part in match.group(1).split("."))


def platform_tag_glibc(platform_tag: str) -> tuple[int, int] | None:
    """manylinux 平台标签 → 最低 glibc 要求；未知形式返回 None（fail-closed，不猜）。"""
    legacy = {"manylinux1": (2, 5), "manylinux2010": (2, 12), "manylinux2014": (2, 17)}
    for name, requirement in legacy.items():
        if platform_tag.startswith(name + "_"):
            return requirement
    match = re.match(r"^manylinux_([0-9]+)_([0-9]+)_(?:aarch64|x86_64)$", platform_tag)
    if match:
        return int(match.group(1)), int(match.group(2))
    return None


def python_tag_version(python_tag: str) -> tuple[int, int] | None:
    match = re.match(r"^cp([0-9])([0-9]+)$", python_tag or "")
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))


def is_relative_member(name: str) -> bool:
    """bundle 内成员路径必须相对、无 `..`、无绝对路径（设计 §4 的产物硬要求）。"""
    if not name or name.startswith("/") or re.match(r"^[A-Za-z]:[\\/]", name):
        return False
    parts = Path(name.replace("\\", "/")).parts
    return ".." not in parts and not name.endswith("/")


# --- 声明层 --------------------------------------------------------------------


def load_delivery(matrix: dict) -> dict:
    """取出交付形态声明；缺字段即显式失败（schema 已保证存在，这里做二次 fail-closed）。"""
    delivery = matrix.get("delivery")
    if not isinstance(delivery, dict):
        raise BoardBundleError(
            f"矩阵缺少 delivery 声明（bundle 命名与目标端安装布局）：{MATRIX_SCHEMA_REL} "
            "已把它列为必需字段，请先补声明（禁止在脚本里写默认路径）",
            ExitCode.PRECHECK,
        )
    bundle_conf = delivery.get("board_bundle") or {}
    install_conf = delivery.get("install") or {}
    required_bundle = ("name_template", "manifest_name", "checksum_suffix", "staging_dir_name")
    required_install = (
        "prefix",
        "config_dir",
        "log_dir",
        "systemd_dir",
        "service_unit",
        "env_file",
        "current_link",
        "python_min",
        "python_bin",
    )
    missing = [f"delivery.board_bundle.{key}" for key in required_bundle if key not in bundle_conf]
    missing += [f"delivery.install.{key}" for key in required_install if key not in install_conf]
    if missing:
        raise BoardBundleError("delivery 声明不完整，缺失字段：" + "、".join(missing), ExitCode.PRECHECK)
    return {"board_bundle": bundle_conf, "install": install_conf}


def read_board_profile(board_path: Path) -> dict:
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺 PyYAML
        raise BoardBundleError(
            f"当前解释器缺少 PyYAML（{exc}）：无法读取 BoardProfile", ExitCode.PRECHECK
        ) from exc
    if not board_path.is_file():
        raise BoardBundleError(f"BoardProfile 不存在：{board_path}", ExitCode.PRECHECK)
    try:
        data = yaml.safe_load(board_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise BoardBundleError(f"BoardProfile 不是合法 YAML：{board_path}（{exc}）", ExitCode.PRECHECK)
    if not isinstance(data, dict):
        raise BoardBundleError(f"BoardProfile 顶层必须是对象：{board_path}", ExitCode.PRECHECK)
    return data


def run_profile_check(
    root: Path, board_rel: str, python_exe: str, allow_unverified: bool
) -> dict:
    """调用步骤 03 的 `profile_check.py --board`（门禁复用，不重新实现）。"""
    args = [python_exe, PROFILE_CHECK_SCRIPT, "--board", board_rel]
    if allow_unverified:
        args.append("--allow-unverified")
    try:
        proc = subprocess.run(args, cwd=str(root), capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        raise BoardBundleError(
            f"无法执行 {PROFILE_CHECK_SCRIPT} --board（{exc}）：板卡声明门禁不可跳过",
            ExitCode.PRECHECK,
        ) from exc
    report = {}
    if proc.stdout.strip():
        try:
            report = json.loads(proc.stdout)
        except json.JSONDecodeError as exc:
            raise BoardBundleError(
                f"{PROFILE_CHECK_SCRIPT} 的 stdout 不是合法 JSON（{exc}）：输出契约被破坏，拒绝继续",
                ExitCode.PRECHECK,
            ) from exc
    stderr_lines = [line for line in proc.stderr.strip().splitlines() if line]
    return {
        "exit_code": proc.returncode,
        "report": report,
        "stderr_tail": stderr_lines[-1:],
        "argv": args[1:],
    }


def board_gate_summary(result: dict) -> dict:
    """只保留可入库字段：**不要**把 report 里的本机绝对路径（board_path）带进产物。"""
    report = result.get("report") or {}
    return {
        "exit_code": result["exit_code"],
        "verified": bool(report.get("verified")),
        "status": report.get("status"),
        "board": report.get("board"),
        "unverified_fields": list(report.get("unverified_fields") or []),
        "pending_fields": list(report.get("pending_fields") or []),
        "reasons": list(report.get("reasons") or []),
        "failures": list(report.get("failures") or []),
        "capabilities": list(report.get("capabilities") or []),
        "matrix_targets": list(report.get("matrix_targets") or []),
        "hyper_models": list(report.get("hyper_models") or []),
    }


# --- 板级 bundle 组装 -----------------------------------------------------------


def env_template_text(*, sdk_name: str, version: str, board: str, target_id: str) -> str:
    """生成 env 模板：占位符由安装期按 bundle 内的 delivery.install 渲染。"""
    lines = [
        "# IRAF SDK 板级运行环境（由 deploy/sdk/package_board_bundle.sh 从声明渲染，勿手改）",
        "# 占位符在安装期由 install.sh 用 bundle 内 config/sdk/package_matrix.yaml 渲染。",
        f"IRAF_SDK_NAME={sdk_name}",
        f"IRAF_SDK_VERSION={version}",
        f"IRAF_BOARD={board}",
        f"IRAF_SDK_TARGET={target_id}",
        "IRAF_HOME=@PREFIX@/@CURRENT_LINK@",
        "IRAF_CONFIG_DIR=@CONFIG_DIR@",
        "IRAF_LOG_DIR=@LOG_DIR@",
        "IRAF_ENV_FILE=@CONFIG_DIR@/@ENV_FILE@",
        "IRAF_SDK_LIB=@PREFIX@/@CURRENT_LINK@/lib",
        "IRAF_SERVICE_UNIT=@SERVICE_UNIT@",
        "IRAF_PYTHON=@PYTHON_BIN@",
        "PYTHONPATH=@PREFIX@/@CURRENT_LINK@/src:@PREFIX@/@CURRENT_LINK@/lib",
    ]
    return "\n".join(lines) + "\n"


def render_template(text: str, values: dict, *, label: str) -> str:
    """渲染模板：残留占位符或缺少取值都显式失败。"""
    rendered = text
    for placeholder, value in values.items():
        rendered = rendered.replace(placeholder, str(value))
    leftovers = sorted(set(re.findall(r"@[A-Z0-9_]+@", rendered)))
    if leftovers:
        raise BoardBundleError(
            f"{label} 存在未声明的占位符：" + "、".join(leftovers) + "（未知占位符必须显式失败）",
            ExitCode.BUILD,
        )
    unknown = [name for name in PLACEHOLDERS if name not in values]
    if unknown:
        raise BoardBundleError(
            f"渲染 {label} 时缺少占位符取值：" + "、".join(unknown) + "（模板与声明不一致）",
            ExitCode.BUILD,
        )
    return rendered


def render_env_template(text: str, values: dict) -> str:
    """向后兼容的薄封装：env 模板渲染。"""
    return render_template(text, values, label="env 模板")


def bundle_name(delivery: dict, *, board: str, version: str, arch: str) -> str:
    template = delivery["board_bundle"]["name_template"]
    name = template.replace("{board}", board).replace("{version}", version).replace("{arch}", arch)
    if "{" in name or "}" in name:
        raise BoardBundleError(
            f"bundle 命名模板含未识别占位符：{template}（只支持 {{board}}/{{version}}/{{arch}}）",
            ExitCode.PRECHECK,
        )
    return name


def _write_deterministic_tar(members: list[dict]) -> bytes:
    """确定性 tar.gz：成员排序、mtime=0、uid/gid=0、uname/gname 空、gzip mtime=0。"""
    buffer = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buffer, mtime=0) as gz:
        with tarfile.open(fileobj=gz, mode="w", format=tarfile.GNU_FORMAT) as tar:
            for member in sorted(members, key=lambda item: item["path"]):
                if member.get("source") is not None:
                    entry = tar.gettarinfo(str(member["source"]), arcname=member["path"])
                    payload = Path(member["source"]).read_bytes()
                else:
                    entry = tarfile.TarInfo(member["path"])
                    entry.size = len(member["bytes"])
                    entry.mode = 0o644
                    payload = member["bytes"]
                entry.mtime = 0
                entry.uid = 0
                entry.gid = 0
                entry.uname = ""
                entry.gname = ""
                tar.addfile(entry, io.BytesIO(payload))
    return buffer.getvalue()


def build_board_bundle(options: dict) -> tuple[Path, dict, dict]:
    """组装板级 bundle；返回 (bundle 路径, bundle 清单, 组装摘要)。"""
    repo_root = Path(options["repo_root"]).resolve()
    matrix_path = _abspath(repo_root, options.get("matrix") or MATRIX_REL)
    out_dir = _abspath(repo_root, options.get("out") or DEFAULT_OUT)
    python_exe = options.get("python") or "python3"
    allow_unverified = bool(options.get("allow_unverified"))

    if not options.get("board"):
        raise BoardBundleError("缺少 --board（BoardProfile 路径）", ExitCode.USAGE)
    board_path = _abspath(repo_root, options["board"])
    if not _within(board_path, repo_root):
        raise BoardBundleError(
            f"BoardProfile 必须在仓库内（bundle 成员只用相对路径）：{options['board']}",
            ExitCode.PRECHECK,
        )
    board_rel = relpath_posix(board_path, repo_root)

    matrix = load_matrix(repo_root, matrix_path)
    target = select_target(matrix, options.get("target"))
    delivery = load_delivery(matrix)
    version = read_declared_version(repo_root, matrix["sdk"]["version_ref"])
    sdk_name = matrix["sdk"]["name"]
    board = read_board_profile(board_path)
    board_meta = board.get("metadata") or {}
    board_spec = board.get("spec") or {}
    board_name = str(board_meta.get("name") or "")
    if not board_name:
        raise BoardBundleError(f"BoardProfile 缺少 metadata.name：{board_rel}", ExitCode.PRECHECK)
    declared_boards = [str(item) for item in (target.get("boards") or [])]
    if board_name not in declared_boards:
        raise BoardBundleError(
            f"BoardProfile {board_name} 未在产物矩阵目标 {target['id']} 的 boards 中声明："
            f"{declared_boards}（架构/标签会出现两个来源，拒绝组装）",
            ExitCode.PRECHECK,
        )

    # 门禁：板卡声明（复用 profile_check.py；未加 --allow-unverified 即退出 2）
    gate = run_profile_check(repo_root, board_rel, python_exe, allow_unverified=False)
    if gate["exit_code"] == 0:
        gate_final = gate
    elif not allow_unverified:
        summary = board_gate_summary(gate)
        reasons = summary["reasons"] or summary["failures"]
        raise BoardBundleError(
            f"BoardProfile {board_name} 含未实测声明（profile_check 退出码 {gate['exit_code']}），"
            f"默认拒绝组装：{len(reasons)} 项；确认要在未实测状态下打包请加 --allow-unverified",
            ExitCode.PRECHECK,
            reasons,
        )
    else:
        gate_final = run_profile_check(repo_root, board_rel, python_exe, allow_unverified=True)
        if gate_final["exit_code"] != 0:
            raise BoardBundleError(
                f"BoardProfile {board_name} 契约层失败（exit={gate_final['exit_code']}）："
                "--allow-unverified 不能放行契约层失败",
                ExitCode.PRECHECK,
                board_gate_summary(gate_final)["failures"],
            )
    gate_summary = board_gate_summary(gate_final)

    # 输入产物
    runtime_bundle = _abspath(
        repo_root,
        options.get("runtime")
        or (out_dir / RUNTIME_BUNDLE_TEMPLATE.format(version=version, arch=target["arch"])),
    )
    if not runtime_bundle.is_file():
        raise BoardBundleError(
            "runtime bundle 不存在："
            + display_path(runtime_bundle, repo_root)
            + "（先执行 bash deploy/sdk/build_sdk.sh）",
            ExitCode.PRECHECK,
        )
    sdk_wheel = _abspath(
        repo_root,
        options.get("sdk_wheel")
        or (
            out_dir
            / SDK_WHEEL_TEMPLATE.format(
                dist=re.sub(r"[^A-Za-z0-9.]+", "_", sdk_name), version=version
            )
        ),
    )
    if not sdk_wheel.is_file():
        raise BoardBundleError(
            "SDK wheel 不存在："
            + display_path(sdk_wheel, repo_root)
            + "（先执行 bash deploy/sdk/build_sdk.sh）",
            ExitCode.PRECHECK,
        )
    wheelhouse_dir = _abspath(
        repo_root, options.get("wheelhouse") or (Path(DEFAULT_WHEELHOUSE_ROOT) / target["id"])
    )
    if not wheelhouse_dir.is_dir():
        raise BoardBundleError(
            "wheelhouse 目录不存在："
            + display_path(wheelhouse_dir, repo_root)
            + "（先执行 bash deploy/sdk/fetch_wheelhouse.sh）",
            ExitCode.PRECHECK,
        )
    if not (wheelhouse_dir / "wheelhouse.json").is_file():
        raise BoardBundleError(
            f"wheelhouse 缺少 wheelhouse.json（{display_path(wheelhouse_dir, repo_root)}）："
            "无法确认抓取结果与声明一致",
            ExitCode.PRECHECK,
        )

    # 第二道标签门：对**目录**再校验一次（不信任抓取时的结论）
    tag_check = subprocess.run(
        [
            python_exe,
            str(repo_root / "deploy" / "sdk" / "check_wheel_tags.py"),
            "--repo-root",
            str(repo_root),
            "--matrix",
            display_path(matrix_path, repo_root),
            "--target",
            target["id"],
            "verify",
            "--dir",
            display_path(wheelhouse_dir, repo_root),
        ],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
        timeout=300,
    )
    if tag_check.returncode != 0:
        tail = [line for line in tag_check.stderr.strip().splitlines() if line][-1:]
        raise BoardBundleError(
            f"wheelhouse 平台标签校验失败（退出码 {tag_check.returncode}）：{tail}",
            ExitCode.VERIFY if tag_check.returncode == 4 else ExitCode.PRECHECK,
        )

    staging_root = out_dir / f".board-bundle-staging-{board_name}"
    if staging_root.exists():
        shutil.rmtree(staging_root)
    staging_root.mkdir(parents=True, exist_ok=True)
    try:
        members, sections = _collect_members(
            repo_root=repo_root,
            staging_root=staging_root,
            board_path=board_path,
            board_name=board_name,
            target=target,
            runtime_bundle=runtime_bundle,
            wheelhouse_dir=wheelhouse_dir,
            sdk_wheel=sdk_wheel,
            delivery=delivery,
            version=version,
            sdk_name=sdk_name,
        )
        file_entries = [
            {
                "path": member["path"],
                "sha256": sha256_file(member["source"]),
                "size_bytes": member["source"].stat().st_size,
            }
            for member in members
        ]
        manifest_name = delivery["board_bundle"]["manifest_name"]
        post_verify_member = f"scripts/{Path(POST_VERIFY_SCRIPT).name}"
        post_verify_present = any(member["path"] == post_verify_member for member in members)
        glibc_requirement = platform_tag_glibc(target["platform_tag"])
        manifest = {
            "schema_version": BUNDLE_SCHEMA_VERSION,
            "bundle": {
                "board": board_name,
                "target": target["id"],
                "arch": target["arch"],
                "python_tag": target["python_tag"],
                "platform_tag": target["platform_tag"],
                "glibc_min_declared": (
                    "%d.%d" % glibc_requirement if glibc_requirement else "unverified"
                ),
                "version": version,
                "manifest_member": manifest_name,
                "staging_dir_name": delivery["board_bundle"]["staging_dir_name"],
                "checksum_suffix": delivery["board_bundle"]["checksum_suffix"],
            },
            "board": {
                "profile_member": f"board/{board_path.name}",
                "status": gate_summary["status"],
                "verified": gate_summary["verified"],
                "allow_unverified_at_build": bool(allow_unverified),
                "hyper_models": gate_summary["hyper_models"],
                "capabilities": gate_summary["capabilities"],
                "limits": dict((board_spec.get("limits") or {})),
                "unverified_fields": gate_summary["unverified_fields"],
                "pending_fields": gate_summary["pending_fields"],
                "profile_check_exit_code": gate_summary["exit_code"],
            },
            "sdk": {
                "name": sdk_name,
                "version": version,
                "wheel_member": f"sdk/{sdk_wheel.name}",
                "sha256": sha256_file(sdk_wheel),
            },
            "runtime": {
                "bundle_member": f"runtime/{runtime_bundle.name}",
                "sha256": sha256_file(runtime_bundle),
                "size_bytes": runtime_bundle.stat().st_size,
            },
            "wheelhouse": {
                "dir_member": f"wheelhouse/{target['id']}",
                "manifest_member": f"wheelhouse/{target['id']}/wheelhouse.json",
                "verify_member": f"wheelhouse/{target['id']}/verify-tags.json",
                "index_url": target["index_url"],
                "declared": {
                    "wheels": list(target.get("wheels") or []),
                    "pure_python": list(target.get("pure_python") or []),
                },
                "wheels": sections["wheelhouse_wheels"],
                "wheel_count": len(sections["wheelhouse_wheels"]),
                "tag_check_exit_code": tag_check.returncode,
            },
            "scripts": {
                "install_member": "scripts/install.sh",
                "profile_check_member": f"scripts/{Path(PROFILE_CHECK_SCRIPT).name}",
                "missing": sections["scripts_missing"],
                "members": sorted(
                    member["path"] for member in members if member["path"].startswith("scripts/")
                ),
            },
            "post_verify": {
                "script": POST_VERIFY_SCRIPT,
                "member": post_verify_member,
                "present": post_verify_present,
                "status": "available" if post_verify_present else "pending_step_09",
                "required_in_real_install": True,
                "reason": (
                    "后置校验脚本随 bundle 交付，真实安装必须执行"
                    if post_verify_present
                    else "verify.sh 由步骤 09 交付；真实安装遇到该状态必须失败并回滚（不得静默跳过）"
                ),
            },
            "systemd": {
                "unit_member": sections["unit_member"],
                "unit_name": delivery["install"]["service_unit"],
            },
            "env": {
                "template_member": sections["env_template_member"],
                "template_sha256": sections["env_template_sha256"],
            },
            "gates": {
                "matrix": MATRIX_SCHEMA_REL,
                "matrix_target": target["id"],
                "delivery_declared": True,
                "board_in_target_boards": True,
                "wheelhouse_tag_check": tag_check.returncode,
                "install_layout_declared": sorted(delivery["install"].keys()),
            },
            "git": git_state(repo_root),
            "files": file_entries,
            "notes": [
                "x86-first 战役（iraf-24h）：本 bundle 在开发端 x86_64 组装；目标端安装验收 DEFERRED（板卡不在场）",
                f"board.verified={gate_summary['verified']}：板卡 OS/Python/平台标签仍未实测"
                f"（unverified {len(gate_summary['unverified_fields'])} 项 / "
                f"pending {len(gate_summary['pending_fields'])} 项）",
                "signature=unverified：签名流程未落地，本 bundle 不得当作已签名分发",
                "bundle 清单只含 bundle 相对路径（绝对路径与 `..` 由组装期门禁拒绝）",
            ],
        }
        if sections["scripts_missing"]:
            manifest["notes"].append(
                f"缺失脚本 {len(sections['scripts_missing'])} 个（后续步骤交付）："
                + "、".join(item["path"] for item in sections["scripts_missing"])
            )
        violations = absolute_path_violations(manifest)
        if violations:
            raise BoardBundleError("bundle 清单含本机绝对路径，拒绝写出", ExitCode.BUILD, violations)

        manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        all_members = members + [
            {"path": manifest_name, "bytes": manifest_bytes, "source": None}
        ]
        out_dir.mkdir(parents=True, exist_ok=True)
        name = bundle_name(delivery, board=board_name, version=version, arch=target["arch"])
        bundle_path = out_dir / name
        bundle_bytes = _write_deterministic_tar(all_members)
        bundle_path.write_bytes(bundle_bytes)
        digest = sha256_bytes(bundle_bytes)
        checksum = out_dir / (name + delivery["board_bundle"]["checksum_suffix"])
        checksum.write_text(f"{digest}  {name}\n", encoding="utf-8")
    finally:
        if staging_root.exists():
            shutil.rmtree(staging_root, ignore_errors=True)

    summary = {
        "schema_version": BUILD_SUMMARY_SCHEMA_VERSION,
        "board": board_name,
        "board_profile": board_rel,
        "target": target["id"],
        "version": version,
        "bundle": display_path(bundle_path, repo_root),
        "checksum_file": display_path(checksum, repo_root),
        "sha256": digest,
        "size_bytes": bundle_path.stat().st_size,
        "member_count": len(all_members),
        "wheelhouse_wheel_count": len(sections["wheelhouse_wheels"]),
        "scripts_missing": [item["path"] for item in sections["scripts_missing"]],
        "post_verify_status": manifest["post_verify"]["status"],
        "verified": gate_summary["verified"],
        "allow_unverified": bool(allow_unverified),
        "exit_code": ExitCode.OK,
    }
    return bundle_path, manifest, summary


def _collect_members(
    *,
    repo_root: Path,
    staging_root: Path,
    board_path: Path,
    board_name: str,
    target: dict,
    runtime_bundle: Path,
    wheelhouse_dir: Path,
    sdk_wheel: Path,
    delivery: dict,
    version: str,
    sdk_name: str,
) -> tuple[list[dict], dict]:
    """整理待打包成员 [(bundle 相对路径, 源文件)]，并产出各段元信息。"""
    members: list[dict] = []
    scripts_missing: list[dict] = []

    def add(bundle_rel: str, source: Path) -> None:
        if not is_relative_member(bundle_rel):
            raise BoardBundleError(f"bundle 成员路径非法：{bundle_rel}", ExitCode.BUILD)
        if not source.is_file():
            raise BoardBundleError(f"成员源文件不存在：{source}", ExitCode.PRECHECK)
        members.append({"path": bundle_rel, "source": source})

    add(f"board/{board_path.name}", board_path)
    add(f"runtime/{runtime_bundle.name}", runtime_bundle)
    add(f"sdk/{sdk_wheel.name}", sdk_wheel)

    wheel_entries = []
    for wheel in sorted(wheelhouse_dir.iterdir()):
        if not wheel.is_file():
            continue
        if wheel.suffix != ".whl" and wheel.name not in ("wheelhouse.json", "verify-tags.json"):
            continue
        add(f"wheelhouse/{target['id']}/{wheel.name}", wheel)
        if wheel.suffix == ".whl":
            wheel_entries.append(
                {
                    "name": wheel.name,
                    "sha256": sha256_file(wheel),
                    "size_bytes": wheel.stat().st_size,
                }
            )

    for rel in BUNDLE_SCRIPT_FILES:
        add(f"scripts/{Path(rel).name}", repo_root / rel)
    for rel, owner in sorted(BUNDLE_PENDING_SCRIPTS.items()):
        source = repo_root / rel
        if source.is_file():
            add(f"scripts/{source.name}", source)
        else:
            scripts_missing.append({"path": rel, "delivered_by": owner})

    unit = repo_root / "deploy" / "sdk" / delivery["install"]["service_unit"]
    add(f"systemd/{unit.name}", unit)

    env_text = env_template_text(
        sdk_name=sdk_name, version=version, board=board_name, target_id=target["id"]
    )
    env_name = f"{delivery['install']['env_file']}.template"
    env_path = staging_root / "env" / env_name
    env_path.parent.mkdir(parents=True, exist_ok=True)
    env_path.write_text(env_text, encoding="utf-8")
    add(f"env/{env_name}", env_path)

    sections = {
        "wheelhouse_wheels": wheel_entries,
        "env_template_member": f"env/{env_name}",
        "env_template_sha256": sha256_bytes(env_text.encode("utf-8")),
        "scripts_missing": scripts_missing,
        "unit_member": f"systemd/{unit.name}",
    }
    return members, sections


# --- bundle 校验与解包（目标端预检） --------------------------------------------


def read_bundle_members(bundle_path: Path) -> tuple[dict, list[dict]]:
    """读取 bundle 内清单与成员内容（不落盘）；成员路径非法即失败。"""
    entries: list[dict] = []
    try:
        with tarfile.open(bundle_path, "r:gz") as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    raise BoardBundleError(
                        f"bundle 含非普通文件成员：{member.name}（拒绝解包）", ExitCode.VERIFY
                    )
                if not is_relative_member(member.name):
                    raise BoardBundleError(
                        f"bundle 成员路径非法（绝对路径或含 ..）：{member.name}", ExitCode.VERIFY
                    )
                handle = tar.extractfile(member)
                entries.append({"path": member.name, "data": handle.read() if handle else b""})
    except (tarfile.TarError, OSError, EOFError) as exc:
        raise BoardBundleError(
            f"bundle 无法解析（gzip/tar 破损）：{bundle_path}（{exc}）", ExitCode.VERIFY
        ) from exc

    manifest = None
    for entry in entries:
        if entry["path"].endswith(".json") and "/" not in entry["path"]:
            try:
                manifest = json.loads(entry["data"].decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise BoardBundleError(
                    f"bundle 清单不是合法 JSON（{entry['path']}）：{exc}", ExitCode.PRECHECK
                ) from exc
            break
    if not isinstance(manifest, dict):
        raise BoardBundleError(
            "bundle 内找不到清单 JSON（iraf.board-bundle/v1）：无法校验成员，拒绝安装",
            ExitCode.PRECHECK,
        )
    return manifest, entries


def verify_bundle_contract(bundle_path: Path) -> tuple[dict, list[dict], list[str]]:
    """校验 bundle 契约与逐成员 SHA-256；返回 (清单, 成员, 违规清单)。"""
    manifest, entries = read_bundle_members(bundle_path)
    violations: list[str] = []
    if manifest.get("schema_version") != BUNDLE_SCHEMA_VERSION:
        violations.append(
            f"清单 schema_version 不是 {BUNDLE_SCHEMA_VERSION}：{manifest.get('schema_version')!r}"
        )
    required = ("bundle", "board", "sdk", "runtime", "wheelhouse", "scripts", "post_verify", "files")
    missing = [key for key in required if key not in manifest]
    if missing:
        violations.append("清单缺少必需段：" + "、".join(missing))
    violations.extend(absolute_path_violations(manifest, "manifest"))
    listed = {item.get("path"): item for item in (manifest.get("files") or [])}
    manifest_member = (manifest.get("bundle") or {}).get("manifest_member")
    for entry in entries:
        if entry["path"] == manifest_member:
            continue
        record = listed.get(entry["path"])
        if record is None:
            violations.append(f"bundle 内成员未在清单登记：{entry['path']}")
            continue
        if record.get("size_bytes") != len(entry["data"]):
            violations.append(
                f"成员大小与清单不符：{entry['path']}"
                f"（清单 {record.get('size_bytes')} / 实际 {len(entry['data'])}）"
            )
        actual = sha256_bytes(entry["data"])
        if record.get("sha256") != actual:
            violations.append(
                f"成员被篡改（SHA-256 不符）：{entry['path']}"
                f"（清单 {str(record.get('sha256'))[:16]}… / 实际 {actual[:16]}…）"
            )
    for path in listed:
        if not any(entry["path"] == path for entry in entries):
            violations.append(f"清单登记了 bundle 内不存在的成员：{path}")
    return manifest, entries, violations


def extract_bundle(entries: list[dict], dest: Path, manifest_member: str | None) -> None:
    """逐成员写盘（已通过路径安全检查）：不依赖 tarfile 的默认解包行为。"""
    dest.mkdir(parents=True, exist_ok=True)
    for entry in entries:
        target = dest / entry["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(entry["data"])
    if manifest_member and not (dest / manifest_member).is_file():
        raise BoardBundleError(f"解包后缺少清单 {manifest_member}", ExitCode.PRECHECK)


# --- 宿主事实与预检 -------------------------------------------------------------


def read_install_layout(staging: Path) -> dict:
    """从 bundle 内（解包后）的矩阵读取目标端布局声明。"""
    matrix_path = staging / MATRIX_REL
    if not matrix_path.is_file():
        raise BoardBundleError(
            f"bundle 内缺少矩阵声明 {MATRIX_REL}：无法确定安装布局（拒绝猜默认路径）",
            ExitCode.PRECHECK,
        )
    try:
        matrix = load_matrix(staging, matrix_path)
    except ManifestError as exc:
        raise BoardBundleError(
            "bundle 内声明无法校验（目标端需要 PyYAML + jsonschema）："
            f"{exc}；请先用 bundle 内 wheelhouse 里的 pyyaml/jsonschema 安装这两个依赖后重跑",
            exc.exit_code,
        ) from exc
    return {"matrix": matrix, "delivery": load_delivery(matrix), "matrix_path": matrix_path}


_HOST_PROBE = (
    "import json,platform,shutil,sys\n"
    "facts={'machine':platform.machine(),'python_version':'%d.%d.%d'%sys.version_info[:3],"
    "'python_major_minor':'%d.%d'%sys.version_info[:2],'system':platform.system(),"
    "'release':platform.release()}\n"
    "try:\n"
    "    facts['disk_free_mb']=shutil.disk_usage('/').free//(1024*1024)\n"
    "except OSError:\n"
    "    facts['disk_free_mb']=None\n"
    "mem=None\n"
    "try:\n"
    "    with open('/proc/meminfo') as handle:\n"
    "        for line in handle:\n"
    "            if line.startswith('MemAvailable:'):\n"
    "                mem=int(line.split()[1])//1024\n"
    "                break\n"
    "except OSError:\n"
    "    mem=None\n"
    "facts['mem_available_mb']=mem\n"
    "print(json.dumps(facts))\n"
)


def collect_host_facts(python_exe: str) -> dict:
    """采集宿主事实：架构 / Python / glibc / 磁盘 / RAM / 依赖 / systemd。"""
    try:
        proc = subprocess.run(
            [python_exe, "-c", _HOST_PROBE], capture_output=True, text=True, timeout=60, cwd="/tmp"
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise BoardBundleError(f"无法探测宿主事实（{python_exe}）：{exc}", ExitCode.PRECHECK) from exc
    if proc.returncode != 0:
        tail = [line for line in proc.stderr.strip().splitlines() if line][-1:]
        raise BoardBundleError(
            f"宿主事实探测失败（{python_exe} 退出码 {proc.returncode}）：{tail}", ExitCode.PRECHECK
        )
    try:
        facts = json.loads(proc.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError) as exc:
        raise BoardBundleError(f"宿主事实探测输出无法解析：{exc}", ExitCode.PRECHECK) from exc
    facts["glibc"] = detect_glibc_version()
    facts["python_exe"] = python_exe
    systemctl = shutil.which("systemctl")
    facts["systemd_available"] = bool(systemctl)
    facts["pip_available"] = module_available(python_exe, "pip")
    facts["yaml_available"] = module_available(python_exe, "yaml")
    facts["jsonschema_available"] = module_available(python_exe, "jsonschema")
    return facts


def module_available(python_exe: str, module: str) -> bool:
    code = f"import importlib.util,sys;sys.exit(0 if importlib.util.find_spec({module!r}) else 1)"
    try:
        proc = subprocess.run(
            [python_exe, "-c", code], capture_output=True, text=True, timeout=60, cwd="/tmp"
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return proc.returncode == 0


def precheck_violations(
    *,
    facts: dict,
    manifest: dict,
    target: dict,
    board_report: dict,
    disk_needed_mb: int,
    layout: dict,
    allow_unverified: bool,
) -> list[dict]:
    """逐项比对声明与实测；返回 [{kind, message}]（空 = 全部通过）。"""
    violations: list[dict] = []

    def add(kind: str, message: str) -> None:
        violations.append({"kind": kind, "message": message})

    if not board_report.get("verified") and not allow_unverified:
        reasons = board_report.get("reasons") or board_report.get("failures") or []
        add(
            "board_profile",
            f"BoardProfile 未通过实测门禁（profile_check 退出码 {board_report.get('exit_code')}）："
            f"{len(reasons)} 项未实测/未确定；真实安装需显式 --allow-unverified",
        )
    elif not board_report.get("verified"):
        add(
            "board_profile_warning",
            f"BoardProfile 仍为 unverified（{len(board_report.get('unverified_fields') or [])} 项），"
            "已由 --allow-unverified 显式放行：本次安装不得作为板卡验收证据",
        )

    expected_arch = target.get("arch")
    if facts.get("machine") != expected_arch:
        add(
            "arch",
            f"宿主架构 {facts.get('machine')!r} 与声明 {expected_arch!r} 不符"
            "（aarch64 的 wheel 不能在 x86_64 上安装）",
        )

    python_min = parse_version(layout.get("python_min", ""))
    host_python = parse_version(facts.get("python_version", "")) or ()
    if python_min and tuple(host_python[:2]) < tuple(python_min[:2]):
        add(
            "python",
            f"宿主 Python {facts.get('python_version')} 低于声明的最低版本 {layout.get('python_min')}",
        )
    python_tag = python_tag_version(target.get("python_tag", ""))
    if python_tag and host_python and python_tag != tuple(host_python[:2]):
        add(
            "python_tag",
            f"宿主 Python {facts.get('python_major_minor')} 与声明的 ABI 标签 "
            f"{target.get('python_tag')} 不一致（wheel 的 cp 标签不匹配）",
        )

    glibc_requirement = platform_tag_glibc(target.get("platform_tag", ""))
    host_glibc = parse_version(facts.get("glibc") or "")
    if glibc_requirement is None:
        add("glibc", f"无法从平台标签解析 glibc 要求：{target.get('platform_tag')!r}")
    elif host_glibc is None:
        add("glibc", "无法判定宿主 glibc 版本（ctypes 与 platform 两条路径都失败），拒绝放行")
    elif tuple(host_glibc[:2]) < tuple(glibc_requirement):
        add(
            "glibc",
            f"宿主 glibc {facts.get('glibc')} 低于 {target.get('platform_tag')} 要求的 "
            f"{glibc_requirement[0]}.{glibc_requirement[1]}",
        )

    disk_free = facts.get("disk_free_mb")
    if disk_free is None:
        add("disk", "无法判定宿主可用磁盘，拒绝放行")
    elif disk_free < disk_needed_mb:
        add(
            "disk",
            f"宿主可用磁盘 {disk_free} MB < 本次安装所需 {disk_needed_mb} MB"
            "（解包后大小 × 余量系数）",
        )

    limits = (manifest.get("board") or {}).get("limits") or {}
    declared_ram = limits.get("ram_mb")
    if declared_ram in (None, "unverified", "pending"):
        add(
            "ram_declaration",
            "BoardProfile 未声明 limits.ram_mb（unverified）：无法比对可用内存，真实安装不得默认放行",
        )
    elif isinstance(declared_ram, int) and (facts.get("mem_available_mb") or 0) < declared_ram:
        add("ram", f"宿主可用内存 {facts.get('mem_available_mb')} MB < 声明下限 {declared_ram} MB")

    for key, label in (("yaml_available", "PyYAML"), ("jsonschema_available", "jsonschema")):
        if not facts.get(key):
            add(
                "dependency",
                f"宿主 Python 缺少 {label}：声明层契约校验不可跳过；请先用 bundle 内 wheelhouse 安装"
                "该依赖后重跑（真实安装不得跳过契约校验）",
            )
    if not facts.get("systemd_available"):
        add("systemd", "宿主没有 systemctl：无法注册/启动板级服务")
    return violations


def required_disk_mb(manifest: dict) -> int:
    """解包后所需空间 = runtime 解包估算 + sdk wheel + wheelhouse，再乘余量系数。"""
    runtime_size = int((manifest.get("runtime") or {}).get("size_bytes") or 0)
    sdk_size = sum(
        int(item.get("size_bytes") or 0)
        for item in manifest.get("files") or []
        if str(item.get("path", "")).startswith("sdk/")
    )
    wheelhouse_size = sum(
        int(item.get("size_bytes") or 0)
        for item in (manifest.get("wheelhouse") or {}).get("wheels") or []
    )
    uncompressed = runtime_size * RUNTIME_EXPANSION_FACTOR + sdk_size + wheelhouse_size
    return max(1, (uncompressed // (1024 * 1024) + 1) * DISK_HEADROOM_FACTOR)


# --- 目标端安装 ----------------------------------------------------------------


class Installer:
    """目标端安装流水线。

    apply（真实安装）与 rehearsal（--dry-run）走**同一段代码**，差别只在判定策略（写在本类里，
    不散落在 shell）：rehearsal 把不合格项记进 `blockers` 并最终退出 0（演练完成），apply 把同一
    批不合格项当作预检失败并给出对应退出码。rehearsal 的全部文件系统副作用都在 `--root` 沙箱内，
    且不执行 systemctl、不安装目标端二进制 wheel（aarch64 wheel 在 x86 上装不了，只登记 DEFERRED）。
    """

    def __init__(self, options: dict):
        self.mode = "rehearsal" if options.get("dry_run") else "apply"
        self.allow_unverified = bool(options.get("allow_unverified"))
        self.python_exe = options.get("python") or "python3"
        if self.mode == "rehearsal" and not options.get("root"):
            raise BoardBundleError(
                "演练模式必须显式给出 --root（沙箱目录），不得落到真实系统路径", ExitCode.USAGE
            )
        self.root = Path(options.get("root") or "/").resolve()
        bundle_value = options.get("bundle")
        if not bundle_value:
            raise BoardBundleError("缺少 --bundle（板级 bundle tar.gz）", ExitCode.USAGE)
        self.bundle_path = Path(str(bundle_value))
        if not self.bundle_path.is_absolute():
            self.bundle_path = Path.cwd() / self.bundle_path
        self.staging: Path | None = None
        self.state: dict | None = None
        self.layout: dict = {}
        self.bundle_manifest: dict = {}
        self.runtime_dirs: list[str] = []
        self.report: dict = {
            "schema_version": INSTALL_REPORT_SCHEMA_VERSION,
            "mode": self.mode,
            "install_performed": False,
            "version_dir_created": False,
            "evidence_scope": "rehearsal_only" if self.mode == "rehearsal" else "target_end_install",
            "simulation": True,
            "bundle": {},
            "stages": [],
            "blockers": [],
            "warnings": [],
            "exit_code": ExitCode.OK,
        }

    # --- 报告 ---

    def stage(self, name: str, status: str, detail: str, **extra) -> None:
        entry = {"stage": name, "status": status, "detail": detail}
        entry.update(extra)
        self.report["stages"].append(entry)
        info(f"[{status}] {name}：{detail}")

    def blocker(self, kind: str, message: str) -> None:
        self.report["blockers"].append({"kind": kind, "message": message})

    # --- 阶段 ---

    def verify_bundle_file(self) -> None:
        if not self.bundle_path.is_file():
            raise BoardBundleError(f"bundle 不存在：{self.bundle_path}", ExitCode.PRECHECK)
        checksum_path = Path(str(self.bundle_path) + ".sha256")
        if not checksum_path.is_file():
            raise BoardBundleError(
                f"缺少伴随校验和文件 {checksum_path.name}（不信任无校验和的 bundle；"
                "由 package_board_bundle.sh 产出）",
                ExitCode.PRECHECK,
            )
        fields = checksum_path.read_text(encoding="utf-8").strip().split()
        expected = fields[0] if fields else ""
        if len(expected) != 64:
            raise BoardBundleError(
                f"伴随校验和文件内容非法：{checksum_path.name}（期望 64 位 SHA-256）",
                ExitCode.PRECHECK,
            )
        actual = sha256_file(self.bundle_path)
        self.report["bundle"] = {
            "name": self.bundle_path.name,
            "sha256": actual,
            "expected_sha256": expected,
            "size_bytes": self.bundle_path.stat().st_size,
        }
        if actual != expected:
            raise BoardBundleError(
                f"bundle SHA-256 与伴随校验和不符：清单 {expected[:16]}… / 实际 {actual[:16]}…",
                ExitCode.VERIFY,
            )
        self.stage("P1 bundle 校验和", "PASS", f"sha256={actual[:16]}…（与伴随文件一致）")

    def extract_to_staging(self) -> dict:
        manifest, entries, violations = verify_bundle_contract(self.bundle_path)
        if violations:
            raise BoardBundleError(
                f"bundle 契约校验失败：{len(violations)} 处不符", ExitCode.VERIFY, violations
            )
        self.bundle_manifest = manifest
        staging_name = (manifest.get("bundle") or {}).get("staging_dir_name")
        if not staging_name or not re.match(r"^[a-z0-9][a-z0-9._-]*$", str(staging_name)):
            raise BoardBundleError(
                f"bundle 清单缺少合法的 staging_dir_name：{staging_name!r}"
                "（声明缺失必须显式失败，不猜默认目录名）",
                ExitCode.PRECHECK,
            )
        self.staging = self.root / str(staging_name)
        if self.staging.exists():
            shutil.rmtree(self.staging)
        extract_bundle(entries, self.staging, (manifest.get("bundle") or {}).get("manifest_member"))
        # runtime bundle 就地展开到暂存根：安装布局声明（config/sdk/package_matrix.yaml）在被校验
        # 之前必须先可见，否则预检只能"猜"路径——本步实测发现的顺序缺陷，禁止用默认值绕过。
        runtime_member = str((manifest.get("runtime") or {}).get("bundle_member"))
        runtime_top_dirs = self._extract_runtime(self.staging / runtime_member, self.staging)
        if not runtime_top_dirs:
            raise BoardBundleError(
                f"runtime bundle 内没有普通文件成员：{runtime_member}（拒绝继续）", ExitCode.PRECHECK
            )
        self.runtime_dirs = runtime_top_dirs
        self.report["runtime_top_dirs"] = runtime_top_dirs
        self.report["bundle"].update(
            {
                "manifest_schema": manifest.get("schema_version"),
                "board": (manifest.get("bundle") or {}).get("board"),
                "version": (manifest.get("bundle") or {}).get("version"),
                "target": (manifest.get("bundle") or {}).get("target"),
                "member_count": len(entries),
                "post_verify_status": (manifest.get("post_verify") or {}).get("status"),
            }
        )
        self.stage(
            "P2 bundle 解包与契约",
            "PASS",
            f"成员 {len(entries)} 个解包到 {staging_name}/，逐成员 SHA-256 与清单一致",
        )
        return manifest

    def precheck(self, manifest: dict) -> None:
        assert self.staging is not None
        layout_info = read_install_layout(self.staging)
        self.layout = layout_info["delivery"]["install"]
        bundle_section = manifest.get("bundle") or {}
        target = {
            "id": bundle_section.get("target"),
            "arch": bundle_section.get("arch"),
            "python_tag": bundle_section.get("python_tag"),
            "platform_tag": bundle_section.get("platform_tag"),
        }
        self.stage(
            "P3 bundle 内声明",
            "PASS",
            f"矩阵 {MATRIX_REL} 契约合法；目标 {target['id']}（arch={target['arch']}）",
        )
        facts = collect_host_facts(self.python_exe)
        self.report["host_facts"] = facts
        board_rel = (manifest.get("board") or {}).get("profile_member") or ""
        if not (self.staging / str(board_rel)).is_file():
            raise BoardBundleError(f"bundle 内 BoardProfile 缺失：{board_rel}", ExitCode.PRECHECK)
        result = run_profile_check(self.staging, str(board_rel), self.python_exe, self.allow_unverified)
        gate = board_gate_summary(result)
        self.report["board_profile_check"] = gate
        needed = required_disk_mb(manifest)
        self.report["disk_needed_mb"] = needed
        violations = precheck_violations(
            facts=facts,
            manifest=manifest,
            target=target,
            board_report=gate,
            disk_needed_mb=needed,
            layout=self.layout,
            allow_unverified=self.allow_unverified,
        )
        blocking = [item for item in violations if item["kind"] != "board_profile_warning"]
        for item in violations:
            if item["kind"] == "board_profile_warning":
                self.report["warnings"].append(item["message"])
        self.report["precheck_violations"] = blocking
        if self.mode == "rehearsal":
            for item in blocking:
                self.blocker(item["kind"], item["message"])
            self.stage(
                "P4 宿主事实与声明比对",
                "REHEARSAL_BLOCKED" if blocking else "PASS",
                f"实测事实已记录；不合格项 {len(blocking)} 项（演练只登记：真实安装会据此拒绝）",
                violations=blocking,
            )
        elif blocking:
            raise BoardBundleError(
                f"预检失败：{len(blocking)} 项不合格",
                ExitCode.PRECHECK,
                [item["message"] for item in blocking],
            )
        else:
            self.stage("P4 宿主事实与声明比对", "PASS", "架构/Python/glibc/磁盘/RAM/依赖/服务 全部匹配")

    def install_tree(self, manifest: dict) -> None:
        assert self.staging is not None
        version = str((manifest.get("bundle") or {}).get("version"))
        prefix = self.root / self.layout["prefix"].lstrip("/")
        version_dir = prefix / version
        # 先记录**当前**激活链接指向（可能不是本次安装建的）：任何阶段失败后回滚都必须恢复到它，
        # 否则"安装失败"会顺手破坏上一版可用版本（本步验收实测到的缺陷）。
        existing_link = prefix / self.layout["current_link"]
        previous_target = os.readlink(existing_link) if existing_link.is_symlink() else None
        self.state = {
            "version_dir": version_dir,
            "prefix_path": prefix,
            "layout": self.layout,
            "previous": previous_target,
        }
        files = {item["path"]: item for item in manifest.get("files") or []}
        idempotent = False
        if version_dir.is_dir() and any(version_dir.iterdir()):
            violations = self.compare_installed(version_dir, manifest)
            if violations:
                raise BoardBundleError(
                    f"已存在的安装目录与 bundle 不一致（{len(violations)} 处）：拒绝覆盖",
                    ExitCode.VERIFY,
                    violations,
                )
            idempotent = True
            self.stage(
                "I1 版本目录",
                "SKIPPED_IDEMPOTENT",
                f"{self.layout['prefix']}/{version} 已存在且与 bundle 逐文件一致（幂等：不重复写入）",
            )
        else:
            if version_dir.exists():
                shutil.rmtree(version_dir)
            prefix.mkdir(parents=True, exist_ok=True)
            version_dir.mkdir(parents=True, exist_ok=True)
            self.report["version_dir_created"] = True
            try:
                for top in self.runtime_dirs:
                    shutil.copytree(self.staging / top, version_dir / top)
                for rel in sorted(files):
                    if rel.startswith(("runtime/", "board/", "sdk/", "env/")):
                        continue
                    if rel.startswith("scripts/"):
                        self._copy(self.staging / rel, version_dir / "scripts" / Path(rel).name)
                    else:
                        self._copy(self.staging / rel, version_dir / rel)
                board_member = str((manifest.get("board") or {}).get("profile_member"))
                self._copy(
                    self.staging / board_member, version_dir / "board" / Path(board_member).name
                )
                sdk_member = str((manifest.get("sdk") or {}).get("wheel_member"))
                self._copy(self.staging / sdk_member, version_dir / "sdk" / Path(sdk_member).name)
                env_member = str((manifest.get("env") or {}).get("template_member"))
                self._copy(self.staging / env_member, version_dir / env_member)
            except (OSError, shutil.Error) as exc:
                raise BoardBundleError(
                    f"安装目录写入失败：{exc}", ExitCode.BUILD
                ) from exc
            self.stage(
                "I1 版本目录",
                "PASS",
                f"{self.layout['prefix']}/{version} 已建立（runtime 解包 + scripts/wheelhouse/systemd/env 就位）",
            )

        env_src = version_dir / str((manifest.get("env") or {}).get("template_member"))
        config_dir = self.root / self.layout["config_dir"].lstrip("/")
        values = self.render_values(manifest)
        rendered = render_template(
            env_src.read_text(encoding="utf-8"), values, label="env 模板"
        )
        config_dir.mkdir(parents=True, exist_ok=True)
        (config_dir / self.layout["env_file"]).write_text(rendered, encoding="utf-8")
        self.stage(
            "I2 写 env",
            "PASS",
            f"{self.layout['config_dir']}/{self.layout['env_file']}（占位符已按声明渲染）",
        )

        wheel = version_dir / "sdk" / Path(str((manifest.get("sdk") or {}).get("wheel_member"))).name
        if not idempotent:
            self._pip_install_sdk(wheel, version_dir / "lib")
        self.stage(
            "I3 离线安装 SDK wheel",
            "SKIPPED_IDEMPOTENT" if idempotent else "PASS",
            f"{wheel.name} → {self.layout['prefix']}/{version}/lib（--no-index --no-deps，纯 Python）",
        )

        declared = (manifest.get("wheelhouse") or {}).get("declared") or {}
        packages = sorted(str(name) for name in declared.get("wheels", [])) + sorted(
            str(name) for name in declared.get("pure_python", [])
        )
        wheel_count = len((manifest.get("wheelhouse") or {}).get("wheels") or [])
        if self.mode == "rehearsal":
            self.stage(
                "I4 目标端二进制 wheelhouse",
                "DEFERRED",
                f"{wheel_count} 个 wheel（{bundle_section_target(manifest)}）不能在 x86_64 开发端安装："
                "演练只登记，目标端执行同一命令",
                deferred=BINARY_WHEELS_DEFERRED,
            )
        else:
            self._pip_install_wheelhouse(version_dir, packages)

        self.state["previous"] = self.switch_current(
            prefix, self.layout["current_link"], version
        )
        self.report["link_switch"] = {
            "link": f"{self.layout['prefix']}/{self.layout['current_link']}",
            "old_target": self.state["previous"],
            "new_target": version,
        }
        self.stage(
            "I5 符号链接切换",
            "PASS",
            f"{self.layout['prefix']}/{self.layout['current_link']} → {version}"
            f"（旧目标 {self.state['previous'] or '无'}）",
        )
        self.report["install_performed"] = not idempotent
        self.report["install_scope"] = "sandbox_root" if self.mode == "rehearsal" else "system"
        self.report["idempotent"] = idempotent

    def postcheck(self, manifest: dict) -> None:
        assert self.staging is not None and self.state is not None
        # 单元文件必须先落盘：后置 verify.sh 生成安装记录时要把 systemd 单元与 env 文件一并登记，
        # 单元晚于后置校验渲染会让记录生成失败（实测：干净安装上 verify.sh 退出 2 → V2 假失败）。
        # 阶段顺序（V1 → V2 → V3）与报告内容不变，仅把"写单元文件"提前到 V1 之前。
        unit_member, unit_dest = self.render_unit(manifest)
        board_rel = str((manifest.get("board") or {}).get("profile_member"))
        result = run_profile_check(self.staging, board_rel, self.python_exe, self.allow_unverified)
        gate = board_gate_summary(result)
        self.report["post_verify_profile_check"] = gate
        if gate["exit_code"] != 0:
            count = len(gate.get("reasons") or gate.get("failures") or [])
            message = f"后置 profile-check 失败（退出码 {gate['exit_code']}）：{count} 项不合格"
            if self.mode == "rehearsal":
                self.blocker("post_verify_profile_check", message)
                self.stage("V1 后置 profile-check", "REHEARSAL_BLOCKED", message)
            else:
                raise BoardBundleError(message + "；已回滚激活链接，不启动服务", 5)
        else:
            self.stage("V1 后置 profile-check", "PASS", "声明与实现一致（exit=0）")

        post = manifest.get("post_verify") or {}
        script = self.state["version_dir"] / "scripts" / Path(str(post.get("script"))).name
        if not script.is_file():
            message = (
                f"bundle 缺少后置校验脚本 {post.get('script')}（状态 {post.get('status')}）："
                "真实安装必须失败并回滚，不得静默跳过"
            )
            if self.mode == "rehearsal":
                self.blocker("post_verify_script", message)
                self.stage("V2 后置校验 verify.sh", "DEFERRED", message + "（演练登记）")
            else:
                raise BoardBundleError(message, 5)
        else:
            script.chmod(0o755)
            env = dict(os.environ)
            env["IRAF_SDK_ROOT"] = str(self.root)
            env["IRAF_SDK_CONFIG_DIR"] = self.layout["config_dir"]
            # 演练时把 --dry-run 传给后置校验：verify.sh 仍会**如实记录**服务未启动
            # （service_state=SERVICE_NOT_RUNNING、verified=false），只是不据此判定失败——
            # 目标端真实安装同一入口不带该参数，服务未启动即退出 5。
            verify_args = ["bash", str(script), "--root", str(self.root)]
            if self.mode == "rehearsal":
                verify_args.append("--dry-run")
            proc = subprocess.run(
                verify_args,
                capture_output=True,
                text=True,
                timeout=900,
                env=env,
            )
            self.report["post_verify_script"] = {
                "script": post.get("script"),
                "returncode": proc.returncode,
                "stdout_tail": proc.stdout.strip().splitlines()[-3:],
            }
            if proc.returncode != 0:
                message = f"后置校验 {post.get('script')} 失败（退出码 {proc.returncode}）"
                if self.mode == "rehearsal":
                    self.blocker("post_verify_script", message)
                    self.stage("V2 后置校验 verify.sh", "REHEARSAL_BLOCKED", message)
                else:
                    raise BoardBundleError(message + "；已回滚激活链接", 5)
            else:
                self.stage("V2 后置校验 verify.sh", "PASS", "后置校验通过（exit=0）")

        self.report["unit"] = {
            "member": unit_member,
            "installed_as": f"{self.layout['systemd_dir']}/{self.layout['service_unit']}",
            "starts": False,
            "note": "就绪占位单元（oneshot + RemainAfterExit）；不代表板级运行时入口已定义",
        }
        if self.mode == "rehearsal":
            self.stage(
                "V3 systemd 单元",
                "DEFERRED",
                f"单元已渲染到 {self.layout['systemd_dir']}/{self.layout['service_unit']}；"
                "演练不执行 systemctl（不启动服务）",
            )
        else:
            try:
                subprocess.run(
                    ["systemctl", "daemon-reload"], check=True, capture_output=True, timeout=120
                )
                subprocess.run(
                    ["systemctl", "enable", "--now", self.layout["service_unit"]],
                    check=True,
                    capture_output=True,
                    timeout=300,
                )
            except (subprocess.CalledProcessError, subprocess.SubprocessError, OSError) as exc:
                raise BoardBundleError(
                    f"板级服务启动失败（{exc}）；已回滚激活链接", ExitCode.BUILD
                ) from exc
            self.report["unit"]["starts"] = True
            self.stage("V3 systemd 单元", "PASS", "服务已 enable --now（就绪占位单元）")

    # --- 子步骤 ---

    def render_unit(self, manifest: dict) -> tuple[str, Path]:
        """渲染 systemd 单元到目标位置；返回 (成员名, 落盘路径)。

        与 env 文件一样属于"本 bundle 落地的目录外文件"，必须在后置校验（verify.sh 生成
        安装记录）之前就位——顺序缺陷实测见 docs/debug/2026-09-20-verify-uninstall-scope-gates.md。
        """
        assert self.state is not None
        unit_member = str((manifest.get("systemd") or {}).get("unit_member"))
        unit_src = self.state["version_dir"] / unit_member
        unit_dir = self.root / self.layout["systemd_dir"].lstrip("/")
        unit_dir.mkdir(parents=True, exist_ok=True)
        unit_dest = unit_dir / self.layout["service_unit"]
        unit_dest.write_text(
            render_template(
                unit_src.read_text(encoding="utf-8"),
                self.render_values(manifest),
                label="systemd 单元模板",
            ),
            encoding="utf-8",
        )
        return unit_member, unit_dest

    def _extract_runtime(self, runtime_file: Path, dest: Path) -> list[str]:
        """展开 runtime bundle 到 dest，返回顶层目录名（供安装阶段原样搬运）。

        返回值只来自 tar 成员本身（不依赖任何硬编码目录清单），并对每个成员做路径安全检查。
        """
        if not runtime_file.is_file():
            raise BoardBundleError(f"bundle 内 runtime bundle 缺失：{runtime_file}", ExitCode.PRECHECK)
        top_dirs: set[str] = set()
        with tarfile.open(runtime_file, "r:gz") as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    continue
                if not is_relative_member(member.name):
                    raise BoardBundleError(
                        f"runtime bundle 成员路径非法：{member.name}", ExitCode.VERIFY
                    )
                target = dest / member.name
                target.parent.mkdir(parents=True, exist_ok=True)
                handle = tar.extractfile(member)
                target.write_bytes(handle.read() if handle else b"")
                top_dirs.add(member.name.split("/", 1)[0])
        return sorted(top_dirs)

    def _copy(self, source: Path, target: Path) -> None:
        if not source.is_file():
            raise BoardBundleError(f"待安装文件不存在：{source}", ExitCode.PRECHECK)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    def _pip_install_sdk(self, wheel: Path, lib_dir: Path) -> None:
        args = [
            self.python_exe,
            "-m",
            "pip",
            "install",
            "--no-index",
            "--no-deps",
            "--disable-pip-version-check",
            "--no-warn-script-location",
            "--target",
            str(lib_dir),
            str(wheel),
        ]
        try:
            proc = subprocess.run(args, capture_output=True, text=True, timeout=900)
        except (OSError, subprocess.SubprocessError) as exc:
            raise BoardBundleError(f"离线安装 SDK wheel 失败（{exc}）", ExitCode.BUILD) from exc
        self.report["sdk_install"] = {
            "returncode": proc.returncode,
            "command": [
                "<python>",
                "-m",
                "pip",
                "install",
                "--no-index",
                "--no-deps",
                "--target",
                "<lib>",
                wheel.name,
            ],
            "stdout_tail": proc.stdout.strip().splitlines()[-2:],
        }
        if proc.returncode != 0:
            tail = [line for line in proc.stderr.strip().splitlines() if line][-1:]
            raise BoardBundleError(
                f"离线安装 SDK wheel 失败（pip 退出码 {proc.returncode}）：{tail}", ExitCode.BUILD
            )
        installed = lib_dir / "iraf_sdk" / "__init__.py"
        if not installed.is_file():
            raise BoardBundleError(
                f"pip 报告成功但 {installed.name} 未落地：拒绝把未完成的安装当作成功",
                ExitCode.VERIFY,
            )

    def _pip_install_wheelhouse(self, version_dir: Path, packages: list[str]) -> None:
        args = [
            self.python_exe,
            "-m",
            "pip",
            "install",
            "--no-index",
            "--find-links",
            str(version_dir / "wheelhouse"),
            "--disable-pip-version-check",
            *packages,
        ]
        try:
            proc = subprocess.run(args, capture_output=True, text=True, timeout=1800)
        except (OSError, subprocess.SubprocessError) as exc:
            raise BoardBundleError(f"目标端 wheelhouse 安装失败（{exc}）", ExitCode.BUILD) from exc
        if proc.returncode != 0:
            tail = [line for line in proc.stderr.strip().splitlines() if line][-1:]
            raise BoardBundleError(
                f"目标端 wheelhouse 安装失败（pip 退出码 {proc.returncode}）：{tail}", ExitCode.BUILD
            )
        self.stage("I4 目标端二进制 wheelhouse", "PASS", f"{len(packages)} 个依赖已离线安装")

    def compare_installed(self, version_dir: Path, manifest: dict) -> list[str]:
        """已安装目录 vs 清单：逐文件 SHA-256（幂等复用同一份判定）。

        只比对 bundle 直接落地的文件（scripts/、wheelhouse/、systemd/、sdk/）；env 模板渲染后
        必然不同、runtime/ 是一个压缩包成员，这两类不做逐文件比对（已在清单层校验哈希）。
        """
        violations: list[str] = []
        checks = 0
        for item in manifest.get("files") or []:
            rel = str(item.get("path"))
            if rel.startswith(("runtime/", "board/", "env/")):
                continue
            if rel == str((manifest.get("sdk") or {}).get("wheel_member")):
                candidate = version_dir / "sdk" / Path(rel).name
            elif rel.startswith("scripts/"):
                candidate = version_dir / "scripts" / Path(rel).name
            else:
                candidate = version_dir / rel
            if not candidate.is_file():
                violations.append(f"清单登记的文件未安装：{rel}")
                continue
            checks += 1
            if sha256_file(candidate) != item.get("sha256"):
                violations.append(f"已安装文件与 bundle 不一致：{rel}")
        self.report["idempotency_checked_files"] = checks
        return violations

    def render_values(self, manifest: dict) -> dict:
        """模板占位符取值：全部来自 bundle 内清单与 delivery 声明，不猜默认值。"""
        bundle_section = manifest.get("bundle") or {}
        values = {
            "@SDK_NAME@": (manifest.get("sdk") or {}).get("name"),
            "@SDK_VERSION@": bundle_section.get("version"),
            "@BOARD@": bundle_section.get("board"),
            "@TARGET_ID@": bundle_section.get("target"),
            "@PREFIX@": self.layout["prefix"],
            "@CURRENT_LINK@": self.layout["current_link"],
            "@CONFIG_DIR@": self.layout["config_dir"],
            "@LOG_DIR@": self.layout["log_dir"],
            "@SERVICE_UNIT@": self.layout["service_unit"],
            "@ENV_FILE@": self.layout["env_file"],
            "@PYTHON_BIN@": self.layout["python_bin"],
        }
        missing = sorted(name for name, value in values.items() if value in (None, ""))
        if missing:
            raise BoardBundleError(
                "模板取值缺失（清单或 delivery 声明不完整）：" + "、".join(missing),
                ExitCode.PRECHECK,
            )
        return values

    def switch_current(self, prefix: Path, link_name: str, version: str) -> str | None:
        prefix.mkdir(parents=True, exist_ok=True)
        link = prefix / link_name
        previous = None
        if link.is_symlink():
            previous = os.readlink(link)
        elif link.exists():
            raise BoardBundleError(
                f"激活入口 {link} 已存在且不是符号链接：拒绝覆盖未知文件", ExitCode.PRECHECK
            )
        tmp = prefix / f".{link_name}.tmp-{os.getpid()}"
        if tmp.is_symlink() or tmp.exists():
            tmp.unlink()
        os.symlink(version, tmp)
        os.replace(tmp, link)
        return previous

    def rollback(self) -> None:
        """回滚：恢复旧激活链接；删除本次新建的版本目录（幂等目录不删）。"""
        if self.state is None:
            self.stage("回滚", "SKIPPED", "尚未创建任何版本目录，无需回滚")
            self.report["rolled_back"] = False
            return
        prefix: Path = self.state["prefix_path"]
        link = prefix / self.state["layout"]["current_link"]
        previous = self.state.get("previous")
        if link.is_symlink() or link.exists():
            try:
                link.unlink()
            except FileNotFoundError:
                pass
        if previous:
            os.symlink(previous, link)
        removed = False
        if self.report.get("version_dir_created"):
            shutil.rmtree(self.state["version_dir"], ignore_errors=True)
            removed = True
        self.report["rolled_back"] = True
        self.stage(
            "回滚",
            "DONE",
            f"激活链接恢复为 {previous or '（无）'}；"
            + ("新版本目录已删除" if removed else "未新建版本目录（幂等目录保留）"),
        )

    def cleanup_staging(self) -> None:
        if self.staging is not None and self.staging.exists():
            shutil.rmtree(self.staging, ignore_errors=True)
            self.report["staging_removed"] = True

    def write_report(self) -> Path:
        if self.layout.get("log_dir"):
            directory = self.root / str(self.layout["log_dir"]).lstrip("/")
        else:
            directory = self.root
        directory.mkdir(parents=True, exist_ok=True)
        version = self.report.get("bundle", {}).get("version") or "unknown"
        path = directory / f"{INSTALL_REPORT_FALLBACK_NAME}-{version}.json"
        path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.report["report_file"] = str(path)
        return path


def bundle_section_target(manifest: dict) -> str:
    section = manifest.get("bundle") or {}
    return f"{section.get('target')}/{section.get('platform_tag')}"


def run_install(options: dict) -> int:
    installer = Installer(options)
    code = ExitCode.OK
    try:
        installer.verify_bundle_file()
        manifest = installer.extract_to_staging()
        installer.precheck(manifest)
        installer.install_tree(manifest)
        installer.postcheck(manifest)
        installer.report["stages"].append(
            {
                "stage": "收尾",
                "status": "PASS",
                "detail": (
                    "演练流水线完成（演练 ≠ 已安装；目标端安装验收 DEFERRED：板卡不在场）"
                    if installer.mode == "rehearsal"
                    else "流水线完成：服务已注册并启动"
                ),
            }
        )
    except BoardBundleError as exc:
        installer.report["failure"] = {"message": str(exc), "details": exc.details}
        code = exc.exit_code
        if installer.state is not None and code in (
            ExitCode.BUILD,
            ExitCode.VERIFY,
            ExitCode.PRECHECK,
            5,
        ):
            installer.rollback()
        print(f"[board_bundle][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[board_bundle][错误]   - {detail}", file=sys.stderr)
    except ManifestError as exc:
        installer.report["failure"] = {"message": str(exc), "details": exc.details}
        code = exc.exit_code
        print(f"[board_bundle][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[board_bundle][错误]   - {detail}", file=sys.stderr)
    finally:
        installer.cleanup_staging()
    installer.report["exit_code"] = code
    installer.report["real_install_allowed"] = bool(
        installer.mode == "apply" and code == ExitCode.OK and not installer.report["blockers"]
    )
    installer.report["summary"] = {
        "stages": len(installer.report["stages"]),
        "blockers": len(installer.report["blockers"]),
        "warnings": len(installer.report["warnings"]),
        "install_performed": installer.report["install_performed"],
    }
    installer.write_report()
    if options.get("json_out"):
        extra = Path(str(options["json_out"]))
        if not extra.is_absolute():
            extra = Path.cwd() / extra
        extra.parent.mkdir(parents=True, exist_ok=True)
        extra.write_text(
            json.dumps(installer.report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        info(f"安装报告副本：{extra}（含本机路径，只允许落在证据区，不入库）")
    print(json.dumps(installer.report, ensure_ascii=False, indent=2))
    info(f"安装报告：{installer.report['report_file']}（本机路径，不入库）")
    return code


# --- 子命令 --------------------------------------------------------------------


def cmd_plan(options: dict) -> int:
    repo_root = Path(options["repo_root"]).resolve()
    matrix_path = _abspath(repo_root, options.get("matrix") or MATRIX_REL)
    matrix = load_matrix(repo_root, matrix_path)
    target = select_target(matrix, options.get("target"))
    delivery = load_delivery(matrix)
    version = read_declared_version(repo_root, matrix["sdk"]["version_ref"])
    board_rel = options.get("board") or "<--board 未给出>"
    name = bundle_name(delivery, board="<board>", version=version, arch=target["arch"])
    info("板级 bundle 组装计划（--dry-run：不写任何文件）")
    info(f"  构建目标  : {target['id']}（arch={target['arch']} python_tag={target['python_tag']}）")
    info(f"  板卡声明  : {board_rel}（未实测时组装默认拒绝，需 --allow-unverified）")
    info(f"  版本      : {version}（← {matrix['sdk']['version_ref']}）")
    info(
        "  产物      : "
        f"{display_path(_abspath(repo_root, options.get('out') or DEFAULT_OUT), repo_root)}/{name}"
    )
    info(
        f"  安装布局  : prefix={delivery['install']['prefix']} "
        f"service={delivery['install']['service_unit']} "
        f"env={delivery['install']['config_dir']}/{delivery['install']['env_file']}"
    )
    info(f"  wheelhouse: 默认 {DEFAULT_WHEELHOUSE_ROOT}/{target['id']}（缺失即失败）")
    info("  后置校验  : deploy/sdk/verify.sh（步骤 09 交付；缺失时真实安装必须失败并回滚）")
    return ExitCode.OK


def cmd_build(options: dict) -> int:
    _, _, summary = build_board_bundle(options)
    info(f"bundle 产出：{summary['bundle']}（{summary['size_bytes']} 字节，成员 {summary['member_count']}）")
    info(f"  sha256={summary['sha256']}")
    info(f"  校验和文件：{summary['checksum_file']}")
    info(
        f"  wheelhouse wheel {summary['wheelhouse_wheel_count']} 个；"
        f"后置校验脚本状态 {summary['post_verify_status']}"
    )
    if summary["scripts_missing"]:
        info(f"  缺失脚本（后续步骤交付或真实安装失败）：{'、'.join(summary['scripts_missing'])}")
    if not summary["verified"]:
        warn("board.verified=false：本 bundle 在板卡未实测状态下组装（--allow-unverified 已记录）")
    if options.get("json_out"):
        target = Path(str(options["json_out"]))
        if not target.is_absolute():
            target = Path.cwd() / target
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        info(f"组装摘要已落盘：{display_path(target, Path.cwd())}")
    return ExitCode.OK


def cmd_inspect(options: dict) -> int:
    if not options.get("bundle"):
        raise BoardBundleError("缺少 --bundle", ExitCode.USAGE)
    path = Path(str(options["bundle"]))
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.is_file():
        raise BoardBundleError(f"bundle 不存在：{path}", ExitCode.PRECHECK)
    checksum = Path(str(path) + ".sha256")
    if not checksum.is_file():
        raise BoardBundleError(f"缺少伴随校验和文件：{checksum.name}", ExitCode.PRECHECK)
    expected = checksum.read_text(encoding="utf-8").strip().split()[0]
    digest = sha256_file(path)
    if expected != digest:
        raise BoardBundleError(
            f"bundle SHA-256 与伴随校验和不符：{expected[:16]}… / {digest[:16]}…", ExitCode.VERIFY
        )
    manifest, entries, violations = verify_bundle_contract(path)
    if violations:
        raise BoardBundleError(
            f"bundle 契约校验失败：{len(violations)} 处", ExitCode.VERIFY, violations
        )
    info(f"bundle 契约校验通过：{path.name}")
    info(
        f"  schema={manifest['schema_version']} 板卡={manifest['bundle']['board']} "
        f"版本={manifest['bundle']['version']} 成员={len(entries)}"
    )
    info(f"  sha256={digest}")
    info(
        f"  后置校验脚本状态：{manifest['post_verify']['status']}"
        f"（required_in_real_install={manifest['post_verify']['required_in_real_install']}）"
    )
    return ExitCode.OK


COMMANDS = {
    "plan": cmd_plan,
    "build": cmd_build,
    "inspect": cmd_inspect,
    "install": run_install,
}


def parse_options(argv: list[str]) -> dict:
    options: dict = {"repo_root": str(Path(__file__).resolve().parents[2])}
    value_flags = {
        "--repo-root": "repo_root",
        "--matrix": "matrix",
        "--out": "out",
        "--target": "target",
        "--board": "board",
        "--wheelhouse": "wheelhouse",
        "--runtime": "runtime",
        "--sdk-wheel": "sdk_wheel",
        "--python": "python",
        "--json-out": "json_out",
        "--bundle": "bundle",
        "--root": "root",
    }
    bool_flags = {"--allow-unverified": "allow_unverified", "--dry-run": "dry_run"}
    index = 0
    while index < len(argv):
        item = argv[index]
        if item in ("-h", "--help"):
            options["help"] = True
        elif item in value_flags:
            if index + 1 >= len(argv):
                raise BoardBundleError(f"{item} 缺少取值", ExitCode.USAGE)
            options[value_flags[item]] = argv[index + 1]
            index += 1
        elif item in bool_flags:
            options[bool_flags[item]] = True
        else:
            raise BoardBundleError(f"无法识别的参数：{item}（用 --help 查看用法）", ExitCode.USAGE)
        index += 1
    return options


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return ExitCode.OK if argv else ExitCode.USAGE
    command, rest = argv[0], argv[1:]
    if command not in COMMANDS:
        print(f"[board_bundle] 无法识别的子命令：{command}\n", file=sys.stderr)
        return ExitCode.USAGE
    try:
        options = parse_options(rest)
        if options.get("help"):
            print(USAGE)
            return ExitCode.OK
        return COMMANDS[command](options)
    except (BoardBundleError, ManifestError) as exc:
        print(f"[board_bundle][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[board_bundle][错误]   - {detail}", file=sys.stderr)
        return exc.exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
