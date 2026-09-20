#!/usr/bin/env python3
"""SDK 打包产物库：wheel / runtime bundle / manifest 的构建与校验（步骤 06）。

本文件是 `deploy/sdk/build_sdk.sh` 的实现层，也可以直接当 CLI 用（子命令见 `USAGE`）：

    /usr/bin/python3 deploy/sdk/lib_manifest.py plan      --out build/sdk
    /usr/bin/python3 deploy/sdk/lib_manifest.py preflight --matrix config/sdk/package_matrix.yaml
    /usr/bin/python3 deploy/sdk/lib_manifest.py gen-stubs
    /usr/bin/python3 deploy/sdk/lib_manifest.py wheel  --out build/sdk
    /usr/bin/python3 deploy/sdk/lib_manifest.py bundle --out build/sdk
    /usr/bin/python3 deploy/sdk/lib_manifest.py manifest --out build/sdk
    /usr/bin/python3 deploy/sdk/lib_manifest.py verify --manifest build/sdk/manifest.json

分层约定（铁律 5.3、设计 §4/§5/§7）：

- **声明只读**：版本、Python/平台标签、包清单、镜像源、板卡映射全部来自
  `config/sdk/package_matrix.yaml`；本文件不复制任何版本号或标签字面量。
- **声明层即门禁**：矩阵用 `config/sdk/package_matrix.schema.json` 的**根结构**校验
  （`Draft7Validator`，与 `tests/unit/test_board_profile_schema.py` 同一份契约），
  契约不合法即退出码 2，绝不"尽力解析"。
- **产出物只含仓库相对路径**：manifest 内不得出现本机绝对路径
  （`/home/...` 之类）；解释器信息只记录实现与版本，不记录可执行文件路径。
- **可复算**：wheel 用固定 `ZipInfo.date_time`，bundle 用 `gzip mtime=0` + 成员
  `mtime=0`/`uid=gid=0` + 路径排序，因此同一棵源码树连续两次打包的 SHA-256 必须相同
  （`--verify` 之外的"可复算"证据见步骤 06 的验收记录）。

退出码（与设计 §5 全脚本统一约定一致）：

    0  成功
    1  参数/用法错误
    2  预检失败（声明缺失/不合法、版本无法确定、git 状态不可读、manifest 不存在）
    3  构建失败（产物缺失、wheel 校验不通过、目录不可写）
    4  校验失败（SHA-256 不符、产物被篡改、IDL 破坏性变更未声明兼容版本）

未实测的东西一律写 `unverified`（本文件里体现为 `target.verified=false`、
`wheelhouse.status=pending_fetch`、`signature.scheme=pending`），不得用默认值冒充。
"""

from __future__ import annotations

import base64
import gzip
import hashlib
import io
import json
import re
import shutil
import subprocess
import sys
import tarfile
import zipfile
from pathlib import Path

# --- 契约常量：只在这里声明一次 ------------------------------------------------

SCHEMA_VERSION = "iraf.package-manifest/v1"
MATRIX_SCHEMA_PATH = Path("config/sdk/package_matrix.schema.json")
DEFAULT_MATRIX = Path("config/sdk/package_matrix.yaml")
DEFAULT_OUT = Path("build/sdk")
DEFAULT_PROTO_ROOT = Path("api/proto")
#: IDL 消息层 stub 的源文件（相对 `api/proto`）；新增 proto 必须同步这里与矩阵。
IDL_PROTO_FILES = (
    Path("iraf/v1/common.proto"),
    Path("iraf/v1/skill.proto"),
    Path("iraf/v1/runtime.proto"),
    Path("iraf/v1/events.proto"),
)
#: 生成 stub 的既有目录（`src/iraf_adapters/**` 与 `src/iraf_sdk/client.py` 依赖的
#: `iraf.v1.*` 从这里 import），也是验收口径里的"既有流程"。
CANONICAL_STUB_DIR = Path("build/generated/python")
#: runtime bundle 的成员目录（步骤 06「步骤 4」声明，顺序即 tar 顺序来源）。
BUNDLE_DIRS = ("src", "skills", "profiles", "config", "deploy")
#: 打包时排除的目录名/后缀：编辑器备份与字节码不是运行时内容。
#: 排除不是"静默过滤"——被排掉的路径会写进 manifest 的 `bundle.excluded`。
EXCLUDE_DIR_NAMES = frozenset({"__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache"})
EXCLUDE_FILE_SUFFIXES = (".pyc", ".pyo", ".orig", ".bak", "~")
#: wheel 内固定时间戳（PEP 427 允许，可复算必需）。
ZIP_EPOCH = (1980, 1, 1, 0, 0, 0)
WHEEL_TAG = "py3-none-any"


class ExitCode:
    """与设计 §5 全脚本统一约定的退出码（勿在别处复制这些字面量）。"""

    OK = 0
    USAGE = 1
    PRECHECK = 2
    BUILD = 3
    VERIFY = 4


class ManifestError(RuntimeError):
    """带退出码的中文可操作错误。"""

    def __init__(self, message: str, exit_code: int = ExitCode.PRECHECK, details=None):
        super().__init__(message)
        self.exit_code = int(exit_code)
        self.details = [str(item) for item in (details or [])]


def info(message: str) -> None:
    print(f"[build_sdk] {message}", flush=True)


def warn(message: str) -> None:
    print(f"[build_sdk][警告] {message}", flush=True)


# --- 基础工具 ------------------------------------------------------------------


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _record_digest(digest_hex: str) -> str:
    """wheel RECORD 使用 urlsafe base64 且去掉 `=` 填充。"""
    return base64.urlsafe_b64encode(bytes.fromhex(digest_hex)).rstrip(b"=").decode("ascii")


def normalize_dist_name(name: str) -> str:
    """PEP 427 文件名转义：非字母数字的点分串统一成下划线。"""
    return re.sub(r"[^A-Za-z0-9.]+", "_", name)


def relpath_posix(path: Path, repo_root: Path) -> str:
    return path.resolve().relative_to(repo_root.resolve()).as_posix()


def display_path(path: Path, repo_root: Path) -> str:
    """日志用路径：优先仓库相对路径；--out 指到仓库外时退回绝对路径。

    只用于终端输出（可给人看）；写进产物的路径必须用 `relpath_posix` 并且保持相对。
    """
    try:
        return relpath_posix(path, repo_root)
    except ValueError:
        return str(path)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ManifestError(f"无法读取 {path}：{exc}", ExitCode.PRECHECK) from exc


# --- 声明层：矩阵 / 版本单一来源 / git 状态 ------------------------------------


def load_matrix(repo_root: Path, matrix_path: Path) -> dict:
    """读取并**按契约**校验产物矩阵；任何契约问题都是退出码 2。"""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺 PyYAML 时的显式失败
        raise ManifestError(
            f"当前解释器缺少 PyYAML（{exc}）：打包入口需要解析声明文件；"
            "请用 PYTHON=/usr/bin/python3 重跑，或先安装 PyYAML",
            ExitCode.PRECHECK,
        ) from exc
    try:
        from jsonschema import Draft7Validator
    except ImportError as exc:  # pragma: no cover - 环境缺 jsonschema 时的显式失败
        raise ManifestError(
            f"当前解释器缺少 jsonschema（{exc}）：矩阵契约校验不可跳过（声明层即门禁）",
            ExitCode.PRECHECK,
        ) from exc

    if not matrix_path.is_file():
        raise ManifestError(f"声明文件不存在：{matrix_path}", ExitCode.PRECHECK)
    schema_path = repo_root / MATRIX_SCHEMA_PATH
    if not schema_path.is_file():
        raise ManifestError(f"矩阵契约不存在：{MATRIX_SCHEMA_PATH}", ExitCode.PRECHECK)

    schema = json.loads(_read_text(schema_path))
    try:
        data = yaml.safe_load(_read_text(matrix_path))
    except yaml.YAMLError as exc:
        raise ManifestError(f"矩阵不是合法 YAML：{exc}", ExitCode.PRECHECK) from exc
    if not isinstance(data, dict):
        raise ManifestError("矩阵根结构必须是映射（object）", ExitCode.PRECHECK)

    errors = sorted(Draft7Validator(schema).iter_errors(data), key=lambda e: list(e.path))
    if errors:
        details = [
            f"{'/'.join(str(p) for p in err.path) or '<root>'}: {err.message}" for err in errors
        ]
        raise ManifestError(
            f"矩阵不符合 {MATRIX_SCHEMA_PATH}（{len(errors)} 处）", ExitCode.PRECHECK, details
        )
    return data


def select_target(matrix: dict, target_id: str | None) -> dict:
    """选择构建目标；多目标时必须显式声明，不做隐式默认。"""
    targets = matrix.get("targets") or []
    if target_id:
        for target in targets:
            if target.get("id") == target_id:
                return target
        raise ManifestError(
            f"矩阵中没有声明构建目标 {target_id}；已声明：{[t.get('id') for t in targets]}",
            ExitCode.PRECHECK,
        )
    if len(targets) == 1:
        return targets[0]
    raise ManifestError(
        f"矩阵声明了 {len(targets)} 个构建目标，必须用 --target 显式指定："
        f"{[t.get('id') for t in targets]}",
        ExitCode.PRECHECK,
    )


def read_declared_version(repo_root: Path, version_ref: str) -> str:
    """从 `version_ref`（形如 `pyproject.toml#project.version`）读版本。

    不用 `tomllib`：本机 /usr/bin/python3 是 3.10，`tomllib` 3.11 才有；打包入口不允许
    为解析两行 TOML 引入第三方依赖。这里只按段/键精确取值，取不到即显式失败。
    """
    if "#" not in version_ref:
        raise ManifestError(
            f"version_ref 格式非法：{version_ref!r}（期望 <相对路径>#<段>.<键>）",
            ExitCode.PRECHECK,
        )
    rel, _, pointer = version_ref.partition("#")
    section, _, key = pointer.rpartition(".")
    if not section or not key:
        raise ManifestError(
            f"version_ref 的定位部分非法：{pointer!r}（期望 <段>.<键>）", ExitCode.PRECHECK
        )
    path = repo_root / rel
    if not path.is_file():
        raise ManifestError(f"版本来源文件不存在：{rel}", ExitCode.PRECHECK)

    current_section = None
    for raw_line in _read_text(path).splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            current_section = line[1:-1].strip()
            continue
        if current_section != section:
            continue
        match = re.match(rf'^{re.escape(key)}\s*=\s*"([^"]+)"\s*$', line)
        if match:
            return match.group(1)
    raise ManifestError(
        f"未在 {rel} 的 [{section}] 段找到 {key} = \"...\"（版本唯一来源缺失，拒绝猜测）",
        ExitCode.PRECHECK,
    )


def git_state(repo_root: Path) -> dict:
    """产物必须可追溯到提交：读不到 git 状态即失败，不写 `unknown`。"""

    def _git(*args: str) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                ["git", *args], cwd=str(repo_root), capture_output=True, text=True, timeout=30
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise ManifestError(f"无法执行 git {' '.join(args)}：{exc}", ExitCode.PRECHECK) from exc

    head = _git("rev-parse", "HEAD")
    if head.returncode != 0:
        raise ManifestError(
            f"无法读取 git HEAD（{head.stderr.strip() or head.stdout.strip()}）："
            "打包产物必须可追溯到提交（fail-closed）",
            ExitCode.PRECHECK,
        )
    status = _git("status", "--porcelain")
    if status.returncode != 0:
        raise ManifestError(
            f"无法读取 git 工作区状态：{status.stderr.strip()}", ExitCode.PRECHECK
        )
    dirty_lines = [line for line in status.stdout.splitlines() if line.strip()]
    # 只记录计数，不记录另一窗口的 WIP 路径（证据最小化）。
    return {"commit": head.stdout.strip(), "dirty": bool(dirty_lines),
            "dirty_file_count": len(dirty_lines)}


# --- 产物：wheel ---------------------------------------------------------------


def _iter_package_files(source_dir: Path) -> list[Path]:
    if not source_dir.is_dir():
        raise ManifestError(f"SDK 包目录不存在：{source_dir}", ExitCode.BUILD)
    files = []
    for path in sorted(source_dir.rglob("*")):
        if not path.is_file():
            continue
        if set(path.parts) & EXCLUDE_DIR_NAMES:
            continue
        if path.suffix in EXCLUDE_FILE_SUFFIXES or path.name.endswith("~"):
            continue
        files.append(path)
    if not files:
        raise ManifestError(f"SDK 包目录内没有可打包文件：{source_dir}", ExitCode.BUILD)
    return files


def _zip_writestr(handle: zipfile.ZipFile, arcname: str, data: bytes) -> None:
    entry = zipfile.ZipInfo(arcname, date_time=ZIP_EPOCH)
    entry.compress_type = zipfile.ZIP_DEFLATED
    entry.create_system = 3  # Unix
    entry.external_attr = 0o100644 << 16
    handle.writestr(entry, data)


def build_wheel(
    repo_root: Path,
    out_dir: Path,
    *,
    dist_name: str,
    version: str,
    package: str,
    summary: str,
    requires_python: str,
    license_text: str = "Proprietary",
) -> dict:
    """手工组装纯 Python wheel（PEP 427）。

    为什么不用 `python3 -m build` / `setup.py bdist_wheel`：实测本机 `pypa/build` 未安装，
    而 `pip wheel . --no-build-isolation` 在 setuptools 59.6.0（<61，不认 PEP 621 的
    `[project]` 表）下**退出码为 0 但产出 `UNKNOWN-0.0.0-py3-none-any.whl`（961 字节、空包）**，
    即"绿灯 + 假产物"。因此这里用标准库直接组 `zip + dist-info + RECORD`，并在
    `validate_wheel` 里把上面那个假产物变成硬失败（文件名/版本/内容三重校验）。
    细节与实测输出见 docs/debug/2026-09-20-sdk-wheel-build-offline.md。
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    normalized = normalize_dist_name(dist_name)
    source_dir = repo_root / "src" / package
    dist_info = f"{normalized}-{version}.dist-info"
    metadata = "\n".join(
        [
            "Metadata-Version: 2.1",
            f"Name: {dist_name}",
            f"Version: {version}",
            f"Summary: {summary}",
            f"License: {license_text}",
            f"Requires-Python: {requires_python}",
            "",
        ]
    ).encode("utf-8")
    wheel_meta = "\n".join(
        [
            "Wheel-Version: 1.0",
            "Generator: iraf-build-sdk (hand-assembled PEP 427; "
            "docs/debug/2026-09-20-sdk-wheel-build-offline.md)",
            "Root-Is-Purelib: true",
            f"Tag: {WHEEL_TAG}",
            "",
        ]
    ).encode("utf-8")

    payload: list[tuple[str, bytes]] = []
    for path in _iter_package_files(source_dir):
        payload.append((f"{package}/{path.relative_to(source_dir).as_posix()}", path.read_bytes()))
    payload.append((f"{dist_info}/METADATA", metadata))
    payload.append((f"{dist_info}/WHEEL", wheel_meta))

    record_lines = []
    for arcname, data in payload:
        record_lines.append(f"{arcname},sha256={_record_digest(sha256_bytes(data))},{len(data)}")
    record_lines.append(f"{dist_info}/RECORD,,")
    payload.append((f"{dist_info}/RECORD", ("\n".join(record_lines) + "\n").encode("utf-8")))

    wheel_path = out_dir / f"{normalized}-{version}-{WHEEL_TAG}.whl"
    with zipfile.ZipFile(wheel_path, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for arcname, data in sorted(payload):
            _zip_writestr(handle, arcname, data)

    files = [
        {"path": arcname, "sha256": sha256_bytes(data), "size_bytes": len(data)}
        for arcname, data in sorted(payload)
    ]
    return {
        "path": wheel_path,
        "dist_info": dist_info,
        "files": files,
        "required_members": [f"{package}/__init__.py"],
    }


def validate_wheel(
    wheel_path: Path,
    *,
    dist_name: str,
    version: str,
    package: str,
    required_members=(),
) -> list[str]:
    """wheel 的三重校验：文件名/版本/标签、结构性、内容性（PEP 427 + 零依赖契约）。"""
    normalized = normalize_dist_name(dist_name)
    expected_name = f"{normalized}-{version}-{WHEEL_TAG}.whl"
    dist_info = f"{normalized}-{version}.dist-info"
    violations: list[str] = []

    if wheel_path.name != expected_name:
        violations.append(
            f"wheel 文件名与声明不符：实际 {wheel_path.name}，期望 {expected_name}"
            "（声明来源：package_matrix.yaml 的 sdk.name + pyproject.toml 的 project.version）"
        )
    if not wheel_path.is_file():
        return [f"wheel 不存在：{wheel_path}"]

    try:
        with zipfile.ZipFile(wheel_path) as handle:
            names = handle.namelist()
            for member in list(required_members) or [f"{package}/__init__.py"]:
                if member not in names:
                    violations.append(f"wheel 缺少必需成员：{member}")
            for meta in ("METADATA", "WHEEL", "RECORD"):
                if f"{dist_info}/{meta}" not in names:
                    violations.append(f"wheel 缺少 {dist_info}/{meta}")
            binaries = [n for n in names if n.endswith((".so", ".pyd", ".dll", ".dylib"))]
            if binaries:
                violations.append(
                    f"纯 Python wheel 含二进制扩展：{binaries}（SDK 必须与目标架构无关）"
                )
            if f"{dist_info}/METADATA" in names:
                text = handle.read(f"{dist_info}/METADATA").decode("utf-8")
                if not re.search(rf"^Name: {re.escape(dist_name)}$", text, re.MULTILINE):
                    violations.append(f"METADATA 的 Name 不是 {dist_name}")
                if not re.search(rf"^Version: {re.escape(version)}$", text, re.MULTILINE):
                    violations.append(f"METADATA 的 Version 不是 {version}")
                requires = [l for l in text.splitlines() if l.startswith("Requires-Dist:")]
                if requires:
                    violations.append(
                        f"SDK wheel 声明了第三方依赖：{requires}"
                        "（契约：SDK 只依赖标准库，目标端才可独立安装）"
                    )
            if f"{dist_info}/RECORD" in names:
                recorded = set()
                for line in handle.read(f"{dist_info}/RECORD").decode("utf-8").splitlines():
                    if line.strip():
                        recorded.add(line.split(",", 1)[0])
                missing = [
                    n for n in names if n != f"{dist_info}/RECORD" and n not in recorded
                ]
                if missing:
                    violations.append(f"RECORD 未覆盖全部成员：{missing}")
                for arcname in names:
                    if arcname in recorded:
                        continue
    except zipfile.BadZipFile as exc:
        violations.append(f"wheel 不是合法 zip：{exc}")
    return violations


# --- 产物：runtime bundle ------------------------------------------------------


def iter_bundle_files(repo_root: Path, dirs=BUNDLE_DIRS) -> tuple[list[str], list[dict]]:
    """列出 bundle 成员（仓库相对 POSIX 路径）与被排除项（排除必须可审计）。"""
    members: list[str] = []
    excluded: list[dict] = []
    for name in dirs:
        base = repo_root / name
        if not base.is_dir():
            raise ManifestError(
                f"声明的打包目录不存在：{name}（声明与仓库不一致，拒绝用空目录顶替）",
                ExitCode.PRECHECK,
            )
        for path in sorted(base.rglob("*")):
            if not path.is_file():
                continue
            rel = path.relative_to(repo_root).as_posix()
            if set(path.parts) & EXCLUDE_DIR_NAMES:
                excluded.append({"path": rel, "reason": "字节码/缓存目录"})
                continue
            if path.suffix in EXCLUDE_FILE_SUFFIXES or path.name.endswith("~"):
                excluded.append({"path": rel, "reason": "编辑器备份/字节码文件"})
                continue
            members.append(rel)
    members.sort()
    return members, excluded


def build_runtime_bundle(
    repo_root: Path,
    out_dir: Path,
    *,
    name: str,
    version: str,
    arch: str,
    dirs=BUNDLE_DIRS,
) -> dict:
    """确定性 tar.gz：gzip mtime=0、成员排序、uid/gid=0、mtime=0。"""
    members, excluded = iter_bundle_files(repo_root, dirs)
    if not members:
        raise ManifestError("bundle 成员为空，拒绝产出空包", ExitCode.BUILD)
    out_dir.mkdir(parents=True, exist_ok=True)
    bundle_path = out_dir / f"{name}-{version}-{arch}.tar.gz"

    buffer = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buffer, mtime=0) as gz:
        with tarfile.open(fileobj=gz, mode="w", format=tarfile.GNU_FORMAT) as tar:
            for rel in members:
                source = repo_root / rel
                entry = tar.gettarinfo(str(source), arcname=rel)
                entry.mtime = 0
                entry.uid = 0
                entry.gid = 0
                entry.uname = ""
                entry.gname = ""
                with open(source, "rb") as handle:
                    tar.addfile(entry, handle)
    bundle_path.write_bytes(buffer.getvalue())

    files = [
        {"path": rel, "sha256": sha256_file(repo_root / rel), "size_bytes": (repo_root / rel).stat().st_size}
        for rel in members
    ]
    return {"path": bundle_path, "files": files, "excluded": excluded}


# --- IDL 摘要（破坏性变更门禁） ------------------------------------------------


def idl_digest(repo_root: Path, proto_root: Path = DEFAULT_PROTO_ROOT) -> str:
    digest = hashlib.sha256()
    for rel in IDL_PROTO_FILES:
        path = repo_root / proto_root / rel
        if not path.is_file():
            raise ManifestError(f"IDL 声明缺失：{proto_root / rel}", ExitCode.PRECHECK)
        digest.update(rel.as_posix().encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


# --- proto stub（生成流程 + 既有产物状态） --------------------------------------


def protoc_version(protoc: str) -> str:
    try:
        proc = subprocess.run([protoc, "--version"], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as exc:
        raise ManifestError(f"无法执行 {protoc} --version：{exc}", ExitCode.BUILD) from exc
    if proc.returncode != 0:
        raise ManifestError(
            f"{protoc} --version 失败（退出码 {proc.returncode}）：{proc.stderr.strip()}",
            ExitCode.BUILD,
        )
    return proc.stdout.strip()


def generate_pystubs(
    protoc: str, repo_root: Path, out_dir: Path, proto_root: Path = DEFAULT_PROTO_ROOT
) -> dict:
    """按既有流程生成消息层 stub（`--python_out`），不改 protoc 版本。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    args = [
        protoc,
        f"-I{proto_root}",
        f"--python_out={relpath_posix(out_dir, repo_root)}",
        *[str(proto_root / rel) for rel in IDL_PROTO_FILES],
    ]
    proc = subprocess.run(args, cwd=str(repo_root), capture_output=True, text=True, timeout=120)
    generated = sorted(
        path.relative_to(out_dir).as_posix()
        for path in out_dir.rglob("*_pb2.py")
        if path.is_file()
    )
    return {
        "command": args,
        "returncode": proc.returncode,
        "stderr_tail": (proc.stderr.strip().splitlines() or [""])[-1],
        "files": generated,
    }


def stub_import_probe(stub_dir: Path, module: str = "iraf.v1.skill_pb2") -> dict:
    """在**子进程**里试导入生成的 stub（cwd=/tmp 避免仓库目录造成同名遮蔽）。

    只报告事实（能否导入 + 首行原因），不把"跳过"说成"通过"。
    """
    code = (
        "import sys, importlib, json\n"
        f"sys.path.insert(0, {str(stub_dir)!r})\n"
        "try:\n"
        f"    importlib.import_module({module!r})\n"
        "except Exception as exc:\n"
        "    print(json.dumps({'ok': False, 'reason': type(exc).__name__ + ': ' + str(exc)[:200]}))\n"
        "else:\n"
        "    print(json.dumps({'ok': True, 'reason': ''}))\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=60, cwd="/tmp"
    )
    if proc.returncode != 0:
        return {"ok": False, "reason": (proc.stderr.strip().splitlines() or ["<无输出>"])[-1]}
    try:
        payload = json.loads(proc.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {"ok": False, "reason": f"探针输出无法解析：{proc.stdout.strip()[:200]}"}
    return {"ok": bool(payload.get("ok")), "reason": str(payload.get("reason") or "")}


def proto_services(repo_root: Path, proto_root: Path, rel: Path) -> list[str]:
    """从 proto 源码**实测**声明了哪些 service。

    protoc 只为声明了 `service` 的 proto 生成 `*_pb2_grpc.py`（本仓库 common/skill 无 service），
    所以"应该有哪些 grpc stub"必须从 IDL 读出来，不能假设四个 proto 都有。
    """
    path = repo_root / proto_root / rel
    if not path.is_file():
        raise ManifestError(f"IDL 声明缺失：{proto_root / rel}", ExitCode.PRECHECK)
    return re.findall(r"^\s*service\s+([A-Za-z_][A-Za-z0-9_]*)", _read_text(path), re.MULTILINE)


def stub_status(repo_root: Path, stub_dir: Path = CANONICAL_STUB_DIR) -> dict:
    """既有 stub 目录的状态：消息层 + gRPC 层（后者本机无法生成，见 grpc_plugin）。"""
    base = repo_root / stub_dir
    messages = {}
    grpc = {}
    for rel in IDL_PROTO_FILES:
        message_rel = (stub_dir / rel.parent / (rel.stem + "_pb2.py")).as_posix()
        messages[rel.stem] = {
            "path": message_rel,
            "present": (base / rel.parent / (rel.stem + "_pb2.py")).is_file(),
        }
        services = proto_services(repo_root, DEFAULT_PROTO_ROOT, rel)
        grpc_rel = (stub_dir / rel.parent / (rel.stem + "_pb2_grpc.py")).as_posix()
        grpc_target = base / rel.parent / (rel.stem + "_pb2_grpc.py")
        grpc[rel.stem] = {
            "path": grpc_rel,
            "services": services,
            "expected": bool(services),
            "present": grpc_target.is_file(),
            "sha256": sha256_file(grpc_target) if grpc_target.is_file() else None,
        }
    expected_grpc = sorted(name for name, item in grpc.items() if item["expected"])
    plugin = shutil.which("grpc_python_plugin")
    return {
        "dir": stub_dir.as_posix(),
        "messages": messages,
        "messages_present": all(item["present"] for item in messages.values()),
        "grpc": grpc,
        "grpc_expected": expected_grpc,
        "grpc_present": all(grpc[name]["present"] for name in expected_grpc),
        "grpc_plugin": plugin or "missing",
        "grpc_regenerable_locally": bool(plugin),
    }


def declared_protobuf_spec(repo_root: Path) -> str:
    """从 pyproject 的依赖里取 protobuf 区间（用于解释"为何不用本机 protoc 覆盖既有 stub"）。"""
    text = _read_text(repo_root / "pyproject.toml")
    match = re.search(r'"(protobuf[^"]*)"', text)
    return match.group(1) if match else "unverified"


# --- manifest ------------------------------------------------------------------


def sbom_placeholder(matrix: dict, target: dict, version: str) -> dict:
    """SBOM 占位（spdx 2.3 骨架）：依赖明细由步骤 07 的 wheelhouse 抓取后回填。"""
    packages = []
    for name in ["jsonschema", *target.get("pure_python", []), *target.get("wheels", [])]:
        packages.append(
            {
                "SPDXID": f"SPDXRef-Package-{name}",
                "name": name,
                "versionInfo": "unverified",
                "downloadLocation": "NOASSERTION",
                "licenseConcluded": "NOASSERTION",
                "comment": "占位：版本与许可证由 deploy/sdk/fetch_wheelhouse.sh 抓取后回填（步骤 07）",
            }
        )
    return {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{matrix['sdk']['name']}-{version}-sbom-placeholder",
        "documentNamespace": "https://iraf.intewell.io/sbom/placeholder",
        "creationInfo": {
            "creators": ["Tool: deploy/sdk/lib_manifest.py"],
            "comment": "占位文档：不由构建时间/机器生成内容，版本字段一律 unverified",
        },
        "packages": packages,
    }


def build_manifest(
    *,
    matrix: dict,
    target: dict,
    version: str,
    repo_root: Path,
    artifacts: list[dict],
    bundle_excluded: list[dict],
    idl_digest_value: str,
    stubs: dict,
    stub_probe: dict,
    local_stub_generation: dict,
) -> dict:
    git = git_state(repo_root)
    notes = [
        "x86-first 战役（iraf-24h）：本产物在开发端 x86_64 构建，目标端验收 DEFERRED（板卡不在场）",
        f"target.verified=false：{'/'.join(target.get('boards', []))} 的 OS/Python/平台标签仍为 unverified（未实测）",
        "signature.scheme=pending：签名流程未落地，产物不得当作已签名分发",
        f"wheelhouse.status=pending_fetch：由 deploy/sdk/fetch_wheelhouse.sh 抓取（步骤 07），index_url={target['index_url']}",
        "IDL gencode：本机 protoc 生成的 stub 无法在本机 protobuf 下导入（见 stubs.importable_locally），该层未验证",
    ]
    if bundle_excluded:
        notes.append(
            f"bundle 排除了 {len(bundle_excluded)} 个非运行时文件（字节码/编辑器备份），清单见 bundle.excluded"
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "manifest_scope": "sdk+runtime",
        "name": matrix["sdk"]["name"],
        "version": version,
        "idlv1_compatible": [matrix["idlv1_version"]],
        "git": git,
        "target": {
            "id": target["id"],
            "arch": target["arch"],
            "python_tag": target["python_tag"],
            "platform_tag": target["platform_tag"],
            "boards": list(target.get("boards", [])),
            "verified": False,
        },
        "build": {
            "in": "x86_64 (开发端)",
            "out": "aarch64 (目标端，未验证)",
            "python": {
                "version": "%d.%d.%d" % sys.version_info[:3],
                "implementation": sys.implementation.name,
            },
            "kind": "source_bundle + pure_python_wheel",
        },
        "artifacts": artifacts,
        "bundle": {"dirs": list(BUNDLE_DIRS), "excluded": bundle_excluded},
        "idl": {
            "version": matrix["idlv1_version"],
            "proto_root": DEFAULT_PROTO_ROOT.as_posix(),
            "files": [rel.as_posix() for rel in IDL_PROTO_FILES],
            "digest": idl_digest_value,
        },
        "stubs": {
            "canonical_dir": stubs["dir"],
            "messages_present": stubs["messages_present"],
            "grpc_present": stubs["grpc_present"],
            "grpc_expected": stubs["grpc_expected"],
            "services": {
                name: item["services"] for name, item in stubs["grpc"].items() if item["services"]
            },
            "grpc_plugin": stubs["grpc_plugin"],
            "grpc_regenerable_locally": stubs["grpc_regenerable_locally"],
            "grpc": stubs["grpc"],
            "local_generation": local_stub_generation,
            "importable_locally": stub_probe["ok"],
            "import_probe_reason": stub_probe["reason"],
            "declared_protobuf": declared_protobuf_spec(repo_root),
        },
        "wheelhouse": {
            "status": "pending_fetch",
            "script": "deploy/sdk/fetch_wheelhouse.sh",
            "index_url": target["index_url"],
            "declared": {"wheels": list(target.get("wheels", [])),
                         "pure_python": list(target.get("pure_python", []))},
        },
        "sbom": {"path": "sbom.spdx.json", "status": "placeholder", "note": "步骤 07 抓取后回填版本"},
        "signature": {"scheme": "pending", "value": None},
        "notes": notes,
    }


def absolute_path_violations(node, path: str = "$") -> list[str]:
    """递归找出本机绝对路径：manifest 只允许仓库相对路径。"""
    violations: list[str] = []
    if isinstance(node, str):
        if node.startswith("/") or re.match(r"^[A-Za-z]:[\\/]", node):
            violations.append(f"{path} 出现绝对路径：{node[:120]}")
        elif re.search(r"(^|[\s\"'(])(/home|/Users|/root|/mnt|/data|/srv)/", node):
            violations.append(f"{path} 出现本机路径片段：{node[:120]}")
    elif isinstance(node, dict):
        for key, value in node.items():
            violations.extend(absolute_path_violations(value, f"{path}.{key}"))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            violations.extend(absolute_path_violations(value, f"{path}[{index}]"))
    return violations


def enforce_idl_contract(
    previous_manifest_path: Path, current_digest: str, declared_version: str
) -> None:
    """IDL 破坏性变更门禁：proto 内容变了但 `idlv1_version` 没升 → 退出码 4。

    "关键拒绝条件"（设计 §5 的 build_sdk.sh 行）：IDL 破坏性变更但未声明兼容版本。
    """
    if not previous_manifest_path.is_file():
        return
    try:
        previous = json.loads(_read_text(previous_manifest_path))
    except json.JSONDecodeError as exc:
        warn(f"上一份 manifest 不是合法 JSON（{exc}），无法做 IDL 兼容性比对；本次将覆盖它")
        return
    old_idl = (previous.get("idl") or {}).get("digest")
    old_version = (previous.get("idl") or {}).get("version")
    if old_idl and old_idl != current_digest and old_version == declared_version:
        raise ManifestError(
            "IDL 内容已变化但 idlv1_version 未升版本"
            f"（旧 digest {old_idl[:16]}… / 新 digest {current_digest[:16]}…，"
            f"版本均为 {old_version}）：破坏性变更必须先声明兼容版本",
            ExitCode.VERIFY,
        )


def write_manifest(out_dir: Path, manifest: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "manifest.json"
    path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    return path


def write_sha256sums(out_dir: Path, artifacts: list[dict]) -> Path:
    path = out_dir / "sha256sums.txt"
    lines = [f"{artifact['sha256']}  {Path(artifact['path']).name}" for artifact in artifacts]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _manifest_member_hashes(repo_root: Path, artifact: dict) -> tuple[dict, list[str]]:
    """重算产物内部每个文件的 SHA-256（用于发现"包内被改一个字节"）。"""
    path = repo_root / artifact["path"]
    hashes: dict[str, str] = {}
    violations: list[str] = []
    kind = artifact.get("kind")
    if kind == "wheel":
        try:
            with zipfile.ZipFile(path) as handle:
                for name in handle.namelist():
                    hashes[name] = sha256_bytes(handle.read(name))
        except (zipfile.BadZipFile, OSError) as exc:
            violations.append(f"wheel 无法解析（CRC/结构破损）：{artifact['path']}：{exc}")
    elif kind == "runtime_bundle":
        try:
            with tarfile.open(path, "r:gz") as tar:
                for member in tar.getmembers():
                    if not member.isfile():
                        continue
                    extracted = tar.extractfile(member)
                    hashes[member.name] = sha256_bytes(extracted.read()) if extracted else ""
        except (tarfile.TarError, OSError, EOFError) as exc:
            violations.append(f"bundle 无法解析（gzip/tar 破损）：{artifact['path']}：{exc}")
    else:
        violations.append(f"未知产物类型：{kind}（无法逐文件校验）")
    return hashes, violations


def verify_manifest(manifest_path: Path, repo_root: Path) -> list[str]:
    """校验 manifest 与磁盘上产物的一致性；返回中文违规清单（空 = 通过）。"""
    if not manifest_path.is_file():
        raise ManifestError(
            f"manifest 不存在：{manifest_path}（先执行 bash deploy/sdk/build_sdk.sh）",
            ExitCode.PRECHECK,
        )
    try:
        manifest = json.loads(_read_text(manifest_path))
    except json.JSONDecodeError as exc:
        return [f"manifest 不是合法 JSON：{exc}"]

    violations: list[str] = []
    if manifest.get("schema_version") != SCHEMA_VERSION:
        violations.append(
            f"manifest 的 schema_version 不是 {SCHEMA_VERSION}：{manifest.get('schema_version')!r}"
        )
    violations.extend(absolute_path_violations(manifest, "manifest"))

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        violations.append("manifest 缺少非空的 artifacts 列表")
        return violations

    for artifact in artifacts:
        rel = artifact.get("path")
        path = repo_root / str(rel)
        if not path.is_file():
            violations.append(f"产物缺失：{rel}（先在开发端重新执行 build_sdk.sh）")
            continue
        actual = sha256_file(path)
        if actual != artifact.get("sha256"):
            violations.append(
                f"产物 SHA-256 不符：{rel}"
                f"（manifest {str(artifact.get('sha256'))[:16]}… / 实际 {actual[:16]}…）"
            )
        if path.stat().st_size != artifact.get("size_bytes"):
            violations.append(
                f"产物大小不符：{rel}（manifest {artifact.get('size_bytes')} / 实际 {path.stat().st_size}）"
            )
        declared_files = artifact.get("files") or []
        if not declared_files:
            violations.append(f"产物未登记逐文件 SHA-256：{rel}")
            continue
        hashes, member_violations = _manifest_member_hashes(repo_root, artifact)
        violations.extend(member_violations)
        if member_violations:
            continue
        for entry in declared_files:
            member = entry.get("path")
            if member not in hashes:
                violations.append(f"产物内文件缺失：{rel} :: {member}")
            elif hashes[member] != entry.get("sha256"):
                violations.append(
                    f"产物内文件被篡改：{rel} :: {member}"
                    f"（manifest {str(entry.get('sha256'))[:16]}… / 实际 {hashes[member][:16]}…）"
                )
        extra = sorted(set(hashes) - {entry.get("path") for entry in declared_files})
        if extra:
            violations.append(f"产物含未登记文件：{rel} :: {extra[:5]}")
    return violations


# --- CLI -----------------------------------------------------------------------

USAGE = """用法：lib_manifest.py <子命令> [选项]

子命令（build_sdk.sh 按序调用；也可单独跑）：
  plan        打印本轮构建计划（声明值、预期产物名、成员数），不写任何文件
  preflight   校验声明（矩阵契约、版本唯一来源、git 状态、protoc、既有 stub 状态）
  gen-stubs   用本机 protoc 生成消息层 stub 到 --pystub-out，并报告既有 stub 状态
  wheel       构建纯 Python SDK wheel（并做文件名/版本/零依赖三重校验）
  bundle      构建 runtime bundle tar.gz（确定性、只含仓库相对路径）
  manifest    汇总产物 → 写 manifest.json + sbom.spdx.json + sha256sums.txt
  verify      校验 manifest 与产物（含逐文件 SHA-256），任何不符即退出码 4

选项：
  --repo-root <dir>      仓库根目录（默认：本文件的上两级）
  --matrix <path>        产物矩阵（默认 config/sdk/package_matrix.yaml）
  --out <dir>            产物输出目录（默认 build/sdk）
  --target <id>          构建目标 id（矩阵声明多个目标时必填）
  --manifest <path>      verify 用的 manifest（默认 <out>/manifest.json）
  --protoc <path>        protoc 可执行文件（默认取 PATH 中的 protoc）
  --pystub-out <dir>     本机 stub 生成目录（默认 build/sdk/pystub-local）
  --json-out <path>      preflight 的 JSON 摘要落盘路径
  --require-stubs        既有 stub 目录不完整即失败（默认只警告并写进 manifest）
"""

_OPTIONS_WITH_VALUE = {
    "--repo-root",
    "--matrix",
    "--out",
    "--target",
    "--manifest",
    "--protoc",
    "--pystub-out",
    "--json-out",
}
_FLAGS = {"--require-stubs"}


def parse_options(argv: list[str]) -> dict:
    options: dict = {"flags": set()}
    index = 0
    while index < len(argv):
        token = argv[index]
        if token in _FLAGS:
            options["flags"].add(token)
        elif token in _OPTIONS_WITH_VALUE:
            if index + 1 >= len(argv):
                raise ManifestError(f"选项 {token} 缺少取值", ExitCode.USAGE)
            options[token.lstrip("-").replace("-", "_")] = argv[index + 1]
            index += 1
        else:
            raise ManifestError(f"无法识别的参数：{token}（--help 查看用法）", ExitCode.USAGE)
        index += 1
    return options


def _paths(options: dict) -> tuple[Path, Path, Path]:
    repo_root = Path(options.get("repo_root") or Path(__file__).resolve().parents[2]).resolve()
    matrix = Path(options.get("matrix") or DEFAULT_MATRIX)
    if not matrix.is_absolute():
        matrix = repo_root / matrix
    out_dir = Path(options.get("out") or DEFAULT_OUT)
    if not out_dir.is_absolute():
        out_dir = repo_root / out_dir
    return repo_root, matrix, out_dir


def _preflight(repo_root: Path, matrix_path: Path, options: dict) -> dict:
    matrix = load_matrix(repo_root, matrix_path)
    target = select_target(matrix, options.get("target"))
    version = read_declared_version(repo_root, matrix["sdk"]["version_ref"])
    git = git_state(repo_root)
    protoc = options.get("protoc") or shutil.which("protoc")
    protoc_info = protoc_version(protoc) if protoc else "missing"
    stubs = stub_status(repo_root)
    declared = declared_protobuf_spec(repo_root)
    return {
        "matrix": matrix,
        "target": target,
        "version": version,
        "git": git,
        # 只记录版本与来源类别：产物/摘要里不得出现本机绝对路径（只写盘的内容同样受此约束）
        "protoc": protoc_info,
        "protoc_source": "explicit --protoc" if options.get("protoc") else "PATH",
        "stubs": stubs,
        "declared_protobuf": declared,
        "idl_digest": idl_digest(repo_root),
    }


def _planned_artifacts(repo_root: Path, matrix: dict, target: dict, version: str) -> dict:
    wheel_name = (
        f"{normalize_dist_name(matrix['sdk']['name'])}-{version}-{WHEEL_TAG}.whl"
    )
    bundle_name = f"iraf-runtime-{version}-{target['arch']}.tar.gz"
    members, excluded = iter_bundle_files(repo_root)
    return {
        "wheel": wheel_name,
        "bundle": bundle_name,
        "bundle_members": len(members),
        "bundle_excluded": len(excluded),
    }


def cmd_plan(options: dict) -> int:
    repo_root, matrix_path, out_dir = _paths(options)
    plan = _preflight(repo_root, matrix_path, options)
    artifacts = _planned_artifacts(repo_root, plan["matrix"], plan["target"], plan["version"])
    info("打包计划（--dry-run：不写任何文件）")
    info(f"  仓库根          : {relpath_posix(repo_root, repo_root) or '.'}")
    info(f"  声明矩阵        : {display_path(matrix_path, repo_root)}"
         f"（契约 {MATRIX_SCHEMA_PATH.as_posix()}：通过）")
    info(f"  构建目标        : {plan['target']['id']}"
         f"（arch={plan['target']['arch']}, python_tag={plan['target']['python_tag']},"
         f" platform_tag={plan['target']['platform_tag']}, verified=false）")
    info(f"  板卡            : {', '.join(plan['target']['boards'])}（未实测，目标端安装 DEFERRED）")
    info(f"  版本（单一来源）: {plan['version']}  ← {plan['matrix']['sdk']['version_ref']}")
    info(f"  IDL             : v{plan['matrix']['idlv1_version']}"
         f" digest={plan['idl_digest'][:16]}…（{len(IDL_PROTO_FILES)} 个 proto）")
    info(f"  git             : {plan['git']['commit'][:12]}"
         f" dirty={plan['git']['dirty']}（{plan['git']['dirty_file_count']} 个未提交路径）")
    info(f"  protoc          : {plan['protoc']}")
    info(f"  既有 stub 目录  : {plan['stubs']['dir']}"
         f" 消息层={plan['stubs']['messages_present']} grpc层={plan['stubs']['grpc_present']}"
         f" 插件={plan['stubs']['grpc_plugin']}")
    info(f"  产出 1（SDK）   : {display_path(out_dir, repo_root)}/{artifacts['wheel']}")
    info(f"  产出 2（runtime）: {display_path(out_dir, repo_root)}/{artifacts['bundle']}"
         f"（{artifacts['bundle_members']} 个文件，排除 {artifacts['bundle_excluded']} 个非运行时文件）")
    info(f"  产出 3（清单）   : {display_path(out_dir, repo_root)}/"
         "{manifest.json,sbom.spdx.json,sha256sums.txt}")
    info("  说明            : 目标端安装/健康检查不在本轮承诺内（板卡不在场 → DEFERRED）")
    return ExitCode.OK


def cmd_preflight(options: dict) -> int:
    repo_root, matrix_path, out_dir = _paths(options)
    plan = _preflight(repo_root, matrix_path, options)
    summary = {
        "schema_version": "iraf.sdk-build-preflight/v1",
        "matrix": relpath_posix(matrix_path, repo_root),
        "matrix_schema": MATRIX_SCHEMA_PATH.as_posix(),
        "target": plan["target"],
        "version": plan["version"],
        "git": plan["git"],
        "protoc": {"version": plan["protoc"], "source": plan["protoc_source"]},
        "stubs": plan["stubs"],
        "declared_protobuf": plan["declared_protobuf"],
        "idl_digest": plan["idl_digest"],
    }
    violations = absolute_path_violations(summary)
    if violations:
        raise ManifestError("预检摘要出现绝对路径（只允许仓库相对路径）", ExitCode.PRECHECK, violations)
    if options.get("json_out"):
        target = Path(options["json_out"])
        if not target.is_absolute():
            target = repo_root / target
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        info(f"预检摘要已落盘：{display_path(target, repo_root)}")
    info(f"预检通过：矩阵契约合法，版本 {plan['version']}，目标 {plan['target']['id']}，"
         f"protoc {plan['protoc']}")
    if plan["stubs"]["messages_present"] and not plan["stubs"]["grpc_present"]:
        warn("既有 stub 目录缺少 *_pb2_grpc.py：本机无 grpc_python_plugin，无法生成该层")
    return ExitCode.OK


def cmd_gen_stubs(options: dict) -> int:
    repo_root, _, _ = _paths(options)
    protoc = options.get("protoc") or shutil.which("protoc")
    if not protoc:
        raise ManifestError("PATH 中没有 protoc：无法生成 IDL stub", ExitCode.PRECHECK)
    version = protoc_version(protoc)
    scratch = Path(options.get("pystub_out") or Path("build/sdk/pystub-local"))
    if not scratch.is_absolute():
        scratch = repo_root / scratch
    generation = generate_pystubs(protoc, repo_root, scratch)
    info(f"protoc {version} → {relpath_posix(scratch, repo_root)}")
    info(f"生成消息层 stub {len(generation['files'])} 个：{generation['files']}")
    if generation["returncode"] != 0:
        raise ManifestError(
            f"protoc 生成失败（退出码 {generation['returncode']}）：{generation['stderr_tail']}",
            ExitCode.BUILD,
        )
    probe = stub_import_probe(scratch)
    if probe["ok"]:
        info("本机导入探针：通过")
    else:
        warn(f"本机导入探针：不通过（{probe['reason']}）")
    stubs = stub_status(repo_root)
    if stubs["messages_present"] and stubs["grpc_present"]:
        info(f"既有 stub 目录完整（{stubs['dir']}）")
    else:
        message = (
            f"既有 stub 目录不完整（{stubs['dir']}）："
            f"消息层={stubs['messages_present']} grpc层={stubs['grpc_present']}"
        )
        if "--require-stubs" in options["flags"]:
            raise ManifestError(message + "；--require-stubs 已开启", ExitCode.BUILD)
        warn(message + "；未开启 --require-stubs，仅记录进 manifest（该层未验证）")
    if not stubs["grpc_regenerable_locally"]:
        warn(
            "本机没有 grpc_python_plugin，*_pb2_grpc.py 无法在本机复现；"
            "既有 grpc stub 按原样登记（provenance=prebuilt）"
        )
    return ExitCode.OK


def cmd_wheel(options: dict) -> int:
    repo_root, matrix_path, out_dir = _paths(options)
    plan = _preflight(repo_root, matrix_path, options)
    matrix, target, version = plan["matrix"], plan["target"], plan["version"]
    built = build_wheel(
        repo_root,
        out_dir,
        dist_name=matrix["sdk"]["name"],
        version=version,
        package=matrix["sdk"]["module"],
        summary="IRAF Python SDK (pure Python, cross-arch)",
        requires_python=">=3.9",
    )
    violations = validate_wheel(
        built["path"],
        dist_name=matrix["sdk"]["name"],
        version=version,
        package=matrix["sdk"]["module"],
        required_members=built["required_members"],
    )
    if violations:
        raise ManifestError("wheel 校验失败（构建失败，退出码 3）", ExitCode.BUILD, violations)
    digest = sha256_file(built["path"])
    info(f"wheel 产出：{display_path(built['path'], repo_root)}"
         f"（{len(built['files'])} 个成员）")
    info(f"  sha256={digest}")
    info("  校验：文件名/版本/标签、METADATA、RECORD 覆盖、零第三方依赖 —— 通过")
    info(f"  目标端标签（声明，未验证）：{target['platform_tag']} / {target['python_tag']}"
         "（SDK 为 py3-none-any，与架构无关）")
    return ExitCode.OK


def cmd_bundle(options: dict) -> int:
    repo_root, matrix_path, out_dir = _paths(options)
    plan = _preflight(repo_root, matrix_path, options)
    matrix, target, version = plan["matrix"], plan["target"], plan["version"]
    built = build_runtime_bundle(
        repo_root,
        out_dir,
        name="iraf-runtime",
        version=version,
        arch=target["arch"],
    )
    digest = sha256_file(built["path"])
    info(f"bundle 产出：{display_path(built['path'], repo_root)}"
         f"（{len(built['files'])} 个文件）")
    info(f"  sha256={digest}")
    if built["excluded"]:
        info(f"  排除 {len(built['excluded'])} 个非运行时文件（已登记进 manifest.bundle.excluded）")
    info(f"  成员目录：{', '.join(BUNDLE_DIRS)}（均为仓库相对路径）")
    return ExitCode.OK


def cmd_manifest(options: dict) -> int:
    repo_root, matrix_path, out_dir = _paths(options)
    plan = _preflight(repo_root, matrix_path, options)
    matrix, target, version = plan["matrix"], plan["target"], plan["version"]

    wheel_path = out_dir / f"{normalize_dist_name(matrix['sdk']['name'])}-{version}-{WHEEL_TAG}.whl"
    bundle_path = out_dir / f"iraf-runtime-{version}-{target['arch']}.tar.gz"
    missing = [
        relpath_posix(path, repo_root) for path in (wheel_path, bundle_path) if not path.is_file()
    ]
    if missing:
        raise ManifestError(
            f"产物缺失，无法写 manifest：{missing}（先执行 wheel/bundle 步骤）", ExitCode.BUILD
        )
    try:
        out_dir.resolve().relative_to(repo_root.resolve())
    except ValueError as exc:
        raise ManifestError(
            f"产物目录必须在仓库内（manifest 只允许仓库相对路径）：{out_dir}", ExitCode.PRECHECK
        ) from exc

    artifacts = []
    for kind, path, reader in (
        ("wheel", wheel_path, _zip_members),
        ("runtime_bundle", bundle_path, _tar_members),
    ):
        files = [
            {"path": name, "sha256": sha256_bytes(data), "size_bytes": len(data)}
            for name, data in sorted(reader(path), key=lambda item: item[0])
        ]
        artifacts.append(
            {
                "kind": kind,
                "path": relpath_posix(path, repo_root),
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
                "files": files,
            }
        )

    idl_now = idl_digest(repo_root)
    enforce_idl_contract(out_dir / "manifest.json", idl_now, matrix["idlv1_version"])

    stubs = plan["stubs"]
    scratch = Path(options.get("pystub_out") or Path("build/sdk/pystub-local"))
    if not scratch.is_absolute():
        scratch = repo_root / scratch
    scratch_stubs = scratch / "iraf" / "v1"
    local_generation = {
        "dir": relpath_posix(scratch, repo_root),
        "protoc": plan["protoc"],
        "files": sorted(p.relative_to(scratch).as_posix() for p in scratch.rglob("*_pb2.py"))
        if scratch_stubs.is_dir()
        else [],
        "regenerable_locally": True,
        "grpc_included": False,
    }
    _, excluded = iter_bundle_files(repo_root)
    manifest = build_manifest(
        matrix=matrix,
        target=target,
        version=version,
        repo_root=repo_root,
        artifacts=artifacts,
        bundle_excluded=excluded,
        idl_digest_value=idl_now,
        stubs=stubs,
        stub_probe=stub_import_probe(scratch),
        local_stub_generation=local_generation,
    )
    violations = absolute_path_violations(manifest)
    if violations:
        raise ManifestError("manifest 含绝对路径，拒绝写出", ExitCode.BUILD, violations)
    manifest_path = write_manifest(out_dir, manifest)
    sbom_path = out_dir / "sbom.spdx.json"
    sbom_path.write_text(
        json.dumps(sbom_placeholder(matrix, target, version), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    sums_path = write_sha256sums(out_dir, artifacts)
    info(f"manifest 写出：{display_path(manifest_path, repo_root)}"
         f"（schema={SCHEMA_VERSION}，artifacts={len(artifacts)}，"
         f"逐文件条目={sum(len(a['files']) for a in artifacts)}）")
    info(f"SBOM 占位：{display_path(sbom_path, repo_root)}"
         f"；校验和：{display_path(sums_path, repo_root)}")
    return ExitCode.OK


def _zip_members(path: Path) -> list[tuple[str, bytes]]:
    with zipfile.ZipFile(path) as handle:
        return [(name, handle.read(name)) for name in handle.namelist()]


def _tar_members(path: Path) -> list[tuple[str, bytes]]:
    members = []
    with tarfile.open(path, "r:gz") as tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            handle = tar.extractfile(member)
            members.append((member.name, handle.read() if handle else b""))
    return members


def cmd_verify(options: dict) -> int:
    repo_root, _, out_dir = _paths(options)
    manifest_path = Path(options.get("manifest") or (out_dir / "manifest.json"))
    if not manifest_path.is_absolute():
        manifest_path = repo_root / manifest_path
    violations = verify_manifest(manifest_path, repo_root)
    if violations:
        for item in violations:
            info(f"  违规：{item}")
        raise ManifestError(f"校验失败：{len(violations)} 处不符", ExitCode.VERIFY, violations)
    info(f"校验通过：{display_path(manifest_path, repo_root)} 与产物逐文件 SHA-256 一致")
    return ExitCode.OK


COMMANDS = {
    "plan": cmd_plan,
    "preflight": cmd_preflight,
    "gen-stubs": cmd_gen_stubs,
    "wheel": cmd_wheel,
    "bundle": cmd_bundle,
    "manifest": cmd_manifest,
    "verify": cmd_verify,
}


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return ExitCode.OK if argv else ExitCode.USAGE
    command, rest = argv[0], argv[1:]
    if command not in COMMANDS:
        print(f"[build_sdk] 无法识别的子命令：{command}\n", file=sys.stderr)
        print(USAGE, file=sys.stderr)
        return ExitCode.USAGE
    try:
        options = parse_options(rest)
        return COMMANDS[command](options)
    except ManifestError as exc:
        print(f"[build_sdk][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[build_sdk][错误]   - {detail}", file=sys.stderr)
        return exc.exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
