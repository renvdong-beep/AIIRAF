#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IRAF 安装记录 / 残留核验 / 受控卸载（实现层，2026-10-12）。

定位：`deploy/sdk/{install-iraf,uninstall-iraf,verify_residue}.sh` 的实现层，只做三件事：
  ① **安装记录**（record）—— 每次安装把"装了什么"逐文件落盘（路径 + SHA-256 + 大小 + 元数据）；
  ② **残留核验**（residue）—— 卸载后六路扫描（文件/目录/systemd 单元/env 文件/监听端口/进程），
     全部为 0 才算"清干净"；
  ③ **受控卸载**（uninstall）—— **只**按安装记录删；清单外文件、记录内缺失、哈希不符一律拒绝，
     不做"尽力而为"删除。

为什么必须这样（本项目反复踩的坑）：
  · "卸载脚本自己说自己删干净了"不是证据 ⇒ 残留核验是**独立入口**，它不信任卸载脚本的自述；
  · 同一事实散落多处 ⇒ 布局（prefix/版本目录/unit/env/端口/保留项）**只**来自
    `config/install/install_profiles.yaml`，本文件不写死任何路径或端口；
  · "改了声明没生效/改错段" ⇒ 解析语义复用 `scripts/install_profile_check.py` 的 `resolve()`
    （同一份实现，避免两套解析规则）。

边界（不得违反）：
  · 只读为主：`residue` 与 `uninstall --dry-run` 不删任何文件；
  · `uninstall`（非 dry-run）只删记录内、且在声明根（prefix/config_dir）之下的文件；
  · 数据与日志（`uninstall.keep`）默认保留，只有 `--purge` 才删；
  · **不**碰 dpkg/dnf 系统包、**不**碰 AgentOS 自己的文件（install_boundaries 已在声明里固定为 false/禁止）。

用法（一般由 shell 入口调用；也可直接跑）：
    python3 deploy/sdk/lib_install.py residue     --profile single-ubuntu2204 [--root <沙箱>] [--json]
    python3 deploy/sdk/lib_install.py uninstall   --profile <id> [--root <沙箱>] [--dry-run] [--purge]
    python3 deploy/sdk/lib_install.py record-write --installed-dir <dir> --profile <id> [--note ...]
    python3 deploy/sdk/lib_install.py record-show --installed-dir <dir> --profile <id>
    python3 deploy/sdk/lib_install.py layout      --profile <id>

退出码（与 docs/iraf-multiplatform-sdk-design.md §5 统一约定一致）：
    0 成功/演练完成   1 参数或用法错误   2 声明/预检失败
    3 安装/卸载 IO 失败   4 校验失败（哈希不符、清单外文件、记录与树不一致）   5 服务状态不符
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_IO = 3
EXIT_MISMATCH = 4
EXIT_SERVICE = 5

REPO_ROOT = Path(__file__).resolve().parents[2]
DECLARATION = Path("config/install/install_profiles.yaml")
CHECKER = Path("scripts/install_profile_check.py")

RECORD_SCHEMA = "iraf.install-record/v1"

#: 安装后**由运行本身产生**的生成物（不属于安装清单，也不应让卸载被拒）：
#: Python 运行时写出的字节码缓存。判据只影响"清单外文件"的判定；它们同样随版本目录一起删除。
GENERATED_PATH_PATTERNS = ("/__pycache__/", "__pycache__/", ".pyc", ".pyo")


def is_generated_artifact(relative_path: str) -> bool:
    return any(token in relative_path for token in GENERATED_PATH_PATTERNS)


class InstallError(Exception):
    """带退出码的显式失败（禁止裸异常，禁止默认成功）。"""

    def __init__(self, message: str, code: int = EXIT_DECLARATION):
        super().__init__(message)
        self.code = code


def info(message: str) -> None:
    print("[install] %s" % message)


def _load_checker(repo_root: Path):
    """复用声明解析实现（同一份 resolve()，避免两套解析规则）。"""
    path = repo_root / CHECKER
    if not path.is_file():
        raise InstallError("缺少声明解析实现 %s" % CHECKER)
    spec = importlib.util.spec_from_file_location("install_profile_check", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    return module


def load_declaration(repo_root: Path) -> dict:
    path = repo_root / DECLARATION
    if not path.is_file():
        raise InstallError("缺少安装形态声明 %s" % DECLARATION)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "profiles" not in data:
        raise InstallError("%s 结构不合法（应由 install_profile_check.py 校验）" % DECLARATION)
    return data


def profile_by_id(declaration: dict, profile_id: str) -> dict:
    for item in declaration["profiles"]:
        if str(item["id"]) == profile_id:
            return item
    raise InstallError("未知安装角色 %r（可选：%s）"
                       % (profile_id, ", ".join(str(i["id"]) for i in declaration["profiles"])),
                       EXIT_USAGE)


def effective_layout(repo_root: Path, profile_id: str) -> dict:
    """把 defaults + 单角色覆盖解析成**生效值**（复用校验器实现）。"""
    checker = _load_checker(repo_root)
    declaration = load_declaration(repo_root)
    profile = profile_by_id(declaration, profile_id)
    effective = checker.resolve(profile, declaration["defaults"])
    target = profile.get("target", {})
    effective["id"] = profile_id
    effective["label"] = profile.get("label")
    effective["version"] = checker.read_toml_field(repo_root, declaration["sdk"]["version_ref"])
    effective["dist_name"] = checker.read_toml_field(repo_root, declaration["sdk"]["dist_name_ref"])
    effective["components"] = list(profile["components"])
    effective["backend_entrypoint"] = profile["backend_entrypoint"]
    effective["services"] = list(profile["services"])
    effective["simulation_only"] = bool(profile.get("simulation_only"))
    effective["purge_flag"] = declaration["defaults"]["uninstall"]["purge_flag"]
    # 预检需要的目标端事实与部署期参数（全部来自声明；这里只做展开，不补默认值）
    effective["declaration_verification"] = profile.get("verification")
    effective["target_arch"] = target.get("arch")
    effective["target_os"] = target.get("os")
    effective["target_os_version"] = target.get("os_version")
    effective["target_kernel"] = target.get("kernel")
    effective["target_python_tag"] = target.get("python_tag")
    effective["target_platform_tag"] = target.get("platform_tag")
    effective["target_glibc_min"] = target.get("glibc_min")
    for key in ("min_free_mb", "python_bin", "run_user", "robot_profile", "safety_policy",
                "backend_config", "runtime_subject", "runtime_dependencies", "python_stubs_dir",
                "sim_scene", "env_template", "provider_display"):
        # **角色级优先**：仿真档声明仿真机型/仿真策略，真机档留空待运控域 —— 不能让真机档继承仿真档的
        # safety_policy（那会把 simulation_only=true 的策略带到真机路径上）。
        if key in profile:
            effective[key] = profile[key]
        elif key in declaration["defaults"]:
            effective[key] = declaration["defaults"][key]
    effective["_repo_root"] = str(repo_root)
    return effective


def under_root(root: Path, declared: str) -> Path:
    """把声明的绝对路径挂到（可能是沙箱的）根下：root="/" + "/opt/x" 应为 "/opt/x" 而不是 "//opt/x"
    （实测踩到：安装记录里出现 `//opt/iraf-sdk/...`，虽能解析但难看且易误判）。"""
    text = str(declared)
    if str(root) in ("", "/"):
        return Path(text)
    return Path(str(root).rstrip("/")) / text.lstrip("/")


def unit_dir(root: Path, scope: str) -> Path:
    """单元落点（**唯一规则**：install / uninstall / residue 三处共用，避免规则漂移留残留）。

    system → `<root>/etc/systemd/system`；
    user   → 真实根且调用者**非 root** ⇒ `<家目录>/.config/systemd/user`
             （写 /etc/systemd/user 需要 root，单机开发档根本装不上，装完就没有服务）；
             否则（沙箱 root 或 root 身份）⇒ `<root>/etc/systemd/user`，便于演练逐字比对。
    """
    if str(scope) == "system":
        return under_root(root, "/etc/systemd/system")
    if str(root) in ("", "/") and hasattr(os, "geteuid") and os.geteuid() != 0:
        return Path.home() / ".config" / "systemd" / "user"
    return under_root(root, "/etc/systemd/user")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


# ─────────────────────────── 安装记录 ───────────────────────────

def record_path(effective: dict, version_dir: Path) -> Path:
    return version_dir / str(effective["file_record"])


def collect_installed_files(version_dir: Path, *, exclude: set) -> list:
    """列出已装版本目录下的全部文件（排除记录自身与 __pycache__ 之类生成物不算——它们也要删）。"""
    files = []
    if not version_dir.is_dir():
        raise InstallError("已装版本目录不存在：%s" % version_dir, EXIT_IO)
    for path in sorted(version_dir.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(version_dir)
        if str(rel) in exclude:
            continue
        files.append({"path": str(rel), "size": path.stat().st_size, "sha256": _sha256(path)})
    return files


def write_record(effective: dict, version_dir: Path, *, note: str = "",
                 config_paths: list = (), created_dirs: list = ()) -> Path:
    """写安装记录（install 的最后一步；uninstall 只依据它决定删什么）。

    `config_paths`：装在**版本目录之外**的文件（如 `/etc/iraf/<profile>.env`）。
    `created_dirs`：本次安装**新建**的目录（前缀、配置目录等）。
    漏记它们 ⇒ 卸载后残留（本轮实测踩到两次：env 文件不在记录里、安装目录空壳留下）。
    """
    record_name = str(effective["file_record"])
    files = collect_installed_files(version_dir, exclude={record_name})
    config_files = []
    for item in config_paths:
        path = Path(item)
        if path.is_file():
            config_files.append({"path": str(path), "size": path.stat().st_size, "sha256": _sha256(path)})
    record = {
        "schema_version": RECORD_SCHEMA,
        "profile": effective["id"],
        "label": effective.get("label"),
        "dist_name": effective["dist_name"],
        "version": effective["version"],
        "simulation": bool(effective["simulation_only"]),
        "installed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "version_dir": str(version_dir),
        "current_link": str(Path(effective["prefix"]) / str(effective["current_link"])),
        "units": [str(item["name"]) + ".service" for item in effective["services"]],
        "ports": [int(port) for item in effective["services"] for port in (item.get("ports") or [])],
        "config_files": config_files,
        "created_dirs": [str(Path(item)) for item in created_dirs],
        "file_count": len(files) + len(config_files),
        "files": files,
        "note": note,
    }
    target = record_path(effective, version_dir)
    target.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    info("安装记录已写入：%s（版本目录 %d 个文件 + 配置 %d 个 + 新建目录 %d 个）"
         % (target, len(files), len(config_files), len(record["created_dirs"])))
    return target


def read_record(effective: dict, version_dir: Path) -> dict:
    target = record_path(effective, version_dir)
    if not target.is_file():
        raise InstallError("缺少安装记录 %s ⇒ 拒绝卸载/核验（没有记录就无从判断"
                           "\"只动本包\"，本工具不做尽力而为的删除）" % target, EXIT_MISMATCH)
    try:
        record = json.loads(target.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise InstallError("安装记录不是合法 JSON：%s（%s）" % (target, exc), EXIT_MISMATCH)
    if record.get("schema_version") != RECORD_SCHEMA:
        raise InstallError("安装记录 schema 不符：期望 %s，实际 %r"
                           % (RECORD_SCHEMA, record.get("schema_version")), EXIT_MISMATCH)
    if str(record.get("profile")) != str(effective["id"]):
        raise InstallError("安装记录属于角色 %r，与本次角色 %r 不符"
                           % (record.get("profile"), effective["id"]), EXIT_MISMATCH)
    return record


def resolve_version_dir(effective: dict, root: Path, *, version: str | None = None) -> Path:
    """<root><prefix>/<version>；version 缺省取 current 链接指向的目录名。"""
    prefix = under_root(root, str(effective["prefix"]))
    if not version:
        link = prefix / str(effective["current_link"])
        if link.is_symlink():
            version = os.path.basename(os.path.realpath(str(link)))
        elif link.is_dir():
            # 非符号链接（沙箱演练常见）：取唯一子目录
            children = [item for item in prefix.iterdir() if item.is_dir() and item.name != str(effective["current_link"])]
            if len(children) == 1:
                version = children[0].name
            else:
                raise InstallError("无法确定已装版本：%s 下有 %d 个版本目录，请用 --installed-dir 指定"
                                   % (prefix, len(children)), EXIT_USAGE)
        else:
            raise InstallError("未找到已装版本（%s 既不是符号链接也不是目录）" % link, EXIT_MISMATCH)
    return prefix / str(version)


# ─────────────────────────── 残留核验（六路） ───────────────────────────

def _listening_ports() -> set:
    """本机监听端口（只读；ss 优先，退化到 /proc/net/tcp{,6}）。"""
    ports = set()
    try:
        proc = subprocess.run(["ss", "-lntuH"], capture_output=True, text=True, timeout=10)
        if proc.returncode == 0:
            for line in proc.stdout.splitlines():
                fields = line.split()
                if len(fields) >= 5:
                    match = re.search(r":(\d+)$", fields[4])
                    if match:
                        ports.add(int(match.group(1)))
            return ports
    except (OSError, subprocess.SubprocessError):
        pass
    for name in ("/proc/net/tcp", "/proc/net/tcp6", "/proc/net/udp"):
        path = Path(name)
        if not path.is_file():
            continue
        for line in path.read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) > 3 and fields[3] == "0A":  # 0A = LISTEN
                try:
                    ports.add(int(fields[1].split(":")[1], 16))
                except (IndexError, ValueError):
                    continue
    return ports


def _running_processes() -> list:
    """扫描 /proc 下的 cmdline（只读；不用 pgrep，避免任何"误杀"可能）。"""
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        cmdline = entry / "cmdline"
        try:
            raw = cmdline.read_bytes().decode("utf-8", errors="replace").replace("\0", " ").strip()
        except OSError:
            continue
        if raw:
            found.append(raw)
    return found


def _unit_state(names: list, *, root: Path, scope: str) -> dict:
    """单元是否存在/启用（沙箱演练不查 systemd，直接看文件）。"""
    result = {}
    system_dir = unit_dir(root, scope)
    for name in names:
        candidates = [system_dir / name]
        if scope == "system":
            candidates.append(system_dir / "multi-user.target.wants" / name)
        result[name] = {"unit_file": str(candidates[0]), "exists": any(c.exists() for c in candidates),
                        "enabled_link": candidates[-1].exists()}
    return result


def residue_scan(effective: dict, root: Path, *, record: dict | None = None) -> dict:
    """六路残留扫描。record=None 时不比对记录（只回答"这些根下还剩什么"）。"""
    findings = {"files": [], "dirs": [], "units": {}, "env_files": [], "ports": [], "processes": []}
    roots = [under_root(root, str(item)) for item in effective["residue_scan_roots"]]
    for scan_root in roots:
        if not scan_root.exists():
            continue
        for path in sorted(scan_root.rglob("*")):
            if path.is_file():
                findings["files"].append(str(path))
            elif path.is_dir() and not any(path.iterdir()):
                findings["dirs"].append(str(path))
        if scan_root.is_file():
            findings["files"].append(str(scan_root))

    findings["units"] = _unit_state(list(effective["residue_units"]), root=root,
                                    scope=str(effective["systemd_scope"]))
    # env 文件（声明的 config_dir 下，按 profile 的服务命名）
    config_dir = under_root(root, str(effective["config_dir"]))
    if config_dir.is_dir():
        findings["env_files"] = [str(p) for p in sorted(config_dir.glob("*.env"))]
    listening = _listening_ports()
    findings["ports"] = [port for port in effective["residue_ports"] if port in listening]
    running = _running_processes()
    for name in effective["residue_processes"]:
        findings["processes"] += [cmd for cmd in running if name in cmd]

    if record is not None:
        recorded = {str(item["path"]) for item in record.get("files", [])}
        version_dir = Path(str(record.get("version_dir")))
        record_name = str(effective["file_record"])
        unlisted, generated = [], []
        for path_str in findings["files"]:
            path = Path(path_str)
            try:
                rel = str(path.relative_to(version_dir))
            except ValueError:
                continue  # 在其它根下（如 config_dir 的 env），另由 env_files 报
            if rel == record_name:
                continue
            if is_generated_artifact(rel):
                generated.append(rel)
                continue
            if rel not in recorded:
                unlisted.append(rel)
        missing = sorted(item["path"] for item in record.get("files", [])
                         if not (version_dir / item["path"]).is_file())
        findings["unlisted"] = unlisted
        findings["missing"] = missing
        findings["generated"] = generated
    return findings


def residue_is_clean(findings: dict) -> bool:
    """判定"清干净"：六路全 0（units 按 exists/enabled_link 判）。"""
    if findings["files"] or findings["dirs"] or findings["env_files"]:
        return False
    if findings["ports"] or findings["processes"]:
        return False
    for state in findings["units"].values():
        if state.get("exists") or state.get("enabled_link"):
            return False
    if findings.get("unlisted") or findings.get("missing"):
        return False
    return True


# ─────────────────────────── 受控卸载 ───────────────────────────

def plan_uninstall(effective: dict, root: Path, record: dict, *, purge: bool = False) -> dict:
    """生成卸载计划；任何越界/不一致 ⇒ 抛 InstallError（不做尽力而为）。"""
    version_dir = Path(str(record.get("version_dir")))
    keep_dirs = [under_root(root, str(item)) for item in effective["uninstall_keep"]]
    allowed_roots = [under_root(root, str(effective["prefix"])),
                     under_root(root, str(effective["config_dir"]))]
    allowed_roots = [item.resolve() for item in allowed_roots]

    recorded_rel = {str(item["path"]) for item in record.get("files", [])}
    record_name = str(effective["file_record"])
    delete, kept, refusals = [], [], []

    # ① 清单外文件：出现在版本目录里但不在记录中 ⇒ 拒绝（这是"只动本包"的核心保证）
    #    两个例外（都在声明/实现里写死，不是临时放宽）：① 安装记录自身；② 运行时生成物（__pycache__/*.pyc）
    if version_dir.is_dir():
        for path in sorted(version_dir.rglob("*")):
            if not path.is_file():
                continue
            rel = str(path.relative_to(version_dir))
            if rel == record_name or is_generated_artifact(rel):
                continue
            if rel not in recorded_rel:
                refusals.append("清单外文件（不在安装记录里，拒绝删除）：%s" % path)

    # ② 记录内文件：缺失/哈希不符/越界 ⇒ 拒绝
    for item in record.get("files", []):
        target = version_dir / str(item["path"])
        if not target.is_file():
            refusals.append("记录内文件缺失：%s" % target)
            continue
        if _sha256(target) != item.get("sha256"):
            refusals.append("哈希不符（说明文件被改过，拒绝删除）：%s" % target)
            continue
        resolved = target.resolve()
        if not any(str(resolved).startswith(str(base)) for base in allowed_roots):
            refusals.append("越界（不在声明根内）：%s" % target)
            continue
        delete.append(target)

    # ③ 版本目录之外的文件（env 等）：同样逐文件校验后删除，漏记就会残留（本轮实测踩过）
    for item in record.get("config_files", []):
        target = Path(str(item["path"]))
        if not target.is_file():
            refusals.append("记录内配置文件缺失：%s" % target)
            continue
        if _sha256(target) != item.get("sha256"):
            refusals.append("配置文件哈希不符（拒绝删除）：%s" % target)
            continue
        resolved = target.resolve()
        if not any(str(resolved).startswith(str(base)) for base in allowed_roots):
            refusals.append("配置文件越界（不在声明根内）：%s" % target)
            continue
        delete.append(target)

    # 数据与日志：默认保留；--purge 才删
    for keep in keep_dirs:
        if keep.exists() or keep.is_symlink():
            if purge:
                delete.append(keep)
            else:
                kept.append(keep)

    # 单元文件与 current 链接（记录里记了名字 ⇒ 属于本包）
    system_dir = unit_dir(root, str(effective["systemd_scope"]))
    for unit in record.get("units", []):
        for candidate in (system_dir / unit, system_dir / "multi-user.target.wants" / unit):
            if candidate.exists() or candidate.is_symlink():
                delete.append(candidate)
    link = under_root(root, str(effective["prefix"]) + "/" + str(effective["current_link"]))
    if link.is_symlink():
        delete.append(link)

    if refusals:
        raise InstallError("卸载被拒绝（%d 项）：\n  - %s" % (len(refusals), "\n  - ".join(refusals[:10])),
                           EXIT_MISMATCH)
    # 版本目录自身（内含记录）：删完文件后目录应可删
    if version_dir.is_dir():
        delete.append(version_dir)
    # 本次安装新建的目录：只在**变空**时才删（非空说明用户自己放了东西，保留）
    delete_dirs = [Path(str(item)) for item in record.get("created_dirs", [])]
    return {"delete": delete, "delete_dirs": delete_dirs, "keep": kept, "refusals": [],
            "prefix": str(effective["prefix"])}


def plan_force_clean(effective: dict, root: Path, *, purge: bool = False) -> dict:
    """**无记录兜底清理**：只删声明里列出的路径（前缀目录、unit 文件、env 文件、current 链接）。

    适用场景：有安装痕迹但**安装记录缺失**的半状态（例如上一次安装中途失败且回滚不完整）。
    与受控卸载的区别：没有逐文件 SHA-256 校验（没有记录可比），因此边界靠"只碰声明路径"保证，
    并且**逐项打印**；数据与日志仍默认保留（--purge 才删）。这不是"尽力而为删除"。
    """
    delete = []
    prefix = under_root(root, str(effective["prefix"]))
    if prefix.exists() or prefix.is_symlink():
        if prefix.is_dir():
            for child in sorted(prefix.iterdir()):
                delete.append(child)
        else:
            delete.append(prefix)
    unit_dir_path = unit_dir(root, str(effective["systemd_scope"]))
    for name in effective["residue_units"]:
        for candidate in (unit_dir_path / name, unit_dir_path / "multi-user.target.wants" / name):
            if candidate.exists() or candidate.is_symlink():
                delete.append(candidate)
    config_dir = under_root(root, str(effective["config_dir"]))
    if config_dir.is_dir():
        for candidate in sorted(config_dir.glob("*.env")):
            delete.append(candidate)
    kept = []
    for item in effective["uninstall_keep"]:
        path = under_root(root, str(item))
        if path.exists():
            (kept.append(path) if not purge else delete.append(path))
    return {"delete": delete, "keep": kept, "prefix": str(effective["prefix"]), "force_clean": True}


def apply_uninstall(plan: dict, *, root: Path) -> list:
    """执行卸载计划（只删计划内的路径），并清理因此变空的父目录（到安装前缀为止）。"""
    removed = []
    for target in plan["delete"]:
        try:
            if target.is_symlink() or target.is_file():
                target.unlink()
            elif target.is_dir():
                shutil.rmtree(target)
            removed.append(str(target))
        except OSError as exc:
            raise InstallError("删除失败 %s（%s）" % (target, exc), EXIT_IO)

    # 空目录收尾（两段，边界不同）：
    #   ① 记录里"本次安装新建的、位于前缀之外"的目录（如 /etc/iraf）：只尝试删它自己，
    #      **不向上走**（/etc 之类绝不能碰）。
    #   ② 前缀之内的空目录收敛：把删/建路径的**全部前缀内祖先**收集起来，按深度从深到浅
    #      逐个尝试 rmdir（只删空目录），最远到前缀本身为止。
    stop = under_root(root, str(plan["prefix"])).resolve()
    created = [Path(str(item)) for item in plan.get("delete_dirs", [])]
    for candidate in sorted(created, key=lambda item: len(str(item)), reverse=True):
        try:
            resolved = candidate.resolve()
        except OSError:
            continue
        if str(resolved).startswith(str(stop)):
            continue          # 前缀内的交给第 ② 段统一收敛
        if resolved.is_dir() and not any(resolved.iterdir()):
            try:
                resolved.rmdir()
                removed.append(str(resolved) + "（空目录）")
            except OSError:
                pass

    ancestors = set()
    for item in list(plan["delete"]) + list(created):
        path = Path(str(item)).parent
        while True:
            try:
                resolved = path.resolve()
            except OSError:
                break
            if not str(resolved).startswith(str(stop)):
                break
            ancestors.add(resolved)
            if resolved == stop:
                break
            path = resolved.parent
    for resolved in sorted(ancestors, key=lambda item: len(str(item)), reverse=True):
        if resolved.is_dir() and not any(resolved.iterdir()):
            try:
                resolved.rmdir()
                removed.append(str(resolved) + "（空目录）")
            except OSError:
                continue
    return removed


# ─────────────────────────── 安装动作 ───────────────────────────

#: env 文件的头部（由声明与安装记录共同解释；不含任何机器硬编码）
ENV_FILE_HEADER = """# IRAF 运行期环境（由 install-iraf.sh 按 config/install/install_profiles.yaml 渲染）
# 角色=%(profile)s 版本=%(version)s 生成于 %(generated_at)s
# 修改请改声明后重装；本文件已登记进安装记录（卸载会移除；手工改动会被哈希校验拦下）
"""

#: 渲染占位符时使用的 token → 取值函数（机器相关的值一律来自声明的生效值）
UNIT_TOKENS = ("PROFILE", "VERSION", "PREFIX", "CURRENT_LINK", "PYTHON_BIN", "CONFIG_DIR",
               "LOG_DIR", "DATA_DIR", "RUN_USER", "ENV_FILE",
               # 独立资源限额（AGENTS 6.11）：按 services[] 条目取值，未声明则由渲染器整行删除。
               "CPU_QUOTA", "MEMORY_MAX",
               # 装配单模板（声明的 env_template，如 edge-intel）额外允许的占位符：
               # 路径/身份来自声明生效值，部署期值（如 @PROVIDER_ENDPOINT@）必须由 --env-set 显式给。
               "SAFETY_POLICY", "VERSION_DIR", "RUNTIME_SUBJECT",
               "PROVIDER_ENDPOINT", "INTENT_MODEL_URL", "INTENT_MODEL_NAME",
               # 仿真 Provider 装配（2026-10-26）：后端配置由场景报告生成后注入；
               # 显示模式来自声明 provider_display；端口来自声明 services[].ports；
               # 绑定地址 / DISPLAY / XAUTHORITY 只能由部署期给。
               "BACKEND_CONFIG", "PROVIDER_DISPLAY_MODE", "PROVIDER_DISPLAY", "PROVIDER_XAUTHORITY",
               "PROVIDER_PORT", "PROVIDER_BIND")


def _unit_tokens(effective: dict, root: Path, *, python_bin: str, run_user: str) -> dict:
    return {
        "PROFILE": str(effective["id"]),
        "VERSION": str(effective["version"]),
        "PREFIX": str(under_root(root, str(effective["prefix"]))),
        "CURRENT_LINK": str(effective["current_link"]),
        "PYTHON_BIN": python_bin,
        "CONFIG_DIR": str(under_root(root, str(effective["config_dir"]))),
        "LOG_DIR": str(under_root(root, str(effective["log_dir"]))),
        "DATA_DIR": str(under_root(root, str(effective["data_dir"]))),
        "RUN_USER": run_user,
        "ENV_FILE": "%s.env" % effective["id"],
    }


def render_template(template: Path, tokens: dict) -> str:
    """按 @TOKEN@ 渲染模板；出现未登记的占位符即失败（防止模板悄悄多出一个没人填的 token）。"""
    if not template.is_file():
        raise InstallError("unit 模板不存在：%s" % template, EXIT_DECLARATION)
    text = template.read_text(encoding="utf-8")
    for key, value in tokens.items():
        text = text.replace("@%s@" % key, str(value))
    leftovers = sorted(set(re.findall(r"@([A-Z_]+)@", text)))
    if leftovers:
        raise InstallError(
            "模板 %s 含未知占位符 %s（渲染后仍残留）：要么没登记进 UNIT_TOKENS，要么部署期没给取值"
            "（后者用 --env-set KEY=VALUE 补上；注释里的 @大写占位符@ 也算）。已登记 token：%s"
            % (template, leftovers, list(UNIT_TOKENS)), EXIT_DECLARATION)
    return text


def render_env_file(effective: dict, root: Path, tokens: dict, *,
                    repo_root=None, env_values=None) -> str:
    """渲染运行装配单。

    两种来源（**同一事实只允许出现一处**）：
      · 角色声明了 `env_template` ⇒ 用模板渲染（占位符化，适合不同角色的装配形态差异，
        如 edge-intel 的远端 Provider 配置）；部署期值只能由 `env_values` 显式传入，
        缺必需占位符时 `render_template` 会显式失败（不写半个文件、不猜默认值）。
      · 未声明 ⇒ 沿用程序化生成（single-ubuntu2204 / sim-x86 走这条）。
    """
    template_rel = str(effective.get("env_template") or "").strip()
    if template_rel:
        if repo_root is None:
            raise InstallError("角色 %s 声明了 env_template 但未提供 repo_root，无法定位模板 %s"
                               % (effective.get("id"), template_rel), EXIT_DECLARATION)
        merged = dict(tokens)
        merged.update({
            "PROFILE": str(effective.get("robot_profile") or ""),
            "SAFETY_POLICY": str(effective.get("safety_policy") or ""),
            "VERSION_DIR": "%s/%s" % (tokens["PREFIX"], tokens["CURRENT_LINK"]),
            "RUNTIME_SUBJECT": str(effective.get("runtime_subject") or "iraf-runtime"),
        })
        # 仿真 Provider 装配（2026-10-26）：
        #   BACKEND_CONFIG 由安装期场景报告生成（effective["backend_config"]）——**只在非空时**注入，
        #   否则占位符残留会被 render_template 显式拒绝（绝不能落一条 `IRAF_BACKEND_CONFIG=` 空值：
        #   那会让服务"看起来起了、装配其实是空的"）。
        if str(effective.get("backend_config") or "").strip():
            merged["BACKEND_CONFIG"] = str(effective["backend_config"]).strip()
        # 显示模式来自声明；headless 模式下不要求部署期给 DISPLAY/XAUTHORITY（给了也不用）。
        display_mode = str(effective.get("provider_display") or "").strip().lower()
        if display_mode:
            merged.setdefault("PROVIDER_DISPLAY_MODE", display_mode)
        if display_mode in {"headless", "offscreen", "offscreen_frames"}:
            merged.setdefault("PROVIDER_DISPLAY", "")
            merged.setdefault("PROVIDER_XAUTHORITY", "")
        # 服务端口来自声明（services[].ports）——唯一来源，模板里不许再写一遍数字。
        declared_ports = [int(p) for item in effective.get("services") or [] for p in (item.get("ports") or [])]
        if len(declared_ports) == 1:
            merged.setdefault("PROVIDER_PORT", str(declared_ports[0]))
        merged.update({str(k).strip().upper(): str(v) for k, v in dict(env_values or {}).items()})
        template_path = Path(repo_root) / template_rel
        try:
            return render_template(template_path, merged)
        except InstallError as exc:
            # 把"缺哪个部署期取值"变成可操作信息：字段名 + 模板行号（部署时不必翻模板）。
            text = template_path.read_text(encoding="utf-8")
            leftovers = sorted(set(re.findall(r"@([A-Z_]+)@", text)) - set(merged))
            if not leftovers:
                raise
            lines = text.splitlines()
            detail = []
            for token in leftovers:
                where = [str(index + 1) for index, line in enumerate(lines) if "@%s@" % token in line]
                detail.append("%s（模板第 %s 行）" % (token, ",".join(where) or "?"))
            raise InstallError(
                "装配单模板缺部署期取值：%s。用 --env-set %s=… 或 --values <YAML> 提供"
                % ("；".join(detail), leftovers[0]), EXIT_DECLARATION) from exc

    lines = [ENV_FILE_HEADER % {"profile": effective["id"], "version": effective["version"],
                                "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds")}]
    prefix_current = "%s/%s" % (tokens["PREFIX"], tokens["CURRENT_LINK"])
    lines += [
        "# —— 运行期必需环境（与 src/iraf_adapters/bootstrap.py 同源；缺项即显式失败）——",
        # 未填的必填项**留空**并在上一行给 TODO 注释：值里不能出现 <…>（shell source 时会被当重定向，
        # 实测报 `未预期的记号 "newline"`）。空值 + 注释既安全又显式"尚未填写"。
        "# TODO(部署时必填)：机型 Profile 路径，例如 profiles/piper_mujoco.yaml",
        "IRAF_PROFILE=%s" % (effective.get("robot_profile") or ""),
        "# TODO(部署时必填)：安全策略路径，例如 profiles/safety/simulation_lab.yaml",
        "IRAF_SAFETY_POLICY=%s" % (effective.get("safety_policy") or ""),
        "IRAF_SKILL_ROOT=%s/skills" % prefix_current,
        "IRAF_BACKEND_ENTRYPOINT=%s" % effective["backend_entrypoint"],
        "# TODO(部署时必填)：由 scripts/emit_backend_config.py 生成（勿手写，手写副本会静默过期）",
        "IRAF_BACKEND_CONFIG=%s" % (effective.get("backend_config") or ""),
        "IRAF_EVENT_STORE=%s/iraf-runtime.db" % tokens["DATA_DIR"],
        "# —— 跨域/入口 ——",
        "IRAF_RUNTIME_SUBJECT=%s" % (effective.get("runtime_subject") or "iraf-runtime"),
    ]
    for item in effective["services"]:
        for port in (item.get("ports") or []):
            key = "IRAF_RUNTIME_PORT" if int(port) == 8765 else "IRAF_GRPC_PORT"
            lines.append("%s=%d" % (key, int(port)))
    lines.append("IRAF_PATH_ROOT=%s" % prefix_current)
    return "\n".join(lines) + "\n"


def precheck(effective: dict, root: Path, *, allow_unverified: bool, wheelhouse: Path | None,
             sdk_wheel: Path | None, python_exe: str, rehearsal: bool = False) -> list:
    """安装预检（fail-closed；每项都产出可读的实测值，便于写进证据）。

    `rehearsal`（演练）：只在**跨架构沙箱**里放宽"目标端事实"三项（架构 / OS / Python 标签 / glibc），
    其余检查（磁盘、端口、wheelhouse、SDK wheel、模板、未实测门禁）一律照旧。
    放宽项在报告里逐条标明 `relaxed=True`，且 CLI 层禁止在真实根（/）上开演练。
    """
    import platform
    import shutil as _shutil
    import sys as _sys

    checks = []

    def add(name, ok, detail, relaxed=False):
        checks.append({"name": name, "ok": bool(ok), "detail": detail, "relaxed": bool(relaxed)})

    target = {"arch": effective.get("target_arch"), "os": effective.get("target_os"),
              "os_version": effective.get("target_os_version"), "python_tag": effective.get("target_python_tag"),
              "glibc_min": effective.get("target_glibc_min"), "platform_tag": effective.get("target_platform_tag")}

    measured_arch = platform.machine()
    add("%s：目标架构" % effective["id"],
        rehearsal or target["arch"] in (None, "unverified", measured_arch),
        "声明=%s 实测=%s%s" % (target["arch"], measured_arch, "（演练放宽）" if rehearsal else ""),
        relaxed=rehearsal)

    if target["python_tag"] not in (None, "unverified"):
        expected = "cp%d%d" % (_sys.version_info[:2])
        add("%s：Python 标签" % effective["id"], rehearsal or expected == target["python_tag"],
            "声明=%s 实测=%s（%s）%s" % (target["python_tag"], expected, _sys.version.split()[0],
                                       "（演练放宽）" if rehearsal else ""), relaxed=rehearsal)

    if target["glibc_min"] not in (None, "unverified"):
        try:
            measured = os.confstr("CS_GNU_LIBC_VERSION").split()[1]
        except (ValueError, OSError):
            measured = "unverified"
        ok = measured != "unverified" and tuple(map(int, measured.split("."))) >= \
            tuple(map(int, str(target["glibc_min"]).split(".")))
        add("%s：glibc" % effective["id"], rehearsal or ok,
            "声明最低=%s 实测=%s%s" % (target["glibc_min"], measured, "（演练放宽）" if rehearsal else ""),
            relaxed=rehearsal)

    add("%s：解释器存在" % effective["id"],
        rehearsal or bool(python_exe and Path(python_exe).exists()),
        "python_bin=%s%s" % (python_exe, "（演练放宽：改用当前解释器）" if rehearsal else ""),
        relaxed=rehearsal)

    if str(target["os"]) not in ("unverified", "None") and target["os"]:
        os_release = under_root(root, "/etc/os-release")
        text = os_release.read_text(encoding="utf-8", errors="replace") if os_release.is_file() else ""
        got = ""
        for line in text.splitlines():
            if line.startswith("ID="):
                got = line.split("=", 1)[1].strip().strip('"')
        want = str(target["os"]).split()[0].lower()
        add("%s：OS" % effective["id"], rehearsal or got == want or not got,
            "声明=%s 实测=%s（%s）%s" % (target["os"], got or "unverified", os_release,
                                       "（演练放宽）" if rehearsal else ""), relaxed=rehearsal)

    # 磁盘余量：沙箱根可能**尚不存在**（演练时由安装过程创建）⇒ 用最近的已存在祖先测量，
    # 且任何 statvfs 失败都必须变成"一项失败的检查"，不能抛裸异常（实测踩到过 traceback）。
    probe_root = Path(root)
    while not probe_root.exists() and probe_root != probe_root.parent:
        probe_root = probe_root.parent
    try:
        free_mb = _shutil.disk_usage(str(probe_root)).free // (1024 * 1024)
        measured_ok = True
        measured_detail = "根=%s（实测于 %s）" % (root, probe_root)
    except OSError as exc:
        free_mb, measured_ok, measured_detail = 0, False, "无法测量（%s）：%s" % (probe_root, exc)
    need_mb = int(effective.get("min_free_mb") or 0)
    add("%s：磁盘余量" % effective["id"], measured_ok and free_mb >= need_mb,
        "需要≥%d MB 实测余量=%d MB %s" % (need_mb, free_mb, measured_detail))

    if effective.get("services"):
        add("%s：systemd 可用" % effective["id"], bool(_shutil.which("systemctl")),
            "systemctl=%s scope=%s" % (_shutil.which("systemctl") or "缺失", effective.get("systemd_scope")))

    unverified = []
    if str(effective.get("declaration_verification")) == "unverified" or \
            any(str(target[key]) == "unverified" for key in ("arch", "os", "os_version", "python_tag",
                                                            "platform_tag", "glibc_min")):
        unverified = [key for key in ("arch", "os", "os_version", "python_tag", "platform_tag", "glibc_min")
                      if str(target.get(key)) == "unverified"]
    add("%s：声明已实测（否则需 --allow-unverified）" % effective["id"],
        (not unverified) or allow_unverified,
        "未实测字段=%s 放行=%s（板卡声明回填见 profiles/boards/*.yaml）"
        % (unverified, allow_unverified))

    if effective.get("dependency_source", {}).get("kind") == "wheelhouse":
        manifest = (wheelhouse / "wheelhouse.json") if wheelhouse else None
        add("%s：离线 wheelhouse" % effective["id"],
            bool(wheelhouse and wheelhouse.is_dir() and manifest and manifest.is_file()),
            "目录=%s 清单=%s" % (wheelhouse, manifest))
        # 声明了 target_id 就做一致性校验：抓的包必须属于声明的构建目标（装错标签是"静默装错"）
        declared_target = effective.get("dependency_source", {}).get("target_id")
        if declared_target and manifest and manifest.is_file():
            try:
                got = json.loads(manifest.read_text(encoding="utf-8")).get("target")
            except json.JSONDecodeError:
                got = None
            got_id = got.get("id") if isinstance(got, dict) else got
            add("%s：wheelhouse 构建目标一致" % effective["id"],
                str(got_id) == str(declared_target),
                "声明=%s 清单=%s" % (declared_target, got_id))
    add("%s：SDK wheel 可读" % effective["id"], bool(sdk_wheel and Path(sdk_wheel).is_file()),
        str(sdk_wheel))

    listening = _listening_ports()
    conflicts = [int(port) for item in effective["services"] for port in (item.get("ports") or [])
                 if int(port) in listening]
    add("%s：端口空闲" % effective["id"], not conflicts,
        "冲突端口=%s（本机监听=%s）" % (conflicts, sorted(listening)[:12]))

    for item in effective["services"]:
        template = Path(str(effective["_repo_root"])) / str(item["template"])
        add("%s：模板可渲染（%s）" % (effective["id"], item["name"]), template.is_file(), str(template))
    return checks


def apply_install(effective: dict, root: Path, *, sdk_wheel: Path, runtime_bundle: Path | None = None,
                  wheelhouse: Path | None = None, python_exe: str | None = None,
                  allow_unverified: bool = False, dry_run: bool = False,
                  run_user: str | None = None, rehearsal: bool = False,
                  env_values: dict | None = None,
                  secrets_file: Path | None = None) -> dict:
    """按声明安装；**任何一步失败即回滚本次创建的内容**（只回滚本次，不碰既有安装）。"""
    import getpass
    import tarfile

    if rehearsal and Path(root).resolve() == Path("/").resolve():
        raise InstallError("拒绝在真实根（/）上开演练：--rehearsal 必须配 --root <沙箱目录>", EXIT_USAGE)
    if rehearsal and not allow_unverified:
        raise InstallError("演练会放宽目标端事实检查 ⇒ 必须同时给 --allow-unverified"
                           "（两者都只用于探索，不构成验收证据）", EXIT_USAGE)

    python_exe = python_exe or str(effective.get("python_bin") or sys.executable)
    # 解释器是**目标端事实**：声明里的 python_bin（如 /usr/local/bin/python3.10）在开发机上不存在，
    # 跨架构演练必须能跑 ⇒ 只在 --rehearsal 下回退到当前解释器，并在报告里标 relaxed。
    # 真实安装**绝不回退**：静默换解释器等于装到错误 ABI 上（铁律 3.2 的静默失效）。
    if not Path(python_exe).exists():
        if not rehearsal:
            raise InstallError("声明/指定的解释器不存在：%s（目标端事实；若为跨架构演练请加 --rehearsal）"
                               % python_exe, EXIT_DECLARATION)
        python_exe = sys.executable
    run_user = run_user or ("<current>" if str(effective.get("run_user")) == "@current@"
                            else str(effective.get("run_user") or "root"))
    if run_user == "<current>":
        try:
            run_user = getpass.getuser()
        except Exception:  # pragma: no cover - 无用户名环境
            run_user = "root"

    # 预检本身也要 fail-closed 且**不许抛裸异常**（实测踩到：沙箱根不存在时 statvfs 抛
    # FileNotFoundError，直接把 traceback 甩给使用者；那不是可操作的中文原因）
    try:
        checks = precheck(effective, root, allow_unverified=allow_unverified, wheelhouse=wheelhouse,
                          sdk_wheel=sdk_wheel, python_exe=python_exe, rehearsal=rehearsal)
    except Exception as exc:  # noqa: BLE001 - 预检任何意外都必须变成显式失败
        raise InstallError("预检本身出错（未写任何文件）：%s: %s" % (type(exc).__name__, exc),
                           EXIT_DECLARATION) from exc
    failed = [item for item in checks if not item["ok"]]
    if failed:
        raise InstallError("安装预检未通过（%d 项）：\n  - %s"
                           % (len(failed), "\n  - ".join("%s：%s" % (i["name"], i["detail"]) for i in failed)),
                           EXIT_DECLARATION)

    prefix = under_root(root, str(effective["prefix"]))
    config_dir = under_root(root, str(effective["config_dir"]))
    log_dir = under_root(root, str(effective["log_dir"]))
    data_dir = under_root(root, str(effective["data_dir"]))
    version_dir = prefix / str(effective["version"])
    current = prefix / str(effective["current_link"])
    tokens = _unit_tokens(effective, root, python_bin=python_exe, run_user=run_user)

    # ── 前置检查（**在写任何文件之前**）：目标端已有本角色的安装痕迹就拒绝。
    # 为什么必须前置：实测踩到过一次——安装写到 unit 那一层才发现"unit 已存在"，
    # 而回滚又删掉了上一轮留下的版本目录 ⇒ 留下"单元在、记录没了"的半状态，
    # 受控卸载因无记录而正确拒绝，只能人工清理。前置检查把这类破坏挡在写盘之前。
    traces = []
    if version_dir.exists():
        traces.append("版本目录已存在：%s" % version_dir)
    if current.is_symlink() or current.exists():
        traces.append("current 链接已存在：%s" % current)
    if record_path(effective, version_dir).is_file():
        traces.append("安装记录已存在：%s" % record_path(effective, version_dir))
    unit_dir_probe = unit_dir(root, str(effective["systemd_scope"]))
    for item in effective["services"]:
        candidate = unit_dir_probe / ("%s.service" % item["name"])
        if candidate.exists() or candidate.is_symlink():
            traces.append("systemd 单元已存在：%s" % candidate)
    if traces:
        raise InstallError(
            "目标端已有本角色的安装痕迹（%d 项），拒绝在其上覆盖安装：\n  - %s\n"
            "处置：先跑 `uninstall-iraf.sh --profile %s`（受控卸载，按安装记录删）；"
            "若因缺少安装记录而无法卸载（半状态），用 `uninstall-iraf.sh --profile %s --force-clean`"
            "（只删声明里列出的路径，逐项打印）。"
            % (len(traces), "\n  - ".join(traces), effective["id"], effective["id"]), EXIT_MISMATCH)

    plan = {
        "profile": effective["id"], "root": str(root), "version_dir": str(version_dir),
        "prefix": str(prefix), "python_exe": python_exe, "run_user": run_user,
        "install_method": effective.get("install_method"),
        "wheelhouse": str(wheelhouse) if wheelhouse else None,
        "sdk_wheel": str(sdk_wheel), "runtime_bundle": str(runtime_bundle) if runtime_bundle else None,
        "checks": checks,
        "steps": ["创建版本目录", "安装 SDK wheel（+ wheelhouse 依赖）",
                  "解 runtime bundle（若给）", "渲染 env 文件", "渲染 systemd 单元",
                  "切换 current 链接", "写安装记录", "后置自检（import iraf_sdk）"],
    }
    if dry_run:
        return {"dry_run": True, "will_do": plan}

    created_dirs, created_files, removed = [], [], []
    made_version_dir = not version_dir.exists()      # 只有本次创建的版本目录才允许被回滚删除
    previous_link_target = None
    if current.is_symlink():
        previous_link_target = os.readlink(str(current))

    def _mkdir(path: Path):
        if not path.exists():
            created_dirs.append(path)
        path.mkdir(parents=True, exist_ok=True)

    try:
        _mkdir(version_dir)
        lib_dir = version_dir / "lib"
        src_dir = version_dir / "src"
        _mkdir(lib_dir)
        pip_args = [python_exe, "-m", "pip", "install", "--disable-pip-version-check",
                    "--no-warn-script-location", "--target", str(lib_dir)]
        if effective.get("dependency_source", {}).get("kind") == "wheelhouse":
            pip_args += ["--no-index", "--find-links", str(wheelhouse)]
        else:
            index = effective.get("dependency_source", {}).get("index_url")
            if index:
                pip_args += ["--index-url", str(index)]
        runtime_deps = [str(item) for item in (effective.get("runtime_dependencies") or [])]
        pip_args += runtime_deps
        pip_args.append(str(sdk_wheel))
        info("安装 SDK wheel%s → %s（%s）"
             % (" + 运行时依赖 %s" % runtime_deps if runtime_deps else "",
                lib_dir, "离线 wheelhouse" if wheelhouse else "索引"))
        proc = subprocess.run(pip_args, capture_output=True, text=True, timeout=1800)
        if proc.returncode != 0:
            tail = [line for line in (proc.stderr or "").strip().splitlines() if line][-3:]
            raise InstallError("pip 安装失败（%d）：%s" % (proc.returncode, tail), EXIT_IO)
        plan["pip_returncode"] = proc.returncode

        if runtime_bundle:
            bundle = Path(runtime_bundle)
            if not bundle.is_file():
                raise InstallError("runtime bundle 不存在：%s" % bundle, EXIT_IO)
            # 解到**版本目录根**（不是 src/）：真实 runtime 包内含 src/ config/ skills/ profiles/ 等多顶层，
            # 若解到 src/ 会变成 src/src/…（实测的错配）。含单一顶层目录时剥掉该层。
            with tarfile.open(bundle, "r:gz") as tar:
                members = tar.getmembers()
                top = {item.name.split("/")[0] for item in members}
                if len(top) == 1:
                    # 剥掉单一顶层目录：必须**改写 member.name**，只过滤列表不会改变解包路径
                    # （实测踩到：过滤后仍解成 <version>/<top>/src/… ⇒ 多一层目录）
                    prefix_name = top.pop() + "/"
                    stripped = []
                    for item in members:
                        if item.name == prefix_name.rstrip("/"):
                            continue
                        if not item.name.startswith(prefix_name):
                            continue
                        item.name = item.name[len(prefix_name):]
                        stripped.append(item)
                    members = stripped
                tar.extractall(path=str(version_dir), members=members,
                               filter="data" if hasattr(tarfile, "data_filter") else None)
            plan["runtime_extracted_to"] = str(version_dir)

        stubs_dir = effective.get("python_stubs_dir")
        if stubs_dir:
            source = Path(str(effective["_repo_root"])) / str(stubs_dir)
            if not source.is_dir():
                raise InstallError("声明的 proto 桩目录不存在：%s" % source, EXIT_IO)
            copied = 0
            for item in source.rglob("*.py"):
                rel = item.relative_to(source)
                target = lib_dir / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, target)
                copied += 1
            plan["stubs_copied"] = copied
            info("proto 桩已拷入 lib/：%d 个（来自 %s）" % (copied, source))
            probe_ok = (lib_dir / "iraf" / "v1").is_dir()
            if not probe_ok:
                raise InstallError("桩目录结构异常：%s 下没有 iraf/v1/" % source, EXIT_IO)

        # 仿真档：把声明的场景（模型 XML + 同名旁挂报告）复制进版本目录，并在**目标端**生成
        # IRAF_BACKEND_CONFIG（铁律：这份 JSON 一律由场景报告生成，不手写 —— 手写副本已实测静默过期两次）。
        sim_scene = effective.get("sim_scene")
        if sim_scene and not effective.get("backend_config"):
            source_xml = Path(str(effective["_repo_root"])) / str(sim_scene)
            source_json = source_xml.with_suffix(".json")
            if not source_xml.is_file() or not source_json.is_file():
                raise InstallError("声明的仿真场景缺文件：%s（需同时存在同名 .json 旁挂报告）"
                                   % source_xml, EXIT_IO)
            models_dir = version_dir / "models"
            models_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_xml, models_dir / source_xml.name)
            shutil.copy2(source_json, models_dir / source_json.name)
            # 场景里的 mesh 是**相对路径**（形如 ../../../piper_ros/.../meshes/link1.STL）：直接拷贝
            # 进版本目录后 MuJoCo 找不到 STL（实装报 `Error opening file '...link1.STL'`，服务起不来）。
            # 处置：把被引用的 mesh 目录**一起装进** <version>/models/meshes，并把引用改写成 meshes/<name>，
            # 使安装物自包含（不依赖仓库 checkout 或构建机的邻居目录）。
            xml_target = models_dir / source_xml.name
            xml_text = xml_target.read_text(encoding="utf-8")
            mesh_prefixes = sorted({ref.rsplit("/meshes/", 1)[0] + "/meshes/"
                                    for ref in re.findall(r'file="([^"]+)"', xml_text) if "/meshes/" in ref})
            for prefix in mesh_prefixes:
                if not prefix.startswith(".."):
                    continue   # 已经是可用的相对/绝对路径，不改写
                src_mesh_dir = (source_xml.parent / prefix).resolve()
                if not src_mesh_dir.is_dir():
                    raise InstallError("场景引用的 mesh 目录不存在：%s（场景=%s）"
                                       % (src_mesh_dir, source_xml), EXIT_IO)
                dst_mesh_dir = models_dir / "meshes"
                copied_meshes = 0
                for item in src_mesh_dir.rglob("*"):
                    if not item.is_file():
                        continue
                    target = dst_mesh_dir / item.relative_to(src_mesh_dir)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(item, target)
                    copied_meshes += 1
                xml_text = xml_text.replace(prefix, "meshes/")
                plan.setdefault("scene_meshes", []).append(
                    {"from": str(src_mesh_dir), "files": copied_meshes})
                info("场景 mesh 已随包：%d 个（来自 %s，引用改写为 meshes/）"
                     % (copied_meshes, src_mesh_dir))
            xml_target.write_text(xml_text, encoding="utf-8")
            emitter = Path(str(effective["_repo_root"])) / "scripts" / "emit_backend_config.py"
            if not emitter.is_file():
                raise InstallError("缺少 %s（生成 IRAF_BACKEND_CONFIG 的唯一入口）" % emitter, EXIT_IO)
            proc_emit = subprocess.run(
                [python_exe, str(emitter), "--scene", str(models_dir / source_xml.name),
                 "--compact", "--root", str(effective["_repo_root"])],
                capture_output=True, text=True, timeout=300)
            if proc_emit.returncode != 0:
                raise InstallError("生成 IRAF_BACKEND_CONFIG 失败（%d）：%s"
                                   % (proc_emit.returncode, (proc_emit.stderr or "").strip()[-300:]),
                                   EXIT_IO)
            config_text = proc_emit.stdout.strip()
            try:
                json.loads(config_text)
            except json.JSONDecodeError as exc:
                raise InstallError("生成的 IRAF_BACKEND_CONFIG 不是合法 JSON（%s）" % exc, EXIT_IO)
            effective["backend_config"] = config_text
            plan["backend_config_from"] = str(models_dir / source_xml.name)
            plan["backend_config_bytes"] = len(config_text)
            info("IRAF_BACKEND_CONFIG 已由场景报告生成：%d 字节（场景=%s）"
                 % (len(config_text), source_xml.name))

        env_path = config_dir / ("%s.env" % effective["id"])
        _mkdir(config_dir)
        env_path.write_text(render_env_file(effective, root, tokens,
                                            repo_root=effective.get("_repo_root"),
                                            env_values=env_values), encoding="utf-8")
        created_files.append(env_path)
        plan["env_file"] = str(env_path)

        # 凭据单独落地（2026-10-22：板上 runtime 因缺凭据永远起不来）。
        # 装配单按设计**不写凭据**（不进 git / 不进命令行），所以由部署期给一份 600 文件，
        # 这里只做"拷成 600 + 登记进安装记录"，卸载时随记录删除（不留残留凭据）。
        # 为什么是**多键容器**而不是单个 token：板上 runtime 同时需要
        #   IRAF_PROVIDER_TOKEN（出站调 x86 Capability Provider）
        #   IRAF_GRPC_TOKEN    （本域 gRPC 服务端入站鉴权）
        # 实测踩到：只给前者时单元仍 exit 2 `IRAF_GRPC_TOKEN must be provided by deployment`。
        secrets_written = None
        if secrets_file is not None:
            src = Path(secrets_file)
            if not src.is_file():
                raise InstallError("--secrets-file 指定的文件不存在：%s" % src, EXIT_IO)
            allowed = ("IRAF_PROVIDER_TOKEN", "IRAF_GRPC_TOKEN", "IRAF_AGENTOS_TOKEN")
            lines, seen = [], []
            for line in src.read_text(encoding="utf-8", errors="replace").splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    raise InstallError("--secrets-file 每行必须是 KEY=VALUE（或注释）：%r" % line, EXIT_IO)
                key = line.split("=", 1)[0].strip()
                if key not in allowed:
                    raise InstallError("--secrets-file 里出现未登记的键 %r（允许：%s）"
                                       % (key, ", ".join(allowed)), EXIT_IO)
                lines.append(line)
                seen.append(key)
            if not lines:
                raise InstallError("--secrets-file 没有任何有效键：%s（允许：%s）"
                                   % (src, ", ".join(allowed)), EXIT_IO)
            secrets_path = config_dir / "runtime.secrets"
            secrets_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            try:
                secrets_path.chmod(0o600)
            except OSError as exc:  # 权限设不上就必须显式失败（凭据不能被别人读）
                raise InstallError("runtime.secrets 权限设置失败（%s）：%s" % (secrets_path, exc), EXIT_IO)
            created_files.append(secrets_path)
            secrets_written = secrets_path
            plan["secrets_file"] = str(secrets_path)
            info("凭据已落地：%s（600，键=%s；值不打印）" % (secrets_path, ",".join(seen)))
        elif any(str(s.get("name") or "").startswith("iraf-runtime") for s in effective["services"]):
            # 不 fail（x86 开发档可能用本地 backend，不需要跨机凭据），但必须显式提示，
            # 否则就是 2026-10-22 那种"装完才发现 runtime 起不来"的静默状态。
            print("[install][警告] 未提供 --secrets-file：%s/runtime.secrets 不存在，"
                  "runtime 会因缺 IRAF_PROVIDER_TOKEN / IRAF_GRPC_TOKEN fail-closed（起不来）"
                  % config_dir, file=sys.stderr)

        unit_dir_path = unit_dir(root, str(effective["systemd_scope"]))
        if not unit_dir_path.exists():
            created_dirs.append(unit_dir_path)
        rendered_units = []
        for item in effective["services"]:
            template = Path(str(effective["_repo_root"])) / str(item["template"])
            svc_tokens = dict(tokens)
            svc_tokens["CPU_QUOTA"] = str(item.get("cpu_quota") or "")
            svc_tokens["MEMORY_MAX"] = str(item.get("memory_max") or "")
            content = render_template(template, svc_tokens)
            # 未声明限额的角色（x86 开发档）⇒ 整行删除。不能留着 "CPUQuota=" 空值：
            # systemd 对空值要么报错要么静默忽略，两者都会让"限额是否存在"变得不可判定。
            for token, directive in (("CPU_QUOTA", "CPUQuota"), ("MEMORY_MAX", "MemoryMax")):
                if not svc_tokens[token]:
                    kept = [line for line in content.splitlines()
                            if not line.startswith(directive + "=")]
                    content = "\n".join(kept) + "\n"
            # user 作用域不得有 User=/Group=：用户级 systemd 无权切换身份，实装会以
            # `status=216/GROUP Failed to determine supplementary groups` 直接失败（2026-10-26 实测）。
            if str(effective["systemd_scope"]) != "system":
                kept = [line for line in content.splitlines()
                        if not line.startswith(("User=", "Group="))]
                content = "\n".join(kept) + "\n"
            _mkdir(unit_dir_path)
            unit_path = unit_dir_path / ("%s.service" % item["name"])
            if unit_path.exists():
                raise InstallError("unit 已存在，拒绝覆盖（请先走受控卸载/变更流程）：%s" % unit_path,
                                   EXIT_MISMATCH)
            unit_path.write_text(content, encoding="utf-8")
            created_files.append(unit_path)
            rendered_units.append(str(unit_path))
        plan["units"] = rendered_units

        if current.is_symlink() or current.exists():
            current.unlink()
        current.symlink_to(version_dir.name)
        plan["current_link"] = "%s -> %s" % (current, version_dir.name)

        for directory in (log_dir, data_dir):
            if not directory.exists():
                created_dirs.append(directory)
            directory.mkdir(parents=True, exist_ok=True)

        record = write_record(effective, version_dir, note="install-iraf.sh",
                              config_paths=[env_path] + ([secrets_written] if secrets_written else []),
                              created_dirs=created_dirs)
        plan["record"] = str(record)

        probe = subprocess.run([python_exe, "-c",
                                "import iraf_sdk; import iraf.v1.skill_pb2; print('iraf_sdk-ok+stubs')"
                                if effective.get("python_stubs_dir") else
                                "import iraf_sdk; print('iraf_sdk-ok')"],
                               capture_output=True, text=True,
                               env={**os.environ, "PYTHONPATH": "%s:%s" % (lib_dir, src_dir)},
                               timeout=120)
        plan["self_check"] = {"returncode": probe.returncode,
                              "stdout": (probe.stdout or "").strip().splitlines()[-1:] or [],
                              "stderr_tail": (probe.stderr or "").strip().splitlines()[-2:]}
        if probe.returncode != 0:
            raise InstallError("后置自检失败：import iraf_sdk 不通过（%s）"
                               % plan["self_check"]["stderr_tail"], EXIT_SERVICE)
        plan["ok"] = True
        return plan
    except Exception as exc:  # 回滚：只删本次创建的内容，并还原链接
        info("安装失败，开始回滚本次创建的内容（%d 个目录 / %d 个文件）"
             % (len(created_dirs), len(created_files)))
        for path in reversed(created_files):
            try:
                path.unlink()
                removed.append(str(path))
            except OSError:
                pass
        if made_version_dir and version_dir.is_dir():
            # 只删本次创建的版本目录：预先存在的目录不属于本次改动，删掉会造成半状态
            try:
                shutil.rmtree(version_dir)
                removed.append(str(version_dir))
            except OSError:
                pass
        elif version_dir.is_dir():
            info("保留预先存在的版本目录（不属于本次改动）：%s" % version_dir)
        if current.is_symlink() or current.exists():
            try:
                current.unlink()
            except OSError:
                pass
        if previous_link_target:
            try:
                current.symlink_to(previous_link_target)
                removed.append("已还原 current 链接 → %s" % previous_link_target)
            except OSError:
                pass
        for path in reversed(created_dirs):
            try:
                if path.is_dir() and not any(path.iterdir()):
                    path.rmdir()
                    removed.append(str(path) + "（空目录）")
            except OSError:
                pass
        plan["ok"] = False
        plan["rolled_back"] = removed
        if isinstance(exc, InstallError):
            raise
        raise InstallError("安装失败并已回滚：%s" % exc, EXIT_IO) from exc
    return plan


# ─────────────────────────── 命令行 ───────────────────────────

def _print_json(payload) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))


def main(argv=None) -> int:
    # --repo-root 同时挂在主解析器与各子命令上（parents=common）：这样 shell 入口既可以
    # 在子命令前传，也可以直接传给子命令，不会出现 "unrecognized arguments"。
    # 默认值优先取环境变量 IRAF_REPO_ROOT（由 shell 入口导出），避免入口与调用方重复传参。
    default_root = Path(os.environ.get("IRAF_REPO_ROOT") or REPO_ROOT)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--repo-root", type=Path, default=default_root)

    parser = argparse.ArgumentParser(description="IRAF 安装记录 / 残留核验 / 受控卸载（实现层）",
                                     parents=[common])
    sub = parser.add_subparsers(dest="command", required=True)
    p_layout = sub.add_parser("layout", help="打印某角色的生效布局（含解析后的路径）", parents=[common])
    p_layout.add_argument("--profile", required=True)

    p_res = sub.add_parser("residue", help="六路残留核验（只读）", parents=[common])
    p_res.add_argument("--profile", required=True)
    p_res.add_argument("--root", type=Path, default=Path("/"), help="文件系统根（沙箱演练给沙箱目录）")
    p_res.add_argument("--installed-dir", type=Path, default=None, help="直接指定已装版本目录")
    p_res.add_argument("--json", action="store_true")

    p_un = sub.add_parser("uninstall", help="受控卸载（只按安装记录删）", parents=[common])
    p_un.add_argument("--profile", required=True)
    p_un.add_argument("--root", type=Path, default=Path("/"))
    p_un.add_argument("--installed-dir", type=Path, default=None)
    p_un.add_argument("--dry-run", action="store_true", help="只列计划，不删任何文件")
    p_un.add_argument("--purge", action="store_true", help="连数据与日志一起删（默认保留）")
    p_un.add_argument("--force-clean", action="store_true",
                      help="无安装记录时的兜底清理：只删声明里列出的路径（前缀/unit/env），逐项打印；"
                           "用于有痕迹但安装记录缺失的半状态")
    p_un.add_argument("--json", action="store_true")

    p_rw = sub.add_parser("record-write", help="把已装版本目录写成安装记录（安装的最后一步）",
                          parents=[common])
    p_rw.add_argument("--profile", required=True)
    p_rw.add_argument("--installed-dir", type=Path, required=True)
    p_rw.add_argument("--config-file", type=Path, action="append", default=[],
                      help="版本目录之外的已装文件（如 env 文件）；可重复。漏记 ⇒ 卸载后残留")
    p_rw.add_argument("--created-dir", type=Path, action="append", default=[],
                      help="本次安装新建的目录（变空才删）；可重复。漏记 ⇒ 卸载后留空壳")
    p_rw.add_argument("--note", default="")

    p_ins = sub.add_parser("install", help="按声明安装（预检 → 版本化目录 → 渲染 → 写记录 → 自检；失败回滚）",
                           parents=[common])
    p_ins.add_argument("--profile", required=True)
    p_ins.add_argument("--root", type=Path, default=Path("/"))
    p_ins.add_argument("--sdk-wheel", type=Path, required=True, help="SDK wheel（由 build_sdk.sh 产出）")
    p_ins.add_argument("--runtime-bundle", type=Path, default=None, help="runtime 源码包（可选）")
    p_ins.add_argument("--wheelhouse", type=Path, default=None, help="离线 wheelhouse 目录（依赖来源=wheelhouse 时必填）")
    p_ins.add_argument("--python", dest="python_exe", default=None, help="解释器（默认取声明的 python_bin）")
    p_ins.add_argument("--run-user", default=None, help="单元里的运行用户（默认取声明的 run_user）")
    p_ins.add_argument("--allow-unverified", action="store_true",
                       help="显式放行\"声明未实测\"（板卡声明回填前只能这样探索，不构成验收证据）")
    p_ins.add_argument("--stubs-dir", type=Path, default=None,
                       help="覆盖声明的 python_stubs_dir（演练或非常规交付用；默认按声明）")
    p_ins.add_argument("--rehearsal", action="store_true",
                       help="沙箱演练：放宽目标端事实检查（架构/OS/Python 标签/glibc）；必须配 --root 沙箱")
    p_ins.add_argument("--env-set", action="append", default=[], metavar="KEY=VALUE",
                       help="装配单模板的部署期取值（可多次，如 --env-set PROVIDER_ENDPOINT=host:50052）；"
                            "缺必需占位符即失败，不猜默认值")
    p_ins.add_argument("--secrets-file", type=Path, default=None, metavar="PATH",
                       help="运行期凭据文件（600；每行 KEY=VALUE，允许 IRAF_PROVIDER_TOKEN / "
                            "IRAF_GRPC_TOKEN / IRAF_AGENTOS_TOKEN）；落地为 <config_dir>/runtime.secrets "
                            "并登记进安装记录（卸载即删）。凭据不进装配单、不进命令行值，只能走文件")
    p_ins.add_argument("--dry-run", action="store_true", help="只跑预检并打印计划，不写任何文件")
    p_ins.add_argument("--json", action="store_true")

    p_rs = sub.add_parser("record-show", help="打印安装记录摘要", parents=[common])
    p_rs.add_argument("--profile", required=True)
    p_rs.add_argument("--installed-dir", type=Path, required=True)

    args = parser.parse_args(argv)
    try:
        effective = effective_layout(args.repo_root, args.profile)
    except InstallError as exc:
        print("[install][错误] %s" % exc, file=sys.stderr)
        return exc.code

    try:
        if args.command == "layout":
            _print_json(effective)
            return EXIT_OK

        if args.command == "record-write":
            write_record(effective, args.installed_dir, note=args.note,
                         config_paths=list(args.config_file or []),
                         created_dirs=list(args.created_dir or []))
            return EXIT_OK

        if args.command == "install":
            if args.stubs_dir is not None:
                effective["python_stubs_dir"] = str(args.stubs_dir)
            env_values = {}
            for item in (args.env_set or []):
                if "=" not in item:
                    print("[install][错误] --env-set 需要 KEY=VALUE 形式，实际：%r" % item, file=sys.stderr)
                    return EXIT_USAGE
                key, _, value = item.partition("=")
                env_values[key.strip().upper()] = value.strip()
            plan = apply_install(effective, args.root, sdk_wheel=args.sdk_wheel,
                                 runtime_bundle=args.runtime_bundle, wheelhouse=args.wheelhouse,
                                 python_exe=args.python_exe, allow_unverified=args.allow_unverified,
                                 dry_run=args.dry_run, run_user=args.run_user,
                                 rehearsal=args.rehearsal, env_values=env_values,
                                 secrets_file=args.secrets_file)
            if args.json:
                _print_json(plan)
            elif args.dry_run:
                info("演练：预检 %d 项全过，未写任何文件" % len(plan["will_do"]["checks"]))
                for name, value in plan["will_do"].items():
                    if name != "checks":
                        print("  %-20s %s" % (name, value))
            else:
                info("安装完成：%s" % plan.get("version_dir"))
                print("  env 文件   %s" % plan.get("env_file"))
                print("  单元       %s" % ", ".join(plan.get("units") or []))
                print("  current    %s" % plan.get("current_link"))
                print("  安装记录   %s" % plan.get("record"))
                print("  自检       %s" % plan.get("self_check", {}).get("stdout"))
                info("下一步：bash deploy/sdk/verify_residue.sh 会报'不干净'（正常，已装即有内容）；"
                     "要确认可干净清除，请跑 deploy/sdk/rehearse_install_chain.sh 或走受控卸载")
            return EXIT_OK

        if args.command == "record-show":
            record = read_record(effective, args.installed_dir)
            _print_json({key: record[key] for key in
                         ("schema_version", "profile", "version", "installed_at_utc",
                          "version_dir", "units", "ports", "file_count", "note")})
            return EXIT_OK

        if args.command == "residue":
            # 与 uninstall 的区别：残留核验**必须能在"什么都没装"时正常工作** —— 卸载之后正是
            # 最主要的用法。因此这里解析不到已装版本不算错误，只把 record 记为 None。
            version_dir = args.installed_dir
            if version_dir is None:
                try:
                    version_dir = resolve_version_dir(effective, args.root)
                except InstallError as exc:
                    info("未发现已装版本（%s）⇒ 只扫描声明根下的残留" % exc)
                    version_dir = None
            record = None
            if version_dir is not None:
                try:
                    record = read_record(effective, version_dir)
                except InstallError as exc:
                    if exc.code != EXIT_MISMATCH:
                        raise
                    info("未找到安装记录（%s）⇒ 只报告根下剩余内容，不做记录比对" % exc)
            findings = residue_scan(effective, args.root, record=record)
            clean = residue_is_clean(findings)
            report = {"profile": args.profile, "root": str(args.root), "version_dir": str(version_dir),
                      "record_found": record is not None, "clean": clean, "findings": findings}
            if args.json:
                _print_json(report)
            else:
                info("残留核验：角色=%s 根=%s" % (args.profile, args.root))
                for key in ("files", "dirs", "env_files", "ports", "processes", "unlisted", "missing"):
                    items = findings.get(key) or []
                    print("  %-10s %d %s" % (key, len(items), items[:5] if items else ""))
                for name, state in findings["units"].items():
                    print("  unit       %s exists=%s enabled=%s"
                          % (name, state.get("exists"), state.get("enabled_link")))
                print("结果：%s" % ("干净（六路全 0）" if clean else "**不干净**（见上面非 0 项）"))
            return EXIT_OK if clean else EXIT_MISMATCH

        if args.command == "uninstall":
            try:
                version_dir = args.installed_dir or resolve_version_dir(effective, args.root)
                record = read_record(effective, version_dir)
            except InstallError as exc:
                if not getattr(args, "force_clean", False):
                    raise
                info("受控卸载不可用（%s）⇒ 走 --force-clean 兜底清理" % exc)
                plan = plan_force_clean(effective, args.root, purge=args.purge)
                if not plan["delete"]:
                    info("兜底清理：声明路径下没有可清理的内容 ⇒ 无需操作")
                    return EXIT_OK
                if args.dry_run:
                    info("演练：兜底清理将删除 %d 项（只删声明路径，不删任何文件）" % len(plan["delete"]))
                    for item in plan["delete"]:
                        print("  - %s" % item)
                    return EXIT_OK
                removed = apply_uninstall(plan, root=args.root)
                info("兜底清理完成：删除 %d 项（保留 %d 项）" % (len(removed), len(plan["keep"])))
                for item in plan["keep"]:
                    print("  = 保留 %s" % item)
                return EXIT_OK
            plan = plan_uninstall(effective, args.root, record, purge=args.purge)
            if args.dry_run:
                report = {"profile": args.profile, "dry_run": True, "version_dir": str(version_dir),
                          "delete_count": len(plan["delete"]),
                          "delete": [str(item) for item in plan["delete"]],
                          "keep": [str(item) for item in plan["keep"]]}
                if args.json:
                    _print_json(report)
                else:
                    info("演练：将删除 %d 项（不删任何文件）" % len(plan["delete"]))
                    for item in plan["delete"]:
                        print("  - %s" % item)
                    for item in plan["keep"]:
                        print("  = 保留 %s" % item)
                return EXIT_OK
            removed = apply_uninstall(plan, root=args.root)
            info("已删除 %d 项（保留 %d 项）" % (len(removed), len(plan["keep"])))
            for item in plan["keep"]:
                print("  = 保留 %s" % item)
            return EXIT_OK
    except InstallError as exc:
        print("[install][错误] %s" % exc, file=sys.stderr)
        return exc.code

    print("[install][错误] 未知子命令", file=sys.stderr)
    return EXIT_USAGE


if __name__ == "__main__":
    raise SystemExit(main())
