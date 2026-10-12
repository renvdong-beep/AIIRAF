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
    effective["id"] = profile_id
    effective["label"] = profile.get("label")
    effective["version"] = checker.read_toml_field(repo_root, declaration["sdk"]["version_ref"])
    effective["dist_name"] = checker.read_toml_field(repo_root, declaration["sdk"]["dist_name_ref"])
    effective["components"] = list(profile["components"])
    effective["backend_entrypoint"] = profile["backend_entrypoint"]
    effective["services"] = list(profile["services"])
    effective["simulation_only"] = bool(profile.get("simulation_only"))
    effective["purge_flag"] = declaration["defaults"]["uninstall"]["purge_flag"]
    return effective


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
    prefix = Path(str(root) + str(effective["prefix"]))
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
    system_dir = Path(str(root) + ("/etc/systemd/system" if scope == "system"
                                   else "/etc/systemd/user"))
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
    roots = [Path(str(root) + str(item)) for item in effective["residue_scan_roots"]]
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
    config_dir = Path(str(root) + str(effective["config_dir"]))
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
    keep_dirs = [Path(str(root) + str(item)) for item in effective["uninstall_keep"]]
    allowed_roots = [Path(str(root) + str(effective["prefix"])),
                     Path(str(root) + str(effective["config_dir"]))]
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
    system_dir = Path(str(root) + ("/etc/systemd/system" if str(effective["systemd_scope"]) == "system"
                                   else "/etc/systemd/user"))
    for unit in record.get("units", []):
        for candidate in (system_dir / unit, system_dir / "multi-user.target.wants" / unit):
            if candidate.exists() or candidate.is_symlink():
                delete.append(candidate)
    link = Path(str(root) + str(effective["prefix"]) + "/" + str(effective["current_link"]))
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
    stop = Path(str(root) + str(plan["prefix"])).resolve()
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
            version_dir = args.installed_dir or resolve_version_dir(effective, args.root)
            record = read_record(effective, version_dir)
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
