#!/usr/bin/env python3
"""目标端自检（verify.sh）与受控卸载（uninstall.sh）的实现层（步骤 09）。

两个 shell 入口只做参数解析与退出码传递，逻辑全在这里：

    bash deploy/sdk/verify.sh --bundle build/sdk/iraf-board-e300-*.tar.gz --dry-run [--json-out <p>]
    bash deploy/sdk/verify.sh --root <沙箱或 />            # 安装后置校验（install.sh 调用同一入口）
    bash deploy/sdk/uninstall.sh --root <沙箱或 /> [--version <v>] [--dry-run] [--json-out <p>]

也可直接当 CLI 用：

    python3 deploy/sdk/lib_target_verify.py verify    --bundle <tar.gz> --dry-run
    python3 deploy/sdk/lib_target_verify.py uninstall --root <dir> --dry-run

设计要点（AGENTS.md 铁律 2.2/2.5/5.2、设计 §5/§9）：

- **声明单一来源**：目标端端点、超时、事件库路径、安装记录名全部来自 `config/sdk/package_matrix.yaml`
  的 `delivery.health` / `delivery.install`；缺声明即显式失败（退出码 2），脚本里不写死任何
  URL、端口或路径（铁律 5.3）。
- **服务未启动必须显式报告**：`/health`、gRPC、事件库三项探针拿不到结果时写入
  `state=SERVICE_NOT_RUNNING` 且 `verified=false`；**非演练**一律退出 5，禁止把"服务没起来"
  当成通过（步骤 09 边界要求）。
- **演练 ≠ 目标端证据**：演练只证明校验和/契约/探针解析逻辑可跑通；证据里
  `evidence_scope=rehearsal_only`、`simulation=true`，目标端 `/health`、systemd 与真实事件库写入
  一律记 `DEFERRED`（x86-first 战役，板卡不在场）。stub/loopback 只算解析逻辑证据。
- **只动本 bundle**：卸载前必须先通过"安装记录 ↔ 实际树"逐文件比对——清单外文件 → 退出 1、
  记录内文件缺失 → 退出 1、SHA-256 不符 → 退出 4；日志目录与事件库属数据，卸载保留。
- **已装版本目录靠"脚本自身位置"判定**，不做全盘搜索：verify.sh 在
  `<version_dir>/scripts/verify.sh` 时自动带上 `--installed-dir <version_dir>`，避免在
  安装暂存目录与已装目录之间猜。
- **stdout 契约**：stdout 只有一份 JSON（自检证据 / 卸载报告），日志一律走 stderr。

退出码（设计 §5 全脚本统一约定）：

    0 成功 / 演练完成      1 参数或卸载范围错误（清单外文件等）
    2 声明缺失、契约不符     4 校验失败（SHA-256 不符、成员被篡改、已装树漂移）
    5 后置判定失败（服务未启动、探针不通过、事件库不可写）
"""

from __future__ import annotations

import importlib.util
import json
import os
import shutil
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from lib_manifest import (  # noqa: E402 - 同一目录的实现层，复用同一份契约
    ExitCode,
    ManifestError,
    load_matrix,
    sha256_file,
)
from lib_board_bundle import (  # noqa: E402 - 复用 bundle 契约/解包/板卡门禁，不重复实现
    MATRIX_REL,
    MATRIX_SCHEMA_REL,
    Installer,
    run_profile_check,
    verify_bundle_contract,
)

USAGE = __doc__

# --- 契约常量：只在这里声明一次 ------------------------------------------------

VERIFY_EVIDENCE_SCHEMA_VERSION = "iraf.target-verify-evidence/v1"
UNINSTALL_REPORT_SCHEMA_VERSION = "iraf.board-uninstall-report/v1"
INSTALL_RECORD_SCHEMA_VERSION = "iraf.sdk-install-record/v1"

#: 后置判定失败（服务未启动 / 探针不通过 / 事件库不可写）。
EXIT_POST_CHECK = 5
#: 探针状态：服务未启动（**不是**通过，也不是契约层失败）。
SERVICE_NOT_RUNNING = "SERVICE_NOT_RUNNING"
STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_DEFERRED = "DEFERRED"
STATUS_SKIPPED = "SKIPPED"
STATUS_REHEARSAL = "REHEARSAL_ONLY"
#: 事件库写入探针的表名（与运行时业务表隔离，探针失败不污染执行记录）。
PROBE_TABLE = "iraf_verify_probe"
#: gRPC 探针分层标记：TCP 可达 ≠ gRPC 健康检查通过，必须如实标注。
GRPC_PROBE_HEALTH_SERVICE = "grpc_health_v1"
GRPC_PROBE_CHANNEL = "grpc_channel_ready"
GRPC_PROBE_TCP = "tcp_connect"

REQUIRED_HEALTH_KEYS = (
    "http_url",
    "expect_http_status",
    "grpc_target",
    "timeout_s",
    "event_store_env",
    "event_store_default",
)
REQUIRED_INSTALL_KEYS = (
    "prefix",
    "config_dir",
    "log_dir",
    "systemd_dir",
    "service_unit",
    "env_file",
    "current_link",
    "python_bin",
    "file_record",
)

NOTES_ALWAYS = (
    "本机是 x86_64 开发端、目标板卡不在场（x86-first 战役）：目标端 /health、systemd 与真实"
    "事件库写入验收 DEFERRED，任何演练结果都不得表述为\"板卡可用\"。",
    "探针只证明探针与解析逻辑可跑通；stub/loopback 结果不构成运行时健康证据。",
)

__all__ = [
    "TargetVerifyError",
    "collect_tree_files",
    "compare_record",
    "event_store_probe",
    "grpc_probe",
    "http_probe",
    "load_verify_declaration",
    "read_install_record",
    "resolve_installed",
    "run_uninstall",
    "run_verify",
    "write_install_record",
    "main",
]


class TargetVerifyError(ManifestError):
    """带退出码的中文可操作错误（沿用同一套退出码语义）。"""


def info(message: str) -> None:
    print(f"[target_verify] {message}", file=sys.stderr, flush=True)


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _abspath(base: Path, value) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else base / path


def _under_root(root: Path, declared: str) -> Path:
    """把声明的绝对目标端路径重挂到 `--root`（沙箱演练与真实执行共用同一段代码）。"""
    return root / str(declared).lstrip("/")


def _rel_to_root(root: Path, declared: str) -> str:
    """记录里存相对 root 的 POSIX 路径（不含本机绝对路径，便于审计与迁移）。"""
    path = _under_root(root, declared)
    try:
        return path.relative_to(root).as_posix()
    except ValueError as exc:  # pragma: no cover - _under_root 不会越界
        raise TargetVerifyError(f"声明的路径越出 --root：{declared}", ExitCode.PRECHECK) from exc


# --- 声明层 --------------------------------------------------------------------


def load_verify_declaration(matrix: dict) -> dict:
    """取出 verify/uninstall 需要的交付声明；缺字段即显式失败（禁止脚本内默认值）。"""
    delivery = matrix.get("delivery")
    if not isinstance(delivery, dict):
        raise TargetVerifyError(
            f"矩阵缺少 delivery 段：{MATRIX_REL}（自检与卸载的路径/端点无来源，拒绝继续）",
            ExitCode.PRECHECK,
        )
    install = delivery.get("install") or {}
    health = delivery.get("health")
    missing = [f"delivery.install.{key}" for key in REQUIRED_INSTALL_KEYS if key not in install]
    if not isinstance(health, dict):
        missing.append("delivery.health（整个段）")
        health = {}
    else:
        missing += [f"delivery.health.{key}" for key in REQUIRED_HEALTH_KEYS if key not in health]
    if missing:
        raise TargetVerifyError(
            "交付声明不完整（铁律 5.3：端点/路径不得在脚本里硬编码）：" + "、".join(missing),
            ExitCode.PRECHECK,
        )
    return {
        "install": install,
        "health": health,
        "board_bundle": delivery.get("board_bundle") or {},
    }


def load_installed_matrix(version_dir: Path) -> dict:
    """从**已装树**读取矩阵声明（目标端没有仓库，只有安装目录里的随包声明）。"""
    matrix_path = version_dir / MATRIX_REL
    if not matrix_path.is_file():
        raise TargetVerifyError(
            f"已装树缺少矩阵声明 {MATRIX_REL}（{version_dir}）：无法确定挂载点与端点，拒绝猜默认值",
            ExitCode.PRECHECK,
        )
    try:
        return load_matrix(version_dir, matrix_path)
    except ManifestError as exc:
        raise TargetVerifyError(
            f"已装树声明无法校验（需要 PyYAML + jsonschema）：{exc}", exc.exit_code
        ) from exc


# --- 已装树与安装记录 -----------------------------------------------------------


def resolve_installed(root: Path, install: dict, version: str | None = None) -> dict:
    """解析已装版本：<prefix>/<current_link> → <prefix>/<version>；找不到即 None（不猜）。"""
    prefix = _under_root(root, install["prefix"])
    link = prefix / install["current_link"]
    resolved = os.readlink(link) if link.is_symlink() else None
    target = version or resolved
    version_dir = prefix / str(target) if target else None
    return {
        "root": str(root),
        "prefix": str(prefix),
        "link": str(link),
        "link_target": resolved,
        "version": str(target) if target else None,
        "version_dir": str(version_dir) if version_dir else None,
        "installed": bool(version_dir and version_dir.is_dir()),
    }


def collect_tree_files(base: Path) -> list[dict]:
    """枚举目录下所有普通文件（相对路径 + SHA-256 + 大小），路径排序保证可复算。"""
    if not base.is_dir():
        raise TargetVerifyError(f"目录不存在：{base}", ExitCode.PRECHECK)
    return [
        {
            "path": path.relative_to(base).as_posix(),
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(item for item in base.rglob("*") if item.is_file())
    ]


def read_install_record(version_dir: Path, record_name: str) -> dict:
    """读取安装记录；缺失或结构不符即**拒绝卸载**（不得凭目录名猜删）。"""
    record_path = version_dir / record_name
    if not record_path.is_file():
        raise TargetVerifyError(
            f"缺少安装记录 {record_name}（{version_dir}）：无法确定本 bundle 落地了哪些文件，"
            "拒绝卸载",
            ExitCode.USAGE,
        )
    try:
        record = json.loads(record_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TargetVerifyError(
            f"安装记录不是合法 JSON：{record_path}（{exc}）", ExitCode.USAGE
        ) from exc
    if not isinstance(record, dict) or record.get("schema_version") != INSTALL_RECORD_SCHEMA_VERSION:
        raise TargetVerifyError(
            f"安装记录 schema 不是 {INSTALL_RECORD_SCHEMA_VERSION}：{record_path}", ExitCode.USAGE
        )
    for key in ("files", "external", "install"):
        if key not in record:
            raise TargetVerifyError(
                f"安装记录缺少必需段：{key}（{record_path}）", ExitCode.USAGE
            )
    return record


def write_install_record(
    *, root: Path, version_dir: Path, install: dict, health: dict, version: str
) -> tuple[dict, Path]:
    """写安装记录：版本目录逐文件 + 目录外的 env/单元文件。卸载只依据它删文件。

    记录文件自身**不登记**在 `files` 里（否则第二次生成记录时会把上一份记录当成待管文件，
    卸载比对即报"记录内文件缺失"——实测缺陷）；与 `compare_record` 的排除保持对称。
    """
    files = [
        item for item in collect_tree_files(version_dir) if item["path"] != install["file_record"]
    ]
    external = []
    for kind, declared in (
        ("systemd_unit", f"{install['systemd_dir']}/{install['service_unit']}"),
        ("env_file", f"{install['config_dir']}/{install['env_file']}"),
    ):
        path = _under_root(root, declared)
        if not path.is_file():
            raise TargetVerifyError(
                f"安装记录无法生成：{kind} 缺失（{declared}）", ExitCode.PRECHECK
            )
        external.append(
            {
                "kind": kind,
                "path": _rel_to_root(root, declared),
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    record = {
        "schema_version": INSTALL_RECORD_SCHEMA_VERSION,
        "recorded_at": _utc_now(),
        "install": {
            "version": version,
            "prefix": install["prefix"],
            "current_link": install["current_link"],
            "log_dir": install["log_dir"],
            "event_store_env": health["event_store_env"],
            "record_name": install["file_record"],
        },
        "files": files,
        "external": external,
        "notes": [
            "本记录由 verify.sh 在安装后立即生成，是 uninstall.sh 唯一的删除依据。",
            "记录内路径一律相对（files 相对版本目录、external 相对 --root），不含本机绝对路径。",
            "记录自身不登记自身哈希：卸载比对时显式排除该文件（见 record_file_excluded），"
            "它随版本目录一起删除。",
        ],
    }
    record_path = version_dir / install["file_record"]
    record_path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return record, record_path


def compare_record(root: Path, version_dir: Path, record: dict, record_name: str) -> dict:
    """安装记录 ↔ 实际树逐文件比对：清单外/缺失 → 范围错误，哈希不符 → 校验失败。

    安装记录自身**显式排除**在比对之外（记录不可能登记自己的哈希）；它随版本目录一起删除，
    这一点写在 `record_file_excluded` 里，不做静默忽略。
    """
    actual = {item["path"]: item for item in collect_tree_files(version_dir)}
    actual.pop(record_name, None)
    declared = {item["path"]: item for item in record["files"]}
    external_issues = []
    for item in record["external"]:
        path = root / item["path"]
        if not path.is_file():
            external_issues.append(f"记录内的目录外文件不存在：{item['path']}（{item['kind']}）")
        elif sha256_file(path) != item["sha256"]:
            external_issues.append(f"目录外文件与记录不一致：{item['path']}（{item['kind']}）")
    return {
        "file_count_record": len(declared),
        "file_count_actual": len(actual),
        "record_file_excluded": record_name,
        "extras": sorted(set(actual) - set(declared)),
        "missing": sorted(set(declared) - set(actual)),
        "tampered": sorted(
            path
            for path in set(actual) & set(declared)
            if actual[path]["sha256"] != declared[path]["sha256"]
        ),
        "external_issues": external_issues,
    }


# --- 探针（HTTP / gRPC / 事件库） -----------------------------------------------


def http_probe(url: str, timeout_s: float, expected_status: int) -> dict:
    """HTTP /health 探针：连接失败 = 服务未启动（显式状态，不是通过）。"""
    result: dict = {"url": url, "timeout_s": timeout_s, "expected_status": expected_status}
    started = time.monotonic()
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            body = response.read(4096).decode("utf-8", "replace")
            result.update({"reachable": True, "status": int(response.status)})
            try:
                parsed = json.loads(body)
            except json.JSONDecodeError:
                parsed = None
                result["body_head"] = body[:200]
            if isinstance(parsed, dict):
                result["body_keys"] = sorted(parsed)
                result["body_status"] = parsed.get("status")
                result["body_simulation"] = parsed.get("simulation")
    except urllib.error.HTTPError as exc:  # 服务在，但返回非 2xx：健康状态不对
        result.update({"reachable": True, "status": int(exc.code), "error": f"HTTP {exc.code}"})
    except (urllib.error.URLError, OSError, ValueError) as exc:
        result.update(
            {"reachable": False, "status": None, "error": f"{type(exc).__name__}: {exc}"}
        )
    result["elapsed_ms"] = int((time.monotonic() - started) * 1000)
    ok = bool(result.get("reachable")) and result.get("status") == expected_status
    result["passed"] = ok
    if not result.get("reachable"):
        result["state"] = SERVICE_NOT_RUNNING
        result["failure"] = "服务未启动：/health 连接失败（不得视为通过）"
    elif not ok:
        result["failure"] = (
            f"/health 返回 {result.get('status')}，期望 {expected_status}（服务在但不健康）"
        )
    return result


def _tcp_reachable(host: str, port: int, timeout_s: float) -> tuple[bool, str | None]:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True, None
    except OSError as exc:
        return False, f"{type(exc).__name__}: {exc}"


def grpc_probe(target: str, timeout_s: float) -> dict:
    """gRPC 探针，三级如实降级：health service → channel ready → TCP 连接。

    只有 TCP 可达时**不得**宣称"gRPC 健康检查通过"：结果保留 `probe_kind` 与 `limitation`，
    供人工判断（铁律 2.5：拿不到真证据就不说通过）。
    """
    host, _, port_text = str(target).rpartition(":")
    result: dict = {"target": target, "timeout_s": timeout_s, "probe_kind": None}
    try:
        port = int(port_text)
    except ValueError:
        result.update(
            {"passed": False, "failure": f"gRPC 目标格式非法：{target!r}（应为 host:port）"}
        )
        return result
    started = time.monotonic()
    reachable, error = _tcp_reachable(host, port, timeout_s)
    result["tcp_reachable"] = reachable
    result["elapsed_ms"] = int((time.monotonic() - started) * 1000)
    if not reachable:
        result.update(
            {
                "passed": False,
                "state": SERVICE_NOT_RUNNING,
                "probe_kind": GRPC_PROBE_TCP,
                "failure": f"gRPC 端口不可达（{target}）：{error}（不得视为通过）",
            }
        )
        return result

    has_grpc = importlib.util.find_spec("grpc") is not None
    has_health = importlib.util.find_spec("grpc_health") is not None
    result["grpc_module"] = has_grpc
    if has_grpc and has_health:
        try:  # pragma: no cover - 目标端才有的可选依赖
            import grpc
            from grpc_health.v1 import health_pb2, health_pb2_grpc

            channel = grpc.insecure_channel(target)
            stub = health_pb2_grpc.HealthStub(channel)
            response = stub.Check(health_pb2.HealthCheckRequest(), timeout=timeout_s)
            status = health_pb2.HealthCheckResponse.ServingStatus.Name(response.status)
            result.update({"probe_kind": GRPC_PROBE_HEALTH_SERVICE, "serving_status": status})
            result["passed"] = status == "SERVING"
            if not result["passed"]:
                result["failure"] = f"gRPC health 服务返回 {status}（期望 SERVING）"
            channel.close()
            return result
        except Exception as exc:  # noqa: BLE001 - 探针失败降级到下一级并记录原因
            result["health_service_error"] = f"{type(exc).__name__}: {exc}"
    if has_grpc:
        try:  # pragma: no cover - 目标端才有的可选依赖
            import grpc

            channel = grpc.insecure_channel(target)
            grpc.channel_ready_future(channel).result(timeout=timeout_s)
            result.update(
                {"probe_kind": GRPC_PROBE_CHANNEL, "channel_state": "READY", "passed": True}
            )
            channel.close()
            return result
        except Exception as exc:  # noqa: BLE001
            result.update({"probe_kind": GRPC_PROBE_CHANNEL, "channel_state": "NOT_READY"})
            result["passed"] = False
            result["channel_error"] = f"{type(exc).__name__}: {exc}"
            result["failure"] = "gRPC 通道未就绪（TCP 可达但协议层无响应）"
            return result
    result.update({"probe_kind": GRPC_PROBE_TCP, "passed": reachable})
    result["limitation"] = (
        "本机无 grpcio：只证明 TCP 端口可达，未做 gRPC 协议层/health 服务判定，"
        "不得表述为\"gRPC 健康检查通过\""
    )
    return result


def event_store_probe(path: Path, *, scope: str, version: str) -> dict:
    """事件库写入检查：建表 + 写探针行 + 回读 + integrity_check（探针表与业务表隔离）。"""
    result: dict = {
        "path": str(path),
        "scope": scope,
        "table": PROBE_TABLE,
        "write_check": False,
        "passed": False,
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        result["failure"] = f"事件库目录不可创建（{path.parent}）：{exc}"
        return result
    conn = None
    try:
        conn = sqlite3.connect(str(path), timeout=10)
        with conn:
            conn.execute(
                f"CREATE TABLE IF NOT EXISTS {PROBE_TABLE} ("
                "recorded_at TEXT NOT NULL, version TEXT NOT NULL, source TEXT NOT NULL)"
            )
            conn.execute(
                f"INSERT INTO {PROBE_TABLE} (recorded_at, version, source) VALUES (?,?,?)",
                (_utc_now(), version, "verify.sh"),
            )
        rows = int(conn.execute(f"SELECT count(*) FROM {PROBE_TABLE}").fetchone()[0])
        integrity = conn.execute("PRAGMA integrity_check").fetchone()[0]
        tables = [row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")]
        result.update(
            {
                "write_check": True,
                "probe_rows": rows,
                "integrity_check": integrity,
                "tables": sorted(tables),
            }
        )
        result["passed"] = rows >= 1 and integrity == "ok"
        if not result["passed"]:
            result["failure"] = f"事件库写入后回读不一致（integrity={integrity}）"
    except sqlite3.Error as exc:
        result["failure"] = f"事件库写入失败：{type(exc).__name__}: {exc}"
    finally:
        if conn is not None:
            conn.close()
    if not result["passed"] and "failure" not in result:
        result["failure"] = "事件库写入检查未通过"
    return result


# --- verify --------------------------------------------------------------------


class VerifyReport:
    """自检证据收集器：stdout 只输出它的一份 JSON。"""

    def __init__(self, mode: str, evidence_scope: str):
        self.report: dict = {
            "schema_version": VERIFY_EVIDENCE_SCHEMA_VERSION,
            "generated_at": _utc_now(),
            "mode": mode,
            "evidence_scope": evidence_scope,
            "simulation": True,
            "verified": False,
            "steps": [],
            "bundle": {},
            "declaration": {},
            "installed": {},
            "health": {},
            "grpc": {},
            "event_store": {},
            "install_record": {},
            "notes": list(NOTES_ALWAYS),
            "exit_code": ExitCode.OK,
        }

    def step(self, name: str, status: str, detail: str, **extra) -> None:
        entry = {"step": name, "status": status, "detail": detail}
        entry.update(extra)
        self.report["steps"].append(entry)
        info(f"[{status}] {name}：{detail}")


def _installed_drift(version_dir: Path, manifest: dict) -> list[str]:
    """已装树 ↔ bundle 清单：只比对 bundle 直接落地的文件（runtime 与 env 另有说明）。"""
    violations: list[str] = []
    for item in manifest.get("files") or []:
        rel = str(item.get("path"))
        if rel.startswith(("runtime/", "env/")):
            continue
        if rel.startswith("scripts/"):
            candidate = version_dir / "scripts" / Path(rel).name
        else:
            candidate = version_dir / rel
        if not candidate.is_file():
            violations.append(f"清单登记的文件未安装：{rel}")
        elif sha256_file(candidate) != item.get("sha256"):
            violations.append(f"已安装文件与 bundle 不一致：{rel}")
    return violations


def _finish_evidence(report: VerifyReport, options: dict, scratch: Path | None, temporary: bool) -> None:
    evidence = report.report
    evidence["summary"] = {
        "steps": len(evidence["steps"]),
        "passed": sum(1 for item in evidence["steps"] if item["status"] == STATUS_PASS),
        "failed": sum(1 for item in evidence["steps"] if item["status"] == STATUS_FAIL),
        "service_not_running": sum(
            1 for item in evidence["steps"] if item["status"] == SERVICE_NOT_RUNNING
        ),
    }
    outputs = []
    if options.get("json_out"):
        path = _abspath(Path.cwd(), options["json_out"])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        outputs.append(str(path))
    if scratch is not None and not temporary and options.get("_log_dir"):
        directory = _under_root(scratch, options["_log_dir"])
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (
            f"verify-evidence-{evidence.get('bundle', {}).get('version') or 'unknown'}.json"
        )
        path.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        outputs.append(str(path))
    if outputs:
        evidence["evidence_files"] = outputs
        for path in outputs:
            info(f"自检证据：{path}（本机路径，不入库）")
    if temporary and scratch is not None:
        shutil.rmtree(scratch, ignore_errors=True)
        evidence["scratch_removed"] = True
    print(json.dumps(evidence, ensure_ascii=False, indent=2))


def _verify_pipeline(
    options: dict, report: VerifyReport, scratch_holder: dict
) -> tuple[Path | None, bool]:
    """执行 S1..S7；契约层失败抛 TargetVerifyError，服务层失败抛退出码 5。"""
    evidence = report.report
    dry_run = bool(options.get("dry_run"))
    root = Path(str(options["root"])).resolve() if options.get("root") else None
    python_exe = options.get("python") or "python3"
    bundle_value = options.get("bundle")
    installed_dir_value = options.get("installed_dir")
    manifest: dict | None = None
    install: dict | None = None
    health: dict | None = None
    version_dir: Path | None = None
    scratch: Path | None = None

    if bundle_value:
        bundle_path = _abspath(Path.cwd(), bundle_value)
        if not bundle_path.is_file():
            raise TargetVerifyError(f"bundle 不存在：{bundle_path}", ExitCode.PRECHECK)
        checksum = Path(str(bundle_path) + ".sha256")
        if not checksum.is_file():
            raise TargetVerifyError(
                f"缺少伴随校验和文件 {checksum.name}（不信任无校验和的 bundle）",
                ExitCode.PRECHECK,
            )
        expected = checksum.read_text(encoding="utf-8").strip().split()[0]
        actual = sha256_file(bundle_path)
        evidence["bundle"] = {
            "name": bundle_path.name,
            "sha256": actual,
            "expected_sha256": expected,
            "size_bytes": bundle_path.stat().st_size,
        }
        if len(expected) != 64 or actual != expected:
            raise TargetVerifyError(
                "bundle SHA-256 与伴随校验和不符",
                ExitCode.VERIFY,
                [f"清单 {expected[:16]}… / 实际 {actual[:16]}…"],
            )
        report.step("S1 bundle 校验和", STATUS_PASS, f"sha256={actual[:16]}…（与伴随文件一致）")

        _, entries, violations = verify_bundle_contract(bundle_path)
        evidence["bundle"]["member_count"] = len(entries)
        if violations:
            raise TargetVerifyError(
                f"bundle 成员校验失败：{len(violations)} 处", ExitCode.VERIFY, violations
            )
        report.step(
            "S2 bundle 成员 SHA-256",
            STATUS_PASS,
            f"{len(entries)} 个成员逐一复算与清单一致",
        )
        scratch_root = root if root is not None else Path(tempfile.mkdtemp(prefix="iraf-verify-"))
        scratch_holder["temporary"] = root is None
        installer = Installer(
            {
                "bundle": str(bundle_path),
                "root": str(scratch_root),
                "python": python_exe,
                "dry_run": True,
            }
        )
        installer.extract_to_staging()
        assert installer.staging is not None  # noqa: S101 - extract_to_staging 成功即已设置
        scratch = installer.staging
        scratch_holder["path"] = scratch
        manifest = installer.bundle_manifest
        evidence["bundle"].update(
            {
                "board": (manifest.get("bundle") or {}).get("board"),
                "version": (manifest.get("bundle") or {}).get("version"),
                "post_verify_status": (manifest.get("post_verify") or {}).get("status"),
                "members_pending": (manifest.get("scripts") or {}).get("missing"),
            }
        )
        report.step(
            "S2b bundle 解包",
            STATUS_PASS,
            f"解包到 {'沙箱' if root is not None else '临时暂存目录（结束后删除）'}"
            f"；post_verify={evidence['bundle']['post_verify_status']}",
        )
        matrix = load_installed_matrix(scratch)
        declaration = load_verify_declaration(matrix)
        install, health = declaration["install"], declaration["health"]
        board_rel = str((manifest.get("board") or {}).get("profile_member"))
        gate = run_profile_check(scratch, board_rel, python_exe, bool(options.get("allow_unverified")))
        gate_report = gate["report"] or {}
        evidence["board_profile_check"] = {
            "exit_code": gate["exit_code"],
            "argv": gate["argv"],
            "verified": bool(gate_report.get("verified")),
            "unverified_count": len(gate_report.get("unverified_fields") or []),
            "pending_count": len(gate_report.get("pending_fields") or []),
            "stderr_tail": gate["stderr_tail"],
        }
        if gate["exit_code"] != 0:
            reasons = gate_report.get("reasons") or gate_report.get("failures") or []
            raise TargetVerifyError(
                f"板卡声明门禁未通过（profile_check 退出码 {gate['exit_code']}）",
                ExitCode.PRECHECK,
                reasons[:5],
            )
        report.step(
            "S3 板卡声明门禁",
            STATUS_PASS,
            f"profile_check --board exit=0（verified={bool(gate_report.get('verified'))}，"
            f"unverified {len(gate_report.get('unverified_fields') or [])} 项、"
            f"pending {len(gate_report.get('pending_fields') or [])} 项已显式放行）",
        )

    if installed_dir_value:
        version_dir = Path(str(installed_dir_value)).resolve()
        matrix = load_installed_matrix(version_dir)
        declaration = load_verify_declaration(matrix)
        install = install or declaration["install"]
        health = health or declaration["health"]
        version = version_dir.name
        if root is not None:
            expected_dir = _under_root(root, install["prefix"]) / version
            if expected_dir.resolve() != version_dir:
                raise TargetVerifyError(
                    f"已装版本目录 {version_dir} 与声明布局不符（按声明应为 {expected_dir}）："
                    "拒绝继续（声明与盘上事实不一致）",
                    ExitCode.PRECHECK,
                )
        report.step(
            "S3b 已装版本目录",
            STATUS_PASS,
            f"{version_dir}（版本 {version}，声明来自随包矩阵 {MATRIX_REL}）",
        )

    if root is not None and install is not None:
        installed = resolve_installed(root, install)
        evidence["installed"] = installed
        if not installed["installed"] and version_dir is not None:
            evidence["installed"].update(
                {
                    "version_dir": str(version_dir),
                    "version": version_dir.name,
                    "installed": True,
                    "note": "未通过激活链接解析（沙箱演练可能只安装未切换），以 --installed-dir 为准",
                }
            )
        if evidence["installed"]["installed"]:
            target_dir = Path(str(evidence["installed"]["version_dir"]))
            if manifest is not None:
                drift = _installed_drift(target_dir, manifest)
                evidence["installed"]["drift_vs_bundle"] = drift
                if drift:
                    raise TargetVerifyError(
                        f"已装树与 bundle 清单不一致：{len(drift)} 处", ExitCode.VERIFY, drift
                    )
                report.step("S4 已装树 ↔ bundle 清单", STATUS_PASS, "逐文件 SHA-256 一致")
            record, record_path = write_install_record(
                root=root,
                version_dir=target_dir,
                install=install,
                health=health,
                version=str(evidence["installed"]["version"]),
            )
            evidence["install_record"] = {
                "path": str(record_path),
                "files": len(record["files"]),
                "external": len(record["external"]),
                "recorded_at": record["recorded_at"],
            }
            report.step(
                "S4 安装记录",
                STATUS_PASS,
                f"{len(record['files'])} 个版本目录文件 + {len(record['external'])} 个目录外文件"
                f"（{record_path.name}，uninstall.sh 的唯一删除依据）",
            )
        else:
            report.step(
                "S4 已装树",
                STATUS_DEFERRED,
                f"{installed['prefix']}/{install['current_link']} 未指向已装版本：尚未安装"
                "（本机演练属预期）",
            )
    elif not bundle_value:
        raise TargetVerifyError(
            "既没有 --bundle 也没有 --installed-dir/--root 可自检对象："
            "--root 只有配合 --bundle 或 --installed-dir 才构成自检对象（拒绝空转）",
            ExitCode.PRECHECK,
        )
    else:
        report.step("S4 已装树", STATUS_SKIPPED, "未给 --root：只做 bundle 侧自检")

    if health is None:
        report.step("S5-S7 服务与事件库探针", STATUS_DEFERRED, "缺少声明来源")
        return scratch, False

    timeout_s = float(options.get("timeout_s") or health["timeout_s"])
    http_url = options.get("health_url") or health["http_url"]
    grpc_target = options.get("grpc_target") or health["grpc_target"]
    overrides = sorted(
        key
        for key, value in (
            ("health_url", options.get("health_url")),
            ("grpc_target", options.get("grpc_target")),
            ("timeout_s", options.get("timeout_s")),
        )
        if value
    )
    http_result = http_probe(http_url, timeout_s, int(health["expect_http_status"]))
    evidence["health"] = {
        **http_result,
        "declared_url": health["http_url"],
        "overridden": overrides,
        "note": "本机探针只证明探针与解析逻辑；stub/loopback 不构成目标端健康证据",
    }
    report.step(
        "S5 /health HTTP 探针",
        STATUS_PASS if http_result["passed"] else (http_result.get("state") or STATUS_FAIL),
        http_result.get("failure") or f"HTTP {http_result.get('status')}（{http_url}）",
    )
    grpc_result = grpc_probe(grpc_target, timeout_s)
    evidence["grpc"] = {**grpc_result, "declared_target": health["grpc_target"]}
    report.step(
        "S6 gRPC 探针",
        STATUS_PASS if grpc_result.get("passed") else (grpc_result.get("state") or STATUS_FAIL),
        grpc_result.get("failure")
        or f"probe_kind={grpc_result.get('probe_kind')}（{grpc_target}）",
    )
    store_path, scope = _resolve_event_store(options, health, install, root=root, scratch=scratch, dry_run=dry_run)
    store_result = event_store_probe(
        store_path,
        scope=scope,
        version=str((evidence.get("installed") or {}).get("version") or "unknown"),
    )
    evidence["event_store"] = store_result
    report.step(
        "S7 事件库写入检查",
        STATUS_PASS if store_result.get("passed") else STATUS_FAIL,
        store_result.get("failure")
        or f"写入+回读通过（探针 {store_result['probe_rows']} 行，"
        f"integrity={store_result['integrity_check']}，scope={scope}）",
    )

    contract_failed = any(
        item["status"] == STATUS_FAIL
        for item in evidence["steps"]
        if item["step"].startswith(("S1", "S2", "S3", "S4"))
    )
    if contract_failed:
        raise TargetVerifyError(
            "自检存在校验失败项：禁止打印\"通过\"",
            ExitCode.VERIFY,
            [f"{item['step']}: {item['detail']}" for item in evidence["steps"] if item["status"] == STATUS_FAIL],
        )
    service_ok = bool(
        evidence.get("health", {}).get("passed")
        and evidence.get("grpc", {}).get("passed")
        and evidence.get("event_store", {}).get("passed")
    )
    service_states = {
        evidence.get("health", {}).get("state"),
        evidence.get("grpc", {}).get("state"),
    }
    not_running = SERVICE_NOT_RUNNING in service_states
    if not_running:
        evidence["service_state"] = SERVICE_NOT_RUNNING
    elif not service_ok:
        evidence["service_state"] = "unhealthy_in_probe"
    else:
        evidence["service_state"] = "healthy_in_probe"
    if dry_run:
        if not_running or not service_ok:
            evidence["service_state"] = SERVICE_NOT_RUNNING
        evidence["verified"] = False
        report.step(
            "判定",
            STATUS_REHEARSAL,
            "演练完成：bundle/契约/探针逻辑已跑通；"
            f"服务状态={evidence['service_state']}，目标端 /health、systemd 与事件库验收 DEFERRED"
            "（板卡不在场）——演练结果不得表述为\"已安装\"或\"板卡可用\"",
        )
        return scratch, True
    if not bool((evidence.get("installed") or {}).get("installed")):
        raise TargetVerifyError("未找到已安装版本：不得把\"未安装\"当作通过", EXIT_POST_CHECK)
    if evidence.get("health", {}).get("state") == SERVICE_NOT_RUNNING:
        raise TargetVerifyError(
            "服务未启动：/health 探针不可达，判定失败（不得视为通过）", EXIT_POST_CHECK
        )
    for label, key in (("/health", "health"), ("gRPC", "grpc"), ("事件库", "event_store")):
        if not evidence.get(key, {}).get("passed"):
            raise TargetVerifyError(
                f"{label} 判定失败：{evidence.get(key, {}).get('failure')}", EXIT_POST_CHECK
            )
    evidence["verified"] = True
    evidence["service_state"] = "healthy"
    report.step("判定", STATUS_PASS, "bundle、已装树、服务与事件库全部通过")
    return scratch, True


def _resolve_event_store(
    options: dict, health: dict, install: dict | None, *, root: Path | None, scratch: Path | None, dry_run: bool
) -> tuple[Path, str]:
    """事件库路径：--event-store > 已装 env 文件 > 声明默认值。

    演练一律把路径重挂到暂存/沙箱（`scope=sandbox_scratch`），**不触碰真实事件库**；
    只有真实执行（非演练、root 为真实根）才写声明的目标端路径（`scope=target_store`）。
    """
    if options.get("event_store"):
        return _abspath(Path.cwd(), options["event_store"]), "explicit_override"
    if install is not None and root is not None and not dry_run:
        # 真实执行：只认「已装 env 文件」里的路径或声明默认值。--root 不是 / 时按 root 重挂
        # （沙箱执行不得写到真实 /var/lib），并如实标注 scope，避免把沙箱结果当成目标端证据。
        sandbox = root != Path("/")
        env_file = _under_root(root, f"{install['config_dir']}/{install['env_file']}")
        declared = None
        if env_file.is_file():
            declared = _read_env_file(env_file).get(health["event_store_env"])
        if not declared:
            declared = str(health["event_store_default"])
        if sandbox:
            path = (
                Path(declared)
                if Path(declared).is_relative_to(root)
                else _under_root(root, declared)
            )
            return path, "declared_path_under_root"
        return Path(declared), (
            "installed_env_file" if env_file.is_file() else "target_store"
        )
    base = scratch
    if base is None:
        base = Path(tempfile.gettempdir()) / "iraf-verify-store"
        return base / Path(str(health["event_store_default"])).name, "ephemeral_scratch"
    return _under_root(base, health["event_store_default"]), "sandbox_scratch"


def _read_env_file(path: Path) -> dict:
    values: dict = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip()
    return values


def run_verify(options: dict) -> int:
    dry_run = bool(options.get("dry_run"))
    report = VerifyReport(
        "rehearsal" if dry_run else "apply",
        "rehearsal_only" if dry_run else "target_end_verify",
    )
    holder: dict = {"path": None, "temporary": False}
    code = ExitCode.OK
    try:
        if dry_run and not options.get("bundle") and not options.get("installed_dir") and not options.get("root"):
            raise TargetVerifyError(
                "演练必须给出 --bundle 或 --root/--installed-dir（演练对象不得为空）", ExitCode.USAGE
            )
        if not dry_run and not options.get("root"):
            raise TargetVerifyError(
                "真实自检必须显式给出 --root（目标端为 /）", ExitCode.USAGE
            )
        scratch, _ = _verify_pipeline(options, report, holder)
    except (TargetVerifyError, ManifestError) as exc:
        code = exc.exit_code
        report.report["failure"] = {"message": str(exc), "details": exc.details}
        report.step("判定", STATUS_FAIL, str(exc))
        print(f"[target_verify][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[target_verify][错误]   - {detail}", file=sys.stderr)
    finally:
        report.report["exit_code"] = code
        _finish_evidence(report, options, holder.get("path"), bool(holder.get("temporary")))
    return code


# --- uninstall -----------------------------------------------------------------


def run_uninstall(options: dict) -> int:
    dry_run = bool(options.get("dry_run"))
    if not options.get("root"):
        raise TargetVerifyError(
            "缺少 --root（卸载必须显式给出文件系统根：沙箱演练给沙箱目录，目标端给 /）",
            ExitCode.USAGE,
        )
    root = Path(str(options["root"])).resolve()
    report: dict = {
        "schema_version": UNINSTALL_REPORT_SCHEMA_VERSION,
        "generated_at": _utc_now(),
        "mode": "rehearsal" if dry_run else "apply",
        "evidence_scope": "rehearsal_only" if dry_run else "target_end_uninstall",
        "simulation": True,
        "root": str(root),
        "steps": [],
        "removed": [],
        "kept": [],
        "exit_code": ExitCode.OK,
    }

    def step(name: str, status: str, detail: str, **extra) -> None:
        entry = {"step": name, "status": status, "detail": detail}
        entry.update(extra)
        report["steps"].append(entry)
        info(f"[{status}] {name}：{detail}")

    context = _uninstall_context(options)
    install = context["install"]
    health = context["health"]
    version_dir = context["version_dir"]
    version = version_dir.name
    record = read_install_record(version_dir, install["file_record"])
    step(
        "U1 安装记录",
        STATUS_PASS,
        f"{install['file_record']}：{len(record['files'])} 个版本目录文件 + "
        f"{len(record['external'])} 个目录外文件（recorded_at={record.get('recorded_at')}）",
    )
    comparison = compare_record(root, version_dir, record, install["file_record"])
    report["comparison"] = comparison
    if comparison["extras"]:
        raise TargetVerifyError(
            f"发现清单外文件 {len(comparison['extras'])} 个：拒绝卸载（只动本 bundle 记录的文件）",
            ExitCode.USAGE,
            comparison["extras"],
        )
    if comparison["missing"]:
        raise TargetVerifyError(
            f"记录内文件缺失 {len(comparison['missing'])} 个：安装树已损坏，拒绝卸载",
            ExitCode.USAGE,
            comparison["missing"],
        )
    if comparison["tampered"]:
        raise TargetVerifyError(
            f"文件与安装记录不一致 {len(comparison['tampered'])} 个：拒绝卸载",
            ExitCode.VERIFY,
            comparison["tampered"],
        )
    if comparison["external_issues"]:
        raise TargetVerifyError(
            f"目录外文件与记录不一致 {len(comparison['external_issues'])} 处：拒绝卸载",
            ExitCode.VERIFY,
            comparison["external_issues"],
        )
    step(
        "U2 范围比对",
        STATUS_PASS,
        f"版本目录 {comparison['file_count_actual']} 个文件逐 SHA-256 与记录一致，"
        "零清单外文件、零缺失、零篡改",
    )

    link = Path(context["link"])
    if link.is_symlink() and os.readlink(link) != version:
        raise TargetVerifyError(
            f"激活链接 {link} 指向 {os.readlink(link)}，不是待卸载版本 {version}：拒绝卸载",
            ExitCode.USAGE,
        )
    unit_path = _under_root(root, context["unit_declared"])
    log_dir = _under_root(root, install["log_dir"])
    report["kept"] = [
        f"{install['log_dir']}（日志属数据，卸载保留）",
        f"{health['event_store_env']}={health['event_store_default']}（事件库属数据，卸载保留）",
    ]
    if dry_run:
        report["would_remove"] = [str(version_dir), str(link), str(unit_path)]
        step(
            "U3 演练",
            STATUS_REHEARSAL,
            "演练只做范围比对：未执行 systemctl、未删除任何文件；目标端卸载 DEFERRED（板卡不在场）",
        )
        step("U4 保留项", STATUS_DEFERRED, "演练不改变盘上状态，保留项仅登记")
        _finish_uninstall(report, options)
        return ExitCode.OK

    if root == Path("/"):
        service = install["service_unit"]
        try:
            active = subprocess.run(
                ["systemctl", "is-active", service], capture_output=True, text=True, timeout=60
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError) as exc:
            raise TargetVerifyError(
                f"无法查询服务状态（{service}）：{exc}；拒绝在状态未知时删除文件", EXIT_POST_CHECK
            ) from exc
        if active == "active":
            try:
                subprocess.run(
                    ["systemctl", "stop", service], check=True, capture_output=True, timeout=300
                )
                subprocess.run(
                    ["systemctl", "disable", service], check=True, capture_output=True, timeout=300
                )
            except (subprocess.SubprocessError, OSError) as exc:
                raise TargetVerifyError(
                    f"停止/注销服务失败（{service}）：{exc}；未删除任何文件", EXIT_POST_CHECK
                ) from exc
            step("U3 停服务", STATUS_PASS, f"{service} 已 stop + disable")
        else:
            step("U3 停服务", STATUS_DEFERRED, f"{service} 当前状态 {active or 'unknown'}")
    else:
        step("U3 停服务", STATUS_DEFERRED, "非 / 根（沙箱）：不执行 systemctl")

    for item in record["external"]:
        (root / item["path"]).unlink()
        report["removed"].append(item["path"])
    if link.is_symlink():
        link.unlink()
        report["removed"].append(str(link))
    shutil.rmtree(version_dir)
    report["removed"].append(str(version_dir))
    step(
        "U4 删除",
        STATUS_PASS,
        f"删除版本目录 + 激活链接 + {len(record['external'])} 个目录外文件（env/单元）",
    )
    remaining = [path for path in (version_dir, unit_path, log_dir) if path.exists()]
    report["remaining_after"] = [str(path) for path in remaining]
    report["verified_scope"] = not version_dir.exists() and bool(log_dir.exists() or True)
    step(
        "U5 保留项",
        STATUS_PASS,
        "日志目录与事件库保留；卸载只动本 bundle 记录的文件",
        kept=report["kept"],
    )
    _finish_uninstall(report, options)
    return ExitCode.OK


def _uninstall_context(options: dict) -> dict:
    """解析卸载上下文：声明必须先于删除可见（否则只能猜路径）。

    版本目录来源优先级：`--version` → 激活链接 `<prefix>/<current_link>` → `--installed-dir`。
    """
    root = Path(str(options["root"])).resolve()
    version_hint = options.get("version")
    installed_dir = options.get("installed_dir")
    version_dir = None
    # 已装矩阵声明的深度是 <prefix>/<version>/config/sdk/package_matrix.yaml（前缀层级由声明决定，
    # 不假设固定深度）：用 rglob 找到声明，再按 parents[2] 反推版本目录，最后用声明自校验。
    candidates = sorted(path for path in root.rglob(MATRIX_REL) if path.is_file())
    if installed_dir:
        version_dir = Path(str(installed_dir)).resolve()
    elif version_hint:
        matched = [path for path in candidates if path.parents[2].name == str(version_hint)]
        if len(matched) != 1:
            raise TargetVerifyError(
                f"按 --version {version_hint} 在 {root} 下找到 {len(matched)} 个候选安装目录："
                "拒绝卸载（用 --installed-dir 显式指定）",
                ExitCode.USAGE,
            )
        version_dir = matched[0].parents[2]
    else:
        for path in candidates:
            prefix = path.parents[2].parent
            link = prefix / "current"
            if link.is_symlink():
                version_dir = prefix / os.readlink(link)
                break
        if version_dir is None:
            raise TargetVerifyError(
                f"在 {root} 下找不到激活链接指向的已装版本：用 --version/--installed-dir 显式指定，"
                "拒绝卸载（不得凭目录名猜删）",
                ExitCode.USAGE,
            )
    if not version_dir.is_dir():
        raise TargetVerifyError(f"版本目录不存在：{version_dir}", ExitCode.USAGE)
    matrix = load_installed_matrix(version_dir)
    declaration = load_verify_declaration(matrix)
    install = declaration["install"]
    prefix = _under_root(root, install["prefix"])
    if version_dir.parent.resolve() != prefix.resolve():
        raise TargetVerifyError(
            f"待卸载目录 {version_dir} 不在声明的安装前缀 {install['prefix']} 下（按 --root 应为 {prefix}）："
            "拒绝卸载",
            ExitCode.USAGE,
        )
    return {
        "version_dir": version_dir,
        "install": install,
        "health": declaration["health"],
        "link": str(prefix / install["current_link"]),
        "unit_declared": f"{install['systemd_dir']}/{install['service_unit']}",
    }


def _finish_uninstall(report: dict, options: dict) -> None:
    report["summary"] = {
        "steps": len(report["steps"]),
        "removed": len(report.get("removed") or []),
        "kept": len(report.get("kept") or []),
    }
    if options.get("json_out"):
        path = _abspath(Path.cwd(), options["json_out"])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        report["report_file"] = str(path)
    print(json.dumps(report, ensure_ascii=False, indent=2))


# --- CLI -----------------------------------------------------------------------


def parse_options(argv: list[str]) -> dict:
    options: dict = {}
    value_flags = {
        "--bundle": "bundle",
        "--root": "root",
        "--installed-dir": "installed_dir",
        "--python": "python",
        "--json-out": "json_out",
        "--version": "version",
        "--health-url": "health_url",
        "--grpc-target": "grpc_target",
        "--timeout-s": "timeout_s",
        "--event-store": "event_store",
    }
    bool_flags = {"--dry-run": "dry_run", "--allow-unverified": "allow_unverified"}
    index = 0
    while index < len(argv):
        item = argv[index]
        if item in ("-h", "--help"):
            options["help"] = True
        elif item in value_flags:
            if index + 1 >= len(argv):
                raise TargetVerifyError(f"{item} 缺少取值", ExitCode.USAGE)
            options[value_flags[item]] = argv[index + 1]
            index += 1
        elif item in bool_flags:
            options[bool_flags[item]] = True
        else:
            raise TargetVerifyError(f"无法识别的参数：{item}（用 --help 查看用法）", ExitCode.USAGE)
        index += 1
    return options


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return ExitCode.OK if argv else ExitCode.USAGE
    command, rest = argv[0], argv[1:]
    if command not in ("verify", "uninstall"):
        print(f"[target_verify] 无法识别的子命令：{command}\n", file=sys.stderr)
        return ExitCode.USAGE
    try:
        options = parse_options(rest)
        if options.get("help"):
            print(USAGE)
            return ExitCode.OK
        return run_verify(options) if command == "verify" else run_uninstall(options)
    except (TargetVerifyError, ManifestError) as exc:
        print(f"[target_verify][错误] {exc}", file=sys.stderr)
        for detail in exc.details:
            print(f"[target_verify][错误]   - {detail}", file=sys.stderr)
        return exc.exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
