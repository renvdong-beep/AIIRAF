"""实时性能测试引擎：在 **RTOS/运控域** 上跑 cyclictest，边缘侧只做 SSH 编排与解析。

铁律依据（IRAF AGENTS.md）：
- RTOS 侧**不安装任何 IRAF 进程**（会干扰实时性）；这里只通过 SSH 执行一条固定命令。
- 命令模板固定、参数有界（时长/间隔/直方图/优先级），不接受任意 shell。
- 目标地址/账号/密钥一律来自配置（env），代码不写死 IP。
- 失败必须显式：ssh 不可达 / cyclictest 缺失 / 权限不足 / 解析失败 各有错误码。

命令（与使用者给定的一致）：
    cyclictest --mlockall --smp --priority=<P> --interval=<I> --distance=0 -n -h <H> -q -D <D>s

注意：`-n` 只在支持 clock_nanosleep 的 rt-tests 版本可用；若目标返回 invalid option，
引擎会自动去掉 `-n` 重试一次并在报告里记录 `fallback`。
"""
from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

DEFAULT_HISTOGRAM_US = 100
DEFAULT_DURATION_S = 10
DEFAULT_INTERVAL_US = 200
DEFAULT_PRIORITY = 240
MAX_DURATION_S = 300


class RtTestError(RuntimeError):
    def __init__(self, code: str, detail: str):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


@dataclass
class RtTarget:
    mode: str = "ssh"           # ssh | local
    host: str = ""
    user: str = ""
    port: int = 22
    identity_file: str = ""
    known_hosts: str = ""
    strict_host_key: str = "accept-new"
    sudo: bool = False
    report_dir: str = ""
    max_us: Optional[float] = None

    @classmethod
    def from_env(cls) -> "RtTarget":
        def _f(name: str, default: str = "") -> str:
            return (os.environ.get(name) or default).strip()

        mode = _f("IRAF_RT_TARGET_MODE", "ssh").lower()
        max_us_raw = _f("IRAF_RT_MAX_US")
        return cls(
            mode=mode if mode in ("ssh", "local") else "ssh",
            host=_f("IRAF_RT_TARGET_HOST"),
            user=_f("IRAF_RT_TARGET_USER", "root"),
            port=int(_f("IRAF_RT_TARGET_PORT", "22") or 22),
            identity_file=_f("IRAF_RT_TARGET_IDENTITY"),
            known_hosts=_f("IRAF_RT_TARGET_KNOWN_HOSTS"),
            sudo=_f("IRAF_RT_TARGET_SUDO").lower() in ("1", "true", "yes", "on"),
            report_dir=_f("IRAF_RT_REPORT_DIR"),
            max_us=float(max_us_raw) if max_us_raw else None,
        )


def build_command(duration_s: int, interval_us: int, priority: int,
                  histogram_us: int, with_n: bool = True,
                  with_distance: bool = True) -> List[str]:
    if not 1 <= int(duration_s) <= MAX_DURATION_S:
        raise RtTestError("INPUT-INVALID", "duration_s 必须在 1..%d 秒" % MAX_DURATION_S)
    if not 50 <= int(interval_us) <= 100000:
        raise RtTestError("INPUT-INVALID", "interval_us 必须在 50..100000")
    if not 1 <= int(priority) <= 255:
        raise RtTestError("INPUT-INVALID", "priority 必须在 1..255（Intewell RT 常用 240，Linux 上限 99）")
    if not 0 <= int(histogram_us) <= 100000:
        raise RtTestError("INPUT-INVALID", "histogram_us 必须在 0..100000")
    cmd = ["cyclictest", "--mlockall", "--smp", "--priority=%d" % int(priority),
           "--interval=%d" % int(interval_us)]
    if with_distance:
        cmd.append("--distance=0")
    if with_n:
        cmd.append("-n")
    cmd += ["-h", str(int(histogram_us)), "-q", "-D", "%ds" % int(duration_s)]
    return cmd


def _wrap_ssh(cmd: List[str], target: RtTarget) -> List[str]:
    if target.mode == "local":
        return (["sudo", "-n"] + cmd) if target.sudo else cmd
    if not target.host:
        raise RtTestError("TARGET-UNCONFIGURED",
                          "未配置 RTOS 目标：设置 IRAF_RT_TARGET_HOST（RTOS 的 IP/主机名）")
    ssh = ["ssh", "-o", "BatchMode=yes",
           "-o", "StrictHostKeyChecking=%s" % (target.strict_host_key or "accept-new"),
           "-p", str(target.port)]
    if target.known_hosts:
        ssh += ["-o", "UserKnownHostsFile=%s" % target.known_hosts]
    if target.identity_file:
        ssh += ["-i", target.identity_file]
    remote = " ".join(shlex.quote(x) for x in cmd)
    if target.sudo:
        remote = "sudo -n " + remote
    return ssh + ["%s@%s" % (target.user or "root", target.host), remote]


@dataclass
class RtResult:
    per_core: List[dict] = field(default_factory=list)
    overflows: List[int] = field(default_factory=list)
    raw: str = ""
    command: str = ""
    fallback: str = ""
    elapsed_s: float = 0.0
    host: str = ""
    verdict: dict = field(default_factory=dict)
    report_paths: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        max_us = max((c["max_us"] for c in self.per_core), default=None)
        avg_us = max((c["avg_us"] for c in self.per_core), default=None)
        return {
            "schema": "iraf.rt_latency_test/v1",
            "status": "SUCCEEDED",
            "target": self.host,
            "command": self.command,
            "fallback": self.fallback,
            "elapsed_s": round(self.elapsed_s, 2),
            "per_core": self.per_core,
            "overflows": self.overflows,
            "summary": {"cores": len(self.per_core), "max_us": max_us, "max_avg_us": avg_us,
                        "worst_core": max(self.per_core, key=lambda c: c["max_us"])["cpu"]
                        if self.per_core else None},
            "verdict": self.verdict,
            "report": self.report_paths,
        }


_SUMMARY_RE = {
    "min_us": re.compile(r"^#\s*Min Latencies:\s*(.+)$", re.M),
    "avg_us": re.compile(r"^#\s*Avg Latencies:\s*(.+)$", re.M),
    "max_us": re.compile(r"^#\s*Max Latencies:\s*(.+)$", re.M),
}
_OVERFLOW_RE = re.compile(r"^#\s*Overflows:\s*(.+)$", re.M)
_THREAD_RE = re.compile(
    r"^T:\s*(\d+).*?Min:\s*(\d+).*?Act:\s*(\d+).*?Avg:\s*(\d+).*?Max:\s*(\d+)", re.M)


def parse_output(raw: str) -> tuple[List[dict], List[int]]:
    """解析 `-q` 汇总行；若没有汇总行再回退到 T: 行。"""
    cols = {}
    for key, rx in _SUMMARY_RE.items():
        m = rx.search(raw)
        if m:
            cols[key] = [int(x) for x in re.findall(r"\d+", m.group(1))]
    per_core: List[dict] = []
    if cols.get("min_us") and cols.get("max_us"):
        n = min(len(cols.get("min_us", [])), len(cols.get("avg_us", [])), len(cols["max_us"]))
        for i in range(n):
            per_core.append({"cpu": i,
                             "min_us": cols["min_us"][i],
                             "avg_us": cols["avg_us"][i] if i < len(cols.get("avg_us", [])) else None,
                             "max_us": cols["max_us"][i]})
    if not per_core:
        for m in _THREAD_RE.finditer(raw):
            per_core.append({"cpu": int(m.group(1)), "min_us": int(m.group(2)),
                             "avg_us": int(m.group(4)), "max_us": int(m.group(5))})
    if not per_core:
        raise RtTestError("PARSE-FAILED", "无法从 cyclictest 输出解析出每核延迟：%.200s" % raw)
    overflows: List[int] = []
    mo = _OVERFLOW_RE.search(raw)
    if mo:
        overflows = [int(x) for x in re.findall(r"\d+", mo.group(1))]
    return per_core, overflows


def _write_reports(result: RtResult, target: RtTarget) -> dict:
    base = Path(target.report_dir or os.environ.get("IRAF_RT_REPORT_DIR")
                or "build/acceptance/rt")
    base.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    payload = result.as_dict()
    json_path = base / ("rt_latency_%s.json" % stamp)
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 实时性能测试报告（cyclictest）", "",
             "- 目标：`%s`" % payload["target"], "- 命令：`%s`" % payload["command"],
             "- 时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"), ""]
    if payload["fallback"]:
        lines += ["- 注：%s" % payload["fallback"], ""]
    lines += ["| CPU | min (µs) | avg (µs) | max (µs) |", "|---|---|---|---|"]
    for c in payload["per_core"]:
        lines.append("| %s | %s | %s | %s |" % (c["cpu"], c["min_us"], c["avg_us"], c["max_us"]))
    lines += ["", "汇总：核数 %d，最大延迟 %s µs（core %s）" % (
        payload["summary"]["cores"], payload["summary"]["max_us"], payload["summary"]["worst_core"])]
    if payload["overflows"]:
        lines.append("溢出计数：%s" % payload["overflows"])
    if payload["verdict"]:
        lines.append("判定：%s（阈值 %s µs）" % (
            "PASS" if payload["verdict"].get("ok") else "FAIL", payload["verdict"].get("threshold_us")))
    md_path = base / ("rt_latency_%s.md" % stamp)
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"json": str(json_path), "markdown": str(md_path)}


def run_test(duration_s: int = DEFAULT_DURATION_S, interval_us: int = DEFAULT_INTERVAL_US,
             priority: int = DEFAULT_PRIORITY, histogram_us: int = DEFAULT_HISTOGRAM_US,
             target: Optional[RtTarget] = None) -> dict:
    tgt = target or RtTarget.from_env()
    # 参数兼容矩阵（2026-11-11 实测：rt-tests 2.20 不支持 -n；--distance 与直方图同时开启会打印
    # WARN 并落到 usage）。按"使用者原始命令优先，逐级降级"的顺序尝试，报告里记录实际使用的组合。
    attempts = [(True, True), (False, True), (True, False), (False, False)]
    last_err: Optional[RtTestError] = None
    for with_n, with_distance in attempts:
        cmd = build_command(duration_s, interval_us, priority, histogram_us,
                            with_n=with_n, with_distance=with_distance)
        full = _wrap_ssh(cmd, tgt)
        t0 = time.time()
        try:
            proc = subprocess.run(full, capture_output=True, text=True,
                                  timeout=int(duration_s) + 30)
        except FileNotFoundError as exc:
            raise RtTestError("SSH-UNAVAILABLE", "找不到 ssh 客户端：%s" % exc)
        except subprocess.TimeoutExpired:
            raise RtTestError("TIMEOUT", "命令超时（>%ds）：%s" % (int(duration_s) + 30, " ".join(full)))
        elapsed = time.time() - t0
        out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        # 注意：rt-tests 2.20 对未知选项打印 Usage 但**退出码仍为 0**（实测），
        # 所以兼容性判断不能只看 returncode。
        if re.search(r"Usage:\s*cyclictest|invalid option|unrecognized option|distance is ignored",
                     out, re.I):
            last_err = RtTestError("FALLBACK", "参数组合不被目标 rt-tests 接受，已降级重试")
            continue
        if re.search(r"Unable to change scheduling policy|missing capabilities|must be root", out, re.I):
            raise RtTestError("PERMISSION",
                              "目标机权限不足：需要 root/CAP_SYS_NICE；可设 IRAF_RT_TARGET_SUDO=1 或改用 root")
        if proc.returncode != 0:
            if re.search(r"command not found|not found", out):
                raise RtTestError("CYCLICTEST-MISSING",
                                  "目标机未安装 cyclictest（rt-tests）。安装后重试：dnf install -y rt-tests")
            if re.search(r"Operation not permitted|Permission denied|must be root", out, re.I):
                raise RtTestError("PERMISSION", "权限不足（需要 root/CAP_SYS_NICE 与 mlockall 许可）：%.200s" % out.strip())
            if "ssh:" in out.lower() or "connection refused" in out.lower() or "timed out" in out.lower():
                raise RtTestError("SSH-FAILED", "无法连接 RTOS 目标：%.200s" % out.strip())
            raise RtTestError("COMMAND-FAILED", "cyclictest 退出码 %d：%.300s" % (proc.returncode, out.strip()))
        per_core, overflows = parse_output(out)
        notes = []
        if not with_n:
            notes.append("去掉了 -n（目标 rt-tests 不支持）")
        if not with_distance:
            notes.append("去掉了 --distance=0（与直方图冲突）")
        res = RtResult(per_core=per_core, overflows=overflows, raw=out,
                       command=" ".join(cmd), fallback="；".join(notes),
                       elapsed_s=elapsed, host=tgt.host or "local")
        if tgt.max_us is not None:
            worst = max(c["max_us"] for c in per_core)
            res.verdict = {"ok": worst <= tgt.max_us, "threshold_us": tgt.max_us, "max_us": worst}
        res.report_paths = _write_reports(res, tgt)
        return res.as_dict()
    raise last_err or RtTestError("UNKNOWN", "未知失败")