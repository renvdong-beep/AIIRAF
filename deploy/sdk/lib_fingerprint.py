#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""板卡指纹提取与交叉校验（实现层，2026-10-15）。

用途：从**只读采集**的证据目录（`scripts/board_collect.sh` 产出的 build/board/<name>/）里提取一组
**板级指纹**，再与 `profiles/boards/*.yaml` 里声明的 `spec.fingerprint` 逐项比对，回答两件事：
  ① 这台机器与哪个已声明板卡**一致**（防装错板 / 防拿错 profile 去部署）；
  ② 哪里**不一致**（不一致即 fail-closed 报警，而不是"大概像"）。

定位与边界（AGENTS 6.10 是硬约束）
  · 构型名、VM 数量/编号/IP/角色映射的**事实来源只能是签名 HyperProfile/Release manifest**；
  · 本工具**不产生**构型结论，它只做"实测 ↔ 声明"的交叉校验：声明是权威，指纹是防呆。
    因此：没有签名清单时，本工具只能报"无法判定（缺少权威声明）"，不得据此认定机型。
  · 只读：不连板卡、不改任何文件（输入是已采集的证据目录）。

用法：
    python3 deploy/sdk/lib_fingerprint.py --collect-dir build/board/e300-lab [--json]
退出码：0 与唯一已声明板卡一致 / 1 参数错误 / 2 无法判定（声明缺 fingerprint 或采集缺项）/ 4 与所有声明不符
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
BOARDS_DIR = Path("profiles/boards")

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_UNDECIDABLE = 2
EXIT_MISMATCH = 4

#: 指纹字段 → 从采集证据里怎么取（每条都能从 raw/*.txt 原文复现；取不到就记 None=unverified）
def _read(raw: Path, name: str) -> str:
    path = raw / ("%s.txt" % name)
    return path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""


def extract_fingerprint(collect_dir: Path) -> dict:
    """从采集目录提取指纹。缺失项记 None（= unverified），不做猜测。"""
    raw = collect_dir / "raw"
    if not raw.is_dir():
        raise SystemExit("采集目录里没有 raw/：%s（先跑 scripts/board_collect.sh）" % collect_dir)
    domains = _read(raw, "domains")
    identity = _read(raw, "identity")
    iaros = _read(raw, "iaros_version")
    npu = _read(raw, "npu_media")
    host = _read(raw, "host")

    # ① Hyper 来宾数量与编号：dmesg 里 "virtio_blk_probe: vmN ..." 的不同 vmN 个数
    guests = sorted(set(re.findall(r"virtio_blk_probe:\s*(vm\d+)", domains)))
    # ② vdisk 文件（来宾根盘名，构型相关）
    vdisks = sorted(set(re.findall(r"blkfile=(\S+)", domains)))
    # ③ 来宾网卡名（vm-0-net0 …），能反映每域网卡数
    nets = sorted(set(re.findall(r"net_name:(\S+)", domains)))
    # ④ 内核命令行里的板级分配项（maxcpus / iommu / smmu 等）
    cmdline = ""
    for line in identity.splitlines():
        if line.startswith("rw ") or " fw_devlink=" in line:
            cmdline = line.strip()
    tokens = sorted(re.findall(r"(maxcpus=\d+|smmu=\S+|iommu\.passthrough=\S+|root=\S+)", cmdline))
    # ⑤ 板级 NPU / 媒体栈驱动名（强指纹：NPU 型号不同会直接暴露）
    npu_tokens = sorted({line.strip() for line in npu.splitlines()
                         if line.strip() and not line.startswith("/dev") and "-" in line or
                         re.match(r"^(houmo|musa|mt_|m\d{3,4})", line.strip() or " ")})
    npu_tokens = [t for t in npu_tokens if t and not t.startswith("---")]
    # ⑥ /etc/iaros-version 的组件版本
    comps = {}
    for line in iaros.splitlines():
        if ":" in line:
            key, _, value = line.partition(":")
            comps[key.strip()] = value.strip()
    # ⑦ CPU 核数（板级分配，实测 maxcpus）
    cpu = None
    match = re.search(r"maxcpus=(\d+)", cmdline)
    if match:
        cpu = int(match.group(1))

    return {
        "hyper_guests": guests or None,
        "guest_vdisks": vdisks or None,
        "guest_net_names": nets or None,
        "kernel_cmdline_tokens": tokens or None,
        "npu_stack_tokens": npu_tokens or None,
        "iaros_components": comps or None,
        "cpu_cores": cpu,
        "source": {"collect_dir": str(collect_dir),
                   "raw_files": ["domains.txt", "identity.txt", "iaros_version.txt", "npu_media.txt", "host.txt"],
                   "note": "全部取自只读采集原文；缺失即 unverified，不做推断"},
    }


def load_declared(repo_root: Path) -> dict:
    """读取所有 BoardProfile 的 spec.fingerprint（没有该节的板卡不参与比对）。"""
    declared = {}
    for path in sorted((repo_root / BOARDS_DIR).glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        spec = data.get("spec") or {}
        fingerprint = spec.get("fingerprint")
        if isinstance(fingerprint, dict):
            declared[data.get("metadata", {}).get("name") or path.stem] = {
                "path": str(path.relative_to(repo_root)), "fingerprint": fingerprint,
                "status": spec.get("status"), "hyper_models": (data.get("metadata") or {}).get("hyper_models"),
            }
    return declared


def _norm_list(value) -> list:
    if value in (None, "unverified"):
        return []
    if isinstance(value, str):
        return [value]
    return [str(item) for item in value]


def compare(declared_fp: dict, measured: dict) -> list:
    """逐字段比对：返回 [{field, expected, actual, ok, note}]。未声明的字段不参与（不算冲突）。"""
    rows = []
    for field, expected in sorted(declared_fp.items()):
        if expected in (None, "unverified"):
            rows.append({"field": field, "expected": expected, "actual": measured.get(field),
                         "ok": None, "note": "声明为 unverified ⇒ 只记录实测值，不判定"})
            continue
        actual = measured.get(field)
        if isinstance(expected, dict) and "contains" in expected:
            # 子集语义：声明的是"必须出现的标记"（如 NPU 驱动名），实测多出别的工具链不算不符
            wanted = _norm_list(expected["contains"])
            have = _norm_list(actual)
            ok = all(item in have for item in wanted)
        elif isinstance(expected, list):
            # 全等语义：拓扑/文件名类字段必须逐项一致（少一项都说明不是同一构型）
            ok = sorted(_norm_list(actual)) == sorted(_norm_list(expected))
        elif isinstance(expected, dict):
            ok = all(str((actual or {}).get(k)) == str(v) for k, v in expected.items())
        else:
            ok = str(actual) == str(expected)
        rows.append({"field": field, "expected": expected, "actual": actual, "ok": bool(ok),
                     "note": "" if ok else "实测与声明不一致（fail-closed：先查是不是拿错板/错 profile）"})
    return rows


def identify(repo_root: Path, collect_dir: Path) -> dict:
    measured = extract_fingerprint(collect_dir)
    declared = load_declared(repo_root)
    results = {}
    matches = []
    for name, item in declared.items():
        rows = compare(item["fingerprint"], measured)
        decisive = [r for r in rows if r["ok"] is not None]
        mismatch = [r for r in decisive if not r["ok"]]
        results[name] = {"path": item["path"], "status": item["status"],
                         "hyper_models": item["hyper_models"], "rows": rows,
                         "decisive": len(decisive), "mismatch": mismatch}
        if decisive and not mismatch:
            matches.append(name)

    decisive_any = any(item["decisive"] for item in results.values())
    if not declared:
        verdict, code = "无法判定：没有任何 BoardProfile 声明 spec.fingerprint", EXIT_UNDECIDABLE
    elif not decisive_any:
        # 声明了 fingerprint 但字段全是 unverified ⇒ 没有任何可判定的项，同样属于"无法判定"，
        # 不能报"不匹配"（那会误导成"拿错板"，实际是"声明还没回填"）
        verdict, code = "无法判定：已声明的 fingerprint 字段全为 unverified（先在板卡实测后回填）", EXIT_UNDECIDABLE
    elif not matches:
        verdict, code = ("不匹配：与本机实测不符于全部已声明板卡（%s）"
                         % "，".join(sorted({r["field"] for item in results.values()
                                            for r in item["mismatch"]}))), EXIT_MISMATCH
    elif len(matches) > 1:
        verdict, code = "无法判定：匹配到多块板卡 %s ⇒ 声明区分度不足（请补 decisive 指纹字段）" % matches, EXIT_UNDECIDABLE
    else:
        verdict, code = "匹配到唯一已声明板卡：%s" % matches[0], EXIT_OK
    return {"measured": measured, "declared": results, "matches": matches,
            "verdict": verdict, "exit_code": code}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="板卡指纹提取与交叉校验（只读；声明是权威，指纹是防呆）")
    parser.add_argument("--collect-dir", type=Path, required=True,
                        help="board_collect.sh 的产出目录（build/board/<name>）")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if not args.collect_dir.is_dir():
        print("[board-identify][错误] 采集目录不存在：%s" % args.collect_dir, file=sys.stderr)
        return EXIT_USAGE

    report = identify(args.repo_root.resolve(), args.collect_dir)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        measured = report["measured"]
        print("[board-identify] 实测指纹（来自 %s）" % measured["source"]["collect_dir"])
        for key in ("hyper_guests", "guest_vdisks", "guest_net_names", "cpu_cores",
                    "kernel_cmdline_tokens", "npu_stack_tokens", "iaros_components"):
            print("  %-24s %s" % (key, measured.get(key)))
        print("\n[board-identify] 与已声明 BoardProfile 比对：")
        for name, item in report["declared"].items():
            print("  == %s（%s，status=%s）" % (name, item["path"], item["status"]))
            for row in item["rows"]:
                flag = "✓" if row["ok"] else ("-" if row["ok"] is None else "✗")
                print("     %s %-24s 声明=%s 实测=%s %s"
                      % (flag, row["field"], row["expected"], row["actual"], row["note"]))
        print("\n结论：%s" % report["verdict"])
        print("提醒：构型事实来源仍是签名 HyperProfile/Release manifest（AGENTS 6.10）；本工具只做交叉校验。")
    return report["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
