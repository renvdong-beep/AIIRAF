#!/usr/bin/env python3
"""wheelhouse 目录修剪：只保留抓取脚本自己登记的 wheel，删掉陈旧残留。

================================ 为什么需要 ================================
`fetch_wheelhouse.sh` 只负责按声明解析并下载，**不清理**上一次抓取留下、这次不再
被选中的 wheel。实测踩到：声明回填后重抓，protobuf 同时存在 7.36.1（上轮残留）与
7.36.2（本轮选中）两份 —— 随 bundle 一起打进目标端，`pip --no-index --find-links`
会自行挑最新，dev 与目标端可能装的不是同一份，且"manifest 记录了哪些文件"与
"目录里实际有什么"不再一致。

事实来源：同一目录下的 `wheelhouse.json`（抓取脚本写的 `planned[].filename`）。
本工具只删"该目录里存在、但不在登记清单中"的 `*.whl`，其余一律不动。

退出码：0 成功（含无需修剪）；1 参数错误；2 预检失败（目录/清单缺失、清单为空）；
        3 清理失败（删除出错）；4 校验失败（--verify：存在未登记的 wheel）。
用法：
  /usr/bin/python3 deploy/sdk/prune_wheelhouse.py --dir build/wheelhouse/<target-id> [--dry-run] [--verify]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


class PruneError(Exception):
    """显式失败。"""


def load_registered_names(directory: Path) -> set:
    manifest = directory / "wheelhouse.json"
    if not manifest.is_file():
        raise PruneError(f"缺少抓取清单：{manifest}（先用 fetch_wheelhouse.sh 抓取）")
    data = json.loads(manifest.read_text(encoding="utf-8"))
    planned = data.get("planned")
    if not isinstance(planned, list) or not planned:
        raise PruneError(f"抓取清单没有 planned 条目：{manifest}（不能据此判断该删什么）")
    names = set()
    for item in planned:
        filename = item.get("filename")
        if filename:
            names.add(str(filename))
    if not names:
        raise PruneError(f"抓取清单里没有任何 resolved 的 filename：{manifest}")
    return names


def find_stale(directory: Path, registered: set) -> list:
    if not directory.is_dir():
        raise PruneError(f"目录不存在：{directory}")
    return sorted(p for p in directory.glob("*.whl") if p.name not in registered)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="wheelhouse 陈旧 wheel 修剪（只删未登记项）")
    parser.add_argument("--dir", type=Path, required=True, help="wheelhouse 目录")
    parser.add_argument("--dry-run", action="store_true", help="只报告，不删除")
    parser.add_argument("--verify", action="store_true", help="只校验：存在未登记 wheel 即退出码 4")
    args = parser.parse_args(argv)

    try:
        registered = load_registered_names(args.dir)
        stale = find_stale(args.dir, registered)
    except PruneError as exc:
        print(f"[wheelhouse 修剪失败] {exc}", file=sys.stderr)
        return 2

    total = len(list(args.dir.glob("*.whl")))
    print(f"[wheelhouse 修剪] 目录：{args.dir}")
    print(f"  登记 wheel {len(registered)} 个；目录内 wheel {total} 个；未登记 {len(stale)} 个")

    if args.verify:
        for path in stale:
            print(f"  未登记：{path.name}")
        if stale:
            print("[wheelhouse 校验失败] 存在未被抓取清单登记的 wheel（目录与清单不一致）", file=sys.stderr)
            return 4
        print("  校验通过：目录与抓取清单一致")
        return 0

    if not stale:
        print("  无需修剪")
        return 0

    for path in stale:
        if args.dry_run:
            print(f"  [dry-run] 将删除：{path.name}")
            continue
        try:
            size = path.stat().st_size
            path.unlink()
        except OSError as exc:
            print(f"[wheelhouse 修剪失败] 删除 {path.name} 出错：{exc}", file=sys.stderr)
            return 3
        print(f"  已删除：{path.name}（{size} 字节）")
    remaining = sorted(p.name for p in args.dir.glob("*.whl"))
    print(f"  修剪后 wheel {len(remaining)} 个")
    return 0


if __name__ == "__main__":
    sys.exit(main())
