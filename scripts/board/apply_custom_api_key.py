#!/usr/bin/env python3
"""在板卡上把 CUSTOM_API_KEY 写入 AgentOS deploy/.env（含备份、打码回显）。

用法：apply_custom_api_key.py <key_file> <env_file>
只改 CUSTOM_API_KEY 一行；其它行逐字节保留。原文件备份为 <env_file>.bak-<ts>。
"""
import shutil
import sys
import time
from pathlib import Path


def mask(v: str) -> str:
    return f"len={len(v)} head={v[:6]}***"


def main() -> int:
    if len(sys.argv) != 3:
        print("用法: apply_custom_api_key.py <key_file> <env_file>")
        return 2
    key_file = Path(sys.argv[1])
    env_file = Path(sys.argv[2])
    if not key_file.is_file():
        print(f"FAIL 读不到 key 文件: {key_file}")
        return 3
    if not env_file.is_file():
        print(f"FAIL 读不到 env 文件: {env_file}")
        return 4

    key = key_file.read_text(encoding="utf-8").strip()
    if not key or "\n" in key or " " in key:
        print("FAIL key 为空或含空白/换行")
        return 5

    lines = env_file.read_text(encoding="utf-8").splitlines(keepends=True)
    hit = 0
    out = []
    old_masked = "none"
    for line in lines:
        if line.startswith("CUSTOM_API_KEY="):
            old_masked = mask(line.strip().split("=", 1)[1])
            out.append(f"CUSTOM_API_KEY={key}\n")
            hit += 1
        else:
            out.append(line)
    if hit != 1:
        print(f"FAIL 期望恰好 1 行 CUSTOM_API_KEY=，实际 {hit}")
        return 6

    backup = env_file.with_name(env_file.name + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(env_file, backup)
    env_file.write_text("".join(out), encoding="utf-8")
    env_file.chmod(0o600)

    now = env_file.read_text(encoding="utf-8")
    new_val = [ln for ln in now.splitlines() if ln.startswith("CUSTOM_API_KEY=")][0].split("=", 1)[1]
    print(f"OK env={env_file}")
    print(f"OK backup={backup}")
    print(f"OLD {old_masked}")
    print(f"NEW {mask(new_val)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
