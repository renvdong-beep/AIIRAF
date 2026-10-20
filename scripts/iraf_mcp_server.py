#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[兼容入口] 任务级 MCP server —— 实现已迁到 `iraf_adapters.mcp.server`。

为什么保留：板卡上早期的 `mcp add` 登记指向本文件路径；转发可以避免现场立刻改配置。
新配置请直接使用模块：
    intewell-agent-cli mcp add iraf-task --command /usr/bin/python3 \
      --args -m iraf_adapters.mcp.server
"""
import sys
from pathlib import Path

VERSION_DIR = Path("/opt/iraf-sdk/current")
for _extra in (str(VERSION_DIR / "lib"), str(VERSION_DIR / "src")):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)

print("[deprecated] scripts/iraf_mcp_server.py 已由 `python3 -m iraf_adapters.mcp.server` 取代",
      file=sys.stderr)

from iraf_adapters.mcp.server import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
