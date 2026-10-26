#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容壳：Provider 服务端入口已下沉到 `iraf_adapters.remote.provider_cli`（2026-10-26）。

为什么还要这个文件：开发/调试链路（scripts/dev_provider_up.sh、dev_reset_provider.py、
docs 里的用法）都按 `scripts/iraf_provider_serve.py` 这个路径调用，保留它以免打断已验收的流程。

区别与边界
----------
· **产品入口**（装完 SDK 后的 systemd 单元）= `python -m iraf_adapters.remote.provider_cli`；
  交付包不含 `scripts/`，所以服务单元**不能**指向本文件。
· 本文件只做转发，不含任何逻辑；改行为请改 `src/iraf_adapters/remote/provider_cli.py`。
"""
from __future__ import annotations

import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO / "src") not in sys.path:
    sys.path.insert(0, str(_REPO / "src"))

from iraf_adapters.remote.provider_cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
