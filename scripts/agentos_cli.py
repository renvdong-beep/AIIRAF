#!/usr/bin/env python3
"""IRAF × AgentOS 交互式 CLI 入口（仓库根可跑；安装后亦可 `iraf-agentos`）。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/agentos_cli.py                 # 交互 REPL
    PYTHONPATH=src python3 scripts/agentos_cli.py --text "让机器狗站起来"
    PYTHONPATH=src python3 scripts/agentos_cli.py --list-skills
    PYTHONPATH=src python3 scripts/agentos_cli.py --json           # 机器模式（AgentOS CLI 绑定用）

本文件只是入口层（与 `scripts/scenario.py` / `scripts/build_scene.py` 同风格）：
解析参数、把 `src/` 挂到 import path、转发给 `iraf_adapters.agentos.cli`。
所有逻辑（请求构建 / 身份 / 结果渲染 / 退出码）都在该模块里，便于单测。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.agentos.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
