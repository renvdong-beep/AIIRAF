# -*- coding: utf-8 -*-
"""Provider 服务端**产品入口**的契约（2026-10-26 入口下沉）。

为什么要有这些用例：SDK 的 runtime 交付包不含 `scripts/`（实测 `tar -tzf` 只有 src/ 与
deploy/sdk/{units,env}），所以"装完 SDK 就有 Provider"要求入口是**包内模块**
（`python -m iraf_adapters.remote.provider_cli`）。入口一旦回退成脚本路径，服务单元就会
在目标端 `ExecStart` 直接失败——那是装完才发现的问题，必须在这里拦住。

本文件不 import grpc/proto（--help 与 fail-closed 分支都在建链之前），因此任何环境都能跑。
"""
from __future__ import annotations

import os
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from unittest import mock

from iraf_adapters.remote import provider_cli


class EntryPointContractTest(unittest.TestCase):
    def test_module_entry_help_works(self):
        """`python -m iraf_adapters.remote.provider_cli --help` 必须可用（服务单元靠它自检）。"""
        buf = StringIO()
        with redirect_stdout(buf), self.assertRaises(SystemExit) as ctx:
            provider_cli.main(["--help"])
        self.assertEqual(ctx.exception.code, 0)
        text = buf.getvalue()
        for flag in ("--host", "--port", "--no-window", "--self-check"):
            self.assertIn(flag, text)

    def test_missing_token_fails_closed_before_listening(self):
        """缺 token 必须以退出码 2 结束（不监听端口、不伪造可用）。"""
        env = {k: v for k, v in os.environ.items() if k != "IRAF_PROVIDER_TOKEN"}
        out, err = StringIO(), StringIO()
        with mock.patch.dict(os.environ, env, clear=True), redirect_stdout(out), redirect_stderr(err):
            rc = provider_cli.main([])
        self.assertEqual(rc, provider_cli.EXIT_NOTOKEN)
        self.assertIn("IRAF_PROVIDER_TOKEN", out.getvalue() + err.getvalue())

    def test_thin_script_forwards_to_module(self):
        """`scripts/iraf_provider_serve.py` 只做转发：不得再出现自己的 argparser/逻辑。"""
        from pathlib import Path

        text = (Path(__file__).resolve().parents[2] / "scripts" / "iraf_provider_serve.py").read_text(
            encoding="utf-8")
        self.assertIn("from iraf_adapters.remote.provider_cli import main", text)
        self.assertNotIn("argparse", text, "薄壳里不应再出现自己的参数解析（会与包内入口分叉）")
        self.assertNotIn("def self_check", text)


class DisplayModeDeclarationTest(unittest.TestCase):
    """装配单决定显示模式（安装后的服务单元不给命令行参数）。"""

    class _Args:
        def __init__(self, no_window=False):
            self.no_window = no_window

    def test_headless_from_env_forces_no_window(self):
        for value in ("headless", "offscreen", "offscreen_frames", "HEADLESS"):
            with self.subTest(value=value), mock.patch.dict(os.environ, {"IRAF_PROVIDER_DISPLAY_MODE": value}):
                args = self._Args()
                self.assertTrue(provider_cli.apply_display_env(args))
                self.assertTrue(args.no_window)

    def test_window_mode_is_not_overridden(self):
        with mock.patch.dict(os.environ, {"IRAF_PROVIDER_DISPLAY_MODE": "window"}):
            args = self._Args()
            self.assertFalse(provider_cli.apply_display_env(args))
            self.assertFalse(args.no_window, "声明 window 时不得被静默改成无窗口")

    def test_absent_declaration_keeps_parser_decision(self):
        env = {k: v for k, v in os.environ.items() if k != "IRAF_PROVIDER_DISPLAY_MODE"}
        with mock.patch.dict(os.environ, env, clear=True):
            args = self._Args(no_window=True)
            self.assertFalse(provider_cli.apply_display_env(args))
            self.assertTrue(args.no_window)


if __name__ == "__main__":
    unittest.main()
