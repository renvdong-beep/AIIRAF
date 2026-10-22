# -*- coding: utf-8 -*-
"""provider token 的受控来源（环境变量 → 600 文件 → fail-closed）。

实机（2026-10-22，E300 智控域 0.3.11 装完）：板上 runtime 始终起不来，
`board_diag` 三项红同因 `ValueError: token <redacted>` —— 装配单按设计不写凭据，
systemd 也没拿到 token，而 CLI / 探针不走 systemd。所以给 600 文件一条路，
但**优先级仍是环境变量优先**，且两处都没有时依旧 fail-closed。
"""
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from iraf_adapters.remote.provider_backend import RemoteProviderBackend, read_token_file


class ReadTokenFileTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)

    def _write(self, text, name="provider.token"):
        p = self.dir / name
        p.write_text(text, encoding="utf-8")
        return p

    def test_key_value_form(self):
        self.assertEqual(read_token_file(self._write("IRAF_PROVIDER_TOKEN=abc123\n")), "abc123")

    def test_bare_value_form(self):
        self.assertEqual(read_token_file(self._write("abc123\n")), "abc123")

    def test_comments_and_blank_lines_skipped(self):
        self.assertEqual(read_token_file(self._write("# 注释\n\nIRAF_PROVIDER_TOKEN='q-1'\n")), "q-1")

    def test_missing_file_returns_empty(self):
        self.assertEqual(read_token_file(self.dir / "nope.token"), "")

    def test_unrelated_keys_do_not_leak(self):
        self.assertEqual(read_token_file(self._write("OTHER=1\nIRAF_PROVIDER_TOKEN=ok\n")), "ok")


class FromConfigTokenTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.token_file = self.dir / "provider.token"
        self.token_file.write_text("IRAF_PROVIDER_TOKEN=from-file\n", encoding="utf-8")
        self.cfg = {"endpoint": "10.0.0.9:50052", "token_file": str(self.token_file)}

    def test_env_wins_over_file(self):
        with mock.patch.dict(os.environ, {"IRAF_PROVIDER_TOKEN": "from-env"}, clear=False):
            be = RemoteProviderBackend.from_config(self.cfg, None, None)
        self.assertEqual(be._token, "from-env")

    def test_file_used_when_env_absent(self):
        env = {k: v for k, v in os.environ.items() if k != "IRAF_PROVIDER_TOKEN"}
        with mock.patch.dict(os.environ, env, clear=True):
            be = RemoteProviderBackend.from_config(self.cfg, None, None)
        self.assertEqual(be._token, "from-file")

    def test_fail_closed_when_no_source(self):
        cfg = {"endpoint": "10.0.0.9:50052"}
        env = {k: v for k, v in os.environ.items()
               if k not in ("IRAF_PROVIDER_TOKEN", "IRAF_PROVIDER_TOKEN_FILE")}
        with mock.patch.dict(os.environ, env, clear=True):
            with self.assertRaises(ValueError):
                RemoteProviderBackend.from_config(cfg, None, None)

    def test_direct_constructor_still_fail_closed(self):
        with self.assertRaises(ValueError):
            RemoteProviderBackend("10.0.0.9:50052", "")

    def test_env_token_file_variable_is_honoured(self):
        cfg = {"endpoint": "10.0.0.9:50052"}
        env = {k: v for k, v in os.environ.items() if k != "IRAF_PROVIDER_TOKEN"}
        env["IRAF_PROVIDER_TOKEN_FILE"] = str(self.token_file)
        with mock.patch.dict(os.environ, env, clear=True):
            be = RemoteProviderBackend.from_config(cfg, None, None)
        self.assertEqual(be._token, "from-file")


if __name__ == "__main__":
    unittest.main()
