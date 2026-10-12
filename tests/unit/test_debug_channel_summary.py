# -*- coding: utf-8 -*-
"""报告「调试通路」留痕的回归测试（P8 / C2，2026-10-12）。

锁住：clean 判定（无开关 ⇒ clean=True）、激活判定（非空且非 "0"）、
以及**不泄露取值**（返回里只有开关名，没有值）。
"""
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import scenario  # noqa: E402
from scenario import KNOWN_DEBUG_CHANNELS, _debug_channel_summary  # noqa: E402


class DebugChannelSummaryTests(unittest.TestCase):
    def test_clean_when_no_channels(self):
        with patch.dict(os.environ, {}, clear=True):
            summary = _debug_channel_summary()
        self.assertTrue(summary["clean"])
        self.assertEqual(summary["active_channels"], [])

    def test_active_when_channel_set(self):
        with patch.dict(os.environ, {"IRAF_DEBUG_PLACE": "1"}, clear=True):
            summary = _debug_channel_summary()
        self.assertFalse(summary["clean"])
        self.assertEqual(summary["active_channels"], ["IRAF_DEBUG_PLACE"])

    def test_zero_and_empty_are_not_active(self):
        with patch.dict(os.environ, {"IRAF_DEBUG_PLACE": "0", "IRAF_DEBUG_PICK": ""}, clear=True):
            summary = _debug_channel_summary()
        self.assertTrue(summary["clean"])

    def test_does_not_leak_values(self):
        """只列开关名，不写取值（避免敏感值随报告落盘）。"""
        with patch.dict(os.environ, {"IRAF_DEBUG_CTRL_DETAIL": "s06_unload_at_b"}, clear=True):
            summary = _debug_channel_summary()
        blob = repr(summary)
        self.assertIn("IRAF_DEBUG_CTRL_DETAIL", blob)
        self.assertNotIn("s06_unload_at_b", blob)

    def test_unknown_switch_is_ignored(self):
        with patch.dict(os.environ, {"IRAF_DEBUG_NOT_REGISTERED": "1"}, clear=True):
            summary = _debug_channel_summary()
        self.assertTrue(summary["clean"])
        self.assertNotIn("IRAF_DEBUG_NOT_REGISTERED", summary["active_channels"])

    def test_known_channels_is_a_tuple_of_names(self):
        self.assertIsInstance(KNOWN_DEBUG_CHANNELS, tuple)
        for name in KNOWN_DEBUG_CHANNELS:
            self.assertTrue(name.startswith("IRAF_DEBUG_"), name)


if __name__ == "__main__":
    unittest.main()
