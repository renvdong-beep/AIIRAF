# -*- coding: utf-8 -*-
"""AgentOS worker MCP 配置合并（纯逻辑，不依赖 docker/AgentOS）。"""
import tempfile
import unittest
from pathlib import Path

import lib_agentos_mcp as lib


class SharedDirTest(unittest.TestCase):
    def test_find_bind_mount_source(self):
        payload = {"Mounts": [
            {"Type": "bind", "Source": "/", "Destination": "/host"},
            {"Type": "bind", "Source": "/var/lib/agentos-shared/.intewell",
             "Destination": "/home/agentos/.intewell"},
        ]}
        self.assertEqual(lib.discover_shared_dir(payload),
                         "/var/lib/agentos-shared/.intewell")

    def test_missing_mount_returns_empty(self):
        self.assertEqual(lib.discover_shared_dir({"Mounts": []}), "")
        self.assertEqual(lib.discover_shared_dir({}), "")


class MergeTest(unittest.TestCase):
    def test_merge_preserves_other_keys_and_servers(self):
        original = {"model": {"name": "x"}, "mcp_servers": {"other": {"url": "http://o"}}}
        merged = lib.merge_server(original, name="iraf-task",
                                  url="http://board:8765/mcp", token="t0ken")
        self.assertEqual(merged["model"], {"name": "x"})
        self.assertIn("other", merged["mcp_servers"])
        self.assertEqual(merged["mcp_servers"]["iraf-task"]["url"],
                         "http://board:8765/mcp")
        self.assertEqual(merged["mcp_servers"]["iraf-task"]["headers"]["Authorization"],
                         "Bearer t0ken")
        self.assertTrue(merged["mcp_servers"]["iraf-task"]["enabled"])
        # 原对象不被修改
        self.assertNotIn("iraf-task", original["mcp_servers"])

    def test_merge_is_idempotent(self):
        first = lib.merge_server({}, name="iraf-task", url="u", token="t")
        second = lib.merge_server(first, name="iraf-task", url="u", token="t")
        self.assertEqual(first, second)

    def test_invalid_mcp_servers_fails_closed(self):
        with self.assertRaises(ValueError):
            lib.merge_server({"mcp_servers": ["bad"]}, name="iraf-task", url="u", token="t")

    def test_remove_server_only_removes_target(self):
        data = {"mcp_servers": {"iraf-task": {"url": "u"}, "other": {"url": "o"}}}
        result, changed = lib.remove_server(data, name="iraf-task")
        self.assertTrue(changed)
        self.assertNotIn("iraf-task", result["mcp_servers"])
        self.assertIn("other", result["mcp_servers"])

    def test_remove_last_server_drops_empty_section(self):
        data = {"mcp_servers": {"iraf-task": {"url": "u"}}}
        result, changed = lib.remove_server(data, name="iraf-task")
        self.assertTrue(changed)
        self.assertNotIn("mcp_servers", result)

    def test_remove_missing_is_noop(self):
        data = {"mcp_servers": {"other": {"url": "o"}}}
        result, changed = lib.remove_server(data, name="iraf-task")
        self.assertFalse(changed)
        self.assertEqual(result, data)


class ConfigFileTest(unittest.TestCase):
    def test_roundtrip_and_atomic_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.yaml"
            data = lib.merge_server({"model": {"name": "x"}}, name="iraf-task",
                                    url="http://board:8765/mcp", token="t")
            lib.write_config(path, data, owner=None)
            loaded = lib.load_config(path)
            self.assertEqual(loaded["mcp_servers"]["iraf-task"]["url"],
                             "http://board:8765/mcp")
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(list(Path(tmp).glob(".config.yaml.*")), [])

    def test_missing_file_loads_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(lib.load_config(Path(tmp) / "missing.yaml"), {})

    def test_non_object_config_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.yaml"
            path.write_text("- a\n- b\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                lib.load_config(path)


if __name__ == "__main__":
    unittest.main()
