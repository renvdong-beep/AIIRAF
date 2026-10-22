# -*- coding: utf-8 -*-
"""装配单里的相对路径必须按版本目录解析（不依赖调用方 cwd）。

实机缺陷（2026-10-22，E300 智控域，0.3.10 装完 board_diag 自检）：
`IRAF_PROFILE=profiles/piper_mujoco.yaml` 是相对路径，`iraf-agentos --list-skills` 的 cwd
不是版本目录 ⇒ `FileNotFoundError: 'profiles/piper_mujoco.yaml'`，CLI 完全不可用。
systemd 单元靠 WorkingDirectory= 兜住，CLI / MCP / 探针各有各的 cwd，所以基准要在装配层给。
"""
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from iraf_adapters.bootstrap import resolve_path


class ResolvePathTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        (self.base / "profiles").mkdir()
        (self.base / "profiles" / "piper_mujoco.yaml").write_text("metadata: {}\n", encoding="utf-8")
        patcher = mock.patch.dict(os.environ, {"IRAF_VERSION_DIR": str(self.base)})
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_relative_resolved_under_version_dir(self):
        got = resolve_path("profiles/piper_mujoco.yaml")
        self.assertEqual(got, str(self.base / "profiles" / "piper_mujoco.yaml"))
        self.assertTrue(Path(got).is_file())

    def test_absolute_kept_as_is(self):
        p = str(self.base / "profiles" / "piper_mujoco.yaml")
        self.assertEqual(resolve_path(p), p)

    def test_missing_file_returns_original(self):
        """解析不到就原样返回：下游要报出装配单里的原始值，不能伪造一个绝对路径。"""
        self.assertEqual(resolve_path("profiles/not_here.yaml"), "profiles/not_here.yaml")

    def test_default_base_is_current_symlink(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("IRAF_VERSION_DIR", None)
            # 默认基准 /opt/iraf-sdk/current 在开发机上通常不存在 ⇒ 仍原样返回
            self.assertEqual(resolve_path("profiles/piper_mujoco.yaml"),
                             "profiles/piper_mujoco.yaml")

    def test_empty_value_passthrough(self):
        self.assertEqual(resolve_path(""), "")

    def test_runtime_build_uses_resolved_paths(self):
        """build_runtime_from_env 必须把装配单里的相对路径交给解析器（AST 层防回潮）。"""
        import ast
        import inspect

        import iraf_adapters.bootstrap as bootstrap

        tree = ast.parse(inspect.getsource(bootstrap))
        fn = next(n for n in tree.body
                  if isinstance(n, ast.FunctionDef) and n.name == "build_runtime_from_env")
        calls = [n for n in ast.walk(fn)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == "resolve_path"]
        self.assertGreaterEqual(len(calls), 4,
                                "build_runtime_from_env 里对装配单路径调用 resolve_path 的次数不足")


if __name__ == "__main__":
    unittest.main()
