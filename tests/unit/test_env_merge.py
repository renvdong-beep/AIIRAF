# -*- coding: utf-8 -*-
"""安装器升级保留站点 env 键（2026-11-11：0.4.4 升级曾把 PROFILE/SAFETY_POLICY 抹空）。"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "deploy" / "sdk"))

import lib_install  # noqa: E402


class EnvMergeTest(unittest.TestCase):
    def test_empty_rendered_keeps_previous(self):
        rendered = "IRAF_PROFILE=\nIRAF_SAFETY_POLICY=\n"
        previous = "IRAF_PROFILE=profiles/piper_mujoco.yaml\nIRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml\n"
        out = lib_install._merge_env_text(rendered, previous)
        self.assertIn("IRAF_PROFILE=profiles/piper_mujoco.yaml", out)
        self.assertIn("IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml", out)

    def test_declared_non_empty_wins(self):
        rendered = "IRAF_MCP_TRANSPORT=http\n"
        previous = "IRAF_MCP_TRANSPORT=stdio\n"
        out = lib_install._merge_env_text(rendered, previous)
        self.assertIn("IRAF_MCP_TRANSPORT=http", out)
        self.assertNotIn("IRAF_MCP_TRANSPORT=stdio", out)

    def test_site_only_keys_are_appended(self):
        rendered = "IRAF_PROFILE=profiles/piper_mujoco.yaml\n"
        previous = ("IRAF_PROFILE=profiles/piper_mujoco.yaml\n"
                    "IRAF_DEFAULT_SCENE=piper-pick\n"
                    "IRAF_AGENTOS_MODEL=deepseek-flash\n")
        out = lib_install._merge_env_text(rendered, previous)
        self.assertIn("IRAF_DEFAULT_SCENE=piper-pick", out)
        self.assertIn("IRAF_AGENTOS_MODEL=deepseek-flash", out)

    def test_allowed_hosts_are_unioned(self):
        rendered = "IRAF_MCP_ALLOWED_HOSTS=127.0.0.1:8765,localhost:8765\n"
        previous = "IRAF_MCP_ALLOWED_HOSTS=127.0.0.1:8765,10.203.247.147:8765\n"
        out = lib_install._merge_env_text(rendered, previous)
        self.assertIn("localhost:8765", out)
        self.assertIn("10.203.247.147:8765", out)

    def test_no_previous_is_noop(self):
        rendered = "IRAF_PROFILE=\n"
        self.assertEqual(rendered, lib_install._merge_env_text(rendered, ""))


if __name__ == "__main__":
    unittest.main()
