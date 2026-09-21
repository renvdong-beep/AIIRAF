"""技能租约 TTL 门禁（profile_check --quadruped 第 8 条）的正/负路径测试。

背景（实测）：`skill.yaml` 的 `timeoutSeconds` 直接作为 `ControlAuthorityManager` 的租约 TTL；
如果它小于机型声明的动作时长，长动作会在执行中途报 `LeaseConflict: lease expired`。
真实事故：stop 声明 TTL=2 s 而 stop.duration_s=3.0 s，快路径墙钟仅约 0.5 s 所以长期未暴露，
挂上 MuJoCo 窗口后被拖长才暴露。本测试钉住"TTL ≥ 声明时长"这条规则。
"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


profile_check = _load(ROOT / "scripts" / "profile_check.py", "profile_check_mod")


class SkillTtlGateTests(unittest.TestCase):
    def setUp(self):
        self.declaration_path = ROOT / "config" / "go2_loopback.yaml"
        if not self.declaration_path.is_file():
            self.skipTest("四足声明缺失")
        self.document = yaml.safe_load(self.declaration_path.read_text(encoding="utf-8"))

    def _run_with(self, mutate):
        document = yaml.safe_load(yaml.safe_dump(self.document))
        mutate(document)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "declaration.yaml"
            path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
            return profile_check.check_quadruped(str(path), root=ROOT)

    def test_current_declaration_passes_and_records_both_capabilities(self):
        report = self._run_with(lambda doc: None)
        self.assertTrue(report["passed"], report["failures"])
        gate = {item["capability"]: item for item in report["skill_ttl_gate"]}
        self.assertEqual(set(gate), {"stand", "stop"})
        for item in gate.values():
            self.assertGreaterEqual(item["timeout_seconds"], item["declared_duration_s"])

    def test_ttl_shorter_than_declared_duration_fails_with_actionable_reason(self):
        # 仿真时长拉到超过 stop 的租约 TTL（10 s）⇒ 门禁必须拒绝，并点出 lease expired 的后果
        report = self._run_with(lambda doc: doc["stop"].__setitem__("duration_s", 30.0))
        self.assertFalse(report["passed"])
        joined = "；".join(report["failures"])
        self.assertIn("覆盖不了机型声明的动作时长", joined)
        self.assertIn("lease expired", joined)

    def test_missing_duration_key_fails_instead_of_defaulting(self):
        report = self._run_with(lambda doc: doc["stand"].pop("duration_s"))
        self.assertFalse(report["passed"])
        self.assertIn("stand.duration_s", "；".join(report["failures"]))


if __name__ == "__main__":
    unittest.main()
