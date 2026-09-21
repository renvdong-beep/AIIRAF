"""机型声明（运行时绑定）与「后端配置来源」的正/负路径测试。

契约：`robot.backend_config` 必须显式声明来源；支持 `mode: scene_report`（机械臂：后端配置取自
场景报告）。缺声明＝用声明文件本身（四足路径）。缺 report / 未知 mode / 缺容差 / realtime 非布尔
一律显式失败——抓取容差与是否实时步进都只来自声明，不允许实现层默认值。
"""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import scenario  # noqa: E402


class BuildBackendConfigTests(unittest.TestCase):
    def setUp(self):
        self.declaration_path = ROOT / "config" / "machines" / "piper.yaml"
        if not self.declaration_path.is_file():
            self.skipTest("机型声明缺失")
        self.document = yaml.safe_load(self.declaration_path.read_text(encoding="utf-8"))
        self.spec = copy.deepcopy(self.document["robot"]["backend_config"])
        self.report_path = ROOT / str(self.spec["report"])
        if not self.report_path.is_file():
            self.skipTest("piper 场景报告未生成（先跑构建入口）")

    def _build(self, spec=None):
        return scenario.build_backend_config(ROOT, self.document, spec or self.spec)

    def test_positive_builds_backend_config_from_scene_report(self):
        config = self._build()
        self.assertEqual(config["model_path"], json.loads(self.report_path.read_text())["output"])
        self.assertTrue(Path(config["model_path"]).is_file())
        self.assertEqual(set(config["manipulation"]["targets"]), {"box_01"})
        target = config["manipulation"]["targets"]["box_01"]
        self.assertEqual(target["body"], "box_01")
        self.assertEqual(target["pose_tolerance_m"], float(self.spec["target_tolerance_m"]))
        self.assertTrue(config["manipulation"]["gripper"])
        self.assertIs(config["realtime"], True)
        self.assertEqual(config["source_report"], str(self.report_path))

    def test_missing_report_is_refused(self):
        spec = dict(self.spec, report="build/models/does-not-exist.json")
        with self.assertRaises(scenario.ScenarioError) as ctx:
            self._build(spec)
        self.assertIn("场景报告不存在", str(ctx.exception))

    def test_unknown_mode_is_refused_and_lists_supported(self):
        spec = dict(self.spec, mode="whatever")
        with self.assertRaises(scenario.ScenarioError) as ctx:
            self._build(spec)
        self.assertIn("不受支持", str(ctx.exception))
        self.assertIn("scene_report", str(ctx.exception))

    def test_missing_tolerance_is_refused(self):
        spec = {k: v for k, v in self.spec.items() if k != "target_tolerance_m"}
        with self.assertRaises(scenario.ScenarioError) as ctx:
            self._build(spec)
        self.assertIn("target_tolerance_m", str(ctx.exception))

    def test_non_boolean_realtime_is_refused(self):
        spec = dict(self.spec, realtime="yes")
        with self.assertRaises(scenario.ScenarioError) as ctx:
            self._build(spec)
        self.assertIn("realtime", str(ctx.exception))

    def test_report_missing_gripper_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            broken = Path(tmp) / "report.json"
            payload = json.loads(self.report_path.read_text(encoding="utf-8"))
            payload.pop("gripper", None)
            broken.write_text(json.dumps(payload), encoding="utf-8")
            spec = dict(self.spec, report=str(broken))
            with self.assertRaises(scenario.ScenarioError) as ctx:
                self._build(spec)
            self.assertIn("gripper", str(ctx.exception))


class MachineDeclarationTests(unittest.TestCase):
    def test_scene_baseline_points_piper_at_runtime_binding(self):
        baseline = yaml.safe_load((ROOT / "scenes" / "handoff_lab" / "baseline.yaml").read_text(encoding="utf-8"))
        ref = (baseline.get("robots") or {}).get("piper")
        self.assertEqual(ref, "config/machines/piper.yaml",
                         "场景基线的 piper 必须指向运行时绑定声明，而不是构建基线")

    def test_runtime_binding_declares_backend_and_safety(self):
        document = yaml.safe_load((ROOT / "config" / "machines" / "piper.yaml").read_text(encoding="utf-8"))
        robot = document["robot"]
        self.assertEqual(robot["backend"], "mujoco_arm")
        self.assertEqual(robot["profile"], "profiles/piper_mujoco.yaml")
        self.assertTrue((document.get("skills") or {}).get("safety_policy"))
        # 后端入口必须已在 Backend 登记表里（禁止猜测入口名）
        from iraf_adapters.factory import KNOWN_BACKENDS

        self.assertIn(robot["backend"], KNOWN_BACKENDS)


if __name__ == "__main__":
    unittest.main()
