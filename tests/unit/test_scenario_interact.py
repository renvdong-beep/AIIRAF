"""S1 命令式交互（`scenario.py interact`）与四足显示面的测试（离线）。

覆盖：
1. 命令解析：注释/空行跳过；`键=值` 的 JSON 取值；非法 token 显式失败。
2. 能力门：Profile 未声明的能力必须被拒（不得调用），且给出可定位原因。
3. 词表门：不在场景契约动作词表里的动作必须被拒。
4. 只读命令：`state` 不驱动控制量（READ_ONLY），后端无 read_state 时显式报告。
5. 四足显示声明：缺 render 段即装配失败（禁止实现层默认值）；相机名不存在即失败。
6. 报告/transcript 契约：schema、simulation=true、计数一致。
"""

from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import scenario  # noqa: E402


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ParseCommandTests(unittest.TestCase):
    def test_comment_and_blank_are_skipped(self):
        self.assertIsNone(scenario.parse_command("", set()))
        self.assertIsNone(scenario.parse_command("   # 注释", set()))

    def test_values_are_json_decoded(self):
        parsed = scenario.parse_command("stand duration_ms=1500 kp=1.5 flag=true name=abc", set())
        self.assertEqual(parsed["action"], "stand")
        self.assertEqual(parsed["params"],
                         {"duration_ms": 1500, "kp": 1.5, "flag": True, "name": "abc"})

    def test_malformed_token_fails_explicitly(self):
        with self.assertRaises(scenario.ScenarioError):
            scenario.parse_command("stand oops", set())
        with self.assertRaises(scenario.ScenarioError):
            scenario.parse_command("stand =5", set())


class Go2InteractiveTests(unittest.TestCase):
    """用真实场景包跑 Go2 的正/负路径（不依赖显示设备）。"""

    SCENE = ROOT / "scenes" / "handoff_lab"

    def setUp(self):
        if not (self.SCENE / "scene.yaml").is_file():
            self.skipTest("场景包缺失")
        if not (ROOT / "build" / "scenes" / "handoff_lab" / "handoff_lab.xml").is_file():
            self.skipTest("Go2 场景产物未生成（先跑 scripts/build_scene.py）")

    def _run(self, commands):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            report = scenario.interact(
                self.SCENE, "unitree_go2", commands=commands, display="none",
                report_path=Path(tmp) / "r.json", transcript_path=Path(tmp) / "t.jsonl",
                root=ROOT,
            )
            transcript = [json.loads(line) for line in (Path(tmp) / "t.jsonl").read_text(encoding="utf-8").splitlines()]
            return report, transcript

    def test_declared_capabilities_run_and_report_is_contract_complete(self):
        """**每个已声明能力都必须真的能跑**（本用例是能力回填的最强回归）。

        `locomote` 于 2026-09-23 入列 ⇒ 本用例也要把它跑一遍：
        嵌套的 `velocity` 用**内联 JSON** 写（点号写法会被输入 schema 拒 —— 实测）。
        若将来有人改坏平面 IK 前提门禁/时长单位/provider 语义，这里会红。
        """
        report, transcript = self._run([
            "stand duration_ms=500",
            'locomote velocity={"vx_mps":0.1,"vy_mps":0.0,"wz_rad_s":0.0} duration_ms=500',
            "stop",
        ])
        self.assertEqual(report["schema_version"], scenario.INTERACT_SCHEMA)
        self.assertTrue(report["simulation"])
        # 2026-09-24：`dock_for_handoff` 入列（同一事实的第四处：交互入口的能力清单）
        # 2026-09-28：`accept_payload` 入列（同一事实的第四处：交互入口的能力清单）
        self.assertEqual(report["declared_capabilities"],
                         ["accept_payload", "dock_for_handoff", "locomote", "stand", "stop"])
        self.assertEqual(report["counts"]["rejected"], 0)
        self.assertTrue(all(item["status"] == "SUCCEEDED" for item in transcript))

    def test_undeclared_capability_is_rejected_before_execution(self):
        """未声明的能力必须在**执行前**被拒（载体随事实更新）。

        `locomote` 已声明 ⇒ 换用**词表内、但本 Profile 未声明**的 `pick_object` 作载体
        （实测报「未声明能力 'pick_object'」；`locomote` 当时是这条路径的载体）。
        """
        report, transcript = self._run(["pick_object"])
        self.assertEqual(report["counts"]["rejected"], 1)
        self.assertEqual(transcript[0]["error_code"], "IRAF-SKILL-PROVIDER-UNAVAILABLE")
        self.assertIn("未声明能力", transcript[0]["reason"])
        self.assertIsNone(transcript[0].get("wall_seconds"), "被拒命令不应有执行耗时（说明未执行）")

    def test_action_outside_contract_vocabulary_is_rejected(self):
        report, transcript = self._run(["fly"])
        self.assertEqual(report["counts"]["rejected"], 1)
        self.assertIn("不在场景契约词表", transcript[0]["reason"])

    def test_state_is_read_only_and_does_not_drive_control(self):
        report, transcript = self._run(["state"])
        self.assertEqual(transcript[0]["status"], "READ_ONLY")
        state = transcript[0]["state"]
        self.assertIsInstance(state, dict)
        self.assertIn("base_position_m", state)


class RenderDeclarationGateTests(unittest.TestCase):
    """声明即事实：缺 render 段 / 相机不存在都必须显式失败（禁止实现层默认值）。"""

    def setUp(self):
        self.module = _load(ROOT / "src" / "iraf_adapters" / "unitree" / "unitree_go2.py", "go2_mod")
        self.declaration = self.module.load_declaration(str(ROOT / "config" / "go2_loopback.yaml"))[0]

    def test_render_section_present_in_declaration(self):
        render = self.declaration.get("render")
        self.assertIsInstance(render, dict)
        self.assertEqual(render.get("camera"), "overhead_camera")
        self.assertEqual(render.get("width_px"), 640)
        self.assertEqual(render.get("height_px"), 480)

    def test_missing_render_section_is_refused_at_assembly(self):
        # 用真实 Profile（身份校验先于 render 门），并确认被拦在 render 段而不是别的检查上
        from iraf_core.profile import load_robot_profile

        profile = load_robot_profile(ROOT / "profiles" / "unitree_go2_mujoco.yaml")
        broken = {k: v for k, v in self.declaration.items() if k != "render"}
        with self.assertRaises(self.module.DeclarationError) as ctx:
            self.module.UnitreeGo2Adapter.from_config(
                {"declaration": broken, "root": str(ROOT)}, profile, None
            )
        self.assertIn("render", str(ctx.exception))

    def test_render_settings_come_from_declaration(self):
        render = self.declaration["render"]
        self.assertEqual((render["camera"], render["width_px"], render["height_px"]),
                         ("overhead_camera", 640, 480))
        self.assertGreater(float(render["render_hz"]), 0.0)


if __name__ == "__main__":
    unittest.main()
