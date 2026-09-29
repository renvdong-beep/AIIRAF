"""联合报告的**所有权**门禁：跨本体错绑必须在装配前被拦下（2026-09-29 多臂场景引入）。

背景（为什么必须有这条测试）：联合报告顶层的 `manipulation`/`gripper`/`targets` 只描述
`scene.model.joint_manipulator` 指定的**那一台**臂。第二台臂（ur5e）的机型声明若把
`backend_config.report` 指向同一份联合报告，装配出来的后端会去驱动**另一台臂**的执行器与指腹，
而且**不会报错**：ur5e 的 Profile 里 `left_finger_geom` 解析出的 `piper_left_finger`
在联合模型里**确实存在** ⇒ 夹爪"张开/闭合"实际作用在 piper 上。

本模块只做一件事：把"报告描述的是别的本体"这条判据的正反两面钉住（缺少正向对照的话，
一个"永远抛异常"的实现也能让负向用例通过）。
"""

import json
import pathlib
import sys
import tempfile
import unittest

# 与 tests/unit/test_scenario_runner.py 同一约定：`scripts/` 必须在 sys.path 上
# （scenario.py 里有同目录的入口层导入 `import scene_check`）。
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import scenario as scenario_runner  # noqa: E402


def _write_minimal_report(root, attached_robot, attached_manipulators):
    """最小可装配报告：只需 output/gripper/targets/manipulation（其余由装配层判空）。"""
    model = root / "model.xml"
    model.write_text("<mujoco/>\n", encoding="utf-8")
    report = {
        "output": str(model),
        "gripper": {"left_finger_geom": "piper_left_finger",
                    "right_finger_geom": "piper_right_finger"},
        "targets": [{"id": "box_01", "geom": "box_01_geom"}],
        "manipulation": {
            "attached_robot": attached_robot,
            "attached_manipulators": attached_manipulators,
            "name_map": {"joint1": "piper_joint1"},
        },
    }
    path = root / "joint-report.json"
    path.write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")
    return path


def _declaration(robot_id, report_path):
    return {
        "schema_version": "iraf.machine-declaration/v1",
        "robot": {
            "id": robot_id,
            "profile": "profiles/ur5_mujoco.yaml",
            "backend": "mujoco_arm",
            "backend_config": {
                "mode": "scene_report",
                "report": str(report_path),
                "target_tolerance_m": 0.005,
                "realtime": True,
            },
        },
    }


class JointReportOwnershipTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_other_robot_report_is_rejected(self):
        """报告描述的是别的本体 ⇒ 装配前显式失败（退出码 3 / EXIT_DECLARATION）。"""
        report = _write_minimal_report(
            self.root, "piper",
            [{"id": "piper", "resolved": True}, {"id": "ur5e", "resolved": False}])
        declaration = _declaration("ur5e", report)
        with self.assertRaises(scenario_runner.ScenarioError) as ctx:
            scenario_runner.build_backend_config(
                self.root, declaration, declaration["robot"]["backend_config"])
        self.assertEqual(ctx.exception.code, scenario_runner.EXIT_DECLARATION)
        message = str(ctx.exception)
        # 消息必须让人能直接行动：指出是谁的报告、以及"本本体未解"这个显式状态
        self.assertIn("另一台臂", message)
        self.assertIn("ur5e", message)
        self.assertIn("resolved=False", message)

    def test_own_report_is_accepted(self):
        """正向对照：报告描述的正是本本体 ⇒ 正常装配（否则上面那条用例可能被恒失败实现骗过）。"""
        report = _write_minimal_report(
            self.root, "ur5e",
            [{"id": "ur5e", "resolved": True}])
        declaration = _declaration("ur5e", report)
        config = scenario_runner.build_backend_config(
            self.root, declaration, declaration["robot"]["backend_config"])
        self.assertEqual(config["model_path"], str(self.root / "model.xml"))
        self.assertEqual(config["manipulation"]["gripper"]["left_finger_geom"],
                         "piper_left_finger")
        self.assertTrue(config["realtime"])
        # 装配层另外要求：声明名 → 模型名的 name_map 必须随报告透传
        self.assertEqual(config["name_map"], {"joint1": "piper_joint1"})


if __name__ == "__main__":
    unittest.main()
