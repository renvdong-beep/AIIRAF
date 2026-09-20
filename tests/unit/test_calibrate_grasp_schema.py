"""契约测试：后端返回的证据必须满足 Skill 声明的输出 schema。

为什么需要：Skill 的输出 schema（`skills/<skill>/<skill>.output.json`）是公开契约，
由 `iraf_core.registry` 在运行期校验。改字段名时若只改实现不改 schema，
故障要等"策略放行该 skill"的真实调用才会暴露 —— 实测 `tcp_offset_from_link6_m`
改名时就出现过这种不一致（Piper 的 safety policy 不放行 calibrate_grasp，
因此跑验收根本碰不到）。

本测试用最小 MJCF 直接调用后端，拿真实证据去过 schema，
把"实现 ⇄ 契约"的一致性锁在单元测试里。
"""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
from jsonschema import Draft202012Validator

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend

ROOT = Path(__file__).resolve().parents[2]

#: 腕部 + 左右指的单链夹具：足以覆盖 calibrate_grasp 需要的全部几何声明。
FIXTURE_MJCF = """
<mujoco>
  <worldbody>
    <body name="wrist_link">
      <geom name="wrist_geom" type="sphere" size="0.02"/>
      <site name="flange" pos="0 0 0.02"/>
      <body name="left_finger" pos="0 0.03 0.05">
        <geom name="left_pad" type="box" size="0.005 0.005 0.005"/>
      </body>
      <body name="right_finger" pos="0 -0.03 0.05">
        <geom name="right_pad" type="box" size="0.005 0.005 0.005"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class CalibrateGraspSchemaContractTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.model_path = Path(self.tmp.name) / "fixture.xml"
        self.model_path.write_text(FIXTURE_MJCF, encoding="utf-8")
        self.schema = json.loads(
            (ROOT / "skills/calibrate_grasp/calibrate_grasp.output.json").read_text(
                encoding="utf-8"
            )
        )

    def _backend(self):
        manipulation = {
            "targets": {},
            "gripper": {
                "wrist_body": "wrist_link",
                "left_finger_body": "left_finger",
                "right_finger_body": "right_finger",
                "left_finger_geom": "left_pad",
                "right_finger_geom": "right_pad",
                "open_positions": {"joint7": 0.035, "joint8": -0.035},
                "closed_positions": {"joint7": 0.0, "joint8": 0.0},
            },
        }
        profile = SimpleNamespace(joints=["joint7", "joint8"], capabilities=())
        # authority 只用于租约校验，本测试传替身即可。
        authority = SimpleNamespace(validate=lambda lease: None)
        return MujocoBackend(
            str(self.model_path),
            profile,
            authority,
            manipulation_config=manipulation,
        )

    def test_evidence_satisfies_declared_output_schema(self):
        evidence = self._backend().calibrate_grasp({}, None)
        payload = {
            "skill": "calibrate_grasp",
            "accepted": True,
            "evidence": evidence,
        }
        errors = sorted(
            Draft202012Validator(self.schema).iter_errors(payload),
            key=lambda error: list(error.path),
        )
        self.assertEqual(
            [],
            [error.message for error in errors],
            "后端证据与 Skill 输出 schema 不一致（改字段名必须同步改 schema）",
        )

    def test_evidence_uses_declared_names_not_robot_specific_ones(self):
        """字段名必须来自声明，不得残留某机型的专有名（如 link6）。"""
        evidence = self._backend().calibrate_grasp({}, None)
        self.assertEqual("wrist_link", evidence["wrist_body"])
        self.assertIn("tcp_offset_from_wrist_m", evidence)
        self.assertNotIn("tcp_offset_from_link6_m", evidence)


if __name__ == "__main__":
    unittest.main()
