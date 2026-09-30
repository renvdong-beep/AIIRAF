"""重力前馈通道与位置指令通道的**对账**回归测试（2026-09-30，§11.26）。

背景（实测）：UR5e 的**同一条控制通道有两个名字** —— 关节名 `shoulder_pan_joint`、
执行器名 `shoulder_pan`。臂侧报告的 `gravity_feedforward` 用执行器名，联合侧重解后的
位置指令用关节名 ⇒ 后端的"前馈通道 ⊆ 位置指令通道"声明检查把整个装配挡死（退出码 4，
`config/machines/ur5e_joint.yaml`）。构建期必须按**模型自己的传动表**（`actuator_trnid`）
把两侧判成同一条通道并统一名字，**且不猜名字后缀**。

锁住四条：
1. 执行器名（ff）与关节名（位置）判成同一通道 ⇒ 前馈键改写成位置字典里的键，且**数值不变**；
2. 名字本来就一致（Piper 情形）⇒ 逐位无改动（不引入新名字）；
3. 对不上的通道 ⇒ 构建期**显式失败**（不许静默丢弃前馈，否则表现为"精度莫名不达标"）；
4. 未知段名 ⇒ 构建期显式失败（与后端同口径，但更早）。

模型刻意让关节名与执行器名不同（`shoulder_pan_joint` vs `shoulder_pan`），复现 UR5e 的命名。
"""

import unittest

import mujoco

from iraf_adapters.unitree.scene_builder import (
    EXIT_MODEL,
    SceneBuildError,
    _align_feedforward_channels,
    _channel_joint_id,
)

MODEL = """
<mujoco>
  <worldbody>
    <body name="base_link">
      <body name="link1">
        <joint name="shoulder_pan_joint" type="hinge" axis="0 0 1" range="-3 3"/>
        <geom size="0.03"/>
        <body name="link2">
          <joint name="elbow_joint" type="hinge" axis="0 1 0" range="-3 3"/>
          <geom size="0.03"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="shoulder_pan" joint="shoulder_pan_joint" kp="100"/>
    <position name="elbow_joint" joint="elbow_joint" kp="100"/>
  </actuator>
</mujoco>
"""

# 位置指令（重解后口径：关节名）；前馈（继承臂侧口径：执行器名）
POSITIONS = {"shoulder_pan_joint": 0.0, "elbow_joint": -0.5}


def _gripper(feedforward):
    gripper = {"home_positions": dict(POSITIONS),
               "approach_positions": dict(POSITIONS),
               "grasp_positions": dict(POSITIONS),
               "lift_positions": dict(POSITIONS)}
    if feedforward is not None:
        gripper["gravity_feedforward"] = feedforward
    return gripper


class ChannelJointIdTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(MODEL)

    def test_joint_and_actuator_names_resolve_to_same_joint(self):
        """关节名与执行器名 → 同一个关节 id（这是"同一通道"的唯一可靠判据）。"""
        by_joint = _channel_joint_id(self.model, "shoulder_pan_joint")
        by_actuator = _channel_joint_id(self.model, "shoulder_pan")
        self.assertIsNotNone(by_joint)
        self.assertEqual(by_joint, by_actuator)
        self.assertEqual(by_joint, int(mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan_joint")))

    def test_unknown_name_resolves_to_none(self):
        self.assertIsNone(_channel_joint_id(self.model, "no_such_channel"))


class AlignFeedforwardChannelTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(MODEL)

    def test_actuator_style_feedforward_is_renamed_to_position_keys(self):
        """UR5e 情形：前馈用执行器名、位置用关节名 ⇒ 改写成位置键，数值逐位不变。"""
        gripper = _gripper({"home": {"shoulder_pan": -0.014, "elbow_joint": -0.009}})
        aligned, evidence = _align_feedforward_channels(self.model, gripper, "inherited_from_arm_report")
        self.assertEqual({"shoulder_pan_joint": -0.014, "elbow_joint": -0.009}, aligned["home"])
        self.assertEqual([{"from": "shoulder_pan", "to": "shoulder_pan_joint"}],
                         evidence["phases"]["home"]["renamed"])
        self.assertEqual(2, evidence["phases"]["home"]["channels"])

    def test_identical_names_are_untouched(self):
        """Piper 情形：名字一致 ⇒ 输出与输入逐位相同（不引入新名字）。"""
        declared = {"home": dict(POSITIONS), "grasp": dict(POSITIONS)}
        gripper = _gripper(declared)
        aligned, evidence = _align_feedforward_channels(self.model, gripper, "inherited_from_arm_report")
        self.assertEqual(declared, aligned)
        self.assertEqual([], evidence["phases"]["home"]["renamed"])

    def test_unmappable_channel_fails_closed(self):
        """对不上的通道 ⇒ 显式失败（静默丢弃前馈会让"精度莫名不达标"极难定位）。"""
        gripper = _gripper({"home": {"no_such_channel": 0.01}})
        with self.assertRaises(SceneBuildError) as ctx:
            _align_feedforward_channels(self.model, gripper, "inherited_from_arm_report")
        self.assertEqual(EXIT_MODEL, ctx.exception.code)
        self.assertIn("对不上", str(ctx.exception))
        self.assertIn("no_such_channel", str(ctx.exception))

    def test_unknown_phase_fails_closed(self):
        gripper = _gripper({"home_pose": {"shoulder_pan": 0.01}})
        with self.assertRaises(SceneBuildError) as ctx:
            _align_feedforward_channels(self.model, gripper, "inherited_from_arm_report")
        self.assertEqual(EXIT_MODEL, ctx.exception.code)
        self.assertIn("未知段名", str(ctx.exception))

    def test_feedforward_without_positions_fails_closed(self):
        gripper = _gripper({"home": {"shoulder_pan": 0.01}})
        gripper.pop("home_positions")
        with self.assertRaises(SceneBuildError) as ctx:
            _align_feedforward_channels(self.model, gripper, "inherited_from_arm_report")
        self.assertEqual(EXIT_MODEL, ctx.exception.code)
        self.assertIn("缺少对应的", str(ctx.exception))

    def test_no_feedforward_declared_is_noop(self):
        aligned, evidence = _align_feedforward_channels(self.model, _gripper(None), "inherited_from_arm_report")
        self.assertEqual({}, aligned)
        self.assertTrue(evidence["checked"])


if __name__ == "__main__":
    unittest.main()
