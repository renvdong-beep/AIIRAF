"""重解姿态"侵入自检"的回归测试（2026-09-24）。

锁住两条：
1. **归属必须按宿主 body 判定，不能按 geom 名**——联合模型里臂的 link geom 无名
   （实测 `<未命名#66>(body=piper_link6)` 与 `box_01_geom` 重叠 −0.014516 m），
   按"名字带 piper_ 前缀"过滤会把唯一真正侵入的 geom 静默跳过（这是本轮踩到的坑）；
2. 主本体（狗）的 geom 与目标接触**不算**附加本体的侵入（判据只管附加本体，按宿主 body 前缀界定）。

模型：主本体 `base_link`（geom 压在方块上，属**被过滤**的一类）+ 目标 `box_01`
+ 附加本体 `piper_link6`（geom **无名**，沿 z 滑动，滑上去就不碰）。
"""

import unittest

import mujoco

from iraf_adapters.unitree.scene_builder import _reference_pose_clearance_check

MODEL = """
<mujoco>
  <worldbody>
    <body name="base_link">
      <geom size="0.05"/>
    </body>
    <body name="box_01" pos="0 0 0">
      <geom name="box_01_geom" size="0.03"/>
    </body>
    <body name="piper_link6" pos="0 0 0.05">
      <joint name="piper_joint6" type="slide" axis="0 0 1"/>
      <geom size="0.04"/>
    </body>
  </worldbody>
</mujoco>
"""

GRIPPER = {"left_finger_geom": "piper_left_finger", "right_finger_geom": "piper_right_finger"}
TARGETS = [{"id": "box_01", "body": "box_01", "geom": "box_01_geom"}]


class ReferenceClearanceCheckTests(unittest.TestCase):
    def _check(self, joint6_m):
        model = mujoco.MjModel.from_xml_string(MODEL)
        gripper = dict(GRIPPER)
        gripper["grasp_positions"] = {"piper_joint6": joint6_m}
        return _reference_pose_clearance_check(model, gripper, list(TARGETS), "piper_")

    def test_unnamed_arm_geom_intrusion_is_reported(self):
        """无名的附加本体 geom 侵入目标 ⇒ 必须报出来（不能因"名字没有前缀"被跳过）。"""
        result = self._check(0.0)
        self.assertTrue(result["checked"])
        self.assertTrue(result["overlapping"], "未命名 geom 的侵入被静默跳过了")
        violation = result["non_pad_touching_target"][0]
        self.assertEqual(violation["body"], "piper_link6")
        self.assertTrue(violation["geom"].startswith("<未命名#"),
                        "无名 geom 必须用 <未命名#id> 标注，便于定位：%r" % violation["geom"])
        self.assertLess(float(violation["dist_m"]), 0.0)

    def test_main_body_contact_is_not_attached_arm_intrusion(self):
        """主本体的 geom 压着目标也不算侵入（判据按宿主 body 前缀界定）；滑开后整体干净。"""
        far = self._check(0.20)  # 滑上去 ⇒ 附加本体不碰
        self.assertTrue(far["checked"])
        self.assertFalse(far["overlapping"])
        self.assertEqual(far["arm_geoms_touching_target"], [])
        self.assertFalse([item for item in far["arm_geoms_touching_target"]
                          if item["body"] == "base_link"])

    def test_missing_target_skips_loudly(self):
        model = mujoco.MjModel.from_xml_string(MODEL)
        gripper = dict(GRIPPER)
        gripper["grasp_positions"] = {"piper_joint6": 0.0}
        result = _reference_pose_clearance_check(model, gripper, [], "piper_")
        self.assertIn("skipped", result)


if __name__ == "__main__":
    unittest.main()
