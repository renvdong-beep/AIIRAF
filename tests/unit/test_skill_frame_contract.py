# -*- coding: utf-8 -*-
"""契约回归：抓取位姿的坐标系必须是 world（不是自由字符串）。

2026-10-21 实测：`pick_object.input.json` 里 frame_id 允许任意字符串，意图模型偶发给出
非 world 的坐标系（4 次里 2 次），MuJoCo 后端只能 fail-closed：
`MuJoCo 抓取当前只接受 world 坐标系`。契约本身就写明 `pose_source=world_absolute`
是"调用方给世界系坐标"，所以把 frame_id 收窄成 const "world"，
让约束在**schema**这一层生效（意图模型拿到的 JSON Schema 也就没有别的可选值）。
"""
import json
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PICK = REPO / "skills/pick_object/pick_object.input.json"


class FrameContractTest(unittest.TestCase):
    def test_frame_id_is_world_only(self):
        schema = json.loads(PICK.read_text(encoding="utf-8"))
        frame = schema["properties"]["grasp_pose"]["properties"]["frame_id"]
        self.assertEqual(
            frame.get("const"), "world",
            "grasp_pose.frame_id 必须 const 'world'（否则意图模型会给出 base/camera 等系，"
            "后端只能 fail-closed）：%s" % frame)

    def test_other_pose_fields_still_open(self):
        """只收窄坐标系，不动 pose_source / position / orientation 的既有语义。"""
        schema = json.loads(PICK.read_text(encoding="utf-8"))
        grip = schema["properties"]["grasp_pose"]["properties"]
        self.assertEqual(sorted(grip["pose_source"]["enum"]),
                         ["live_target_body", "world_absolute"])
        self.assertIn("position", grip)
        self.assertIn("orientation", grip)


if __name__ == "__main__":
    unittest.main()
