"""打印场景 report 的 gripper 段与关键帧，便于核对后端实际拿到的目标值。"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        ROOT / "build/models/ur5-pick-scene.json"
    )
    data = json.loads(path.read_text(encoding="utf-8"))
    gripper = data["gripper"]
    for key in (
        "home_positions",
        "approach_positions",
        "grasp_positions",
        "lift_positions",
        "open_positions",
        "closed_positions",
        "pad_offset_m",
        "pad_offset_axis",
        "lift_constraint",
        "left_finger_geom",
        "right_finger_geom",
    ):
        if key in gripper:
            print("%-20s %s" % (key, json.dumps(gripper[key], ensure_ascii=False)))
    print()
    print("reference_poses 存在:", "reference_poses" in data)
    if "reference_poses" in data:
        ref = data["reference_poses"]
        print("grasp.position_error_m:", ref["grasp"]["position_error_m"])
        print("orientation_error_deg :", ref.get("orientation_error_deg"))
        print("pad_height_diff_m     :", ref.get("pad_height_diff_m"))
        print("home_source           :", ref.get("home_source"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
