"""汇总展示 UR5 基线求解结果的关键量（避免翻长 JSON）。"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    scene_path = ROOT / "build/models/ur5-pick-scene.json"
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    # reference_poses 由 build_ur5_baseline 在调用 build_scene 之后附加，
    # 而 .json 是 build_scene 内部落的盘，因此这里以**姿态证据文件**为准。
    ref = json.loads(
        (ROOT / "build/calibration/ur5-baseline-pose.json").read_text(encoding="utf-8")
    )

    print("=== 场景 ===")
    print("target_id        :", scene["target_id"])
    print("target_position  :", scene["target_position"])
    print("half_size_m      :", scene["target_half_size_m"])
    print("arm_gains_injected:", scene["arm_gains_injected"], "kp:", scene["arm_position_kp"])
    print("finger_geoms     :", scene["finger_geoms"])

    print("\n=== 参考姿态求解 ===")
    print("tip_clearance_m         :", ref["tip_clearance_m"])
    print("finger_tip_z_m          :", ref["finger_tip_z_m"])
    print("height_correction_m     :", ref["finger_height_correction_m"])
    print("gripper_axis_world      :", ref["gripper_axis_world"])
    print("approach_direction_world:", ref["approach_direction_world"])
    print("pregrasp_offset_m       :", ref["pregrasp_offset_m"])
    print("lift_offset_m           :", ref["lift_offset_m"])
    for key in ("home", "approach", "grasp", "lift"):
        block = ref[key]
        if key == "home":
            print("\n[home]（种子位形）")
            print("  ", {k: round(v, 6) for k, v in block.items()})
            continue
        print("\n[%s]" % key)
        print("   joint_positions :", {
            k: round(v, 6) for k, v in block["joint_positions"].items()
        })
        print("   finger_center_m :", [round(v, 6) for v in block["finger_center_m"]])
        print("   target_m        :", [round(v, 6) for v in block["target_m"]])
        print("   position_error  : %.3e m" % block["position_error_m"])
        print("   iterations      :", block["iterations"])

    print("\n=== 抓取姿态校验 ===")
    print(json.dumps(scene.get("grasp_pose_validation", {}), ensure_ascii=False, indent=2))

    # 抬升位移必须 >= 验收阈值
    lift_delta = ref["lift"]["finger_center_m"][2] - ref["grasp"]["finger_center_m"][2]
    print("\n参考抬升位移 = %.6f m" % lift_delta)

    # 求解残差门禁
    acceptance_path = ROOT / "config/ur5_simulation_baseline.yaml"
    import yaml

    acceptance = (yaml.safe_load(acceptance_path.read_text(encoding="utf-8"))
                  .get("acceptance") or {})
    tolerance = float(acceptance.get("pose_tolerance_m", 0.005))
    factor = float(acceptance.get("solver_error_factor", 0.1))
    worst = max(ref[k]["position_error_m"] for k in ("approach", "grasp", "lift"))
    print("残差门禁: worst=%.3e  limit=%.3e  %s"
          % (worst, tolerance * factor, "PASS" if worst <= tolerance * factor else "FAIL"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
