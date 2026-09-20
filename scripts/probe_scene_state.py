"""只读探针：报告残留场景/参考姿态的状态，用于定位构建失败发生在哪一步。

背景：把 IK 目标从"pad1 中点"改成"4 点夹持区中点"后，
`validate_grasp_pose` 报 `tip_z=-0.0013 required=0.003`。

需要先回答一个问题：**这次失败发生在写入路径还是求解阶段？**
`build_scene` 会在收到 `reference` 之前先落一次盘，因此若磁盘上的
`build/models/ur5-pick-scene.json` 里已经含 `home_positions` /
`grasp_positions`，说明 `reference` 已被成功求解并传入，
失败只发生在最后的 `validate_grasp_pose` 复核。

本脚本无副作用，只读。
"""

import json
from pathlib import Path

ROOT = Path("/home/coretek/AIIRAF")


def describe(label, path):
    path = ROOT / path
    print("=== %s ===" % label)
    print("path      :", path)
    if not path.is_file():
        print("状态      : 不存在")
        return None
    print("bytes     :", path.stat().st_size)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as error:  # noqa: BLE001 - 探针要报告任何解析问题
        print("解析失败  :", error)
        return None
    return data


def main():
    scene = describe("场景 report", "build/models/ur5-pick-scene.json")
    pose = describe("参考姿态 evidence", "build/calibration/ur5-baseline-pose.json")

    if scene:
        gripper = scene.get("gripper") or {}
        print("gripper keys          :", sorted(gripper.keys()))
        for key in ("home_positions", "approach_positions", "grasp_positions"):
            print("%-22s: %s" % (key, "有" if gripper.get(key) else "**缺失**"))
        print("target_position       :", scene.get("target_position"))
        print("scene schema_version  :", scene.get("schema_version"))
        print("有 reference_poses    :", "reference_poses" in scene)
        print("有 grasp_pose_validation:",
              "grasp_pose_validation" in scene)

    if pose:
        print()
        print("=== 参考姿态关键量 ===")
        for key in (
            "schema_version", "finger_height_correction_m", "finger_tip_z_m",
            "finger_center_m", "grasp_point_m", "target_z_m", "pad_height_diff_m",
            "lift_offset_m", "home_rise_m",
        ):
            print("%-26s: %s" % (key, pose.get(key)))
        print("grip_region               :", json.dumps(
            pose.get("grip_region"), ensure_ascii=False))
        print("orientation_error_deg     :", json.dumps(
            pose.get("orientation_error_deg"), ensure_ascii=False))
        print("grasp position_error_m    :",
              (pose.get("grasp") or {}).get("position_error_m"))
        print()
        print("=== 间隙配平轨迹 ===")
        for row in pose.get("clearance_trace") or []:
            print("  it=%d corr=%+.6f tip_z=%+.6f deficit=%+.6f "
                  "pos_err=%.2e ori=%s"
                  % (row["iteration"], row["correction_m"], row["tip_z_m"],
                     row["deficit_m"], row["position_error_m"],
                     row.get("orientation_deg")))

    validation = (scene or {}).get("grasp_pose_validation")
    if validation:
        print()
        print("=== 抓取姿态校验 ===")
        print(json.dumps(validation, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
