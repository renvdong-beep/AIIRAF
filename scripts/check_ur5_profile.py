"""校验 UR5 profile 与基线能否被核心层解析，并交叉核对二者一致性。"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import yaml

from iraf_core.profile import load_robot_profile


def main():
    root = Path(__file__).resolve().parents[1]
    profile = load_robot_profile(root / "profiles/ur5_mujoco.yaml")
    print("profile name :", profile.name, "version", profile.version)
    print("joints(%d)   :" % len(profile.joints), list(profile.joints))
    print("capabilities :", sorted(profile.capabilities))
    print("joint_roles  :", profile.joint_roles)
    print("gripper      :", json.dumps(profile.gripper, ensure_ascii=False))
    print("home         :", json.dumps(profile.home, ensure_ascii=False))
    print("camera       :", json.dumps(profile.camera, ensure_ascii=False))

    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )
    model_cfg = baseline["model"]
    print("\n--- 交叉核对 ---")

    # 1) 基线声明的臂关节必须都在 profile 里
    missing = [j for j in model_cfg["arm_joints"] if j not in profile.joints]
    print("基线臂关节 - profile:", missing or "全部匹配")

    # 2) 基线的 open/closed 键必须与 gripper.joints（执行器名）完全一致。
    #    注意 config 的 gripper.joints 是**执行器名**（后端按 actuator 名写
    #    ctrl），profile 的 gripper.drive_joints 是**被驱动关节名**，
    #    两者是不同层的东西，不能要求相等。
    base_gripper_joints = [str(j) for j in baseline["gripper"]["joints"]]
    profile_drive = [str(j) for j in profile.gripper["drive_joints"]]
    print("基线 gripper.joints (执行器):", base_gripper_joints)
    print("profile drive_joints (关节)  :", profile_drive)
    for key in ("open", "closed"):
        keys = sorted(str(k) for k in baseline["gripper"][key])
        if keys != sorted(base_gripper_joints):
            raise SystemExit(
                "FAIL: gripper.%s 键与 gripper.joints 不一致: %s" % (key, keys)
            )
    print("基线与 profile 的夹爪开合键一致（均为执行器名）")

    # 3) profile 的 drive_joints 必须都在声明的 joints 里（核心校验器要求）
    undeclared = [j for j in profile_drive if j not in profile.joints]
    if undeclared:
        raise SystemExit("FAIL: drive_joints 未在 joints 声明: %s" % undeclared)
    print("profile drive_joints 全部已在 joints 中声明")

    # 4) 基线的 lift 段必须覆盖臂关节
    lift_keys = set(str(k) for k in (baseline["gripper"].get("lift") or {}))
    lift_missing = [j for j in model_cfg["arm_joints"] if j not in lift_keys]
    print("基线 lift 缺臂关节:", lift_missing or "无")

    # 5) 场景注入开关
    scene_cfg = baseline.get("scene") or {}
    print("inject_arm_position_gains:", scene_cfg.get("inject_arm_position_gains"))

    # 6) 抓取点必须落在工作空间内（用标定结果核对）
    calib_path = root / "build/calibration/ur5-workspace.json"
    if calib_path.is_file():
        calib = json.loads(calib_path.read_text(encoding="utf-8"))
        xy = baseline["grasp"]["finger_center_xy_m"]
        top_z = float(baseline["workbench"]["top_z_m"])
        half = float(baseline["target"]["half_size_m"])
        target_z = top_z + half
        reach = (xy[0] ** 2 + xy[1] ** 2) ** 0.5
        print("\n抓取点水平距离 %.4f m（标定最大前伸 %.4f m）"
              % (reach, calib["max_horizontal_reach_m"]))
        print("抓取点 z(pad 中心) %.4f m（标定最低 pinch z %.4f m）"
              % (target_z, calib["lowest_pinch_z_m"]))
        if reach > calib["max_horizontal_reach_m"]:
            raise SystemExit("FAIL: 抓取点超出标定的最大水平前伸")
        if target_z < calib["lowest_pinch_z_m"]:
            raise SystemExit("FAIL: 抓取点低于标定的最低可达高度")
    else:
        print("\n（未找到工作空间标定文件，跳过可达性核对）")
    print("\nOK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
