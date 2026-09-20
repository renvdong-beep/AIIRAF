"""标定 UR5e 的工作空间几何：Home 位姿下法兰/夹爪在哪、夹爪朝向是什么。

做 UR5 profile 与场景基线之前必须先有这组数据：
- Piper 的 `grasp.finger_center_xy_m` 是"世界系下双指中点目标位置"，
  换到 UR5 必须知道**UR5 的基座朝向**：UR5e 的基座 z 轴朝上，
  而肩部轴是 y 轴，因此工作空间主要落在**基座 +y 方向**。
- 夹爪是挂在法兰上的，法兰姿态由 `attachment_site` 的 quat 决定，
  夹爪指向（pinch 相对法兰）也必须在世界系里量出来，
  否则 IK 求出的关节角会让夹爪指向错误方向。

本脚本只做**几何标定**（纯 mj_forward），输出：
1. 官方 home 关键帧下的法兰/夹爪位置与姿态；
2. 几个典型肩/肘配置下夹爪可达的最大前伸与最低高度；
3. 夹爪 z 轴（夹爪指向）在世界系的方向，供确定接近轴。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _site(model, name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError("缺少 site: " + name)
    return int(site_id)


def _body(model, name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError("缺少 body: " + name)
    return int(body_id)


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def _qadr(model, name):
    return int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    ])


def set_arm(model, data, values):
    for name, value in values.items():
        data.qpos[_qadr(model, name)] = float(value)


def pose_report(model, data, label):
    pinch = _site(model, "pinch")
    flange = _site(model, "attachment_site")
    w3 = _body(model, "wrist_3_link")
    lp = _geom(model, "rq2f85_left_pad1")
    rp = _geom(model, "rq2f85_right_pad1")
    pad_mid = (
        np.asarray(data.geom_xpos[lp], dtype=float)
        + np.asarray(data.geom_xpos[rp], dtype=float)
    ) / 2.0
    pad_axis = (
        np.asarray(data.geom_xpos[rp], dtype=float)
        - np.asarray(data.geom_xpos[lp], dtype=float)
    )
    pad_axis = pad_axis / max(1e-12, float(np.linalg.norm(pad_axis)))
    # 夹爪指向 = pinch - flange（法兰到夹爪中心的连线）
    reach = np.asarray(data.site_xpos[pinch], dtype=float) - np.asarray(
        data.site_xpos[flange], dtype=float
    )
    return {
        "label": label,
        "flange_world_m": [round(float(v), 6) for v in data.site_xpos[flange]],
        "pinch_world_m": [round(float(v), 6) for v in data.site_xpos[pinch]],
        "wrist3_world_m": [round(float(v), 6) for v in data.xpos[w3]],
        "pad_mid_world_m": [round(float(v), 6) for v in pad_mid],
        "pad_gap_axis_world": [round(float(v), 6) for v in pad_axis],
        "pinch_minus_flange_m": [round(float(v), 6) for v in reach],
        "pinch_minus_flange_norm_m": round(float(np.linalg.norm(reach)), 6),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--output", type=Path, default=Path("build/calibration/ur5-workspace.json")
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    data = mujoco.MjData(model)

    print("nkey=%d  key names=%s" % (
        model.nkey,
        [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
         for i in range(model.nkey)],
    ))

    report = {"schema_version": "iraf.ur5-workspace/v1", "poses": []}

    # --- 官方 home ---
    home = {name: 0.0 for name in ARM_JOINTS}
    if model.nkey:
        key = model.key_qpos[0]
        for index, name in enumerate(ARM_JOINTS):
            home[name] = float(key[_qadr(model, name)])
    data.qpos[:] = 0.0
    set_arm(model, data, home)
    mujoco.mj_forward(model, data)
    report["official_home_rad"] = {k: round(v, 6) for k, v in home.items()}
    report["poses"].append(pose_report(model, data, "official_home"))

    # --- 零位 ---
    # 关键帧位置已知：把 home 的 6 个臂关节写进 key_qpos，避免下游重复解析
    # （keyframe 在本模型里已被补齐到 nq 维度，直接可用）。
    zero = {name: 0.0 for name in ARM_JOINTS}
    data.qpos[:] = 0.0
    set_arm(model, data, zero)
    mujoco.mj_forward(model, data)
    report["poses"].append(pose_report(model, data, "all_zero"))

    # --- 采样若干配置，找"向下抓取"姿态 ---
    # 目标：让 pinch 在基座 +x 方向、z 约 0.05~0.15m，且夹爪指向朝下。
    # 对 UR5e（基座 z 上、肩轴 y），常见的向下姿态是
    #   shoulder_lift ≈ -90°, elbow ≈ +90°, wrist_1 ≈ -90°, wrist_2 ≈ -90°
    candidates = {
        "reach_forward_down": {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -np.pi / 2,
            "elbow_joint": np.pi / 2,
            "wrist_1_joint": -np.pi / 2,
            "wrist_2_joint": -np.pi / 2,
            "wrist_3_joint": 0.0,
        },
        "reach_forward_down_bent": {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -np.deg2rad(60.0),
            "elbow_joint": np.deg2rad(100.0),
            "wrist_1_joint": -np.deg2rad(130.0),
            "wrist_2_joint": -np.pi / 2,
            "wrist_3_joint": 0.0,
        },
        "hover_high": {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -np.deg2rad(45.0),
            "elbow_joint": np.deg2rad(90.0),
            "wrist_1_joint": -np.deg2rad(135.0),
            "wrist_2_joint": -np.pi / 2,
            "wrist_3_joint": 0.0,
        },
    }
    for label, values in candidates.items():
        data.qpos[:] = 0.0
        set_arm(model, data, values)
        mujoco.mj_forward(model, data)
        entry = pose_report(model, data, label)
        entry["arm_rad"] = {k: round(float(v), 6) for k, v in values.items()}
        # 记录夹爪指向与重力的夹角，用于判断能否竖直向下抓取
        direction = np.asarray(entry["pinch_minus_flange_m"], dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm > 1e-9:
            direction = direction / norm
            entry["pinch_dir_minus_z_cos"] = round(float(-direction[2]), 6)
        report["poses"].append(entry)

    for entry in report["poses"]:
        print("\n[%s]" % entry["label"])
        if "arm_rad" in entry:
            print("  arm_rad:", entry["arm_rad"])
        print("  flange   :", entry["flange_world_m"])
        print("  pinch    :", entry["pinch_world_m"])
        print("  pad_mid  :", entry["pad_mid_world_m"])
        print("  pinch-flange:", entry["pinch_minus_flange_m"],
              "norm=%.4f" % entry["pinch_minus_flange_norm_m"])
        print("  pad gap axis:", entry["pad_gap_axis_world"])
        if "pinch_dir_minus_z_cos" in entry:
            print("  指向与 -Z 的 cos: %.4f" % entry["pinch_dir_minus_z_cos"])

    # --- 可达性粗扫：扫肩/肘，记录 pinch 的最低 z 与最大水平前伸 ---
    lowest_z = float("inf")
    lowest_cfg = None
    max_reach = 0.0
    max_reach_cfg = None
    pinch_site = _site(model, "pinch")
    for pan in (0.0,):
        for lift in np.deg2rad(np.arange(-160, 10, 10)):
            for elbow in np.deg2rad(np.arange(10, 170, 10)):
                for w1 in np.deg2rad(np.arange(-170, 10, 10)):
                    data.qpos[:] = 0.0
                    set_arm(model, data, {
                        "shoulder_pan_joint": pan,
                        "shoulder_lift_joint": lift,
                        "elbow_joint": elbow,
                        "wrist_1_joint": w1,
                        "wrist_2_joint": -np.pi / 2,
                        "wrist_3_joint": 0.0,
                    })
                    mujoco.mj_forward(model, data)
                    pos = np.asarray(data.site_xpos[pinch_site], dtype=float)
                    if pos[2] < lowest_z:
                        lowest_z = float(pos[2])
                        lowest_cfg = {
                            "shoulder_lift_deg": round(float(np.rad2deg(lift)), 3),
                            "elbow_deg": round(float(np.rad2deg(elbow)), 3),
                            "wrist_1_deg": round(float(np.rad2deg(w1)), 3),
                            "pinch_world_m": [round(float(v), 6) for v in pos],
                        }
                    horizontal = float(np.linalg.norm(pos[:2]))
                    if horizontal > max_reach and pos[2] > 0.0:
                        max_reach = horizontal
                        max_reach_cfg = {
                            "shoulder_lift_deg": round(float(np.rad2deg(lift)), 3),
                            "elbow_deg": round(float(np.rad2deg(elbow)), 3),
                            "wrist_1_deg": round(float(np.rad2deg(w1)), 3),
                            "pinch_world_m": [round(float(v), 6) for v in pos],
                        }
    report["lowest_pinch_z_m"] = round(lowest_z, 6)
    report["lowest_pinch_cfg"] = lowest_cfg
    report["max_horizontal_reach_m"] = round(max_reach, 6)
    report["max_horizontal_reach_cfg"] = max_reach_cfg
    print("\n最低 pinch z = %.6f  cfg=%s" % (lowest_z, lowest_cfg))
    print("最大水平前伸 = %.6f  cfg=%s" % (max_reach, max_reach_cfg))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
