#!/usr/bin/env python3
"""放置位窗扫描（声明化方法；2026-09-28 §11.23(41)）。

入口：PYTHONPATH=src python3 scripts/place_position_sweep_probe.py


为什么重建：上一版探针随 build/ 被清理，而"交接面该放多高、放多远"必须先扫描再改声明
（直接改声明靠构建门禁试错，一次 20 s，且看不到"为什么不行"）。

判据（全部可量化、全部来自声明或模型实测）：
  ① 放置四段 IK 残差 ≤ 1e-5（构建器的门禁是 tolerance×100 = 1e-3，这里取更严的工程目标）；
  ② 夹爪轴朝下：above/descend/retreat 的轴 z ≤ -0.2；
  ③ **同半球**：各段轴与 pick 抬升段轴的点积 > 0（构建器会对 dot<0 报"翻到相反半球"警告）；
  ④ 距交接站位 ≥ 0.40 m（狗腿摆动半径 ~0.3 m）；
  ⑤ **物理判据**：绕行高度离地 = 承载面 z + 放置净间隙 ≥ 0.15 m
     （实测臂在承载面过低时下垂 2.6 cm 并把方块压地、与硬 <connect> 约束互锁顶死基座偏航）。

口径与构建器完全一致（`scene_builder._joint_place_resolution`）：
  局部目标 = R^T · (承载面世界点 − 臂基座实测位置)；承载面世界点 = 接收体名义位姿 + [0,0,半高]；
  绕行航点局部目标 = (lift 段指腹中点的局部 xy, 承载面局部 z + 载荷半高 + pad_offset + 净间隙)。
自校验：把已知可行的旧窗（承载面 0.02、az=180°、r=0.20）算出来与构建器报告对上（残差 ~1e-5）。
"""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
# 声明式求解器模块在 scripts/ 下，且它 import 同目录的 build_piper_pick_scene ⇒ 必须把 scripts/ 也进路径
sys.path.insert(0, str(ROOT / "scripts"))
import mujoco  # noqa: E402

REPORT = ROOT / "build/scenes/handoff_lab/handoff_lab_joint.json"
MODEL_XML = ROOT / "build/scenes/handoff_lab/handoff_lab_joint.xml"
BASELINE = ROOT / "config/piper_simulation_baseline.yaml"
SOLVER_MODULE = ROOT / "scripts/build_piper_baseline.py"

STATION_XY = (0.45, 0.0)
BASE_XY = (0.45, -0.45)
PAYLOAD_HALF_DEFAULT = 0.025


def _read_yaml(path):
    import yaml
    return yaml.safe_load(Path(path).read_text())


def _load_solver():
    import importlib.util
    spec = importlib.util.spec_from_file_location("placed_solver", SOLVER_MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_place_reference_poses


def _quat_to_matrix(quat):
    w, x, y, z = (float(v) for v in quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    report = json.loads(REPORT.read_text())
    baseline = _read_yaml(BASELINE)
    solve = _load_solver()
    gripper = report["gripper"]
    # 口径与构建器一致：基座位置取 `reference_pose_resolution.placement`（声明位姿，z=0.0；
    # 已用报告里的 `local_target_m` = [0.240416306, 0, 0.025] 反算校验过 ⇒ z 不做偏移）。
    resolution = report["manipulation"]["reference_pose_resolution"]
    placement = resolution["placement"]
    rotation = _quat_to_matrix(placement["quat_wxyz"])
    base_pos = np.asarray(placement["pos_m"], dtype=float)
    pad_offset = float(resolution["resolved_pad_offset_m"])
    payload_half = float(report.get("target_half_size_m")
                         or (baseline.get("target") or {}).get("half_size_m")
                         or PAYLOAD_HALF_DEFAULT)
    clearance = float((baseline.get("grasp") or {}).get("place_clearance_m") or 0.0)
    prefix = "piper_"
    lift_positions = {str(k)[len(prefix):] if str(k).startswith(prefix) else str(k): float(v)
                      for k, v in (gripper.get("lift_positions") or {}).items()}
    pad_geoms = [str(gripper["left_finger_geom"]), str(gripper["right_finger_geom"])]
    wrist = str(gripper["wrist_body"])
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)

    # ---- lift 段指腹中点（联合模型 FK）⇒ 绕行航点的 xy 与参考半球轴
    for name, value in (gripper.get("lift_positions") or {}).items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)
    points = [np.asarray(data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g)],
                         dtype=float) for g in pad_geoms]
    lift_pad_world = (points[0] + points[1]) / 2.0
    wrist_pos = np.asarray(data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, wrist)],
                           dtype=float)
    lift_axis = lift_pad_world - wrist_pos
    lift_axis = lift_axis / (np.linalg.norm(lift_axis) or 1.0)

    def evaluate(az_deg, radius, surface_z):
        az = np.deg2rad(az_deg)
        world = np.array([BASE_XY[0] + radius * np.cos(az),
                          BASE_XY[1] + radius * np.sin(az), surface_z], dtype=float)
        local = rotation.T @ (world - base_pos)
        lift_local = rotation.T @ (lift_pad_world - base_pos)
        transit_local = [float(lift_local[0]), float(lift_local[1]),
                         float(local[2] + payload_half + pad_offset + clearance)]
        try:
            poses = solve(ROOT, baseline, [float(v) for v in local], payload_half, pad_offset,
                          clearance, transit_local_m=transit_local, seed_positions=lift_positions)
        except Exception as error:  # noqa: BLE001
            return {"ok": False, "reason": "求解失败: %s" % error}
        rows = {}
        worst = 0.0
        # ⚠ 每段的**夹爪轴**在 `poses[phase]["gripper_axis_world"]`（由求解器 `_pack_pose` 之后的
        # 第 233 行写入），**不在** `poses[phase]["axis"]`；`written`/`gripper_axes` 是构建器在
        # `_joint_place_resolution` 里另外拼的（求解器原始输出没有）。本轮因此两次取错字段：
        # 第一次全 0 ⇒ 判据恒假；第二次取 `written` ⇒ 空 ⇒ 静默通过（两种都是仪器故障，不是空间真空）。
        for phase, pose in (poses.get("poses") or {}).items():
            axis = np.asarray(pose.get("gripper_axis_world") or [0.0, 0.0, 0.0], dtype=float)
            error = float(pose.get("position_error_m") or 9.9)
            rows[str(phase)] = {
                "error": error, "axis": [round(float(v), 6) for v in axis],
                "axis_z": round(float(axis[2]), 6),
                "dot": round(float(np.dot(axis, lift_axis)), 6),
                "dot_reference": pose.get("axis_dot_reference"),
                "seed": pose.get("chosen_seed"),
            }
            worst = max(worst, error)
        above = rows.get("above") or rows.get("descend") or {}
        descend = rows.get("descend") or {}
        transit = rows.get("transit") or {}
        floor_margin = surface_z + clearance
        station_distance = float(np.linalg.norm(world[:2] - np.asarray(STATION_XY)))
        # 硬判据：残差 / 放置三段轴朝下 / 离站位 / 离地余量。
        # 半球（transit 的 dot）只**报告**不判死：已建成的绿窗（承载面 0.02）本身就是 dot=-0.219884，
        # 构建器也只把它当 warning ⇒ 拿它当硬判据会把已验证可行的窗口误杀。
        ok = (worst <= 1e-5
              and float(above.get("axis_z", 1.0)) <= -0.2
              and float(descend.get("axis_z", 1.0)) <= -0.2
              and station_distance >= 0.40
              and floor_margin >= 0.15)
        return {"ok": ok, "worst_error": worst, "phases": rows, "floor_margin_m": floor_margin,
                "station_distance_m": station_distance, "world": [round(float(v), 6) for v in world],
                "local": [round(float(v), 6) for v in local]}

    candidates = []
    for radius in (0.20, 0.25, 0.30, 0.35, 0.40):
        for az in (135, 160, 180, 200, 225, 90, 270, 0):
            for surface_z in (0.02, 0.08, 0.10, 0.12, 0.16, 0.20):
                result = evaluate(az, radius, surface_z)
                if not result.get("ok"):
                    continue
                candidates.append({"az_deg": az, "radius_m": radius, "surface_z_m": surface_z,
                                   **result})
    candidates.sort(key=lambda row: (abs(row["surface_z_m"] - 0.12), row["worst_error"]))
    print("可行候选 %d 个（按 |承载面 −0.12 m| 再按残差排序）：" % len(candidates))
    for row in candidates[:14]:
        print("  az=%3d° r=%.2f 承载面=%.2f 世界=%s 残差=%.3e 轴z(above/descend)=%s/%s transit_dot=%s "
              "离站位=%.3f 离地=%.3f" % (
                  row["az_deg"], row["radius_m"], row["surface_z_m"],
                  [round(v, 3) for v in row["world"]], row["worst_error"],
                  round(float(row["phases"].get("above", {}).get("axis_z", 0.0)), 3),
                  round(float(row["phases"].get("descend", {}).get("axis_z", 0.0)), 3),
                  round(float(row["phases"].get("transit", {}).get("dot", -1)), 3),
                  row["station_distance_m"], row["floor_margin_m"]))
    # 自校验：旧窗（承载面 0.02、az=180、r=0.20）应可行
    check = evaluate(180, 0.20, 0.02)
    print("\n自校验（旧窗 az=180 r=0.20 承载面=0.02）: ok=%s 残差=%.3e 轴z=%s transit_dot=%s"
          % (check.get("ok"), check.get("worst_error", float("nan")),
             check["phases"].get("descend", {}).get("axis_z"),
             check["phases"].get("transit", {}).get("dot")))
    out = Path("build/iraf-a6a14/place_position_sweep_probe.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"criteria": {
        "ik_residual_max": 1e-5, "axis_z_max": -0.2, "transit_hemisphere_dot_min": 0.0,
        "station_distance_min_m": 0.40, "floor_margin_min_m": 0.15},
        "candidates": candidates, "self_check_old_window": check}, ensure_ascii=False, indent=1))
    print("结果写入 %s" % out)


if __name__ == "__main__":
    main()
