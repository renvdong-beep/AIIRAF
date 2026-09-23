#!/usr/bin/env python3
"""口径 B：MPC 动力学（`Ad/Bd/gd`）的**自洽性**验收（不依赖上游模型）。

为什么需要它（见 docs/debug/2026-09-23-a6a4-state-bridge-facts.md §6.1）：上游的
`x0/Ad/Bd/gd` 来自 pinocchio/URDF，我们是 MuJoCo 厂商 MJCF ⇒ 逐位对照**不可达**；
而 MPC 要控制的是**我们这台本体**（含托盘），所以真正要验的是"我们的模型说得对不对"。

两条判据（都可复跑、都有解析/独立算路做对照）：
  1. **重力语义**：`u = 0`（无接触力）时，`x_{k+1} = Ad·x_k + gd` 给出的竖直速度增量必须等于
     `−g·dt`（`g` 由模型读，不写数字）；残差应在浮点级。
  2. **独立积分对照**：用 RK4 在**连续时间**上积分质心动力学（`ṗ=v`、`v̇=g+Σf/m`、
     `ω̇=I⁻¹(Στ − ω×Iω)`、姿态按 yaw 平均近似），与离散 `Ad/Bd/gd` 的一步预测比对；
     残差 = 离散化误差，阈值取声明 `mpc_model.dynamics_consistency_tol`。

用法：
  PYTHONPATH=src /usr/bin/python3 scripts/verify_mpc_dynamics_consistency.py \
      --config config/go2_locomote.yaml --robot-config config/go2_loopback.yaml
退出码：0 = 全部通过；5 = 有判据失败；2/3 = 声明或装配问题。
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree.mpc import dynamics as dyn  # noqa: E402
from iraf_adapters.unitree.mpc.contact import LEG_ORDER, contact_table  # noqa: E402
from iraf_adapters.unitree.mpc.gait_trot import merge_trot_declaration  # noqa: E402
from iraf_adapters.unitree.mpc.plan import time_step_s  # noqa: E402
from iraf_adapters.unitree.mpc.reference import (foot_reference_trajectory,  # noqa: E402
                                                 reference_state_trajectory,
                                                 touchdown_parameters, yaw_rotation)
from iraf_adapters.unitree.mpc.state_bridge import subtree_mass_inertia  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_DECLARATION, EXIT_BACKEND, EXIT_FAILED = 0, 1, 2, 3, 5
REPORT_SCHEMA = "iraf.mpc-dynamics-consistency/v1"


def _rk4_step(x, dt, mass, inertia, gravity, forces_sum, torque_sum):
    """连续质心动力学的一步 RK4（**独立算路**，只用于对照）。"""
    def deriv(state):
        p, rpy, v, omega = state[0:3], state[3:6], state[6:9], state[9:12]
        d = np.zeros(12)
        d[0:3] = v
        d[3:5] = 0.0                                   # roll/pitch 参考为 0（与参考轨迹同口径）
        d[5] = float(omega[2])
        d[6:9] = np.array([0.0, 0.0, -float(gravity)]) + forces_sum / mass
        d[9:12] = np.linalg.solve(inertia, torque_sum - np.cross(omega, inertia @ omega))
        return d
    k1 = deriv(x)
    k2 = deriv(x + 0.5 * dt * k1)
    k3 = deriv(x + 0.5 * dt * k2)
    k4 = deriv(x + dt * k3)
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def main(argv=None):
    parser = argparse.ArgumentParser(description="MPC 动力学自洽性验收（口径 B）")
    parser.add_argument("--config", type=Path, default=Path("config/go2_locomote.yaml"))
    parser.add_argument("--robot-config", type=Path, default=Path("config/go2_loopback.yaml"))
    parser.add_argument("--report", type=Path,
                        default=Path("build/acceptance/mpc-dynamics-consistency/report.json"))
    args = parser.parse_args(argv)

    locomote = yaml.safe_load((ROOT / args.config).read_text(encoding="utf-8"))
    robot = yaml.safe_load((ROOT / args.robot_config).read_text(encoding="utf-8"))
    profile = load_robot_profile(ROOT / robot["robot"]["profile"])
    mpc_model = dict(locomote["mpc_model"])
    if "dynamics_consistency_tol" not in mpc_model:
        print("声明非法：mpc_model 缺 dynamics_consistency_tol（口径 B 的判据阈值必须来自声明）",
              file=sys.stderr)
        return EXIT_DECLARATION
    tol = float(mpc_model["dynamics_consistency_tol"])

    merged = merge_trot_declaration(robot, locomote["mpc_gait"])
    gait_params = gait.load_gait_declaration(merged, profile.joints)

    authority = ControlAuthorityManager()
    try:
        backend = load_backend(KNOWN_BACKENDS[str(robot["robot"]["backend"])],
                               str(ROOT / args.robot_config), profile, authority)
    except Exception as exc:  # noqa: BLE001
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    mujoco, model, data = backend.mujoco, backend.model, backend.data
    trunk = gait.trunk_body_id(model, mujoco, gait_params)
    dt = time_step_s(mpc_model)
    horizon = int(mpc_model["horizon"])
    gravity = backend.gravity_mps2()

    # 状态序列（关键帧 + 少量姿态扰动；每例都从同一被控对象取几何/惯量）
    mujoco.mj_resetDataKeyframe(model, data,
                               mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,
                                                 robot["initial"]["keyframe"]))
    mujoco.mj_forward(model, data)

    results = []
    for label, perturb in (("home", np.zeros(6)), ("roll+2deg", np.array([0.035, 0.0, 0.0, 0, 0, 0])),
                           ("pitch-3deg", np.array([0.0, -0.052, 0.0, 0, 0, 0])),
                           ("yaw90", np.array([0.0, 0.0, np.pi / 2.0, 0, 0, 0]))):
        mujoco.mj_resetDataKeyframe(model, data,
                                   mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,
                                                     robot["initial"]["keyframe"]))
        mujoco.mj_forward(model, data)
        # 用自由关节的 7 维表示设置姿态/偏航（roll/pitch 通过四元数给出）
        q = np.array([0.0, 120.0, 0.0001, 0.0, 0.0, 0.0, 0.0], dtype=float)
        mujoco.mj_resetData(model, data)
        data.qpos[:] = 0.0
        data.qpos[0:3] = [0.0, 0.0, 0.30]
        data.qpos[2] = 0.30
        # home 位形：直接取关键帧的关节角
        mujoco.mj_resetDataKeyframe(model, data,
                                    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,
                                                      robot["initial"]["keyframe"]))
        quat = _quat_from_rpy(perturb[0], perturb[1], perturb[2])
        data.qpos[3:7] = quat
        data.qpos[2] = 0.32                      # 抬离台面 ⇒ 无接触（纯重力语义段）
        mujoco.mj_forward(model, data)

        mass, com, inertia, _bodies = subtree_mass_inertia(model, data, mujoco, trunk)
        base = np.asarray(data.qpos[0:3], dtype=float)
        quat_now = np.asarray(data.qpos[3:7], dtype=float)
        vel = np.asarray(data.qvel[0:3], dtype=float)
        omega_body = np.asarray(data.qvel[3:6], dtype=float)
        rot = np.asarray(data.xmat[int(trunk)], dtype=float).reshape(3, 3)
        x0 = np.concatenate([com, _rpy_from_matrix(rot), vel, rot @ omega_body])

        masks = contact_table(gait_params, 0.0, dt, horizon, half_step=True)
        pos_traj, vel_traj, rpy_traj, omega_traj, _p = reference_state_trajectory(
            x0, com, float(x0[5]), horizon, dt, 0.0, 0.0, float(base[2]), 0.0,
            np.array([com[0], com[1], base[2]]), float(mpc_model["max_pos_error_m"]))
        # 足端参考（落足点）——与 plan.py 同一条路径；`foot_world` 必须是 (4,3,N)，不能用单拍位置
        geometry = backend._leg_geometry(gait_params)
        hip_offsets = {}
        for code in LEG_ORDER:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                         geometry[code]["joints"]["hip_joint"])
            hip_body = int(model.jnt_bodyid[joint_id])
            delta = np.asarray(data.xpos[hip_body], dtype=float) - np.asarray(data.xpos[int(trunk)],
                                                                             dtype=float)
            hip_offsets[code] = rot.T @ delta          # 髋偏移（**机身系**，与足端参考同一口径）
        td = touchdown_parameters(gait_params, mpc_model["touchdown"])
        foot_ref = foot_reference_trajectory(masks, pos_traj, vel, yaw_rotation(float(x0[5])), 0.0,
                                             hip_offsets, td["nominal_z_m"], td["pred_time_s"])
        foot_world = np.stack([foot_ref[code] for code in LEG_ORDER])
        ad, bd, gd = dyn.discrete_dynamics(dt, mass, inertia, rpy_traj, foot_world)

        # 判据 1：u = 0 时的重力语义
        pred_zero = (ad @ x0).reshape(12) + np.asarray(gd, dtype=float).reshape(12)
        dv_z_pred = float(pred_zero[8] - x0[8])
        dv_z_expected = -gravity * dt
        gravity_residual = abs(dv_z_pred - dv_z_expected)

        # 判据 2：独立 RK4 对照（u = 0）
        rk4 = _rk4_step(x0, dt, mass, inertia, gravity, np.zeros(3), np.zeros(3))
        rk4_residual = float(np.max(np.abs(rk4 - pred_zero)))

        results.append({
            "case": label,
            "mass_kg": mass,
            "dt_s": dt,
            "gravity_residual_mps": gravity_residual,
            "gravity_expected_mps": dv_z_expected,
            "gravity_predicted_mps": dv_z_pred,
            "rk4_residual": rk4_residual,
            "rk4_max_abs": float(np.max(np.abs(rk4))),
            "passed": bool(gravity_residual <= 1e-9 and rk4_residual <= tol),
        })

    failed = [item["case"] for item in results if not item["passed"]]
    report = {
        "schema_version": REPORT_SCHEMA,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "config": str(args.config),
        "robot_config": str(args.robot_config),
        "mpc_model": {"horizon": horizon, "gait_hz": mpc_model["gait_hz"],
                      "dynamics_consistency_tol": tol},
        "time_step_s": dt,
        "cases": results,
        "passed": not failed,
        "failed_cases": failed,
    }
    print("口径 B：动力学自洽性（%d 例）" % len(results))
    for item in results:
        print("  %-12s 重力残差=%.3e m/s（期望 %+.6f / 预测 %+.6f）  RK4 残差=%.3e  %s"
              % (item["case"], item["gravity_residual_mps"], item["gravity_expected_mps"],
                 item["gravity_predicted_mps"], item["rk4_residual"],
                 "通过" if item["passed"] else "**失败**"))
    path = ROOT / args.report
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("报告：%s" % path)
    return EXIT_OK if not failed else EXIT_FAILED


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy], dtype=float)


def _rpy_from_matrix(rot):
    return np.array([np.arctan2(rot[2, 1], rot[2, 2]),
                     np.arcsin(max(-1.0, min(1.0, -rot[2, 0]))),
                     np.arctan2(rot[1, 0], rot[0, 0])], dtype=float)


def geometry_foot(model, mujoco, code):
    """足端 body id（与 gait 的腿部身份同一来源：声明的 leg→body 映射）。"""
    name = "%s_foot" % code
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body < 0:
        raise SystemExit("模型里找不到足端 body %r" % name)
    return body


if __name__ == "__main__":
    raise SystemExit(main())
