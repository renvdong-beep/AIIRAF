"""状态 → 一份完整 MPC 请求（A6a-④ ②b）：把已验证的各件按上游顺序串起来。

组合顺序（与上游 `centroidal_mpc` 每拍做的事同序，本模块不重写任何一件的数学）：
  ① 时步：`time_step = GAIT_T / horizon = 1/(gait_hz·horizon)`（声明派生，不写数字）；
  ② 接触表 `contact.contact_table`（复用 `gait.py` 的相位事实，`half_step=True` 复刻上游 +dt/2）；
  ③ 参考轨迹 `reference.reference_state_trajectory` → `x_ref_vec`；
  ④ 足端参考 `reference.foot_reference_trajectory`（落足点公式，含上游的"机身相对量"语义）；
  ⑤ 离散动力学 `dynamics.discrete_dynamics` → `(Ad, Bd, gd)`；
  ⑥ 装配 `qp_builder.assemble_qp` → ⑦ 协议编码 `protocol.build_request`。

⚠ **验证状态（诚实标注）**：本模块目前只做**结构/接口**级单测（形状、键、协议校验、缺键显式失败）。
`x0/Ad/Bd/gd/x_ref` 与上游的**逐位**对照尚未做 —— 已锁存的 `qp_inputs.json` 来自上游的
pinocchio/URDF 模型，而我们的 `x0/Ad/Bd/gd` 来自 vendor MJCF（模型不同），因此只能在
"上游 MuJoCo 路径（同一份 MJCF）"上做逐位探针（见 docs/debug/2026-09-23-a6a4-state-bridge-facts.md §6）。
⇒ 在逐位通过之前，本模块**不得**接进 `locomote()`。
"""

from __future__ import annotations

import numpy as np

from . import protocol as pr
from .contact import LEG_ORDER, contact_table
from .dynamics import discrete_dynamics
from .qp_builder import assemble_qp
from .reference import (foot_reference_trajectory, reference_state_trajectory,
                        touchdown_parameters, x_ref_vec, yaw_rotation)

__all__ = ["build_mpc_request", "time_step_s", "REQUIRED_MODEL_KEYS"]

#: 本模块从 `mpc_model` 声明里消费的键（缺一即显式失败，不设默认值）。
REQUIRED_MODEL_KEYS = ("horizon", "gait_hz", "max_pos_error_m", "touchdown")


def time_step_s(mpc_model):
    """MPC 内部离散步长 = `GAIT_T / horizon`（由 `gait_hz` 与 `horizon` 派生，不写数字）。

    与截获的上游实测一致：`gait_hz=3.0`、`horizon=16` ⇒ `0.020833333333333332`。
    """
    if not isinstance(mpc_model, dict):
        raise ValueError("mpc_model 必须是映射")
    missing = [key for key in ("horizon", "gait_hz") if key not in mpc_model]
    if missing:
        raise ValueError("mpc_model 缺少必需键: %s" % missing)
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    gait_hz = float(mpc_model["gait_hz"])
    if not gait_hz > 0.0:
        raise ValueError("mpc_model.gait_hz 必须为正，实际 %r" % (mpc_model["gait_hz"],))
    return 1.0 / (gait_hz * float(horizon))


def build_mpc_request(mpc_model, gait_params, *, com_state, mass, inertia_com_world,
                      hip_offsets, body_velocity_body, pos_des_world, command, t0=0.0):
    """组装一份 MPC 请求（协议请求字典，可直接交给 `MpcProcessClient.call`）。

    参数（全部由调用方给出，本模块不内置任何数字）
    ----
    mpc_model      : `config/go2_locomote.yaml` 的 `mpc_model` 段（含 `touchdown` 与 `max_pos_error_m`）
    gait_params    : `mpc.gait_trot` 合并出的 trot 声明（接触表与落足点共用同一份）
    com_state      : `(12,)` 机身状态 `[p_com, rpy, v_com, ω_world]`（`state_bridge.com_state_vector` 的输出）
    mass           : 整机质量（trunk 子树，`state_bridge.subtree_mass_inertia`）
    inertia_com_world : `(3,3)` 世界系质心惯量（同上）
    hip_offsets    : `{腿码: (3,)}` 髋关节在**机身系**的偏移（来自实测几何，不重造）
    body_velocity_body : `(3,)` 体坐标机身速度（落足点前瞻项用；上游既有"体坐标当世界系"口径）
    pos_des_world  : `(3,)` 期望质心位置（会被钳位，属**有状态**量，由调用方跨拍保留）
    command        : `{"vx_body", "vy_body", "yaw_rate", "z_des"}`（高层指令，来自请求/策略门禁）
    t0             : 相位参考时刻（s）
    """
    if not isinstance(mpc_model, dict):
        raise ValueError("mpc_model 必须是映射")
    missing = [key for key in REQUIRED_MODEL_KEYS if key not in mpc_model]
    if missing:
        raise ValueError("mpc_model 缺少必需键: %s" % missing)
    state = np.asarray(com_state, dtype=float).reshape(-1)
    if state.size != 12:
        raise ValueError("com_state 必须是 12 维 [p, rpy, v, ω]，实际 %d" % state.size)
    if not np.all(np.isfinite(state)):
        raise ValueError("com_state 含非有限值：不得据此组装 MPC 请求")
    horizon = int(mpc_model["horizon"])
    dt = time_step_s(mpc_model)
    max_pos_error = float(mpc_model["max_pos_error_m"])
    if not max_pos_error >= 0.0:
        raise ValueError("mpc_model.max_pos_error_m 必须非负，实际 %r" % (max_pos_error,))
    for key in ("vx_body", "vy_body", "yaw_rate", "z_des"):
        if key not in command:
            raise ValueError("command 缺少键 %r（高层指令必须显式给出，不设默认值）" % key)

    # ② 接触表：行序 LEG_ORDER、列 = 视界（half_step 复刻上游 compute_contact_table 的 +dt/2）
    contact = contact_table(gait_params, float(t0), dt, horizon, half_step=True)

    # ③ 参考轨迹 → x_ref
    pos_traj, vel_traj, rpy_traj, omega_traj, pos_des_out = reference_state_trajectory(
        state, state[0:3], float(state[5]), horizon, dt,
        float(command["vx_body"]), float(command["vy_body"]), float(command["z_des"]),
        float(command["yaw_rate"]), np.asarray(pos_des_world, dtype=float).reshape(3),
        max_pos_error,
    )
    x_ref = x_ref_vec(pos_traj, rpy_traj, vel_traj, omega_traj)

    # ④ 足端参考（落足点）：前瞻时间只由 touchdown_parameters 算一处
    params = touchdown_parameters(gait_params, mpc_model["touchdown"])
    foot_ref = foot_reference_trajectory(
        contact, pos_traj, np.asarray(body_velocity_body, dtype=float).reshape(3),
        yaw_rotation(float(state[5])), float(command["yaw_rate"]),
        {code: np.asarray(hip_offsets[code], dtype=float).reshape(3) for code in LEG_ORDER},
        params["nominal_z_m"], params["pred_time_s"],
    )
    foot_world = np.stack([foot_ref[code] for code in LEG_ORDER])          # (4, 3, N)

    # ⑤ 离散动力学 → ⑥ 装配
    ad, bd, gd = discrete_dynamics(dt, float(mass),
                                   np.asarray(inertia_com_world, dtype=float), rpy_traj,
                                   foot_world)
    qp = assemble_qp(mpc_model, x_ref, contact, ad, bd, state, gd)

    # ⑦ 协议编码（跨进程边界上只有数字列表；meta 只放可追溯标量）
    return pr.build_request(
        qp["h_diag"], qp["g"], qp["a_rows"], qp["a_cols"], qp["a_vals"],
        qp["lbx"], qp["ubx"], qp["lba"], qp["uba"],
        meta={"t0_s": float(t0), "horizon": horizon, "time_step_s": dt,
              "n_vars": int(qp["n_vars"]), "n_cons": int(qp["n_cons"])},
    )
