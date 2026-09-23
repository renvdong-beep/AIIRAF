"""单测：`mpc.plan`（A6a-④ ②b）—— 组装顺序、协议编码与门禁。

本文件**只做结构与接口级**验证（形状/键/协议校验/缺键显式失败）。
`x0/Ad/Bd/gd/x_ref` 与上游的**逐位**对照需要在"上游 MuJoCo 路径（同一份 vendor MJCF）"上做探针，
尚未完成（见 docs/debug/2026-09-23-a6a4-state-bridge-facts.md §6）；
⇒ 通过本文件**不等于**可接 `locomote()`。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from iraf_adapters.unitree import gait
from iraf_adapters.unitree.mpc import protocol as pr
from iraf_adapters.unitree.mpc.gait_trot import merge_trot_declaration
from iraf_adapters.unitree.mpc.plan import REQUIRED_MODEL_KEYS, build_mpc_request, time_step_s
from iraf_core.profile import load_robot_profile

ROOT = Path(__file__).resolve().parents[2]


def _inputs():
    profile = load_robot_profile(ROOT / "profiles/unitree_go2_mujoco.yaml")
    base = yaml.safe_load((ROOT / "config/go2_loopback.yaml").read_text(encoding="utf-8"))
    loc = yaml.safe_load((ROOT / "config/go2_locomote.yaml").read_text(encoding="utf-8"))
    merged = merge_trot_declaration(base, loc["mpc_gait"])
    gait_params = gait.load_gait_declaration(merged, profile.joints)
    mpc_model = dict(loc["mpc_model"])
    com_state = np.array([0.0, 0.0, 0.268730095598, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    hip_offsets = {"FL": [0.1934, 0.0465, 0.0], "FR": [0.1934, -0.0465, 0.0],
                   "RL": [-0.1934, 0.0465, 0.0], "RR": [-0.1934, -0.0465, 0.0]}
    return mpc_model, gait_params, com_state, hip_offsets


def test_time_step_matches_captured_upstream_value():
    mpc_model, _g, _s, _h = _inputs()
    assert time_step_s(mpc_model) == pytest.approx(0.020833333333333332, abs=1e-18)


def test_build_request_shape_and_protocol_validity():
    mpc_model, gait_params, com_state, hip_offsets = _inputs()
    request = build_mpc_request(
        mpc_model, gait_params, com_state=com_state, mass=15.556408,
        inertia_com_world=np.diag([0.1731519, 0.4875332, 0.5378004]),
        hip_offsets=hip_offsets, body_velocity_body=[0.0, 0.0, 0.0],
        pos_des_world=[0.0, 0.0, 0.27],
        command={"vx_body": 0.0, "vy_body": 0.0, "yaw_rate": 0.0, "z_des": 0.27},
    )
    # 协议校验必须通过（长度自洽、无 NaN、版本正确）
    checked = pr.validate_request(request)
    horizon = int(mpc_model["horizon"])
    assert len(checked["h_diag"]) == horizon * 24 == 384
    assert len(checked["lbx"]) == len(checked["ubx"]) == 384
    # 等式段 192 行（N·12）+ 摩擦段 256 行（16·N）= 448
    assert len(checked["lba"]) == len(checked["uba"]) == horizon * 12 + horizon * 16
    assert len(checked["a_rows"]) == len(checked["a_vals"]) == len(checked["a_cols"])
    assert request["meta"]["time_step_s"] == pytest.approx(0.020833333333333332, abs=1e-18)
    assert request["meta"]["horizon"] == horizon
    # 摩擦锥上界：摆动腿 4 面为 +inf（不受约束）⇒ 至少有一个 ±inf 项被协议放行。
    # 注意 `validate_request` **保持记号**（`"inf"` / `"-inf"`）：`json.dumps(allow_nan=False)`
    # 拒收 ±inf，线上格式必须用显式记号；还原成 float 会让随后的 encode 再炸一次
    # （见 `mpc/protocol.py` `_INF_TO_TOKEN` 与 `decode`）。
    tokens = {pr._INF_TO_TOKEN[float("inf")], pr._INF_TO_TOKEN[float("-inf")]}
    inf_entries = [v for v in checked["uba"] if isinstance(v, str) and v in tokens]
    assert inf_entries, "协议未放行任何 ±inf 记号（摆动腿摩擦锥上界）"
    assert all(v == "inf" for v in inf_entries), "上界只应出现 +inf 记号"
    # 盒约束下界同样含 −inf（位置约束的绝对值上界那条）⇒ jog 一次序列化必须不炸
    assert all(isinstance(v, (int, float, str)) for v in checked["lbx"])
    pr.encode(checked)                  # 端到端：记号必须能被 JSON 编码


@pytest.mark.parametrize("missing_key", list(REQUIRED_MODEL_KEYS))
def test_missing_model_key_is_explicit(missing_key):
    mpc_model, gait_params, com_state, hip_offsets = _inputs()
    broken = {k: v for k, v in mpc_model.items() if k != missing_key}
    with pytest.raises(ValueError):
        build_mpc_request(broken, gait_params, com_state=com_state, mass=15.0,
                          inertia_com_world=np.eye(3), hip_offsets=hip_offsets,
                          body_velocity_body=[0.0, 0.0, 0.0], pos_des_world=[0.0, 0.0, 0.27],
                          command={"vx_body": 0.0, "vy_body": 0.0, "yaw_rate": 0.0,
                                   "z_des": 0.27})


def test_bad_com_state_and_missing_command_are_explicit():
    mpc_model, gait_params, com_state, hip_offsets = _inputs()
    kwargs = dict(mass=15.0, inertia_com_world=np.eye(3), hip_offsets=hip_offsets,
                  body_velocity_body=[0.0, 0.0, 0.0], pos_des_world=[0.0, 0.0, 0.27])
    with pytest.raises(ValueError):
        build_mpc_request(mpc_model, gait_params, com_state=np.zeros(11),
                          command={"vx_body": 0, "vy_body": 0, "yaw_rate": 0, "z_des": 0.27},
                          **kwargs)
    bad = np.array(com_state, dtype=float)
    bad[7] = np.nan
    with pytest.raises(ValueError):
        build_mpc_request(mpc_model, gait_params, com_state=bad,
                          command={"vx_body": 0, "vy_body": 0, "yaw_rate": 0, "z_des": 0.27},
                          **kwargs)
    with pytest.raises(ValueError):
        build_mpc_request(mpc_model, gait_params, com_state=com_state,
                          command={"vx_body": 0, "vy_body": 0, "z_des": 0.27}, **kwargs)


def test_zero_command_gives_zero_velocity_reference():
    """零指令 ⇒ 参考速度必须为 0（原地）；位置参考仍是"当前 + 0"（钳位不引入偏移）。"""
    mpc_model, gait_params, com_state, hip_offsets = _inputs()
    request = build_mpc_request(
        mpc_model, gait_params, com_state=com_state, mass=15.556408,
        inertia_com_world=np.diag([0.1731519, 0.4875332, 0.5378004]),
        hip_offsets=hip_offsets, body_velocity_body=[0.0, 0.0, 0.0],
        pos_des_world=[0.0, 0.0, 0.27],
        command={"vx_body": 0.0, "vy_body": 0.0, "yaw_rate": 0.0, "z_des": 0.27},
    )
    g = np.asarray(pr.validate_request(request)["g"], dtype=float)
    horizon = int(mpc_model["horizon"])
    # g = −2·q_diag ⊙ x_ref（列优先拍平）＋ 力段 0 ⇒ 力段必须恒为 0
    assert np.allclose(g[horizon * 12:], 0.0)
    assert np.all(np.isfinite(g))
