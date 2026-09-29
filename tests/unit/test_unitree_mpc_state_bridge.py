"""单测：`mpc.state_bridge`（A6a-④ ②b 第一块）—— 状态口径 + yaw 解卷绕 + 子树质量/惯量。

对照值来自 `docs/debug/2026-09-23-a6a4-state-bridge-facts.md` 的实测（keyframe `home`）：
  · trunk 子树质量 `15.556408000000001` kg（≠ `sum(body_mass)`：差 = **世界属主道具质量之和**，
    由 `_declared_prop_mass_kg()` 从场景声明派生 —— 写死数字会在"新增一个道具"时假失败）；
  · trunk 子树 COM `[-0.002120319044, 0.0, 0.268730095598]`；
  · 质心惯量（世界系、绕 trunk COM）手工装 vs `crb[trunk]` 最大差 `1.110e-16`。

复刻语义的**负向**也在这里钉死：四元数顺序写反、yaw 不解卷绕，都必须被测出来。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from iraf_adapters.unitree.mpc.state_bridge import (STATE_DIM, ComStateTracker,
                                                    com_state_vector, crb_to_matrix,
                                                    hand_built_inertia, matrix_to_rpy,
                                                    quat_to_rotation, quat_wxyz_to_xyzw,
                                                    subtree_mass_inertia, trunk_subtree_bodies)

ROOT = Path(__file__).resolve().parents[2]
TRUNK_REF_MASS = 15.556408000000001          # 实测（facts 文档 §1 第 2 行）


def _world_parented_prop_mass_kg():
    """**世界属主**道具的质量之和（从 `scenes/handoff_lab/scene.yaml` 派生，不写死数字）。

    口径：不带 `pose.mount` 的道具（= 直接挂在 worldbody）不进躯干子树；带 `mount` 的
    （如挂在 `base_link` 下的托盘）**属于**本体 ⇒ 不计入本函数。

    为什么派生（2026-09-29）：本文件要守的性质是"世界属主的道具不进入躯干子树"，
    不是某个具体公斤数。原先写死 `sum(body_mass)=15.596408`（差 0.04 kg = 载荷）⇒
    新增 `arm_pedestal`（5 kg 静态基座）后变成假失败、掩盖真实回归；
    同时托盘从"世界固定"改成"挂回狗背"后，这一项也从排除集移入子树 —— 只有派生才对得上。
    """
    scene = yaml.safe_load((ROOT / "scenes/handoff_lab/scene.yaml").read_text(encoding="utf-8"))
    total = 0.0
    for item in (scene.get("props") or []):
        pose = item.get("pose") or {}
        if not pose.get("mount"):
            total += float(item.get("mass_kg") or 0.0)
    return total


#: 场景总质量 = 躯干子树 + 世界属主道具（挂在载体上的道具已含在子树里）
SCENE_TOTAL_MASS = TRUNK_REF_MASS + _world_parented_prop_mass_kg()
#: 世界属主（不进本体子树）的道具质量之和 = 场景总质量 − 躯干子树质量
SCENE_EXCLUDED_MASS = _world_parented_prop_mass_kg()
COM_TOL = 1e-12
INERTIA_TOL = 1e-12


def _scene():
    mujoco = pytest.importorskip("mujoco")
    with open(ROOT / "config/go2_loopback.yaml") as handle:
        decl = yaml.safe_load(handle)
    model = mujoco.MjModel.from_xml_path(str(ROOT / decl["model"]["file"]))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, decl["initial"]["keyframe"])
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return mujoco, model, data, trunk


# ---------- 四元数 / 欧拉角 ----------

def test_quat_order_conversion_is_explicit_and_normalized():
    assert np.allclose(quat_wxyz_to_xyzw([1.0, 0.0, 0.0, 0.0]), [0.0, 0.0, 0.0, 1.0])
    out = quat_wxyz_to_xyzw([2.0, 0.0, 0.0, 0.0])          # 归一化
    assert np.allclose(out, [0.0, 0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        quat_wxyz_to_xyzw([0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        quat_wxyz_to_xyzw([1.0, 0.0, 0.0])


def test_rotation_matrix_matches_axis_rotations():
    half = math.pi / 4.0
    yaw90 = [math.cos(half), 0.0, 0.0, math.sin(half)]      # 绕 z 转 90°
    r = quat_to_rotation(yaw90)
    assert np.allclose(r @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-15)
    roll90 = [math.cos(half), math.sin(half), 0.0, 0.0]     # 绕 x 转 90°
    assert np.allclose(quat_to_rotation(roll90) @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0],
                       atol=1e-15)


@pytest.mark.parametrize("rpy", [(0.0, 0.0, 0.0), (0.3, -0.2, 1.1), (-0.05, 0.4, -2.9)])
def test_matrix_to_rpy_is_zyx_and_round_trips(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    r_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    r_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    got = matrix_to_rpy(r_z @ r_y @ r_x)
    assert np.allclose(got, rpy, atol=1e-12)


def test_matrix_to_rpy_pitch_is_clamped_not_nan():
    r = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # pitch = +90°
    out = matrix_to_rpy(r)
    assert np.all(np.isfinite(out)) and out[1] == pytest.approx(math.pi / 2.0, abs=1e-12)


# ---------- yaw 解卷绕（有状态） ----------

def test_yaw_unwrap_keeps_continuity_across_pi():
    tracker = ComStateTracker()
    assert tracker.yaw is None
    values = [3.0, 3.1, -3.1, -3.0]              # 跨 +π 的实际运动（3.1 → −3.1 是连续小步）
    got = [tracker.unwrap(v) for v in values]
    assert np.allclose(got, [3.0, 3.1, 3.1831853071795867, 3.2831853071795867], atol=1e-12)
    assert tracker.yaw == pytest.approx(got[-1])


def test_yaw_unwrap_first_call_is_measurement_itself():
    tracker = ComStateTracker()
    assert tracker.unwrap(-2.5) == -2.5
    assert tracker.unwrap(-2.5) == -2.5          # 同值 ⇒ 增量为 0


def test_state_vector_uses_continuous_yaw_and_world_omega():
    tracker = ComStateTracker()
    half = math.pi / 4.0
    quat = [math.cos(half), 0.0, 0.0, math.sin(half)]          # 绕 z 90°（wxyz）
    state = com_state_vector([1.0, 2.0, 0.27], quat, [0.1, 0.2, 0.3], [0.0, 0.0, 0.5], tracker)
    assert state.shape == (STATE_DIM,)
    assert np.allclose(state[0:3], [1.0, 2.0, 0.27])
    assert np.allclose(state[3:5], [0.0, 0.0]) and state[5] == pytest.approx(math.pi / 2.0)
    assert np.allclose(state[6:9], [0.1, 0.2, 0.3])
    assert np.allclose(state[9:12], [0.0, 0.0, 0.5])            # R@ω_body：绕 z 不变


def test_state_vector_matches_mujoco_quat_convention_direction():
    """写反四元数分量（把 w 当 x）必须得到**不同**的状态 ⇒ 该语义可被测出。"""
    tracker_a, tracker_b = ComStateTracker(), ComStateTracker()
    half = math.pi / 6.0
    quat = [math.cos(half), math.sin(half), 0.0, 0.0]           # 绕 x 30°
    correct = com_state_vector([0.0, 0.0, 0.27], quat, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], tracker_a)
    swapped = com_state_vector([0.0, 0.0, 0.27], [quat[1], quat[0], quat[2], quat[3]],
                               [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], tracker_b)
    assert not np.allclose(correct[3:6], swapped[3:6])


def test_tracker_is_required_and_state_vector_rejects_bad_inputs():
    with pytest.raises(TypeError):
        com_state_vector([0.0, 0.0, 0.27], [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3, None)
    tracker = ComStateTracker()
    with pytest.raises(ValueError):
        tracker.state_vector([0.0, 0.0, np.nan], [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3,
                             rotation=np.eye(3))
    with pytest.raises(ValueError):
        tracker.state_vector([0.0] * 3, [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3,
                             rotation=np.eye(2))


# ---------- 子树质量 / 质心 / 惯量（与实测数字对照） ----------

def test_trunk_subtree_excludes_world_parented_scene_bodies():
    mujoco, model, data, trunk = _scene()
    bodies = trunk_subtree_bodies(model, trunk)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in bodies}
    assert "box_01" not in names                 # parent=world ⇒ 不属于本体
    assert "tray_01" in names                    # 挂在 base_link 下 ⇒ 属本体
    # 子树体数**从模型与声明派生**（不写死）：全部 body − 世界属主道具（无 `pose.mount`）− world 自身。
    # 2026-09-29：托盘由"世界固定"改为"挂回狗背"（带 `mount`）⇒ 它从排除集移入子树，计数随之变化；
    # 写死 18 会变成假失败并掩盖真实回归。
    prop_rows = yaml.safe_load((ROOT / "scenes/handoff_lab/scene.yaml").read_text(encoding="utf-8"))
    world_parented = len([item for item in (prop_rows.get("props") or [])
                          if not (item.get("pose") or {}).get("mount")])
    assert len(bodies) == int(model.nbody) - world_parented - 1
    assert sum(float(model.body_mass[i]) for i in bodies) == pytest.approx(TRUNK_REF_MASS,
                                                                          abs=1e-12)
    assert float(np.sum(model.body_mass)) == pytest.approx(SCENE_TOTAL_MASS, abs=1e-12)


def test_subtree_mass_inertia_matches_measured_inertia_and_oracle():
    mujoco, model, data, trunk = _scene()
    mass, com, inertia, _bodies = subtree_mass_inertia(model, data, mujoco, trunk)
    assert mass == pytest.approx(TRUNK_REF_MASS, abs=1e-12)
    assert np.allclose(com, [-0.002120319043515, 0.0, 0.268730095985], atol=COM_TOL)
    # 内部对称 + 正定
    assert np.max(np.abs(inertia - inertia.T)) == 0.0
    assert np.all(np.linalg.eigvalsh(inertia) > 0.0)
    # 独立算路（不读 crb）对照：实测最大差 1.110e-16
    m2, com2, inertia2 = hand_built_inertia(model, data, trunk)
    assert m2 == pytest.approx(mass, abs=1e-12)
    assert np.allclose(com2, com, atol=COM_TOL)
    assert np.max(np.abs(inertia2 - inertia)) <= INERTIA_TOL
    assert np.max(np.abs(inertia2 - inertia)) <= 1e-15


def test_subtree_mass_inertia_rejects_scene_total_mass_route():
    """把 `sum(body_mass)` 当本体质量必须**可被本层的对照测出**（差 = 世界属主道具质量之和）。"""
    mujoco, model, data, trunk = _scene()
    mass, _com, _inertia, _bodies = subtree_mass_inertia(model, data, mujoco, trunk)
    assert abs(float(np.sum(model.body_mass)) - mass) == pytest.approx(SCENE_EXCLUDED_MASS, abs=1e-12)


def test_crb_layout_and_gates():
    entry = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 9.0, 9.0, 9.0, 5.0]
    matrix = crb_to_matrix(entry)
    assert matrix[0, 0] == 1.0 and matrix[1, 1] == 2.0 and matrix[2, 2] == 3.0
    assert matrix[0, 1] == matrix[1, 0] == 0.1
    assert matrix[0, 2] == matrix[2, 0] == 0.2
    assert matrix[1, 2] == matrix[2, 1] == 0.3
    with pytest.raises(ValueError):
        crb_to_matrix([1.0, 2.0, 3.0])


def test_subtree_helpers_gate_bad_arguments():
    mujoco, model, data, trunk = _scene()
    with pytest.raises(ValueError):
        trunk_subtree_bodies(model, int(model.nbody) + 5)
    # 世界体（root=0）自指：子树遍历必须**终止**并给出整场景（20 体），不得挂死
    whole = trunk_subtree_bodies(model, 0)
    assert len(whole) == int(model.nbody) and 0 in whole
    with pytest.raises(ValueError):
        subtree_mass_inertia(model, data, mujoco, 0)          # 世界体质量为 0 ⇒ 显式失败
