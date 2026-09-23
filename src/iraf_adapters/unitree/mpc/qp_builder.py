"""QP 构造器（A6a-3b 第③步；**本文件目前只落"代价部分"**）。

上游 `centroidal_mpc.py` 的 QP 入参由三块组成，本模块按块推进，**每块都单独校验**：
  ① **代价**：`H = 2·diag(tile([q_diag, r_diag], N))`（384×384 对角、nnz = 384）、
     `g = −2·Q_bar·x_ref`（`Q_bar` 为状态代价在视界上的分块对角）——**本文件已落代价矩阵 H**。
  ② **接触/动力学约束**：`A = [I; A_contact]`、`lba/uba`、`lbx/ubx`（含摩擦锥、力上界）——待做。
  ③ 变量缩放接入（`qp_scaling`）与原生 solver 调用（`osqp_native`）——待做。

⚠ **当前状态（勿误读）**：**代价部分已对上上游**——2026-09-23 新写探针
`build/research/mpc-repo/verify_qp_inputs.py` 截获上游 20 个真实 QP 入参
（`build/research/mpc-repo/qp_inputs.json`：`a` 形状 `[448, 384]`、`a` nnz 5168/拍），
`cost_diagonal` 的 `h_diag` 与上游 `h` 对角 **20 拍全部逐位一致（max|Δ| = 0.0）**，
且当场抓出我首版的布局错误（按"每拍 [Q,R]"实现 ⇒ max|Δ| 99.99998）。
② 约束块（`A`/`lba`/`uba`/`lbx`/`ubx`）与 ③ 缩放+solver 接入**仍未做** ⇒
**整模块在 ②③ 完成前不得接进 `provider_runtime`。**
"""

from __future__ import annotations

import numpy as np

__all__ = ["REQUIRED_MODEL_KEYS", "cost_diagonal", "state_cost_weights"]

#: `mpc_model` 中本模块消费的键。
REQUIRED_MODEL_KEYS = ("q_diag", "r_diag", "horizon")

#: 状态维 / 输入维（12 = 质心 6 自由度状态；4 足 × 3 维接触力）。
STATE_DIM = 12
INPUT_DIM = 12


def _vector(declaration, key, length):
    if key not in declaration:
        raise ValueError("mpc_model 缺少必需键: %s" % key)
    value = np.asarray(declaration[key], dtype=float).reshape(-1)
    if value.size != length:
        raise ValueError("mpc_model.%s 必须是 %d 维，实际 %d" % (key, length, value.size))
    if not np.all(np.isfinite(value)):
        raise ValueError("mpc_model.%s 含非有限值" % key)
    if np.any(value < 0):
        raise ValueError("mpc_model.%s 不得含负值（代价权重）" % key)
    return value


def cost_diagonal(mpc_model):
    """`H` 的对角（`2·tile([q_diag, r_diag], horizon)`）与配套元信息。

    上游形态：`H = 2·diag(Q_bar)`，对角长度 = `horizon × (12 + 12)` = `16 × 24` = **384**、
    非零元 **384**（纯对角）；**布局 = 先 192 个状态项（`tile(q_diag, N)`）、后 192 个输入项
    （`tile(r_diag, N)`）**——该布局不是猜的，是对上游真实 QP 入参逐位比对确认的（见上）。
    """
    if not isinstance(mpc_model, dict):
        raise ValueError("mpc_model 必须是映射，实际: %r" % (type(mpc_model).__name__,))
    missing = [key for key in REQUIRED_MODEL_KEYS if key not in mpc_model]
    if missing:
        raise ValueError("mpc_model 缺少必需键: %s" % missing)
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    q_diag = _vector(mpc_model, "q_diag", STATE_DIM)
    r_diag = _vector(mpc_model, "r_diag", INPUT_DIM)
    # **布局经上游实测确认（2026-09-23，`build/research/mpc-repo/verify_qp_inputs.py`）**：
    # 决策向量是「先全部状态、后全部输入」⇒ 对角 = 2·[tile(q, N) ‖ tile(r, N)]，
    # 状态段 = N×12 = 192、输入段从下标 192 起（实测上游 h[12] = 2.0 = 2·q₀、h[192] = 2e-05）。
    # 我最初按「每拍 [Q,R] 交替」实现 ⇒ 与上游 max|Δ| = 99.99998（被截获探针当场抓出）。
    diagonal = 2.0 * np.concatenate([np.tile(q_diag, horizon), np.tile(r_diag, horizon)])
    return {
        "h_diag": diagonal,
        "size": int(diagonal.size),
        "nnz": int(np.count_nonzero(diagonal)),
        "horizon": horizon,
        "q_diag": q_diag,
        "r_diag": r_diag,
    }


def state_cost_weights(mpc_model):
    """`Q_bar·x_ref` 用到的状态权重（`q_diag` 在视界上重复，**不含** 2 倍因子）。"""
    if "horizon" not in mpc_model:
        raise ValueError("mpc_model 缺少必需键: horizon")
    horizon = int(mpc_model["horizon"])
    q_diag = _vector(mpc_model, "q_diag", STATE_DIM)
    return np.tile(q_diag, horizon)
