"""QP 构造器（A6a-3b 第③步；**本文件目前只落"代价部分"**）。

上游 `centroidal_mpc.py` 的 QP 入参由三块组成，本模块按块推进，**每块都单独校验**：
  ① **代价**：`H = 2·diag(tile([q_diag, r_diag], N))`（384×384 对角、nnz = 384）、
     `g = −2·Q_bar·x_ref`（`Q_bar` 为状态代价在视界上的分块对角）——**本文件已落代价矩阵 H**。
  ② **接触/动力学约束**：`A = [I; A_contact]`、`lba/uba`、`lbx/ubx`（含摩擦锥、力上界）——待做。
  ③ 变量缩放接入（`qp_scaling`）与原生 solver 调用（`osqp_native`）——待做。

⚠ **当前状态（勿误读）**：`cost_diagonal` 只有"由声明算出 `H` 对角 + nnz 计数 + 与声明自洽"的
本地校验；**尚未**与上游实际 QP 入参逐位比对（`build/research/mpc-repo/*.json` 里**没有**存过
`h_diag/g/a` 这类入参 ⇒ 需要先写一个用代理截获上游 QP 入参的探针，见
`docs/debug/2026-09-23-qp-builder-port-plan.md` §3 的更正）。**在完成该比对前不得接进 `provider_runtime`。**
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

    上游形态：`H = 2·diag(Q_bar)`，其中每拍的块是 `[Q(12), R(12)]` ⇒ 对角长度
    `horizon × (STATE_DIM + INPUT_DIM)` = `16 × 24` = **384**，且**非零元个数 = 384**（纯对角）。
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
    per_step = np.concatenate([q_diag, r_diag])
    diagonal = 2.0 * np.tile(per_step, horizon)
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
