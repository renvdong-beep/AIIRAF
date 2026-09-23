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

__all__ = ["REQUIRED_MODEL_KEYS", "REQUIRED_BOUNDS_KEYS", "REQUIRED_FRICTION_KEYS",
           "cost_diagonal", "state_cost_weights", "box_bounds", "linear_cost", "friction_rows",
           "dynamics_equality"]

#: `mpc_model` 中本模块消费的键。
REQUIRED_MODEL_KEYS = ("q_diag", "r_diag", "horizon")

#: 状态维 / 输入维（12 = 质心 6 自由度状态；4 足 × 3 维接触力）。
STATE_DIM = 12
INPUT_DIM = 12

#: 盒约束需要的声明键。
REQUIRED_BOUNDS_KEYS = ("horizon", "min_normal_force_n")


def box_bounds(contact_table, mpc_model):
    """盒约束 `(lbx, ubx)`（上游 `_compute_bounds` 的语义移植）。

    上游事实（`centroidal_mpc.py:122-163` 及其后）：
      · `nvars = horizon × (12 + 12)`、`start_u = horizon × 12`（状态段之后才是力段）；
      · 力段布局**按拍**：`[FLx, FLy, FLz, FRx, FRy, FRz, RLx, RLy, RLz, RRx, RRy, RRz]`
        （腿序 FL, FR, RL, RR —— 与 `contact.LEG_ORDER` 同一事实）；
      · 初始 `lbx = −inf`、`ubx = +inf`；
      · **摆动腿**（contact = 0）：三相 `lbx = ubx = 0`（不发力）；
      · **支撑腿**（contact = 1）：法向（每腿第 3 个分量）`lbx = min_normal_force_n`（上游硬编码
        `fz_min = 10`，"Prevent slipping"），上界保持 `+inf`。

    ⚠ 上游 `_compute_bounds` 的**尾部尚未逐行读完**（第 164 行之后）；因此本函数标为
    **未对上游逐位比对**（比对方式：用截获探针落盘的 `contact_table` + `lbx/ubx` 直接比，
    见 `build/research/mpc-repo/verify_qp_inputs.py`），比对通过前不得接 `provider_runtime`。
    """
    if not isinstance(mpc_model, dict):
        raise ValueError("mpc_model 必须是映射")
    missing = [key for key in REQUIRED_BOUNDS_KEYS if key not in mpc_model]
    if missing:
        raise ValueError("mpc_model 缺少必需键: %s" % missing)
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    fz_min = float(mpc_model["min_normal_force_n"])
    if not fz_min >= 0:
        raise ValueError("mpc_model.min_normal_force_n 必须非负，实际 %r" % (fz_min,))
    mask = np.asarray(contact_table)
    if mask.ndim != 2 or mask.shape[0] != 4 or mask.shape[1] != horizon:
        raise ValueError("contact_table 形状必须为 (4, horizon)=(4, %d)，实际 %r"
                         % (horizon, mask.shape))
    if not np.all(np.isin(mask, (0, 1))):
        raise ValueError("contact_table 只能含 0/1，实际取值 %r" % (np.unique(mask).tolist(),))

    nvars = horizon * (STATE_DIM + INPUT_DIM)
    start_u = horizon * STATE_DIM
    lbx = np.full(nvars, -np.inf, dtype=float)
    ubx = np.full(nvars, np.inf, dtype=float)
    for k in range(horizon):
        for leg in range(4):
            base = start_u + 12 * k + 3 * leg
            if int(mask[leg, k]) == 0:                 # 摆动腿：三相恒零
                lbx[base:base + 3] = 0.0
                ubx[base:base + 3] = 0.0
            else:                                      # 支撑腿：法向下界
                lbx[base + 2] = fz_min
    return lbx, ubx


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


#: 摩擦锥块需要的声明键。
REQUIRED_FRICTION_KEYS = ("horizon", "mu")


def friction_rows(contact_table, mpc_model):
    """摩擦锥（金字塔）不等式块：行三元组 + 上界（上游 `_precompute_friction_matrix` +
    `_update_sparse_matrix` 的摩擦部分）。

    上游事实（`centroidal_mpc.py:263-279, 324-359`，逐行读到）：
      · 行数 `n_ineq = 4 腿 × 4 面 × N = 16·N`（N=16 ⇒ 256），位于 `A` 的**后** 256 行；
      · 行序 = 外层**拍** `k`、内层**腿**（0=FL,1=FR,2=RL,3=RR，同 `LEG_ORDER`）、每腿 4 面；
      · 列 = `baseU + k·NU + 3·leg + {0,1,2}`，`baseU = N·NX`；
      · 系数：`+fx − MU·fz`、`−fx − MU·fz`、`+fy − MU·fz`、`−fy − MU·fz`（即 4 面金字塔 ≤ 0）；
      · 上界：**支撑腿** 4 行 = `0.0`、**摆动腿** 4 行 = `+inf`（不受约束）；`lba` 侧统一 `−inf`。

    返回 `{"rows","cols","vals","n_rows","upper_bound","lower_bound"}`（`upper/lower_bound` 长度
    均为 `n_rows`；`lba` 的 −inf 与等式段拼接由调用方做）。
    """
    if not isinstance(mpc_model, dict):
        raise ValueError("mpc_model 必须是映射")
    missing = [key for key in REQUIRED_FRICTION_KEYS if key not in mpc_model]
    if missing:
        raise ValueError("mpc_model 缺少必需键: %s" % missing)
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    mu = float(mpc_model["mu"])
    if not mu > 0:
        raise ValueError("mpc_model.mu 必须为正，实际 %r" % (mu,))
    mask = np.asarray(contact_table)
    if mask.ndim != 2 or mask.shape[0] != 4 or mask.shape[1] != horizon:
        raise ValueError("contact_table 形状必须为 (4, horizon)=(4, %d)，实际 %r"
                         % (horizon, mask.shape))
    if not np.all(np.isin(mask, (0, 1))):
        raise ValueError("contact_table 只能含 0/1，实际取值 %r" % (np.unique(mask).tolist(),))

    base_u = horizon * STATE_DIM
    rows, cols, vals = [], [], []
    upper = np.full(horizon * 4 * 4, np.inf, dtype=float)
    r0 = 0
    for k in range(horizon):
        uk0 = base_u + k * INPUT_DIM
        for leg in range(4):
            fx, fy, fz = 3 * leg, 3 * leg + 1, 3 * leg + 2
            for col_a, sign_a in ((fx, 1.0), (fx, -1.0), (fy, 1.0), (fy, -1.0)):
                rows.extend([r0, r0])
                cols.extend([uk0 + col_a, uk0 + fz])
                vals.extend([sign_a, -mu])
                if int(mask[leg, k]) == 1:
                    upper[r0] = 0.0
                r0 += 1
    return {"rows": rows, "cols": cols, "vals": vals, "n_rows": r0,
            "upper_bound": upper, "lower_bound": np.full(r0, -np.inf, dtype=float)}


def dynamics_equality(ad, bd, x0, gd, mpc_model):
    """动力学等式块 `A_eq` 与右端 `beq`（上游 `_assemble_A_matrix` + `_update_sparse_matrix`）。

    上游事实（逐行读到）：
      · `big_minus_Ad = diagcat([−Ad] × N)`、`big_Bd = diagcat([−Bd_k])`，`S_block` 把 `Ad` 移到
        **下一次对角** ⇒ `A_eq = horzcat(I + S_block@big_minus_Ad, big_Bd)`；
      · `beq_first = Ad@x0 + gd`（用初始状态作为"x₋₁"）、`beq_rest = repmat(gd, N−1, 1)`；
      · `lb = vertcat(beq, l_ineq)`、`ub = vertcat(beq, u_ineq)` ⇒ 等式段两侧同为 `beq`。

    结构记账（与实测 `a` nnz 精确吻合，见 docs/debug/2026-09-23-qp-builder-port-plan.md）：
    `I` 192 + `Ad` 次对角 15×144 + `Bd` 对角 16×144 = 4656（等式段）+ 摩擦 512 = **5168**。
    返回 `(A_eq, beq)`：`A_eq` 形状 `(N·12, N·24)`，`beq` 长度 `N·12`。
    """
    if not isinstance(mpc_model, dict) or "horizon" not in mpc_model:
        raise ValueError("mpc_model 缺 horizon")
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    Ad = np.asarray(ad, dtype=float)
    if Ad.shape != (STATE_DIM, STATE_DIM):
        raise ValueError("Ad 形状必须为 (12, 12)，实际 %r" % (Ad.shape,))
    Bd = np.asarray(bd, dtype=float)
    if Bd.shape == (horizon * STATE_DIM, INPUT_DIM):
        Bd = Bd.reshape(horizon, STATE_DIM, INPUT_DIM)
    if Bd.shape != (horizon, STATE_DIM, INPUT_DIM):
        raise ValueError("Bd 形状必须为 (N, 12, 12) 或 (N·12, 12)，实际 %r" % (Bd.shape,))
    x0 = np.asarray(x0, dtype=float).reshape(-1)
    gd = np.asarray(gd, dtype=float).reshape(-1)
    if x0.size != STATE_DIM or gd.size != STATE_DIM:
        raise ValueError("x0/gd 长度必须为 12，实际 %r/%r" % (x0.size, gd.size))

    n = horizon * STATE_DIM
    shift = np.zeros((n, n))                     # S_block @ diag(−Ad)：−Ad 落在下一次对角
    for k in range(1, horizon):
        shift[k * STATE_DIM:(k + 1) * STATE_DIM, (k - 1) * STATE_DIM:k * STATE_DIM] = -Ad
    bd_block = np.zeros((n, n))
    for k in range(horizon):
        bd_block[k * STATE_DIM:(k + 1) * STATE_DIM, k * STATE_DIM:(k + 1) * STATE_DIM] = -Bd[k]
    A_eq = np.hstack([np.eye(n) + shift, bd_block])
    beq = np.concatenate([Ad @ x0 + gd, np.tile(gd, horizon - 1)] if horizon > 1
                         else [Ad @ x0 + gd])
    return A_eq, beq


def linear_cost(x_ref, mpc_model):
    """线性代价 `g`（上游 `_update_sparse_matrix`：`g = vertcat(vec(−2·Q·x_ref), zeros(N·NU))`）。

    上游事实：`gx_mat = −2·(Q @ x_ref)`（`Q` 为 12×12 对角，`x_ref` 为 (12, N)）⇒ 每列是
    `−2·q_diag ⊙ x_ref[:,k]`；再用 CasADi 的 `ca.vec` 拍平，而 **`ca.vec` 是列优先** ⇒ 与
    "状态段按拍排列（`base = k·NX`）"的布局自洽；力段（后 `N·NU` 项）恒为 `0`（上游只对状态加权）。
    """
    if not isinstance(mpc_model, dict) or "horizon" not in mpc_model:
        raise ValueError("mpc_model 缺 horizon")
    horizon = int(mpc_model["horizon"])
    if horizon < 1:
        raise ValueError("mpc_model.horizon 必须 ≥1，实际 %r" % (mpc_model["horizon"],))
    q_diag = _vector(mpc_model, "q_diag", STATE_DIM)
    x = np.asarray(x_ref, dtype=float)
    if x.shape != (STATE_DIM, horizon):
        raise ValueError("x_ref 形状必须为 (12, horizon)=(12, %d)，实际 %r" % (horizon, x.shape))
    gx = -2.0 * (q_diag.reshape(STATE_DIM, 1) * x)
    return np.concatenate([gx.reshape(-1, order="F"), np.zeros(horizon * INPUT_DIM)])


def state_cost_weights(mpc_model):
    """`Q_bar·x_ref` 用到的状态权重（`q_diag` 在视界上重复，**不含** 2 倍因子）。"""
    if "horizon" not in mpc_model:
        raise ValueError("mpc_model 缺少必需键: horizon")
    horizon = int(mpc_model["horizon"])
    q_diag = _vector(mpc_model, "q_diag", STATE_DIM)
    return np.tile(q_diag, horizon)
