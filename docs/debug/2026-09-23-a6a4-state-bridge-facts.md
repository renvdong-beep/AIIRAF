# A6a-④ ②：MuJoCo 状态 → MPC 输入的口径（实测事实与唯一来源）

- 日期：2026-09-23
- 范围：本文件只回答「② 的每个量从哪取、为什么、怎么复核」；不含实现。
- 现场：模型 `build/scenes/handoff_lab/handoff_lab.xml`（keyframe `home`），
  探针 `build/iraf-a6a4/state_probe.py` + 两次一次性对照（见下"复核脚本"）。

## 1. 实测事实（数字原值，勿四舍五入）

| # | 量 | 实测 | 判据 |
|---|---|---|---|
| 1 | `sum(model.body_mass)`（**全部** body） | `15.596407999999998` kg | — |
| 2 | trunk（`base_link`）**子树**质量（手工 Σ，子树 18 体） | `15.556408000000001` kg | 与 `crb[trunk][9]` **diff = 0.000e+00** |
| 3 | 场景里的**游离**物体 | `box_01` **0.040 kg，parent = world**（`tray_01` 0.350 kg 挂在 `base_link` 下，属本体） | 见 §3 缺陷 |
| 4 | 整机 COM（全 body 口径） | `subtree_com[0]` = `[-0.00162759, 0.0, 0.268105]`；手工 Σm·xipos/M 与之 max\|Δ\| = `7.806e-18` | ✅ 两种路线一致 |
| 5 | trunk 子树 COM | 手工 = `[-0.002120319043515, 4.46e-19, 0.268730095985]`；`subtree_com[trunk]` 与之 max\|Δ\| = `3.469e-18` | ✅ |
| 6 | 质心惯量（世界系、绕 trunk 子树 COM） | 手工装（`body_inertia`+`body_iquat`+`xmat`+`xipos`）vs **`crb[trunk]`** 最大差 `1.110e-16`；两者对称性均为 `0.000e+00` | ✅ 两种路线一致 |
| 7 | COM 线速度 | `mj_jacSubtreeCom @ qvel` 与 `Σ m_i·cvel_i[3:6]/M` **差 `0.000e+00`** | ✅ |
| 8 | `data.cvel` 何时新鲜 | `mj_forward` 之后与再调 `mj_fwdVelocity` 之后 **完全相同**（`changed=False`） | ✅ 不必额外调 |
| 9 | `data.crb` 何时可用 | `mj_forward` **之后即有效**（`crb[0]` 恒零 = world body 无子树质量，**不是** bug）；`mj_crb` 不改变结果 | ✅ |
| 10 | `crb[i]` / `cinert[i]` 的 10 元布局 | `[0:6]` = `(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)`，`[9]` = 质量；`cinert[i][6:9]` = `m·c`（该体 COM 相对其**体坐标原点**的偏移，世界系），`cinert[i][0:6]` 是**绕体坐标原点**（不是绕 COM）的 3×3 | 逐体反解验证（见下） |

复核脚本（一次性，落在 /tmp，结论已抄进本表）：
- `/tmp/crb_check.py`、`/tmp/subtree_check2.py`：手工装惯量 vs `crb[trunk]`；
- `/tmp/bodymass2.py`：逐体质量与父节点（定位 `box_01`）；
- `build/iraf-a6a4/state_probe.py` → `build/iraf-a6a4/state_probe.json`（COM/速度/惯量三种路线）。

## 2. ② 的唯一来源（写入实现，逐条注明出处）

```
质量            M       = Σ body_mass over trunk 子树        == crb[trunk][9]
质心位置        p_com   = data.subtree_com[trunk]            （世界系）
质心线速度      v_com   = mj_jacSubtreeCom(model, data, jacp, trunk) @ qvel
质心惯量        I_com   = crb[trunk] 的 (Ixx,Iyy,Izz,Ixy,Ixz,Iyz) 还原 3×3（世界系、绕 p_com）
角速度          ω_world = R(base quat) @ qvel[3:6]           （qvel[3:6] 是**体坐标**角速度）
```

- 「trunk body」不写死：用既有 `gait.trunk_body_id(model, mujoco, params)`（与平衡/步态同一来源）。
- **不得**用 `sum(model.body_mass)` 当机器人质量（见 §3）。
- `crb[trunk]` 与手工装两路线已互证到 `1.110e-16`；实现取 `crb`（一次读取），
  验收脚本保留手工装那条作为**独立算路**。

## 3. 发现：`robot_mass_kg()` 把场景游离物体算进机器人（待修的既有缺陷，登记 S2）

`src/iraf_adapters/unitree/unitree_go2.py:669-671` 的 `robot_mass_kg()` = `sum(body_mass)`
= **15.596408 kg**，其中含 `box_01`（0.04 kg，**parent = world**，属场景而非本体）。
本体（含已挂载的 `tray_01`）实为 **15.556408 kg** ⇒ 前馈 `mg` 偏大 `0.04 × 9.81 = 0.3924 N`
（`153.0 N` 口径上 **0.256%**），且**会随场景变化**（换场景/物体被移除即变）。

- 生产影响：平衡路径（`balance.enabled` 当前 `false`）与 MPC 的竖直力前馈；量级小但**来源错误**。
- 本变更**不改**该函数：平衡路径的验收证据（`build/acceptance/...` 的冻结基线）会随之整体位移，
  必须在**专门的窗口**里改 + 重新取证。已登记为维护债 **S2**（见持续执行队列）。
- ② 的 MPC 侧**不共用**这个函数：按 §2 走 trunk 子树（`crb[trunk][9]`）。

## 4. 必须**原样复刻**的上游语义（读了上游源码，勿"顺手修正"）

上游 `src/convex_mpc/go2_robot_data.py:176-192, 210-211, 74-97`：

1. `x_vec = [pos_com_world(3), rpy(3), vel_com_world(3), omega_world(3)]`；
   `pos_com_world = data.com[0]`（**质心**，不是 base 原点）、`vel_com_world = data.vcom[0]`。
2. **四元数顺序**：上游 `update_q` 用 `q[3:7] = [x, y, z, w]`（pinocchio 序）；
   我们的 `qpos[3:7]` 是 **`[w, x, y, z]`** ⇒ 转换必须显式，写错不报错但全错。
3. `rpy` = pinocchio `matrixToRpy`（**ZYX**），且 **yaw 逐拍解卷绕**：
   `yaw_delta = (yaw_meas − yaw_prev + π) mod 2π − π`、`yaw_cont += yaw_delta`
   ⇒ **有状态**（跨控制拍保存 `yaw_prev/yaw_cont`），与 `reference.py` 里
   `pos_des_world` 被就地钳位并跨调用保留属同一类"有状态参考量"。漏了它，±π 处会跳变。
4. `omega_world = R_body_to_world @ base_ang_vel`，其中 `base_ang_vel = dq[3:6]` 是**体坐标**角速度
   —— 与 MuJoCo 自由关节 `qvel[3:6]` 同语义（体坐标）⇒ 直接可用，**不要**再转一次。

## 5. 已知限制（写进报告，避免结论超出证据）

- 质量/惯量按 **trunk 子树**计：模型里挂在 `world` 下的物体（如 `box_01`、被夹持物）**不计入**。
  ⇒ 交接/携带工况必须在模型里把载荷挂到 trunk 子树，或在声明里加"额外载荷"项（属 A8 的输入）。
- 本文件只解决"状态从哪取"；`Ad/Bd/gd` 与 `x_ref` 的**逐位**校验仍待与上游 MuJoCo 路径
  （同一份 vendor MJCF）对照 ⇒ 在此之前 `state_bridge` **不得**接进 `locomote()`。

## 6. 下一轮实现清单（本轮已把事实固定，可直接落代码）

1. `src/iraf_adapters/unitree/mpc/state_bridge.py`
   - `ComStateTracker`（有状态：`yaw_prev/yaw_cont`）+ `com_state_vector(...)`
   - `robot_mass_inertia(model, data, mujoco, trunk_body)`（`crb[trunk]` + 手工装，双路线可选）
   - `foot_positions_world(...)`（复用既有足端几何，不重造）
2. 适配器侧只**新增**只读方法（`mpc_state()` / `mpc_mass()` / `mpc_inertia_com_world()`），
   `_run_control` 与既有路径一行不改。
3. 单测：① 与上手写数字/上一次实测值对照；② 复刻语义的负向（quat 顺序写反、yaw 不unwrap）
   必须**能被测出来**；③ 两种质量口径的差 = `0.040000` kg（把 §3 的缺陷钉成回归）。
4. 验收脚本：`scripts/verify_mpc_state_bridge.py` → `build/acceptance/mpc-a6a4-state-bridge/`。
