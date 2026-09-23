# 停靠目标帧自指：`dock_for_handoff` 的一次假通过（2026-09-24）

## 0. 一句话结论

`dock_for_handoff` 的「平移 3.632386e-05 m / 偏航 0.0° / 末速 2.461973e-04 m/s、failure=None」
**不是停靠精度**：目标帧 `tray_frame` 由场景声明挂在四足**躯干 body** 上
（`scene.yaml` 的 `props.tray_01.pose.mount.entity: unitree_go2`），目标帧与机身是**同一刚体**，
量到的是**自指量**，且接近过程**从未被执行**（第 0 拍就已"在位"）。
⇒ 该结论**作废**；`dock_for_handoff` 的适配器层**未达标**。

## 1. 现象与发现路径

上一轮（提交 `52cca7d` 之前的实测）报告 `build/iraf-a6a13/dock-measure.json` 显示停靠收敛到
**36 µm**，我据此在提交信息里写了「`dock_for_handoff` 的适配器层达标」。
复核报告时发现三个无法同时成立的信号：

| 观测量 | 值 | 为什么可疑 |
| --- | --- | --- |
| `final_yaw_error_deg` | `0.0`（**恰好**为零） | `pose_error` 不做吸附 ⇒ 恰好 0 只有"两个 yaw 同源"才能给出 |
| `settled_at_s` | `0.0` | 到位时刻在第 0 拍 ⇒ 接近回路一次都没跑 |
| 逐拍样本 | 末拍仍在 trot（`stance_legs=['FR','RL']`、四腿力 9.40/67.16/0.00/59.62 N） | 指令恒零却一直迈步，说明"停下"不是这个回路给的 |

## 2. 证据（可复跑，不跑新仿真）

`build/iraf-a6a14/dock_frame_probe.py`：用报告里保存的**实测末态**重建 `qpos` → `mj_forward` →
用与适配器**完全相同**的取位姿代码复算：

```
tray_frame（site id=1）所属 body = 1 base_link（躯干）
base_link 的子树 body 数 = 18（含 base_link, FL_hip, …, RL_foot）
⇒ 目标帧在机器人子树内？ True

复算 final_yaw_error_deg = 0         报告 = 0.0        yaw 差恰好 0.0？ True
复算 final_translation_error_m = 3.6346127506620565e-05
报告                                = 3.6323864375270122e-05   （相对差 6.129e-04）
site 本地位置（躯干系）= (0, 0, 0.057)  ← **纯竖直偏置**
实测末态躯干 roll = −0.018264°、pitch = +0.031642°
竖直偏置 × 倾斜投影 = (2.985773e-05, 2.072576e-05) → 模长 3.634612751e-05  ← 与复算值逐位相同
```

即报告里的「36 µm」= **帧自身的竖直偏置 0.057 m 在机身倾斜下的 xy 投影**
（`|R·(0,0,0.057)|_xy`）。它与停靠距离无关：报告值是测量那一拍的同一表达式的取值，
所以两者相差 6.129e-04（探针用的是保持段末态、不是测量那一拍）。
**偏航恰好 0.0** 同理：同一个 `xmat`，两个 `atan2` 逐位相同。

`build/iraf-a6a14/dock-guard-probe.json`（新增门禁的验收）：对真实场景模型调用
`dock_for_handoff` ⇒ **0.001 s** 显式失败，`data.time` 与 `ncon` 前后不变（**零物理步进**）。
该验收已提为可提交、可复跑的入口：`scripts/verify_dock_target_frame.py`
（报告 `build/acceptance/go2-dock-guard/report.json`，7 条判据全通过、退出码 0）。

## 3. 根因（两处，缺一不可）

1. **场景侧**：`scenes/handoff_lab/scene.yaml` 的 `props.tray_01.pose.mount` 明确把托盘
   **固定在四足背上**（"托盘固定在四足背上：位置由被挂载实体与参考系决定"）。
   构建产物 `build/scenes/handoff_lab/handoff_lab.xml` 里 `tray_01` body 与 `tray_frame` site
   都在躯干 body 内。**这是设计意图**，不是构建器 bug。
2. **适配器侧**：`dock_for_handoff` 直接把这个"背上的托盘帧"当**停靠目标**用，
   而没有任何门禁检查"目标帧是否与本体刚性相连"。⇒ 它把一个**没有意义的目标**
   算出了一个**好看的误差**。

## 4. 影响面（诚实清单）

* 作废：`52cca7d` 提交信息里「适配器层达标」的表述；`build/iraf-a6a13/dock-measure.json`
  作为"停靠证据"的资格（它仍然是"自指量 + 保持段末速"的合格证据）。
* 不变：`s02_dock` 仍**待交付**（`pending_closed_by`）—— 这一点事后看是**正确**的，
  场景契约本来就没让一步没有世界固定目标帧的停靠通过。
* 不变：四工况 `locomote` 验收（forward 0.0659 / backward 0.0571 / turn ±）与停靠无关，
  跑在 `config/go2_locomote.yaml`（无场景 props）上，**不受影响**。
* 未解释（本轮新增待办，见 §6）：`halt_at_stance` 在 `command_provider` 路径下**没有冻结相位**。

## 5. 修复（本轮已做）

1. `dock.assert_target_is_world_fixed(...)`（纯函数）：目标帧所属 body 落在
   **机器人子树**内 ⇒ 抛 `DockDeclarationError`，报错含目标帧名、所属 body、
   "刚性挂在机器人 X 上"的判定与**可操作方向**（换世界固定的交接站位帧）。
2. 适配器在**解析目标帧之后、任何接近逻辑之前**调用该门禁；
   报告新增 `target_frame_body` / `target_frame_world_fixed`（自指帧必须一眼可见）。
3. 单测 6 项（`tests/unit/test_unitree_dock_approach.py::TestTargetIsWorldFixed`）：
   躯干帧失败 / 腿帧失败 / 世界物通过 / 自由物通过 / 空子树 / id 类型强制。
4. 删掉 `settle_completed_at_s`：该字段恒为 `None`（`progress["settled_s"]` 从未被写）
   且无消费者 —— 恒 `None` 的字段会让人误以为"有这项测量"。

## 6. `halt_at_stance` 在 `command_provider` 路径下的语义不自洽（D2，已定位并修复）

**先被自己的探针推翻了一半假设**：我原先怀疑"站定门禁根本没触发"。
`build/iraf-a6a14/halt_probe.py`（6 s、基础指令 0.15 m/s、逐拍 provider 恒返回精确零）实测：

```
locomote.halt = {declared_enabled: true, period_s: 0.3333333333333333,
                 frozen_at_s: 0.36000000000000026, zero_command_since_s: 0.0}
halted 样本 563/600（首发 0.380 s）⇒ 门禁**确实触发**
但末拍 stance_legs=['FL','RR']、偏航 0.0007° → 0.6917°（≈0.115°/s）、|v| 4.166e-02 m/s
```

⇒ 真机制是：**冻结只作用于形状目标**（`target_provider` 内把 `elapsed` 换成冻结值），
而 QP 的相位（`plan_fn` 里 `t0 = self.data.time - onset`）**没有冻结** ⇒
形状说"四足落地"、QP 仍按 trot 给两条腿下力（`stance_legs` 仍交替），语义不自洽。

**修复**：把站定状态机与相位时间抽成单一实现 `phase_elapsed(elapsed, command_is_zero)`，
**形状目标与 QP 的 `t0` 共用它**；未触发站定时原样返回 `elapsed` ⇒ 非站定场景逐位不变。

修复前后（同一条 6 s 探针，逐项对照）：

| 量 | 修复前 | 修复后 |
| --- | --- | --- |
| `frozen_at_s` | 0.36 s | 0.36 s（不变） |
| 末拍偏航 | 0.6917° | **0.0449°**（15×） |
| 末拍 \|v\| | 4.166e-02 m/s | **2.1846e-04 m/s**（190×） |
| 末拍 xy | (0.01147, −0.00196) | (0.00910, 0.00308) |

**诚实边界**：站定成立（速度与偏航都稳），但**不是四足均载** —— 末拍逐腿法向力
FL 2.73 / FR 76.53 / RR 0.00 / RL 73.35 N（对角两腿 149.88 N ≈ mg 152.61 N）。
即"四足同时接触"只是**触发条件**，冻结后的支撑分布并不均匀；若停靠要"四足落地"，需要更强的
触发判据或冻结到标称站立位形（留给停靠侧决策，不在本轮改）。

证据可见性也补上了（此前不可判定）：报告新增 `halt` 段（`declared_enabled` / `period_s` /
`frozen_at_s` / `zero_command_since_s` / `frozen_elapsed_s`）、样本新增 `halted` 与
`effective_velocity`（逐拍**生效**指令 —— 此前样本里只有基础指令 0.15 m/s，
看不出"其实一直下的是零"，这是上一轮判断被误导的直接原因）。

### 6.1 证据核对：一处引用笔误更正（2026-09-24 收尾自查）

对"未触发站定 ⇒ 逐位不变"这条结论做**引用核对**时发现自己在提交信息里抄错了一个数字：

```
权威文件对账：
  build/acceptance/go2-locomote/report.json   mtime 2026-09-23 23:59:20  = **提交前基线**
  build/iraf-a6a14/four-cases-after-halt-fix.json                        = 修复后
forward.steady_signed_displacement_m  0.089674417228410491  两侧逐位相同 ✅
backward.steady_signed_displacement_m 0.090520845117540827  两侧逐位相同 ✅
```

* **结论不变**：forward / backward 在上述改动的**前后逐位相同**（这条是用两个归档报告的字段
  直接比对得到的，不依赖我在提交信息里写的字符串）。
* **更正**：提交 `db1287b` 的信息里把 backward 写成 `0.09052514384405007`（**实际
  `0.090520845117540827`**，从第 6 位有效数字起就错了）——属**抄写笔误**，不是测量差异；
  权威值以本节与两个归档报告为准。
* 顺带记一条可复现性事实：canonical 报告 `build/acceptance/go2-locomote/report.json`
  **仍是 2026-09-23 23:59:20 那份**（修复前），因此做 A/B 时必须显式指定归档报告，
  否则会把"基线"和"新跑"混在一处。同理 `turn_left/turn_right` 的差异（28.866768° vs
  28.361466°、−27.223412° vs −27.757224°）是**同一条工况链上的传递效应**（§6 已论证：
  两工况全程 `halted=False` ⇒ 转向路径未被改动），不是转向逻辑回归。

## 7. 下一步：世界固定的交接站位帧（跨界，需授权）

* **路径**：`scenes/handoff_lab/scene.yaml` 新增一个**世界固定**的站位帧
  （台面上的静态 prop/site，例：`handoff_station` + `handoff_station_frame`）；
  `config/go2_loopback.yaml` 的 `dock_for_handoff.target_frame` 指向它；
  站位坐标必须由**机械臂可达范围**定（跨到 Piper 那一侧的事实）。
* **交付物**：场景声明 + 构建产物 + 停靠实测报告（含 `target_frame_body` 为世界物、
  `settled_at_s > 0`、接近过程有位移）三层证据。
* **判据**：① `settled_at_s > 0`（接近真的跑过）；② 起点到站位有**可测位移**
  （`standoff_m` 生效，不再出生即在位）；③ 平移/偏航/末速按步骤级 `criteria` 判；
  ④ 全程 `simulation=true`、无 `damped_hold` 之外的失败。
