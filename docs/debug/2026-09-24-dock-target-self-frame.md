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

## 6. 仍未定位：`halt_at_stance` 在 `command_provider` 下没有生效（D2）

实测数字（同一份 `dock-measure.json`，节拍全部零指令）：

```
settled_at_s = 0.0            → 逐拍 provider 返回**精确零**
control_cycles = 6150         = 61.5 s × 100 Hz
base_yaw_deg   0.0007° → 4.9650°（0.0809°/s）；xy 位移 0.0237 m
末拍四腿法向力 FL 9.40 / FR 67.16 / RR 0.00 / RL 59.62 N  → 仍是 trot 力型（有腿在空中）
四腿同时接触（> 声明阈值 2.0 N）的拍数 = 896 / 6150（14.57%）→ 触发窗口充足
```

代码读解：冻结只作用于**形状目标**（`target_provider` 内 `elapsed = frozen_elapsed`），
而 QP 的接触表与 B1 权重仍用**未冻结**相位（`plan_fn` 用 `self.data.time - onset`）
⇒ "站定"在语义上不自洽（形状说四足落地、QP 仍按 trot 给两条腿下力）。
判据缺口：`halted_at_s` 写进了 `state_holder` 但**没有进报告**，因此从报告无法判定门禁是否触发过
（本轮先补这个字段，再谈修；修完必须复跑四工况并证明**未触发场景逐位不变**）。

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
