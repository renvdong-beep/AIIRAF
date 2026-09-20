# 2026-09-20 后端去机型隐式默认值：夹爪几何必须显式声明

关联：AGENTS.md 铁律 6.2（禁止隐式默认值、隐藏耦合）、6.3（新机型只填声明、
不复制代码）、6.5（只依赖能力，不依赖具体模型名/设备路径）。

## 1. 问题

MuJoCo 后端在"夹爪几何"上带了一批**指向 Piper 的隐式默认值**：

| 位置 | 原缺省值 |
| --- | --- |
| `calibrate_grasp` | `wrist_body="link6"`、`left/right_finger_body="link7"/"link8"` |
| `calibrate_grasp` | 指腹 geom **写死** `piper_left_finger` / `piper_right_finger` |
| `_log_pick_phase` | `wrist_body="link6"` |
| `_grasp_alignment_evidence` | `left/right_finger_geom="piper_left_finger"/"piper_right_finger"` |
| `_grasp_alignment_evidence` | geom 缺失时**回退到 body 原点** |
| 报错文案 | "未配置 Piper 夹爪接触信息"、"Piper 夹爪配置缺少字段" |

问题不在于"这些值恰好是 Piper 的"，而在于换构型后有两种失败模式，且都难定位：

1. **静默用错几何**：配置忘写某个字段时，后端拿 Piper 的名字去查；
   若新模型里恰好也有同名 geom/body 就会算出错误几何而不报错。
2. **运行时才炸**：新模型里没有同名 body 时抛 `body not found: link6`
   （本仓历史实测），报错点离配置缺失点很远。

另外 `UR5e + 2F-85` 的 pad body 原点在铰链附近、与 pad box 中心相差约 2cm，
"geom 缺失就回退 body"这条规则会让对齐门禁把正确抓取误报成 94mm 偏差，
属于同一类"隐式兜底制造假象"。

## 2. 改法

1. 后端新增模块级 `GRIPPER_GEOMETRY_FIELDS` 与 `_declared_gripper_geometry()`：
   五个字段必须**全部显式声明**，缺失即
   `夹爪几何声明不完整，缺少字段: [...]（必须由 profile/config 声明，不允许隐式默认值）`。
   - `wrist_body`
   - `left_finger_body` / `right_finger_body`
   - `left_finger_geom` / `right_finger_geom`
2. `_parse_manipulation_config` 把这五个字段并入必需集合 —— **装配期**即失败，
   不再等到抓取时才报错。
3. 删除 `calibrate_grasp` / `_log_pick_phase` / `_grasp_alignment_evidence` 的
   body 与 geom 回退；geom 不存在时用新助手 `_require_geom_position` 显式失败。
4. 报错文案去掉机型名。
5. **产者侧补齐声明**（这是本条的契约要求）：`scripts/build_piper_pick_scene.py`
   原先写死 `link7/link8` 且不产出 geom 名，现改为从
   `model.bodies` / `model.finger_geoms` 读取并写入场景 report。
   两个 viewer 入口（`view_piper_mujoco.py`、`run_piper_random_view.py`）的字段
   白名单同步补齐，否则会在传给后端时把这些字段静默丢掉。
6. 新增 2 项回归测试：五个字段逐个缺失都必须失败；
   `_declared_gripper_geometry` 的正/负路径。

## 3. 对新机型的契约（接入清单）

场景 report 的 `gripper` 段**至少**要包含：

```
wrist_body, left_finger_body, right_finger_body,
left_finger_geom, right_finger_geom,
open_positions, closed_positions            # 必填
approach_positions, grasp_positions, home_positions, lift_positions   # 抓取所需
pad_boxes                                   # 可选：声明后对齐门禁与 IK 同口径
gravity_feedforward                         # 可选：纯 PD 执行器的逐段伺服前馈
lift_constraint / lift_anchor_body          # 可选：锚点约束搬运（纯摩擦抬升时不提供）
```

## 4. 验证

```
Piper  verify_piper_pick.py  SUCCEEDED  口径=finger_pair   位置误差 0.002826m（与改动前一致）
UR5e   verify_ur5_pick.py    SUCCEEDED  口径=grip_region   位置误差 0.000119m
单元测试 231 项（原 229 + 新增 2），失败项仍为此前已有的 1 项 + 4 项导入错误
```

Piper 场景需重新生成一次（`build_piper_baseline.py`），因为旧的 report 不含
新增的几何声明字段：

```
PYTHONPATH=src python3 scripts/build_piper_baseline.py \
  --baseline config/piper_simulation_baseline.yaml \
  --scene build/models/piper-pick-scene.xml
```

## 5. 仍未处理（下一步）

- 视觉入口仍耦合 Piper：`_visual_pick` 默认读
  `build/calibration/piper-vision-target.json`，并写死
  `scripts/detect_piper_target.py` / `detect_piper_targets.py` 路径。
  按 6.5 应改为**声明式视觉 Provider**（检测器入口与证据路径由配置/profile 声明），
  与本次改动同一性质，但需要新的配置字段与 Provider 边界设计，单独一步做。
- `calibrate_grasp` 的证据字段 `tcp_offset_from_link6_m` 是 Piper 命名，
  已新增通用别名 `tcp_offset_from_wrist_m` 并保留旧字段；
  等调用方（`scripts/plan_piper_pregrasp.py`、`test_calibrate_grasp.py`）
  迁移完再移除旧名。
