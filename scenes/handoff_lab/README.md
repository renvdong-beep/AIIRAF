# handoff_lab 场景包（交接实验室）

Piper 机械臂 + 宇树 Go2（模型待锁定）+ 人形静态实体。本场景包是 `delivery_handoff`
「机器狗停靠 → 机械臂抓取 → 放入托盘 → 载荷确认」演示与回归的**载体**：场景是声明，不是代码。

> 状态：**部分实现**。场景构建器已由步骤 13 交付（下面的"怎么生成模型"可用，产物是仿真模型）；
> 但 Go2 loopback（步骤 15）、S2 执行器（步骤 18）尚未交付 —— 本场景的**运动与交接**能力
> 仍不可跑，也不得按本文声称可跑（`AGENTS.md` 6.4）。四足本体当前 `capabilities: []`
> 是事实：`stand`/`stop`/`locomote` 属 U4，逐项验收后才回填。

## 文件

| 文件 | 契约 | 作用 |
|---|---|---|
| `scene.yaml` | `iraf.scene/v1`（`config/scene.schema.json` 根结构） | 场景唯一事实来源：本体、地形/工作台、道具、传感器、光照、随机种子、`simulation: true` |
| `baseline.yaml` | `iraf.scene-baseline/v1` | 参考位形来源、各本体到机型基线的引用、场景级验收阈值 |
| `scenario.yaml` | `iraf.scenario-catalog/v1` | S2 步骤序列、成功判据、故障注入项 |
| `README.md` | — | 本文：怎么跑、看到什么、失败怎么定位 |

机型数字（关节、限位、Home、相机视角）**不在本目录复制**，仍由 `profiles/*.yaml` 与
`config/*_simulation_baseline.yaml` 持有（单一事实来源）。本目录只写"这个场景用谁、判据是什么"。

## 怎么校验（现在就能跑）

```bash
PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab; echo "exit=$?"
```

退出码：`0` 通过 / `1` 引用完整性失败 / `2` 声明缺字段或非法 / `3` 模型层失败（传感器锚点不存在）
/ `4` 要求模型但模型不存在 / `5` 要求引用全部落地但仍有待交付项 / `6` 用法错误。

声明里凡尚未交付的引用都写成 `{state: unverified, closed_by: 步骤 NN, reason: …}`，
`scene_check.py` 会把它们收进 `pending_refs` / `pending_steps`；这不是"通过"，而是**显式登记的缺口**。

模型层校验（相机/雷达/IMU 的 `anchor` 是否真的存在于生成模型里）需要生成后的模型：

```bash
PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab \
    --model build/scenes/handoff_lab/handoff_lab.xml --require-model
```

## 怎么生成模型（步骤 13 起可用）

```bash
PYTHONPATH=src python3 scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2
```

产物：`build/scenes/handoff_lab/handoff_lab.xml` 与同名 `.json` 报告（`build/` 是 gitignore 的证据区）。
厂商 MJCF 只读：构建前先与 `vendor/unitree_go2/source-lock.json` 对账 SHA-256，不一致即拒绝生成。
退出码：`0` 成功 / `1` 用法错误 / `2` 声明非法 / `3` 引用完整性失败 / `4` 厂商锁校验失败
/ `5` 模型编译或注入校验失败。

模型层校验（三个传感器锚点必须真的在生成模型里）：

```bash
PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab \
    --model build/scenes/handoff_lab/handoff_lab.xml
```

生成后的模型里：`overhead_camera` 是**世界固定相机**（绝对位姿，`anchor.entity` 只登记归属）、
`payload_lidar_site` 是挂在躯干 `base_link` 上的 site（雷达扫描契约 360 线 / 8 m，点云射线统计
属步骤 14）、`imu` 是厂商自带 site（只读引用）、`tray_01` 挂在 `tray_frame` 上。
本机型**没有夹爪**，因此报告里 `target_id`/`gripper`/`vision` 显式为 `null`。

## 怎么跑（依赖后续步骤，尚未可用）

```bash
# 步骤 18 交付后可用（当前会失败：入口不存在）
PYTHONPATH=src python3 scripts/scenario.py list
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario nominal [--viewer]
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario fault_sensor_loss
PYTHONPATH=src python3 scripts/scenario.py interact --scene scenes/handoff_lab   # S1，最后做
```

## 看到什么 / 判据是什么

标称序列（`nominal`）：四足站立稳定 ≥3 s（速度为零）→ 停靠（托盘参考系平移 ≤30 mm、偏航 ≤2°）
→ 机械臂抓取（双指接触 + 抬升 ≥20 mm + 命中 `box_01`）→ 放入托盘 → 载荷确认。
故障序列（`fault_sensor_loss`）：停靠阶段雷达不可用 → 必须进 `SAFE_HOLD`，且**禁止伪造成功**。

判据都写在 `scenarios.<name>.steps[].criteria` 与 `baseline.yaml#acceptance`，脚本不许写死阈值。

## 失败怎么定位

1. 先跑 `scene_check.py`：它区分「声明缺字段」（2）、「引用的文件/本体不存在」（1）、
   「生成模型里锚点不存在」（3）三类失败，逐条给中文原因。
2. `pending_refs` / `pending_steps` 里的每一项都是**未交付**而不是"已通过"：Go2 的 profile 引用
   （能力声明）由步骤 16 关闭、机型基线同样在步骤 16，站立参考位形由步骤 15 关闭，
   人形资产由步骤 12 关闭，`place_object` 属 U6。它们出现在报告里即表示该能力当前不可用。
3. 仿真结论必须带 `simulation: true`；本场景的任何数字都不得表述为真机或实时能力（AGENTS.md 1.7）。
4. 人形条目一律带 `evidence_level: 仅模型`：仅作静态场景实体，不代表人形运动能力（决策 4.B）。
