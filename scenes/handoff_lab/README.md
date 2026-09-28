# handoff_lab 场景包（交接实验室）

Piper 机械臂 + 宇树 Go2（模型待锁定）+ 人形静态实体。本场景包是 `delivery_handoff`
「机器狗停靠 → 机械臂抓取 → 放入托盘 → 载荷确认」演示与回归的**载体**：场景是声明，不是代码。

> 状态：**部分实现**。已交付：场景构建器（步骤 13）、Go2 loopback 与适配器（步骤 15/16）、
> 四足技能层（步骤 17）、S2 脚本化执行器（步骤 18）。因此下面标注为"可跑"的场景**真的可跑**：
> `stand_stop`（站立 → 停机）与 `fault_sensor_unavailable`（传感器不可用 → 拒绝机动 + 安全动作）。
> **停靠 / 抓取 / 放置 / 载荷确认仍未交付**（U5/U6 交接集成）：`nominal` 与 `fault_sensor_loss`
> 里这些步骤要么显式登记为待交付（`pending_closed_by`/`pending_reason`），要么因为本体未接入
> S2 执行器而在预检阶段失败（退出码 3）——不得按本文声称它们可跑（`AGENTS.md` 6.4）。
> 四足本体 `capabilities: [stand, stop]`：`locomote` 未声明（首期无步态控制器），
> 它的**拒绝路径**本身是验收项。

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

## 怎么跑 S2 脚本化场景（步骤 18 起可用）

```bash
PYTHONPATH=src python3 scripts/scenario.py list
# 联合世界（狗+臂同一份 MJCF + 共享植物）并**边跑边看**：--display 只渲染、不推进，
# 时间由 owner 驻留线程推进（`plant_residency`）；实测 exit 0/passed=true，渲染只改墙钟不改判据。
DISPLAY=:0 MUJOCO_GL=glfw PYTHONPATH=src python3 scripts/scenario.py run \
    --scene scenes/handoff_lab --scenario nominal --world joint \
    --display interactive_viewer --render-hz 20 --seconds 4

PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario stand_stop
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario fault_sensor_unavailable
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario fault_sensor_loss --require-injected-faults
```

报告写到 `build/acceptance/<scene>/<scenario>/report.json`（逐步骤 `status` / 墙钟 / 实测值 / 判据 /
错误码），`simulation: true`。执行链是 `scenario.yaml` → `SkillRuntime`（TaskFlow → Policy →
租约 → Provider → 适配器 → MuJoCo），执行器不直接驱动后端。

退出码：`0` 通过 / `1` 用法错误 / `2` 声明非法或超出执行器支持范围（未支持的故障类型、
不可评测的判据）/ `3` 引用完整性（未登记的能力、本体未声明 `robot.backend` 绑定、模型未生成）
/ `4` 后端装配失败 / `5` 判据未通过。

哪些场景**现在跑不了**、为什么：`nominal` 需要 `piper` 的抓取/放置能力，而本战役的 S2 执行器
未接入 piper（其机型基线未声明 `robot.backend`）⇒ 预检阶段退出码 3；`fault_sensor_loss` 的故障
注入点在待交付的停靠步上 ⇒ 故障**不会被注入**，报告里 `faults[].injected=false` 并给出原因
（`--require-injected-faults` 会把这种情况变成退出码 5）。S1 交互模式（`interact`）本战役不做
（决策 6.A：先 S2 再 S1）。

## 看到什么 / 判据是什么

可跑子集（`stand_stop`）：四足站立（仿真时间推进 ≥3 s、末速 ≤0.05 m/s、墙钟 ≤10 s）→ 松力停机
（推进 ≥1 s、末速 ≤0.05 m/s、墙钟 ≤5 s）。判据口径是"时间推进量 + 实测末速 + 墙钟"，
**不是**稳定性分析：物理稳定性证据在 `build/acceptance/go2-loopback/report.json`（步骤 15）。
故障场景（`fault_sensor_unavailable`）：站立前置观测（载荷雷达）不可用 → 该步指令**不下发**
（`IRAF-PRECONDITION-FAILED`，绝不 SUCCEEDED），其后只允许 `safetyAction` 步骤（`stop`）执行，
进入 `SAFE_HOLD`。注入语义与诚实边界写在报告 `faults[].limitation` 里。
标称序列（`nominal`，尚未可跑）：四足站立并停靠（托盘参考系平移 ≤30 mm、偏航 ≤2°）→ 机械臂抓取
（双指接触 + 抬升 ≥20 mm + 命中 `box_01`）→ 放入托盘 → 载荷确认。

判据都写在 `scenarios.<name>.steps[].criteria` 与 `baseline.yaml#acceptance`，脚本不许写死阈值。

## 失败怎么定位

1. 先跑 `scene_check.py`：它区分「声明缺字段」（2）、「引用的文件/本体不存在」（1）、
   「生成模型里锚点不存在」（3）三类失败，逐条给中文原因。
2. `pending_refs` / `pending_steps` 里的每一项都是**未交付**而不是"已通过"：人形资产的 profile 引用
   由步骤 12 关闭；`place_object`（U6 交接集成）与停靠/载荷确认仍未交付 —— 它们出现在报告里
   即表示该能力当前不可用。注意：`s02_dock` / `f02_dock` / `s04_place_in_tray` / `s05_confirm_payload`
   的 `pending_closed_by` 仍写着"步骤 18"，但步骤 18 交付的是**执行器**而不是这些能力本身
   （属 U5/U6，超出本战役范围）——这是声明的**过期关断点**，由本步如实登记为缺口，
   不擅自改他步声明的关断目标。
3. 仿真结论必须带 `simulation: true`；本场景的任何数字都不得表述为真机或实时能力（AGENTS.md 1.7）。
4. 人形条目一律带 `evidence_level: 仅模型`：仅作静态场景实体，不代表人形运动能力（决策 4.B）。
5. `scenario.py` 的失败按退出码定位：`2` 声明非法或超出执行器支持范围（未支持的故障类型 /
   不可评测的判据）、`3` 引用完整性（未登记的能力 / 本体未接入 / 模型未生成）、`5` 判据未通过；
   报错原因里带步骤 id 与维度，不要只看"失败了"。
