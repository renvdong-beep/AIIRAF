# 2026-09-20 Step C：能力契约单向化、诚实能力声明与部署配置单源化

计划：`.hermes/plans/2026-09-20_123000-generality-roadmap.md`
关联：AGENTS.md 铁律 2（能力与实现一致）、5.5（不得把未验证能力标记为可用）、
5.3（配置集中在版本化文件）、6.6（可替换、可降级、可声明）。

## 1. 装配期能力契约由双向改单向

原实现（`src/iraf_adapters/factory.py::verify_backend_contract`）两个方向都拦：

1. Profile 声明了某能力 → 后端必须有对应方法（拦）；
2. 后端实现了某运动能力 → Profile 必须声明（也拦）。

第 2 条在多机型场景下是错的。本项目实测：**同一个 MuJoCo 后端类同时服务
Piper 与 UR5e**，而 UR5e 尚未接入视觉 Provider，profile 诚实地不声明
`visual_pick` —— 双向校验直接报：

```
BackendContractError: Backend 实现了未声明的运动能力，会导致策略静默拒绝任务: visual_pick
```

也就是说，双向约束**逼着** profile 声明它做不到的能力，恰好违反铁律 5.5。

改动：

- 保留方向 1（声明必须实现，含"声明了未登记能力"）—— 这才是防悬空能力的关键；
- 方向 2 改为**记录进装配报告**：`undeclared_motion_capabilities`（该字段原本就
  存在但恒为空），语义是"本机可用但未启用的能力"，供审计；
- 3 项单测随之改写，并新增 1 项锁住方向 1 不被误删
  （`test_declared_capability_without_method_still_fails`）。

## 2. UR5e 不再声明 visual_pick

`profiles/ur5_mujoco.yaml` 的 capabilities 由
`[move_joint, pick_object, visual_pick, stop]` 改为 `[move_joint, pick_object, stop]`。

实测效果（`scripts/probe_ur5_visual_capability_gate.py`）：

```
装配期契约报告: declared=[move_joint, pick_object, stop]
                undeclared_motion_capabilities=["visual_pick"]   passed=true
提交 visual_pick 任务:  status=FAILED  error_code=IRAF-SKILL-PROVIDER-UNAVAILABLE
                        reason=RobotProfile 缺少能力: ['visual_pick']
```

即：任务在**策略/校验层**被拒，而不是落到后端才报"未提供视觉证据路径"。
接入视觉（检测器 + 相机标定 + 视觉验收）后再把该能力加回。

### 未采纳的提议（有意）

原计划还想加一条装配期校验"声明 visual_pick ⇒ 必须有 vision 来源"。
实际设计时发现它会产生**误报**：`visual_pick` 允许调用方在请求里显式给出
`vision_file`（外部感知服务落盘的证据），此时部署配置不需要声明任何来源。
因此不做该硬校验，改用两层既有保障：
后端在缺来源且请求也没给证据时**显式失败**（不会伪造成功），
以及"能力声明只写本部署真正可用的"这一约定（写进 profile 注释）。

## 3. 部署配置单源化（顺带修掉一处真实过期）

`deploy/iraf-runtime.env` 里的 `IRAF_BACKEND_CONFIG` 是手写 JSON，随契约演进静默过期，
实测已发生两次：

1. 夹爪几何字段改为必填后（commit 9091651），旧 JSON 缺
   `wrist_body` / `*_finger_geom` → 服务装配期直接失败；
2. 视觉入口声明式化后（commit 2f2bd1b），旧 JSON 没有 `vision` 段 →
   `visual_pick` 报"未提供视觉证据路径"。

处理：新增 `scripts/emit_backend_config.py`，从**场景旁挂报告**生成该 JSON
（`model_path` / `manipulation.targets` / `manipulation.gripper` / `vision`），
并在模板里写明生成命令；模板内容已按新结构重新生成（保留 `model_path` 占位符）。

验证：生成结果可被真实解析并装配（夹爪五字段齐备、vision 段存在、
契约校验 passed=true）。

## 4. 顺带补齐：viewer 入口透传 vision

`scripts/view_piper_mujoco.py`、`scripts/verify_viewer_chain.py` 原先不透传
`vision`（它们不调用 visual_pick），属于计划里的 D2 项。已补上，
避免将来在这些入口启用视觉抓取时缺声明。

## 5. 验证

```
UR5e  统一验收 SUCCEEDED（profile 去掉 visual_pick 后仍可装配与抓取）
UR5e  visual_pick 任务 → 策略层拒绝（见第 2 节实测）
Piper 统一验收 SUCCEEDED 力 0.240676/0.244112N 抬升 0.087184m 偏差 0.002826042685491991
契约校验      Piper passed=true、undeclared=[]（Piper 声明齐全）
单元测试      259 项（新增 1 项锁定单向化后的强制方向），
              失败项仍为此前已有的 1 项 + 4 项导入错误
```

## 6. 遗留

- UR5e 视觉能力（检测器 + 相机标定 + 视觉验收）仍未做：这是把
  `visual_pick` 加回 UR5 profile 的前提。
- 其它手写后端配置的入口（`render_piper_mujoco.py`、`verify_development_simulation.py`
  等经环境变量注入配置的脚本）尚未统一切到 `emit_backend_config.py`；
  它们的配置来源是部署环境文件，切换需要先统一"场景报告 → 配置"的生成路径。
- `calibrate_grasp` 证据字段 `tcp_offset_from_link6_m`（Piper 命名）仍保留，
  待调用方迁移后由其通用别名 `tcp_offset_from_wrist_m` 取代。
