# Piper 多模式控制与 MuJoCo 实施设计

**状态**：M0-M4 编码输入  
**基线**：Ubuntu 22.04、ROS 2 Humble、MuJoCo、Piper 眼在手

## 1. 两个正交维度

Piper 的策略模式与执行后端必须分离：

| 维度 | 选项 | 说明 |
|---|---|---|
| 策略 Provider | `classical`、`vla_guided`、`learned_policy`、`teleop`、`replay` | 决定如何生成受约束动作方案 |
| 执行后端 | `mujoco`、`hardware`、`hil` | 决定状态来源和确定性控制目标发往何处 |

应用始终调用 `detect_object/pick_object/place_object/stop/recover`。Provider Router 根据 RobotProfile、运行模式、策略偏好、模型健康和 SafetyPolicy 选择组合，例如 `pick_object.classical@mujoco` 或 `pick_object.vla_guided@hardware`。禁止在 TaskFlow 中判断具体模型、仿真器、topic 或 CAN 设备。

## 2. 控制链

```text
SkillGoal
  -> Provider Router
  -> ControlAuthorityManager（单写者、lease、fencing token）
  -> 策略 Provider
       classical: perception -> grasp planner -> MoveIt2
       vla_guided: observation -> VLA -> ActionProposal
       learned_policy: observation history -> ACT/Diffusion/RL -> ActionProposal
       teleop/replay: operator/dataset -> ActionProposal
  -> Action Normalizer（统一坐标、单位、时间和 schema）
  -> Policy Gateway + Safety Projector（限位、速度、碰撞、工作区、互锁）
  -> Deterministic Executor（MoveIt2/ros2_control/局部控制器）
  -> MuJoCoAdapter / PiperHardwareAdapter
```

VLA 和学习策略只能输出 `ActionProposal`，例如目标物体、末端位姿候选、夹爪动作、短时动作块或策略置信度；不得输出 CAN 帧、电机电流或绕过安全投影的关节命令。最终轨迹插值、限位、碰撞检测和停止由确定性执行器负责。

## 3. 策略 Provider 合约

```yaml
kind: PolicyProvider
metadata: {name: piper_pick_vla, version: 0.1.0}
spec:
  mode: vla_guided
  skills: [pick_object, place_object]
  observations: [eye_in_hand_rgb, eye_in_hand_depth, joint_state, gripper_state]
  outputSchema: schemas/action_proposal.v1.json
  actionSpace: cartesian_delta_with_gripper
  horizon: {maxSteps: 16, maxDurationMs: 800}
  requires: [model.vla, arm, gripper, eye_in_hand_camera]
  fallback: piper_pick_classical
  safetyClass: controlled_motion
```

`ActionProposal` 必须包含 proposal_id、source observation sequence、frame、时间范围、动作空间、候选动作、置信度和 model/version digest。Observation 已过 TTL、坐标系未知、动作超出 horizon 或模型版本不符时直接拒绝。

## 4. 模式仲裁和切换

`ControlAuthorityManager` 保证同一执行器只有一个写控制源：

```text
IDLE -> ACQUIRING -> ACTIVE -> DRAINING -> IDLE
任意状态 -> SAFETY_STOP -> RESET_REQUIRED -> IDLE
```

模式切换必须先停止接收旧 Provider proposal，等待当前确定性轨迹停止/完成，确认 safe hold，再生成新 lease 和 fencing token。禁止 VLA、MoveIt2、RViz、teleop 同时发布 Piper 控制 topic。Provider 超时或低置信度时，只有 TaskFlow 明确允许且重新通过 Policy Gateway 才能切换到 fallback；不得在动作中静默切换。

## 5. MuJoCo 模型与 ROS 2 桥

模型资产的事实来源和派生关系如下：

```text
锁定 Piper URDF/Xacro + mesh + joint limits
  -> 受控转换/人工校核
  -> Piper MJCF（惯量、接触、摩擦、actuator、camera）
机器狗 URDF/MJCF + camera + lidar + tray
  -> handoff_lab.xml
```

URDF 与 MJCF 必须运行 joint name/limit、零位、TCP、camera extrinsic、collision geometry 和质量惯量一致性测试。自动转换结果不能未经校核直接作为验收模型。

`SimulationAdapter` 对上提供与真机相同的 RobotState、Observation、SkillFeedback、SafetyEvent 和 ros2_control 接口；对下封装 MuJoCo C API。首选评估 `mujoco_ros2_control`，但在 Humble 兼容性、许可证或稳定性不满足时使用最小 `iraf_mujoco_bridge`。桥实现是可替换细节，不能泄漏进 Skill 合约。

## 6. 数据闭环

每次执行记录策略模式、Provider/model/controller digest、Observation sequence、ActionProposal、安全投影前后摘要、轨迹结果和终态。图像与高频状态写对象存储，事件流只保留 URI/digest。传统控制成功轨迹可形成 VLA/策略训练数据，但必须脱敏、版本化并区分仿真/真机来源。

## 7. 模式验收矩阵

| 验收 | classical | vla_guided | learned_policy | teleop/replay |
|---|---|---|---|---|
| 同一 Skill 输入输出 | 必须 | 必须 | 必须 | 必须 |
| Policy/Safety Projector | 必须 | 必须 | 必须 | 必须 |
| cancel/safety stop | 必须 | 必须 | 必须 | 必须 |
| 模型/网络断开 | 不依赖 | 明确失败/受控 fallback | 明确失败/受控 fallback | 输入断开即停止 |
| MuJoCo 场景回放 | 50 次基线 | 固定模型/种子评测 | 固定 checkpoint/种子评测 | 确定性 replay |
| 真机放行 | M4 后 HIL | 单独安全评审 | 单独安全评审 | 操作员权限与死手开关 |

首期 M4 必须完成 `classical@mujoco` 全链路和 `vla_guided@mujoco` 的受约束 ActionProposal 演示；学习策略、真机 VLA 和自动 fallback 不作为首个 MVP 的强制门禁。
