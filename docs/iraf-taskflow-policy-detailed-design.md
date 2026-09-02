# IRAF TaskFlow 与 Policy Gateway 详细设计

**状态**：编码前基线  
**实现范围**：`core/taskflow/`、`core/policy/`、`profiles/safety/`

## 1. TaskFlow DSL 与持久化

首期采用声明式状态机 DSL，不在 MVP 引入可动态执行任意代码的行为树脚本。每个 TaskFlow 编译为确定性 `TaskPlan`，版本与输入摘要在开始时冻结。Task 实例和事件采用 PostgreSQL/SQLite 可替换 EventStore；运行状态可由事件重建，禁止只存内存。

```yaml
apiVersion: iraf.intewell.io/v1
kind: TaskFlow
metadata: {name: delivery_handoff, version: 1.0.0}
spec:
  inputs: {object_id: {type: string}, delivery_pose: {type: pose}}
  resources:
    required:
      - {name: handoff_zone, scope: task, mode: exclusive}
      - {name: piper.arm, scope: execution, mode: exclusive}
      - {name: quadruped.base, scope: execution, mode: exclusive}
    acquisitionOrder: [handoff_zone, quadruped.base, piper.arm]
  deadlineSeconds: 180
  states:
    VERIFY_READY:
      type: guard
      conditions: [piper.ready, quadruped.ready, handoff_zone.clear]
      onSuccess: DOG_DOCKING
      onFailure: HANDOVER_REQUIRED
    DOG_DOCKING:
      type: skill
      skill: dock_for_handoff
      inputs: {dock_pose: "$context.handoff_pose"}
      onSuccess: ARM_PERCEIVE
      onFailure: SAFE_HOLD
    ARM_PERCEIVE:
      type: skill
      skill: detect_object
      inputs: {object_id: "$input.object_id"}
      onSuccess: ARM_GRASP
      onFailure: RECOVER_PERCEPTION
    ARM_GRASP:
      type: skill
      skill: pick_object
      inputs: {object_pose: "$output.ARM_PERCEIVE.object_pose"}
      onSuccess: PLACE_IN_TRAY
      onFailure: SAFE_HOLD
    PLACE_IN_TRAY:
      type: skill
      skill: place_object
      inputs: {target_frame: tray_frame}
      onSuccess: DOG_CONFIRM
      onFailure: SAFE_HOLD
    DOG_CONFIRM:
      type: skill
      skill: accept_payload
      onSuccess: DOG_DELIVER
      onFailure: SAFE_HOLD
    DOG_DELIVER:
      type: skill
      skill: navigate
      inputs: {destination: "$input.delivery_pose"}
      onSuccess: SUCCEEDED
      onFailure: SAFE_HOLD
    RECOVER_PERCEPTION:
      type: retry
      target: ARM_PERCEIVE
      maxAttempts: 2
      backoffMs: 500
      onExhausted: HANDOVER_REQUIRED
    SAFE_HOLD:
      type: safe_hold
      onOperatorResume: VERIFY_READY
      onOperatorAbort: FAILED
    SUCCEEDED: {type: terminal, result: succeeded}
    HANDOVER_REQUIRED:
      type: handover
      onOperatorResume: VERIFY_READY
      onOperatorAbort: FAILED
    FAILED: {type: terminal, result: failed}
```

DSL 必须有独立 JSON Schema 与版本化编译器。允许的 `$input`、`$context`、`$output.<state>` 引用在编译期做类型、可达性和未定义状态检查。首期节点类型固定为 `guard/skill/retry/safe_hold/handover/terminal`，不支持任意脚本和并行节点；后续并行必须先定义资源合并与取消语义。循环只能通过显式 `retry` 声明，必须有 `maxAttempts`、退避和总 deadline。

guard、precondition 和 interlock 使用同一受限表达式语言：只允许字段引用、字面量、`==/!=/</<=/>/>=`、`and/or/not` 和集合包含；禁止函数调用、I/O、时间读取、循环和动态求值。编译器对字段 registry 做类型检查，并把表达式编译器版本写入 TaskPlan/PolicyDecision。

## 2. 执行状态和恢复

Task 状态：`CREATED -> VALIDATING -> RUNNING -> SUCCEEDED`；非终态包括 `WAITING_RETRY`、`SAFE_HOLD`、`HANDOVER_REQUIRED`，终态包括 `SUCCEEDED/FAILED/CANCELLED/SAFETY_STOP`。每次状态转换写入 `TaskEvent`：TaskFlow/输入摘要、当前节点、关联 execution_id、触发原因、时间和经认证的操作者。重启后恢复器读取末个事件：对终态不再动作；对运行中 Skill 先查询 Runtime 快照，再按照节点的 resume policy 恢复、失败或请求接管，绝不盲目重发。

`SAFE_HOLD` 动作是：取消未完成的非安全 Skill、请求两个本体保持/停止、保留必要诊断上下文。仍在运动或未确认 safe state 的资源转为 `QUARANTINED`，不能提前释放。人工恢复须由 transport 认证上下文证明具备 `task.resume` 权限，并记录理由、复位检查和新的 PolicyDecision。

## 3. Policy Gateway 决策链

Policy Gateway 是唯一的准入点，输入为 `SkillGoal + AuthenticatedContext + ProfileSnapshot + RuntimeInventory`，输出为 `ValidatedGoal` 或拒绝。`AuthenticatedContext` 来自受信 transport interceptor，不能由请求字段提供。顺序不可改变：

```text
认证身份 -> RBAC/租户 -> IDL/JSON Schema -> manifest 前置条件
-> 参数可覆写范围 -> Robot Profile -> Safety Policy -> 能力/健康/模式 -> 执行授权
```

Policy 使用声明式 YAML 规则，不执行来自应用或模型的表达式。规则冲突时按“拒绝优先、范围交集、最小权限”合并。`SafetyPolicy` 可设置最大速度、加速度、工作区、禁区、人机距离、负载、操作员在场要求和允许的 Skill/Provider；`RobotProfile` 不能覆盖 SafetyPolicy。

```yaml
kind: SafetyPolicy
metadata: {name: handoff_lab, version: 1.0.0}
spec:
  default: deny
  roles:
    app.delivery: [task.submit, skill.execute.navigate]
    operator: [task.resume, skill.execute.stop]
  motion:
    piper: {maxLinearSpeedMps: 0.15, forbiddenZones: [human_zone]}
    quadruped: {maxLinearSpeedMps: 0.30, requireLidarForNavigate: true}
  interlocks:
    - when: skill == place_object
      require: [quadruped.speed == 0, quadruped.stable, tray.available]
```

每次允许/拒绝均产生不可修改的 `PolicyDecision`，包含规则 ID、输入摘要、Profile/Policy 摘要、最终约束和 trace ID；不得记录原始图像、密钥或不必要的个人数据。

TaskEngine 调用 SkillRuntime 时使用短期签名 `TaskDelegation`，其中固定 task_id、原始 authenticated subject、允许的 Skill/版本、Robot/Profile digest、资源范围和 expiry。Policy Gateway 同时验证 TaskEngine 工作负载身份、委托约束和原调用者权限；TaskEngine 的系统身份不能自行扩大用户权限。

## 4. 编码接口与关键测试

```cpp
class PolicyGateway {
 public:
  Expected<ValidatedGoal, PolicyDenial> AuthorizeAndValidate(
      const AuthenticatedContext&, const SkillGoal&,
      const RuntimeInventory&, const ProfileSnapshot&);
};
class TaskEngine {
 public:
  TaskHandle Start(const AuthenticatedContext&, const TaskRequest&);
  CommandResult Cancel(const AuthenticatedContext&, TaskId);
  CommandResult Resume(const AuthenticatedContext&, TaskId, std::string reason);
};
```

测试必须覆盖：未授权、参数越界、Profile 缺能力、急停、资源冲突、超时、重复 Task 请求、进程重启恢复、人工接管审计和 `place_object` 双机互锁。Property test 验证任意参数不能使最终约束大于 SafetyPolicy 上限。
