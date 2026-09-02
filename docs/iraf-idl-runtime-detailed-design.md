# IRAF IDL、Skill Registry 与 Runtime 详细设计

**状态**：编码前基线  
**实现范围**：`api/proto/`、`core/registry/`、`core/runtime/`、`sdk/cpp/`、`sdk/python/`

## 1. 设计边界

本模块负责把一个已声明的 Skill 请求变成可取消、可追溯的 Provider 执行。它不解释业务流程（TaskFlow 负责），不决定是否越权（Policy Gateway 负责），不实现 ROS 2/主站调用（adapter 负责）。公开接口唯一来源是 Protobuf；所有内部/ROS 2 映射均从此接口派生。

## 2. Protobuf 包与版本规则

```text
api/proto/iraf/v1/common.proto       # VersionRef、Pose、Timestamp；不承载授权角色
api/proto/iraf/v1/skill.proto        # Goal、Feedback、Result、Error、Provider
api/proto/iraf/v1/task.proto         # TaskStatus、TaskEvent
api/proto/iraf/v1/robot.proto        # RobotState、Observation、SafetyEvent
api/proto/iraf/v1/runtime.proto      # SkillRuntimeService RPC
adapters/agentos/proto/v1/bridge.proto # AgentOS 版本/生命周期管理；不进入 canonical Skill IDL
```

包名固定为 `iraf.v1`。新字段只能追加为 optional/repeated，新枚举值只能追加；禁止复用 field number、改变既有枚举语义、用 `string` 偷渡未定义结构。破坏性变更只能进入 `iraf.v2`，并提供双端兼容测试。

以下片段必须落成能够通过 `buf lint` 和 `buf build` 的真实 `.proto`，禁止文档与代码各维护一份不同定义：

```proto
syntax = "proto3";
package iraf.v1;

import "google/protobuf/struct.proto";
import "google/protobuf/timestamp.proto";

message VersionRef { string name = 1; string version = 2; string digest = 3; }
message Pose { string frame_id = 1; double x = 2; double y = 3; double z = 4;
               double qx = 5; double qy = 6; double qz = 7; double qw = 8; }

enum SkillState {
  SKILL_STATE_UNSPECIFIED = 0; SKILL_STATE_PENDING = 1;
  SKILL_STATE_VALIDATING = 2; SKILL_STATE_RUNNING = 3;
  SKILL_STATE_SUCCEEDED = 4; SKILL_STATE_FAILED = 5;
  SKILL_STATE_ABORTED = 6; SKILL_STATE_CANCELLED = 7;
  SKILL_STATE_SAFETY_STOP = 8;
}
message SkillGoal {
  string correlation_id = 1;
  string skill_name = 2;
  string skill_version_constraint = 3;
  google.protobuf.Struct inputs = 4;
  google.protobuf.Timestamp deadline = 5;
  VersionRef robot_profile = 6;
  VersionRef safety_policy = 7;
  map<string, string> labels = 8;
}
message ExecuteSkillRequest {
  string request_id = 1;
  string idempotency_key = 2;
  SkillGoal goal = 3;
}
message IrafError { string code = 1; string message_zh = 2; bool retryable = 3;
                    map<string, string> details = 4; }
message SkillFeedback {
  string execution_id = 1; uint64 sequence = 2; SkillState state = 3;
  double progress = 4; string phase = 5; google.protobuf.Struct outputs = 6;
  IrafError error = 7; google.protobuf.Timestamp occurred_at = 8;
}
message CancelExecutionRequest { string execution_id = 1; string reason = 2; }
message CancelExecutionResponse { bool accepted = 1; SkillState state = 2; }
message GetExecutionRequest { string execution_id = 1; }
message ExecutionSnapshot {
  string execution_id = 1; SkillState state = 2; uint64 last_sequence = 3;
  VersionRef skill = 4; VersionRef provider = 5; IrafError error = 6;
}
service SkillRuntimeService {
  rpc Execute(ExecuteSkillRequest) returns (stream SkillFeedback);
  rpc Cancel(CancelExecutionRequest) returns (CancelExecutionResponse);
  rpc GetExecution(GetExecutionRequest) returns (ExecutionSnapshot);
}
```

Task、WorldModel 和 Event 使用独立 service 文件，避免把业务编排、状态查询和底层 Skill 生命周期混入一个服务。完整边界见 `iraf-edge-data-detailed-design.md`；M1 必须同时提交四个服务的可编译 proto。尚未实现的方法不得在生产 server descriptor 中注册，更不得返回伪成功。

AgentOSBridge 的 `GetCompatibility/Health/Ready/Drain/Shutdown` 使用独立 bridge proto，映射四个公共服务但不改变其领域消息。Bridge proto 与 AgentOS current/previous 版本运行双端契约测试；版本不兼容时必须在接收任务前失败。

`google.protobuf.Struct` 只承载由 Skill manifest 引用的 JSON Schema 校验过的参数；核心不得根据其自由字段做业务判断。后续高频/稳定 Skill 可增加强类型 request，但仍须保留通用 `SkillGoal` 包装。

## 3. Skill manifest 与 Registry

Registry 的事实来源是签名的 `skill.yaml`，启动时校验 schema、签名、版本和依赖，不接受运行期未签名动态注册。

```yaml
apiVersion: iraf.intewell.io/v1
kind: Skill
metadata: {name: pick_object, version: 1.0.0}
spec:
  inputSchema: schemas/pick_object.input.json
  outputSchema: schemas/pick_object.output.json
  requires: [arm, gripper, eye_in_hand_camera]
  preconditions: [robot.mode == ready, safety.estop == false]
  timeoutSeconds: 30
  cancellation: supported
  recovery: [re_perceive, adjust_grasp, request_handover]
  safetyClass: controlled_motion
  providers:
    - name: piper_moveit_sim
      type: ros2_action
      selector: {mode: simulation, robot: piper_lab}
      endpoint: /piper/pick_object
```

索引键是 `(name, semver)`。请求必须携带精确版本或明确的 SemVer constraint；解析结果的 manifest digest 写入 ExecutionSnapshot，恢复时不得重新解析为新版本。Provider 选择依次按 profile capability、运行模式、selector、健康状态、资源限制和 priority 过滤；候选并列时按 `(priority, provider_name)` 确定性排序。无候选返回 `IRAF-SKILL-PROVIDER-UNAVAILABLE`。

## 4. Runtime 状态、并发与接口

```text
Execute
  -> 创建 ExecutionRecord(PENDING, version snapshot)
  -> PolicyGateway.authorize_and_validate()
  -> Registry.resolve_provider()
  -> Adapter.start() -> RUNNING feedback stream
  -> Adapter terminal result -> Runtime 写入终态并关闭 stream
```

`ExecutionRecord` 是不可变版本快照加仅追加事件流：`execution_id` 由服务端生成，`correlation_id` 可关联多个执行，`sequence` 严格递增。`idempotency_key` 的唯一域为 `(authenticated_subject, tenant, operation)`；同 key 同请求返回原执行流/快照，同 key 不同摘要返回 `IRAF-IDEMPOTENCY-CONFLICT`，绝不重新下达机器人动作。

ResourceCoordinator 支持两级租约：task-scope lease 用于跨步骤保持物理区域（如 `handoff_zone`），execution-scope lease 用于单次动作（如 `piper.arm`、`quadruped.base`）。TaskFlow 必须声明 scope 和全局获取顺序；子 execution 通过受限 delegation 使用所属 task lease，不重复竞争。所有 lease 均包含 TTL、续租、owner task/execution_id 与单调递增 fencing token。Provider 下发控制时携带 token，旧 token 必须被 adapter 拒绝。资源竞争返回 `IRAF-RESOURCE-BUSY`，不得无限等待。

核心 C++ 接口：

```cpp
class SkillProvider {
 public:
  virtual StartResult Start(const ValidatedGoal&, FeedbackSink&) = 0;
  virtual CancelResult Cancel(const ExecutionId&, CancelReason) = 0;
  virtual HealthStatus Health() const = 0;
  virtual ~SkillProvider() = default;
};
class SkillRuntime {
 public:
  ExecutionHandle Execute(const AuthenticatedContext&, const ExecuteSkillRequest&);
  CancelResult Cancel(const AuthenticatedContext&, const ExecutionId&, CancelReason);
  ExecutionSnapshot Get(const AuthenticatedContext&, const ExecutionId&) const;
};
```

`AuthenticatedContext` 只能由 mTLS/JWT/AgentOS 受信网关构造，包含 subject、tenant、roles、credential_id 和认证时间；请求消息不得自行声明角色。Provider 回调只能提交合法状态迁移。Runtime 以 compare-and-swap 持久化 `sequence`，拒绝过期/重复反馈。

收到 `SafetyEvent` 时，Runtime 立即把执行置为 `SAFETY_STOP`，将相关资源租约转为 `QUARANTINED` 并拒绝新动作，然后请求本地控制器安全停止。只有控制器确认 safe state、设备健康检查通过且授权人员完成复位后，资源协调器才能释放隔离；Provider 的迟到成功回调不得改变终态。

Skill 执行的终态仅为 `SUCCEEDED/FAILED/ABORTED/CANCELLED/SAFETY_STOP`。重试、恢复和人工接管由 TaskFlow 创建新的 execution 并关联原 execution_id，避免单个物理动作在失败后被隐式复活。

## 5. 标准错误码与测试

| 错误码 | 含义 | 是否重试 |
|---|---|---|
| `IRAF-INPUT-INVALID` | 参数不符合 manifest schema | 否 |
| `IRAF-POLICY-DENIED` | RBAC/Profile/Safety Policy 拒绝 | 否 |
| `IRAF-PRECONDITION-FAILED` | 状态、标定或能力不满足 | 视 Skill 而定 |
| `IRAF-SKILL-PROVIDER-UNAVAILABLE` | 无健康 Provider | 是，受 deadline 限制 |
| `IRAF-RESOURCE-BUSY` | 独占资源已租用 | 是，受 TaskFlow 策略限制 |
| `IRAF-IDEMPOTENCY-CONFLICT` | 同一幂等键对应不同请求 | 否 |
| `IRAF-DEADLINE-EXCEEDED` | 运行超时 | 否，转恢复/接管 |
| `IRAF-CANCELLED` | 由调用者/编排器取消 | 否 |
| `IRAF-SAFETY-STOP` | 安全事件中断 | 否，须受控复位 |

单元测试覆盖版本选择、状态机、幂等、乱序反馈、租约释放和 deadline；契约测试覆盖 C++/Python SDK；适配器测试必须验证 cancel 在限定时间内使 ROS 2 action 取消或上报失败。
