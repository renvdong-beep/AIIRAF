# ADR-0002：身份、资源与 Provider 边界

**状态**：已接受（待实现验证）  
**日期**：2026-08-28  
**决策人**：架构负责人、安全负责人、运控负责人（待签名）

## 决策

- 身份只能由 mTLS/JWT/AgentOS 受信网关注入 `AuthenticatedContext`，业务消息不得声明授权角色。
- TaskFlow 声明资源 scope 与全局获取顺序；ResourceCoordinator 颁发带 TTL 和 fencing token 的 task/execution 两级租约，子执行通过受限委托继承 task lease。
- SafetyEvent 发生后资源进入 `QUARANTINED`，控制器确认安全状态并完成授权复位前不得释放。
- 第三方/跨发行版 Provider 默认独立进程，通过版本化 gRPC 或 ROS 2 Action 通信；不提供通用 C++ `.so` 插件 ABI。
- 公共 API 拆分为 Task、SkillRuntime、WorldModel、Event 四个服务。

## 验证

必须有身份伪造、旧 fencing token、迟到成功回调、进程崩溃、取消超时、隔离资源重入、流式断线续传和第三方 Provider 版本不兼容测试。
