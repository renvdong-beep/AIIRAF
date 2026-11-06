# ADR-0012：场景、Provider 生命周期与 AgentOS 权限边界

**状态**：提案（2026-11-06）  
**背景**：x86 仿真主机由对方/我方独立运维，AgentOS 在智控域，没有 x86 权限；
现有 piper / ur5 / 联合世界（handoff_lab）三个场景需要共用一个合理、健壮的模型。

## 决策

1. **AgentOS 不获得 x86 权限**。AgentOS/上位机只提交任务、选择 `scene_id` /
   `resource_id`，不直接连 Provider，不执行 x86 命令。
2. **Provider 生命周期由 x86 本地负责**：systemd 或本地 supervisor 启动 MuJoCo +
   ProviderKit，等 Health ready；边缘 IRAF 只调用 provider.proto 的
   Health / Invoke / Cancel，不启动/停止进程。
3. **一个 MuJoCo 世界一个 Provider 进程**；一个 Provider 可承载多个机器人，
   但只有一个 ControlAuthority。
4. **Scene 是一等公民**：新增 Scene Catalog / SceneRegistry，`scene_id` 映射到
   model、robots、Profile、SafetyPolicy、capabilities、endpoint、display。
5. **部署期声明、运行期不可变**：切换场景是受控部署/重启动作，不是运行时参数；
   Profile/SafetyPolicy/Provider manifest 不得热改。
6. **联合世界使用 SceneProfile / 多 resource 声明**；禁止三个独立 Provider
   同时控制同一个 MuJoCo 世界。
7. **失败显式**：Provider 未启动/未 ready 返回
   `IRAF-TRANSPORT-FAILED` / `IRAF-PROVIDER-UNAVAILABLE`，不自动拉起、不重试风暴。

## 验证

- scene_id → manifest → endpoint/profile 一一对应；
- Provider 未起时任务显式失败，无成功 execution；
- ready 后任务成功，`iraf_get_task` 可回查；
- 联合世界同一时刻只有一个控制源；
- 文档与图标注 `simulation=true`。

## 相关

- 设计稿：`docs/scene-provider-architecture.md`
- MCP 绑定：`docs/agentos-mcp-integration.md`、`docs/mcp-binding.md`
- 上层 SDK：`docs/upper-sdk-guide.md`、`docs/provider-sdk-guide.md`
