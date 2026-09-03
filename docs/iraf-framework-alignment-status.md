# IRAF 框架对齐与实施状态

**日期**：2026-09-02  
**状态**：开发仿真基线，不代表真机或生产 AgentOS 放行

## 当前代码边界

- `api/proto/`：canonical Protobuf；生成代码只放在 `build/generated/`，不得手工修改。
- `src/iraf_core/`：平台无关的 Runtime、Policy、Registry、TaskFlow、租约、安全隔离和 Store。
- `src/iraf_adapters/`：AgentOS、HTTP/gRPC、模型、MuJoCo、ROS 2 和 Backend 适配器。
- `src/iraf_skills/`：可复用 Skill Provider；具体 Skill manifest 和 schema 位于 `skills/*/`。
- `src/iraf_tools/`：可安装 CLI；`tools/` 保留脚本兼容入口。
- `profiles/`：RobotProfile 与 SafetyPolicy；`deploy/`：环境和服务模板。
- `tests/`：契约、单元、集成和后续 HIL/fault 验证。

## 已完成

- canonical `src` 包层及旧顶层入口兼容包装。
- Runtime 统一负责 Policy、Registry、资源租约、幂等和事件记录。
- Skill Provider 由 manifest、JSON Schema 和 entrypoint 装配。
- Skill 版本按 SemVer 数值排序；资源租约带 monotonic TTL 和 fencing token。
- development HTTP systemd 模板限制为回环监听。
- `/v1/tasks -> move_joint -> common_motion_sim -> MuJoCo` 已由最小链路脚本验证成功；`/v1/intents` 已由 Qwen3 驱动验证成功；HTTP Runtime 由 `MujocoSimulationSupervisor` 持有唯一长期物理循环，健康接口已验证 `continuous_simulation: true`；framework-owned `render_piper_mujoco.py` 通过 Runtime/SkillRuntime/Backend/Supervisor 执行动作并在 Ubuntu tty/EGL 环境生成 PNG/GIF 画面。

## 尚未完成

- AgentOS 正式 TaskFlow、Skill Query、Tool/Skill Manager 私有接口契约测试。开发版 EventService.ListEvents 已提供，正式回放清单仍未完成。
- gRPC development server 已支持 accepted/terminal feedback 与 cooperative Cancel；生产 server、mTLS、断线续传和多进程租约仍未完成。
- 签名 Profile/Policy/manifest 的正式信任链与 mTLS。
- 已有基于 SQLite 的 SafetyEvent/QUARANTINED 门禁、跨进程状态读取、控制器 safe-state ack 和授权恢复接口；RTOS 回读、事件续传和正式 EventService 回放清单仍未完成。
- HIL/真机证据；当前所有动作结论仅限仿真。
