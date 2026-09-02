# Skill 注册、策略网关与 Runtime 鉴权回归记录

日期：2026-09-02

## 问题

旧版 Dispatcher 直接判断 `move_joint` 和 `stop`，新增 Skill 必须修改 Runtime；HTTP 边界也未把身份、截止时间、Profile 版本和幂等键作为强制契约。

## 修复

- 新增 `api/proto/iraf/v1/runtime.proto`，固定任务请求、结果和身份上下文字段。
- 新增 `SkillDefinition` 与 `SkillRegistry`，参数解析和调用方式由 Skill 注册项负责。
- 新增 `PolicyGateway`，在 Provider 调用前检查受信身份、角色、关联 ID、截止时间、Profile 版本和能力声明。
- execution_id 仅由服务端生成；HTTP Runtime 按 idempotency_key 去重。
- Bearer 身份由 transport 构造，不接受请求体中的身份声明。

## 验证

- 单元测试：成功、关节越界、能力缺失、请求过期、未注册 Skill，共 5 项通过。
- HTTP：无令牌返回 401；有效令牌任务成功；重复幂等键返回相同 execution_id。
- systemd：旧手工进程清理后，端口仅由 `iraf-runtime.service` 托管。
- E300 模型服务连通性本次检查未通过，未影响 Ubuntu 本地仿真链路，需单独排查板卡网络或服务状态。

## 回归命令

```bash
PYTHONPATH=src:skills/common:adapters/agentos python3 -m unittest -v tests/unit/test_dispatcher.py
protoc -I api/proto --descriptor_set_out=/tmp/iraf-runtime.pb api/proto/iraf/v1/runtime.proto
```