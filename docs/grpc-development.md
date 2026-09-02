# IRAF canonical gRPC 开发适配器

本适配器位于 `adapters/grpc/runtime_grpc.py`，只负责 Protobuf 编解码、Bearer 身份映射和结果反馈；任务策略、Skill 选择、租约和 Provider 调用仍由 `src/iraf_core/runtime.py` 完成。

## 开发启动

在 Ubuntu 仿真环境中准备 `IRAF_PROFILE`、`IRAF_SAFETY_POLICY`、`IRAF_SKILL_ROOT`、`IRAF_BACKEND_ENTRYPOINT`、`IRAF_BACKEND_CONFIG` 和 `IRAF_EVENT_STORE`，并设置：

```bash
export IRAF_GRPC_DEVELOPMENT=true
export IRAF_GRPC_HOST=127.0.0.1
export IRAF_GRPC_PORT=50051
export IRAF_GRPC_TOKEN='由部署环境注入'
python tools/runtime_grpc_server.py
```

部署模板为 `deploy/iraf-runtime-grpc.service`。该单元默认只监听回环地址，不能直接作为局域网或生产服务。

## 接口

- `iraf.v1.SkillRuntimeService/Execute`：提交 canonical `ExecuteSkillRequest`，返回终态 `SkillFeedback`。
- `iraf.v1.SkillRuntimeService/GetExecution`：按 `execution_id` 查询 SQLite 中的终态快照。
- `Cancel` 当前未注册，调用会得到 `UNIMPLEMENTED`，避免伪造取消成功。
- 每个 Execute 必须提供 `request_id`、`idempotency_key`、`correlation_id`、Profile/SafetyPolicy 名称、版本和摘要，以及 deadline。

## 验证

```bash
PYTHONPATH=src:skills/common:adapters:adapters/mujoco:adapters/agentos:adapters/model:adapters/grpc:build/generated/python \
python -m unittest discover -s tests/unit -v
```

生产阶段还需要 mTLS、签名信任链、取消语义和多阶段反馈流；这些能力完成前，结果只代表仿真验证。
