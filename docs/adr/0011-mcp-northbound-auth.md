# ADR-0011：MCP 北向鉴权与多调用方隔离

**状态**：已实现（2026-11-03，v0.4.0；mTLS 与 OAuth 待补）
**日期**：2026-11-03
**背景**：对外交付"上位机 SDK"后，合作方 harness / LLM agent 会以 streamable-http
连接边缘侧 IRAF 的任务级 MCP。原实现只支持 stdio（进程边界即身份），HTTP 形态无鉴权、
机器人开发模式是进程级全局状态、`iraf_get_task` 不校验执行归属 ⇒ 不能对外。

## 决策

1. **身份只来自受信 transport**（AGENTS 1.8）
   - HTTP：`Authorization: Bearer <opaque token>`；token 注册表只存 SHA-256，文件必须 600。
   - stdio：`IRAF_RUNTIME_SUBJECT` + `IRAF_MCP_SCOPES`（进程边界即信任边界）。
   - 工具参数不新增 `subject/caller/role/tenant/token`；请求体自报身份一律忽略。

2. **授权模型**
   - scope：`task.submit`、`task.read`（与 PolicyGateway 的 roles 对齐）。
   - 资源：token 可绑定 `resource_id` 列表；提交时校验 `dispatcher.resource_id` 在允许集合内。
   - tenant 仅用于审计/资源绑定，不是安全边界；跨租户隔离靠独立部署 + 资源绑定。

3. **执行归属**
   - `iraf_get_task` 用 `SqliteExecutionStore.get_for_subject(execution_id, subject)`。
   - 别人的 execution_id 一律 `NOT_FOUND`，不泄露存在性。

4. **多调用方状态隔离**
   - `iraf_robot_mode` 状态按 `subject` 分桶（`mode.state_for`），不再全局共享；
     stdio 仍只有一个 subject。

5. **HTTP 失败关闭**
   - 无 token 注册表且未显式 `IRAF_MCP_INSECURE=1` ⇒ 拒绝启动（退出码 2）。
   - `IRAF_MCP_INSECURE=1` 只允许绑定 loopback（127.0.0.1/localhost/::1）。
   - TLS：`IRAF_MCP_TLS_CERT/KEY`（可选 `IRAF_MCP_TLS_CA` + `REQUIRE_CLIENT`）。
   - `/healthz` 免鉴权且不泄露版本/身份。

6. **撤销与审计**
   - 按注册表 `(mtime,size,inode,ctime)` 热加载，撤销/过期无需重启。
   - `IRAF_MCP_AUDIT_LOG` 追加 JSONL：auth_ok/auth_failed/tool_call（含 execution_id/status），
     不含 token 明文或哈希。

7. **兼容性**
   - stdio 行为保持不变，但必须给 `IRAF_RUNTIME_SUBJECT`（缺则显式失败，不猜）。
   - MCP 工具清单仍为 5 个任务级工具，不新增技能级/Provider 级工具。

## 验证（2026-11-03 实测）

- 单测：`tests/unit/test_mcp_auth.py`、`test_mcp_server_layer.py::McpAuthWiringTest`、
  `test_execution_store_ownership.py` 等共 51 项通过。
- HTTP：无 token/失效 token ⇒ 401；有效 token ⇒ 5 tools、`mode=robot_dev`、`skills=15`。
- 撤销：热撤销后同一 token 立即 401。
- 归属：A 提交得到 `execution_id=287191a0-5cbc-5d69-a2c6-12526215887c`，B 的 token 读回 `NOT_FOUND`。
- 授权：只有 `task.read` 的 token 提交 ⇒ `IRAF-POLICY-DENIED`，无动作、无 execution_id。
- 失败关闭：缺注册表启动退出 2；`INSECURE=1` 绑 0.0.0.0 被拒。

## 未完成

- mTLS 客户端证书 CN 绑定：当前 uvicorn 版本不向 ASGI scope 暴露 peer cert；
  需网关终止 mTLS 或升级/自建 server 适配；在此之前对外生产环境必须由反向代理做 mTLS。
- 正式 SDK 客户端适配器（`iraf_sdk.mcp`）与 OAuth 2.1（MCP 规范）未实现。
- 限流（token bucket）未实现；当前依赖部署层网关。
