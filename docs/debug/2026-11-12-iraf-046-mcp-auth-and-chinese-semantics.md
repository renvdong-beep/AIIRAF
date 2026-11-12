# 2026-11-12 · 0.4.6 收口：AgentOS 授权持久化 + MCP 工具面 + 中文语义

- **版本**：0.4.6（`pyproject.toml`），本文记录的是 v0.4.6 发布后的收口补丁
- **提交**：`6066053`（工具面）、`bd3f294`（中文语义）、本文档提交（文档 + 产物）
- **涉及主机**：板卡 `10.203.247.147`（AgentOS + IRAF MCP）、x86 `10.203.247.121`（构建/仓库）
- **产物**：
  - `iraf_sdk-0.4.6-py3-none-any.whl` sha256 `6d25926bd5850b55e0ea6ca8f1abea4b522cc5a1cd2d14c3d75a1032e8febe53`
  - `iraf-runtime-0.4.6-universal.tar.gz` sha256 `b671e5ab5ca518d9cfbb07c69996d1c84c9a8b9480836859d8e42d9078a7daef`
  - `manifest.json` 自校验通过（361 个逐文件条目）

---

## 1. AgentOS 更新后 MCP 401（Authorization 丢失）

**症状**：AgentOS 页面/CLI 里 `iraf-task` 仍显示 enabled，但 `mcp test` 报 401；
`~/.intewell/config.yaml` 的 `mcp_servers.iraf-task` 只剩 `url`，`headers` 消失。

**根因**：前一版把 `Authorization: Bearer <token>` **明文**写进 `config.yaml`。
AgentOS 更新/重置（2026-11-12 10:47、13:43 两次实测）会按自己的字段重建配置，明文 headers 被丢弃；
IRAF 侧缺 Bearer 即 `401`（fail-closed，行为正确）。

**修复**（按 AgentOS 官方 `_save_bearer_auth_token` 口径）：

1. token 只写 `~/.intewell/.env`（600）：`MCP_IRAF_TASK_API_KEY=<token>`
2. `config.yaml` 只存模板：`Authorization: Bearer ${MCP_IRAF_TASK_API_KEY}`
3. 开机自愈 drop-in（防止更新再次丢）：

```ini
# /etc/systemd/system/agentos-dashboard.service.d/10-iraf-mcp-bind.conf
[Service]
ExecStartPre=-/usr/bin/python3 /root/bind_agentos_durable.py
```

脚本：`/root/bind_agentos_durable.py`（幂等；`--check` 只读）。
支持 `config.yaml` 被整体重置（从空配置起步，只补 `mcp_servers`）。

**验证**：

```text
[check] env=OK config=OK config_exists=True token_in_config=False
iraf-task   ✓ enabled
✓ Connected (103ms)   ✓ Tools discovered: 12
agent.log: MCP: registered 16 tool(s) from 1 server(s)
```

---

## 2. MCP 工具面泄漏内部实现 `_submit_task_impl`

**症状**：`tools/list` 返回 13 个工具，其中 `_submit_task_impl` 是内部实现（下划线开头）。

**影响**：模型可直接调用它，绕过 `iraf_submit_task` 的 plan-fallback 与统一审计。

**修复**：去掉该函数的 `@app.tool()`（内部调用不变）；新增
`tests/unit/test_mcp_tool_surface.py` 断言「对外注册的工具必须 `iraf_*` 前缀」。

**验证**：`tools/list` → 12 个，全部 `iraf_*`；`iraf_scene_status` 正常返回。

---

## 3. 中文语义「盒子 / ur」贯通

**症状**：用户口述「piper抓取盒子」「ur抓取盒子」，但

- 工具示例是英文 `抓取 box_01`，模型对中文目标词没把握；
- `ur`/`ur5e` 不在场景别名里，`ur抓取盒子` 会落回默认 `piper-pick`；
- 计划/回放分支另写了一份 `piper/ur5` 关键词表，`ur抓取盒子` 直接报
  `IRAF-PLAN-SCENE-UNKNOWN`；
- 成功摘要里出现 `box_01`，与用户口述不一致。

**修复**：

| 文件 | 变更 |
|---|---|
| `skills/pick_object/skill.yaml` | `intentExamples` 改中文：`piper抓取盒子` / `ur抓取盒子` / `ur5抓取盒子` / `抓取盒子` |
| `skills/pick_object/pick_object.input.json` | `target_id.description` 声明「盒子/方块/box → 场景默认载荷 `box_01`」 |
| `skills/visual_pick/visual_pick.input.json` | 同上（简化版） |
| `config/scenes/piper-pick.yaml` | `aliases: [piper, 派珀]` |
| `config/scenes/ur5-pick.yaml` | `aliases: [ur, ur5, ur5e, 优傲]` |
| `config/scenes/joint-handoff.yaml` | `aliases: [joint, 联合, 交接, handoff]` |
| `src/iraf_core/scene_registry.py` | `infer()` 歧义规则改为「命中词最长优先」（`ur5e` 不再被 joint 的 `ur5` 顶掉） |
| `src/iraf_adapters/mcp/server.py` | 计划/回放分支改用 `SceneRegistry.infer`；摘要中文化：`piper抓取盒子完成：…`、`ur抓取盒子完成：…`；`target_id` 字段仍是规范 ASCII id |
| `src/iraf_adapters/mcp/hints.py` | 工作示例改 `piper 抓取盒子` |

**验证（板卡现网）**：

```text
piper抓取盒子 → {"status":"SUCCEEDED","summary":"piper抓取盒子完成：抬升 83.3 mm，双侧接触，姿态偏角 21.56°"}
ur抓取盒子    → {"status":"SUCCEEDED","summary":"ur抓取盒子完成：抬升 76.3 mm，双侧接触，姿态偏角 2.21°"}
piper抓取box  → 同上（兼容旧说法）
infer: piper抓取盒子→piper-pick；ur/ur5/ur5e→ur5-pick；联合世界交接→joint-handoff；今天天气→''
```

新增回归：`tests/unit/test_scene_inference.py`（中文/别名用例）、
`tests/unit/test_mcp_tool_surface.py`。

---

## 4. 本次收口后的 0.4.6 产物

```text
iraf_sdk-0.4.6-py3-none-any.whl            6d25926bd5850b55e0ea6ca8f1abea4b522cc5a1cd2d14c3d75a1032e8febe53
iraf-runtime-0.4.6-universal.tar.gz        b671e5ab5ca518d9cfbb07c69996d1c84c9a8b9480836859d8e42d9078a7daef
manifest.json                              verify 通过（退出码 0）
```

重打命令：

```bash
cd /home/coretek/AIIRAF
bash deploy/sdk/pin_wheelhouse.sh aarch64-manylinux_2_28-cp311   # 钉版依赖（MCP 闭包）
bash deploy/sdk/build_sdk.sh --target aarch64-manylinux_2_28-cp311
bash deploy/sdk/build_sdk.sh --verify
```

---

## 5. 部署 / 回滚

**部署（板卡）**：

```bash
# 1) 让位旧版本与单元（受控重装）
mv /opt/iraf-sdk/0.4.6 /root/iraf-0.4.6-prev
for u in /etc/systemd/system/iraf-*.service; do mv "$u" /root/units-prev/; done
systemctl daemon-reload
# 2) 安装（env 会与旧装配单合并，站点键保留）
bash deploy/sdk/install-iraf.sh --profile edge-intel \
  --sdk-wheel /root/iraf-0.4.6/iraf_sdk-0.4.6-py3-none-any.whl \
  --runtime-bundle /root/iraf-0.4.6/iraf-runtime-0.4.6-universal.tar.gz \
  --wheelhouse /root/wheelhouse3/wheelhouse/aarch64-manylinux_2_28-cp311 \
  --secrets-file /root/runtime.secrets.copy --allow-unverified
# 3) AgentOS 绑定（drop-in 已自动重放；手动等价命令）
python3 /root/bind_agentos_durable.py && systemctl restart agentos-dashboard
```

**回滚**：`mv /root/iraf-0.4.6-prev /opt/iraf-sdk/0.4.6` + 恢复 `/root/units-prev/*.service`
+ `systemctl daemon-reload`（env 与 `/etc/iraf/scenes|providers` 不被安装器修改）。

**AgentOS 侧回滚**：`jq`/`yq` 删除 `mcp_servers.iraf-task`，或 `intewell-agent-cli mcp remove iraf-task`。

---

## 6. 已知遗留（本次未处理）

1. 板卡 **PostgreSQL 未安装**：`AGENTOS_STORE_BACKEND=postgres` 导致 worker registry 每 3 s 超时、
   退回 30 s 缓存（`journalctl`/`agent.log` 可见）。需要装 `postgresql-server` 或切本地 store。
2. **端侧模型 Qwen3.5-9B** 单轮 30–59 s 无输出（NPU 2 core），是 chat 慢的主因；
   与 IRAF MCP 无关（MCP 工具本身 44 ms）。
3. 板卡仍处演练/回放开关：`IRAF_PLAN_ONLY=1`、`IRAF_REPLAY_EVIDENCE=1`（提交返回录制摘要）。
4. `iraf-runtime-grpc` 默认不自启（`autostart: false`），需要南向 50051 时手工
   `systemctl enable --now iraf-runtime-grpc`。
