# IRAF 24 小时受控调度

本仓库提供 `tools/orchestrate_cycle.py`，用于由本机发起验证调度。它只允许两类任务：

- `verify_development_simulation`：SSH 到 247.145，要求工作区干净，然后执行仓库内的 9 项开发仿真门禁。
- `edge_model_health`：访问 247.86 的 OpenAI-compatible `/v1/models`，只验证模型服务可达和返回模型列表。

调度器禁止执行任意远程代码修改、禁止提交密钥、禁止把边缘模型当作运动控制权限来源。代码实现必须在独立分支完成，并经门禁和人工审核后再合并。

## 配置

复制 `orchestrator/config.example.json` 到本机受控目录，设置：

```bash
export IRAF_RUNTIME_SSH_TARGET='coretek@10.203.247.145'
export IRAF_EDGE_MODELS_URL='https://10.203.247.86:9119/v1'
export IRAF_EDGE_TOKEN='由本机安全凭据注入'
```

Token 不要写入仓库、配置文件或调度日志。247.86 任务默认关闭，只有确认本机到边缘板卡的网络和凭据后才启用。

## 单轮验证

```bash
python tools/orchestrate_cycle.py \
  --config orchestrator/config.example.json \
  --once \
  --output build/orchestrator/manual-run
```

## 24 小时验证

```bash
python tools/orchestrate_cycle.py \
  --config /受控目录/orchestrator.json \
  --duration-hours 24 \
  --interval-seconds 3600 \
  --output build/orchestrator/24h-run
```

任一 Runtime 门禁失败、远端工作区 dirty 或 SSH 超时，当前周期立即停止；已生成的 manifest 和日志保留用于审计。当前调度器只做验证，不自动修改源码、不自动合并、不自动推送。

## 证据

每次运行生成：

```text
build/orchestrator/<run>/
  manifest.json
  manifest.sha256
```

manifest 记录调度器版本、commit、分支、dirty 状态、周期结果和失败原因。远端 247.145 的完整验收包仍保存在其 `build/acceptance/development-simulation/<cycle-id>/` 下。
