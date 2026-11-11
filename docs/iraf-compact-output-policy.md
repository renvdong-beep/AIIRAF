# IRAF 工具输出的"紧凑模式"（本地小模型适配）

> 生效日期：2026-11-11 · 触发原因：板卡改用本地 Qwen3.5-9B（w4a8，2 核）后，chat 里等待时间过长。
> 本文记录：**为什么改、改了什么、怎么恢复**。改动只影响工具返回的文本长度，**不影响任何执行与判据**。

## 1. 为什么改（实测数据）

| 项 | 数值 |
|---|---|
| 本地模型生成速度 | **13.7 tok/s**（200 token / 14.6s） |
| 本地模型预填充 | 12,458 token 输入 → **11.1s** |
| `iraf_submit_task` 原返回体 | 9,761 字符 ≈ **3,253 tokens** |
| `iraf_enter_robot_mode` 原返回体 | 4,350 字符 ≈ **1,450 tokens** |

结论：执行本身没问题（ur5 抓取 ~57–60s），但模型"读工具结果 + 写回复"每轮要几十秒，
一轮对话有 2–3 次工具往返 ⇒ 用户要等几分钟才看到结果。

## 2. 改了什么

三个工具默认返回**紧凑摘要**（证据不删，只不在每轮塞进模型上下文）：

| 工具 | 默认（紧凑） | `detail=true` |
|---|---|---|
| `iraf_submit_task` | `status / execution_id / resolved_skill / metrics / summary / hint` | 全量（含 evidence、resolved、next_steps） |
| `iraf_get_task` | 同上 | 全量执行事实 |
| `iraf_enter_robot_mode` / `iraf_robot_mode` | 剔除 `skill_triggers / intents / routing_policy / examples`，`next_steps` 只留 2 条，附 `reply_hint`（≤3 行汇报） | 全量技能清单 |

实测收缩：submit **3,253 → 89 tokens（-97%）**；enter mode **1,450 → 393 tokens（-73%）**。

## 3. 怎么恢复（三种方式，按粒度从小到大）

### ① 单次调用要全量（不改任何配置）

```json
{"name": "iraf_submit_task", "arguments": {"text": "...", "scene_id": "ur5-pick", "detail": true}}
{"name": "iraf_get_task",   "arguments": {"execution_id": "...", "detail": true}}
{"name": "iraf_enter_robot_mode", "arguments": {"detail": true}}
```

### ② 全局恢复全量（最常用，换更快/更大的模型后就这么做）

在板卡装配单 `/etc/iraf/edge-intel.env` 增加一行，然后重启 MCP：

```bash
echo 'IRAF_MCP_VERBOSE=1' >> /etc/iraf/edge-intel.env
systemctl restart iraf-mcp-http
```

- `IRAF_MCP_VERBOSE=1/true/yes/on` ⇒ 所有工具默认返回**全量**；
- 删掉该行 + 重启 ⇒ 回到紧凑模式。**不需要改代码。**

### ③ 代码级回退（彻底移除紧凑逻辑）

```bash
git revert c584ff8 10c3ac7 66755b8   # 或在目标分支 cherry-pick 反向补丁
```

> 回退后请重跑单测：`pytest tests/unit/test_mcp_scene_mode.py`。

## 4. 什么时候该恢复全量

- 换成更大/更快的模型（例如 x86 侧的 deepseek-flash、或更强的板端模型）；
- 需要把完整 evidence 一并给外部审计/报表系统；
- 排查疑难失败时需要模型直接"看到"完整字段（也可以先用 `detail=true` 单次取，不必全局恢复）。

## 5. 证据没有丢

紧凑模式只改**返回给模型的文本**：事件库（`/var/lib/iraf/iraf-runtime.db` 的 `executions`）、
`runs` 表、`report.json` 仍是全量。任何时候都能用 `detail=true` 或直接读库取回。

## 6. 变更记录

| 提交 | 内容 |
|---|---|
| `66755b8` | `iraf_submit_task` / `iraf_get_task` 加 `detail` 参数，默认紧凑（-97% token） |
| `10c3ac7` | `iraf_enter_robot_mode` / `iraf_robot_mode` 默认紧凑（-73% token）+ `reply_hint` |
| `c584ff8` | 单测随 `detail` 透传更新 |
| 本次 | 新增 `IRAF_MCP_VERBOSE` 全局开关（恢复全量无需改代码）+ 本文件 |

## 7. 相关但不属于本策略的延迟

- **意图解析**：IRAF 的 `IRAF_AGENTOS_MODEL_URL` 仍指向 `192.168.100.89:3000`（单次约 4.2s）；
  若要全本地，可改到板端 `127.0.0.1:17701/v1`，但 9B 小模型的意图 JSON 稳定性需回归验证。
- **模型生成本身**：13.7 tok/s 是当前硬件+模型的上限，紧凑模式只是减少"要读的字"，不改变"写得慢"。