# 机器人开发模式：把"进入=只读盘点，不许冒烟提交"做成机器可查的规则

日期：2026-10-22　方案：上一轮意见里的 B（改我们自己的返回值/提示面）

## 问题（实机现象）

dashboard /chat 里用户说"开始机器人开发模式"，平台自己设计了两段：
第 1 段只读自检（成功），第 2 段"冒烟提交"（被用户拒绝，denied，零 execution_id）。

根因不是模型乱来，是我们给的返回里缺三样：
1. `iraf_robot_mode` 的 `examples.robot_task` 长得像验收断言（`expected.status=SUCCEEDED`），
   模型自然拿去跑一遍；
2. 返回里没有 `next_steps`（`iraf_submit_task` 有，`iraf_robot_mode` 没有），模型只好自己设计下一步；
3. `HINT_ENTER` 只说"动作类请求都应交 submit"，没说"进入时先盘点、不许拿真任务探路"。

代价：MuJoCo 场景有状态，一次 pick_object 会把 box_01 搬走，之后必现
`distance≈0.374435m tolerance=0.005`，只能重启 Provider 复位 —— 冒烟吃掉的是本次唯一干净场景。

## 口径

- 进入模式 = **纯只读盘点**：`mode/skills/intents/profile/safety_policy/provider_endpoint/simulation`
  + 一句"现成能力"报告；**不提交任何任务**。
- 链路验收属于装机/开发阶段（`scripts/probe_mcp_client.py`），不属于对话。
- 平台侧行为不能改（不去动 AgentOS 容器/框架），只能靠工具面与返回值收口。

## 改动清单

1. `src/iraf_adapters/mcp/hints.py`
   - `WORKED_EXAMPLE` 增加 `purpose`，notes 增"不要为了验证链路而执行本示例"；
   - 新增 `ENTER_NEXT_STEPS` / `enter_next_steps()`：固定四步（盘点报清单 → 等真实需求 →
     提交 → `iraf_get_task` 核验），并写明场景有状态、链路验收用装机探针。
2. `src/iraf_adapters/mcp/mode.py`
   - `HINT_ENTER` 增"进入后先只读盘点并报告，不要提交任何任务"。
3. `src/iraf_adapters/mcp/server.py`
   - `iraf_robot_mode` docstring 增"不得为验证链路提交任务"；
   - 返回里补 `next_steps=enter_next_steps()`。
4. 测试
   - `tests/unit/test_mcp_hints.py`：`enter_next_steps()` 首步是盘点、含"不要为了验证链路"、
     含 `iraf_get_task`、含场景有状态；
   - `tests/unit/test_mcp_server_layer.py`：AST 检查 `iraf_robot_mode` 的 payload 里写了 `next_steps`。
5. 文档 `docs/mcp-binding.md` §8：新增「8.5 进入模式不含冒烟提交」，§8.1 表格补 `next_steps`。
6. 版本 `0.3.9 → 0.3.10`（返回契约有增量字段）。

## 验收

```bash
PYTHONPATH=src python3 -m pytest tests/unit/test_mcp_hints.py tests/unit/test_mcp_server_layer.py tests/unit/test_robot_mode.py -q
python3 scripts/probe_mcp_robot_mode.py enter    # 板/本地：next_steps 出现且无 execution_id
```

判绿：enter 返回里 `next_steps` 四步齐全、不产生 `execution_id`、文档与代码一致。
口径：simulation=true，不构成真机能力证据。
