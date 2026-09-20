# 18 S2 脚本化场景入口 scenario.py

- 状态：TODO　　预估：2~3 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §3；决策 6.A

## 目标
把场景包跑起来：S2 脚本化场景按声明执行并出报告（先于 S1 交互）。

## 前置
步骤 11、17 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`scripts/scenario.py`
- 新增：`tests/unit/test_scenario_runner.py`
- 改：`scenes/handoff_lab/scenario.yaml`（补全可执行步骤与判据）

## 步骤
1. `list`：列出可用场景与声明能力（读 `scenes/*/scene.yaml`）。
2. `run --scene <dir> --scenario <name>`：按 `scenario.yaml` 逐步执行（复用 `SkillRuntime` 直连，走 Policy/Authority），每步记录状态、耗时、参数与错误码。
3. 报告：`build/acceptance/<scene>/<scenario>/report.json`，schema 与既有验收一致（含 `passed`、逐步证据、`simulation: true`）。
4. 失败注入场景先只支持两类（传感器不可用、能力未声明），其余留到后续。
5. 负向：场景声明引用不存在的 skill/能力 → 显式失败并给出中文原因。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/scenario.py list
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario stand_stop; echo "exit=$?"   # 期望 exit=0
PYTHONPATH=src python3 -m unittest tests.unit.test_scenario_runner -v
```
报告里每步都有 `status` 与耗时数字；`passed=true` 需所有步骤满足判据。

## 证据落盘
`build/iraf-24h/18/scenario.txt`

## 提交信息
`feat: 新增 S2 脚本化场景执行入口与验收报告`

## 失败 / 阻塞处理
若某步依赖尚未实现的能力（如 navigate），在 `scenario.yaml` 中标注 `pending` 并让 runner 显式跳过并记录原因，不得静默通过。
