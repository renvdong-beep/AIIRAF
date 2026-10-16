# 预检应识别"该步骤技能产不出这条判据"（判据作用域缺失）

状态：**待办**（起点遗留；`tests/unit/test_scenario_runner.py::test_dispatched_step_with_unevaluable_criteria_fails_preflight` 目前红）
记录：2026-09-28

## 1. 现象（实测）

把 `stand_stop` 的首步（`ss01 stand`）判据改成 `{"pose_tolerance_m": 0.005}` 后：

- **期望**（用例意图）：预检以 `EXIT_DECLARATION(2)` 拒绝，消息含"没有评测依据的判据 pose_tolerance_m"；
- **实际**：退出码 **5**（步骤跑完、判据在运行期判失败）。

## 2. 根因

`scripts/scenario.py: plan_steps` 里：

```python
unsupported = [key for key in record["criteria"] if key not in CRITERION_SPEC]
```

只检查**判据名是否在全局词表** `CRITERION_SPEC` 里；`pose_tolerance_m` 在词表里（它属于 pick/place）
⇒ `unsupported == []` ⇒ 认为可评测 ⇒ `check_evaluable_criteria` 不触发。

"这条判据对**这个技能**是否可评测"这一维度**完全没有声明**：`CRITERION_SPEC` 的第三个元素是
**中文散文**（如 "evidence.grasp_alignment.center_distance_m"），机器不可读 ⇒ 无法与技能输出对照。

## 3. 为什么不能立刻小改

- 只把 `unsupported` 改成"按 action 白名单"⇒ 需要新增一张 `action → 可用判据` 的表：又是一处
  与技能契约**重复**的事实来源（违反"单一来源"），且每加一个技能都要记得改它。
- 直接改测试期望 ⇒ 等于把已知缺陷说成设计 ⇒ 违反"不得把已知阻塞说成已解决"。

## 4. 候选设计（二选一，需架构裁定）

**甲案（推荐）：让判据自带机器可读的测量路径。**
把 `CRITERION_SPEC` 的条目从
`(measurement_key, operator, 中文说明)` 扩为
`(measurement_key, operator, 中文说明, evidence_path)`，
其中 `evidence_path` 形如 `("grasp_alignment", "center_distance_m")`（或 `("released",)`）。
预检规则：**该步骤技能的 output schema 必须声明这条路径**（`skills/<action>/<action>.output.json`
的 `properties.evidence.properties` 递归查找）⇒ 声明与技能输出来自**同一份事实**（技能自己的契约）。
优点：新技能只要写对 output.json 就自动获得正确的预检；缺点：要改 10 条 `CRITERION_SPEC` 与
`check_evaluable_criteria`，并给"路径不存在"补一条负向用例。

**乙案：技能清单里声明它产出哪些测量量**（`skill.yaml` 新增 `measurements: [...]`）。
与甲案相比多了一份手工清单、少一次 schema 递归；但仍需与 output.json 保持一致（可加一致性校验）。

## 5. 验收标准

1. `ss01 stand` + `pose_tolerance_m` ⇒ 预检 `EXIT_DECLARATION(2)`，消息含判据名与"没有评测依据"；
2. 现有全部场景（`stand_stop` / `nominal` / `fault_sensor_loss` / `fault_sensor_unavailable`）
   预检通过，`nominal --world joint` 仍 s01–s05 全绿；
3. `tests/unit/test_scenario_runner.py::…fails_preflight` 转绿；新增"技能 schema 里没有该路径"的负向用例；
4. 该用例的注释同步更新（说明判据作用域来自技能输出契约）。
