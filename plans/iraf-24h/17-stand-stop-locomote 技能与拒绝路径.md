# 17 stand/stop/locomote 技能与拒绝路径

- 状态：TODO　　预估：2~3 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §4；`AGENTS.md` 2.8（成功与拒绝路径都要测）

## 目标
让四足能力以标准 Skill 形式走 `TaskFlow -> SkillRuntime -> Policy -> Provider`，成功与拒绝都有证据。

## 前置
步骤 16 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`skills/stand/`、`skills/stop/`、`skills/locomote/`（`skill.yaml` + 输入/输出 schema + 最小示例）
- 改：`profiles/unitree_go2_mujoco.yaml`（声明这三个能力，**仅在验收通过后**）
- 改：`profiles/safety/` 下新增或扩展仿真安全策略（速度上限、工作空间限制）
- 新增：`scripts/verify_quadruped_skills.py`、`tests/unit/test_quadruped_skill_contracts.py`

## 步骤
1. 三个 Skill 的 schema：输入输出、错误码、超时、取消、恢复、安全等级、版本（`AGENTS.md` 3 节）。
2. 通过 `SkillRuntime` 执行：成功路径（站立→行走→停止）产出 `build/acceptance/go2-skills/report.json`。
3. 拒绝路径逐条：越界速度、过期状态、无权限上下文、能力未声明、截止时间已过 → 每条都要有执行记录与错误码。
4. 仿真声明：所有证据带 `simulation: true`，不得表述为真机能力。
5. 只有全部通过后才把能力写进 profile；先写声明后补实现属于违规。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/verify_quadruped_skills.py --config config/go2_loopback.yaml; echo "exit=$?"   # 期望 exit=0
python3 -c "import json;d=json.load(open('build/acceptance/go2-skills/report.json'));print(d['passed'], sum(1 for c in d['cases'] if c['status']=='SUCCEEDED'), sum(1 for c in d['cases'] if c['status']=='FAILED'))"
PYTHONPATH=src python3 -m unittest tests.unit.test_quadruped_skill_contracts -v
```
拒绝用例数 ≥ 5 且全部为预期拒绝（错误码与原因非空）。

## 证据落盘
`build/iraf-24h/17/skills.txt`

## 提交信息
`feat: 新增四足 stand/stop/locomote 技能与拒绝路径验收`

## 失败 / 阻塞处理
若 Policy 拒绝路径缺少可复用夹具，先补测试夹具（属本步骤范围），不要绕过 Policy 直接调 Provider。
