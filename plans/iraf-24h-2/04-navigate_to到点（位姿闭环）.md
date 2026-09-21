# 04 navigate_to：位姿闭环到点（正/负路径）

- 状态：TODO　　预估：3~4 个 tick　　归属：本窗口（四足线）
- 依据：ADR-0008、`.hermes/plans/2026-09-21_quadruped-locomotion-and-goto.md`

## 目标
提供到点能力：目标位姿 (x, y, yaw) + 容差，外环位姿闭环驱动内环速度，单写者、可取消、可看门狗。

## 前置
步骤 03 完成（locomote 速度通道可用）。

## 涉及文件（提交时只 add 这些路径）
- 新增：`skills/navigate_to/skill.yaml` + 输入/输出 schema
- 新增：`src/iraf_skills/navigation.py`（位姿外环：位置误差→速度指令；航向误差→wz；到点判据）
- 改：`config/go2_loopback.yaml`（`navigation` 段：外环频率、到位容差、末速门槛、超时）
- 改：`profiles/unitree_go2_mujoco.yaml`（验收通过后回填 `navigate_to`）
- 新增：`scripts/verify_go2_goto.py`、`tests/unit/test_navigation_contract.py`

## 步骤
1. 声明外环参数（频率、容差、末速门槛、超时），缺键失败。
2. 实现外环：位置/航向误差 → (vx, vy, wz)，钳制到安全策略上限；到达后 `damped_hold` 收尾并把末速压到声明门槛内。
3. 验收：起点到 +x 方向 1.0 m 目标点；判据：位置误差 ≤ 0.10 m、航向误差 ≤ 5°、末速 ≤ 0.05 m/s、期间安全事件 = 0。
4. 负向：目标越界（超出 `workspace_m`）、目标不可达（超时）、无租约调用、取消请求 → 各自终止并给出错误码，且不得留下持续运动。
5. 能力回填仅在全绿后写入 Profile。

## 验收（数字全部来自声明；未达标即 FAILED，不放宽）
```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_goto.py --config config/go2_loopback.yaml; echo exit=$?   # 期望 0
# 报告含：target_pose / final_pose / position_error_m / yaw_error_deg / final_speed_mps / safety_events
```
负向各自有独立用例与退出码；`simulation: true`。

## 证据落盘
`build/acceptance/go2-goto/report.json`

## 提交信息
`feat: 新增四足 navigate_to 到点能力（位姿闭环）`

## 失败 / 阻塞处理
若外环振荡：先降外环增益做参数扫描并留证；不得通过放宽容差来通过。
