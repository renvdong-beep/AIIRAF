# 03 locomote 技能：定速直行与定速转弯（能力回填）

- 状态：TODO　　预估：3~5 个 tick　　归属：本窗口（四足线）
- 依据：ADR-0008、`.hermes/plans/2026-09-21_quadruped-locomotion-and-goto.md`

## 目标
把步态接到 Skill 层：`locomote`（vx/vy/wz + duration_ms）经 Runtime 执行，速度跟踪达标后回填能力。

## 前置
步骤 02 完成（原地踏步稳定）。

## 涉及文件（提交时只 add 这些路径）
- 新增：`skills/locomote/skill.yaml`（输入/输出 schema、错误码、超时、取消、恢复、安全等级；`requires: [locomote]`）
- 新增：`skills/locomote/locomote.input.json`、`locomote.output.json`
- 改：`src/iraf_skills/quadruped.py`（`LocomoteProvider`：速度斜坡 + 看门狗 + damped_hold 收尾）
- 改：`src/iraf_adapters/unitree/unitree_go2.py`（`locomote` 由显式拒绝改为真实实现；`stop` 支持 damped_hold）
- 改：`profiles/unitree_go2_mujoco.yaml`（**仅在验收通过后** capabilities 回填 `locomote`）
- 新增：`scripts/verify_go2_locomote.py`

## 步骤
1. 先声明 schema 与 Provider 骨架（超时/TTL 必须覆盖一次执行；参考 stop 的 TTL 事故）。
2. 实现速度指令 + 斜坡 + 看门狗（超时 → damped_hold）。
3. 直行验收：vx=0.2 m/s 定速 3 s；判据：平均速度误差 ≤ 10%、横向漂移 ≤ 0.10 m、航向偏差 ≤ 声明、无跌倒。
4. 转弯验收：wz=0.5 rad/s；判据：角速度误差 ≤ 10%、转弯半径误差 ≤ 声明；再验**原地转弯**（vx=0、wz 非零）与目标航向保持。
5. 负向：超速（>0.5 m/s）、缺租约、终态执行复用 fencing token、看门狗超时未触发 damped_hold —— 各自被拒或按声明行为收敛。
6. 能力回填只在 3/4 全绿后执行，并记录回填前后的 Profile digest。

## 验收（数字全部来自声明；未达标即 FAILED，不放宽）
```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_locomote.py --config config/go2_loopback.yaml; echo exit=$?   # 期望 0
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml | grep -A3 capabilities
PYTHONPATH=src /usr/bin/python3 scripts/scenario.py interact --scene scenes/handoff_lab --robot unitree_go2 --commands-from /tmp/loco.txt --display none
```
交互入口对 `locomote` 由拒绝变为执行（同一入口、同一门禁）。

## 证据落盘
`build/acceptance/go2-locomote/report.json`

## 提交信息
`feat: 新增四足 locomote 技能（定速直行/转弯）并回填能力`

## 失败 / 阻塞处理
速度跟踪若系统性偏差（如 0.2 走成 0.13）：先量清是斜坡还是步态增益，再改声明；不得改判据阈值。
