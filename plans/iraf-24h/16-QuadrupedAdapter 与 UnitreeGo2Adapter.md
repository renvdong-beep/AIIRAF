# 16 QuadrupedAdapter 与 UnitreeGo2Adapter

- 状态：TODO　　预估：2~3 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §2；`AGENTS.md` 1.13 / 2.11

## 目标
把宇树差异收敛到 adapter 与 profile：能力契约平台无关，厂家细节不外泄。

## 前置
步骤 15 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`src/iraf_adapters/unitree/__init__.py`、`quadruped.py`（通用契约）、`unitree_go2.py`
- 新增：`tests/unit/test_quadruped_adapter.py`
- 改：`src/iraf_adapters/factory.py`（注册新 backend 入口，**不改**原有 one-way 契约语义）
- 改：`scripts/profile_check.py`（把新 backend 纳入 declared ⊆ implemented 校验）

## 步骤
1. 通用契约只暴露能力与标准反馈：`stand`、`stop`、`locomote`（速度指令）、`read_state`、`emergency_stop` 确认；**不**暴露 `FR_hip` 等厂家关节名、DDS domain、CRC 细节。
2. `UnitreeGo2Adapter`：关节名/执行器名映射集中在一处（参考 UR5e 的 joint↔actuator 解析教训）；力矩限幅从模型读取。
3. 控制权：与既有 `ControlAuthorityManager` 对接，同一执行器同一时刻只有一个控制源；旧 token 或终态执行的控制请求必须被拒绝（负向用例）。
4. 单测：能力契约（declared ⊆ implemented）、未知关节名拒绝、越界指令拒绝、无租约拒绝。
5. `profile_check` 对 `profiles/unitree_go2_mujoco.yaml` 全绿。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/profile_check.py --baseline config/ur5_simulation_baseline.yaml; echo "exit=$?"   # 期望 exit=0（回归）
PYTHONPATH=src python3 -m unittest tests.unit.test_quadruped_adapter -v    # 期望 全部通过（含 3 负向）
```

## 证据落盘
`build/iraf-24h/16/adapter.txt`

## 提交信息
`feat: 新增四足通用契约与 Unitree Go2 适配器`

## 失败 / 阻塞处理
若 factory 既有契约校验对多机型后端报错，先确认是否 declared ⊆ implemented 的单向语义，不得为了通过而放宽校验。
