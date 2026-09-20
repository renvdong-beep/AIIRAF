# 03 profile_check 校验 BoardProfile

- 状态：TODO　　预估：1~2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §6；`AGENTS.md` 3 节（未验证不得当作可用）

## 目标
让既有金路径 `profile_check.py` 能校验板卡声明，并对 `unverified` 显式拒绝。

## 前置
步骤 02 完成。

## 涉及文件（提交时只 add 这些路径）
- 改：`scripts/profile_check.py`（新增 `--board` 分支，不动既有 `--baseline` 行为）
- 新增：`tests/unit/test_profile_check_board.py`

## 步骤
1. 加 `--board <yaml>`：读取 BoardProfile → 校验 schema → 检查 `status`、`target.arch`、`python_tag`、`platform_tag`、`limits`、`adapters` 是否仍有 `unverified/pending`。
2. 任一 `unverified` → 退出码 2，中文原因列到具体字段（如 `target.python_tag 未验证：请在板卡实测后回填`）。
3. 加 `--board --allow-unverified`：允许通过但输出 `verified: false` 的摘要 JSON，用于打包前的显式放行。
4. 负向单测：篡改字段 → 断言退出码 2 与错误文案包含字段名。
5. 回归：`--baseline config/piper_simulation_baseline.yaml` 与 `config/ur5_simulation_baseline.yaml` 的输出与改动前逐字一致。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/profile_check.py --board profiles/boards/e300.yaml; echo "exit=$?"     # 期望 exit=2
PYTHONPATH=src python3 scripts/profile_check.py --board profiles/boards/e300.yaml --allow-unverified; echo "exit=$?"  # 期望 exit=0
PYTHONPATH=src python3 -m unittest tests.unit.test_profile_check_board -v   # 期望 全部通过
```

## 证据落盘
`build/iraf-24h/03/profile-check.txt`

## 提交信息
`feat: profile_check 支持 BoardProfile 校验与 unverified 拒绝`

## 失败 / 阻塞处理
若既有 `--baseline` 路径与之耦合（共享出口码逻辑），改为分支同时保留原行为，并在 00-日志.md 记录改动点。
