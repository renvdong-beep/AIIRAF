# 12 vendor 锁定 unitree_mujoco

- 状态：TODO　　预估：2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §1（实测 HEAD `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`）；ADR-0004 验证门禁 1

## 目标
把宇树仿真资产按 commit 锁定进仓库只读目录，附许可证 BOM 与哈希。

## 前置
步骤 11 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`vendor/unitree_go2/source-lock.json`
- 新增：`vendor/unitree_go2/LICENSE-BOM.md`
- 新增：读-only 资产 `vendor/unitree_go2/unitree_robots/go2/**`（按需最小集）
- 新增：`scripts/verify_vendor_lock.py`、`tests/unit/test_vendor_lock.py`

## 步骤
1. 从已探测的远端 clone（`build/u_mj_probe` 或重新浅克隆）取出 go2 目录与所需 mesh；**只取最小集**（go2 本体 + scene），不整仓拷贝。
2. 写 `source-lock.json`：仓库 URL、commit、路径、每文件 SHA-256、抓取时间、许可证标识。
3. `LICENSE-BOM.md`：逐项列出仓库 license 与资产来源（BSD-3-Clause 等），标注「逐项审查未完成即不得对外分发资产」。
4. 资产目录只读（`chmod -R a-w`）；`verify_vendor_lock.py` 复算哈希并断言未被本地修改。
5. 负向单测：改一个字节 → 校验失败。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/verify_vendor_lock.py --lock vendor/unitree_go2/source-lock.json; echo "exit=$?"  # 期望 exit=0
PYTHONPATH=src python3 -m unittest tests.unit.test_vendor_lock -v   # 期望 全部通过
```
资产哈希数 ≥ 20；`source-lock.json` 的 commit 必须等于 `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`。

## 证据落盘
`build/iraf-24h/12/vendor-lock.txt`

## 提交信息
`chore: 锁定宇树 Go2 仿真资产并登记许可证 BOM`

## 失败 / 阻塞处理
若 mesh 体积过大（>50MB）先只锁 MJCF 并在 note 写明「mesh 待定分发策略」，不得静默省略。
