# 02 SDK 产物矩阵与 BoardProfile 声明

- 状态：TODO　　预估：1~2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §6/§7；决策 2.B、3.A

## 目标
建立产物矩阵与两块板卡的声明文件，作为后续所有脚本的唯一事实来源。

## 前置
步骤 01 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`config/sdk/package_matrix.yaml`
- 新增：`profiles/boards/e300.yaml`
- 新增：`profiles/boards/firefly_rk3588.yaml`
- 新增：`config/sdk/package_matrix.schema.json`
- 新增：`tests/unit/test_board_profile_schema.py`

## 步骤
1. 按设计 §7 写 `package_matrix.yaml`：`targets[0]` 为 `aarch64-manylinux_2_28-cp310`；`index_url` 先写 **空字符串** 并在旁边注明「决策 2.B：内网私有源地址待使用者提供，未声明即预检失败」。
2. 写两份 BoardProfile：`arch: aarch64`，`os/python_tag/platform_tag/kernel/limits/adapters.npu` 等全部 `unverified`，`evidence.owner: pending`。
3. 写 schema 文件并在单测中用 `jsonschema` 校验两份 profile 与矩阵。
4. 负向用例：删掉 `target.platform_tag` → 校验必须失败；把 `status` 写成 `verified` 但 `evidence.acceptance_report: pending` → 必须失败（未验收不得声明已验证）。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 -m unittest tests.unit.test_board_profile_schema -v   # 期望 全部通过（含 2 个负向）
```
矩阵中不得出现硬编码的 pip 官方源地址；`grep -c unverified profiles/boards/e300.yaml` 期望 >= 6。

## 证据落盘
`build/iraf-24h/02/unittest.txt`

## 提交信息
`feat: 新增 SDK 产物矩阵与板卡 profile 声明`

## 失败 / 阻塞处理
若 `jsonschema` 版本导致 schema 写法差异，用最小可行 draft（`$schema: draft-07`）并在调试记录中写明。
