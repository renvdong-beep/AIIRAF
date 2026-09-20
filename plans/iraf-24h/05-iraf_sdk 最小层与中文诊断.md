# 05 iraf_sdk 最小层与中文诊断

- 状态：TODO　　预估：1~2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §4（产物 1）；决策 1.A

## 目标
建立纯 Python SDK 层：错误码 → 中文可操作诊断，并复用既有公共契约，不开旁路。

## 前置
步骤 02 完成（矩阵声明就位）。

## 涉及文件（提交时只 add 这些路径）
- 新增：`src/iraf_sdk/__init__.py`、`src/iraf_sdk/errors.py`、`src/iraf_sdk/client.py`
- 改：`pyproject.toml`（把 `iraf_sdk*` 加入 `[tool.setuptools.packages.find] include`）
- 新增：`tests/unit/test_sdk_errors.py`

## 步骤
1. `errors.py`：把既有错误码（`IRAF-INPUT-INVALID` 等，见 `docs/iraf-idl-runtime-detailed-design.md` §错误码表）映射为中文诊断 + 下一步建议；未知错误码不得静默返回原文，需显式标注「未知错误码」。
2. `client.py`：薄封装既有 gRPC/HTTP 公共契约（提交任务、查询执行、查询事件）；**禁止**新增绕过 `TaskFlow -> SkillRuntime -> Policy` 的调用路径。
3. 单测：每个已知错误码都有中文诊断用例；缺字段/未知码为负向用例。
4. 确认 SDK 不 import `mujoco` 等重依赖（保持纯净、可跨架构）。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 -m unittest tests.unit.test_sdk_errors -v   # 期望 全部通过
PYTHONPATH=src python3 -c "import iraf_sdk, sys; print('ok'); assert 'mujoco' not in sys.modules"
```

## 证据落盘
`build/iraf-24h/05/unittest.txt`

## 提交信息
`feat: 新增 IRAF Python SDK 最小层与中文错误诊断`

## 失败 / 阻塞处理
若错误码表与实现不一致，以 `src/iraf_core` 实际常量与 IDL 文档为准，并把差异写入 `docs/debug/`。
