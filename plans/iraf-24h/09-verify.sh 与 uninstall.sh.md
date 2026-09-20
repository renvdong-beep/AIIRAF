# 09 verify.sh 与 uninstall.sh

- 状态：TODO　　预估：1~2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §5/§9

## 目标
提供目标端自检与可回滚卸载：校验和不符必须失败，卸载只动本 bundle。

## 前置
步骤 08 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`deploy/sdk/verify.sh`
- 新增：`deploy/sdk/uninstall.sh`
- 新增：`tests/unit/test_uninstall_scope.py`

## 步骤
1. `verify.sh`：复算 manifest 中所有 SHA-256 → `profile_check.py --board` → `/health` 探针（HTTP 与 gRPC 各一）→ 事件库写入检查 → 输出 `evidence.json`（含 `verified: true/false`、各步骤原始输出摘要、时间戳）。任何一项不符 → 退出 4/5，禁止打印「通过」。
2. `uninstall.sh`：停 systemd → 删除本 bundle 记录的文件（依据 manifest 文件清单）→ 保留数据与日志；发现清单外文件即拒绝（退出 1）。
3. 单测：构造临时目录，注入一个清单外文件 → 断言卸载拒绝。
4. 边界：`verify.sh` 在服务未启动时必须显式报告「服务未启动」，而不是当作通过。

## 验收（必须可复跑，以数字为准）
```
bash deploy/sdk/verify.sh --bundle build/sdk/iraf-board-e300-*.tar.gz --dry-run; echo "exit=$?"    # 期望 exit=0（dry-run 模式）
# 负向：篡改 bundle 内文件后
bash deploy/sdk/verify.sh --bundle <篡改后文件> --dry-run; echo "exit=$?"                          # 期望 exit=4
PYTHONPATH=src python3 -m unittest tests.unit.test_uninstall_scope -v                               # 期望 全部通过
```

## 证据落盘
`build/iraf-24h/09/verify-uninstall.txt`

## 提交信息
`feat: 新增目标端自检与受控卸载脚本`

## 失败 / 阻塞处理
`/health` 探针在开发机无目标服务 → 用 `--dry-run` 与本地临时 HTTP stub 验证解析逻辑；目标端部分置 `DEFERRED`（板卡不在），且 stub 只能证明解析逻辑，不得写成「健康检查通过」。
