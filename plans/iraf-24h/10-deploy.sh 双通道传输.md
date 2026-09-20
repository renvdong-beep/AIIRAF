# 10 deploy.sh 双通道传输

- 状态：TODO　　预估：1~2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §5；`AGENTS.md` 2.7（凭据不入库）

## 目标
统一部署入口：ssh 直连与 media（隔离网人工拷贝）两种传输，均带 dry-run。

## 前置
步骤 08、09 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`deploy/sdk/deploy.sh`
- 新增：`docs/debug/2026-09-20-sdk-cross-arch.md`（本战役的跨架构调试记录，后续步骤继续追加）

## 步骤
1. `--transport ssh --target <user@host> --port <n>`：`scp` bundle + `sha256sum` 校验 → 远端执行 `install.sh` → 取回 `evidence.json`。
2. `--transport media --output <dir>`：产出 `bundle.tar.gz` + `sha256sum.txt` + `README-安装.md`（中文步骤，含校验与回滚）。
3. 凭据与主机地址只来自参数/环境变量；脚本内不得出现任何 IP 常量（`grep -nE "[0-9]{1,3}\.[0-9]{1,3}\."` 期望 0 命中）。
4. 目标不可达 → 退出 2 并给出中文原因（当前 `10.203.247.72:22` 实测不通，可作负向验收）。
5. 调试记录首版：写明本战役目标、实测约束（无 buildx、pypi 不可达、板卡不通）、已实现/未实现清单。

## 验收（必须可复跑，以数字为准）
```
bash deploy/sdk/deploy.sh --transport media --output build/iraf-24h/10; echo "exit=$?"    # 期望 exit=0
bash deploy/sdk/deploy.sh --transport ssh --target coretek@<板卡IP> --dry-run; echo "exit=$?"  # 目标不可达时期望 exit=2（板卡到位后替换为真实 IP，当前为 DEFERRED）
grep -cE "[0-9]{1,3}\.[0-9]{1,3}\." deploy/sdk/deploy.sh    # 期望 0
```

## 证据落盘
`build/iraf-24h/10/deploy.txt`

## 提交信息
`feat: 新增 SDK 部署入口与媒体通道`

## 失败 / 阻塞处理
SSH 真机部署在板卡到位前置 `DEFERRED`；媒体通道（`--transport media`）不受板卡影响，必须完整验收；不得把 dry-run 结果写成部署成功。
