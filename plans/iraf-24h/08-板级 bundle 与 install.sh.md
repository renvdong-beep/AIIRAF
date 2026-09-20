# 08 板级 bundle 与 install.sh

- 状态：TODO　　预估：2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §4/§5/§6；决策 3.A

## 目标
产出可在目标端执行的安装包与安装脚本：预检→离线安装→profile-check→启动。

## 前置
步骤 03、06 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`deploy/sdk/package_board_bundle.sh`
- 新增：`deploy/sdk/install.sh`
- 新增：`deploy/sdk/iraf-sdk-board.service`
- 新增：`tests/unit/test_install_prechecks.py`

## 步骤
1. `package_board_bundle.sh`：组装板级 bundle（BoardProfile + runtime bundle + wheelhouse + deploy/sdk/*.sh + systemd 单元 + env 模板）。
2. 未加 `--allow-unverified` 且 profile 含 `unverified` → 退出 2（这是关键 fail-closed 点）。
3. `install.sh`（目标端执行）：预检（`uname -m`、Python 版本与标签、glibc/manylinux 兼容、磁盘/RAM、systemd 可用、目标端已有包冲突）→ 版本化安装目录 + 符号链接切换 → 离线安装 wheelhouse（`--no-index --find-links`）→ 写 env → 调 `profile_check.py --board` → 启动 systemd 单元 → 调 `verify.sh`。
4. 预检失败必须退出 2 且不写系统目录；安装失败退出 3 并回滚符号链接。
5. `install.sh --dry-run` 在开发机上必须能在临时目录完成全流程演练（不写 `/etc`、不启动服务）。

## 验收（必须可复跑，以数字为准）
```
bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml; echo "exit=$?"                  # 期望 exit=2
bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml --allow-unverified; echo "exit=$?" # 期望 exit=0
bash deploy/sdk/install.sh --bundle build/sdk/iraf-board-e300-*.tar.gz --dry-run --root build/iraf-24h/08/stage; echo "exit=$?"  # 期望 exit=0
PYTHONPATH=src python3 -m unittest tests.unit.test_install_prechecks -v    # 期望 全部通过
```

## 证据落盘
`build/iraf-24h/08/bundle-install.txt`

## 提交信息
`feat: 新增板级 bundle 组装与目标端安装脚本`

## 失败 / 阻塞处理
目标端真实安装需板卡在场 → 该部分置 `DEFERRED`（板卡不在），本步按 `--dry-run` + 临时 `--root` 目录完成 x86 侧全流程验收；不得把 dry-run 结果表述为「已安装」。
