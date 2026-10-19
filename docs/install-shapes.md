# IRAF 安装档位（三档 + 同机变体）

本文件只描述**怎么装**与**怎么判绿**。路径、端口、单元、组件清单的唯一事实来源是
`config/install/install_profiles.yaml`；构建目标来自 `config/sdk/package_matrix.yaml`；
板卡实测事实来自 `profiles/boards/<board>.yaml`。本文不重复这些值（AGENTS 5.3）。

## 四个档位

| 档位 | 装什么 | 目标 | 对应角色 |
|---|---|---|---|
| `x86-all` | 仿真 + 运行同机 | 本机（Ubuntu 22.04 / x86_64） | `single-ubuntu2204` |
| `intel` | 板卡只跑 TaskFlow/Skill/Policy | 一块 aarch64 板 | `edge-intel` |
| `motion-split` | 智控域一台 + 运控域另一台 | 两台 aarch64 机器 | `edge-intel` + `edge-motion` |
| `motion-cohost` | 两域同一块板 | 一块 aarch64 板 | `edge-intel` + `edge-motion`（路径已隔离） |

`motion-split` 与 `motion-cohost` 的差别只有落点：前者两域各在自己的机器上（各自默认
`/opt/iraf-sdk`），后者两域同机（运控域自动落到 `/opt/iraf-sdk-motion`、`/etc/iraf-motion`、
`/var/log|lib/iraf-motion`）。这不是特例代码，而是 `edge-motion` 角色在声明里的角色级路径覆盖。

## 一条命令（默认只演练，真装加 --apply）

```bash
# 1) x86 本机全装（仿真 + 运行）
bash scripts/install_shape.sh --shape x86-all

# 2) 板卡只装智控域
export SSHPASS='<板卡密码>'          # 只从环境变量传，不进命令行
bash scripts/install_shape.sh --shape intel \
  --host <板卡IP> --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052

# 3) 两域分机
bash scripts/install_shape.sh --shape motion-split \
  --host <智控域IP> --motion-host <运控域IP> \
  --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052

# 4) 两域同机
bash scripts/install_shape.sh --shape motion-cohost \
  --host <板卡IP> --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052
```

加 `--apply` 才真装。脚本自身不写任何路径/端口：版本从 `pyproject.toml`、构建目标按
BoardProfile 推断、角色路径读声明。

## 部署期取值（为什么不是 4 个 --env-set）

`--values <yaml>` 与 `--env-set KEY=VALUE` 二选一或合并，取值文件示例见
`config/install/deploy_values/*.example.yaml`。

- 与地址无关的取值（机型 Profile / 安全策略 / 调用方标识）放文件里，可复用；
- **地址类取值不入库**（IP 实测变过），一律部署时 `--env-set` 给；
- 文件里残留 `<...>` 占位符会被拒绝（`--values 里仍是占位符：KEY`）；
- 缺必需占位符时目标端报 **字段名 + 模板行号**，例如
  `装配单模板缺部署期取值：PROVIDER_ENDPOINT（模板第 8,29 行）`。

## 判绿口径（三档一致）

1. **装得上**：安装器退出码 0；`<prefix>/<version>` + `current` 链接 + env 文件 + 单元 + 安装记录齐备；
   后置自检用同一解释器 import 通过（输出形如 `['iraf_sdk-ok+stubs']`）。
2. **能干净卸载**：先 `uninstall-iraf.sh --dry-run` 看将删清单，再真卸，再跑**独立**的
   `verify_residue.sh`，六路（文件/空目录/单元/env/端口/进程）全 0。
3. **跨机调用**（`intel` / `motion-*` 档）：x86 侧起仿真 Capability Provider，板上发起任务跑到 `SUCCEEDED`。

注意两个"按设计为红"：安装**故意不启动服务** ⇒ 单元 `disabled`、进程不存在是正常的；
`verify_residue.sh` 在**装完之后**必然报"不干净"（已装即有内容），那是核对工具在正常工作。

v1 口径（AGENTS §0.1）：以上结论一律 `simulation=true`，**不构成真机能力证据**。

## 已验证 / 未验证（本文件不得夸大）

| 内容 | 状态 |
|---|---|
| `install_profile_check.py` 全量校验 | 已跑：95 项检查 0 未通过 |
| `pytest tests/unit` | 已跑：收集到的用例全绿（4 个 grpc 相关模块因本机缺 `grpc` 无法收集，属既有环境缺口） |
| `--values` 占位符拒绝 | 已跑：报 `--values 里仍是占位符：PROVIDER_ENDPOINT` |
| `--values` 正常载入 + 与 `--env-set` 合并 | 已跑：`载入 4 项`，远端命令含 4 个 `--env-set` |
| 缺部署期取值的报错 | 已跑：`PROVIDER_ENDPOINT（模板第 8,29 行）` |
| 单元资源限额渲染 | 已跑：`CPUQuota=150%` / `MemoryMax=1G`；板上 `systemd-analyze verify` rc=0 |
| `motion-cohost` 双角色**真装**（E300） | 已跑（手工经 `board_deploy.sh`，早于本脚本）：两角色各 exit 0、737 文件、自检通过 |
| `install_shape.sh` 四档 | **只跑过 dry-run**；`--apply` 尚未用它跑过（真装用的是 `board_deploy.sh`） |
| `x86-all` 全装 | **未跑**（本机已有 IRAF 开发环境，装上会与现有 venv/systemd --user 冲突，待单独安排） |
| `motion-split` | **未跑**（没有第二块可达的运控域机器） |

## 退出码

`0` 成功/演练完成｜`1` 参数错误｜`2` 本机预检失败（含构建目标推断失败）｜`3` 构建/组装失败｜
`4` 远端安装失败（范围/校验/单元冲突）｜`5` 后置判定失败。
