# IRAF SDK 安装说明

> 适用版本：v0.4.1（MCP 北向鉴权 + 上位机/南向 SDK + AgentOS worker 绑定）
> 事实来源：`config/install/install_profiles.yaml`（角色/端口/单元）、`config/sdk/package_matrix.yaml`（构建目标）、`profiles/boards/*.yaml`（板卡实测）、`docs/install-shapes.md`（判绿口径）。本文不重复这些值（AGENTS 铁律 5.3）。
> 全程 `simulation=true`，v1 只做仿真，不构成真机/实时能力证据。

---

## 1. 两种安装场景（本说明覆盖）

| 场景 | 部署形态 | 说明 |
|---|---|---|
| **场景一：IRAF 单独运行在 x86** | `single-ubuntu2204`（x86-all） | 仿真 + 运行同机装在一台 Ubuntu 22.04 / x86_64，用于开发、单机演示、回归 |
| **场景二：MuJoCo 仿真在 x86，其余在边缘智控域 + 运控域** | `sim-x86` + `edge-intel` + `edge-motion`（split-x86-edge） | 仿真引擎与 MuJoCo 只在 x86 Ubuntu；板卡侧 aarch64 Hyper 上智控域跑 TaskFlow/Skill/Policy/AgentOSBridge，运控域跑控制域 Capability Provider 实时段 |

## 2. 交付产物（三件）

| 产物 | 内容 |
|---|---|
| `iraf-sdk-<ver>-py3-none-any.whl` | 上位机 SDK（MCP 客户端 `iraf_sdk.mcp` + 南向 ProviderKit `iraf_sdk.provider`） |
| `iraf-runtime-<ver>-<arch>.tar.gz` | 控制面运行时（iraf_core + iraf_adapters + skills + profiles + config + deploy） |
| `iraf-board-<board>-<ver>-<arch>.tar.gz` | 板级 bundle（BoardProfile + install/uninstall/verify.sh + systemd + env 模板） |

构建目标单一：`aarch64-manylinux_2_28-cp311`（目标端 openEuler 24.03 / Python 3.11 / glibc 2.38）。

## 3. 安装档位（4 个）

| 档位 | 装什么 | 目标 | 对应角色 |
|---|---|---|---|
| `x86-all` | 仿真 + 运行同机 | 本机 Ubuntu 22.04 / x86_64 | `single-ubuntu2204` |
| `intel` | 板卡只跑 TaskFlow/Skill/Policy | 一块 aarch64 板 | `edge-intel` |
| `motion-split` | 智控域一台 + 运控域另一台 | 两台 aarch64 机器 | `edge-intel` + `edge-motion` |
| `motion-cohost` | 两域同一块板（路径已隔离） | 一块 aarch64 板 | `edge-intel` + `edge-motion` |

`motion-split` 与 `motion-cohost` 差别只有落点：前者两域各在自己机器（默认 `/opt/iraf-sdk`），后者两域同机（运控域自动落到 `/opt/iraf-sdk-motion`、`/etc/iraf-motion`、`/var/log|lib/iraf-motion`）。这是 `edge-motion` 角色的声明式路径覆盖，不是特例代码。

## 4. 一条命令安装（默认只演练，真装加 `--apply`）

```bash
# 场景一：x86 本机全装（仿真 + 运行）
bash scripts/install_shape.sh --shape x86-all

# 场景二（板卡只装智控域）
export SSHPASS='<板卡密码>'          # 只从环境变量传，不进命令行
bash scripts/install_shape.sh --shape intel \
  --host <板卡IP> --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052

# 场景二（两域分机）
bash scripts/install_shape.sh --shape motion-split \
  --host <智控域IP> --motion-host <运控域IP> \
  --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052

# 场景二（两域同机）
bash scripts/install_shape.sh --shape motion-cohost \
  --host <板卡IP> --values config/install/deploy_values/e300-edge-intel.example.yaml \
  --env-set PROVIDER_ENDPOINT=<x86主机>:50052
```

脚本自身不写任何路径/端口：版本从 `pyproject.toml`、构建目标按 BoardProfile 推断、角色路径读声明。

### 4.1 底层命令与安装包名（`install_shape.sh` 内部两步）

`install_shape.sh` 只是薄壳，内部先构建产物、再按角色分发。底层两步可直接执行（包名可见）：

```bash
# 第 1 步：构建产物（幂等；产物存在且比 src 新时跳过）
bash deploy/sdk/build_sdk.sh

# 产物（三个名字，版本号来自 pyproject.toml）：
#   build/sdk-cp311/iraf_sdk-<ver>-py3-none-any.whl          ← SDK wheel
#   build/sdk-cp311/iraf-runtime-<ver>-aarch64.tar.gz        ← runtime 包（控制面运行时）
#   build/wheelhouse/aarch64-manylinux_2_28-cp311/           ← 离线 wheelhouse

# 第 2 步：按角色分发（带显式包名）
bash deploy/sdk/board_deploy.sh --profile edge-intel \
     --sdk-wheel build/sdk-cp311/iraf_sdk-<ver>-py3-none-any.whl \
     --runtime-bundle build/sdk-cp311/iraf-runtime-<ver>-aarch64.tar.gz \
     --wheelhouse build/wheelhouse/aarch64-manylinux_2_28-cp311 \
     --values config/install/deploy_values/e300-edge-intel.yaml \
     --env-set PROVIDER_ENDPOINT=<x86-host>:50052 \
     --apply
```

`--profile` 取值：`single-ubuntu2204 | sim-x86 | edge-intel | edge-motion`（对应场景一/二的角色）。`install_shape.sh --shape x86-all/intel/motion-split/motion-cohost` 就是把"构建 + 按角色分发"打包成一键语义。

### 4.2 通过 AI 工具安装的语义

安装本质就是两条 shell 命令（`build_sdk.sh` 构建 + `board_deploy.sh` 分发）。AI 工具（AgentOS / AI Studio / 上位机 harness）**不直接"装"**，而是把上述命令作为运维动作执行，或由人工在板/主机上执行；IRAF 本身只提供幂等脚本 + 判绿口径，不在运行态暴露"安装"技能。要判绿只看三个口径：装得上 / 干净卸载 / 跨机调用（见 §6）。

## 5. 部署期取值（`--values` 与 `--env-set`）

- 与地址无关的取值（机型 Profile / 安全策略 / 调用方标识）放 `--values` 文件，可复用；
- **地址类取值不入库**（IP 实测变过），一律部署时 `--env-set` 给；
- `--values` 文件里残留 `<...>` 占位符会被拒绝（报 `--values 里仍是占位符：KEY`）；
- 缺必需占位符时目标端报「字段名 + 模板行号」，例如 `装配单模板缺部署期取值：PROVIDER_ENDPOINT（模板第 8,29 行）`。

## 6. 判绿口径（三档一致）

1. **装得上**：安装器退出码 0；`<prefix>/<version>` + `current` 链接 + env 文件 + 单元 + 安装记录齐备；后置自检同一解释器 import 通过（`['iraf_sdk-ok+stubs']`）。
2. **能干净卸载**：先 `uninstall-iraf.sh --dry-run` 看清单，再真卸，再跑独立 `verify_residue.sh`，六路（文件/空目录/单元/env/端口/进程）全 0。
3. **跨机调用**（`intel` / `motion-*`）：x86 侧起仿真 Capability Provider，板上发起任务跑到 `SUCCEEDED`。

注意两个"按设计为红"：安装**故意不启动服务** ⇒ 单元 `disabled`、进程不存在是正常的；`verify_residue.sh` 在装完后必然报"不干净"（已装即有内容），那是核对工具在正常工作。

## 7. 卸载与升级

- 卸载只回滚本 bundle 的文件（`file_record` 清单），数据目录与日志保留；检测到非本 bundle 文件即拒绝（防误删）。
- 版本化安装目录 `<prefix>/<version>` + 激活入口 `<prefix>/current` 符号链接，切换与回滚只动链接。
- 升级走 `board_upgrade.sh`，会自动重绑 AgentOS worker MCP（共享 config 合并 + token + 重启）。

## 8. 场景二的拓扑约束

- 仿真引擎只部署 x86（`install_profile_check.py` 拒绝 `adapters_sim` + `arch != x86_64`）。
- 板卡智能域不装仿真，只部署运行面（TaskFlow/SkillRuntime/PolicyGateway/AgentOSBridge）与跨域版本化接口。
- 智能域 ↔ 控制域：版本化 gRPC/ROS2 Action，独立进程，有 deadline/取消/健康检查/限流/mTLS；HTTP/网络推理/AgentOS 不得进入 L0/L1 闭环。
- 板卡经公共服务调 IRAF，或智能域经跨域接口把任务交给 x86 仿真 Provider（后者"智能域侧远端 Provider 后端"尚未实现，列为缺口）。

## 9. 已验证状态

- `install_profile_check.py` 全量校验：95 项检查 0 未通过。
- `pytest tests/unit`：收集到的用例全绿（4 个 grpc 相关模块因本机缺 `grpc` 无法收集，属既有环境缺口）。
- 单元资源限额渲染：`CPUQuota=150%` / `MemoryMax=1G`；板上 `systemd-analyze verify` rc=0。

## 10. 装好后进入实例工程开发

板卡装好 IRAF（SDK 在 `/opt/iraf-sdk/current`）后，机器人开发在**实例工程**里做，不与 SDK 混在一起。实例工程的标准结构、与 SDK 的分界、AgentOS 进入实例的 workflow，见 `docs/iraf-instance-project.md`。

一句话：**SDK 负责运行时（只读），实例工程负责场景/标定/任务（开发目录）**——板卡装好后 AgentOS 进入实例工程做机器人开发。
