# IRAF 多架构 SDK 与跨端交付设计（开发端 x86_64 → 目标端 aarch64）

**状态**：设计草案 v0.1（**未实现**；按 AGENTS.md 6.4，命令在实现前只是产品契约，不得在说明或演示中表述为已可用）
**日期**：2026-09-20
**范围**：需求 1 —— 以 SDK 方式交付，开发端 x86_64 构建，目标端 aarch64（E300 / FIREFLY-RK3588）通过 shell 脚本部署与验证
**依据**：`docs/iraf-engineering-design.md`（§3 目标产物、§4 跨发行版与架构策略、§5 Profile 边界）、`AGENTS.md`（铁律 1.x / 2.x / 5.x / 6.x）、`docs/iraf-agentos-hyper-compatibility.md`

---

## 1. 目标与非目标

目标：让"把 IRAF 交付到一块 aarch64 边缘板并跑起来"变成一条**可重复、可离线、可审计**的脚本化路径：

```text
开发端 x86_64（本机 10.203.247.145）
  iraf-sdk 源码 + 声明式矩阵
        |  sh 脚本：构建 → 打包 → 校验和 → （可选）签名
        v
交付产物（tar.gz + manifest.json + wheelhouse + SBOM）
        |  sh 脚本：scp/ssh 到目标端（或介质拷贝）
        v
目标端 aarch64（E300 / FIREFLY-RK3588 的 Linux VM）
  install.sh：预检 → 安装 → profile-check → systemd 启动 → 健康检查 → 证据落盘
```

非目标（明确排除，避免越界）：

1. IRAF 不交付 Hypervisor、内核/BSP、硬实时主站、驱动；这些仍由板厂/系统组提供，IRAF 只通过受控 Capability Provider 使用（`AGENTS.md` 0 节）。
2. 不在目标端编译 Rust/C++ 核心或现场总线 SDK；目标端只做"安装 + 配置 + 预检 + 启动"。
3. 不承诺"同一二进制跨架构"；承诺的是**同一 IDL / TaskFlow / Skill 合约语义**在两端可运行（`iraf-engineering-design.md` §4）。
4. 不在本轮承诺 OCI 多架构镜像（原因见 §3 的实测约束）。

---

## 2. 现状实测约束（2026-09-20 本机实测，命令与结果原样记录）

这一节是设计的事实基础。任何与下表冲突的设计假设都必须先复测再写进文档。

| 项 | 实测值 | 命令 |
|---|---|---|
| 本机身份 | `coretek-System-Product-Name`，`x86_64`，IP `10.203.247.145` | `uname -m` / `hostname -I` |
| 系统 Python | `Python 3.10.12 (main, Aug 31 2026) [GCC 11.4.0]` | `python3 -VV` |
| Conda 环境 | `~/miniconda3/envs/mujoco_graspnet` 为 `Python 3.9.21`；服务当前用它启动 | `deploy/iraf-runtime.service` |
| docker | `29.1.3 x86_64 linux`，**`docker buildx` 不存在** | `docker info` / `docker buildx ls` |
| docker hub | `registry-1.docker.io/v2/` **不可达** | `curl` |
| qemu 用户态 | `/usr/bin/qemu-aarch64-static` 存在；`/proc/sys/fs/binfmt_misc/qemu-aarch64` 已注册 | `command -v` / `ls` |
| aarch64 交叉编译器 | `aarch64-linux-gnu-gcc` **缺失** | `command -v` |
| protoc | `libprotoc 3.12.4`；`grpc_python_plugin` **缺失** | `protoc --version` |
| PyPI 官方源 | `pypi.org` **不可达** | `curl` |
| 发行版镜像源 | `mirrors.aliyun.com/pypi/simple` 与 `pypi.tuna.tsinghua.edu.cn` 均 `200` | `curl` |
| GitHub / Gitee | `github.com` `200`、`gitee.com` `200`（https 可用；仓库 push 另受限制） | `curl` |
| 板卡 `10.203.247.72` | 22 与 9119 **均不通** | `/dev/tcp` 探测 |
| 板卡 `10.203.247.86` | 22 与 9119 **均不通** | `/dev/tcp` 探测 |
| 已存在目录 | `deploy/`（systemd + 1 个 sh）、`api/proto/iraf/v1/`、`build/generated/python/` | `find` |
| 尚不存在 | `sdk/`、`profiles/boards/`、任何打包产物（无 `.whl/.tar.gz/.deb`） | `find` |

**aarch64 wheel 可得性实测**（`build/probe_aarch64_wheels.py`，源为 aliyun 镜像；数字为索引内文件名计数）：

| 包 | aarch64 wheel 数 | cp39~cp312 可用 | 样例 |
|---|---|---|---|
| mujoco | 492 | 350 | `mujoco-3.9.0-cp312-cp312-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl` |
| numpy | 1088 | 526 | （含 `win_arm64` 干扰项，选择时必须按平台标签过滤） |
| grpcio | 953 | 593 | `grpcio-1.84.0rc2-cp312-cp312-musllinux_1_2_aarch64.whl` |
| protobuf | 212 | 78 | `protobuf-7.36.1-cp310-abi3-manylinux2014_aarch64.whl` |
| pyyaml | 210 | — | 注意索引名必须用小写 `pyyaml`，`PyYAML` 返回 `404` |
| pillow | 1224 | — | — |
| jsonschema | 0 | — | 纯 Python（`py3-none-any`），无需 ABI wheel |

**由此得到的三条硬约束**：

1. **离线优先**：官方 PyPI 不可达，目标端安装不能依赖 `pip install <name>` 联网解析。必须由开发端预先抓取 wheelhouse（含 `--platform` 目标标签），随 bundle 交付。
2. **OCI 路线当前不可执行**：无 buildx、docker hub 不可达、无 aarch64 基础镜像缓存 → 多架构 OCI manifest 只能作为后续阶段，前置条件是镜像源或本地 registry 可用。
3. **aarch64 真机验收当前阻塞**：两块板卡 22/9119 均不通，本轮只能做到"产物完整性 + 目标端脚本在 x86 上的 dry-run 级验证"，真机安装与健康检查必须等网络恢复后才能给出证据（禁止用 mock 结果代替，`AGENTS.md` 5/6 节）。

---

## 3. 跨架构交付路线分级（按可行性排序，只允许逐级启用）

| 级别 | 形态 | 当前可行性 | 能力上限 | 启用前置 |
|---|---|---|---|---|
| **L1 纯 Python + 预编译 aarch64 wheel**（首选） | `tar.gz` bundle：`iraf` wheel（含生成的 proto stub）+ 离线 wheelhouse + profiles/configs + systemd 单元 + `install.sh` | **可执行**（本机可完成打包与脚本级验证） | 控制面（TaskFlow/SkillRuntime/Policy/Store）、HTTP/gRPC 适配、MuJoCo 无头（若需）、AgentOSBridge；**不含** ROS 2 / 现场总线 / NPU runtime | 目标端 Python 版本与平台标签由 BoardProfile 声明 |
| L2 目标端 OCI 镜像 | 多架构 OCI（`linux/amd64`、`linux/arm64`）+ digest 引用 | **不可执行**（无 buildx、不可达 docker hub） | 在 L1 基础上隔离依赖、便于回滚 | 镜像源/本地 registry 可用；或板卡自带容器运行时与离线导入通道 |
| L3 板级原生包（BSP/RPM/deb + 厂家 runtime） | 由板厂工具链出包，IRAF 只提供 manifest 与 profile-check | 需板厂协作 | 实时/Master/NPU 全能力 | 板级清单签字（§6） |

**分阶段承诺**：本轮只承诺 L1 的"开发端产物 + 脚本 + 目标端 dry-run"，L2/L3 写入路线图但不实现。

---

## 4. 产物契约（每个产物一份 `manifest.json`）

统一 schema：`iraf.package-manifest/v1`。字段必须集中来自声明文件 `config/sdk/package_matrix.yaml`（见 §7），脚本中禁止硬编码版本、Python 标签、平台标签、包清单。

```json
{
  "schema_version": "iraf.package-manifest/v1",
  "name": "iraf-runtime",
  "version": "0.2.0",
  "idlv1_compatible": ["1.0.0"],
  "git": {"commit": "<sha>", "dirty": false},
  "target": {"os": "ubuntu-22.04", "arch": "aarch64", "python_tag": "cp310",
             "platform_tag": "manylinux_2_28_aarch64", "verified": false},
  "artifacts": [{"path": "iraf-0.2.0-py3-none-any.whl", "sha256": "..."}],
  "wheelhouse": [{"name": "mujoco", "version": "3.3.3", "tag": "manylinux_2_28_aarch64", "sha256": "..."}],
  "sbom": "sbom.spdx.json",
  "signature": {"scheme": "pending", "value": null},
  "notes": ["simulation_only 范围声明", "NPU runtime 未声明 => unverified"]
}
```

三类产物（缺一不可，顺序即构建顺序）：

1. `iraf-sdk-<ver>-py3-none-any.whl`：SDK 层（`iraf_sdk`），纯 Python，不绑定架构；含 `client`、错误码→中文诊断映射、最小示例。**与目标端架构无关，因此可在 x86 上完整测试。**
2. `iraf-runtime-<ver>-<arch>.tar.gz`：控制面运行时（`iraf_core` + `iraf_adapters` + `skills/` + `profiles/` + `config/` + `deploy/`），附离线 wheelhouse。
3. `iraf-board-<board>-<ver>-<arch>.tar.gz`：板级 bundle = BoardProfile（含 `unverified` 标记）+ `install.sh`/`uninstall.sh`/`verify.sh` + systemd 单元 + env 模板 + 目标端证据目录约定。

产物必须满足：`tar.gz` 内路径不含绝对路径与 `..`；每个文件登记 SHA-256；`uninstall.sh` 只回滚本 bundle 且不动数据目录；`install.sh` 幂等（重复执行结果一致，`AGENTS.md` 2.4）。

---

## 5. 脚本契约（金路径的 sh 实现）

目录：`deploy/sdk/`（与既有 `deploy/*.service`、`deploy/install-iraf-orchestrator.sh` 同层）。所有脚本：`set -euo pipefail`、显式依赖与架构检查、中文可操作错误、`--help`、`--dry-run`、退出码约定。

退出码约定（全脚本统一，供 CI 与运维判读）：

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 1 | 参数/用法错误 |
| 2 | 预检失败（架构、Python、依赖、声明缺失或 `unverified`） |
| 3 | 安装/构建失败（IO、空间、权限） |
| 4 | 校验失败（SHA-256、manifest 不一致、IDL 不兼容） |
| 5 | 健康检查或 `profile-check` 失败（已安装但拒绝启动） |

| 脚本 | 运行侧 | 职责 | 关键拒绝条件 |
|---|---|---|---|
| `deploy/sdk/build_sdk.sh` | x86 开发端 | 读矩阵 → 生成 proto stub → 构建 SDK wheel 与 runtime bundle → 写 manifest/SBOM/校验和 | 生成物含本机绝对路径；IDL 破坏性变更但未声明兼容版本 |
| `deploy/sdk/fetch_wheelhouse.sh` | x86 开发端 | 按声明的 `python_tag`/`platform_tag`/包清单抓取离线 wheel（`pip download --only-binary=:all: --platform …`） | 抓到的 wheel 平台标签与声明不符；出现 `win_*`/`macosx_*` 等非目标标签（实测 numpy 索引混有 `win_arm64`） |
| `deploy/sdk/package_board_bundle.sh` | x86 开发端 | 组装板级 bundle（BoardProfile + 上两者 + 脚本 + systemd） | BoardProfile 关键字段为 `pending`/`unverified` 时**默认拒绝**，需显式 `--allow-unverified` 且写进证据 |
| `deploy/sdk/deploy.sh` | x86 开发端 | 传输（scp/介质）→ 远端调用 `install.sh` → 取回证据 JSON | 目标端 arch/Python 与 manifest 不符；磁盘空间不足 |
| `deploy/sdk/install.sh` | aarch64 目标端 | 预检 → 离线安装 → 写 env → `profile-check` → 启动 → 健康检查 | `profile-check` 失败即不启动（fail-closed） |
| `deploy/sdk/verify.sh` | aarch64 目标端 | `profile-check` + `/health` + 事件写入检查 + 输出证据 JSON | 任何一项不符即退出码 5，不打印"通过" |
| `deploy/sdk/uninstall.sh` | aarch64 目标端 | 停服务 → 移除本次 bundle 文件 → 保留数据与日志 | 检测到非本 bundle 文件即拒绝（防误删） |

`deploy.sh` 的两种传输模式必须都支持并声明：`--transport ssh`（需要板卡可达；当前实测不可达）与 `--transport media`（生成 `bundle.tar.gz` + `sha256sum.txt` 由人工拷贝，用于隔离网环境）。**禁止把凭据写入仓库**；SSH 目标、端口、介质路径全部来自配置或环境变量。

---

## 6. BoardProfile 与预检（`profiles/boards/`）

现状：`profiles/` 下只有 `piper_mujoco.yaml`、`ur5_mujoco.yaml`、`engineering_tooling.yaml`、`safety/`；**`profiles/boards/` 尚不存在**，而 `iraf-engineering-design.md` §5 已把 `BoardProfile` 写进契约。本设计补齐该目录与其 schema。

```yaml
apiVersion: iraf.intewell.io/v1
kind: BoardProfile
metadata: {name: e300, version: 0.1.0}
spec:
  status: unverified            # 未签字前只能是 unverified
  target:
    os: unverified
    arch: aarch64
    python_tag: unverified      # 实测后才能填 cp310 等
    platform_tag: unverified    # 实测后才能填 manylinux_2_28_aarch64 等
    kernel: unverified
  runtime: {oci: unavailable, ros2: unavailable, systemd: unverified}
  adapters: {agentos_bridge: unverified, npu: unverified, master: unavailable}
  limits: {ram_mb: unverified, disk_mb: unverified}
  evidence: {owner: pending, acceptance_report: pending}
```

预检（`install.sh` 第 2 步，退出码 2）逐项比对：`uname -m`、Python 版本与标签、glibc/manylinux 兼容性、可用 RAM/磁盘、systemd 可用性、目标端已装包与 wheelhouse 冲突。**`unverified` 视为未验证，不得当作默认放行**（`AGENTS.md` 3 节 + `iraf-engineering-design.md` §5）。

---

## 7. 集中声明（禁止散落硬编码）

新增 `config/sdk/package_matrix.yaml`，作为"产物矩阵"的唯一事实来源：

```yaml
schema_version: iraf.sdk-matrix/v1
sdk: {name: iraf-sdk, module: iraf_sdk}
idlv1_version: 1.0.0
targets:
  - id: aarch64-manylinux_2_28-cp310
    arch: aarch64
    platform_tag: manylinux_2_28_aarch64
    python_tag: cp310
    pure_python: [jsonschema, pyyaml]
    wheels: [mujoco, numpy, grpcio, protobuf, pyyaml, pillow]
    index_url: https://mirrors.aliyun.com/pypi/simple/   # 官方 pypi 不可达，见 §2
    boards: [e300, firefly_rk3588]
```

版本、Python 标签、平台标签、包清单、镜像源、板卡映射全部在此；脚本只读不写。这与现有 `config/*.yaml` + `profiles/*.yaml` 的分工一致（`AGENTS.md` 5.3）。

---

## 8. 契约与 IDL 影响

| 变更 | 类型 | 兼容性 |
|---|---|---|
| 新增 `iraf.package-manifest/v1` schema | 新增（非 IDL） | 向后兼容 |
| 新增 `BoardProfile` 字段 `target.python_tag` / `platform_tag` | profile 扩展 | 向后兼容（缺省即 `unverified`，拒绝装配） |
| 新增 `deploy/sdk/*.sh` | 交付工具 | 不影响运行时 |
| `api/proto/iraf/v1/*.proto` | **不变** | 本设计不改任何 Skill/Task/Event 契约 |

S DK 层如需暴露"提交任务/查询执行"，必须复用既有 `runtime.proto` / `events.proto` 的公共契约，**不得为 SDK 新造一条绕过 `TaskFlow -> SkillRuntime -> Policy` 的旁路**（`AGENTS.md` 1.2）。

---

## 9. 验收场景与证据（分层，禁止越级宣称）

在本机（x86_64）可完成的验收：

1. `build_sdk.sh` 退出码 0，产物 manifest 的 SHA-256 全部复算一致；
2. `fetch_wheelhouse.sh` 抓取的每个 wheel 的平台标签均匹配 `platform_tag`（实测样例：`mujoco-3.9.0-cp312-cp312-manylinux_2_27_aarch64…` 可接受、`numpy-…-win_arm64.whl` 必须被拒绝）；
3. bundle 在干净目录解包后 `install.sh --dry-run` 输出完整预检清单（不写系统目录）；
4. 负向：手动篡改一个文件 → `verify.sh` 必须退出码 4；`unverified` BoardProfile 未经 `--allow-unverified` → `package_board_bundle.sh` 必须退出码 2；
5. `qemu-aarch64-static` 可在本机执行 aarch64 静态二进制，用于验证 `install.sh` 的**纯 shell 逻辑**（不构成目标端 Python/依赖的验收）。

必须等板卡可达才有证据的验收（**当前阻塞项，不得用 mock 代替**）：

6. 目标端安装 + `profile-check` 通过 + systemd `active` + `/health` 返回；
7. 目标端 `AgentOSBridge` 与 AgentOS 的版本协商与任务提交 smoke；
8. 崩溃/断电后重启的幂等恢复与回滚。

后续（L2/L3）另有验收，不在本轮范围。

---

## 10. 风险、失败与接管路径

| 风险 | 失败表现 | 处理（fail-closed，不伪造成功） |
|---|---|---|
| 目标端 glibc 低于 wheel 要求 | `pip install` 报 `manylinux` 不兼容 | 预检阶段拦截（退出码 2），给出"需要 manylinux_2_28_x 或改用 L3 板级包"的中文建议 |
| 目标端 Python 版本未知 | 装了 py3-none-any 但 C 扩展装不上 | BoardProfile 必须显式声明；未声明即预检失败，禁止"先试再错" |
| 磁盘/RAM 不足 | 安装中途失败、半成品目录 | 安装到版本化目录 + 符号链接切换；失败即回滚到上一版本并保留日志 |
| 无 NPU runtime | 语音/视觉本地推理不可用 | 声明为 `unavailable`，降级为远端 Provider；**不得伪造本地推理结果**（`AGENTS.md` 1.5） |
| 传输中断/校验不符 | 文件不完整 | `sha256sum -c` 前置校验，失败立即中止，不进入安装 |
| 隔离网无镜像源 | 目标端无法补依赖 | 全部依赖入 wheelhouse；bundle 内自带，安装期零联网 |

---

## 11. 决策记录（2026-09-20 已确认）

> 原选项文本保留在下方以便回溯。**已确认项**按决策取值改写；未纳入本轮 7 项决策的子问题明确标注「未确认 / 不在本战役范围」，不得当作已确认实施（`AGENTS.md` 2.6、5.5）。

1. **SDK 形态** —— **已确认：A 仅 Python SDK（2026-09-20）**。
   - 原选项：(A) 仅 Python SDK（本轮）；(B) Python SDK + C++ SDK 头文件（`sdk/cpp/`，本轮只出契约与头文件，不出二进制）；(C) Python SDK + gRPC 客户端只读封装。
   - 落点：`sdk/cpp/` 只保留契约占位，**不产出任何 C++ 二进制**；B 进路线图。
2. **wheelhouse 来源** —— **已确认：A（2026-09-20 更新）先用 aliyun 镜像 `https://mirrors.aliyun.com/pypi/simple/`**。
   - 原选项：(A) aliyun 镜像（实测可达，且实测含 aarch64 wheel）；(B) 内网私有源（需提供地址）；(C) 板厂提供依赖清单。
   - 落点：决策由原 B 改为 A；原选项 B 保留为可切换路径——拿到内网私有源后**只改 `config/sdk/package_matrix.yaml` 的 `index_url` 一行**，脚本禁止写死源地址；同时记录镜像与抓取时间便于审计。
3. **目标板卡 Python 版本 / 平台标签** —— **已确认：A 先写 `unverified`（2026-09-20）**。
   - 原选项：(A) 先写 `unverified`，板卡实测后回填；(B) 由提供方给出板卡实测信息；(C) 等板卡可达再定。
   - 落点：`profiles/boards/*.yaml` 中未回填的字段即预检失败（退出码 2）；**禁止猜测默认值**，也不得用 x86_64 wheel 顶替 aarch64（`AGENTS.md` 1.5 / 5.10）。
4. **签名方案** —— **未确认 / 不在本战役范围**：本战役只做 SHA-256 + `manifest.json` 校验和；cosign/minisign 排期到板卡可用之后（见 §12 表）。
   - 原选项：(A) 本轮只做 SHA-256 + manifest 记录签名占位；(B) 引入 cosign/minisign。

---

## 12. 本战役范围（x86-first，2026-09-20；板卡不在场）

本战役（`plans/iraf-24h`）只交付 **L1 路线**，且完成判据限定为**开发端 x86_64 本机可复现并取证**的部分：

| 项 | 本战役 | 依据 / 说明 |
|---|---|---|
| L1 纯 Python SDK + aarch64 离线 wheelhouse | **做** | wheel 抓取是网络行为，不需要板卡；平台标签必须按声明过滤，错标签即失败（退出码 2） |
| 打包、`manifest.json`、校验和 | **做** | `deploy/sdk/build_sdk.sh`，SHA-256 全部复算一致 |
| 安装 / 验证 / 部署脚本 + `--dry-run` + 负向用例 | **做（x86 侧）** | 只证明解析、预检与拒绝逻辑；**不证明目标端可用** |
| 目标端真实安装 / `/health` / AgentOS 联通 / ssh 真机部署 | **排除（DEFERRED）** | 边缘板卡不在场（`10.203.247.72`/`.86` 的 22 与 9119 实测不通）；不得用 dry-run、mock、本地 stub 或历史数据冒充 |
| 多架构 OCI 镜像 | **排除** | 无 `docker buildx` 且 `registry-1.docker.io` 不可达（§2 实测） |
| 目标端编译（Rust/C++/现场总线 SDK） | **排除** | §1 非目标 2 |
| 签名服务（cosign/minisign） | **排除（只留占位）** | 决策 4 未确认，见 §11 |

以上排除项在板卡到位后**复用同一脚本语义**执行，不新增旁路脚本（`AGENTS.md` 6.4：命令实现前不得表述为已可用）。

---

## 13. 文件边界（本战役 vs 另一窗口，2026-09-20 锁定）

本仓库当前有**两个编辑窗口**并行；为避免互相覆盖，路径归属按下表锁定。越界即视为违反 `AGENTS.md` 5.7（变更必须小而可审查）。

| 归属 | 路径 |
|---|---|
| **本战役（SDK / 宇树线）** | `deploy/sdk/`、`config/sdk/`、`profiles/boards/`、`scenes/`、`vendor/unitree_*`、`src/iraf_sdk/`、`src/iraf_adapters/unitree/`、`skills/{stand,stop,locomote}/`、`scripts/build_scene.py`、`scripts/scenario.py`、`scripts/verify_go2_loopback.py`、`plans/iraf-24h/`、`docs/iraf-multiplatform-sdk-design.md`、`docs/iraf-unitree-scenario-interaction-design.md`、`docs/iraf-voice-studio-design.md`、`docs/adr/0006-*`、`docs/adr/0007-*` |
| **另一窗口（不触碰）** | `examples/demo3_arm/`、`scripts/view_mujoco.py`、`scripts/verify_pick.py`、`scripts/build_baseline.py`、`config/piper_simulation_baseline.yaml`、`config/ur5_simulation_baseline.yaml`、`src/iraf_core/`、`src/iraf_adapters/factory.py` |
| **共享但需先声明** | `src/iraf_core/`、`src/iraf_adapters/factory.py`（若确需改动，先在 `plans/iraf-24h/00-日志.md` 登记文件边界并说明原因） |

执行纪律：每个 tick 提交前 `git status --short` 必须只包含本步声明路径；禁止 `git stash` / `git checkout` / `git reset` / `git add -A` / `git add -f`。
