# 跨架构 SDK 交付调试记录（iraf-24h）

- 范围：本战役（24 小时）在 x86 开发端可完成、可本机取证的部分；目标端验收一律 `DEFERRED`。
- 相关 ADR：`docs/adr/0006-sdk-cross-arch-delivery.md`；设计：`docs/iraf-engineering-design.md` §4/§5。
- 本文随战役后续步骤继续追加，不覆盖既有结论；新增小节按步骤编号排。

## 0. 本战役目标（一句话）

把"SDK → 板级 bundle → 目标端安装/自检/卸载 → 部署传输"做成**声明驱动、可复跑、带负向验收**的
交付链，机器人与板卡差异只出现在 `config/sdk/package_matrix.yaml` 与 `profiles/boards/*.yaml` 里。

## 1. 实测约束（量出来的，不是推断的）

| 约束 | 实测命令 | 结果 | 对设计的影响 |
|---|---|---|---|
| 开发端架构 | `uname -m` | `x86_64` | 交付物按 x86-first 取证；aarch64 只能做"可安装性"证明 |
| 交叉构建工具 | `docker buildx ls` / `registry-1.docker.io` 可达性 | 无 `buildx`，镜像源不可达 | **多架构 OCI 路线本战役不做**（不可执行就不承诺） |
| Python 源 | `pypi.org` 与 aliyun 镜像可达性 | 官方源不可达；aliyun 镜像可达 | offline wheelhouse 必须；源地址进声明（`index_url` 一行可换） |
| 目标架构 wheel | 镜像索引统计（按 cp 标签过滤） | aarch64 wheel 齐备（mujoco/grpcio/numpy/protobuf/pyyaml/pillow 均命中） | 可承诺"离线 wheelhouse 可安装性"，但传递依赖缺口需另行声明 |
| 边缘板卡 | `22` / `9119` 端口探测 | 均不通 | 目标端安装、`/health`、AgentOS 联通全部 `DEFERRED`；**禁止**用演练冒充 |
| 本机解释器 | `python3`（PATH）与 `/usr/bin/python3` | 前者 3.11.15（无 mujoco/numpy/grpc），后者 3.10.12（有） | 仓库脚本与单测一律显式 `/usr/bin/python3`；部署入口只用标准库，两者皆可 |

## 2. 已实现（x86 侧，均有验收证据）

| 步骤 | 产物 | 关键证据 |
|---|---|---|
| 01 | 决策记录与文件边界 | 单测基线 `Ran 263 / failures=1 / errors=4` |
| 02 | `config/sdk/package_matrix.yaml` + schema、`profiles/boards/{e300,firefly_rk3588}.yaml` | 板卡字段 `unverified`，负向单测 6 条 |
| 03 | `scripts/profile_check.py --board`（退出码 0/1/2） | `--board e300` → 2（19 条中文原因）；不依赖仿真器 |
| 04 | `docs/adr/0006`、`docs/adr/0007` | 8 段结构与既有 ADR 一致；内网地址 0 命中 |
| 05 | `src/iraf_sdk/`（错误码中文诊断 + 公共契约薄封装） | 三方比对 9/15/19；子进程导入纯净性证明无旁路 |
| 06 | `deploy/sdk/build_sdk.sh` + `lib_manifest.py` | 手工 PEP 427 wheel；连续两次构建 SHA-256 逐位相同；4 类篡改检测 |
| 07 | `deploy/sdk/fetch_wheelhouse.sh` + 标签门禁 | 7 个 wheel 通过 `cp310` + `manylinux_2_28_aarch64`；传递依赖缺口 10/1/9 已量化 |
| 08 | `package_board_bundle.sh` + `install.sh` | bundle 22 成员、两次组装哈希一致；`--dry-run` 沙箱演练；回滚恢复激活链接 |
| 09 | `verify.sh` + `uninstall.sh` | 安装记录 161 == 实测 161；卸载只删记录内文件；服务未启动 → 退出 5 |
| 10 | `deploy.sh`（media / ssh 双通道） | 见 §3 |

## 3. 步骤 10：部署双通道（本轮）

### 3.1 症状/需求
`deploy/sdk/` 已有组装（08）、安装（08）、自检与卸载（09），但**没有统一部署入口**：
隔离网环境（人工拷贝）与联网环境（ssh 直连）此前只能靠口头步骤。需求 1 §5 要求两者都在
同一个入口上，且都要有 `--dry-run`。

### 3.2 实现要点（文件边界：仅本窗口新增 3 个路径）
- `deploy/sdk/deploy.sh`：入口层，只做参数解析、通道校验与退出码透传（与 08/09 入口同构）。
- `deploy/sdk/lib_deploy.py`：实现层。**只用标准库**（`tarfile/json/hashlib/socket/subprocess/shutil`），
  因此部署入口不依赖 PyYAML/jsonschema —— 这正是"目标端缺依赖时仍能跑传输"的前提。
- bundle 的板卡/版本/架构取自 **bundle 内清单成员**（`iraf-board-bundle.json`），
  代码里不写死版本与标签（铁律 5.3）；清单缺字段即 `EXIT_PRECHECK`（不猜默认值）。
- 默认 bundle 解析规则：`build/sdk` 下**唯一**带伴随 `*.sha256` 的 `*.tar.gz`；
  命中 0 个或多个都显式失败并列出候选，**不自动挑第一个**（避免"拿错包"这类静默错误）。

### 3.3 通道语义对照

| 项 | media 通道 | ssh 通道 |
|---|---|---|
| 产物 | bundle 副本 + `sha256sum.txt` + `README-安装.md` + `deploy-report.json` | 远端安装报告 `install-report.json` + 本工具报告 |
| 退出码 0 的含义 | **交接物已产出**（不等于已安装） | 远端 `install.sh` 退出 0（本机无板卡，未实测） |
| 演练（`--dry-run`） | 只打印计划，不创建目录、不写文件 | 只做可达性探测 + 打印 6 条待执行命令 |
| 证据口径 | `evidence_scope=media_handoff_artifact`、`deployed=false` | 演练 `plan_only`；真实 `target_end_install` |
| 目标端验收 | `DEFERRED`（板卡不在场） | `/health`、gRPC、事件库检查仍需板卡上再跑 `verify.sh`（本工具不代跑、不当通过） |

### 3.4 实测（本轮，开发端）

验收 harness：`bash build/iraf-24h/10/run_acceptance.sh` → **EXIT=0，「通过 44 项，失败 0 项」**
（证据 `build/iraf-24h/10/acceptance-rerun.txt`；逐用例原始输出在 `build/iraf-24h/10/cases/`）。
小体积假 bundle 夹具（`make_fake_bundles.py`）只用于验证门禁逻辑，**不是产物证据**；
真实 bundle 的正向验收由 `media-real-apply` 用例单独完成。要点：

- `--transport media --output build/iraf-24h/10` → **exit=0**，产物 4 件（bundle 副本 / `sha256sum.txt` /
  `README-安装.md` / `deploy-report.json`），`sha256sum -c sha256sum.txt` 输出 `OK`（真校验，不是自查）。
- media `--dry-run` → exit=0 且**输出目录未被创建**（证明"演练不写盘"）。
- 负向：bundle 目录存在多个候选（含 `--bundle` 缺省歧义）→ **exit=2**；bundle 缺伴随 `.sha256` → **exit=2**；
  伴随校验和被篡改 → **exit=4**；`--transport ssh` 缺 `--target` / `--target` 不含 `user@host` → **exit=1**。
- ssh 可达性：不可达目标（本机已关闭端口，确定性复现）→ **exit=2** + 中文原因（含 `DEFERRED` 说明）；
  同命令在**可达端口**上（本机既有 TCP 监听）→ **exit=0**，说明可达性门禁不是"恒失败"门禁。
- 地址门禁：`grep -cE "[0-9]{1,3}\.[0-9]{1,3}\." deploy/sdk/deploy.sh` = 0；同式对实现层
  `lib_deploy.py` 亦为 0（主机地址只来自参数/环境变量）。
- **首跑暴露的是断言写错，不是代码缺陷（如实记录）**：`artifacts.length` 期望写成 4，实际 3 ——
  `deploy-report.json` 不登记自己（避免自指哈希），交接物 4 件 ≠ `artifacts` 3 条。
  首跑原始输出另存 `build/iraf-24h/10/acceptance-run1-fail.txt`（未覆盖），修断言后重跑
  → 44/0；同时补了 4 条"交接物文件真实存在"的断言（只断 `artifacts` 长度会漏掉报告自身）。
- 全量单测（`/usr/bin/python3`）：`Ran 528 / failures=1 / errors=4 / skipped=4`，与步骤 09 基线
  逐项一致（4 项 stub 导入 ERROR + `test_vision_processing` FAIL）；本步未新增 `tests/` 路径，
  故 `Ran` 数与上一步相同属预期，不是"少跑了用例"。

### 3.5 未实现 / 未实测（显式登记，不隐藏）

1. **ssh apply 全链路未实测**：本机 x86 开发端无目标板卡，`scp → 远端 sha256sum -c →
   远端 install.sh → 取回报告` 这一段只在 `--dry-run` 下验证了命令构造与可达性门禁；
   真实执行属 `DEFERRED`。板卡到位后的第一条命令（不改脚本语义）：
   `bash deploy/sdk/deploy.sh --transport ssh --target <user@host> --allow-unverified --json-out build/deploy/report.json`。
2. **失败路径的部分报告未落盘**：除"目标不可达"外，传输/远端失败当前只打印中文原因 + 退出码，
   未把半成品报告写入 `--json-out`。已登记为缺口，板卡到位后按需补齐（不影响 fail-closed 语义）。
3. **`StrictHostKeyChecking=accept-new`**：首次连接会写入 known_hosts；隔离网若需要固定指纹校验，
   应在板卡到位后改为预置 `known_hosts` 并保持 `BatchMode=yes`。当前未声明指纹，故不声称已做主机认证。
4. **失败即 DEFERRED 不做 dry-run 冒充**：`install.sh --dry-run` 的沙箱演练证据属于步骤 08/09，
   本步骤未使用、也未引用它作为部署证据。
