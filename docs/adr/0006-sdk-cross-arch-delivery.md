# ADR-0006：SDK 跨架构交付采用 L1「纯 Python + 离线 wheelhouse + shell 脚本」

**状态**：已接受（L1 为当前唯一可执行路由；L2/L3 列为后续，签名仅占位）
**日期**：2026-09-20
**决策人**：架构负责人、SDK 负责人、运维负责人、板卡/系统组（待签名）

## 决策

IRAF 对 aarch64 边缘板（E300 / FIREFLY-RK3588）的交付采用 **L1 分级：纯 Python SDK + 目标架构离线 wheelhouse + POSIX shell 安装/验证/部署脚本**，本轮不实现 OCI 多架构镜像与板级原生包。

1. 产物固定为三类（顺序即构建顺序，缺一不可）：
   1. `iraf-sdk-<ver>-py3-none-any.whl`——SDK 层（`iraf_sdk`），纯 Python、与目标架构无关，可在开发端 x86_64 完整测试；
   2. `iraf-runtime-<ver>-<arch>.tar.gz`——控制面运行时（`iraf_core` + `iraf_adapters` + `skills/` + `profiles/` + `config/` + `deploy/`）+ 离线 wheelhouse；
   3. `iraf-board-<board>-<ver>-<arch>.tar.gz`——板级 bundle = `BoardProfile` + `install.sh`/`uninstall.sh`/`verify.sh` + systemd 单元 + env 模板 + 目标端证据目录约定。
   每个产物附 `manifest.json`（schema `iraf.package-manifest/v1`）与逐文件 SHA-256。
2. **目标端只做「安装 + 配置 + 预检 + 启动」**，不在目标端编译 Rust/C++/现场总线 SDK；跨架构承诺的是**同一 IDL / TaskFlow / Skill 合约语义可运行**，不是同一二进制。
3. 版本、Python 标签、平台标签、包清单、镜像源、板卡映射集中在 `config/sdk/package_matrix.yaml`；板卡事实集中在 `profiles/boards/*.yaml`；**未实测字段一律 `unverified`，预检即失败**，禁止猜测默认值。
4. L2（目标端多架构 OCI）与 L3（板级原生包 / BSP / 厂家 runtime）**不在本轮实现**，保留为路线图并附启用前置条件；不在文档中表述为已可用（`AGENTS.md` 6.4）。
5. 签名本轮只保留 `signature.scheme = "pending"` 占位，只做 SHA-256 + manifest 校验和。

## 理由（开发端 x86_64 实测，2026-09-20，命令与结果原样记录）

| 事实 | 实测结果 | 命令 |
|---|---|---|
| 开发端身份 | x86_64，Ubuntu 22.04，系统 Python 3.10.12 | `uname -m` / `/usr/bin/python3 -VV` |
| 多架构 OCI 构建能力 | **`docker buildx` 不存在** | `docker buildx ls` |
| 镜像仓库可达性 | `registry-1.docker.io/v2/` **不可达** | `curl -o /dev/null -w '%{http_code}'` |
| 官方 PyPI | `pypi.org` **不可达**；`mirrors.aliyun.com/pypi/simple`、`pypi.tuna.tsinghua.edu.cn` 均 `200` | `curl` |
| 目标架构交叉编译器 | `aarch64-linux-gnu-gcc` **缺失** | `command -v` |
| qemu 用户态 | `/usr/bin/qemu-aarch64-static` 存在，binfmt `qemu-aarch64` 已注册 | `ls /proc/sys/fs/binfmt_misc/` |
| 目标板卡 | `<边缘板卡A>` / `<边缘板卡B>` 的 22 与 9119 **均不通** | `timeout 4 bash -c "echo > /dev/tcp/<host>/<port>"` |
| aarch64 wheel 可得性 | mujoco 492 / numpy 1088 / grpcio 953 / protobuf 212 / pyyaml 210 / pillow 1224（索引内文件名计数） | `scripts/probe_aarch64_wheels.py` |

**结论**：目标架构**可离线安装**（wheel 齐备），但多架构 OCI **不可执行**（无 buildx + 无镜像源），目标端原生编译**不可行**（无交叉工具链），真机验收**当前阻塞**（板卡不通）。因此 L1 是当前唯一既能执行、又能在本机完整取证的交付路由。

## 限制与风险

| 风险 | 失败表现 | 处理（fail-closed，不伪造成功） |
|---|---|---|
| 目标端 Python 版本 / 平台标签未知 | 装了 `py3-none-any` 但 C 扩展装不上 | `BoardProfile` 未声明即预检失败（退出码 2），禁止「先试再错」 |
| 抓错平台标签的 wheel | 目标端 `pip install` 报 manylinux 不兼容 | 抓取后按声明 `platform_tag` 逐文件过滤；镜像索引混有 `win_arm64` / `macosx` 干扰项，非目标标签一律拒绝（退出码 2） |
| 镜像索引名大小写 | 误判「包不存在」 | 索引名必须小写（`pyyaml` 有、`PyYAML` 返回 404），已写入声明与脚本约定 |
| 目标端 glibc 低于 wheel 要求 | 安装期报 manylinux 不兼容 | 预检阶段拦截，给出「改用 L3 板级包」的中文建议 |
| 隔离网无镜像源 | 目标端无法补依赖 | 全部依赖入 wheelhouse，bundle 内自带，安装期零联网 |
| 目标端不在场 | 无法产出真机证据 | 目标端验收统一 `DEFERRED`（延后，不是失败）；**严禁**用 dry-run、mock、本地 stub 或历史数据冒充 | 
| 无 NPU runtime | 本地推理不可用 | 声明 `unavailable`，降级为远端 Provider；不得伪造本地推理结果 |
| 传输中断 / 校验不符 | 半成品 bundle | `sha256sum -c` 前置校验，失败立即中止，不进入安装 |

## 验证门禁

1. `deploy/sdk/build_sdk.sh` 退出码 0，`manifest.json` 内全部 SHA-256 复算一致；产物路径不含绝对路径与 `..`。
2. `deploy/sdk/fetch_wheelhouse.sh` 抓取的**每个** wheel 平台标签匹配声明的 `platform_tag`；负向用例：`*-win_arm64.whl` / `*-macosx*.whl` 必须被拒绝（退出码 2）。
3. bundle 在干净目录解包后 `install.sh --dry-run` 输出完整预检清单且**不写系统目录**；`install.sh` 幂等。
4. 负向：手动篡改 bundle 内任一文件 → `verify.sh` 退出码 4；`unverified` 的 `BoardProfile` 未经 `--allow-unverified` → `package_board_bundle.sh` 退出码 2。
5. 全脚本统一退出码：0 成功 / 1 用法错误 / 2 预检失败 / 3 构建安装失败 / 4 校验失败 / 5 健康检查或 `profile-check` 失败。
6. `qemu-aarch64-static` 只用于验证 `install.sh` 的**纯 shell 逻辑**，不构成目标端 Python/依赖验收。
7. **目标端验收（DEFERRED，板卡不在场）**：目标端安装 + `profile-check` 通过 + systemd `active` + `/health` 返回、`AgentOSBridge` 与 AgentOS 版本协商、崩溃/断电重启的幂等恢复与回滚——板卡可达后在同一脚本上执行，不新增旁路脚本。

## 参考

- `docs/iraf-multiplatform-sdk-design.md`（§2 实测约束、§3 分级、§5 脚本契约、§9 验收分层、§12 本战役范围）
- `config/sdk/package_matrix.yaml`、`config/sdk/package_matrix.schema.json`
- `profiles/boards/e300.yaml`、`profiles/boards/firefly_rk3588.yaml`
- `scripts/profile_check.py --board <id> [--allow-unverified]`（退出码 0/1/2）
- `docs/iraf-engineering-design.md` §4 跨发行版与架构策略、§5 Profile 与适配边界
- `AGENTS.md` 5.3（配置集中声明）、6.4（命令实现前不得称已可用）、2.7（不提交内部地址）
