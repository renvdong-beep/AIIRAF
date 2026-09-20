# IRAF 24 小时计划战役 `iraf-24h` 执行结果与缺口汇总（2026-09-20）

> 本文是 `plans/iraf-24h/` 这一轮 24 小时受控编码的**一次性汇总**：产物、证据、关键数字、诚实缺口与下一阶段建议。
> 权威台账仍是 `plans/iraf-24h/00-STATUS.json`；逐步的 tick 记录见 `plans/iraf-24h/00-日志.md`；
> 专项调试记录见 `docs/debug/2026-09-20-*.md`。
>
> **边界声明（先读这一行）**：本轮**全部仿真结论均为 `simulation: true`**，只在开发端 x86_64 主机（`/usr/bin/python3` 3.10.12 + MuJoCo 3.3.3）取证。
> **边缘板卡不在场**，所有目标端/真机验收（真实安装、`/health`、systemd、事件库写入、ssh 真机部署、AgentOS 联通）统一标记 **`DEFERRED`（延后，不是失败）**。
> 本轮**没有**用 dry-run、mock、本地 stub 或历史数据冒充目标端证据；stub 级证据在文中逐条标注。任何未在板卡/真机验证的能力都**不得**表述为已支持。

## 1. 概览

| 项 | 值 |
|---|---|
| 战役 | `iraf-24h`（来源计划 `.hermes/plans/2026-09-20_multiplatform-sdk-unitree-studio.md`） |
| 范围模式 | **x86-first**（边缘板卡不在场，目标端验收统一 `DEFERRED`） |
| 步骤 | 20 步：**19 步 DONE（01–19）+ 1 步收尾 DONE（20，本文）**；无 `BLOCKED`（阻塞项以 `DEFERRED` 登记，见 §5） |
| 已确认决策 | 7 项（`1A / 2 先用 aliyun 镜像 / 3A / 4B / 5A / 6A / 7A`） |
| 引导提交 → 收尾前 HEAD | `5df5d05` → `4a36601`（区间含另一窗口提交；区间统计：103 files changed, 28905 insertions(+), 166 deletions(-)） |
| 各步提交 | 01 `fb0453a` / 02 `7d4d683` / 03 `1922fb9` / 04 `661a003` / 05 `8122ef7` / 06 `a343b74` / 07 `de2ada3` / 08 `74647d1` / 09 `87a4b4b` / 10 `eb24b67` / 11 `0e55043` / 12 `8e8a97c` / 13 `54eee82` / 14 `4190ecf` / 15 `ff2674e` / 16 `b28fde9` / 17 `30091a1` / 18 `4224300` / 19 `4a36601` |
| 全量单测 | 基线 `263 / failures=1 / errors=4` → 终态 `794 / failures=1 / errors=4 / skipped=4`（**新增 531 例，失败集合逐项未变**） |
| 证据目录 | `build/iraf-24h/<id>/`（已 gitignore 的证据区，约 1346 个文件），逐步验收脚本 `run_acceptance.sh` 均可复跑 |

## 2. 分步结果汇总（状态 / 提交 / 关键数字）

数字一律**原样引用**该步证据文件中的实测值；每一步至少有一条负向路径与一条正向路径（或正向对照）。

### 2.1 SDK 与跨架构交付线（01–10）

| id | 标题 | 提交 | 关键实测数字 |
|---|---|---|---|
| 01 | 决策记录与文件边界 | `fb0453a` | 单测基线 `Ran 263 / failures=1 / errors=4`；文件边界（本窗口 vs 另一窗口）写入需求1 §13 |
| 02 | SDK 产物矩阵与 BoardProfile 声明 | `7d4d683` | 步骤单测 `Ran 20 / OK / EXIT=0`（6 负向 + 1 正向对照）；`unverified` 行数 e300=20 / firefly=20；矩阵内 `pypi.org` = **0** 次；`index_url=https://mirrors.aliyun.com/pypi/simple/` |
| 03 | `profile_check` 校验 BoardProfile | `1922fb9` | `--board e300` → `exit=2`（19 条中文原因：17 `unverified` + 2 `pending`）；`--allow-unverified` → `exit=0` 且 `verified=false`；`--baseline` 回归 piper(1114 B)/ur5(1155 B) stdout+stderr **逐字节 IDENTICAL** |
| 04 | ADR-0006 / ADR-0007 落地 | `661a003` | 两份 ADR 8 段结构与 ADR-0004 一致；内网 IP grep=**0**；`L1`=4 / `wheelhouse`=5 / `buildx`=2 / `unverified`=3 / `DEFERRED`=2（ADR-0006） |
| 05 | `iraf_sdk` 最小层与中文诊断 | `8122ef7` | 步骤单测 `Ran 55 / OK (skipped=4)`；错误码映射 **19** 个（IDL §5 表 9 / `src` 抛出点 15 / SDK 映射 19；两侧都有 7 / 仅 IDL 2 / 仅实现 9 / 仅 API 目录 1）；纯依赖探针 `mujoco`、`grpc`、`iraf_core`、`iraf_adapters`、`iraf.v1` 全部 `False`（无旁路） |
| 06 | `build_sdk.sh` 打包与 manifest | `a343b74` | `--dry-run` exit=0 且不落盘；完整构建 exit=0；`manifest artifacts=2` / 逐文件条目 **112**；连续两次构建 SHA-256 **逐位相同=True**；篡改负向 4 例 → `exit=4/4/4/2`；wheel 离线安装 `pip --no-index --no-deps --target` exit=0，导入探针 `{version:0.2.0, known_codes:19, heavy_loaded:[]}` |
| 07 | 离线 wheelhouse 抓取 | `de2ada3` | 真实抓取 **7 个 wheel / 47 620 905 字节**，全部通过 `cp310 + manylinux_2_28_aarch64` 标签校验；mujoco 3.13.0(exact) / numpy 2.2.6(compatible 2.17≤2.28) / grpcio 1.84.0 / protobuf 7.36.1(pure_python) / pyyaml 6.0.3 / pillow 12.3.0 / jsonschema 4.26.0；验收 15 项失败 **0**（空源/非法源/不可达版本/缺 `reject_platform_tags` → 2；混入 `win_arm64`+`macosx` → 4；目录不存在 → 2；用法错误 → 1） |
| 08 | 板级 bundle 与 `install.sh` | `74647d1` | 验收「通过 **37** 项，失败 **0** 项」；`bundle-refuse-unverified=2` / `bundle-allow-unverified=0`；篡改 `{outer,inner,extra,traversal,missing}` 全 = 4；`install-fail-rollback=3`（回滚后激活链接恢复旧目标 `0.1.0`、新版本目录已删除）；该步 bundle **47 441 421 B / 22 成员**（被步骤 09 重建取代，见下行） |
| 09 | `verify.sh` 与 `uninstall.sh` | `87a4b4b` | 重建 bundle 后验收「通过 **26** 项，失败 **0** 项」（修复前为 24/2）；当前 bundle **47 478 361 B / 26 成员 / sha256 `43f61c48aa9e7eefcf517481fb1c3bc08c9dde20f9d89212a51c82718c793573`**（本文实测复核一致）；`uninstall-apply=0`（`file_count_record=161 == file_count_actual=161`、`extras=[]`、`missing=[]`、`record_file_excluded=iraf-sdk-files.json`、`removed=4`）；二次卸载=1；`verify-service-not-running=5`（`service_state=SERVICE_NOT_RUNNING`、`verified=false`） |
| 10 | `deploy.sh` 双通道传输 | `eb24b67` | `media --output` exit=0、交接物 **4** 件、`sha256sum -c` → 「`iraf-board-e300-0.2.0-aarch64.tar.gz`: 成功」；`--dry-run` exit=0 且输出目录未创建；ssh 不可达 → `exit=2`（中文原因含 DEFERRED），可达端口正例对照 → exit=0；apply 到非 ssh 端口 → 3；验收「通过 **44** 项，失败 **0** 项」（首跑 39/1 系**断言写错**：`artifacts.length` 期望 4 实为 3，报告不登记自己） |

### 2.2 宇树场景与技能线（11–18）

| id | 标题 | 提交 | 关键实测数字 |
|---|---|---|---|
| 11 | 场景包契约与 `handoff_lab` 骨架 | `0e55043` | `scene_check --scene scenes/handoff_lab` → exit=0（`pending_refs=7 / pending_steps=8`，**是显式登记的缺口，不是通过**）；`--require-resolved-refs` → 5；`--require-model` → 4；`--scene /nope` → 6；单测 `Ran 42 / OK` |
| 12 | vendor 锁定 `unitree_mujoco` | `8e8a97c` | 锁校验 exit=0（**22 件 / 29 091 323 字节**，`mismatches=[] / unexpected_files=[] / writable_files=[]`）；`--expect-commit 1eb6642e…` exit=0；错锁点 exit=2；三模型编译实测 `nq=19 / nv=18 / nu=12 / nsensor=41 / ncam=0` ⇒ **厂商 MJCF 不含相机/雷达，传感器必须由场景构建器注入** |
| 13 | 场景构建器注入传感器与道具 | `54eee82` | `build_scene --scene scenes/handoff_lab --robot unitree_go2` → exit=0；生成模型 `nq 26 / nv 24 / nu 12 / ncam 1 / nsite 3 / nbody 20 / njnt 14`；厂商 `sha256=2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b` 与锁一致；`scene_check --model` → 0（`body=19 / camera=1 / geom=7 / site=3`）；`git status --short vendor/` = 0 行（厂商只读） |
| 14 | 场景生成验收与传感器统计 | `4190ecf` | 验收「通过 **14** 项，失败 **0** 项」；15 条判据全过、`failed_checks=[]`；`camera.fovy_rel_error=0.010427987255182231`（model vs 声明 `rel=0.0`）、`lidar.points=167`、`lidar.miss_fraction=0.5361111111111111`、`lidar.ground_fraction=0.07777777777777778`、`imu.acc.rel_error=2.0889954113422363e-16`；重建后模型 sha256 逐位一致 `3d7b07c3a21e342e2cf0380062c025ca2a6176220d936fbc6c5d85c6b6d59ca4`；台面命中距离与按声明解析求交最大偏差 **0.0** |
| 15 | Go2 loopback 站立/停止/状态 | `ff2674e` | 验收「通过 **16** 项，失败 **0** 项」；`height_mean_m=0.2801007638069918`、`height_std_m=7.351396160228674e-05`、`hold_seconds=7.499999999999341`（要求 6.0）、`max_attitude_error_deg=0.05779130222480239`、`max_tracking_error_rad=0.04473942561095967`、`ctrl_saturated_samples=0`、`stop.final_speed_mps=0.003964950131491022`；增益扫描证据 `pd-scan.json`（kp=80/150/250 达标；kp=400 与 kp=250/kd=12 发散、饱和 8431/8846 次）⇒ 声明 kp=150 / kd=4 |
| 16 | `QuadrupedAdapter` 与 `UnitreeGo2Adapter` | `b28fde9` | 验收「通过 **12** 项，失败 **0** 项」；stand 1.000 s → 100 周期 × 5 子步、`ctrl_saturated_samples=0`、基座高度 **+0.010216000 m**；stop 3 s → `base_height_drop_m=0.192798228`（力矩型松力躺倒，预期）；力矩上限独立复算 = 模型 `ctrlrange`（hip/thigh **±23.7**、calf **±45.43 N·m**）；`profile_check --quadruped` → 0（纯 JSON 摘要） |
| 17 | `stand/stop/locomote` 技能与拒绝路径 | `30091a1` | 验收「通过 **13** 项，失败 **0** 项」；成功路径 stand 墙钟 **0.39167014486156404 s**（位移 `0.00643820654999534 m ≤ 0.05`、倾角 `0.05779130222366524° ≤ 10.0`）、stop 墙钟 **0.5178679858800024 s**；拒绝用例 **7/7** 命中预期错误码（`IRAF-DEADLINE-EXCEEDED` / `IRAF-POLICY-DENIED` / `IRAF-UNAUTHENTICATED` / `IRAF-SKILL-PROVIDER-UNAVAILABLE(缺少能力: ['locomote'])` / `IRAF-QUADRUPED-COMMAND-REJECTED(越界速度 5 m/s > 0.5)` / `IRAF-EXECUTION-FAILED(关节 FL_calf_joint 目标 0.0 越出声明限位)` / `IRAF-IDEMPOTENCY-CONFLICT`）；能力回填 `capabilities [] -> [stand, stop]`（**仅在验收通过后**；`locomote` 不声明） |
| 18 | S2 脚本化场景入口 `scenario.py` | `4224300` | 验收「通过 **27** 项，失败 **0** 项」；stand 墙钟 **0.3899381598457694 s** / 仿真时间推进 **7.999999999999341 s** / 末速 `3.0387117402068175e-05 m/s`；stop 墙钟 **0.6374013870954514 s** / 推进 `3.000000000001002 s` / 末速 `0.0009179901099397197 m/s`；`fault_sensor_unavailable`（注入点 stand）`injected=true / fake_success=false / verified=true / unverified_faults=0`；负向：未支持故障类型·不可评测判据·参数非法 → 2；未声明 `robot.backend`（未接入）→ 3；未注入故障在 `--require-injected-faults` 下 → 5 |

### 2.3 图与文档（19–20）

| id | 标题 | 提交 | 关键实测数字 |
|---|---|---|---|
| 19 | 架构图与文档同步 | `4a36601` | 验收「通过 **9** 项，失败 **0** 项」（首跑 8/1 系**断言写错**：期望 unittest exit=0，而基线本身就是 exit=1）；两张 SVG **34 911 / 43 683 字节**，XML 可解析、根元素 `svg` 带 `viewBox`、`text` 元素 89 / 111 个、无内网 IP；`grep -c "deploy/sdk" docs/iraf-engineering-design.md` = **11**；`.dot` 重新生成与入库 SVG **逐字节一致**（`MISMATCH=0`）；文档一致性门禁 §8.1.1 段内 **17** 条仓库路径全部存在；负向对照 9 条全部证明门禁会失败 |
| 20 | 24 小时收尾汇总与缺口清单 | （本文所在提交） | 台账状态集合不含 `TODO/IN_PROGRESS`；全量单测 `Ran 794 / failures=1 / errors=4 / skipped=4`（失败集合与基线逐项一致） |

## 3. 交付产物清单（本轮新增，均入库）

| 类别 | 路径 | 说明 |
|---|---|---|
| SDK 层 | `src/iraf_sdk/{__init__,errors,client}.py` | 纯 Python、零第三方依赖、不 import `iraf_core/iraf_adapters`（子进程导入纯净性已证明） |
| 声明（集中式） | `config/sdk/package_matrix.yaml` + `.schema.json`、`profiles/boards/{e300,firefly_rk3588}.yaml` | 版本/标签/包清单/镜像源/路径集中声明；板卡字段实测前一律 `unverified` |
| 打包与安装（10 个文件） | `deploy/sdk/{build_sdk.sh,lib_manifest.py,fetch_wheelhouse.sh,check_wheel_tags.py,package_board_bundle.sh,lib_board_bundle.py,install.sh,verify.sh,uninstall.sh,lib_target_verify.py,deploy.sh,lib_deploy.py,iraf-sdk-board.service}` | 入口层 shell + 实现层 Python；全部带 `--dry-run` / 负向退出码 |
| 场景包 | `config/scene.schema.json`、`scenes/handoff_lab/{scene,baseline,scenario}.yaml` + `README.md` | 场景声明载体（决策 5.A 的 `scenes/`） |
| 场景与传感器 | `scripts/{scene_check,build_scene,verify_scene_sensors}.py`、`src/iraf_adapters/unitree/{scene_builder,scene_sensor_evidence}.py` | 厂商 MJCF 只读 + 按声明注入相机/雷达/道具/光照 |
| 本体与适配器 | `profiles/unitree_go2_mujoco.yaml`、`config/go2_loopback.yaml`、`src/iraf_adapters/unitree/{quadruped,unitree_go2,loopback}.py`、`scripts/{verify_go2_loopback,fetch_vendor_assets}.py` | 通用契约 + 机型实现两层；厂商资产按 `source-lock.json` 对账 |
| 技能层 | `skills/stand/`、`skills/locomote/`、`src/iraf_skills/quadruped.py`、`profiles/safety/quadruped_lab.yaml` | `stand`/`stop` 已验收；`locomote` 只拒绝、不声明（首期无步态控制器） |
| S2 执行器 | `scripts/scenario.py` | `list` / `run`，退出码 0/1/2/3/4/5；执行链不绕层 |
| 契约与 ADR | `docs/adr/0006-sdk-cross-arch-delivery.md`、`docs/adr/0007-unitree-scenario-interaction.md` | 跨架构交付分级 L1/L2/L3；宇树场景交互门禁 U1~U6 |
| 图 | `docs/diagrams/iraf-sdk-delivery.{dot,svg}`、`docs/diagrams/iraf-unitree-scene-stack.{dot,svg}` | 源 + SVG 同时入库，可由 `dot -Tsvg` 逐字节复现 |
| 回归用例 | `tests/unit/test_{sdk_errors,sdk_manifest,wheel_tags,board_profile_schema,profile_check_board,install_prechecks,target_verify_evidence,uninstall_scope,scene_schema,scene_builder_injection,scene_sensor_evidence,go2_loopback_contract,quadruped_adapter,scenario_runner}.py` | 14 个文件、6 929 行、新增 531 例 |
| 调试记录 | `docs/debug/2026-09-20-{sdk-cross-arch,sdk-error-code-contract-divergence,sdk-wheel-build-offline,wheelhouse-fetch-index-api,vendor-asset-lock-unitree,scene-builder-injection,quadruped-adapter-contract,quadruped-skills-and-rejection-paths,verify-uninstall-scope-gates}.md` | 9 份，中文，含症状→证据链→根因→修法→复跑命令 |

## 4. 回归汇总（全量单测）

命令：`PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit`（解释器必须显式 `/usr/bin/python3` 3.10.12，cron 默认 `python3` 指向 Hermes venv 3.11.15，缺 mujoco/numpy/grpc，会产生 29 个 `ModuleNotFoundError` 环境噪声）。

| 里程碑 | Ran | failures | errors | skipped |
|---|---|---|---|---|
| 战役基线（步骤 01） | 263 | 1 | 4 | 0 |
| 05 / 06 / 07 | 360 / 403 / 460 | 1 | 4 | 4 |
| 08 / 09 / 10 | 510 / 528 / 528 | 1 | 4 | 4 |
| 11 / 12 / 13 / 14 | 570 / 602 / 622 / 650 | 1 | 4 | 4 |
| 15 / 16 / 17 / 18 / 19 | 690 / 740 / 764 / 794 / 794 | 1 | 4 | 4 |
| **终态（步骤 20）** | **794** | **1** | **4** | **4** |

- **新增 531 例**（794 − 263），**失败/错误集合逐项未变**：4 项导入 `ERROR`（`test_agentos_bridge` / `test_coding_worker_contract` / `test_runtime_grpc` / `test_runtime_http_health`）+ 1 项既有 `FAIL`（`test_vision_processing.test_depth_projection_and_invalid_filter`）。这是**战役前的既有缺陷**，本轮**未**把它并进基线、也未通过放宽门禁掩盖。
- `skipped=4` 为 gRPC 消息层用例（本机 protobuf 4.25.7 < 生成 stub 要求的 5.29），跳过原因原样写在验收输出中，不伪造通过。
- **门禁口径修正**：全量单测自身退出码在战役基线里就是 1，因此步骤 19 起 harness **不拿退出码当判据**，改为逐项比对「Ran 数 + failures/errors/skipped + 失败测试名集合」。

## 5. 诚实缺口清单（未验证 / 未实现 / 延后）

### 5.1 目标端与真机（`DEFERRED`，板卡不在场）

| 缺口 | 状态 | 说明 |
|---|---|---|
| 08 目标端真实安装 | `DEFERRED` | 未做任何 aarch64 真机安装；dry-run 报告自标 `mode=rehearsal` / `evidence_scope=rehearsal_only` / `simulation=true`（仅沙箱） |
| 09 目标端 `/health`、systemd、事件库写入 | `DEFERRED` | `evidence_scope=rehearsal_only` / `verified=false`；本机 8765 端口确有开发仿真服务（HTTP 200），该通过属**「本机开发服务证据」**，不得表述为目标端健康检查通过；"服务未启动"分支用关闭端口 `127.0.0.1:1` 确定性复现 |
| 10 ssh 真机部署全链路 | `DEFERRED` | 只在 `--dry-run` 下验证命令构造与可达性门禁；除"目标不可达"外的失败路径半成品报告未落 `json-out` |
| AgentOS 联通 / 真机 HIL | `DEFERRED` | 不在本轮范围 |

### 5.2 环境与依赖缺口

| 缺口 | 实测证据 | 关闭条件 |
|---|---|---|
| gRPC SDK 通道未联调 | 本机无 `grpcio`；`build/generated/python` 的 stub 需 `protobuf ≥ 5.29`，本机 4.25.7 ⇒ 4 例 SKIP | 仿真机/目标端装齐依赖后用例自动执行（无代码改动）；步骤 07 已把 `grpcio 1.84.0` / `protobuf 7.36.1` 抓进 wheelhouse |
| **wheelhouse 传递依赖不闭合（需决策）** | 7 个 wheel 共 10 条 `Requires-Dist`，仅 **1** 条覆盖，**9** 条缺失（absl-py / attrs / etils / glfw / jsonschema-specifications / pyopengl / referencing / rpds-py / typing-extensions） | 三选项：(A) 人工盘点后补齐矩阵声明清单；(B) 脚本从 METADATA 递归解析产出**候选清单**供人工确认后回填；(C) 不处理，`install.sh` 目标端缺依赖即 fail-closed 停止 |
| wheelhouse 版本未 pin | 矩阵未声明 `pins`，与本机已装版本 **4 项漂移**（mujoco 3.13.0 vs 3.3.3、protobuf 7.36.1 vs 4.25.7、pyyaml 6.0.3 vs 5.4.1、pillow 12.3.0 vs 9.0.1） | 需要物理/运行期版本一致性时在矩阵 `pins` 声明（能力已实现并有单测），重跑抓取 |
| 无 `docker buildx` 且 `registry-1.docker.io` 不可达 | `docker buildx ls` 无 buildx；镜像源不可达 | 提供镜像源或本地 registry；**OCI 多架构路线（L2）本战役不做** |
| 无 aarch64 交叉工具链 + 板卡不在场 | 无 `aarch64-linux-gnu-gcc` | L3 板级原生包不可执行 |

### 5.3 未实现 / 未提交决策

| 项 | 状态 |
|---|---|
| 人形（G1/H1 等）运动能力 | **未实现**：决策 4.B 只允许 `static_model_only` + `evidence_level=仅模型`，且不得出现在任何 Skill 步骤（已落成硬门禁） |
| 语音 / Studio | **未实现**：决策 7.A 只固定边界（语音仅 Studio 输入法），本轮不引入语音 |
| S1 命令式交互、S3 遥操作 | **未实现**：决策 6.A 先 S2 脚本化；S3 本轮不做 |
| 厂商 22 件资产是否入库 | **待人工决策**：(A) 只入库锁+BOM+校验器+重取入口（本轮采用）/ (B) 连 22 件资产一起入库（+约 29 MB） |
| 厂商资产逐项许可审查与 SDK/固件授权收集 | **未完成**：`license.review_state=pending_per_asset_review` 为 fail-closed 分发门禁 |
| 根级两张总图无 `.dot` 源 | **欠债已登记（M1.7）**：`agentos机器人应用框架-更新版.svg`（11 338 B）与 `IRAF详细技术架构图.svg`（16 918 B）；补源前**不纳入**「每个 svg 必须有同名 dot」的 CI 门禁（避免恒红） |
| 既有文档内网 IP（历史遗留） | `docs/manipulator-grasp-engineering-plan.md`、`docs/iraf-implementation-plan.md`、`docs/project-progress.md`、`docs/24h-orchestration.md` 仍有内部地址；**不属本窗口**，本文只追加新内容且新增文本内网 IP 命中 0，历史暴露交框架窗口决定 |
| `emit_backend_config` 兼容性范围 | 仅指**键集合兼容**，四足场景报告的 `target_id` / `gripper` / `vision` 显式为 `null`，**不得**据此装配操作后端 |

### 5.4 能力边界（防止过度解读）

- Go2 只证明「声明位形下站得住、能进入静止、状态可读、控制权与闭锁生效」，**不代表**步态、导航、停靠能力。
- `stop.mode=torque_zero_release` 是**松力停机**：力矩型执行器松力后失能躺倒（报告显式给 `collapsed=true`）。
- 重力前馈取自由浮动基的 `qfrc_bias`（`qvel=0`），**不含地面约束反力**，它是"消除稳态下垂"而不是支撑力矩。
- 姿态判据取「相对竖直的倾斜角」（不含偏航）；把偏航算进去是首版自身缺陷，已修正。
- 雷达只做**光线投射统计**，无点云消息、无时序、无噪声模型；本场景无平衡控制器，`settle.steps=10` 仅证明采样准静态（躯干漂移 `0.001879794568108979 m`）。
- 相机渲染替身（测试用 stub）在报告 `render.implementation` 留痕；真实离屏渲染由步骤 14 验收入口证明（EGL 640×480 非全黑 0.5503、`resolution_diff_px=0`）。
- 「退出码 0 ≠ 目标端可用」；`pip wheel` 曾出现「绿灯 + `UNKNOWN-0.0.0` 空包」，已被硬门禁拒绝。

## 6. 下一阶段建议（按优先级，附建议的第一步）

1. **闭合 wheelhouse 传递依赖缺口（最高优先，阻断目标端安装）** —— 建议第一步：用脚本从 7 个 wheel 的 `METADATA` 递归解析出候选依赖清单（**只产出候选，不私自扩清单**），人工确认后回填 `config/sdk/package_matrix.yaml`，再重跑 `fetch_wheelhouse.sh` + 目录级标签校验，使「声明清单 == 可安装 wheelhouse」成立。
2. **板卡到位后执行已就绪的目标端验收** —— 三种形态的脚本与检查清单已备好：`install.sh` → `verify.sh`（`/health`、事件库、板卡门禁）→ `deploy.sh --transport ssh`；建议第一步：在板卡上跑 `verify.sh --bundle …` 并回填 `profiles/boards/e300.yaml` 的 17 项 `unverified`（每项都要有实测证据，否则维持 `unverified`）。
3. **从 L1 推进到 L2/L3 的前置条件补齐** —— 需要本地 registry 或可达镜像源（L2 多架构 OCI）+ aarch64 交叉工具链（L3 板级原生包）；建议第一步：先只做**前置条件探测**并写进 ADR-0006 的后续段，避免在条件不具备时承诺交付等级。
4. **Go2 步态/导航能力的独立验收** —— 首期只交付 `stand`/`stop`；`locomote` 已保留「拒绝路径」作为能力门禁证据；建议第一步：先出 ADR 并明确步态控制器的安全边界（速度/倾角/工作空间上限只允许在安全策略声明一次），再落 `unitree_mujoco` 的低层接口适配。
5. **把 S2 判据从 3 条扩到有依据的更多条** —— 现有 `min_stable_hold_s` / `max_speed_m_s` / `timeout_s` 三条均有评测依据；建议第一步：为「故障后安全动作」「待交付步骤跳过」补充可评测量（同样要求"无依据即失败"，不得给默认值），并把 `--require-injected-faults` 纳入 CI。

**如果只能做一件**：选第 1 条（wheelhouse 依赖闭包），因为它是唯一同时阻断「离线安装可用性」和「目标端验收」的硬缺口，且完全可以在 x86 侧取证完成。

## 7. 复盘：本轮暴露的流程问题（如实记录，不美化）

1. **两次「额度耗尽续跑轮」（步骤 08、09、18）**：工具调用额度耗尽时验收已全绿但**未提交、未回写台账**，调度器把整轮判为 `error`，白跑一轮并留下未提交产物。已固化为纪律：**先让产物落盘并跑通一条最关键验收 → 立刻提交 → 回写台账/日志/tick-report → 有余力再补深度复核**；自写验收 harness 的步骤尤其如此。
2. **「修复晚于证据」的时效性陷阱**（步骤 09）：修复补到写入侧后，验收证据与 bundle 都早于该修复，旧结论不可用 ⇒ 必须重新组装产物并重跑，且**保留首跑失败输出**（`tee` 覆盖会毁掉最有价值的证据）。
3. **自伤型测试/门禁缺陷占首跑失败的大多数**（断言写错：步骤 10 的 `artifacts.length`、步骤 19 的 unittest 退出码；门禁恒通过：步骤 11 的 `_expand_mjcf` 只平铺根子元素、步骤 19 的路径正则锚定整段反引号、步骤 16 的 `\bdds\b`）。识别信号统一为一句话：**门禁绿、计数 0** 或 **门禁报"找不到"而计数全为 0** ⇒ 先怀疑门禁，不要怀疑产物。
4. **未放宽任何既有门禁**：本轮所有"不达标"都以修改**声明**或**夹具**收口（如 PD 增益、`max_speed_m_s` 愿望值、`f03_safe_hold` 的非法参数），并逐条在 commit body 与调试记录中说明理由。
5. **另一窗口共存**：提交前均以 `git status --short` 确认只包含本步声明路径；`examples/demo3_arm/`、`scripts/view_mujoco.py`、`src/iraf_adapters/mujoco/viewer_runner.py`、`config/*_baseline.yaml` 全程未触碰。
