# iraf-24h 证据索引（收尾模式生成）

- 生成方式：`/usr/bin/python3 build/iraf-24h/21/build_evidence_index.py`（核对脚本在 gitignore 证据区，不随仓库分发；解释器口径见 `plans/iraf-24h/00-执行规则.md` §5.6）
- 数据来源：`plans/iraf-24h/00-STATUS.json`（唯一权威台账）；核对基准提交 `5b76c8d`
- 核对结果：步骤 20 个，状态 {'DONE': 20}；台账声明证据路径 311 条，盘上存在 311 条，缺失 0 条；提交问题 0 条

> `build/` 已在 `.gitignore`（证据区），索引里的证据路径**不随仓库分发**；需要复核证据时请在原仿真机上按路径读取，或用各步骤的 `run_acceptance.sh` 重跑生成。

> 本文由脚本生成：只要台账 `evidence` 声明与盘上事实不一致，脚本就退 3 且本文不会写出「0 缺失」；因此「0 缺失」本身是一条可复跑的判据，不是人工结论。

## 1. 步骤索引（状态 / 提交 / 证据）

| id | 标题 | 状态 | 提交 | 提交在 HEAD 历史 | 提交改动路径数 | 证据（存在/声明） |
|---|---|---|---|---|---|---|
| 01 | 决策记录与文件边界 | DONE | `fb0453a` | 是 | 5 | 4/4 |
| 02 | SDK 产物矩阵与 BoardProfile 声明 | DONE | `7d4d683` | 是 | 5 | 7/7 |
| 03 | profile_check 校验 BoardProfile | DONE | `1922fb9` | 是 | 2 | 9/9 |
| 04 | ADR-0006 / ADR-0007 落地 | DONE | `661a003` | 是 | 2 | 6/6 |
| 05 | iraf_sdk 最小层与中文诊断 | DONE | `8122ef7` | 是 | 6 | 13/13 |
| 06 | build_sdk.sh 打包与 manifest | DONE | `a343b74` | 是 | 4 | 15/15 |
| 07 | 离线 wheelhouse 抓取 | DONE | `de2ada3` | 是 | 4 | 24/24 |
| 08 | 板级 bundle 与 install.sh | DONE | `74647d1` | 是 | 7 | 13/13 |
| 09 | verify.sh 与 uninstall.sh | DONE | `87a4b4b` | 是 | 9 | 22/22 |
| 10 | deploy.sh 双通道传输 | DONE | `eb24b67` | 是 | 3 | 17/17 |
| 11 | 场景包契约与 handoff_lab 骨架 | DONE | `0e55043` | 是 | 7 | 11/11 |
| 12 | vendor 锁定 unitree_mujoco | DONE | `8e8a97c` | 是 | 7 | 13/13 |
| 13 | 场景构建器注入传感器与道具 | DONE | `54eee82` | 是 | 11 | 24/24 |
| 14 | 场景生成验收与传感器统计 | DONE | `4190ecf` | 是 | 6 | 12/12 |
| 15 | Go2 loopback 站立/停止/状态 | DONE | `ff2674e` | 是 | 7 | 20/20 |
| 16 | QuadrupedAdapter 与 UnitreeGo2Adapter | DONE | `b28fde9` | 是 | 8 | 23/23 |
| 17 | stand/stop/locomote 技能与拒绝路径 | DONE | `30091a1` | 是 | 18 | 20/20 |
| 18 | S2 脚本化场景入口 scenario.py | DONE | `4224300` | 是 | 5 | 20/20 |
| 19 | 架构图与文档同步 | DONE | `4a36601` | 是 | 7 | 16/16 |
| 20 | 24 小时收尾汇总与缺口清单 | DONE | `6b28a82` | 是 | 2 | 22/22 |

## 2. 台账声明的证据文件（按步骤，逐条列出）

路径均为仓库相对路径，位于 gitignore 的证据区 `build/`（另有少量 `docs/` 复核脚本）。

### 步骤 01 — 决策记录与文件边界

- `build/iraf-24h/01/status.txt`
- `build/iraf-24h/01/unittest-baseline.txt`
- `build/iraf-24h/01/commands.log`
- `build/iraf-24h/01/commit-plan.txt`
- 小计：声明 4 条，盘上存在 4 条，缺失 0 条

### 步骤 02 — SDK 产物矩阵与 BoardProfile 声明

- `build/iraf-24h/02/unittest.txt`
- `build/iraf-24h/02/unittest-usr.txt`
- `build/iraf-24h/02/acceptance.txt`
- `build/iraf-24h/02/commands.log`
- `build/iraf-24h/02/commit-plan.txt`
- `build/iraf-24h/02/commit-msg.txt`
- `build/iraf-24h/02/unittest-full-usr.txt`
- 小计：声明 7 条，盘上存在 7 条，缺失 0 条

### 步骤 03 — profile_check 校验 BoardProfile

- `build/iraf-24h/03/acceptance.txt`
- `build/iraf-24h/03/commands.log`
- `build/iraf-24h/03/commit-plan.txt`
- `build/iraf-24h/03/commit-msg.txt`
- `build/iraf-24h/03/unittest-full.txt`
- `build/iraf-24h/03/line-width.txt`
- `build/iraf-24h/03/env_probe.txt`
- `build/iraf-24h/03/check_stdout_json.py`
- `build/iraf-24h/03/run_acceptance.sh`
- 小计：声明 9 条，盘上存在 9 条，缺失 0 条

### 步骤 04 — ADR-0006 / ADR-0007 落地

- `build/iraf-24h/04/adr-check.txt`
- `build/iraf-24h/04/run_acceptance.sh`
- `build/iraf-24h/04/commit-plan.txt`
- `build/iraf-24h/04/commit-msg.txt`
- `build/iraf-24h/04/commands.log`
- `build/iraf-24h/04/tick-report.md`
- 小计：声明 6 条，盘上存在 6 条，缺失 0 条

### 步骤 05 — iraf_sdk 最小层与中文诊断

- `build/iraf-24h/05/acceptance.txt`
- `build/iraf-24h/05/unittest-full.txt`
- `build/iraf-24h/05/failures.txt`
- `build/iraf-24h/05/commands.log`
- `build/iraf-24h/05/status.txt`
- `build/iraf-24h/05/commit-plan.txt`
- `build/iraf-24h/05/commit-msg.txt`
- `build/iraf-24h/05/error-code-divergence.txt`
- `build/iraf-24h/05/env_probe.py`
- `build/iraf-24h/05/purity_probe.py`
- `build/iraf-24h/05/grpc-message-probe.txt`
- `build/iraf-24h/05/run_acceptance.sh`
- `build/iraf-24h/05/tick-report.md`
- 小计：声明 13 条，盘上存在 13 条，缺失 0 条

### 步骤 06 — build_sdk.sh 打包与 manifest

- `build/iraf-24h/06/build-sdk.txt`
- `build/iraf-24h/06/preflight.json`
- `build/iraf-24h/06/manifest-run1.json`
- `build/iraf-24h/06/manifest-run2.json`
- `build/iraf-24h/06/run_acceptance.sh`
- `build/iraf-24h/06/check_manifest.py`
- `build/iraf-24h/06/tamper.py`
- `build/iraf-24h/06/wheel_install_check.py`
- `build/iraf-24h/06/compare_repro.py`
- `build/iraf-24h/06/import_probe.py`
- `build/iraf-24h/06/toolchain-probe.txt`
- `build/iraf-24h/06/pystub-import-probe.txt`
- `build/iraf-24h/06/unittest-full.txt`
- `build/iraf-24h/06/commit-plan.txt`
- `build/iraf-24h/06/commit-msg.txt`
- 小计：声明 15 条，盘上存在 15 条，缺失 0 条

### 步骤 07 — 离线 wheelhouse 抓取

- `build/iraf-24h/07/wheelhouse.txt`
- `build/iraf-24h/07/fetch.json`
- `build/iraf-24h/07/dry-run-resolve.json`
- `build/iraf-24h/07/dry-run-resolve.txt`
- `build/iraf-24h/07/acceptance.txt`
- `build/iraf-24h/07/unittest-wheel-tags.txt`
- `build/iraf-24h/07/unittest-full.txt`
- `build/iraf-24h/07/requires-gap.json`
- `build/iraf-24h/07/requires-gap.txt`
- `build/iraf-24h/07/repro-generator-truthiness.txt`
- `build/iraf-24h/07/env-probe.txt`
- `build/iraf-24h/07/index-probe.txt`
- `build/iraf-24h/07/run_acceptance.sh`
- `build/iraf-24h/07/make_broken_matrix.py`
- `build/iraf-24h/07/analyze_requires.py`
- `build/iraf-24h/07/env_probe.py`
- `build/iraf-24h/07/index_probe.py`
- `build/iraf-24h/07/repro-generator-truthiness.py`
- `build/iraf-24h/07/commit-plan.txt`
- `build/iraf-24h/07/commit-msg.txt`
- `build/iraf-24h/07/commands.log`
- `build/iraf-24h/07/tick-report.md`
- `build/wheelhouse/aarch64-manylinux_2_28-cp310/wheelhouse.json`
- `build/wheelhouse/aarch64-manylinux_2_28-cp310/verify-tags.json`
- 叙述性条目（非文件名，显式登记、不计入分子分母）：`broken-matrix-*.yaml`；`7 个 wheel`
- 小计：声明 24 条，盘上存在 24 条，缺失 0 条

### 步骤 08 — 板级 bundle 与 install.sh

- `build/iraf-24h/08/acceptance.txt`
- `build/iraf-24h/08/acceptance-rerun.txt`
- `build/iraf-24h/08/unittest-full.txt`
- `build/iraf-24h/08/cases/`
- `build/iraf-24h/08/run_acceptance.sh`
- `build/iraf-24h/08/check_json.py`
- `build/iraf-24h/08/tamper_bundle.py`
- `build/iraf-24h/08/clean_stage.py`
- `build/iraf-24h/08/dry-run-1.json`
- `build/iraf-24h/08/preflight-after-delivery.json`
- `build/iraf-24h/08/commit-plan.txt`
- `build/iraf-24h/08/commit-msg.txt`
- `build/iraf-24h/08/tick-report.md`
- 小计：声明 13 条，盘上存在 13 条，缺失 0 条

### 步骤 09 — verify.sh 与 uninstall.sh

- `build/iraf-24h/09/acceptance.txt`
- `build/iraf-24h/09/acceptance-rerun.txt`
- `build/iraf-24h/09/unittest-full.txt`
- `build/iraf-24h/09/unittest-uninstall-scope-v2.txt`
- `build/iraf-24h/09/cases/`
- `build/iraf-24h/09/run_acceptance.sh`
- `build/iraf-24h/09/check_evidence.py`
- `build/iraf-24h/09/tamper_bundle.py`
- `build/iraf-24h/09/clean_stage.py`
- `build/iraf-24h/09/inject_extra.py`
- `build/iraf-24h/09/bundle-build-rebuild.txt`
- `build/iraf-24h/09/verify-evidence-dryrun.json`
- `build/iraf-24h/09/verify-evidence-installed.json`
- `build/iraf-24h/09/verify-evidence-not-running.json`
- `build/iraf-24h/09/install-dry-run.json`
- `build/iraf-24h/09/install-dry-run-probe.json`
- `build/iraf-24h/09/uninstall-dryrun.json`
- `build/iraf-24h/09/uninstall-apply.json`
- `build/iraf-24h/09/commit-plan.txt`
- `build/iraf-24h/09/commit-msg.txt`
- `build/iraf-24h/09/tick-report.md`
- `build/iraf-24h/09/commands.log`
- 小计：声明 22 条，盘上存在 22 条，缺失 0 条

### 步骤 10 — deploy.sh 双通道传输

- `build/iraf-24h/10/acceptance.txt`
- `build/iraf-24h/10/acceptance-rerun.txt`
- `build/iraf-24h/10/acceptance-run1-fail.txt`
- `build/iraf-24h/10/deploy.txt`
- `build/iraf-24h/10/run_acceptance.sh`
- `build/iraf-24h/10/check_report.py`
- `build/iraf-24h/10/make_fake_bundles.py`
- `build/iraf-24h/10/cases/`
- `build/iraf-24h/10/tmp/`
- `build/iraf-24h/10/unittest-full.txt`
- `build/iraf-24h/10/commit-plan.txt`
- `build/iraf-24h/10/commit-msg.txt`
- `build/iraf-24h/10/tick-report.md`
- `build/iraf-24h/10/ssh-unreachable.json`
- `build/iraf-24h/10/ssh-unreachable.stdout.json`
- `build/iraf-24h/10/sha256sum.txt`
- `build/iraf-24h/10/deploy-report.json`
- 叙述性条目（非文件名，显式登记、不计入分子分母）：`README-安装.md`；`media-*.json`
- 小计：声明 17 条，盘上存在 17 条，缺失 0 条

### 步骤 11 — 场景包契约与 handoff_lab 骨架

- `build/iraf-24h/11/scene-check.json`
- `build/iraf-24h/11/scene-check.stdout.json`
- `build/iraf-24h/11/scene-check.stderr.txt`
- `build/iraf-24h/11/strict-mode.stderr.txt`
- `build/iraf-24h/11/require-model.stderr.txt`
- `build/iraf-24h/11/unittest-scene-schema.txt`
- `build/iraf-24h/11/unittest-full.txt`
- `build/iraf-24h/11/commit-plan.txt`
- `build/iraf-24h/11/commit-msg.txt`
- `build/iraf-24h/11/tick-report.md`
- `build/iraf-24h/11/commands.log`
- 小计：声明 11 条，盘上存在 11 条，缺失 0 条

### 步骤 12 — vendor 锁定 unitree_mujoco

- `build/iraf-24h/12/acceptance.txt`
- `build/iraf-24h/12/vendor-lock.txt`
- `build/iraf-24h/12/fresh-clone.txt`
- `build/iraf-24h/12/vendor-lock-require-readonly.json`
- `build/iraf-24h/12/mjcf-load-probe.json`
- `build/iraf-24h/12/check_lock_numbers.py`
- `build/iraf-24h/12/check_fresh_clone.py`
- `build/iraf-24h/12/run_acceptance.sh`
- `build/iraf-24h/12/unittest-full.txt`
- `build/iraf-24h/12/commit-plan.txt`
- `build/iraf-24h/12/commit-msg.txt`
- `build/iraf-24h/12/tick-report.md`
- `build/iraf-24h/12/commands.log`
- 小计：声明 13 条，盘上存在 13 条，缺失 0 条

### 步骤 13 — 场景构建器注入传感器与道具

- `build/iraf-24h/13/acceptance.txt`
- `build/iraf-24h/13/build-scene.txt`
- `build/iraf-24h/13/build-scene.stderr.txt`
- `build/iraf-24h/13/check_report.py`
- `build/iraf-24h/13/run_acceptance.sh`
- `build/iraf-24h/13/scene-check-model.json`
- `build/iraf-24h/13/scene-check-model.stderr.txt`
- `build/iraf-24h/13/scene-check-after.json`
- `build/iraf-24h/13/negative-unknown-robot.txt`
- `build/iraf-24h/13/negative-missing-scene.txt`
- `build/iraf-24h/13/unittest-full.txt`
- `build/iraf-24h/13/failures.txt`
- `build/iraf-24h/13/env-probe.txt`
- `build/iraf-24h/13/trunk-geom.txt`
- `build/iraf-24h/13/go2-model-facts.json`
- `build/iraf-24h/13/probe_env.py`
- `build/iraf-24h/13/probe_trunk.py`
- `build/iraf-24h/13/probe_go2_model.py`
- `build/iraf-24h/13/commit-plan.txt`
- `build/iraf-24h/13/commit-msg.txt`
- `build/iraf-24h/13/tick-report.md`
- `build/iraf-24h/13/commands.log`
- `build/scenes/handoff_lab/handoff_lab.xml`
- `build/scenes/handoff_lab/handoff_lab.json`
- 小计：声明 24 条，盘上存在 24 条，缺失 0 条

### 步骤 14 — 场景生成验收与传感器统计

- `build/iraf-24h/14/acceptance.txt`
- `build/iraf-24h/14/cases/sensors.txt`
- `build/iraf-24h/14/cases/check-report.txt`
- `build/iraf-24h/14/check_report.py`
- `build/iraf-24h/14/run_acceptance.sh`
- `build/iraf-24h/14/report-pretty.json`
- `build/iraf-24h/14/cases/`
- `build/iraf-24h/14/unittest-full.txt`
- `build/iraf-24h/14/commit-plan.txt`
- `build/iraf-24h/14/commit-msg.txt`
- `build/iraf-24h/14/tick-report.md`
- `build/acceptance/handoff-lab-scene-sensors/report.json`
- 叙述性条目（非文件名，显式登记、不计入分子分母）：`unittest-runs*.txt`；`probe*.txt`
- 小计：声明 12 条，盘上存在 12 条，缺失 0 条

### 步骤 15 — Go2 loopback 站立/停止/状态

- `build/iraf-24h/15/acceptance.txt`
- `build/iraf-24h/15/loopback.txt`
- `build/iraf-24h/15/check-report.txt`
- `build/iraf-24h/15/check_report.py`
- `build/iraf-24h/15/make_variants.py`
- `build/iraf-24h/15/run_acceptance.sh`
- `build/iraf-24h/15/cases/`
- `build/iraf-24h/15/pd-scan.json`
- `build/iraf-24h/15/probe-api.txt`
- `build/iraf-24h/15/diag_kit.py`
- `build/iraf-24h/15/kit_settle_probe.py`
- `build/iraf-24h/15/unittest-step15.txt`
- `build/iraf-24h/15/unittest-full.txt`
- `build/iraf-24h/15/scene-check.json`
- `build/iraf-24h/15/variants.txt`
- `build/iraf-24h/15/tight-height.txt`
- `build/iraf-24h/15/commit-plan.txt`
- `build/iraf-24h/15/commit-msg.txt`
- `build/iraf-24h/15/tick-report.md`
- `build/acceptance/go2-loopback/report.json`
- 小计：声明 20 条，盘上存在 20 条，缺失 0 条

### 步骤 16 — QuadrupedAdapter 与 UnitreeGo2Adapter

- `build/iraf-24h/16/acceptance.txt`
- `build/iraf-24h/16/adapter.txt`
- `build/iraf-24h/16/adapter.stderr.txt`
- `build/iraf-24h/16/unittest-step16.run1.txt`
- `build/iraf-24h/16/unittest-step16.run2.txt`
- `build/iraf-24h/16/unittest-step16.run3.txt`
- `build/iraf-24h/16/unittest-full.txt`
- `build/iraf-24h/16/unittest-sdk-errors.txt`
- `build/iraf-24h/16/quadruped-check.run1.json`
- `build/iraf-24h/16/quadruped-check.run1.stderr.txt`
- `build/iraf-24h/16/profile-check-ur5.json`
- `build/iraf-24h/16/profile-check-piper.json`
- `build/iraf-24h/16/profile-check-noargs.err`
- `build/iraf-24h/16/error-code-scan.txt`
- `build/iraf-24h/16/cases/`
- `build/iraf-24h/16/run_acceptance.sh`
- `build/iraf-24h/16/probe_adapter_real.py`
- `build/iraf-24h/16/scan_error_codes.py`
- `build/iraf-24h/16/patch_factory.py`
- `build/iraf-24h/16/commit-plan.txt`
- `build/iraf-24h/16/commit-msg.txt`
- `build/iraf-24h/16/tick-report.md`
- `build/iraf-24h/16/commands.log`
- 小计：声明 23 条，盘上存在 23 条，缺失 0 条

### 步骤 17 — stand/stop/locomote 技能与拒绝路径

- `build/iraf-24h/17/acceptance.txt`
- `build/iraf-24h/17/skills.txt`
- `build/iraf-24h/17/skills-run1.txt`
- `build/iraf-24h/17/skills-run2.txt`
- `build/iraf-24h/17/skills-run3.txt`
- `build/iraf-24h/17/report-snapshot.json`
- `build/iraf-24h/17/scene-check.json`
- `build/iraf-24h/17/scene-check-model.json`
- `build/iraf-24h/17/loopback-rerun.json`
- `build/iraf-24h/17/sensors-rerun.json`
- `build/iraf-24h/17/profile-check-quadruped.json`
- `build/iraf-24h/17/unittest-full2.txt`
- `build/iraf-24h/17/build-scene.txt`
- `build/iraf-24h/17/run_acceptance.sh`
- `build/iraf-24h/17/make_broken_configs.py`
- `build/iraf-24h/17/cases/`
- `build/iraf-24h/17/commit-plan.txt`
- `build/iraf-24h/17/commit-msg.txt`
- `build/iraf-24h/17/tick-report.md`
- `build/acceptance/go2-skills/report.json`
- 小计：声明 20 条，盘上存在 20 条，缺失 0 条

### 步骤 18 — S2 脚本化场景入口 scenario.py

- `build/iraf-24h/18/acceptance.txt`
- `build/iraf-24h/18/acceptance-rerun.txt`
- `build/iraf-24h/18/scenario.txt`
- `build/iraf-24h/18/scenario-run1-before-fix.txt`
- `build/iraf-24h/18/collect_scenario_evidence.sh`
- `build/iraf-24h/18/run_acceptance.sh`
- `build/iraf-24h/18/check_report.py`
- `build/iraf-24h/18/make_broken_scenarios.py`
- `build/iraf-24h/18/probe-step-metrics.json`
- `build/iraf-24h/18/probe_stand.py`
- `build/iraf-24h/18/list-before.json`
- `build/iraf-24h/18/unittest-step18.txt`
- `build/iraf-24h/18/unittest-full.txt`
- `build/iraf-24h/18/unittest-full-rerun.txt`
- `build/iraf-24h/18/cases/`
- `build/iraf-24h/18/tmp/`
- `build/iraf-24h/18/commit-plan.txt`
- `build/iraf-24h/18/commit-msg.txt`
- `build/iraf-24h/18/tick-report.md`
- `build/acceptance/handoff_lab/stand_stop/report.json`
- 小计：声明 20 条，盘上存在 20 条，缺失 0 条

### 步骤 19 — 架构图与文档同步

- `build/iraf-24h/19/acceptance.txt`
- `build/iraf-24h/19/acceptance-run1-fail.txt`
- `build/iraf-24h/19/acceptance-rerun.txt`
- `build/iraf-24h/19/check_svg.py`
- `build/iraf-24h/19/check_docs.py`
- `build/iraf-24h/19/check_dot_repro.py`
- `build/iraf-24h/19/check_unittest.py`
- `build/iraf-24h/19/neg_control.py`
- `build/iraf-24h/19/docs_gates.py`
- `build/iraf-24h/19/run_acceptance.sh`
- `build/iraf-24h/19/dot-repro.txt`
- `build/iraf-24h/19/unittest-full.txt`
- `build/iraf-24h/19/commit-plan.txt`
- `build/iraf-24h/19/commit-msg.txt`
- `build/iraf-24h/19/commands.log`
- `build/iraf-24h/19/tick-report.md`
- 小计：声明 16 条，盘上存在 16 条，缺失 0 条

### 步骤 20 — 24 小时收尾汇总与缺口清单

- `build/iraf-24h/20/acceptance.txt`
- `build/iraf-24h/20/acceptance-run1-fail.txt`
- `build/iraf-24h/20/check_ledger.py`
- `build/iraf-24h/20/check_ledger.txt`
- `build/iraf-24h/20/check_ledger-negative.txt`
- `build/iraf-24h/20/make_broken_ledger.py`
- `build/iraf-24h/20/ledger-todo.json`
- `build/iraf-24h/20/ledger-bad-commit.json`
- `build/iraf-24h/20/check_unittest.py`
- `build/iraf-24h/20/check_unittest.txt`
- `build/iraf-24h/20/check_docs_gates.txt`
- `build/iraf-24h/20/ip-gate.txt`
- `build/iraf-24h/20/git-status.txt`
- `build/iraf-24h/20/unittest-full.txt`
- `build/iraf-24h/20/summary.txt`
- `build/iraf-24h/20/run_acceptance.sh`
- `build/iraf-24h/20/commit-plan.txt`
- `build/iraf-24h/20/commit-msg.txt`
- `build/iraf-24h/20/commit-msg-fix.txt`
- `build/iraf-24h/20/commit-msg-plans.txt`
- `build/iraf-24h/20/tick-report.md`
- `build/iraf-24h/20/commands.log`
- 小计：声明 22 条，盘上存在 22 条，缺失 0 条

## 3. 缺失、提交问题与已登记缺口

- 台账声明与盘上事实一致：全部声明路径存在，全部步骤提交均在 HEAD 历史中（本轮退出码 0）。

### 3.1 本轮（收尾复核）对台账的校正

- 步骤 14 声明的 sensors.txt / check-report.txt 实际写在 cases/ 子目录（mtime 19:26 与本步 acceptance.txt 同轮，内容为该轮正向输出：points=167、fovy_rel_error=0.010427987255182231 等），evidence 已改为 cases/ 下的实际路径——原写法是路径不精确，不是证据丢失
- 步骤 06 声明的 commands.log、tick-report.md 与步骤 14 声明的 commands.log 盘上确实不存在，已从 evidence 声明中移除，并登记进本 wrapup.evidence_gaps（移除声明的唯一理由是「声明必须与盘上事实一致」，缺口本身不隐藏）

### 3.2 已登记证据缺口（`00-STATUS.json` → `wrapup.evidence_gaps`，与台账同文）

| 步骤 | 缺失文件 | 原因 | 影响 | 关闭条件 |
|---|---|---|---|---|
| 06 | `build/iraf-24h/06/commands.log` | 00-执行规则.md §5.5 要求每轮把命令与退出码落盘 commands.log；步骤 06 那一轮未落盘 | 该轮命令序列只能从 06/run_acceptance.sh 与 build-sdk.txt 复原，无法逐条审计退出码；不影响验收结论（build-sdk.txt 保留了原始输出） | 不补写（补写等于伪造记录）；如需逐条命令审计，按 06/run_acceptance.sh 重跑生成 |
| 06 | `build/iraf-24h/06/tick-report.md` | 同一轮未落盘 tick 汇报；06 是本战役 20 个步骤中唯一既无 commands.log 又无 tick-report.md 的步骤 | 该轮汇报文本缺失，调度器当轮输出仍在 ~/.hermes/cron/output/ 下（若未被轮转） | 不补写；结论以 06/tick-report 之外的台账 note 与 build-sdk.txt 为准 |
| 14 | `build/iraf-24h/14/commands.log` | 同上；步骤 14 的 tick-report.md 与其余证据齐全，仅缺 commands.log | 同步骤 14 的验收输出（acceptance.txt + cases/）完整，命令序列可由 run_acceptance.sh 复原 | 不补写；按 14/run_acceptance.sh 重跑即可重建 |

### 3.3 未补写声明（不伪造）

- 缺失的命令日志与 tick 报告一律不补写；本节的证据键集合与 docs/progress/iraf-24h-证据索引.md §3 同文，且由 build/iraf-24h/21 的核对脚本可复跑（同一脚本在 0 缺失时才退 0）

## 4. 边界说明（不伪造）

- 本索引只核对「台账声明 ↔ 盘上存在 ↔ 提交可达」三者一致，**不代表**证据内容正确：内容级判据见各步骤的验收输出与 `docs/debug/` 记录。
- 全部仿真结论 `simulation=true`；目标端/真机验收按范围调整为 `DEFERRED`（板卡不在场），未用 dry-run、mock 或历史数据冒充。
- 后续动作（台账 `wrapup.next_action`）：无未完成步骤：20/20 DONE 且无 BLOCKED。后续 tick 若无新增内容应保持静默（[SILENT]），不得重复生成索引、不得新增实现；板卡到位后按 00-执行规则.md §0 在既有脚本上执行目标端验收（DEFERRED 项），不改脚本语义
