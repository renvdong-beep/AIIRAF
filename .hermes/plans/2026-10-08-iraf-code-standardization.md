# IRAF 规范化整理计划（基于 2 臂 + Go2 联合演示这一条已跑通的链路）

日期：2026-10-08　　依据：AGENTS.md（安全铁律 / 编程铁律 / 分层铁律 / 交付铁律 / 工程习惯 / 易用性）
范围：**只整理本次演示实际走到的代码与产物**（不铺开全仓），每项都要有证据、可验收、可回滚。

## 0. 本条链路的基线（先钉住，整理不得破坏）
```
整链：s01~s07 9/9 通过（三次独立运行 + 1 次带接触扫描）；UR5e↔Go2 接触 0 条；静态净空 +0.111100 m
产物：build/scenes/handoff_lab/handoff_lab_joint.{json,xml}；报告 build/acceptance/handoff_lab/nominal/report.json
入口：scripts/build_scene.py → scripts/scenario.py run → scripts/campaign_pass_rate.sh
判据（一个都不许动）：0.030 m / 2.0° / s06 0.06 / s07 0.065 / s03 0.005
```

## 1. 不符合项清单（本次链路，按铁律分组；每项给证据与验收）

### A. 契约 / 声明（AGENTS 2.1、5.3；本仓习惯"先改 IDL/schema 再改实现"）
| # | 问题 | 证据 | 修法 | 验收 |
|---|---|---|---|---|
| A1 | 本次新增的 4 个声明键**未登记进 schema**：`grasp.place_via`、`grasp.place_elbow_body`、`grasp.place_elbow_pref`、`grasp.place_transit_extra_z_m` | 三处代码读它们（`scripts/build_piper_baseline.py` 的 via/肘部段、`src/iraf_adapters/unitree/scene_builder.py` 的 `place_transit_extra_z_m`）；`config/scene.schema.json` 未见对应项 | 先查 schema 是否把 `baseline_overrides` 定义成自由对象；若是，则补**受控枚举/类型**并在 description 里写契约 | `scene_check` 通过；故意写错类型/枚举值应被 schema 拒绝（拒绝路径要有证据） |
| A2 | **同一事实多处**：B 站站位 ↔ `reference_solver.target_override_world_m`（抓取名义目标）必须手抄同改 | 实测：不同改 ⇒ 运行期「实测目标 vs 名义解差 0.272365942 m > 0.07」⇒ s06 拒 | 让该键**从载体站位帧派生**（与 `place_targets[].nominal_pose_m` 同一算法，见 `scene_builder.py` 里算 nominal_docked_station 的那段）；保留显式字面量作为**覆盖**（缺省派生） | 站位改动后**不改**该键，s06 仍过；参数化测试：把站位移 0.05 m，重建后 nominal 目标随之动 |
| A3 | 落点垫/站位/容差等"位置类事实"分散在 4 处（scene frames、place_targets、dock stations、reference_solver），缺**一致性校验** | 本次逐个踩到（③④⑤⑥ 四类症状） | 构建期加一条**交叉校验**：若同时声明了站位帧与抓取目标字面量，则两者矛盾即失败（fail-closed） | 故意写矛盾值 ⇒ 构建期报错并给出两处名字与差值 |

### B. 分层（AGENTS 3）
| # | 问题 | 证据 | 修法 | 验收 |
|---|---|---|---|---|
| B1 | `mujoco_backend.py` 里**放置相位的回放/白名单/接触事实**混在一个 5k 行文件里；本次为加 `via` 相位改了 4 处（求解器/构建器序/白名单/回放） | 本轮改动 5 文件 33 行（commit abe59fa） | 把"相位集合"收敛为**一处声明**（如 `PLACE_PHASES` 常量表：名字/顺序/是否可选/是否需要 CarryGoal），求解器、构建器、后端、探针都从它取；`via`/`pre_lift`/`regrasp` 统一登记 | 加第 6 个相位只需改这一处；现有 9/9 不变（逐位） |
| B2 | 探针与产品代码各写一份相位顺序（本次 `probe_place_carrier_dock_offset.py` 就漏掉了 `via`，导致一次误判） | 实测：探针先报 `path:transit>above` 而实际已是 `via>above` | 探针改为**从报告读相位顺序**（或复用 B1 的常量表） | 探针输出里的相位序与报告一致（自动比对） |
| B3 | `scripts/probe_*.py` 与临时 one-liner 混杂：本次"录屏"用了 xwininfo+ffmpeg 内联命令（重复、易错：我连续踩了两次 SIZE 解析） | 会话记录：两次 ffmpeg 参数错（`0x5200007`、`1280x720+5+29`） | 落成 `scripts/record_demo.sh`（查窗口几何→校验参数→录制→帧差校验→落盘两处） | 连续两次跑同一脚本产出可播 MP4，且日志里打印窗口几何与校验数字 |

| B4 | **跨域 Provider 缺口（原判"服务端缺失"于同日自我更正）**：gRPC **服务端已存在** —— `src/iraf_adapters/grpc/runtime_grpc.py` 的 `SkillRuntimeServicer`（`Execute` 返回 `stream SkillFeedback`、`Cancel`、`GetExecution` + token 鉴权、`add_execute_get_servicer_to_server`）；`server.py`（43 行）只是入口桩。真正缺的是**服务端后面要接的 Provider 与注册表**（有界范围 grpc/sdk/core 内 grep 未见 `CapabilityProvider`/`register_provider`/`PROVIDERS`）。契约（`api/proto/iraf/v1/*.proto`）与客户端（`src/iraf_sdk/client.py`，579 行含 `deadline_after`）均在 | 2026-10-08 实读：`wc -l server.py`=43；`grep CapabilityProvider\|register_provider\|PROVIDERS` 零命中 | 实现 `servicer.py`（iraf.v1 服务端：能力执行 → Provider → 实测事实；deadline/取消/健康检查）+ `provider_stub.py`（控制域替身，可注入超时/不可达）+ `scripts/verify_domain_split.sh`（四例验证） | 双域容器链路四例通过：①正常 ②策略拒绝 ③deadline 超时/取消显式失败 ④不可达 ⇒ 显式失败+安全停机；报告标注"容器≠板卡、不构成实时性证据" |

### C. 证据 / 观测者效应（AGENTS 2.8、5.4）
| # | 问题 | 证据 | 修法 | 验收 |
|---|---|---|---|---|
| C1 | 运行期"臂↔载体"接触事实**只在调试开关下**（`IRAF_DEBUG_PLACE=1`）可得；默认报告里没有"演示是否碰载体"这项证据 | 本次要判"会不会碰"必须开调试跑一次（且调试通路会改变墙钟） | 把"臂↔载体接触对"作为**常规证据**写进 `place_object` 的 evidence（数量 + 最深 pair），默认开启、零打印 | 默认运行的报告里能直接读到"臂↔Go2 接触 0 条"；与调试通路数值一致 |
| C2 | 调试通路与 clean 运行**数值不同**（本次多次遇到），但没有一处显式提醒 | 例如 demo2 与 clean 的停靠残差不同 | 在报告 notes 里加一条：本报告是否带调试通路（env 白名单摘要） | 报告 notes 里能看到该轮是否带调试开关 |
| C3 | `place_settled_speed_mps` 出现 66~495 m/s（与载荷静止矛盾），至今**未定性** | docs §11.93 附 已留档；本轮未解决 | 加默认关闭的逐自由关节 dump 定性（否则该字段不得作为"已落稳"的完整证据） | 一次运行给出"是哪个 body、哪一步"的结论 |

### D. 测试（AGENTS 2.8、3、交付铁律）
| # | 问题 | 证据 | 修法 | 验收 |
|---|---|---|---|---|
| D1 | 本次修复（碰撞/停靠容差/布局）**只靠整链与批次**，没有**单元级回归**：净空判据、可达性判据（√(dy²+home_z²) ≤ reach）、"同半球规则"都没有独立测试 | 结论都来自整链运行 | 把本轮用过的三条一次性判据固化为 `tests/`（或 `scripts/` 里的可复跑检查）：① 净空探针判据 ② 可达性公式 ③ 相位集合一致性 | 三条检查进 CI/预提交；破坏任一条件能被单独抓到（不必跑整链） |
| D2 | 拒绝路径：本次新键的错误值（如 `place_via: xx`、`place_elbow_pref: mid`）**有 fail-closed 但没有测试** | 代码里已 `raise`，无测试覆盖 | 补拒绝用例（错误枚举/缺配对键/body 不存在） | 每个拒绝分支有测试 |

### E. 文档 / 可追溯（AGENTS 5.6、3）
| # | 问题 | 证据 | 修法 | 验收 |
|---|---|---|---|---|
| E1 | 六处声明改动只写在**声明旁注释**与 §11.95，未进**总体架构图/ADR**（铁律 6.6：架构/边界变化须同变更更新图与 ADR） | §11.95 表格 | 至少加一页 ADR（`docs/adr/`）："交接布局的可达性-净空联合约束"，含公式与实测表 | ADR 与 §11.95 数字一致 |
| E2 | 本次新增的两个惰性机制（via 相位、肘部偏好）**没有最小可运行示例**（铁律 6.2：每项公共能力须有 schema + 中文说明 + 最小示例） | 代码里有注释，无示例 | 在场景文档或 `docs/` 里给 3 行"怎么开/怎么验" | 按示例能一键复现探针数字 |

## 2. 执行顺序（每步小、可验收、可回滚；先做能立刻消除"再犯"的）
```
P1  B3 录屏脚本化            ← 最便宜、且我已重复踩错两次（rule 5.2 明确要求）
P2  A1 schema 登记 4 个新键   ← 契约先行（我这轮欠的债）
P3  C1 常规证据化（臂↔载体）  ← DONE 6a570b6（9/9、接触计数=0、两轮逐位一致）
P4  B1/B2 相位集合单点声明    ← 见下条提交（PLACE_PHASES 单点表；探针从报告读相位）
P5  A2 抓取目标改为派生       ← 见下条提交（派生（声明门控）；站位移 0.05 m 目标自动跟随）
P6  A3 位置类声明交叉校验         ← 见下条提交（fail-closed；报两处名字与差值）
P7  D1/D2 三条判据 + 拒绝路径测试  ← 见下条提交（可达性函数固化；via/elbow 拒绝用例）
P8  C2/C3/E1/E2 报告标注 / settle 速度定性 / ADR / 示例  ← 见下条提交
每个 P 完成后：连跑 2 轮逐位比对（防回归）→ 提交（Conventional Commits，中文）→ 更新本文件的状态行

## 4. 规范化执行结果（2026-10-12，P3–P8 全部完成）
```
P3  C1 臂↔载体接触对常规证据化   → 6a570b6（9/9；接触计数=0；与调试通路一致）
P4  B1/B2 相位单点声明 PLACE_PHASES → 487258e（9/9；两轮与 P3 逐位一致）
P5  A2 抓取目标从载体站位帧派生   → eed6803（缺省逐位不变；站位移 0.05 m 目标自动跟随）
P6  A3 位置类声明构建期交叉校验   → a36069d（矛盾 fail-closed；报两处名字与差值）
P7  D1/D2 判据固化 + 拒绝路径测试  → 39ce14a（可达性函数固化；via/elbow 拒绝用例）
P8  C2/C3/E1/E2 报告标注/定性/ADR/示例 → c30158e
```
验收（最终）：两层分别重建 exit 0；整链 clean 两轮 **9/9 逐位一致**；
全量单测 1305 例（1301 通过 / 5 skip / 4 为环境缺依赖的 loader error，与本次改动无关）。
基线约束全部守住：判据未动、9/9 未退化、改码期间不跑批次。
```

## 3. 约束（不得违反）
- 判据一个不动；基线（9/9 与上述六个数）不得退化。
- 探测/整理本身不得改变被观测对象（改码期间不跑批次；批次期间冻结源码）。
- 每步都要有拒绝路径或反例证据；否定结论照样留档。
- 不提交密钥、内部地址、模型权重、传感器原始数据、构建产物或日志（`build/` 下只留 .gitkeep 与既有 calibration）。
