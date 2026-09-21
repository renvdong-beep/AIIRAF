# 四足技能层（stand/stop/locomote）接入与拒绝路径：实测记录

- 日期：2026-09-20
- 范围：步骤 17「stand/stop/locomote 技能与拒绝路径」（计划战役 `iraf-24h`，x86-first）
- 产物：`skills/stand/`、`skills/locomote/`、`scripts/verify_quadruped_skills.py`、
  `src/iraf_skills/quadruped.py`、`profiles/safety/quadruped_lab.yaml`
- 证据：`build/acceptance/go2-skills/report.json`、`build/iraf-24h/17/`
- 结论口径：全部为 `simulation: true` 的仿真结论；目标端/真机验收 DEFERRED（板卡不在场）

## 1. 交付了什么（链路与判据）

链路不绕过任何一层：`TaskFlow → SkillRuntime → PolicyGateway/ControlAuthority → Provider
（iraf_skills.quadruped）→ 适配器（UnitreeGo2Adapter）→ MuJoCo`。

成功路径（标称序列 `[stand, stop]`，来自 `config/go2_loopback.yaml` 的 `skills` 段）：

| 用例 | 状态 | 关键实测数字 |
|---|---|---|
| stand | SUCCEEDED | 墙钟 0.39167014486156404 s；工作空间位移 0.00643820654999534 m、倾角 0.05779130222366524° |
| stop | SUCCEEDED | 墙钟 0.5178679858800024 s |

> ⚠ **口径（2026-09-21 战役 `iraf-24h-2` 补注）**：上表数字测于**关键帧四足穿入台面 18.372 mm** 的旧场景。
> 该场景缺陷已在战役 `iraf-24h-2` 步骤 02 第五轮修掉（场景生成按实测抬升，提交 `2043111`），
> 对齐后重测为：stand 墙钟 **0.3259358201175928 s** / 位移 **0.0069012464253042785 m** /
> 倾角 **0.07057470486485902°**、stop 墙钟 **0.46566887316294014 s**（全部仍达标）。
> 上表旧值**自提交 `2043111` 起被取代**，见 `docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md` §11。

拒绝路径 7 条（声明下限 5，`skills.minimum_rejection_cases`），逐条都是「错误码 + 非空原因」：

| 用例 | 错误码 | 拒绝维度 | 原因（原样） |
|---|---|---|---|
| reject-deadline-expired | `IRAF-DEADLINE-EXCEEDED` | 截止时间已过 | 任务截止时间已过期 |
| reject-rbac-no-submit-role | `IRAF-POLICY-DENIED` | 无权限上下文 | 调用方无任务提交权限 |
| reject-unauthenticated-context | `IRAF-UNAUTHENTICATED` | 无受信身份 | missing authenticated context |
| reject-capability-not-declared | `IRAF-SKILL-PROVIDER-UNAVAILABLE` | 能力未声明 | RobotProfile 缺少能力: ['locomote'] |
| reject-velocity-limit-exceeded | `IRAF-QUADRUPED-COMMAND-REJECTED` | 越界速度 | 速度指令线速度 5 m/s 超过声明上限 0.5 m/s |
| reject-stand-target-out-of-limit | `IRAF-EXECUTION-FAILED` | 目标越出声明限位 | 关节 FL_calf_joint 目标 0.0 越出声明限位 [-2.7227, -0.83776] |
| reject-stale-idempotency-key | `IRAF-IDEMPOTENCY-CONFLICT` | 过期/重复状态 | same idempotency key maps to different request |

## 2. 设计决策（为什么这样做）

1. **`skills/stop/` 复用而不新增**：`skills/stop` 早已存在且与机型无关（Provider 只调
   `backend.stop(lease)`，停机语义由机型 `stop.mode` 声明）。步骤文件列了「新增 skills/stop/」，
   但复制一份就会出现两套停机实现（AGENTS.md 6.3 明确禁止堆特例、要求先重构接口）。
   实测该技能在四足上可用：墙钟 0.5179 s < 其声明的 `timeoutSeconds: 2`（租约 TTL），
   因此不需要为四足放宽或改动任何既有声明。
2. **速度上限进声明、由调度前门禁消费**：`PolicyGateway` 只懂平台无关维度，不知道四足的
   速度上限。把上限写进脚本或 Skill 的 JSON Schema 都会造成第二份安全边界；因此
   `profiles/safety/quadruped_lab.yaml` 的 `spec.quadruped_limits` 是唯一来源，
   `iraf_skills.quadruped.enforce_velocity_limits` 在**调度前**消费它，超限即拒绝并用
   `SkillRuntime.record_pre_dispatch_failure` 落执行记录。缺段/缺键即退出码 2（有负向用例）。
3. **工作空间限制对实测状态判定**：`max_base_translation_m` / `max_tilt_deg` 与执行前状态
   （`backend.read_state()`，标准反馈接口，只读）比对，写进报告 `workspace.checks`。
   倾角判据取**相对竖直的倾斜角**（`acos(R[2,2])`），不含偏航——步骤 15 的实测教训已钉成用例。
4. **行走（locomote）只交付拒绝路径**：决策 4.B 首期无步态控制器。`locomote` 的 Skill 清单与
   schema 存在，但 Profile **不声明**该能力 ⇒ Policy 在调度前拒绝；即使被绕过，Provider 也会
   显式抛 `UnsupportedCapabilityError`。若某天后端"成功返回"，Provider 仍会拒绝（禁止伪造成功）。
5. **能力回填时机**：`profiles/unitree_go2_mujoco.yaml` 的 `capabilities` 由 `[]` 改为
   `[stand, stop]`，**仅在**本步骤全链路验收通过之后（步骤文件明令）。

## 3. 本轮踩到的自身缺陷（全部在验收前被自己的用例抓到）

1. **`--config` 里"未给出上下文"与"显式 None"被混为一谈**：拒绝用例 `run()` 里
   `context if context is not None else _context(case)` 把「未认证」用例替换成了合法上下文，
   于是该用例"恒通过"（实际返回 SUCCEEDED）。修法：用 `_UNSET` 哨兵区分两种语义。
   *识别信号*：拒绝用例出现 `status: SUCCEEDED` 且 `error_code` 为空。
2. **幂等键作用域被忽略，导致假用例**：`store` 的幂等键是 `UNIQUE(subject, idempotency_key)`，
   而"过期/重复状态"用例的两次请求用了**不同 subject** ⇒ 第二次被当成全新执行并成功。
   修法：两次请求共用同一 subject，并在代码注释里写明这条作用域。
3. **忘记把幂等键传给第二个请求**：`run()` 没有转发 `idempotency_key`，第二次仍拿到随机键，
   用例继续"恒通过"。这类"用例自己把条件抹掉"的缺陷比实现缺陷更危险，已用
   `passed` 断言 + 逐条 `expected_error_code` 比对钉住。
4. **既有用例因事实变化而失败（不是回归）**：`test_scene_builder_injection` 断言
   `capabilities == []` 与 `profile_source == declared_identity_lookup`。步骤 17 把
   `scene.robots[].profile` 从占位改为路径后，构建器走"声明路径"来源、能力变为非空。
   两处断言按**事实**同步（不是放宽门禁），并写明变化来源。
5. **占位夹具的 `closed_by` 有格式契约**：`unverified_ref.closed_by` 必须匹配 `^步骤 [0-9]{2}$`，
   夹具写自由文本会被 **schema**（退出码 2）拦下，测不到"两处声明不一致"这条规则。

## 4. 与既有交付的交叉验证（数字原样）

- `verify_go2_loopback.py`：exit=0，`height_mean_m=0.2801007638069918`、
  `max_attitude_error_deg=0.05779130222480239`、`max_tracking_error_rad=0.04473942561095967`
  ——与步骤 15 记录逐位一致。
  > ⚠ **口径（2026-09-21 战役 `iraf-24h-2` 补注）**：这三个数字测于"关键帧四足穿台 18.372 mm"的旧场景；
  > 该缺陷已在战役 `iraf-24h-2` 步骤 02 第五轮修掉（提交 `2043111`）。对齐后重测为
  > `height_mean_m=0.279953602548388`、`max_attitude_error_deg=0.11199278558472758`、
  > `max_tracking_error_rad=0.04646826440572238`（判据仍全过，退出码 0）。
  > 上面的旧值**自提交 `2043111` 起被取代**，见 `docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md` §11。
- `verify_scene_sensors.py`：exit=0，`fovy_rel_error=0.010427987255182231`、`points=167`、
  `miss_fraction=0.5361111111111111`、`acc_rel_error=2.0889954113422363e-16` ——与步骤 14 逐位一致。
- `scene_check.py --scene scenes/handoff_lab --model <生成模型>`：exit=0；Go2 的
  `pending_refs` 已清空（仅剩 humanoid 三处：`robots.humanoid_static.profile`、
  `baseline.robots.humanoid_static`、`baseline.initial_state.humanoid_static`）；
  `pending_steps` 剩 4 条：`nominal.s02_dock`、`nominal.s04_place_in_tray`、
  `nominal.s05_confirm_payload`、`fault_sensor_loss.f02_dock`（停靠/放置/载荷确认均未交付）。
- `profile_check.py --quadruped config/go2_loopback.yaml`：exit=0，`failures: []`。
- 全量单测（`/usr/bin/python3`）：`Ran 764 / failures=1 / errors=4 / skipped=4`，失败集合与基线
  （4 项 stub 导入 ERROR + `test_vision_processing`）逐项一致；+24 为本步新增用例。

## 5. 复跑命令

```bash
cd /home/coretek/AIIRAF
PYTHONPATH=src /usr/bin/python3 scripts/verify_quadruped_skills.py --config config/go2_loopback.yaml; echo "exit=$?"
bash build/iraf-24h/17/run_acceptance.sh            # 13 项验收，含 5 条负向（退出码 1/2/5）
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_skill_contracts -v
```

## 6. 未证明 / 缺口（不得当成已通过）

- **没有行走**：首期无步态控制器，`locomote` 只证明"被拒绝"；步态/导航/停靠/载荷确认均未验收。
- `locomote.output.json` 描述的成功形状**未被任何用例触发**（能力被拒绝），它只是契约声明。
- 站立/停止结论只代表"声明位形下站得住、能进入静止、状态可读"，不代表机动能力。
- 目标端/真机验收 DEFERRED（板卡不在场）：本文件全部数字来自 MuJoCo 仿真（`simulation: true`）。
