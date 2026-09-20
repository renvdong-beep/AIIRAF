# 2026-09-20 四足适配器契约（步骤 16）实测与踩坑记录

计划战役：`iraf-24h` 步骤 16（QuadrupedAdapter 与 UnitreeGo2Adapter）。
本记录只写**实测数字与机制**，不写推测；无法证实的部分在「未证明」里显式登记。

## 1. 目标与边界

- 通用契约（`src/iraf_adapters/unitree/quadruped.py`）：能力词表 `stand/stop/locomote/read_state/emergency_stop`
  平台无关，厂家关节名 / DDS domain / CRC 一律不得进入；能力契约按 `declared ⊆ implemented`
  双向语义（方法存在 ≠ 能力已实现）。
- 机型实现（`src/iraf_adapters/unitree/unitree_go2.py`）：关节↔执行器映射一处解析，
  力矩上限只从模型 `actuator_ctrlrange` 读；`locomote` 首期未实现（无步态控制器）显式拒绝。
- 控制权：运动调用必须持有 `ControlAuthorityManager` 租约；旧 fencing token、
  终态执行的控制请求一律拒绝。全部结论 `simulation: true`；真机/目标端验收 DEFERRED（板卡不在场）。

## 2. 真实模型上的实测（`build/iraf-24h/16/adapter.txt`）

模型 = 步骤 13 生成的场景 `build/scenes/handoff_lab/handoff_lab.xml`
（`nq=26 / nv=24 / nu=12 / timestep=0.002`），声明 = `config/go2_loopback.yaml`。

| 项 | 实测值 |
|---|---|
| 关节↔执行器绑定 | 12 个关节各绑定 1 个直接执行器（`FL_hip_joint → FL_hip` …） |
| 力矩上限（模型 ctrlrange） | hip ±23.7 N·m、thigh ±23.7 N·m、calf ±45.43 N·m（逐关节独立复算一致） |
| `stand` 1.000 s | 100 个控制周期 × 5 子步；`ctrl_saturated_samples=0`；`target_source=profile_home` |
| 站立后基座高度变化 | **+0.010216000 m**（与步骤 15 的 0.280101 − 0.27 = ≈0.0101 m 同现象：接触地面后腿略伸） |
| `stand` 部分目标 `{FL_hip_joint: 0.5}` | `target_source=explicit_partial`，未指定关节回落到 Profile `spec.home` |
| `stop`（松力停机 3 s） | `base_height_drop_m=0.192798228`、`final_tilt_deg` 实测非零 ⇒ 力矩型电机松力后躺倒（预期） |
| 急停（无租约） | 闭锁生效、`torque_released=true`、`stepped=false`、`evidence_scope=adapter_state_only` |
| 闭锁期间 `stand` | 拒绝，码 `IRAF-QUADRUPED-SAFETY-LATCHED` |
| 授权复位后再 `stand` | 通过（复位不自动重新调度运动） |
| 终态执行续跑 | 拒绝，码 `IRAF-QUADRUPED-TERMINAL-EXECUTION` |

验收：`bash build/iraf-24h/16/run_acceptance.sh` → **通过 12 项，失败 0 项**；
`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_adapter` → **Ran 50 / OK**；
全量单测 **Ran 740 / failures=1 / errors=4 / skipped=4**（基线 690/1/4/4，失败集合逐项一致，+50 为本步新增）。

## 3. 踩坑（全部本轮实测）

1. **测试夹具的关节 `range` 单位是度**：MuJoCo 默认 `angle` 单位是**度**，夹具里写
   `range="-1 1"` 编译成 ±0.017453 rad，于是"Profile 限位 [-1,1] 宽于模型 jnt_range"这条
   **正确的**门禁把 26 个用例全部拦下（假失败）。修法是夹具加 `<compiler angle="radian"/>`，
   不是放宽门禁。识别信号：门禁消息里的数值恰好是 0.017453（= π/180）的整数倍。
2. **部分关节目标必须回落声明来源，而不是"当前位置"**：`stand(targets={单关节})` 首次实现
   在 `_run_control` 里 `target[joint]` 直接 KeyError。修法是在 `resolve_targets` 里把未指定
   关节补齐为 Profile 的 `spec.home`，并回报 `target_source=explicit_partial`——用"当前位置"
   兜底是隐式默认值，会把"保持姿态"和"目标姿态"混为一谈。
3. **混合行尾文件必须按字节改**：`src/iraf_adapters/factory.py` 是 114 行 CRLF + 30 行 LF，
   用 `Path.read_text()`/`write_text()`（通用换行翻译）改一处会让整文件行尾翻成 LF，
   diff 变成 135+/114− 的整文件改写。改用 `read_bytes()`/`write_bytes()`、锚点与新内容都按
   CRLF 书写后，diff 收敛为 23+/1−。教训：**先量行尾，再决定编辑粒度**。
4. **新增错误码会触发现有 SDK 门禁**（`test_every_code_emitted_in_source_is_registered`）：
   四足适配器新增 9 个 `IRAF-QUADRUPED-*` 码，`src/**` 扫描立刻报"源码抛出但 SDK 未登记"。
   处理方式是**登记**（`src/iraf_sdk/errors.py` 逐码给中文诊断 + `provenance=code-only` +
   `emitter` 路径:行号），不是绕过扫描；关闭条件（并入 IDL §5 表或声明分层规则）写进了映射注释。
   实测：登记后 `src` 出现码 28 = 已登记 28，`test_sdk_errors` Ran 55 / OK。
5. **`require_active_execution` 的判定顺序影响诊断质量**：原先"终态检查"先于"能力是否对得上"，
   导致用 `stop` 续跑一个已 `SUCCEEDED` 的 `stand` 执行报 `TERMINAL-EXECUTION` 而非
   `COMMAND-REJECTED`。三条检查（存在性 → 能力匹配 → 终态/token）都在下发控制量之前，
   因此把结构性错误提前不影响安全性，只让报错更精确。
6. **能力面门禁必须能真的命中厂家标识**：`(?i)\bdds\b` 匹配不到 `dds_domain`（`_` 是词字符），
   是"恒通过"的假门禁；改为 `(?i)dds` 后由 `test_vendor_tokens_are_detected` 正向对照钉住。
   （同类教训：负向用例旁边必须有正向对照，否则分不清"严格"与"恒失败"。）

## 4. 未证明 / 诚实边界

- 本步只证明**仿真**里"声明位形下站得住、能进入静止、状态可读、控制权与闭锁生效"；
  不代表步态 / 导航 / 停靠能力。`locomote` 未实现，速度指令被显式拒绝。
- `base_height_drop_m` 等停机数字只是**实测差值**；本步不给出"塌没塌"的布尔结论
  （阈值必须来自声明，不能由代码发明）。
- 关节限位与模型 `jnt_range` 的比对容差 1e-4 rad 来自 Profile 里四舍五入的实测值；
  收紧方向由 `_assert_limits_tighten_only` 强制，放宽即失败。
- 目标端 / 真机 / HIL 验收 **DEFERRED**（板卡不在场）：本轮未做任何 aarch64 或真机测试，
  未用 dry-run 或 mock 冒充真机证据。
- `scene.robots[unitree_go2].profile` 与 `baseline.robots[unitree_go2].baseline` 两处占位
  `closed_by: 步骤 16` **本轮未闭合**：其闭合条件文本要求 stand/stop 先经技能层验收（步骤 17），
  适配器层验收不足以声明框架能力；下一步在步骤 17 一并闭合（能力回填后占位改路径即可）。

## 5. 复跑命令

```
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --baseline config/ur5_simulation_baseline.yaml   # exit=0（回归）
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml             # exit=0 + 纯 JSON 摘要
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_adapter -v                          # Ran 50 / OK
PYTHONPATH=src /usr/bin/python3 build/iraf-24h/16/probe_adapter_real.py                                   # 真实场景模型上的适配器闭环
bash build/iraf-24h/16/run_acceptance.sh                                                                  # 通过 12 项、失败 0 项
PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit                          # Ran 740 / 1 FAIL / 4 ERROR（与基线同集合）
```
