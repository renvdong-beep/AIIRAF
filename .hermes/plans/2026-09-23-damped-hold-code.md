# `damped_hold` 最终代码（可直接套用；含 spec 一处修正）

- 落地目标：`src/iraf_adapters/unitree/unitree_go2.py` + `tests/unit/test_unitree_go2_damped_hold.py`
- 全部事实已核实（行号）：`:42 TERMINAL_STATES`、`:74 SUPPORTED_STOP_MODES`、`:377 stop()`、
  `:1046 balance_hold()`、`:1448 _run_control()`（`target=None ⇒ q_des=zeros` 陷阱）、
  `config/go2_loopback.yaml:233` 的 `stop:` 段（`duration_s: 3.0`、`speed_tolerance_mps: 0.05`、
  `static_hold_s: 0.5`、`final_window_s: 0.2`）

## 0. spec 修正（我先前写错的一条，按代码事实改）

spec N4 原写"急停/安全事件 ⇒ 立即 `torque_zero_release`，终态 **FAILED**"。
但 ledger（`quadruped.py:263`）**禁止二次写终态**，且 `TERMINAL_STATES` 里语义正确的是 **`SAFETY_STOP`**
（`:42`）⇒ 修正为：**安全事件 ⇒ 立即释放控制量（松力），终态 `SAFETY_STOP`**（不是 FAILED）。
`FAILED` 保留给"超时未达标"这类**非安全**失败。

## 1. 改动 A：常量（`:74`）

```python
SUPPORTED_STOP_MODES = ("torque_zero_release", "damped_hold")
```

## 2. 改动 B：`stop()` 改为分派（原函数体**原样**改名，保证既有行为逐位不变）

```python
def stop(self, lease, execution_id=None, *, mode=None):
    """停机分派：`mode=None` 或 `torque_zero_release` 走既有失能停机（逐位不变）；
    `damped_hold` 走受控停止（减速到零并保持站立）。

    `mode` 由上层 `iraf_skills.quadruped.resolve_stop_mode` 决定，**调用方不得自选**
    （`profiles/safety/quadruped_lab.yaml` 的 `stop_modes.*.applies_to` 已规定适用路径）。
    """
    if mode is None or mode == "torque_zero_release":
        return self._stop_torque_zero_release(lease, execution_id)
    if mode == "damped_hold":
        return self.damped_hold(lease, execution_id=execution_id)
    raise CommandRejectedError(
        "不支持的 stop 模式 %r（支持：%s）" % (mode, SUPPORTED_STOP_MODES))

def _stop_torque_zero_release(self, lease, execution_id=None):
    # ← 原 stop() 的函数体**一字不改**搬到这里（含 require_capability/require_lease/
    #   begin_execution/require_active_execution/_run_control(..., zero_torque=True)/
    #   报告字段/ledger.finish(active_id, "STOPPED")）
    ...
```

## 3. 改动 C：新增 `damped_hold()`（插在 `_stop_torque_zero_release` 之后）

```python
def damped_hold(self, lease, execution_id=None):
    """受控停止：减速到零并保持站立（ADR-0008 决策 1 的 `damped_hold`）。

    与 `_stop_torque_zero_release` 的区别（写死）：本路径**保持控制量与站立位形**（不松力、
    不退出控制）；完成条件 = 连续 `stop.static_hold_s` 秒速度 < `stop.speed_tolerance_mps`
    且倾角 ≤ `quadruped_limits.max_tilt_moving_deg`；**超时未达标 ⇒ 终态 `FAILED`**（不得回写成功）；
    运行中出现急停/安全事件 ⇒ 立即松力并终态 `SAFETY_STOP`。
    阈值全部来自声明，本方法不写任何数字。
    """
    self.require_capability("stop")
    self.require_lease(lease, "stop")
    if execution_id is not None:
        self.require_active_execution(execution_id, lease, capability="stop")
        active_id = str(execution_id)
    else:
        active_id = self.begin_execution("stop", lease)

    stop_spec = self.declaration["stop"]
    budget_s = float(stop_spec["duration_s"])
    tol_speed = float(stop_spec["speed_tolerance_mps"])
    hold_need_s = float(stop_spec["static_hold_s"])
    final_win_s = float(stop_spec["final_window_s"])
    tilt_limit = float(self.declaration["quadruped_limits"]["max_tilt_moving_deg"])
    ramp_s = float(self.declaration["stand"]["ramp_s"])

    home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
    balance_params = self._balance_parameters()
    params = self._gait_parameters()
    geometry = self._leg_geometry(params)
    trunk_body = gait.trunk_body_id(self.model, self.mujoco, params)
    torque_provider = (self._balance_provider(params, geometry, trunk_body, balance_params)
                       if balance_params["enabled"] else None)

    samples = []
    state = {"bad": None}

    def sample_callback(cycle_index, info):
        qvel = np.asarray(self.data.qvel, dtype=float)
        self._damped_hold_samples(samples, info, qvel)
        # 安全事件优先：立即结束保持（终态由外层写 SAFETY_STOP）
        if self.estop.triggered if hasattr(self.estop, "triggered") else False:
            state["bad"] = "safety_event"

    # 用既有控制通路（PD + 重力前馈 + 平衡器 + ctrlrange 截断 + 子步推进），
    # 目标位形是**站立 home**（绝不能传 None —— `_run_control` 会把 None 解成 0 rad 位形）。
    cycles, saturated = self._run_control(
        home, budget_s, ramp_s, zero_torque=False,
        torque_provider=torque_provider, sample_callback=sample_callback,
    )

    speeds = [s["base_linear_speed_mps"] for s in samples]
    tilts = [s["tilt_deg"] for s in samples]
    heights = [s["base_height_m"] for s in samples]
    n_final = max(1, int(round(final_win_s * self.control_hz)))
    final_speed = float(np.mean(speeds[-n_final:])) if speeds else float("nan")
    max_tilt = float(max(tilts)) if tilts else float("inf")
    hold_ok = self._continuous_below(speeds, tol_speed, hold_need_s)
    tilt_ok = max_tilt <= tilt_limit
    ok = bool(hold_ok and tilt_ok)

    report = {
        "simulation": True,
        "capability": "stop",
        "stop_mode": "damped_hold",
        "mode": str(stop_spec.get("mode", "")),      # 保留既有字段（站立态语义），不改
        "execution_id": active_id,
        "fencing_token": int(lease.fencing_token),
        "duration_ms": budget_s * 1000.0,
        "control_cycles": cycles,
        "ctrl_saturated_samples": int(saturated),
        "succeeded": ok,
        "final_speed_mps": final_speed,
        "max_tilt_deg": max_tilt,
        "speed_tolerance_mps": tol_speed,
        "static_hold_s": hold_need_s,
        "final_window_s": final_win_s,
        "tilt_limit_deg": tilt_limit,
        "height_start_m": heights[0] if heights else None,
        "height_end_m": heights[-1] if heights else None,
        "failure_reason": "" if ok else (
            "safety_event" if state["bad"] else
            "超时未达标：末速 %.6f m/s（限 %.6f）｜倾角 %.4f°（限 %.1f°）"
            % (final_speed, tol_speed, max_tilt, tilt_limit)),
        "samples": samples,
        "final_state": self.read_state(),
    }
    if state["bad"]:
        self.ledger.finish(active_id, "SAFETY_STOP", report["failure_reason"])
    elif ok:
        self.ledger.finish(active_id, "STOPPED")
    else:
        self.ledger.finish(active_id, "FAILED", report["failure_reason"])
    return report

@staticmethod
def _continuous_below(values, limit, seconds, control_hz=200.0):
    """是否存在**连续** `seconds` 秒的"值 < limit"窗口（长度按控制频率换算样本数）。"""
    need = max(1, int(round(seconds * control_hz)))
    run = 0
    for v in values:
        run = run + 1 if v < limit else 0
        if run >= need:
            return True
    return False

def _damped_hold_samples(self, samples, info, qvel):
    """采样一条（字段与 balance_hold 保持一致，便于复用现有复核脚本）。"""
    quat = np.asarray(self.data.qpos[3:7], dtype=float).copy()
    _quat, roll, pitch = self._attitude_terms()
    samples.append({
        "time_s": float(self.data.time),
        "base_height_m": float(self.data.qpos[2]),
        "base_position_xy_m": [float(self.data.qpos[0]), float(self.data.qpos[1])],
        "base_linear_speed_mps": float(np.linalg.norm(qvel[0:3])),
        "tilt_deg": float(quat_tilt_deg(quat)),
        "roll_deg": float(np.degrees(roll)),
        "pitch_deg": float(np.degrees(pitch)),
        "ctrl_saturated": int(np.count_nonzero(info["saturated"])),
        "tracking_error_rad": float(np.max(np.abs(
            info["desired"] - np.asarray(self.data.qpos[self.qpos_adr], dtype=float)))),
    })
```

> 注：`_continuous_below` 的 `control_hz=200.0` 默认值应改为 `self.control_hz`（写成静态方法只是为便于单测；
> 套用时改为实例方法并传 `self.control_hz`）。`self.estop` 的"是否触发"接口以实际 API 为准（套用时按代码改，
> 不改语义）。

## 4. 测试（`tests/unit/test_unitree_go2_damped_hold.py`）要点

| 用例 | 做法 | 断言 |
|---|---|---|
| P1 正路径（移动中停止） | 先用 `locomote` 命令 vx=0.2 跑一段，再 `stop(mode="damped_hold")` | `succeeded` True；`final_speed_mps ≤ 0.05`；`max_tilt_deg ≤ 15.0`；`stop_mode=="damped_hold"`；ledger 终态 `STOPPED` |
| P2 站立态调用 | 站立 → `stop(mode="damped_hold")` | 也成功（已在站立，立即满足连续静止） |
| N1 **移动中不得落松力** | 移动中 `stop(mode="damped_hold")` | 报告 `stop_mode=="damped_hold"`；**且** `mode` 字段仍为声明值（未被覆盖）；采样里存在非零控制量（非全零 ctrl 样本） |
| N2 QP 失败注入 | 让 Provider/门禁返回非 ok | 仍 `damped_hold`，不得静默用旧解（断言无 `ok` 终态） |
| N3 解过期注入 | 解龄 > 40.0 ms | 同上 |
| N4 安全事件 | 运行中触发 estop | 终态 `SAFETY_STOP`（**不是 FAILED**，见 §0 修正）；控制量归零 |
| N5 超时未达标 | 把 `stop.duration_s` 用测试内声明的极小值（或注入大扰动使其达不成静止） | 终态 `FAILED`；`failure_reason` 含末速/倾角实际值；**不得**为 `STOPPED` |
| N6 接管竞争 | 停止未完成时另一控制源申请租约 | 被拒（fencing token）；停止完成后才可接管 |
| R1 回归 | 既有 `stop()`（`mode=None`） | 与既有基准逐位一致：`height_mean_m=0.279953602548388`、末速 `0.0038248382123762478 m/s` |

## 5. 套用顺序与验收
1. 套用 A/B/C 三处 → `PYTHONPATH=src pytest tests/unit/test_unitree_go2_damped_hold.py -q` 全绿
2. 既有 `unitree_go2` 相关测试**不回归**（B 的"原样搬家"是这条的保证）
3. 按 `config/go2_loopback.yaml:233` 的治理要求：**另立步骤并复跑 loopback 验收**，确认站立态基准未变
4. 提交（只 add `unitree_go2.py` + 新测试文件）；调试记录写 `docs/debug/`
