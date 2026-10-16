# 补丁草案：`damped_hold`（可直接审查、一次性套用）

- 目标文件：`src/iraf_adapters/unitree/unitree_go2.py`（1603 行）
- 新增测试：`tests/unit/test_unitree_go2_damped_hold.py`
- 契约依据：`docs/debug/2026-09-22-damped-hold-spec.md`（正 3 条 / 负 6 条）
- 前置已核实事实：`:74 SUPPORTED_STOP_MODES`、`:377 stop()`（薄包装，`zero_torque=True`）、
  `:1046 balance_hold()`（"保持"的现成积木）、`:1448 _run_control()` 的 **`target=None ⇒ q_des=zeros`** 陷阱

## 1. 套用前必须先读的三段（不读不准改）

| 行段 | 要确认什么 |
|---|---|
| `:337-376`（`stand`） | 报告字段与 `ramp_s`/`resolve_duration_ms` 的用法（新方法要与之一致） |
| `:1046-1152`（`balance_hold` 全体） | `_balance_stats`/`_balance_summary` 的构造、采样与判定写法、报告字段 |
| `:1448-1603`（`_run_control` + 尾部） | `_run_control` 的返回契约（`cycles, saturated`）、`sample_callback` 调用时机、`self.estop`/租约校验模式 |

## 2. 改动 A：常量

```python
SUPPORTED_STOP_MODES = ("torque_zero_release", "damped_hold")   # :74，扩为两元组
```

## 3. 改动 B：`stop()` 增加模式分派（**向后兼容**）

```python
def stop(self, lease, execution_id=None, *, mode=None):
    """受控/失能停机分派。

    `mode=None` 保持既有行为（`torque_zero_release`，逐位一致，不改变步骤 01 的验收事实）；
    `mode` 由上层 `iraf_skills.quadruped.resolve_stop_mode` 决定，**调用方不得自选**
    （声明 `stop_modes.*.applies_to` 已规定适用路径）。
    """
    if mode is None or mode == "torque_zero_release":
        return self._stop_torque_zero_release(lease, execution_id)   # 现有函数体原样搬过来
    if mode == "damped_hold":
        return self.damped_hold(lease, execution_id=execution_id)
    raise CommandRejectedError(
        "不支持的 stop 模式 %r（声明允许：%s）" % (mode, SUPPORTED_STOP_MODES))
```
> 说明：把现有 `stop()` 的**函数体原样**搬进 `_stop_torque_zero_release`，保证既有行为逐位不变
> （避免"顺手重写"引入回归）；`stop()` 本体只剩分派。

## 4. 改动 C：新增 `damped_hold()`

形态 = `balance_hold` **去掉静态冲量注入** + 换成 spec 的完成条件；**不新写第二套控制律**。

```python
def damped_hold(self, lease, duration_ms=None, execution_id=None):
    """受控停止：减速到零并保持站立（ADR-0008 决策 1 的 `damped_hold`）。

    与 `torque_zero_release` 的区别（写死）：本路径**保持控制量与站立位形**，不松力、不退出控制；
    完成条件是"末速 ≤ stop.speed_tolerance_mps 且倾角 ≤ max_tilt_moving_deg 且高度在容差内"，
    超时未达标 ⇒ **显式失败**（不得回写成功，铁律 6.6）。
    """
    # 0) 与既有停止路径一致的安全属性：不被安全闭锁阻塞（安全方向），但仍须持租约
    self.require_capability("stop")
    self.require_lease(lease, "stop")
    active_id = (str(execution_id) if execution_id is not None
                 else self.begin_execution("stop", lease))
    if execution_id is not None:
        self.require_active_execution(execution_id, lease, capability="stop")

    # 1) 目标位形 = 声明 home（**绝不能传 target=None**：那会驱动到 0 rad，见 :1470）
    home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
    target = np.array([home[j] for j in self.joint_order], dtype=float)
    ramp_s = float(_dig(self.declaration, "stand.ramp_s", "声明"))

    # 2) 保持用平衡器（若声明启用）——与 balance_hold 同一套，不复制参数
    balance_params = self._balance_parameters()
    params = self._gait_parameters()
    geometry = self._leg_geometry(params)
    trunk_body = gait.trunk_body_id(self.model, self.mujoco, params)
    torque_provider = (self._balance_provider(params, geometry, trunk_body, balance_params)
                       if balance_params["enabled"] else None)

    # 3) 判据阈值与超时**全部来自声明**（本方法不写任何数字）
    tol_speed = float(_dig(self.profile.spec, "stop.speed_tolerance_mps", "Profile"))
    tilt_limit = float(_dig(self.declaration, "quadruped_limits.max_tilt_moving_deg", "声明"))
    timeout_s = float(_dig(self.declaration, "stop.timeout_s", "声明"))   # 若声明里键名不同，按声明改

    # 4) 跑控制（复用 _run_control + sample_callback 采样末速/倾角/高度）
    cycles, saturated = self._run_control(
        target, timeout_s, ramp_s, zero_torque=False,
        torque_provider=torque_provider, sample_callback=self._damped_hold_sample)
    # 5) 判定：先满足先成功；超时未达标 ⇒ 显式失败并带回最后状态
    ...
    return report      # 含 "stop": {"mode": "damped_hold", ...} 与成功/失败原因、末速/倾角/高度
```

## 5. 负路径用例（`tests/unit/test_unitree_go2_damped_hold.py`，按 spec N1~N6）

| # | 用例 | 断言要点 |
|---|---|---|
| N1 | 移动中调 `stop(mode="damped_hold")` | 报告 `stop.mode == "damped_hold"`，**不得**落 `torque_zero_release`；松力判据（torque 全 0）不成立 |
| N2 | 注入 QP 失败 | 仍为 `damped_hold`（不静默用旧解、不报成功） |
| N3 | 注入解过期 > 40.0 ms | 同上（判定复用 `adapters/unitree/mpc/freshness.py`） |
| N4 | 注入急停/安全事件 | 立即 `torque_zero_release`，终态 `FAILED`（不得回写成功） |
| N5 | 人为把 `stop.timeout_s` 调小到不达标 | 显式失败 + 原因 + 末速/倾角/高度 |
| N6 | 停止未完成时另一控制源申请接管 | 按租约/fencing token **拒绝**（铁律 12） |

## 6. 验收与证据

- `PYTHONPATH=src /usr/bin/python3 -m pytest tests/unit/test_unitree_go2_damped_hold.py -q` 全绿；
- 既有 `unitree_go2` 相关测试**不得回归**（`stop(mode=None)` 路径逐位不变）；
- 正路径 3 条按 spec §3 出数字，报告落 `build/acceptance/`；结论写调试记录。
