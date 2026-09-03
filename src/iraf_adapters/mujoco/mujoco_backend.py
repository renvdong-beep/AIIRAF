"""带连续步进、指标和受控故障注入的 MuJoCo 3 后端。"""

from pathlib import Path
import threading
import time

import mujoco


class MujocoBackend:
    @classmethod
    def from_config(cls, config, profile, authority):
        fault_injection_enabled = config.get("fault_injection_enabled", False)
        if not isinstance(fault_injection_enabled, bool):
            raise ValueError("fault_injection_enabled 必须是布尔值")
        return cls(
            config["model_path"],
            profile,
            authority,
            fault_injection_enabled=fault_injection_enabled,
        )

    def __init__(self, model_path, profile, authority, fault_injection_enabled=False):
        self.profile = profile
        self.authority = authority
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path)))
        self.data = mujoco.MjData(self.model)
        self._actuators = {
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            ): index
            for index in range(self.model.nu)
        }
        self.last_positions = {joint: 0.0 for joint in profile.joints}
        self.stopped = False
        self._cancel_event = threading.Event()
        self._data_lock = threading.RLock()
        self._loop_lock = threading.RLock()
        self._metrics_lock = threading.RLock()
        self._loop_stop = threading.Event()
        self._loop_thread = None
        self._continuous_mode = False
        self._fault_injection_enabled = bool(fault_injection_enabled)
        self._fault_kind = None
        self._fault_delay_seconds = 0.0
        self._fault_remaining = 0
        self._reset_metrics()

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "mode": "simulation"}

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        self._cancel_event.clear()
        with self._data_lock:
            for joint, value in positions.items():
                if joint not in self._actuators:
                    raise ValueError("actuator not found: " + joint)
                self.data.ctrl[self._actuators[joint]] = float(value)
                self.last_positions[joint] = float(value)
            self.stopped = False
        if self._continuous_mode:
            deadline = time.monotonic() + max(1, int(duration_ms)) / 1000.0
            while not self._cancel_event.is_set() and time.monotonic() < deadline:
                time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
            return {
                joint: float(self.last_positions[joint])
                for joint in self.profile.joints
            }
        for _ in range(max(1, int(duration_ms / 10))):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            self.step()
        return {
            joint: float(self.last_positions[joint]) for joint in self.profile.joints
        }

    def stop(self, lease):
        self.authority.validate(lease)
        self._cancel_event.set()
        self._safe_stop_controls()

    def step(self, count=1):
        count = int(count)
        if count < 1:
            raise ValueError("MuJoCo 步进次数必须为正数")
        for _ in range(count):
            fault_kind, fault_delay = self._consume_fault()
            started = time.monotonic()
            if fault_kind == "step_failure":
                raise RuntimeError("注入的 MuJoCo 步进故障")
            if fault_kind == "step_delay":
                time.sleep(fault_delay)
            with self._data_lock:
                mujoco.mj_step(self.model, self.data)
            self._record_step(time.monotonic() - started)
        return {
            joint: float(self.last_positions[joint]) for joint in self.profile.joints
        }

    def start_continuous(self):
        """启动唯一物理步进线程；故障后必须显式清除故障才能重启。"""
        with self._loop_lock:
            if self._loop_thread is not None and self._loop_thread.is_alive():
                return False
            with self._metrics_lock:
                if self._last_error is not None:
                    raise RuntimeError("MuJoCo 步进异常尚未清除，拒绝重新启动")
                self._reset_metrics_locked()
            self._loop_stop.clear()
            self._continuous_mode = True
            self._loop_thread = threading.Thread(
                target=self._run_loop,
                name="mujoco-stepper",
                daemon=True,
            )
            self._loop_thread.start()
            return True

    def stop_continuous(self, timeout=2.0):
        with self._loop_lock:
            thread = self._loop_thread
            if thread is None:
                self._continuous_mode = False
                return False
            self._loop_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout)))
        with self._loop_lock:
            stopped = not thread.is_alive()
            if stopped:
                self._loop_thread = None
                self._continuous_mode = False
            return stopped

    def is_continuous(self):
        with self._loop_lock:
            return bool(
                self._continuous_mode
                and self._loop_thread
                and self._loop_thread.is_alive()
            )

    def simulation_status(self):
        """返回稳定、可序列化的仿真运行指标快照。"""
        running = self.is_continuous()
        now = time.monotonic()
        with self._metrics_lock:
            elapsed = max(0.0, now - self._loop_started_at)
            mean_duration = (
                self._total_step_duration_seconds / self._step_count
                if self._step_count
                else 0.0
            )
            effective_frequency = self._step_count / elapsed if elapsed > 0 else 0.0
            last_step_age = (
                max(0.0, now - self._last_step_at)
                if self._last_step_at is not None
                else None
            )
            return {
                "schema_version": "iraf.mujoco.simulation-status/v1",
                "simulation": True,
                "healthy": bool(running and self._last_error is None),
                "running": running,
                "target_frequency_hz": round(1.0 / self._target_period_seconds, 3),
                "effective_frequency_hz": round(effective_frequency, 3),
                "step_count": self._step_count,
                "step_overrun_count": self._step_overrun_count,
                "step_failure_count": self._step_failure_count,
                "last_step_duration_ms": round(
                    self._last_step_duration_seconds * 1000.0, 3
                ),
                "mean_step_duration_ms": round(mean_duration * 1000.0, 3),
                "max_step_duration_ms": round(
                    self._max_step_duration_seconds * 1000.0, 3
                ),
                "last_step_age_ms": (
                    round(last_step_age * 1000.0, 3)
                    if last_step_age is not None
                    else None
                ),
                "uptime_ms": round(elapsed * 1000.0, 3),
                "last_error": self._last_error,
                "fault_injection_enabled": self._fault_injection_enabled,
            }

    def inject_fault(self, kind, duration_ms=0, occurrences=1):
        """仅供显式启用的开发验收实例注入有界故障。"""
        if not self._fault_injection_enabled:
            raise PermissionError("当前 Backend 未启用仿真故障注入")
        if kind not in {"step_delay", "step_failure"}:
            raise ValueError("不支持的仿真故障类型: " + str(kind))
        occurrences = int(occurrences)
        if occurrences < 1:
            raise ValueError("故障注入次数必须为正数")
        delay_seconds = 0.0
        if kind == "step_delay":
            duration_ms = float(duration_ms)
            if duration_ms <= 0:
                raise ValueError("step_delay 的 duration_ms 必须为正数")
            delay_seconds = duration_ms / 1000.0
        with self._metrics_lock:
            if self._fault_remaining:
                raise RuntimeError("已有仿真故障等待执行")
            self._fault_kind = kind
            self._fault_delay_seconds = delay_seconds
            self._fault_remaining = occurrences
        return {
            "kind": kind,
            "duration_ms": round(delay_seconds * 1000.0, 3),
            "occurrences": occurrences,
        }

    def clear_faults(self):
        """停止后显式清除故障锁存和待执行注入。"""
        if not self._fault_injection_enabled:
            raise PermissionError("当前 Backend 未启用仿真故障注入")
        if self.is_continuous():
            raise RuntimeError("仿真运行期间禁止清除故障")
        with self._metrics_lock:
            self._fault_kind = None
            self._fault_delay_seconds = 0.0
            self._fault_remaining = 0
            self._last_error = None
        return True

    def render_frame(self, renderer):
        """渲染一致快照，不向外暴露 MuJoCo 控制状态。"""
        with self._data_lock:
            renderer.update_scene(self.data)
            return renderer.render()

    def _run_loop(self):
        period = self._target_period_seconds
        while not self._loop_stop.is_set():
            started = time.monotonic()
            try:
                self.step()
            except Exception as exc:
                self._safe_stop_controls()
                with self._metrics_lock:
                    self._step_failure_count += 1
                    self._last_error = f"{type(exc).__name__}: {exc}"
                self._loop_stop.set()
                break
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                self._loop_stop.wait(remaining)

    def _safe_stop_controls(self):
        with self._data_lock:
            self.data.ctrl[:] = 0.0
            self.stopped = True

    def _consume_fault(self):
        with self._metrics_lock:
            if self._fault_remaining < 1:
                return None, 0.0
            kind = self._fault_kind
            delay = self._fault_delay_seconds
            self._fault_remaining -= 1
            if self._fault_remaining == 0:
                self._fault_kind = None
                self._fault_delay_seconds = 0.0
            return kind, delay

    def _record_step(self, duration_seconds):
        with self._metrics_lock:
            self._step_count += 1
            self._total_step_duration_seconds += duration_seconds
            self._last_step_duration_seconds = duration_seconds
            self._max_step_duration_seconds = max(
                self._max_step_duration_seconds, duration_seconds
            )
            self._last_step_at = time.monotonic()
            if duration_seconds > self._target_period_seconds:
                self._step_overrun_count += 1

    def _reset_metrics(self):
        with self._metrics_lock:
            self._reset_metrics_locked()

    def _reset_metrics_locked(self):
        self._target_period_seconds = max(0.0001, float(self.model.opt.timestep))
        self._loop_started_at = time.monotonic()
        self._last_step_at = None
        self._step_count = 0
        self._step_overrun_count = 0
        self._step_failure_count = 0
        self._last_step_duration_seconds = 0.0
        self._total_step_duration_seconds = 0.0
        self._max_step_duration_seconds = 0.0
        self._last_error = None
