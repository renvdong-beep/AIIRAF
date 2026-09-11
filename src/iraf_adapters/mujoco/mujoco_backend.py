"""带连续步进、指标和受控故障注入的 MuJoCo 3 后端。"""

from pathlib import Path
import math
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
            manipulation_config=config.get("manipulation"),
            realtime=bool(config.get("realtime", False)),
        )

    def __init__(
        self,
        model_path,
        profile,
        authority,
        fault_injection_enabled=False,
        manipulation_config=None,
        realtime=False,
    ):
        self.profile = profile
        self.authority = authority
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path)))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
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
        self._manipulation = self._parse_manipulation_config(manipulation_config)
        self._realtime = bool(realtime)
        self._reset_metrics()

    def runtime_inventory(self):
        return {
            "safety": {"estop": False},
            "manipulation": {
                "target_visible": bool(self._manipulation["targets"])
            },
            "mode": "simulation",
        }

    def calibrate_grasp(self, config=None, lease=None):
        """读取当前模型的腕部和真实指尖 geom，生成可审计标定证据。"""
        self.authority.validate(lease)
        def body(name):
            return self._body_id(name)
        wrist, left, right = body("link6"), body("link7"), body("link8")
        def geom(name, fallback):
            ident = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            return self.data.geom_xpos[ident].copy() if ident >= 0 else self.data.xpos[fallback].copy()
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            w = self.data.xpos[wrist].copy(); l = geom("piper_left_finger", left); r = geom("piper_right_finger", right)
        center = (l + r) / 2.0; offset = center - w; norm = max(float((offset @ offset) ** 0.5), 1e-9)
        return {"wrist_position_m": w.tolist(), "left_finger_position_m": l.tolist(), "right_finger_position_m": r.tolist(), "grasp_center_m": center.tolist(), "finger_separation_m": float(((l-r) @ (l-r)) ** 0.5), "tcp_offset_from_link6_m": offset.tolist(), "approach_axis_world": (offset / norm).tolist(), "recommended_pregrasp_offset_m": 0.04}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        """闭合双指并以目标和两侧手指的真实接触作为抓取确认。"""
        self.authority.validate(lease)
        target = self._manipulation["targets"].get(target_id)
        gripper = self._manipulation["gripper"]
        if target is None:
            raise ValueError("MuJoCo 场景中没有受控目标: " + str(target_id))
        if gripper is None:
            raise RuntimeError("MuJoCo Backend 未配置 Piper 夹爪接触信息")
        if grasp_pose.get("frame_id") != "world":
            raise ValueError("MuJoCo 抓取当前只接受 world 坐标系")

        target_body = self._body_id(target["body"])
        left_body = self._body_id(gripper["left_finger_body"])
        right_body = self._body_id(gripper["right_finger_body"])
        requested = grasp_pose["position"]
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            actual = tuple(float(value) for value in self.data.xpos[target_body])
        distance = math.sqrt(
            sum(
                (actual[index] - float(requested[key])) ** 2
                for index, key in enumerate(("x", "y", "z"))
            )
        )
        if distance > target["pose_tolerance_m"]:
            raise ValueError(
                "抓取位姿与目标位置不一致: "
                f"distance={distance:.6f}m tolerance={target['pose_tolerance_m']:.6f}m"
            )

        duration_ms = max(1, int(duration_ms))
        open_ms = max(1, duration_ms * 2 // 5)
        close_ms = max(1, duration_ms - open_ms)
        lift_ms = 0
        if gripper.get("lift_positions"):
            lift_ms = max(1, duration_ms // 3)
            close_ms = max(1, duration_ms - open_ms - lift_ms)
        self._set_controls(gripper["open_positions"])
        self._advance_for(open_ms)
        self._set_controls(gripper["closed_positions"])
        bilateral = self._advance_for(
            close_ms,
            contact_bodies=(target_body, left_body, right_body),
        )
        force_evidence = self._contact_force_evidence(
            target_body, left_body, right_body
        )
        force_ok = (
            bilateral
            and force_evidence["left_normal_force_n"]
            >= gripper["min_normal_force_n"]
            and force_evidence["right_normal_force_n"]
            >= gripper["min_normal_force_n"]
            and force_evidence["force_imbalance_ratio"]
            <= gripper["max_force_imbalance_ratio"]
        )
        constraint_activated = False
        equality_name = gripper.get("lift_constraint")
        if force_ok and equality_name:
            equality_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name
            )
            if equality_id < 0:
                raise ValueError("抓取约束不存在: " + str(equality_name))
            self.data.eq_active[equality_id] = 1
            constraint_activated = True
        with self._data_lock:
            before_lift_z = float(self.data.xpos[target_body][2])
        lifted = True
        if lift_ms:
            self._set_controls(gripper["lift_positions"])
            if constraint_activated and gripper.get("lift_anchor_body"):
                self._advance_with_grasp_anchor(lift_ms, target_body, left_body, right_body, gripper["lift_anchor_body"])
            else:
                self._advance_for(lift_ms, contact_bodies=(target_body, left_body, right_body))
            with self._data_lock:
                after_lift_z = float(self.data.xpos[target_body][2])
            lifted = (
                after_lift_z - before_lift_z
                >= gripper["min_lift_delta_m"]
                and (
                    constraint_activated
                    or self._has_bilateral_contact(target_body, left_body, right_body)
                )
            )
        self.stopped = False
        return {
            "target_id": target_id,
            "grasped": bool(force_ok and lifted),
            "confirmation": "constraint" if constraint_activated else "contact",
            "evidence": {
                "target_body": target["body"],
                "left_finger_body": gripper["left_finger_body"],
                "right_finger_body": gripper["right_finger_body"],
                "bilateral_contact": bool(bilateral),
                **force_evidence,
                "force_ok": bool(force_ok),
                "lifted": bool(lifted),
                "constraint_activated": constraint_activated,
                "lift_delta_m": round(
                    (float(self.data.xpos[target_body][2]) - before_lift_z), 6
                ),
            },
        }

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
            if self._realtime:
                time.sleep(float(self.model.opt.timestep))
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

    def _set_controls(self, positions):
        with self._data_lock:
            for actuator, value in positions.items():
                if actuator not in self._actuators:
                    raise ValueError("actuator not found: " + actuator)
                self.data.ctrl[self._actuators[actuator]] = float(value)

    def _advance_for(self, duration_ms, contact_bodies=None):
        bilateral = False
        deadline = time.monotonic() + max(1, int(duration_ms)) / 1000.0
        if self._continuous_mode:
            while not self._cancel_event.is_set() and time.monotonic() < deadline:
                if contact_bodies and self._has_bilateral_contact(*contact_bodies):
                    bilateral = True
                time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
            return bilateral

        steps = max(1, int(math.ceil((duration_ms / 1000.0) / self.model.opt.timestep)))
        for _ in range(steps):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            self.step()
            if contact_bodies and self._has_bilateral_contact(*contact_bodies):
                bilateral = True
        return bilateral

    def _advance_with_grasp_anchor(self, duration_ms, target_body, left_body, right_body, anchor_name):
        anchor_body = self._body_id(anchor_name)
        mocap_id = int(self.model.body_mocapid[anchor_body])
        if mocap_id < 0:
            raise ValueError("抓取中点必须是 mocap body: " + str(anchor_name))
        steps = max(1, int(math.ceil((duration_ms / 1000.0) / self.model.opt.timestep)))
        for _ in range(steps):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            with self._data_lock:
                self.data.mocap_pos[mocap_id] = (self.data.xpos[left_body] + self.data.xpos[right_body]) / 2.0
                self.data.mocap_quat[mocap_id] = (1.0, 0.0, 0.0, 0.0)
            self.step()

    def _has_bilateral_contact(self, target_body, left_body, right_body):
        contacted = set()
        with self._data_lock:
            for contact in self.data.contact[: self.data.ncon]:
                first = int(self.model.geom_bodyid[contact.geom1])
                second = int(self.model.geom_bodyid[contact.geom2])
                if first == target_body:
                    contacted.add(second)
                elif second == target_body:
                    contacted.add(first)
        return left_body in contacted and right_body in contacted

    def _contact_force_evidence(self, target_body, left_body, right_body):
        import numpy as np

        forces = {left_body: [], right_body: []}
        with self._data_lock:
            for index in range(self.data.ncon):
                contact = self.data.contact[index]
                bodies = (
                    int(self.model.geom_bodyid[contact.geom1]),
                    int(self.model.geom_bodyid[contact.geom2]),
                )
                finger = left_body if left_body in bodies else right_body if right_body in bodies else None
                if finger is None or target_body not in bodies:
                    continue
                result = np.zeros(6, dtype=float)
                mujoco.mj_contactForce(self.model, self.data, index, result)
                forces[finger].append(max(0.0, float(result[0])))
        left = max(forces[left_body] or [0.0])
        right = max(forces[right_body] or [0.0])
        low = min(left, right)
        ratio = max(left, right) / low if low > 1e-9 else float("inf")
        return {
            "left_normal_force_n": round(left, 6),
            "right_normal_force_n": round(right, 6),
            "force_imbalance_ratio": round(ratio, 6) if math.isfinite(ratio) else None,
        }

    def _body_id(self, name):
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, str(name)
        )
        if body_id < 0:
            raise ValueError("body not found: " + str(name))
        return int(body_id)

    @staticmethod
    def _parse_manipulation_config(config):
        if config is None:
            return {"targets": {}, "gripper": None}
        if not isinstance(config, dict):
            raise ValueError("manipulation 配置必须是对象")
        raw_targets = config.get("targets") or {}
        if not isinstance(raw_targets, dict):
            raise ValueError("manipulation.targets 必须是对象")
        targets = {}
        for target_id, item in raw_targets.items():
            if not isinstance(item, dict) or not item.get("body"):
                raise ValueError("每个 MuJoCo 目标必须声明 body")
            tolerance = float(item.get("pose_tolerance_m", 0.03))
            if not math.isfinite(tolerance) or tolerance <= 0:
                raise ValueError("目标 pose_tolerance_m 必须是正有限数")
            targets[str(target_id)] = {
                "body": str(item["body"]),
                "pose_tolerance_m": tolerance,
            }
        raw_gripper = config.get("gripper")
        if raw_gripper is None:
            gripper = None
        else:
            required = {
                "left_finger_body",
                "right_finger_body",
                "open_positions",
                "closed_positions",
            }
            missing = sorted(required - set(raw_gripper))
            if missing:
                raise ValueError("Piper 夹爪配置缺少字段: " + str(missing))
            open_positions = dict(raw_gripper["open_positions"])
            closed_positions = dict(raw_gripper["closed_positions"])
            if set(open_positions) != set(closed_positions) or not open_positions:
                raise ValueError("夹爪开合执行器配置必须一致且非空")
            gripper = {
                "left_finger_body": str(raw_gripper["left_finger_body"]),
                "right_finger_body": str(raw_gripper["right_finger_body"]),
                "open_positions": {
                    str(key): float(value) for key, value in open_positions.items()
                },
                "closed_positions": {
                    str(key): float(value) for key, value in closed_positions.items()
                },
            }
            if raw_gripper.get("lift_positions") is not None:
                lift_positions = dict(raw_gripper["lift_positions"])
                if not set(open_positions).issubset(lift_positions):
                    raise ValueError("抬升执行器配置必须包含夹爪执行器")
                min_lift = float(raw_gripper.get("min_lift_delta_m", 0.02))
                if not math.isfinite(min_lift) or min_lift <= 0:
                    raise ValueError("min_lift_delta_m 必须是正有限数")
                gripper["lift_positions"] = {
                    str(key): float(value) for key, value in lift_positions.items()
                }
                gripper["min_lift_delta_m"] = min_lift
                min_force = float(raw_gripper.get("min_normal_force_n", 0.2))
                max_ratio = float(raw_gripper.get("max_force_imbalance_ratio", 4.0))
                if not math.isfinite(min_force) or min_force <= 0:
                    raise ValueError("min_normal_force_n 必须是正有限数")
                if not math.isfinite(max_ratio) or max_ratio < 1:
                    raise ValueError("max_force_imbalance_ratio 必须不小于 1")
                gripper["min_normal_force_n"] = min_force
                gripper["max_force_imbalance_ratio"] = max_ratio
            else:
                gripper["lift_positions"] = None
                gripper["min_lift_delta_m"] = 0.02
            # 保证没有抬升动作的旧配置也能走统一的力闭环判定。
            gripper.setdefault("min_normal_force_n", 0.2)
            gripper.setdefault("max_force_imbalance_ratio", 4.0)
            constraint = raw_gripper.get("lift_constraint")
            if constraint is not None:
                if not isinstance(constraint, str) or not constraint:
                    raise ValueError("lift_constraint 必须是非空字符串")
                gripper["lift_constraint"] = constraint
            anchor_body = raw_gripper.get("lift_anchor_body")
            if anchor_body is not None:
                if not isinstance(anchor_body, str) or not anchor_body:
                    raise ValueError("lift_anchor_body 必须是非空字符串")
                gripper["lift_anchor_body"] = anchor_body
        return {"targets": targets, "gripper": gripper}

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
