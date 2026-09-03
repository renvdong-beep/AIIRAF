import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_adapters.mujoco.supervisor import MujocoSimulationSupervisor


class FakeControl:
    def __init__(self):
        self.zeroed = False

    def __setitem__(self, key, value):
        if isinstance(key, slice) and value == 0.0:
            self.zeroed = True


class MujocoBackendMetricsTests(unittest.TestCase):
    def _backend(self, fault_injection_enabled=True):
        model = SimpleNamespace(nu=0, opt=SimpleNamespace(timestep=0.005))
        data = SimpleNamespace(ctrl=FakeControl())
        profile = SimpleNamespace(joints=())
        patches = (
            patch(
                "iraf_adapters.mujoco.mujoco_backend.mujoco.MjModel",
                SimpleNamespace(from_xml_path=lambda path: model),
            ),
            patch(
                "iraf_adapters.mujoco.mujoco_backend.mujoco.MjData",
                return_value=data,
            ),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_step"),
        )
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        return MujocoBackend(
            "model.xml",
            profile,
            authority=SimpleNamespace(),
            fault_injection_enabled=fault_injection_enabled,
        )

    def _wait_for(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        self.fail("等待仿真状态变化超时")

    def test_reports_frequency_latency_and_overrun(self):
        backend = self._backend()
        supervisor = MujocoSimulationSupervisor(backend)
        self.addCleanup(supervisor.stop)
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 3)

        backend.inject_fault("step_delay", duration_ms=20)
        self._wait_for(
            lambda: supervisor.diagnostics()["step_overrun_count"] >= 1
        )
        status = supervisor.diagnostics()

        self.assertTrue(status["healthy"])
        self.assertEqual(200.0, status["target_frequency_hz"])
        self.assertGreater(status["effective_frequency_hz"], 0)
        self.assertGreaterEqual(status["max_step_duration_ms"], 20)

    def test_step_failure_is_fail_closed_and_requires_explicit_clear(self):
        backend = self._backend()
        supervisor = MujocoSimulationSupervisor(backend)
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 1)

        backend.inject_fault("step_failure")
        self._wait_for(lambda: supervisor.diagnostics()["last_error"] is not None)
        status = supervisor.diagnostics()

        self.assertFalse(status["healthy"])
        self.assertFalse(status["running"])
        self.assertEqual(1, status["step_failure_count"])
        self.assertTrue(backend.stopped)
        self.assertTrue(backend.data.ctrl.zeroed)
        with self.assertRaisesRegex(RuntimeError, "尚未清除"):
            supervisor.start()

        self.assertTrue(supervisor.stop())
        self.assertTrue(supervisor.clear_faults())
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 1)
        self.assertTrue(supervisor.diagnostics()["healthy"])
        self.assertTrue(supervisor.stop())

    def test_fault_injection_is_disabled_by_default(self):
        backend = self._backend(fault_injection_enabled=False)

        with self.assertRaisesRegex(PermissionError, "未启用"):
            backend.inject_fault("step_failure")


if __name__ == "__main__":
    unittest.main()
