import unittest

from iraf_adapters.mujoco.supervisor import MujocoSimulationSupervisor


class FakeContinuousBackend:
    def __init__(self):
        self.started = 0
        self.stopped = 0
        self.continuous = False

    def start_continuous(self):
        self.started += 1
        self.continuous = True
        return True

    def stop_continuous(self):
        self.stopped += 1
        self.continuous = False
        return True

    def is_continuous(self):
        return self.continuous

    def simulation_status(self):
        return {
            "schema_version": "iraf.mujoco.simulation-status/v1",
            "simulation": True,
            "healthy": self.continuous,
            "running": self.continuous,
            "step_count": 3,
        }

    def inject_fault(self, kind, duration_ms=0, occurrences=1):
        return {
            "kind": kind,
            "duration_ms": duration_ms,
            "occurrences": occurrences,
        }

    def clear_faults(self):
        self.continuous = False
        return True


class NonContinuousBackend:
    pass


class MujocoSupervisorTests(unittest.TestCase):
    def test_start_stop_owns_backend_lifecycle(self):
        backend = FakeContinuousBackend()
        supervisor = MujocoSimulationSupervisor(backend)

        self.assertFalse(supervisor.running)
        self.assertTrue(supervisor.start())
        self.assertTrue(supervisor.running)
        self.assertEqual(backend.started, 1)
        self.assertFalse(supervisor.start())
        self.assertEqual(backend.started, 1)

        self.assertTrue(supervisor.stop())
        self.assertFalse(supervisor.running)
        self.assertEqual(backend.stopped, 1)
        self.assertFalse(supervisor.stop())

    def test_exposes_diagnostics_and_bounded_fault_contract(self):
        backend = FakeContinuousBackend()
        supervisor = MujocoSimulationSupervisor(backend)
        supervisor.start()

        self.assertEqual(3, supervisor.diagnostics()["step_count"])
        self.assertEqual(
            {"kind": "step_delay", "duration_ms": 20, "occurrences": 1},
            supervisor.inject_fault("step_delay", duration_ms=20),
        )
        supervisor.stop()
        self.assertTrue(supervisor.clear_faults())
        self.assertFalse(supervisor.running)

    def test_rejects_backend_without_continuous_contract(self):
        with self.assertRaisesRegex(RuntimeError, "连续仿真"):
            MujocoSimulationSupervisor(NonContinuousBackend()).start()


if __name__ == "__main__":
    unittest.main()
