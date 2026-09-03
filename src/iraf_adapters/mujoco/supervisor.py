"""单一长周期 MuJoCo 仿真循环的生命周期适配器。"""

import threading


class MujocoSimulationSupervisor:
    """持有连续步进生命周期，Runtime 继续持有命令和安全权限。"""

    def __init__(self, backend):
        self.backend = backend
        self._lock = threading.RLock()
        self._running = False

    def start(self):
        if not hasattr(self.backend, "start_continuous"):
            raise RuntimeError("配置的 Backend 不支持连续仿真")
        with self._lock:
            if self.running:
                return False
            started = self.backend.start_continuous()
            self._running = bool(
                started
                or getattr(self.backend, "is_continuous", lambda: False)()
            )
            return bool(started)

    def stop(self):
        with self._lock:
            if not self._running:
                return False
            stopped = self.backend.stop_continuous()
            self._running = not bool(stopped)
            return bool(stopped)

    @property
    def running(self):
        with self._lock:
            return bool(
                self._running
                and getattr(self.backend, "is_continuous", lambda: False)()
            )

    def diagnostics(self):
        if hasattr(self.backend, "simulation_status"):
            return self.backend.simulation_status()
        return {
            "schema_version": "iraf.mujoco.simulation-status/v1",
            "simulation": True,
            "healthy": self.running,
            "running": self.running,
        }

    def inject_fault(self, kind, duration_ms=0, occurrences=1):
        if not hasattr(self.backend, "inject_fault"):
            raise RuntimeError("配置的 Backend 不支持仿真故障注入")
        return self.backend.inject_fault(kind, duration_ms, occurrences)

    def clear_faults(self):
        if not hasattr(self.backend, "clear_faults"):
            raise RuntimeError("配置的 Backend 不支持清除仿真故障")
        cleared = self.backend.clear_faults()
        with self._lock:
            self._running = False
        return bool(cleared)
