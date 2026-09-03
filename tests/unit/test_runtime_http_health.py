import unittest

from iraf_adapters.http.runtime_http import RuntimeHandler


class FakeSupervisor:
    def __init__(self, healthy):
        self.running = healthy
        self._healthy = healthy

    def diagnostics(self):
        return {
            "schema_version": "iraf.mujoco.simulation-status/v1",
            "simulation": True,
            "healthy": self._healthy,
            "running": self.running,
            "step_count": 12,
        }


class RuntimeHttpHealthTests(unittest.TestCase):
    def _handler(self, path, healthy=True, authorized=False):
        handler = object.__new__(RuntimeHandler)
        handler.path = path
        handler.supervisor = FakeSupervisor(healthy)
        handler.bridge = None
        handler.token = "token"
        handler.subject = "test"
        handler.headers = {
            "Authorization": "Bearer token" if authorized else ""
        }
        responses = []
        handler._json = lambda status, body: responses.append((status, body))
        return handler, responses

    def test_health_is_degraded_when_simulation_loop_failed(self):
        handler, responses = self._handler("/health", healthy=False)

        handler.do_GET()

        status, body = responses[0]
        self.assertEqual(503, status)
        self.assertEqual("degraded", body["status"])
        self.assertFalse(body["continuous_simulation"])
        self.assertFalse(body["simulation"]["healthy"])

    def test_simulation_metrics_require_authentication(self):
        handler, responses = self._handler("/v1/simulation/metrics")
        handler.do_GET()
        self.assertEqual(401, responses[0][0])

        handler, responses = self._handler(
            "/v1/simulation/metrics", authorized=True
        )
        handler.do_GET()
        self.assertEqual(200, responses[0][0])
        self.assertEqual(12, responses[0][1]["step_count"])


if __name__ == "__main__":
    unittest.main()
