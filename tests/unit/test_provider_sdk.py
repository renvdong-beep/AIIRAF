# -*- coding: utf-8 -*-
"""南向 Provider SDK：lease/fencing/幂等/能力契约（不依赖 grpc，纯逻辑）。"""
import tempfile
import time
import unittest
from pathlib import Path

from iraf_sdk.provider import (CAPABILITY_METHODS, LeaseRejected, ProviderKit)


class FakeBackend:
    capabilities = {"move_joint", "pick_object", "read_state"}
    display_mode = "offscreen_frames"

    def __init__(self):
        self.calls = []

    def runtime_inventory(self):
        return {"mode": "simulation", "safety": {"estop": False},
                "manipulation": {"target_visible": True}}

    def move_joint(self, positions, duration_ms, lease):
        self.calls.append(("move_joint", positions, duration_ms, lease.fencing_token))
        return {"moved": True}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.calls.append(("pick_object", target_id))
        return {"lifted": True, "target_body": target_id}

    def read_state(self, lease):
        return {"qpos": [0.0]}


def lease(**overrides):
    data = {
        "execution_id": "exec-1",
        "idempotency_key": "idem-1",
        "correlation_id": "corr-1",
        "resource_id": "robot-a",
        "fencing_token": 1,
        "deadline_unix_ms": int(time.time() * 1000) + 60_000,
    }
    data.update(overrides)
    return data


class ProviderKitTest(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.kit = ProviderKit(self.backend, token="secret", provider_id="acme/mujoco",
                               resource_id="robot-a")

    def test_health_reports_inventory_and_capabilities(self):
        health = self.kit.health()
        self.assertTrue(health["ready"])
        self.assertTrue(health["simulation"])
        self.assertEqual(health["capabilities"], ["move_joint", "pick_object", "read_state"])
        self.assertEqual(health["inventory"]["safety"]["estop"], False)
        self.assertEqual(health["display_mode"], "offscreen_frames")

    def test_invoke_success_and_kwargs_filtering(self):
        result = self.kit.invoke("move_joint",
                                 {"positions": {"j1": 0.1}, "duration_ms": 500, "extra": 1},
                                 lease())
        self.assertTrue(result["accepted"])
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertEqual(result["outputs"], {"moved": True})
        self.assertEqual(self.backend.calls[0][0], "move_joint")

    def test_idempotent_replay(self):
        first = self.kit.invoke("move_joint", {"positions": {}, "duration_ms": 100}, lease())
        second = self.kit.invoke("move_joint", {"positions": {}, "duration_ms": 100}, lease())
        self.assertTrue(second.get("idempotent_replay"))
        self.assertEqual(first["outputs"], second["outputs"])
        self.assertEqual(len(self.backend.calls), 1)

    def test_missing_deadline_rejected(self):
        result = self.kit.invoke("move_joint", {}, lease(deadline_unix_ms=0))
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error_code"], "IRAF-INPUT-INVALID")

    def test_expired_deadline_rejected(self):
        result = self.kit.invoke("move_joint", {}, lease(deadline_unix_ms=int(time.time() * 1000) - 1))
        self.assertEqual(result["error_code"], "IRAF-DEADLINE-EXCEEDED")

    def test_stale_fencing_rejected(self):
        self.kit.invoke("move_joint", {}, lease(fencing_token=5))
        result = self.kit.invoke("move_joint", {},
                                 lease(execution_id="exec-2", idempotency_key="idem-2",
                                       fencing_token=4))
        self.assertEqual(result["error_code"], "IRAF-EXECUTION-CONFLICT")

    def test_terminal_execution_rejects_new_control(self):
        self.kit.invoke("move_joint", {"positions": {}, "duration_ms": 10}, lease())
        result = self.kit.invoke("move_joint", {"positions": {}, "duration_ms": 10},
                                 lease(idempotency_key="idem-2", fencing_token=2))
        self.assertEqual(result["error_code"], "IRAF-EXECUTION-CONFLICT")

    def test_undeclared_capability_rejected_even_if_implemented(self):
        backend = FakeBackend()
        kit = ProviderKit(backend, token="t", provider_id="p",
                          declared_capabilities={"read_state"})
        result = kit.invoke("move_joint", {}, lease())
        self.assertEqual(result["error_code"], "IRAF-POLICY-DENIED")

    def test_unknown_capability_rejected(self):
        result = self.kit.invoke("fly", {}, lease())
        self.assertEqual(result["error_code"], "IRAF-CAPABILITY-UNKNOWN")

    def test_declared_but_unimplemented_fails_startup(self):
        with self.assertRaises(ValueError) as caught:
            ProviderKit(FakeBackend(), token="t", provider_id="p",
                        declared_capabilities={"dock_for_handoff"})
        self.assertIn("未实现", str(caught.exception))

    def test_backend_error_becomes_structured_failure(self):
        class Broken(FakeBackend):
            capabilities = {"move_joint"}

            def move_joint(self, positions, duration_ms, lease):
                raise RuntimeError("boom")

        kit = ProviderKit(Broken(), token="t", provider_id="p")
        result = kit.invoke("move_joint", {"positions": {}, "duration_ms": 10}, lease())
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error_code"], "IRAF-PROVIDER-ERROR")
        self.assertIn("boom", result["error_message"])

    def test_non_mapping_output_rejected(self):
        class Bad(FakeBackend):
            capabilities = {"read_state"}

            def read_state(self, lease):
                return [1, 2]

        kit = ProviderKit(Bad(), token="t", provider_id="p")
        result = kit.invoke("read_state", {}, lease())
        self.assertEqual(result["error_code"], "IRAF-PROVIDER-ERROR")

    def test_auth(self):
        self.kit.authenticate("Bearer secret")
        with self.assertRaises(LeaseRejected) as caught:
            self.kit.authenticate("Bearer wrong")
        self.assertEqual(caught.exception.code, "IRAF-UNAUTHENTICATED")

    def test_capability_vocabulary_matches_edge(self):
        from iraf_adapters.factory import CAPABILITY_METHODS as EDGE

        self.assertEqual(dict(CAPABILITY_METHODS), dict(EDGE))

    def test_sqlite_idempotency_survives_new_kit(self):
        with tempfile.TemporaryDirectory() as tmp:
            db = str(Path(tmp) / "idem.db")
            kit = ProviderKit(FakeBackend(), token="t", provider_id="p", idempotency_db=db)
            first = kit.invoke("move_joint", {"positions": {}, "duration_ms": 10}, lease())
            kit2 = ProviderKit(FakeBackend(), token="t", provider_id="p", idempotency_db=db)
            second = kit2.invoke("move_joint", {"positions": {}, "duration_ms": 10}, lease())
            self.assertTrue(second.get("idempotent_replay"))
            self.assertEqual(first["outputs"], second["outputs"])


if __name__ == "__main__":
    unittest.main()
