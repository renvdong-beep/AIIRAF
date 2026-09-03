import unittest

from iraf_core.store import SqliteExecutionStore


class ReplayManifestStoreTests(unittest.TestCase):
    def setUp(self):
        self.store = SqliteExecutionStore(":memory:")

    def test_builds_deterministic_redacted_manifest(self):
        result = {
            "execution_id": "execution-1",
            "correlation_id": "correlation-1",
            "status": "SUCCEEDED",
            "sequence": 3,
            "error_code": "",
            "reason": "",
            "requested_skill": {
                "name": "move_joint",
                "version_constraint": "1.0.0",
            },
            "skill": {"name": "move_joint", "version": "1.0.0", "digest": "sd"},
            "provider": {"name": "sim", "type": "python_adapter"},
            "profile": {"name": "robot", "version": "1.0.0", "digest": "pd"},
            "safety_policy": {"name": "lab", "version": "1.0.0", "digest": "sp"},
            "policy_decision_id": "decision-1",
            "policy_version": "1.0.0",
            "resource_id": "robot",
            "controller": "test",
            "simulation": True,
            "result": {"positions": {"j1": 0.2}},
        }
        for sequence, status in enumerate(
            ("PENDING", "VALIDATING", "RUNNING", "SUCCEEDED")
        ):
            self.store.append_event("execution-1", sequence, status)
        self.store.save_result(
            "operator@example", "secret-key", "request-digest", result
        )

        first = self.store.get_replay_manifest("execution-1")
        second = self.store.get_replay_manifest("execution-1")

        self.assertEqual(first, second)
        self.assertEqual("iraf.execution-replay/v1", first["schema_version"])
        self.assertEqual([0, 1, 2, 3], [item["sequence"] for item in first["events"]])
        self.assertEqual("SUCCEEDED", first["terminal_status"])
        self.assertTrue(first["simulation"])
        self.assertEqual(64, len(first["subject_digest"]))
        self.assertEqual(64, len(first["idempotency_key_digest"]))
        self.assertEqual(64, len(first["result_digest"]))
        self.assertEqual(64, len(first["event_digest"]))
        self.assertNotIn("subject", first)
        self.assertNotIn("idempotency_key", first)
        self.assertNotIn("result", first)
        self.assertNotIn("operator@example", str(first))
        self.assertNotIn("secret-key", str(first))

    def test_missing_execution_has_no_replay_manifest(self):
        self.assertIsNone(self.store.get_replay_manifest("missing"))

    def test_annotation_is_additive_and_idempotent(self):
        result = {
            "execution_id": "execution-2",
            "status": "FAILED",
            "sequence": 2,
        }
        self.store.save_result("subject", "key-2", "digest", result)
        metadata = {"adapter": "agentos-intent"}

        first = self.store.annotate_result("execution-2", metadata)
        second = self.store.annotate_result("execution-2", metadata)

        self.assertEqual(first, second)
        with self.assertRaisesRegex(ValueError, "不得覆盖"):
            self.store.annotate_result(
                "execution-2", {"adapter": "different-adapter"}
            )


if __name__ == "__main__":
    unittest.main()
