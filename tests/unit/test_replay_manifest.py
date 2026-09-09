import unittest

from iraf_core.store import SqliteExecutionStore
from iraf_core.runtime import SkillRuntime


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

    def test_replay_projects_nested_contract_fields(self):
        secret = "provider-api-key-must-not-be-exported"
        result = {
            "execution_id": "execution-3",
            "status": "FAILED",
            "sequence": 2,
            "requested_skill": {"name": "stop", "credential": secret},
            "skill": {"name": "stop", "version": "1.0.0", "secret": secret},
            "provider": {"name": "sim", "type": "python_adapter", "token": secret},
            "profile": {"name": "robot", "version": "1.0.0", "private": secret},
            "safety_policy": {"name": "lab", "version": "1.0.0", "key": secret},
            "intent_provider": {
                "name": "edge",
                "version": "1.0.0",
                "model": "model",
                "api_key": secret,
            },
        }
        self.store.save_result("subject", "key-3", "digest", result)

        replay = self.store.get_replay_manifest("execution-3")

        self.assertNotIn(secret, str(replay))
        self.assertEqual({"name": "stop"}, replay["requested_skill"])
        self.assertEqual(
            {"name": "edge", "version": "1.0.0", "model": "model"},
            replay["intent_provider"],
        )

    def test_execution_metadata_contract_rejects_unsafe_nested_values(self):
        valid = {
            "adapter": "agentos-intent",
            "intent_provider": {
                "name": "test-provider",
                "version": "1.0.0",
                "model": "test-model",
            },
            "intent_request_digest": "a" * 64,
            "resolved_skill": "move_joint",
        }
        self.assertIsNone(SkillRuntime._validate_execution_metadata(valid))

        invalid_values = (
            {**valid, "intent_request_digest": "not-a-sha256"},
            {**valid, "intent_provider": {**valid["intent_provider"], "api_key": "secret"}},
            {**valid, "intent_provider": {"name": "missing-fields"}},
            {**valid, "adapter": {"name": "not-a-string"}},
        )
        for metadata in invalid_values:
            with self.subTest(metadata=metadata):
                with self.assertRaises(ValueError):
                    SkillRuntime._validate_execution_metadata(metadata)


if __name__ == "__main__":
    unittest.main()
