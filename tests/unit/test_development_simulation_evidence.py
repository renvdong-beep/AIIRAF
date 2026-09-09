import importlib.util
import os
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "scripts" / "verify_development_simulation.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("simulation_evidence", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DevelopmentSimulationEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_script()

    def test_runtime_environment_path_includes_tools(self):
        env = {"PYTHONPATH": os.pathsep.join(["first", "second"])}

        self.module._append_project_python_paths(env)

        paths = env["PYTHONPATH"].split(os.pathsep)
        self.assertEqual(["first", "second"], paths[:-1])
        self.assertEqual(str(PROJECT_ROOT / "tools"), paths[-1])

    def test_runtime_environment_configured_env_still_adds_tools(self):
        configured_env = {key: "configured" for key in self.module.RUNTIME_ENV_KEYS}
        configured_env["PYTHONPATH"] = "existing"

        with mock.patch.dict(self.module.os.environ, configured_env, clear=True):
            env, missing = self.module._runtime_environment()

        self.assertEqual([], missing)
        self.assertIn(str(PROJECT_ROOT / "tools"), env["PYTHONPATH"].split(os.pathsep))

    def test_run_command_records_success_and_digest(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_ROOT / "build") as directory:
            result = self.module._run_command(
                "success",
                [sys.executable, "-c", "print('ok')"],
                Path(directory),
                timeout_seconds=5,
            )

            self.assertTrue(result["passed"])
            self.assertEqual(0, result["exit_code"])
            self.assertEqual(64, len(result["log_sha256"]))
            log_path = PROJECT_ROOT / result["log"]
            self.assertIn("ok", log_path.read_text(encoding="utf-8"))

    def test_run_command_preserves_failure_as_evidence(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_ROOT / "build") as directory:
            result = self.module._run_command(
                "failure",
                [sys.executable, "-c", "import sys; print('拒绝'); sys.exit(7)"],
                Path(directory),
                timeout_seconds=5,
            )

            self.assertFalse(result["passed"])
            self.assertEqual(7, result["exit_code"])
            self.assertFalse(result["timed_out"])
            log_path = PROJECT_ROOT / result["log"]
            self.assertIn("拒绝", log_path.read_text(encoding="utf-8"))

    def test_artifact_records_content_digest(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_ROOT / "build") as directory:
            path = Path(directory) / "report.json"
            path.write_text("{}\n", encoding="utf-8")

            artifact = self.module._artifact(path, "verification_report")

            self.assertEqual("verification_report", artifact["kind"])
            self.assertEqual(3, artifact["size_bytes"])
            self.assertEqual(self.module._sha256(path), artifact["sha256"])

    def test_evidence_output_is_restricted_to_repository_outputs(self):
        self.assertTrue(
            self.module._is_evidence_path(
                PROJECT_ROOT / "build" / "acceptance" / "simulation"
            )
        )
        self.assertFalse(
            self.module._is_evidence_path(PROJECT_ROOT.parent / "simulation")
        )

    def test_online_intent_replay_requires_identity_and_complete_events(self):
        report = {
            "schema_version": "iraf.execution-replay/v1",
            "execution_id": "execution-1",
            "terminal_status": "SUCCEEDED",
            "simulation": True,
            "adapter": "agentos-intent",
            "intent_provider": {"name": "qwen", "model": "Qwen3-0.6B"},
            "intent_request_digest": "a" * 64,
            "resolved_skill": "move_joint",
            "events": [
                {"status": status}
                for status in ("PENDING", "VALIDATING", "RUNNING", "SUCCEEDED")
            ],
        }

        checks = self.module._successful_replay_checks(
            report, "execution-1", require_intent=True
        )

        self.assertTrue(all(checks.values()))
        report["intent_provider"] = {}
        report["events"].pop()
        failed = self.module._successful_replay_checks(
            report, "execution-1", require_intent=True
        )
        self.assertFalse(failed["intent_provider"])
        self.assertFalse(failed["events"])


if __name__ == "__main__":
    unittest.main()
