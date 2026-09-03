import importlib.util
import sys
import tempfile
import unittest
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


if __name__ == "__main__":
    unittest.main()
