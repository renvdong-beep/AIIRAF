import importlib.util
import tempfile
import unittest
from pathlib import Path

from iraf_core.store import SqliteExecutionStore


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "scripts" / "export_replay_manifest.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("export_replay", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ExportReplayManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_script()

    def test_exports_manifest_and_checksum(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_ROOT / "build") as directory:
            root = Path(directory)
            database = root / "events.db"
            output = root / "replay.json"
            store = SqliteExecutionStore(database)
            store.append_event("execution-1", 0, "PENDING")
            store.append_event("execution-1", 1, "FAILED", "rejected")
            store.save_result(
                "subject",
                "key",
                "request-digest",
                {
                    "execution_id": "execution-1",
                    "status": "FAILED",
                    "sequence": 1,
                    "error_code": "IRAF-INPUT-INVALID",
                    "reason": "rejected",
                    "simulation": True,
                },
            )

            exit_code = self.module.main(
                [
                    "--execution-id",
                    "execution-1",
                    "--event-store",
                    str(database),
                    "--output",
                    str(output),
                ]
            )

            self.assertEqual(0, exit_code)
            self.assertTrue(output.is_file())
            checksum = output.with_suffix(".json.sha256").read_text(encoding="ascii")
            self.assertIn(self.module._sha256(output), checksum)
            self.assertIn(self.module._display_path(output), checksum)

    def test_missing_execution_returns_not_found(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_ROOT / "build") as directory:
            root = Path(directory)
            database = root / "events.db"
            SqliteExecutionStore(database)

            exit_code = self.module.main(
                [
                    "--execution-id",
                    "missing",
                    "--event-store",
                    str(database),
                    "--output",
                    str(root / "missing.json"),
                ]
            )

            self.assertEqual(2, exit_code)
            self.assertFalse((root / "missing.json").exists())


if __name__ == "__main__":
    unittest.main()
