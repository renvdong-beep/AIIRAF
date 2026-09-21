"""wheelhouse 修剪工具的负/正路径测试（离线）。

覆盖：清单缺失/为空即失败、只删未登记项、--verify 在有残留时退出 4、--dry-run 不落盘、
登记项一个都不删、目录不存在即失败。
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deploy" / "sdk"))

import prune_wheelhouse as prune  # noqa: E402


def make_dir(base: Path, manifest: dict | None, wheels: list) -> Path:
    directory = base / "wheelhouse"
    directory.mkdir(parents=True, exist_ok=True)
    for name in wheels:
        (directory / name).write_bytes(b"x")
    if manifest is not None:
        (directory / "wheelhouse.json").write_text(json.dumps(manifest), encoding="utf-8")
    return directory


class PruneTests(unittest.TestCase):
    def test_missing_manifest_fails_explicitly(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = make_dir(Path(tmp), None, ["a-1.0-py3-none-any.whl"])
            with self.assertRaises(prune.PruneError):
                prune.load_registered_names(directory)
            self.assertEqual(prune.main(["--dir", str(directory)]), 2)

    def test_empty_planned_list_fails_instead_of_deleting_everything(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = make_dir(Path(tmp), {"planned": []}, ["a-1.0-py3-none-any.whl"])
            self.assertEqual(prune.main(["--dir", str(directory)]), 2)
            self.assertTrue((directory / "a-1.0-py3-none-any.whl").is_file(), "清单为空时不得删任何文件")

    def test_only_unregistered_wheels_are_removed(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {"planned": [{"name": "a", "filename": "a-2.0-py3-none-any.whl", "resolved": True}]}
            directory = make_dir(Path(tmp), manifest, [
                "a-1.0-py3-none-any.whl",   # 陈旧
                "a-2.0-py3-none-any.whl",   # 登记
                "b-9.9-py3-none-any.whl",   # 未声明包（同样属未登记 → 删）
            ])
            rc = prune.main(["--dir", str(directory)])
            self.assertEqual(rc, 0)
            self.assertFalse((directory / "a-1.0-py3-none-any.whl").exists())
            self.assertFalse((directory / "b-9.9-py3-none-any.whl").exists())
            self.assertTrue((directory / "a-2.0-py3-none-any.whl").is_file())

    def test_dry_run_touches_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {"planned": [{"name": "a", "filename": "a-2.0-py3-none-any.whl"}]}
            directory = make_dir(Path(tmp), manifest, ["a-1.0-py3-none-any.whl", "a-2.0-py3-none-any.whl"])
            self.assertEqual(prune.main(["--dir", str(directory), "--dry-run"]), 0)
            self.assertTrue((directory / "a-1.0-py3-none-any.whl").is_file())

    def test_verify_mode_reports_inconsistency_without_deleting(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {"planned": [{"name": "a", "filename": "a-2.0-py3-none-any.whl"}]}
            directory = make_dir(Path(tmp), manifest, ["a-1.0-py3-none-any.whl", "a-2.0-py3-none-any.whl"])
            self.assertEqual(prune.main(["--dir", str(directory), "--verify"]), 4)
            self.assertTrue((directory / "a-1.0-py3-none-any.whl").is_file(), "--verify 不得删除文件")

    def test_verify_mode_passes_on_clean_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {"planned": [{"name": "a", "filename": "a-2.0-py3-none-any.whl"}]}
            directory = make_dir(Path(tmp), manifest, ["a-2.0-py3-none-any.whl"])
            self.assertEqual(prune.main(["--dir", str(directory), "--verify"]), 0)

    def test_missing_directory_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(prune.main(["--dir", str(Path(tmp) / "nope")]), 2)


if __name__ == "__main__":
    unittest.main()
