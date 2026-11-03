# -*- coding: utf-8 -*-
"""执行事实的归属读取（2026-11-03）：跨调用方不得读到别人的 execution。"""
import tempfile
import unittest
from pathlib import Path

from iraf_core.store import SqliteExecutionStore


class ExecutionOwnershipTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = SqliteExecutionStore(str(Path(self.tmp.name) / "events.db"))
        self.result = {"execution_id": "exec-1", "status": "SUCCEEDED",
                       "error_code": "", "reason": ""}
        self.store.save_result("alice", "idem-1", "digest-1", self.result)

    def tearDown(self):
        self.tmp.cleanup()

    def test_owner_can_read(self):
        got = self.store.get_for_subject("exec-1", "alice")
        self.assertEqual(got["status"], "SUCCEEDED")

    def test_other_subject_gets_none(self):
        self.assertIsNone(self.store.get_for_subject("exec-1", "bob"))

    def test_empty_subject_gets_none(self):
        self.assertIsNone(self.store.get_for_subject("exec-1", ""))

    def test_unknown_execution_gets_none(self):
        self.assertIsNone(self.store.get_for_subject("exec-missing", "alice"))


if __name__ == "__main__":
    unittest.main()
