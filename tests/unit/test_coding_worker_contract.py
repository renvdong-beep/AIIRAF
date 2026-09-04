import unittest

import coding_worker


class CodingTaskContractTest(unittest.TestCase):
    def test_accepts_allowlisted_task(self):
        task = {
            "version": 1,
            "id": "profile-contract-001",
            "title": "test: validate coding task contract",
            "verification_profile": "unit",
            "patch": "change.patch",
        }
        self.assertEqual(coding_worker.verify_task(task), task)

    def test_rejects_arbitrary_verification_profile(self):
        task = {
            "version": 1,
            "id": "profile-contract-002",
            "title": "test: reject arbitrary command",
            "verification_profile": "shell",
            "patch": "change.patch",
        }
        with self.assertRaises(ValueError):
            coding_worker.verify_task(task)

    def test_rejects_parent_path_patch(self):
        task = {
            "version": 1,
            "id": "profile-contract-003",
            "title": "test: reject path traversal",
            "verification_profile": "unit",
            "patch": "../change.patch",
        }
        with self.assertRaises(ValueError):
            coding_worker.verify_task(task)


if __name__ == "__main__":
    unittest.main()
