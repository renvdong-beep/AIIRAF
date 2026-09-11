import unittest

from scripts.build_piper_pick_scene import _validate_support_height


class PiperSceneGeometryTests(unittest.TestCase):
    def test_target_bottom_matches_workbench_top(self):
        _validate_support_height(0.03, 0.03, 0.0)

    def test_rejects_floating_target(self):
        with self.assertRaisesRegex(ValueError, "未贴合工作台"):
            _validate_support_height(0.055, 0.03, 0.0)


if __name__ == "__main__":
    unittest.main()
