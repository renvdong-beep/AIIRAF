import tempfile
import unittest
from pathlib import Path

from iraf_core.profile import ProfileError, load_robot_profile


class RobotProfileValidationTests(unittest.TestCase):
    def write_profile(self, text):
        handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        handle.write(text)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_valid_profile_preserves_capabilities_and_simulation_flag(self):
        path = self.write_profile("""
kind: RobotProfile
metadata:
  name: test-arm
  version: 1.0.0
spec:
  simulation: true
  control_frequency_hz: 100
  joints: [joint1]
  joint_limits:
    joint1: [-1.0, 1.0]
  capabilities: [move_joint, stop]
  verification: development
""")
        profile = load_robot_profile(path)
        self.assertTrue(profile.simulation)
        self.assertEqual(profile.joints, ("joint1",))
        self.assertEqual(profile.capabilities, frozenset({"move_joint", "stop"}))

    def test_rejects_mismatched_limits_and_non_positive_frequency(self):
        mismatch = self.write_profile("""
kind: RobotProfile
metadata: {name: bad-arm, version: 1.0.0}
spec:
  simulation: true
  control_frequency_hz: 100
  joints: [joint1]
  joint_limits: {joint2: [-1, 1]}
""")
        with self.assertRaisesRegex(ProfileError, "关节"):
            load_robot_profile(mismatch)

        bad_frequency = self.write_profile("""
kind: RobotProfile
metadata: {name: bad-arm, version: 1.0.0}
spec:
  simulation: true
  control_frequency_hz: 0
  joints: [joint1]
  joint_limits: {joint1: [-1, 1]}
""")
        with self.assertRaisesRegex(ProfileError, "频率"):
            load_robot_profile(bad_frequency)


if __name__ == "__main__":
    unittest.main()
