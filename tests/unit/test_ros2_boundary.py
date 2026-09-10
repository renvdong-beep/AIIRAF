import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from iraf_adapters.ros2 import JointStateAdapter, Ros2CommandBackend
from iraf_core.authority import ControlAuthorityManager, LeaseConflict
from iraf_core.core import RobotProfile


class Ros2BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.profile = RobotProfile(
            "piper", "1.0.0", True, 500.0, joints=("joint1", "joint2")
        )
        self.authority = ControlAuthorityManager()
        self.lease = self.authority.acquire("robot", "test")

    def test_command_validates_and_publishes_fencing_token(self):
        publish = Mock()
        backend = Ros2CommandBackend(
            publish, Mock(), self.authority, profile=self.profile
        )

        backend.move_joint({"joint1": 0.25}, 100, self.lease)

        command = publish.call_args.args[0]
        self.assertEqual({"joint1": 0.25}, command.positions)
        self.assertEqual(100, command.duration_ms)
        self.assertEqual(self.lease.fencing_token, command.fencing_token)

    def test_command_rejects_invalid_payloads_without_publishing(self):
        publish = Mock()
        backend = Ros2CommandBackend(
            publish, Mock(), self.authority, profile=self.profile
        )
        for positions, duration, message in (
            ({}, 100, "non-empty"),
            ({"unknown": 0.1}, 100, "unknown"),
            ({"joint1": float("nan")}, 100, "non-finite"),
            ({"joint1": 0.1}, 0, "positive"),
        ):
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                backend.move_joint(positions, duration, self.lease)
        publish.assert_not_called()

    def test_expired_lease_is_rejected_before_payload_validation(self):
        publish = Mock()
        backend = Ros2CommandBackend(publish, Mock(), self.authority, profile=self.profile)
        self.authority.release(self.lease)

        with self.assertRaisesRegex(LeaseConflict, "invalid fencing token"):
            backend.move_joint({}, 0, self.lease)
        publish.assert_not_called()

    def test_joint_state_rejects_duplicate_unknown_and_non_finite_values(self):
        adapter = JointStateAdapter(self.profile)
        cases = (
            (SimpleNamespace(name=["joint1", "joint1"], position=[0.0, 0.0]), "duplicate"),
            (SimpleNamespace(name=["joint3"], position=[0.0]), "unknown"),
            (SimpleNamespace(name=["joint1"], position=[float("inf")]), "non-finite position"),
        )
        for message, expected in cases:
            with self.subTest(expected=expected), self.assertRaisesRegex(ValueError, expected):
                adapter.convert(message)

    def test_joint_state_defaults_optional_arrays_and_records_state(self):
        adapter = JointStateAdapter(self.profile)
        state = adapter.convert(SimpleNamespace(name=["joint1"], position=[0.5]))

        self.assertEqual("piper", state.robot_profile)
        self.assertEqual({"joint1": 0.5}, state.positions)
        self.assertEqual({"joint1": 0.0}, state.velocities)
        self.assertEqual({"joint1": 0.0}, state.efforts)
        self.assertIs(state, adapter._last_state)


if __name__ == "__main__":
    unittest.main()
