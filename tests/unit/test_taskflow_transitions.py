import unittest

from iraf_core.core import TaskStatus
from iraf_core.taskflow import InvalidTransition, TaskFlow


class TaskFlowTransitionTests(unittest.TestCase):
    def test_valid_path_is_monotonic_and_terminal(self):
        flow = TaskFlow("execution-taskflow-001")
        self.assertEqual(flow.transition(TaskStatus.VALIDATING)["sequence"], 1)
        self.assertEqual(flow.transition(TaskStatus.RUNNING)["sequence"], 2)
        event = flow.transition(TaskStatus.SUCCEEDED, "completed")
        self.assertEqual(event["sequence"], 3)
        self.assertEqual(flow.status, TaskStatus.SUCCEEDED)

    def test_illegal_and_terminal_transitions_are_rejected(self):
        flow = TaskFlow("execution-taskflow-002")
        with self.assertRaises(InvalidTransition):
            flow.transition(TaskStatus.SUCCEEDED)
        flow.transition(TaskStatus.CANCELLED, "operator request")
        with self.assertRaises(InvalidTransition):
            flow.transition(TaskStatus.RUNNING)


if __name__ == "__main__":
    unittest.main()
