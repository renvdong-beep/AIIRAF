import unittest

from iraf_skills.common.trajectory import quintic_position


class QuinticTrajectoryTests(unittest.TestCase):
    def test_boundary_and_clamping(self):
        self.assertEqual([0.0, 1.0], quintic_position([0, 1], [0, 1], 2, 0))
        self.assertEqual([2.0, 3.0], quintic_position([0, 1], [2, 3], 2, 2))
        self.assertEqual([2.0, 3.0], quintic_position([0, 1], [2, 3], 2, 20))

    def test_rejects_invalid_inputs(self):
        with self.assertRaises(ValueError):
            quintic_position([0], [1], 0, 0)
        with self.assertRaises(ValueError):
            quintic_position([0], [1, 2], 1, 0)


if __name__ == "__main__":
    unittest.main()
