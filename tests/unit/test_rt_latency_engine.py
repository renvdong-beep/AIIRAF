import unittest

from iraf_adapters.tooling.rt_latency import (RtTarget, RtTestError, build_command,
                                             parse_output)

SAMPLE = """
# /dev/cpu_dma_latency set to 0us
T: 0 (1234) P:99 I:200 C: 10000 Min: 2 Act: 2 Avg: 2 Max: 66
# Min Latencies: 00002 00002 00002 00002
# Avg Latencies: 00002 00002 00003 00003
# Max Latencies: 00063 00113 00084 00034
# Overflows: 0 0 0 0
"""


class BuildCommandTest(unittest.TestCase):
    def test_default_matches_user_command(self):
        cmd = build_command(10, 200, 240, 100)
        self.assertIn("--mlockall", cmd)
        self.assertIn("--smp", cmd)
        self.assertIn("--priority=240", cmd)
        self.assertIn("--interval=200", cmd)
        self.assertIn("--distance=0", cmd)
        self.assertIn("-n", cmd)
        self.assertIn("-q", cmd)
        self.assertIn("10s", cmd)

    def test_priority_bounds(self):
        with self.assertRaises(RtTestError):
            build_command(10, 200, 300, 100)
        with self.assertRaises(RtTestError):
            build_command(10, 200, 0, 100)


class ParseTest(unittest.TestCase):
    def test_summary_lines(self):
        cores, overflows = parse_output(SAMPLE)
        self.assertEqual([c["cpu"] for c in cores], [0, 1, 2, 3])
        self.assertEqual(cores[1]["max_us"], 113)
        self.assertEqual(cores[2]["avg_us"], 3)
        self.assertEqual(overflows, [0, 0, 0, 0])

    def test_thread_line_fallback(self):
        raw = "T: 0 (1) P:99 I:200 C: 10 Min: 2 Act: 2 Avg: 3 Max: 9\n"
        cores, _ = parse_output(raw)
        self.assertEqual(cores[0], {"cpu": 0, "min_us": 2, "avg_us": 3, "max_us": 9})

    def test_unparseable(self):
        with self.assertRaises(RtTestError):
            parse_output("cyclictest V 2.20\nUsage:")


class TargetTest(unittest.TestCase):
    def test_defaults(self):
        t = RtTarget.from_env()
        self.assertIn(t.mode, ("ssh", "local"))
        self.assertEqual(t.user, "root")


if __name__ == "__main__":
    unittest.main()