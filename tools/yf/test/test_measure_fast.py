"""Limits of the fast runs of M8 in measure.py: the speed guard, the limits of a run, the spread of a case, the stage
to the top speed, and the speeds from the odometry they use.

Run in a sourced workspace (measure.py imports rclpy): python3 -m unittest tools/yf/test/test_measure_fast.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import measure as m  # noqa: E402

D = 0.3014


class TestSpeedGuard(unittest.TestCase):
    def test_trips_after_the_hold(self):
        g = m.SpeedGuard(0.6, 0.2)
        for i in range(5):  # odometry every ~43 ms
            g.update(1.0 + i * 0.043, 0.62)
        self.assertIsNone(g.tripped)
        g.update(1.0 + 5 * 0.043, 0.63)
        self.assertIsNotNone(g.tripped)
        self.assertAlmostEqual(g.tripped[0], 0.63)
        self.assertGreaterEqual(g.tripped[1], 0.2)

    def test_a_dip_starts_over(self):
        g = m.SpeedGuard(0.6, 0.2)
        g.update(0.0, 0.7)
        g.update(0.15, 0.7)
        g.update(0.18, 0.59)
        g.update(0.3, 0.7)
        g.update(0.45, 0.7)
        self.assertIsNone(g.tripped)
        g.update(0.5, 0.7)
        self.assertIsNotNone(g.tripped)

    def test_at_the_limit_nothing(self):
        g = m.SpeedGuard(0.6, 0.2)
        for i in range(50):
            g.update(i * 0.04, 0.6)
        self.assertIsNone(g.tripped)


class TestRunLimits(unittest.TestCase):
    def test_within(self):
        self.assertEqual(m.fast_run_problems("stop", 0.2, 0.12), [])
        self.assertEqual(m.fast_run_problems("quiet", 0.9, 0.55), [])

    def test_late_or_far(self):
        late = m.fast_run_problems("stop", 0.6, 0.1)
        self.assertEqual(len(late), 1)
        self.assertIn("limit 0.5 s", late[0])
        far = m.fast_run_problems("quiet", 0.8, 0.65)
        self.assertEqual(len(far), 1)
        self.assertIn("limit 0.60 m", far[0])
        self.assertEqual(len(m.fast_run_problems("stop", 0.7, 0.4)), 2)

    def test_never_stood(self):
        problems = m.fast_run_problems("stop", None, 0.1)
        self.assertEqual(len(problems), 1)
        self.assertIn("never", problems[0])

    def test_spread(self):
        self.assertIsNone(m.spread_problem([0.12, 0.2, 0.25]))
        self.assertIsNone(m.spread_problem([0.4]))
        self.assertIsNone(m.spread_problem([]))
        self.assertIn("spread above 0.15 m", m.spread_problem([0.1, 0.3]))


class TestStages(unittest.TestCase):
    def test_top_speed_only_if_slow_enough(self):
        # 0.48 at 0.4 m/s scales to 0.6 at 0.5 m/s, the abort.
        self.assertTrue(m.top_speed_allowed(0.42, 0.5))
        self.assertTrue(m.top_speed_allowed(0.48, 0.5))
        self.assertFalse(m.top_speed_allowed(0.49, 0.5))
        self.assertFalse(m.top_speed_allowed(None, 0.5))
        self.assertTrue(m.top_speed_allowed(0.55, 0.43))


class TestOdometry(unittest.TestCase):
    def setUp(self):
        # 0.4 m/s straight until 2.0 s, then down to standing at 2.12 s.
        self.log = [(i * 0.04, 0.4, 0.0) for i in range(51)] + [(2.04, 0.2, 0.0), (2.08, 0.05, 0.0),
                                                                 (2.12, 0.0, 0.0), (2.16, 0.0, 0.0)]

    def test_steady_and_still(self):
        self.assertAlmostEqual(m.steady_speed(self.log, 1.4, 1.9, D), 0.4)
        self.assertIsNone(m.steady_speed(self.log, 5.0, 6.0, D))
        self.assertAlmostEqual(m.still_after(self.log, 2.0, D), 2.12)
        self.assertIsNone(m.still_after(self.log, 3.0, D))

    def test_turning_counts_the_faster_wheel(self):
        self.assertAlmostEqual(m.fastest_wheel(0.0, 0.3, D), 0.3 * D / 2)
        self.assertAlmostEqual(m.fastest_wheel(0.2, -0.3, D), 0.2 + 0.3 * D / 2)


if __name__ == "__main__":
    unittest.main()
