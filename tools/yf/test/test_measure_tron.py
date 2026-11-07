"""The Tron part of measure.py without a mower: the evaluation of the host-gone captures, the limits and the proposal,
the port settings it reads under, the port reader on a pty, and the parameters of the minimal start.

Run in a sourced workspace (measure.py imports rclpy): python3 -m unittest tools/yf/test/test_measure_tron.py
"""
import os
import sys
import tempfile
import termios
import time
import tty
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import measure as m  # noqa: E402

D = 0.40


def speeds(*samples):
    return [{"t": t, "speed": [v, w]} for t, v, w in samples]


class TestHostGone(unittest.TestCase):
    def test_stands_after_the_last_turning_sample(self):
        packets = speeds((9.9, 0.05, 0.0), (10.0, 0.05, 0.0), (11.0, 0.05, 0.0), (11.5, 0.0, 0.0), (11.6, 0.03, 0.0),
                         (12.0, 0.0, 0.0), (14.0, 0.0, 0.0))
        r = m.host_gone_result(packets, 10.0, D)
        self.assertAlmostEqual(r["stand_s"], 2.0)
        self.assertTrue(r["turning_at_start"])
        self.assertEqual(r["samples"], 6)

    def test_still_turning_at_the_end(self):
        r = m.host_gone_result(speeds((1.0, 0.05, 0.0), (3.0, 0.0, 0.0), (5.0, 0.05, 0.0)), 1.0, D)
        self.assertIsNone(r["stand_s"])

    def test_a_turn_on_the_spot_counts_by_its_faster_wheel(self):
        # 0.02 rad/s on 0.40 m: each wheel 0.004 m/s, standing; 0.05 rad/s: 0.01 m/s, turning.
        self.assertAlmostEqual(m.host_gone_result(speeds((0.0, 0.0, 0.05), (1.0, 0.0, 0.02)), 0.0, D)["stand_s"], 1.0)

    def test_flags_and_motors(self):
        packets = speeds((5.0, 0.05, 0.0), (6.5, 0.0, 0.0)) + [
            {"t": 4.0, "sensor": {"lift": 1, "stop": 1}},
            {"t": 5.8, "sensor": {"lift": 1, "stop": 0}},
            {"t": 6.1, "sensor": {"lift": 1, "stop": 1}},
            {"t": 5.5, "motors": {"cutter": 0, "left": 30, "right": 31}},
            {"t": 6.4, "motors": {"cutter": 0, "left": 0, "right": 2}},
            {"t": 6.9, "motors": {"cutter": 0, "left": 0, "right": 0}}]
        r = m.host_gone_result(packets, 5.0, D)
        self.assertAlmostEqual(r["lift_s"], 0.8)
        self.assertAlmostEqual(r["stop_s"], 1.1)
        self.assertAlmostEqual(r["rpm_last_s"], 1.4)

    def test_nothing_after_the_signal(self):
        r = m.host_gone_result(speeds((1.0, 0.05, 0.0)), 2.0, D)
        self.assertEqual((r["samples"], r["stand_s"], r["turning_at_start"]), (0, None, False))

    def test_already_standing(self):
        r = m.host_gone_result(speeds((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), 1.0, D)
        self.assertEqual((r["stand_s"], r["turning_at_start"]), (0.0, False))


class TestLimits(unittest.TestCase):
    def test_within(self):
        self.assertEqual(m.holding_problems([1.9, 2.0, 2.1, 2.05, 2.2]), [])
        self.assertEqual(m.holding_problems([3.0]), [])
        self.assertEqual(m.holding_problems([]), [])

    def test_late(self):
        problems = m.holding_problems([3.1])
        self.assertEqual(len(problems), 1)
        self.assertIn("limit 3.0 s", problems[0])

    def test_spread(self):
        self.assertEqual(m.holding_problems([1.0, 1.3]), [])
        problems = m.holding_problems([1.0, 1.31])
        self.assertEqual(len(problems), 1)
        self.assertIn("spread above 0.3 s", problems[0])

    def test_not_standing_in_a_run(self):
        problems = m.holding_problems([2.0, None, 2.1])
        self.assertEqual(len(problems), 1)
        self.assertIn("1 of 3 runs", problems[0])

    def test_proposal(self):
        self.assertAlmostEqual(m.failsafe_proposal([1.42, 1.5, None]), 1.5)
        self.assertAlmostEqual(m.failsafe_proposal([1.51]), 1.6)
        self.assertAlmostEqual(m.failsafe_proposal([2.0]), 2.0)
        self.assertIsNone(m.failsafe_proposal([None]))
        text = m.tron_values_text(1.6, {"cutter": "0.6.36", "chassis": "0.6.34", "bms": "1.17.0"}, "2026-11-08")
        self.assertEqual(text.splitlines(), ["tron_mcu:", "  ros__parameters:", "    failsafe_without_host: 1.6",
                                             '    safety_measurement_date: "2026-11-08"',
                                             '    expected_cutter_version: "0.6.36"',
                                             '    expected_chassis_version: "0.6.34"'])


class TestPort(unittest.TestCase):
    def setUp(self):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self.slave = slave
        self.work = tempfile.mkdtemp(prefix="measure_tron_")

    def tearDown(self):
        os.close(self.slave)
        os.close(self.master)

    def settings(self, raw=True, speed=termios.B115200, iflag=0):
        if raw:
            tty.setraw(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[0] |= iflag
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)

    def test_settings_that_would_answer_are_refused(self):
        self.settings(raw=False)
        self.assertIn("echo", m.termios_problem(termios.tcgetattr(self.slave)))
        self.settings(iflag=termios.IXOFF)
        self.assertIn("flow control", m.termios_problem(termios.tcgetattr(self.slave)))
        self.settings(speed=termios.B9600)
        self.assertIn("115200", m.termios_problem(termios.tcgetattr(self.slave)))
        self.settings()
        self.assertIsNone(m.termios_problem(termios.tcgetattr(self.slave)))

    def test_reader_refuses_and_keeps_the_settings(self):
        self.settings(raw=False)
        before = termios.tcgetattr(self.slave)
        with self.assertRaises(m.PortRefused):
            m.PortReader(self.path, os.path.join(self.work, "refused.capture"))
        self.assertEqual(termios.tcgetattr(self.slave), before)
        self.assertFalse(os.path.exists(os.path.join(self.work, "refused.capture")))

    def test_reader_captures_with_time_writes_nothing_and_keeps_its_limit(self):
        self.settings()
        before = termios.tcgetattr(self.slave)
        capture = os.path.join(self.work, "a.capture")
        reader = m.PortReader(self.path, capture, max_bytes=60)
        t0 = time.monotonic()
        os.write(self.master, bytes([0xA5, 0x00, 0x5A]))
        time.sleep(0.1)
        os.write(self.master, b"\x01" * 40)
        time.sleep(0.1)
        reader.close()
        with open(capture) as f:
            lines = f.read().splitlines()
        self.assertEqual(len(lines), 1)
        t, data = lines[0].split()
        self.assertEqual(data, "a5005a")
        self.assertGreaterEqual(float(t), t0)
        self.assertEqual(termios.tcgetattr(self.slave), before)
        os.set_blocking(self.master, False)
        with self.assertRaises(BlockingIOError):
            os.read(self.master, 64)

    def test_holders_and_state(self):
        with open(self.path, "rb", buffering=0):
            pass
        self.assertEqual(m.port_holders(self.path), [])
        self.assertIn(m.process_state(os.getpid()), ("R", "S"))
        self.assertIsNone(m.process_state(2 ** 22 + 1))


class TestStartParameters(unittest.TestCase):
    def test_last_file_wins_and_own_section_over_all(self):
        work = tempfile.mkdtemp(prefix="measure_tron_")
        first, second = os.path.join(work, "a.yaml"), os.path.join(work, "b.yaml")
        with open(first, "w") as f:
            f.write("/**:\n  ros__parameters:\n    port: x\n    t: 1\ntron_mcu:\n  ros__parameters:\n    port: /dev/ttyS9\n")
        with open(second, "w") as f:
            f.write("tron_mcu:\n  ros__parameters:\n    failsafe_without_host: 1.6\n")
        start = m.TronStart.__new__(m.TronStart)
        start.files = [first, second]
        self.assertEqual(start.param("tron_mcu", "port"), "/dev/ttyS9")
        self.assertEqual(start.param("tron_imu", "port"), "x")
        self.assertEqual(start.param("tron_mcu", "failsafe_without_host"), 1.6)
        self.assertEqual(start.param("tron_mcu", "t"), 1)
        self.assertIsNone(start.param("tron_mcu", "missing"))


if __name__ == "__main__":
    unittest.main()
