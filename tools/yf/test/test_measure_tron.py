"""The Tron part of measure.py without a mower: the evaluation of the host-gone captures, the limits and the proposal,
the port settings it reads under, the port reader and the window without host on a pty, the parameters of the minimal
start, the blade motor without host (T6) with its options, and the lifts on the blade motor (T7).

Run in a sourced workspace (measure.py imports rclpy): python3 -m unittest tools/yf/test/test_measure_tron.py
"""
import math
import os
import signal
import subprocess
import sys
import tempfile
import termios
import time
import tty
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import measure as m  # noqa: E402

D = 0.40


def speeds(*samples):
    return [{"t": t, "speed": [v, w]} for t, v, w in samples]


class TestTilt(unittest.TestCase):
    def test_upright_and_on_its_side(self):
        self.assertAlmostEqual(m.tilt_of((0.0, 0.0, 9.81)), 0.0)
        self.assertAlmostEqual(m.tilt_of((0.0, 9.81, 0.0)), 90.0)

    def test_a_tilted_sample(self):
        self.assertAlmostEqual(m.tilt_of((0.0, 9.81 * math.sin(math.radians(30)), 9.81 * math.cos(math.radians(30)))), 30.0)

    def test_a_knock_is_no_tilt(self):
        # Seen on the Tron when a hand took the hood.
        self.assertIsNone(m.tilt_of((-0.1, -10.03, 17.38)))
        self.assertIsNone(m.tilt_of((-0.54, -1.81, 6.34)))

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
        # 0.05 rad/s on 0.40 m: each wheel 0.01 m/s, standing; 0.1 rad/s: 0.02 m/s, turning.
        self.assertAlmostEqual(m.host_gone_result(speeds((0.0, 0.0, 0.1), (1.0, 0.0, 0.05)), 0.0, D)["stand_s"], 1.0)

    def test_the_noise_of_standing_wheels_stands(self):
        # The largest the Tron reported standing (07.11.): one wheel at 0.0106 m/s.
        packets = speeds((1.0, 0.05, 0.0), (1.2, 0.0, 0.0), (2.5, 0.0049, 0.0284), (3.0, -0.0033, 0.0208))
        self.assertAlmostEqual(m.host_gone_result(packets, 1.0, D)["stand_s"], 0.2)

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


def motors(*samples):
    return [{"t": t, "motors": {"cutter": rpm, "left": 0, "right": 0}} for t, rpm in samples]


class TestBladeGone(unittest.TestCase):
    def test_stands_at_the_first_report_below_the_limit(self):
        result = m.blade_gone_result(motors((9.5, 840), (10.4, 840), (11.4, 300), (12.4, 60), (13.4, 0)), 10.0, 100)
        self.assertAlmostEqual(result["still_s"], 2.4)
        self.assertTrue(result["turning_at_start"])
        self.assertEqual(result["last_rpm"], 0)
        self.assertEqual(result["samples"], 4)

    def test_still_turning_at_the_end(self):
        result = m.blade_gone_result(motors((10.5, 2800), (11.5, 2800), (19.5, 2800)), 10.0, 100)
        self.assertIsNone(result["still_s"])
        self.assertEqual(result["last_rpm"], 2800)

    def test_a_motor_that_starts_again_did_not_stand(self):
        result = m.blade_gone_result(motors((10.5, 840), (11.5, 0), (12.5, 840), (13.5, 0)), 10.0, 100)
        self.assertAlmostEqual(result["still_s"], 3.5)

    def test_the_limit_itself_turns_and_the_direction_does_not_count(self):
        result = m.blade_gone_result(motors((10.5, -840), (11.5, -100), (12.5, -99)), 10.0, 100)
        self.assertAlmostEqual(result["still_s"], 2.5)
        self.assertTrue(result["turning_at_start"])

    def test_already_standing_and_nothing_after_the_signal(self):
        result = m.blade_gone_result(motors((10.3, 0), (11.3, 0)), 10.0, 100)
        self.assertAlmostEqual(result["still_s"], 0.3)
        self.assertFalse(result["turning_at_start"])
        result = m.blade_gone_result(motors((9.0, 840)) + speeds((10.2, 0.0, 0.0)), 10.0, 100)
        self.assertEqual(result, {"still_s": None, "turning_at_start": False, "last_rpm": None, "samples": 0})


class TestBladeArguments(unittest.TestCase):
    def test_t8_needs_no_blade(self):
        self.assertIn("T8", m.TronMeasurement.STEPS)
        self.assertIn("T8", m.TronMeasurement.STANDSTILL_STEPS)
        with self.assertRaises(SystemExit) as refused:
            m.main(["--platform", "tron", "--steps", "T8", "--blade"])
        self.assertIn("go together", str(refused.exception.code))

    def test_blade_and_t6_or_t7_go_together(self):
        for argv in (["--steps", "T6"], ["--blade"], ["--steps", "T1,T6"], ["--steps", "T1", "--blade"],
                     ["--steps", "T7"], ["--steps", "T6,T7"], ["--steps", "T10"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(["--platform", "tron", *argv])
            self.assertIn("go together", str(refused.exception.code), argv)

    def test_percent_values(self):
        self.assertEqual(m.blade_percents("30,100"), (30, 100))
        self.assertEqual(m.blade_percents(" 100 "), (100,))
        self.assertEqual(m.blade_percents("1,50,100"), (1, 50, 100))
        for text in ("", "0", "101", "30,", "30,,100", "30.5", "-30", "1e2", "²", "abc"):
            with self.assertRaisesRegex(ValueError, "a whole number from 1 to 100", msg=text):
                m.blade_percents(text)
        with self.assertRaisesRegex(ValueError, "at most 3 values, got 4"):
            m.blade_percents("10,20,30,40")
        for text in ("30,30", "100,30,030"):
            with self.assertRaisesRegex(ValueError, "each value once", msg=text):
                m.blade_percents(text)

    def test_window_values(self):
        self.assertEqual(m.whole_seconds("5", m.BLADE_WINDOW_RANGE_S), 5.0)
        self.assertEqual(m.whole_seconds(" 120 ", m.BLADE_WINDOW_RANGE_S), 120.0)
        for text in ("4", "121", "60.5", "1e1", "0", "-10", "nan", "inf", "", "ten"):
            with self.assertRaisesRegex(ValueError, "a whole number from 5 to 120", msg=text):
                m.whole_seconds(text, m.BLADE_WINDOW_RANGE_S)

    def test_input_wait_values(self):
        self.assertEqual(m.whole_seconds("5", m.INPUT_WAIT_RANGE_S), 5.0)
        self.assertEqual(m.whole_seconds("60", m.INPUT_WAIT_RANGE_S), 60.0)
        self.assertEqual(m.BLADE_INPUT_WAIT_S, 10.0)
        for text in ("4", "61", "10.5", "0", "-10", "", "ten"):
            with self.assertRaisesRegex(ValueError, "a whole number from 5 to 60", msg=text):
                m.whole_seconds(text, m.INPUT_WAIT_RANGE_S)

    def test_the_capture_limit_follows_a_long_window(self):
        self.assertEqual(m.capture_limit(m.HOST_GONE_WINDOW_S), m.CAPTURE_MAX_BYTES)
        self.assertEqual(m.capture_limit(m.BLADE_GONE_WINDOW_S), m.CAPTURE_MAX_BYTES)
        # The Tron's MCU filled 8.4 kB a second (T2, 08.11.): the longest window has room for three times that.
        longest = m.BLADE_WINDOW_RANGE_S[1]
        self.assertGreaterEqual(m.capture_limit(longest), 3 * 8400 * longest)

    def test_options_only_with_t6_on_the_tron(self):
        for argv in (["--blade-percent", "100"], ["--blade-window", "60"],
                     ["--platform", "tron", "--steps", "T2", "--blade-window", "60"],
                     ["--platform", "nx60i", "--steps", "M7", "--blade", "--blade-percent", "100"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(argv)
            self.assertIn("for T6 on the Tron", str(refused.exception.code), argv)

    def test_input_wait_only_with_t6_or_t7_on_the_tron(self):
        for argv in (["--input-wait", "30"], ["--platform", "tron", "--steps", "T2", "--input-wait", "30"],
                     ["--platform", "nx60i", "--steps", "M7", "--blade", "--input-wait", "30"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(argv)
            self.assertIn("for T6 and T7 on the Tron", str(refused.exception.code), argv)
        # Accepted, the options pass: the run would begin with the record dir.
        for steps in ("T6", "T7", "T1,T7"):
            with mock.patch.object(m.os, "makedirs", side_effect=RuntimeError("past the options")):
                with self.assertRaisesRegex(RuntimeError, "past the options"):
                    m.main(["--platform", "tron", "--steps", steps, "--blade", "--input-wait", "60"])

    def test_blade_options_of_t6_are_not_for_t7(self):
        for option, value in (("--blade-percent", "100"), ("--blade-window", "60")):
            with self.assertRaises(SystemExit) as refused:
                m.main(["--platform", "tron", "--steps", "T7", "--blade", option, value])
            self.assertIn("for T6 on the Tron", str(refused.exception.code))

    def test_wrong_values_are_refused_before_anything_starts(self):
        t6 = ["--platform", "tron", "--steps", "T6", "--blade", "--record-dir", "/nonexistent/record"]
        for option, value in (("--blade-percent", "0"), ("--blade-percent", "30,60,90,100"),
                              ("--blade-percent", "30,30"), ("--blade-window", "4"), ("--blade-window", "121"),
                              ("--blade-window", "7.5")):
            with self.assertRaises(SystemExit) as refused:
                m.main([*t6, option, value])
            self.assertTrue(str(refused.exception.code).startswith(option + ": "), refused.exception.code)
        t7 = ["--platform", "tron", "--steps", "T7", "--blade", "--record-dir", "/nonexistent/record"]
        for value in ("4", "61", "7.5", ""):
            with self.assertRaises(SystemExit) as refused:
                m.main([*t7, "--input-wait", value])
            self.assertTrue(str(refused.exception.code).startswith("--input-wait: "), refused.exception.code)

    def test_lift_sides_values(self):
        self.assertEqual(m.lift_sides("front"), ("front",))
        self.assertEqual(m.lift_sides(" right, left "), ("right", "left"))
        self.assertEqual(m.lift_sides("rear,front,left,right"), ("rear", "front", "left", "right"))
        for text in ("", "top", "front,", "Front", "front,,left", "left;right"):
            with self.assertRaisesRegex(ValueError, "each side one of front, rear, left, right", msg=text):
                m.lift_sides(text)
        for text in ("left,left", "front,right,front"):
            with self.assertRaisesRegex(ValueError, "each side once", msg=text):
                m.lift_sides(text)

    def test_lift_sides_only_with_t7_on_the_tron(self):
        for argv in (["--lift-sides", "left"],
                     ["--platform", "tron", "--steps", "T6", "--blade", "--lift-sides", "left"],
                     ["--platform", "tron", "--steps", "T2", "--lift-sides", "front"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(argv)
            self.assertIn("--lift-sides is for T7 on the Tron", str(refused.exception.code), argv)
        t7 = ["--platform", "tron", "--steps", "T7", "--blade", "--record-dir", "/nonexistent/record"]
        for value in ("top", "left,left", ""):
            with self.assertRaises(SystemExit) as refused:
                m.main([*t7, "--lift-sides", value])
            self.assertTrue(str(refused.exception.code).startswith("--lift-sides: "), refused.exception.code)

    def lift_sides_of(self, *argv):
        seen = []

        def start(args, rec):
            seen.append(args.lift_sides)
            raise RuntimeError("past the options")
        with mock.patch.object(m, "Recorder"), mock.patch.object(m, "TronStart", side_effect=start):
            with self.assertRaisesRegex(RuntimeError, "past the options"):
                m.main(["--platform", "tron", "--steps", "T7", "--blade", *argv])
        return seen[0]

    def test_lift_sides_default_to_the_front_and_keep_their_order(self):
        self.assertEqual(self.lift_sides_of(), ("front",))
        self.assertEqual(self.lift_sides_of("--lift-sides", "right,rear,front"), ("right", "rear", "front"))

    def test_hold_cases_values(self):
        self.assertEqual(m.hold_cases("v0"), ("V0",))
        self.assertEqual(m.hold_cases(" V2, G ,V0"), ("V2", "G", "V0"))
        for text in ("", "V4", "V0,", "V0;V1"):
            with self.assertRaisesRegex(ValueError, "each case one of", msg=text):
                m.hold_cases(text)
        with self.assertRaisesRegex(ValueError, "each case once"):
            m.hold_cases("V1,v1")

    def test_hold_only_with_t9_and_its_blade_case_only_with_blade(self):
        for argv in (["--hold", "V0"], ["--platform", "tron", "--steps", "T2", "--hold", "V0"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(argv)
            self.assertIn("--hold is for T9 on the Tron", str(refused.exception.code), argv)
        t9 = ["--platform", "tron", "--steps", "T9", "--record-dir", "/nonexistent/record"]
        with self.assertRaises(SystemExit) as refused:
            m.main([*t9, "--hold", "V9"])
        self.assertTrue(str(refused.exception.code).startswith("--hold: "), refused.exception.code)
        for argv in ([*t9, "--hold", "V0,B"], [*t9, "--blade"]):
            with self.assertRaises(SystemExit) as refused:
                m.main(argv)
            self.assertIn("go together", str(refused.exception.code), argv)

    def hold_of(self, *argv):
        seen = []

        def start(args, rec):
            seen.append(args.hold)
            raise RuntimeError("past the options")
        with mock.patch.object(m, "Recorder"), mock.patch.object(m, "TronStart", side_effect=start):
            with self.assertRaisesRegex(RuntimeError, "past the options"):
                m.main(["--platform", "tron", "--steps", "T9", *argv])
        return seen[0]

    def test_hold_defaults_and_the_blade_case(self):
        self.assertEqual(self.hold_of(), m.HOLD_DEFAULT)
        self.assertEqual(self.hold_of("--hold", "B,V0", "--blade"), ("B", "V0"))

    def test_every_hold_case_silences_after_its_frame_within_the_driver_limits(self):
        for case, (text, overrides, silence, runs) in m.HOLD_CASES.items():
            self.assertIn(overrides["silence_after"], ("speed", "cutter"), case)
            self.assertIn(overrides.get("speed_heartbeat", ""), ("", "after", "frame", "later"), case)
            self.assertGreater(silence, m.HOLD_HELD_S, case)
            self.assertEqual(overrides.get("allow_blade", False), case == "B", case)

    def test_the_cases_before_t9_keep_the_heartbeat_ahead_of_the_setpoint_only(self):
        for case in ("V0", "G"):
            self.assertEqual(m.HOLD_CASES[case][1]["speed_heartbeat"], "", case)

    def test_t6_and_t7_are_not_in_the_default_steps(self):
        self.assertIn("T6", m.TronMeasurement.STEPS)
        self.assertIn("T7", m.TronMeasurement.STEPS)
        with self.assertRaises(SystemExit) as refused:
            m.main(["--platform", "tron", "--ground"])
        self.assertIn("not for the Tron", str(refused.exception.code))


class TestCuttingHeight(unittest.TestCase):
    def test_the_deck_times_are_asked_after_the_run_with_the_blade_off(self):
        events = []
        t = m.TronMeasurement.__new__(m.TronMeasurement)
        t.ask = lambda text: True
        t.ready = t.mark = lambda *a: None
        t.blade_turning = lambda *a: events.append("on")
        t.blade_run = lambda step, percent, s, until, height_mm: events.append(f"on {height_mm}")
        t.blade_stands = lambda *a: 0.2
        t.blade_still_rpm = 300
        t.mower = types.SimpleNamespace(set_blade=lambda on: events.append("off"))
        t.person = types.SimpleNamespace(number=lambda text: events.append("asked") or 2.0)
        t.rec = types.SimpleNamespace(write=lambda kind, **r: events.append((r["way"], r["from_mm"], r["to_mm"],
                                                                             r["moved_s"])))
        t.height_runs("T10")
        # A question between two changes would let the lease run out: none while the blade is on.
        on = False
        for e in events:
            if isinstance(e, str) and e.startswith("on"):
                on = True
            elif e == "off":
                on = False
            self.assertFalse(on and e == "asked", events)
        self.assertEqual([e for e in events if isinstance(e, tuple)],
                         [("on", 90, 30, 2.0), ("off", 30, 90, 2.0), ("on", 90, 60, 2.0), ("on", 60, 30, 2.0),
                          ("on", 30, 90, 2.0), ("off", 90, 90, 2.0)])


class TestLiftItems(unittest.TestCase):
    def test_only_the_front_past_the_tilt_of_t71(self):
        items = m.lift_blade_items({"front": 37.6, "rear": 41.0, "left": 39.4, "right": 40.0}, {"rear"}, 3)
        self.assertEqual([(text, fake, runs, free) for text, fake, runs, free in items],
                         [("lift the front past 38 deg while the blade motor turns",
                           {"lift": True, "tilt_pitch": -38.0}, 3, "set the front down")])

    def test_none_if_t71_saw_no_lift_at_the_front(self):
        self.assertEqual(m.lift_blade_items({"left": 39.0}, {"front"}, 3), [])

    def test_the_front_not_lifted_or_without_tilt_goes_until_lift_shows(self):
        for tilts, unlifted in (({}, {"left", "right"}), ({"front": None}, set())):
            items = m.lift_blade_items(tilts, unlifted, 1)
            self.assertEqual([(text, runs) for text, _, runs, _ in items],
                             [("lift the front until lift shows while the blade motor turns", 1)])


def lift_programme(sides, results):
    """A TronMeasurement that lifts `sides` in T7.1, each giving its entry of `results` (lift, tilt at lift)."""
    run = m.TronMeasurement.__new__(m.TronMeasurement)
    run.findings, run.written, run.lifted = [], [], []
    run.args = types.SimpleNamespace(lift_sides=sides, repeats=5, input_wait=10.0)
    run.rec = types.SimpleNamespace(write=lambda kind, **fields: run.written.append((kind, fields)))
    run.ask = lambda text: True

    def by_hand(label, text, down, hold, tilt):
        run.lifted.append(label)
        lift, at = results[label.split()[-1]]
        return {"lift": lift, "tilt_at_lift": at, "latched": lift or None, "max_tilt": 40.0, "released": True}
    run.lift_by_hand = by_hand
    return run


class TestLiftAngles(unittest.TestCase):
    def test_the_sides_go_up_in_their_order(self):
        run = lift_programme(("right", "front", "rear"), {"right": (False, None), "front": (True, 3.5),
                                                           "rear": (False, None)})
        with mock.patch("builtins.print"):
            tilts, unlifted = run.lift_angles("T7.1")
        self.assertEqual(run.lifted, ["T7.1 right", "T7.1 front", "T7.1 rear"])
        self.assertEqual((tilts, unlifted), ({"front": 3.5}, {"right", "rear"}))
        angles = [f for kind, f in run.written if kind == "lift_angle"]
        self.assertEqual([(f["side"], f["expected_lift"], f["lift"]) for f in angles],
                         [("right", False, False), ("front", True, True), ("rear", False, False)])

    def test_no_lift_is_a_finding_only_at_the_front(self):
        run = lift_programme(("left", "right"), {"left": (False, None), "right": (True, 20.0)})
        with mock.patch("builtins.print") as shown:
            run.lift_angles("T7.1")
        self.assertEqual(run.findings, [])
        notes = [c.args[0] for c in shown.call_args_list if "note:" in c.args[0]]
        self.assertEqual(len(notes), 2)
        self.assertIn("no lift lifting the left side (largest tilt 40.0 deg), as expected", notes[0])
        self.assertIn("lift lifting the right side, where the MCU is not expected to show it", notes[1])
        run = lift_programme(("front",), {"front": (False, None)})
        with mock.patch("builtins.print"):
            run.lift_angles("T7.1")
        finding, = run.findings
        self.assertTrue(finding.startswith("T7.1 front: no lift in hw/status lifting the front"), finding)

    def t73_items(self, sides, results):
        run = lift_programme(sides, results)
        run.lift_level = lambda step: None
        run.blade_setup = lambda: None
        run.blade_stop_s = 5.0
        run.blade_allowed = lambda step, act: act()
        inputs = []
        run.blade_inputs = lambda step, items, percent: inputs.append((step, items, percent))
        with mock.patch("builtins.print"):
            run.t7()
        return inputs

    def test_t73_lifts_the_front_only(self):
        (step, items, percent), = self.t73_items(("front", "left"), {"front": (True, 4.2), "left": (True, 30.0)})
        self.assertEqual((step, percent), ("T7.3", m.LIFT_BLADE_PERCENT))
        self.assertEqual([(text, runs) for text, _, runs, _ in items],
                         [("lift the front past 4 deg while the blade motor turns", m.TRON_SHORT_REPEATS)])
        (_, items, _), = self.t73_items(("left", "right"), {"left": (False, None), "right": (False, None)})
        self.assertEqual([text for text, _, _, _ in items],
                         ["lift the front until lift shows while the blade motor turns"])

    def test_t73_skipped_without_lift_at_the_front(self):
        self.assertEqual(self.t73_items(("front",), {"front": (False, None)}), [])


def hold_programme(reason=None, delay=0.1):
    """A TronMeasurement whose driver goes silent `delay` s after the frame and shows the latch with `reason` only at
    the end of the silence, as tron_mcu does; reason None: no silence begins."""
    run = m.TronMeasurement.__new__(m.TronMeasurement)
    run.findings = []
    run.finding = lambda step, text: run.findings.append(text)
    run.check_keys = lambda: None
    mower = types.SimpleNamespace(emergency_since=None, end=None)
    run.mower = mower

    def frame():
        run.sent = time.monotonic()
        if reason is not None:
            mower.end = run.sent + delay + SILENCE
            mower.emergency_since = mower.end

    def snapshot():
        on = mower.end is not None and time.monotonic() >= mower.end
        status = types.SimpleNamespace(emergency=on, emergency_reason=reason if on else 0)
        return status, 0.0, (0.0, 0.0), 0.0
    mower.snapshot = snapshot
    run.start_hold = lambda: run.hold_start("T9", "V0", 1, SILENCE, lambda: types.SimpleNamespace(success=True),
                                            frame)
    return run


SILENCE = 0.3


class TestHoldStart(unittest.TestCase):
    def test_the_start_comes_from_the_latch_at_the_end_of_the_silence(self):
        run = hold_programme(m.HOLD_END_REASON)
        began = run.start_hold()
        self.assertAlmostEqual(began, run.sent + 0.1, places=6)
        self.assertGreaterEqual(time.monotonic(), run.sent + 0.1 + SILENCE)
        self.assertEqual(run.findings, [])

    def test_stop_during_the_silence_aborts(self):
        run = hold_programme(1 << 0 | 1 << 2)
        with self.assertRaises(m.Abort):
            run.start_hold()
        self.assertIn("LATCH|STOP instead of the end", run.findings[0])

    def test_no_silence_aborts(self):
        run = hold_programme(None)
        with self.assertRaises(m.Abort):
            run.start_hold()
        self.assertIn("no latch", run.findings[0])

    def test_a_silence_long_after_the_frame_aborts(self):
        run = hold_programme(m.HOLD_END_REASON, delay=m.HOLD_ARM_S + 0.2)
        with self.assertRaises(m.Abort):
            run.start_hold()
        self.assertIn("after the frame", run.findings[0])


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
        self.assertFalse(reader.full)
        os.write(self.master, b"\x01" * 40)
        time.sleep(0.1)
        reader.close()
        self.assertTrue(reader.full)
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

    def stand_in(self):
        """A measurement whose tron_mcu is a sleeping process that holds the port, and that process."""
        self.settings()
        driver = subprocess.Popen(["sleep", "30"], stdin=self.slave)
        self.addCleanup(driver.wait)
        self.addCleanup(driver.kill)

        class Start:
            mcu_port, log_stem = self.path, os.path.join(self.work, "run")
            processes, overrides, started = {"tron_mcu": driver}, {}, []

            @property
            def driver_pid(self):
                return driver.pid if driver.poll() is None else None

            def signal_driver(self, sig):
                os.kill(driver.pid, sig)
                return driver.pid

            def start_driver(self, **overrides):
                self.started.append(overrides)
        run = m.TronMeasurement.__new__(m.TronMeasurement)
        run.findings, run.back = [], []
        run.rec = types.SimpleNamespace(write=lambda kind, **fields: None)
        run.keys = types.SimpleNamespace(pressed=lambda: False)
        run.mower = types.SimpleNamespace(tripped=lambda: None)
        run.start = Start()
        run.wait_driver_back = lambda step, pid: run.back.append(pid)
        return run, driver

    def test_without_host_reads_the_window_and_restarts_a_frozen_driver(self):
        run, driver = self.stand_in()
        os.write(self.master, b"\x01" * 40)
        since, capture = run.without_host("T6", "freeze", signal.SIGSTOP, "freeze-30-1", 0.3)
        self.assertGreaterEqual(time.monotonic() - since, 0.3)
        with open(capture) as f:
            self.assertEqual(f.read().split()[1], "01" * 40)
        self.assertEqual(run.findings, [])
        self.assertEqual(driver.returncode, -signal.SIGKILL)
        self.assertEqual(run.start.started, [{}])
        self.assertEqual(run.back, [driver.pid])

    def test_a_capture_at_its_limit_ends_the_step(self):
        run, driver = self.stand_in()
        os.write(self.master, b"\x01" * 40)
        with mock.patch.object(m, "capture_limit", return_value=60), self.assertRaises(m.Abort):
            run.without_host("T6", "freeze", signal.SIGSTOP, "freeze-30-1", 0.3)
        self.assertEqual(len(run.findings), 1)
        self.assertIn("reached its limit of 60 bytes", run.findings[0])
        self.assertEqual(driver.returncode, -signal.SIGKILL)
        self.assertEqual(run.start.started, [{}])


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

    def test_watchdog_of_the_running_start(self):
        work = tempfile.mkdtemp(prefix="measure_tron_")
        path = os.path.join(work, "tron.yaml")
        with open(path, "w") as f:
            f.write("tron_mcu:\n  ros__parameters:\n    cmd_vel_timeout: 0.5\n    speed_repeat_window: 0.3\n")
        start = m.TronStart.__new__(m.TronStart)
        start.files = [path]
        start.overrides = {}
        self.assertAlmostEqual(start.watchdog(), 0.3)
        start.overrides = {"cmd_vel_timeout": 0.2}
        self.assertAlmostEqual(start.watchdog(), 0.2)
        start.overrides = dict(m.HEARTBEAT_ONLY)
        self.assertAlmostEqual(start.watchdog(), m.HEARTBEAT_ONLY_TIMEOUT_S)

    def test_time_for_the_imu_after_the_start(self):
        work = tempfile.mkdtemp(prefix="measure_tron_")
        path = os.path.join(work, "tron.yaml")
        with open(path, "w") as f:
            f.write("tron_mcu:\n  ros__parameters:\n    imu_timeout: 0.5\n    imu_start_timeout: 2.0\n")
        start = m.TronStart.__new__(m.TronStart)
        start.files = [path]
        self.assertAlmostEqual(start.imu_start(), 2.0)
        with open(path, "w") as f:
            f.write("tron_mcu:\n  ros__parameters:\n    imu_timeout: 3.0\n    imu_start_timeout: 2.0\n")
        self.assertAlmostEqual(start.imu_start(), 3.0)


if __name__ == "__main__":
    unittest.main()
