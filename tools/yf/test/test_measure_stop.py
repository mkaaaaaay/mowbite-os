"""The STOP check of measure.py without a mower: what hw/status has to show of each STOP button before a step may turn
a motor, what ends it, and that neither a wheel command nor blade on goes out before it passed in this run.

Run in a sourced workspace (measure.py imports rclpy): python3 -m unittest tools/yf/test/test_measure_stop.py
"""
import os
import sys
import threading
import time
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import measure as m  # noqa: E402


class Board:
    """Stands in for Mower: hw/status follows the flags, an emergency latches until the release with stop free."""

    def __init__(self, emergency=True, stale=False):
        self.stop_checked = False
        self.flags = set()
        self.latched = False
        self.with_emergency = emergency
        self.stale = stale
        self.releases = 0

    def snapshot(self):
        stop = bool(self.flags)
        self.latched = self.with_emergency and (self.latched or stop)
        status = types.SimpleNamespace(stop=stop, emergency=self.latched, emergency_latched=self.latched,
                                       emergency_reason=4 if self.latched else 0, telemetry_ok=True, lift=False,
                                       bumper=False, blade_motor=types.SimpleNamespace(rpm=0))
        return status, 5.0 if self.stale else 0.0, (0.0, 0.0), 0.0

    def wheel_speeds(self, v, w):
        return v, v

    def tripped(self):
        return None

    def release(self):
        self.releases += 1
        if not self.flags:
            self.latched = False
        return types.SimpleNamespace(success=not self.flags, message="", code=0)


class Hand:
    """Stands in for Person: presses each button `after` s after the request for `hold` s, or not at all."""

    def __init__(self, board, after=0.05, hold=0.1, answer=True):
        self.board, self.after, self.hold, self.answer = board, after, hold, answer
        self.fake = True
        self.asked = []

    def ask(self, text):
        self.asked.append(text)
        return self.answer

    def act_later(self, text, fake=None, after=1.0, hold=1.0):
        if self.after is None:
            return

        def press():
            time.sleep(self.after)
            self.board.flags.update(fake)
            if self.hold is not None:
                time.sleep(self.hold)
                self.board.flags.difference_update(fake)
        threading.Thread(target=press, daemon=True).start()


def programme(kind=m.Measurement, **hand):
    board = Board(**{k: hand.pop(k) for k in ("emergency", "stale") if k in hand})
    run = kind.__new__(kind)
    run.findings, run.written = [], []
    run.rec = types.SimpleNamespace(write=lambda kind, **fields: run.written.append((kind, fields)), over=lambda: False)
    run.keys = types.SimpleNamespace(pressed=lambda: False)
    run.mower = board
    run.person = Hand(board, **hand)
    run.ready = lambda step: None
    return run


def checks(run):
    return [fields for kind, fields in run.written if kind == "stop_check"]


@mock.patch.object(m, "STOP_CHECK_S", 1.0)
@mock.patch.object(m, "STOP_EMERGENCY_S", 1.0)
class TestStopCheck(unittest.TestCase):
    def test_every_button_pressed_and_let_go(self):
        run = programme()
        self.assertTrue(run.stop_check("M3"))
        self.assertTrue(run.mower.stop_checked)
        self.assertEqual(run.findings, [])
        self.assertEqual([c["button"] for c in checks(run)], ["STOP top", "STOP rear"])
        for check in checks(run):
            self.assertTrue(check["free"] and check["released"] and check["latched"])
            self.assertEqual(check["reason"], 4)
            self.assertAlmostEqual(check["pressed_after_s"], 0.05, delta=0.05)
            self.assertAlmostEqual(check["held_s"], 0.1, delta=0.05)
        self.assertEqual(run.mower.releases, 2)
        self.assertIn("before M3", run.person.asked[0])

    def test_the_tron_has_one_button(self):
        run = programme(m.TronMeasurement)
        self.assertTrue(run.stop_check("T2"))
        self.assertEqual([c["button"] for c in checks(run)], ["STOP"])

    def test_a_passed_check_is_not_asked_again(self):
        run = programme()
        run.mower.stop_checked = True
        self.assertTrue(run.stop_check("M4"))
        self.assertEqual(run.person.asked, [])

    def test_a_button_that_never_shows_ends_it(self):
        run = programme(after=None)
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertEqual(len(run.findings), 1)
        self.assertIn("did not show stop within 1 s for STOP top: is the button plugged in?", run.findings[0])
        self.assertEqual(checks(run), [])

    def test_the_second_button_counts_as_well(self):
        run = programme()
        act = run.person.act_later
        run.person.act_later = lambda text, fake=None, **_: act(text, fake) if "top" in text else None
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertEqual([c["button"] for c in checks(run)], ["STOP top"])
        self.assertIn("for STOP rear", run.findings[0])

    def test_stop_before_the_press_ends_it(self):
        run = programme()
        run.mower.flags.add("stop")
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertIn("shows stop before STOP top was pressed", run.findings[0])

    def test_an_emergency_before_the_press_ends_it(self):
        run = programme()
        run.mower.latched = True
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertIn("shows an emergency before STOP top was pressed", run.findings[0])
        self.assertEqual(checks(run), [])

    def test_without_a_fresh_status_nothing_counts(self):
        run = programme(stale=True)
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertIn("shows nothing before STOP top was pressed", run.findings[0])

    def test_stop_without_an_emergency_ends_it(self):
        run = programme(emergency=False)
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertIn("no emergency within 1 s of STOP top", run.findings[0])

    def test_a_button_that_stays_down_ends_it(self):
        run = programme(hold=None)
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        self.assertIn("still shows stop", run.findings[0])
        check, = checks(run)
        self.assertFalse(check["free"] or check["released"])
        self.assertEqual(run.mower.releases, 0)

    def test_an_emergency_that_stays_ends_it(self):
        run = programme()
        run.release = lambda step: False
        with self.assertRaises(m.Abort):
            run.stop_check("M3")
        self.assertFalse(run.mower.stop_checked)
        check, = checks(run)
        self.assertTrue(check["free"])
        self.assertFalse(check["released"])

    def test_skipped_by_the_person(self):
        run = programme(answer=False)
        self.assertFalse(run.stop_check("M3"))
        self.assertFalse(run.mower.stop_checked)
        self.assertEqual(run.written, [])


def locked_mower():
    """A Mower without its node: what it would send lands in `sent`."""
    mower = m.Mower.__new__(m.Mower)
    mower.stop_checked = False
    mower.wheel_limit, mower.wheel_distance = 0.1, 0.4
    mower.sent = []
    mower.rec = types.SimpleNamespace(write=lambda kind, **fields: None)
    mower.cmd_vel = types.SimpleNamespace(publish=mower.sent.append)
    mower.blade = None
    mower.call = lambda client, request, timeout=3.0: mower.sent.append(request)
    return mower


class TestMotorLock(unittest.TestCase):
    def test_no_wheel_command_before_the_check(self):
        mower = locked_mower()
        with self.assertRaises(m.Abort) as abort:
            mower.drive(0.05, 0.0)
        self.assertIn("before the STOP check", abort.exception.args[0])
        self.assertEqual(mower.sent, [])
        mower.stop_checked = True
        mower.drive(0.05, 0.0)
        self.assertEqual(len(mower.sent), 1)

    def test_no_blade_on_before_the_check_but_off(self):
        mower = locked_mower()
        with self.assertRaises(m.Abort):
            mower.set_blade(True, 2.0)
        self.assertEqual(mower.sent, [])
        mower.set_blade(False)
        self.assertFalse(mower.sent[0].on)
        mower.stop_checked = True
        mower.set_blade(True, 2.0)
        self.assertTrue(mower.sent[1].on)

    def test_a_new_mower_is_locked(self):
        with mock.patch.object(m.rclpy, "create_node"), mock.patch.object(m.threading, "Thread"):
            self.assertFalse(m.Mower(None, 0.1, 0.4).stop_checked)


class Steps(m.Measurement):
    STANDSTILL_STEPS = ("M1",)

    def still(self):
        self.ran.append("M1")

    def turn(self):
        self.ran.append("M3")
        self.mower.drive(0.05, 0.0)

    STEPS = {"M1": still, "M3": turn, "M4": turn}


class TestRun(unittest.TestCase):
    def run_steps(self, steps, check):
        run = Steps.__new__(Steps)
        run.ran, run.findings, run.checked, run.stopped = [], [], [], []
        run.args = types.SimpleNamespace(fast=False)
        run.rec = types.SimpleNamespace(write=lambda kind, **fields: None)
        run.person = types.SimpleNamespace(assume_yes=True)
        run.mower = locked_mower()
        run.mower.max_linear_speed = 0.3
        run.safe_stop = run.stopped.append

        def stop_check(before):
            run.checked.append(before)
            return check(run)
        run.stop_check = stop_check
        return run, run.run(steps)

    @staticmethod
    def passed(run):
        run.mower.stop_checked = True
        return True

    def test_the_check_comes_before_the_first_motor_step(self):
        run, complete = self.run_steps(["M1", "M3"], self.passed)
        self.assertTrue(complete)
        self.assertEqual(run.ran, ["M1", "M3"])
        self.assertEqual(run.checked, ["M3"])
        self.assertEqual(len(run.mower.sent), 1)

    def test_a_skipped_check_skips_every_motor_step(self):
        run, complete = self.run_steps(["M3", "M1", "M4"], lambda run: False)
        self.assertTrue(complete)
        self.assertEqual(run.ran, ["M1"])
        self.assertEqual(run.checked, ["M3", "M4"])
        self.assertEqual(run.mower.sent, [])

    def test_a_failed_check_ends_the_run_without_a_command(self):
        def failed(run):
            raise m.Abort()
        run, complete = self.run_steps(["M3", "M4"], failed)
        self.assertFalse(complete)
        self.assertEqual(run.ran, [])
        self.assertEqual(run.mower.sent, [])
        self.assertEqual(len(run.stopped), 1)

    def test_a_step_that_gets_past_the_check_is_stopped_at_the_command(self):
        run, complete = self.run_steps(["M3"], lambda run: True)
        self.assertFalse(complete)
        self.assertEqual(run.mower.sent, [])
        self.assertEqual(run.findings, ["M3: a wheel command before the STOP check"])


if __name__ == "__main__":
    unittest.main()
