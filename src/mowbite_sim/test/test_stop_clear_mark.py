"""STOP cleared during an emergency (G37), here tron_mcu against fake_tron on a pty with STOP kept by the board as on the
Tron: the mark is on disk before the first clear frame, which comes stop_clear_delay after the latch; a driver killed
with the mark starts latched (HIGH_LEVEL) and writes the mark again before its own first clear frame, the release
removes the mark, and a start without it is free.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_stop_clear_mark.py
"""
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (DriverProbe, TestShutdown, fake_tron, last_time, package_program, sim_files,  # noqa: E402,F401
                        sim_links, stop_process, sub_packets, wait_until, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_stop_mark_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
MARK = os.path.join(WORK, "stop-clear-mark")
SENSOR = 10
DELAY = 1.5
BOARD = {"stop_latches": 1}


def generate_test_description():
    write_json(FLAGS, BOARD)
    return launch.LaunchDescription([fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


def clears(since):
    """The clear frames fake_tron got since its time since."""
    return [e for e in sub_packets(LOG, since, SENSOR) if e.get("clear")]


class Driver:
    """tron_mcu as a process of its own, so the test can kill it and start it again."""

    def __init__(self):
        exe = package_program("tron_mcu", "tron_mcu_node")
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", "stop_release:=true", "-p", "stop_clear_period:=0.25",
                                      "-p", f"stop_clear_delay:={DELAY}", "-p", f"stop_clear_mark:={MARK}"],
                                     start_new_session=True)

    def stop(self, sig=signal.SIGINT):
        return stop_process(self.proc, sig, group=True)


class Probe(DriverProbe):
    def __init__(self):
        super().__init__("stop_clear_mark_probe", log=LOG)
        self.statuses = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def _status(self, msg):
        with self.lock:
            self.statuses.append(msg)

    @property
    def status(self):
        with self.lock:
            return self.statuses[-1] if self.statuses else None

    def clear(self):
        with self.lock:
            self.statuses = []

    def wait(self, pred, timeout=10.0):
        """The latest status once pred(status) holds, else None."""
        def fits():
            s = self.status
            return s if s is not None and pred(s) else None
        return wait_until(fits, timeout, poll=self.poll)


def press():
    """STOP pressed 0.3 s and let go; the board keeps it."""
    write_json(FLAGS, {**BOARD, "stop": 1})
    time.sleep(0.3)
    write_json(FLAGS, BOARD)


class TestStopClearMark(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.driver = Driver()

    @classmethod
    def tearDownClass(cls):
        if cls.driver:
            cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def restart(self, sig):
        self.driver.stop(sig)
        self.probe.clear()
        type(self).driver = Driver()
        return self.probe.wait(lambda s: s.telemetry_ok, 15)

    def test_a_free_without_the_mark(self):
        s = self.probe.wait(lambda s: s.telemetry_ok, 15)
        self.assertTrue(s, "no telemetry")
        self.assertFalse(s.emergency)
        self.assertFalse(os.path.exists(MARK))

    def test_b_the_mark_is_written_before_the_first_clear_frame(self):
        since = last_time(LOG)
        press()
        self.assertTrue(self.probe.wait(lambda s: s.emergency and s.stop, 2.0), "STOP did not latch")
        first = wait_until(lambda: clears(since) or None, 5.0, poll=0.01)
        self.assertTrue(first, "no clear frame")
        self.assertTrue(os.path.exists(MARK), "a clear frame went out without the mark")
        self.assertGreaterEqual(first[0]["t"] - since, DELAY - 0.2, "the clear frame came before the delay")
        # STOP let go: the clear frame drops it, the emergency stays.
        s = self.probe.wait(lambda s: not s.stop, 2.0)
        self.assertTrue(s, "stop stayed after the clear frame")
        self.assertTrue(s.emergency)

    def test_c_killed_with_the_mark_it_starts_latched(self):
        s = self.restart(signal.SIGKILL)
        self.assertTrue(s, "no telemetry after the restart")
        self.assertTrue(os.path.exists(MARK))
        self.assertTrue(s.emergency, "the driver started free with the mark")
        self.assertTrue(s.emergency_reason & HardwareStatus.REASON_HIGH_LEVEL, s.emergency_reason)

    def test_cb_a_mark_found_at_the_start_is_written_again_before_a_clear_frame(self):
        # Empty as a driver killed between creating and syncing it may leave it.
        open(MARK, "w").close()
        since = last_time(LOG)
        press()
        self.assertTrue(wait_until(lambda: clears(since) or None, DELAY + 3.0, poll=0.01), "no clear frame")
        self.assertGreater(os.path.getsize(MARK), 0, "a clear frame went out on the mark found at the start")

    def test_d_the_release_removes_the_mark(self):
        answer = self.probe.call(self.probe.emergency, SetEmergency.Request(latch=False, reason="test"))
        self.assertTrue(answer and answer.success, answer)
        self.assertTrue(self.probe.wait(lambda s: not s.emergency, 3.0), "the release did not end the emergency")
        self.assertTrue(wait_until(lambda: not os.path.exists(MARK) or None, 2.0, poll=0.05), "the mark stayed")

    def test_e_a_restart_without_the_mark_is_free(self):
        s = self.restart(signal.SIGINT)
        self.assertTrue(s, "no telemetry after the restart")
        self.assertFalse(s.emergency, s.emergency_reason)
        self.assertFalse(os.path.exists(MARK))


if __name__ == "__main__":
    unittest.main()
