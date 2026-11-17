"""A driver the launch starts again after the last one ended (start_latched), here tron_mcu against fake_tron on a pty:
it is latched with HIGH_LEVEL from the start, its first frames carry three stops beside the heartbeat, no command
reaches the board until the release, and read only it neither latches nor sends.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_start_latched.py
"""
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (DriverProbe, TestShutdown, fake_tron, host_packets, last_time, moving,  # noqa: E402,F401
                        package_program, sim_files, sim_links, stop_process, sub_packets, wait_until)

WORK = tempfile.mkdtemp(prefix="mowbite_start_latched_")
LINKS = sim_links(WORK)
_, _, LOG, _ = sim_files(WORK)
SPEED = 4
LATCHED = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_HIGH_LEVEL


def generate_test_description():
    return launch.LaunchDescription([fake_tron(LINKS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


class Driver:
    """tron_mcu as a process of its own, started the way the launch starts it again."""

    def __init__(self, start_latched, read_only=False):
        exe = package_program("tron_mcu", "tron_mcu_node")
        # After the newest entry of the log: what the driver before this one sent is not counted.
        self.since = last_time(LOG) + 0.001
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", f"start_latched:={str(start_latched).lower()}",
                                      "-p", f"read_only:={str(read_only).lower()}"], start_new_session=True)

    def stop(self):
        return stop_process(self.proc, group=True)


class Probe(DriverProbe):
    def __init__(self):
        super().__init__("start_latched_probe", log=LOG)
        self.statuses = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def _status(self, msg):
        with self.lock:
            self.statuses.append(msg)

    def clear(self):
        with self.lock:
            self.statuses = []

    def all(self):
        with self.lock:
            return list(self.statuses)

    def wait(self, pred, timeout=10.0):
        """The latest status once pred(status) holds, else None."""
        def fits():
            seen = self.all()
            return seen[-1] if seen and pred(seen[-1]) else None
        return wait_until(fits, timeout, poll=self.poll)

    def drive(self, seconds):
        """Commands motion at 20 Hz for that long."""
        msg = Twist()
        msg.linear.x = 0.2
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(msg)
            time.sleep(0.05)


class TestStartLatched(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.driver = Driver(start_latched=True)

    @classmethod
    def tearDownClass(cls):
        if cls.driver:
            cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def restart(self, **kwargs):
        self.driver.stop()
        self.probe.clear()
        type(self).driver = Driver(**kwargs)
        return self.probe.wait(lambda s: s.telemetry_ok, 15)

    def test_a_latched_from_the_first_status(self):
        s = self.probe.wait(lambda s: s.telemetry_ok, 15)
        self.assertTrue(s, "no telemetry")
        self.assertEqual(s.emergency_reason, LATCHED)
        self.assertEqual(s.latch_count, 1)
        for status in self.probe.all():
            self.assertTrue(status.emergency and status.emergency_latched, "a status without the latch")
            self.assertTrue(status.emergency_reason & HardwareStatus.REASON_HIGH_LEVEL, status.emergency_reason)

    def test_b_three_stops_with_the_first_frames(self):
        time.sleep(1.0)
        got = host_packets(LOG, self.driver.since)
        self.assertTrue(got, "the board got nothing")
        speeds = [e for e in got if e.get("type") == 0 and e["mod"] == SPEED]
        self.assertEqual(len(speeds), 3, speeds)
        self.assertFalse(any(moving(e) for e in speeds), speeds)
        # The first stop comes with the first frames of the driver, not after them.
        self.assertLessEqual(speeds[0]["t"] - got[0]["t"], 0.05)

    def test_c_no_command_reaches_the_board_until_the_release(self):
        since = last_time(LOG)
        self.probe.drive(1.0)
        self.assertEqual(sub_packets(LOG, since, SPEED), [])
        answer = self.probe.call(self.probe.emergency, SetEmergency.Request(latch=False, reason="test"))
        self.assertTrue(answer and answer.success, answer)
        self.assertTrue(self.probe.wait(lambda s: not s.emergency, 3.0), "the release did not end the emergency")
        time.sleep(0.6)
        self.assertEqual([e for e in sub_packets(LOG, since, SPEED) if moving(e)], [],
                         "the command from before the release moved the wheels")
        since = last_time(LOG)
        self.probe.drive(0.5)
        self.assertTrue([e for e in sub_packets(LOG, since, SPEED) if moving(e)], "no motion after the release")

    def test_d_read_only_neither_latches_nor_sends(self):
        s = self.restart(start_latched=True, read_only=True)
        self.assertTrue(s, "no telemetry after the restart")
        time.sleep(1.0)
        self.assertFalse(any(status.emergency_latched for status in self.probe.all()))
        self.assertEqual(host_packets(LOG, self.driver.since), [])

    def test_e_a_first_start_is_free_and_sends_no_stop(self):
        s = self.restart(start_latched=False)
        self.assertTrue(s, "no telemetry after the restart")
        time.sleep(1.0)
        self.assertFalse(self.probe.all()[-1].emergency, self.probe.all()[-1].emergency_reason)
        self.assertEqual(sub_packets(LOG, self.driver.since, SPEED), [])


if __name__ == "__main__":
    unittest.main()
