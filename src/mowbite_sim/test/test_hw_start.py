"""The start of a driver behind the hardware contract hw/*, here tron_mcu against fake_tron on a pty: until the first
sample of its IMU it refuses drive and blade without an emergency, for its start time at most; after it the driver
reports IMU_TIMEOUT, and the first sample frees it.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hw_start.py
"""
import functools
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
from geometry_msgs.msg import Twist, Vector3
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetBlade
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import TRON_CUTTER as CUTTER, TRON_SPEED as SPEED  # noqa: E402
from sim_common import (DriverProbe, TestShutdown, fake_tron, package_program, sim_files, sim_links,  # noqa: E402,F401
                        stop_process, sub_packets, wheel_speeds, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_start_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
# imu_start_timeout of the driver here, s: long enough to ask within it on a busy machine.
START = 6.0


def generate_test_description():
    write_json(FLAGS, {})
    return launch.LaunchDescription([fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


received = functools.partial(sub_packets, LOG)


def moving(since):
    return wheel_speeds(LOG, since, SPEED)


class Driver:
    """tron_mcu as a process of its own, so the test knows when it started."""

    def __init__(self):
        exe = package_program("tron_mcu", "tron_mcu_node")
        self.started = time.time()
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=true",
                                      "-p", f"imu_start_timeout:={START}", "-p", "allow_blade:=true"],
                                     start_new_session=True)

    def stop(self):
        return stop_process(self.proc, signal.SIGINT, group=True)


class Probe(DriverProbe):
    spin_timeout = 0.02

    def __init__(self):
        super().__init__("hw_start_probe", log=LOG)
        # (time of arrival, message)
        self.statuses = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.imu = self.node.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        # Whether the IMU of this test gives samples.
        self.sampling = False
        self.start()

    def _spin(self):
        while self.running:
            if self.sampling:
                self.imu.publish(Imu(linear_acceleration=Vector3(z=9.81)))
            rclpy.spin_once(self.node, timeout_sec=self.spin_timeout)

    def _status(self, msg):
        with self.lock:
            self.statuses.append((time.time(), msg))

    def wait(self, pred, timeout, fresh=False):
        """(arrival, status) of the first status that fits; with fresh only of those still to come."""
        with self.lock:
            seen = len(self.statuses) if fresh else 0
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                new = self.statuses[seen:]
            for entry in new:
                if pred(entry[1]):
                    return entry
            seen += len(new)
            time.sleep(self.poll)
        return None

    def set_blade(self, on, lease_s=1.0):
        return super().set_blade(on, lease_s)

    def drive(self, seconds):
        return self.cmd_vel_for(0.2, seconds)


class TestHardwareStart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.driver = Driver()

    @classmethod
    def tearDownClass(cls):
        cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_a_blocked_without_an_emergency_in_the_start_time(self):
        # Half the start time is left for the questions below.
        free = self.probe.wait(lambda s: s.telemetry_ok and not s.emergency, START / 2)
        self.assertTrue(free, "no status with telemetry and without an emergency in the first half of the start time")
        r = self.probe.set_blade(True)
        self.assertFalse(r.success)
        self.assertIn("blocked", r.message)
        self.assertEqual(r.emergency_reason, 0)
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertEqual(moving(t0), [], "the wheels got a speed before the first IMU sample")
        self.assertEqual([e for e in received(t0, CUTTER) if e["hex"][:2] == "01"], [], "blade on without an IMU")

    def test_b_emergency_after_the_start_time(self):
        first = self.probe.wait(lambda s: True, 1.0)
        late = self.probe.wait(lambda s: s.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, START + 5.0)
        self.assertTrue(late, "no IMU_TIMEOUT after the start time")
        self.assertTrue(late[1].emergency)
        # Not latched: the first sample frees it.
        self.assertEqual(late[1].emergency_reason, HardwareStatus.REASON_IMU_TIMEOUT)
        self.assertGreaterEqual(late[0] - self.driver.started, START)
        self.assertLessEqual(late[0] - first[0], START + 1.0)
        r = self.probe.set_blade(True)
        self.assertFalse(r.success)
        self.assertEqual(r.emergency_reason, HardwareStatus.REASON_IMU_TIMEOUT)

    def test_c_free_with_the_first_sample(self):
        self.probe.sampling = True
        self.assertTrue(self.probe.wait(lambda s: not s.emergency, 2.0, fresh=True),
                        "still an emergency 2 s after the first IMU sample")
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertGreater(len(moving(t0)), 3, "no motion with the IMU there")
        r = self.probe.set_blade(True)
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.probe.set_blade(False).success)
