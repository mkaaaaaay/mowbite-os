"""The hold of an update in the driver (S1), here tron_mcu against fake_tron on a pty: it reads <data_dir>/hold.json
itself, blocks motion and blade without an emergency, holds a file it cannot read, and holds from its first status after
a restart with the file there.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hold_driver.py
"""
import functools
import json
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
from geometry_msgs.msg import Twist
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetBlade

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (DriverProbe, TestShutdown, fake_tron, package_program, sim_files, sim_links,  # noqa: E402,F401
                        stop_process, sub_packets, wait_until, wheel_speeds, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_hold_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
HOLD = os.path.join(WORK, "hold.json")
CUTTER = 11
SPEED = 4


def generate_test_description():
    write_json(FLAGS, {})
    return launch.LaunchDescription([fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


received = functools.partial(sub_packets, LOG)


def put(text):
    with open(HOLD + ".new", "w") as f:
        f.write(text)
    os.replace(HOLD + ".new", HOLD)


def hold_for(seconds):
    now = int(time.time())
    put(json.dumps({"until": now + seconds, "set_at": now}))


class Driver:
    """tron_mcu as a process of its own, so the test can stop it and start it again."""

    def __init__(self):
        exe = package_program("tron_mcu", "tron_mcu_node")
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", "allow_blade:=true", "-p", f"hold_file:={HOLD}"],
                                     start_new_session=True)

    def stop(self):
        return stop_process(self.proc, signal.SIGINT, group=True)


class Probe(DriverProbe):
    def __init__(self):
        super().__init__("hold_driver_probe", log=LOG)
        self.statuses = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
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

    def first(self, timeout=10.0):
        return wait_until(lambda: self.statuses[0] if self.statuses else None, timeout, self.lock, self.poll)

    def wait(self, pred, timeout=10.0):
        """The latest status once pred(status) holds, else None."""
        def fits():
            s = self.status
            return s if s is not None and pred(s) else None
        return wait_until(fits, timeout, poll=self.poll)

    def set_blade(self, on, lease_s=1.0):
        return super().set_blade(on, lease_s)

    def drive(self, seconds):
        return self.cmd_vel_for(0.2, seconds)


def moving(since):
    return wheel_speeds(LOG, since, SPEED)


class TestHoldDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        hold_for(600)
        cls.driver = Driver()

    @classmethod
    def tearDownClass(cls):
        if cls.driver:
            cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_a_held_from_the_start(self):
        first = self.probe.first(15)
        self.assertIsNotNone(first, "no hw/status")
        self.assertTrue(first.hold, "the first status is not held")
        s = self.probe.wait(lambda s: s.telemetry_ok, 10)
        self.assertTrue(s, "no telemetry")
        self.assertTrue(s.hold)
        self.assertFalse(s.emergency)
        self.assertEqual(s.emergency_reason, 0)
        t0 = self.probe.drive(1.5)
        time.sleep(0.3)
        self.assertEqual(moving(t0), [], "the wheels got a speed while held")
        r = self.probe.set_blade(True)
        self.assertFalse(r.success)
        self.assertIn("held", r.message)
        self.assertEqual(r.emergency_reason, 0)
        self.assertEqual([e for e in received(t0, CUTTER) if e["hex"][:2] == "01"], [], "blade on while held")

    def test_b_free_once_the_file_is_gone(self):
        os.remove(HOLD)
        s = self.probe.wait(lambda s: not s.hold, 2.0)
        self.assertTrue(s, "still held 2 s after the file went")
        self.assertFalse(s.emergency)
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertGreater(len(moving(t0)), 3, "no motion after the hold")
        self.assertTrue(self.probe.set_blade(False).success)

    def test_c_a_file_it_cannot_read_holds(self):
        # The wheels stop on their watchdog first.
        time.sleep(1.0)
        put('{"until": ')
        s = self.probe.wait(lambda s: s.hold, 2.0)
        self.assertTrue(s, "not held by a broken file")
        self.assertFalse(s.emergency)
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertEqual(moving(t0), [])
        os.remove(HOLD)
        self.assertTrue(self.probe.wait(lambda s: not s.hold, 2.0), "still held after the broken file went")

    def test_d_held_at_once_after_a_restart(self):
        time.sleep(1.0)
        self.assertIn(self.driver.stop(), (0, -2, 130))
        hold_for(900)
        self.probe.clear()
        type(self).driver = Driver()
        first = self.probe.first(15)
        self.assertIsNotNone(first, "no hw/status after the restart")
        self.assertTrue(first.hold, "the first status after the restart is not held")
        self.assertTrue(self.probe.wait(lambda s: s.telemetry_ok and s.hold, 10))
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertEqual(moving(t0), [])

    def test_e_a_hold_that_ran_out_holds_nothing(self):
        now = int(time.time())
        put(json.dumps({"until": now - 5, "set_at": now - 600}))
        self.assertTrue(self.probe.wait(lambda s: not s.hold, 2.0), "held by a hold that ran out")
        t0 = self.probe.drive(1.0)
        time.sleep(0.3)
        self.assertGreater(len(moving(t0)), 3)
