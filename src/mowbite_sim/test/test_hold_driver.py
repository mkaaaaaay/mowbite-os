"""The hold of an update in the driver (S1), here tron_mcu against fake_tron on a pty: it reads <data_dir>/hold.json
itself, blocks motion and blade without an emergency, holds a file it cannot read, and holds from its first status after
a restart with the file there.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hold_driver.py
"""
import json
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist, Vector3
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetBlade

WORK = tempfile.mkdtemp(prefix="mowbite_hold_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
HOLD = os.path.join(WORK, "hold.json")
CUTTER = 11
SPEED = 4


def generate_test_description():
    with open(FLAGS, "w") as f:
        json.dump({}, f)
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"],
        output="screen")
    return launch.LaunchDescription([fake_tron, launch_testing.actions.ReadyToTest()]), {}


def received(since, module=None):
    """Sub packets fake_tron got from the driver since sim time `since`."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if entry.get("type") == 0 and entry["t"] >= since and (module is None or entry["mod"] == module):
                out.append(entry)
    return out


def sim_time():
    with open(LOG) as f:
        lines = f.readlines()
    return json.loads(lines[-1])["t"] if lines else 0.0


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
        exe = os.path.join(get_package_prefix("tron_mcu"), "lib", "tron_mcu", "tron_mcu_node")
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", "allow_blade:=true", "-p", f"hold_file:={HOLD}"],
                                     start_new_session=True)

    def stop(self):
        os.killpg(self.proc.pid, signal.SIGINT)
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait(5)
        return self.proc.returncode


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.statuses = []
        self.node = rclpy.create_node("hold_driver_probe")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

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
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.statuses:
                    return self.statuses[0]
            time.sleep(0.02)
        return None

    def wait(self, pred, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            s = self.status
            if s is not None and pred(s):
                return s
            time.sleep(0.02)
        return None

    def set_blade(self, on, lease_s=1.0):
        assert self.blade.wait_for_service(timeout_sec=5.0), "hw/blade missing"
        future = self.blade.call_async(SetBlade.Request(on=on, lease_s=lease_s))
        end = time.time() + 5.0
        while not future.done() and time.time() < end:
            time.sleep(0.01)
        return future.result()

    def drive(self, seconds):
        """cmd_vel for this long; the sim time it began."""
        t0 = sim_time()
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(Twist(linear=Vector3(x=0.2)))
            time.sleep(0.1)
        return t0

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


def moving(since):
    return [e for e in received(since, SPEED) if e["speed"] != [0.0, 0.0]]


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


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
