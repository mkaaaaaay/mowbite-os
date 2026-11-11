"""The start of a driver behind the hardware contract hw/*, here tron_mcu against fake_tron on a pty: until the first
sample of its IMU it refuses drive and blade without an emergency, for its start time at most; after it the driver
reports IMU_TIMEOUT, and the first sample frees it.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hw_start.py
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

WORK = tempfile.mkdtemp(prefix="mowbite_start_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
CUTTER = 11
SPEED = 4
# imu_start_timeout of the driver here, s: long enough to ask within it on a busy machine.
START = 6.0


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


def moving(since):
    return [e for e in received(since, SPEED) if e["speed"] != [0.0, 0.0]]


class Driver:
    """tron_mcu as a process of its own, so the test knows when it started."""

    def __init__(self):
        exe = os.path.join(get_package_prefix("tron_mcu"), "lib", "tron_mcu", "tron_mcu_node")
        self.started = time.time()
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=true",
                                      "-p", f"imu_start_timeout:={START}", "-p", "allow_blade:=true"],
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
        # (time of arrival, message)
        self.statuses = []
        self.node = rclpy.create_node("hw_start_probe")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.imu = self.node.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        # Whether the IMU of this test gives samples.
        self.sampling = False
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            if self.sampling:
                self.imu.publish(Imu(linear_acceleration=Vector3(z=9.81)))
            rclpy.spin_once(self.node, timeout_sec=0.02)

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


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
