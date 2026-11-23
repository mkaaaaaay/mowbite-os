"""tron_imu on a pty that plays the WIT sensor at 50 Hz: imu/data comes, and when the port fails (its other end closes
while a new one waits under the same name) the node opens it again with a gap in imu/data of at most 1 s.

Run in a sourced workspace: launch_test src/tron_imu/test/test_imu_reopen.py
"""
import os
import struct
import tempfile
import termios
import threading
import time
import tty
import unittest

import launch
import launch_testing.actions
import rclpy
from launch_ros.actions import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

LINK = os.path.join(tempfile.mkdtemp(prefix="tron_imu_"), "ttyS1")
PERIOD = 0.02
MAX_GAP = 1.0


def wit_frame(kind, values):
    body = bytearray([0x55, kind]) + struct.pack("<4h", *values)
    return bytes(body + bytes([sum(body) % 256]))


# At rest and upright: 1 g up (16 g full scale), no turn, level.
SAMPLE = wit_frame(0x51, [0, 0, 2048, 0]) + wit_frame(0x52, [0, 0, 0, 0]) + wit_frame(0x53, [0, 0, 0, 0])


def open_pty(link):
    """Master end of a new pty, put under link in one step; the slave stays closed so the node may open it."""
    master, slave = os.openpty()
    tty.setraw(master)
    attrs = termios.tcgetattr(master)
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(master, termios.TCSANOW, attrs)
    os.set_blocking(master, False)
    os.symlink(os.ttyname(slave), link + ".new")
    os.replace(link + ".new", link)
    os.close(slave)
    return master


class Sensor:
    """Writes SAMPLE every PERIOD to the current pty; fail() puts a new pty under the link and closes the old one."""

    def __init__(self):
        self.lock = threading.Lock()
        self.master = open_pty(LINK)
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            with self.lock:
                try:
                    os.write(self.master, SAMPLE)
                except OSError:
                    pass
            time.sleep(PERIOD)

    def fail(self):
        with self.lock:
            old, self.master = self.master, open_pty(LINK)
            os.close(old)


SENSOR = Sensor()


def generate_test_description():
    node = Node(package="tron_imu", executable="tron_imu_node", name="tron_imu", output="screen",
                parameters=[{"port": LINK}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


class TestImuReopen(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("imu_reopen_probe")
        cls.stamps = []
        cls.lock = threading.Lock()
        cls.node.create_subscription(Imu, "imu/data", cls._imu, qos_profile_sensor_data)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        SENSOR.running = False
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _imu(cls, msg):
        with cls.lock:
            cls.stamps.append(time.monotonic())

    def since(self, t):
        with self.lock:
            return [s for s in self.stamps if s >= t]

    def wait_for(self, t, count, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and len(self.since(t)) < count:
            time.sleep(0.05)
        return len(self.since(t)) >= count

    def test_1_samples_come(self):
        self.assertTrue(self.wait_for(time.monotonic(), 50, 15.0), "no imu/data from the pty")

    def test_2_after_a_failed_port_it_reads_again_within_a_second(self):
        start = time.monotonic()
        self.assertTrue(self.wait_for(start, 25, 5.0), "no imu/data before the failure")
        failed = time.monotonic()
        SENSOR.fail()
        self.assertTrue(self.wait_for(failed, 50, 5.0), "no imu/data after the port failed")
        stamps = self.since(start)
        gap = max(b - a for a, b in zip(stamps, stamps[1:]))
        print(f"reopen: largest gap in imu/data {gap:.3f} s")
        self.assertLessEqual(gap, MAX_GAP)
