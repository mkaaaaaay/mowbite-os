"""tron_gnss against fake_tron, whose receiver reports RTK fixed without any RTCM from the host, as with a coupled LoRa
base of another reference (G24). Without <data_dir>/ntrip.yaml such a fix counts; with it, only once the first NTRIP
corrections went to the receiver. The test plays mowbite_ntrip: it writes the access file and sends RTCM on gnss/rtcm.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_gnss_lora.py
"""
import importlib.machinery
import importlib.util
import json
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import AbsolutePose, Rtcm
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy

WORK = tempfile.mkdtemp(prefix="mowbite_lora_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
ACCESS = os.path.join(WORK, "ntrip.yaml")
FIXED = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FIXED
FLOAT = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FLOAT

# The CRC of fake_tron, so there is one in the sim.
_loader = importlib.machinery.SourceFileLoader(
    "fake_tron", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "fake_tron"))
ft = importlib.util.module_from_spec(importlib.util.spec_from_loader("fake_tron", _loader))
_loader.exec_module(ft)

with open(FLAGS, "w") as f:
    json.dump({}, f)


def rtcm_frame(message=1005, length=19):
    payload = bytes([message >> 4, (message & 0x0F) << 4]) + bytes((i * 7) & 0xFF for i in range(length - 2))
    frame = bytes([0xD3, length >> 8, length & 0xFF]) + payload
    return frame + ft.crc24q(frame).to_bytes(3, "big")


def generate_test_description():
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"],
        output="screen")
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0,
                             "data_dir": WORK}])
    # The pty has to exist before the driver looks for it.
    return launch.LaunchDescription([
        fake_tron,
        TimerAction(period=1.0, actions=[gnss, launch_testing.actions.ReadyToTest()]),
    ]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.flags = []
        self.logs = []
        self.node = rclpy.create_node("lora_probe")
        self.node.create_subscription(AbsolutePose, "gps/absolute_pose", self._pose, 10)
        self.node.create_subscription(Log, "/rosout", self._log, 100)
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=50,
                                                                              reliability=ReliabilityPolicy.RELIABLE))
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _pose(self, msg):
        with self.lock:
            self.flags.append((time.time(), msg.flags))

    def _log(self, msg):
        if msg.name == "tron_gnss":
            with self.lock:
                self.logs.append(msg.msg)

    def latest(self):
        return self.flags[-1][1] if self.flags else None

    def flags_since(self, since):
        with self.lock:
            return [f for t, f in self.flags if t >= since]

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def send_rtcm(self):
        msg = Rtcm()
        msg.data = list(rtcm_frame())
        self.rtcm_pub.publish(msg)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestGnssLora(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_without_ntrip_a_fix_from_elsewhere_counts(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.latest() == FIXED, 20), f"flags {p.latest()}")

    def test_2_with_ntrip_set_up_it_counts_as_float(self):
        p = self.probe
        with open(ACCESS, "w") as f:
            f.write("host: 127.0.0.1\nmountpoint: TEST\n")
        self.assertTrue(p.wait(lambda: p.latest() == FLOAT, 5), f"flags {p.latest()}")
        since = time.time()
        time.sleep(2.0)
        self.assertEqual(set(p.flags_since(since)), {FLOAT})
        self.assertTrue(p.wait(lambda: any("before any NTRIP corrections went to the receiver" in line
                                           for line in p.logs), 5), p.logs)
        self.assertTrue(any("ntrip.yaml is there" in line for line in p.logs), p.logs)

    def test_3_fixed_again_once_ntrip_corrections_went_through(self):
        p = self.probe
        # The first frames wake the board (network RTK ON), then they pass to the receiver.
        end = time.time() + 15.0
        while time.time() < end and p.latest() != FIXED:
            p.send_rtcm()
            time.sleep(0.5)
        self.assertEqual(p.latest(), FIXED)
        with open(LOG) as f:
            entries = [json.loads(line) for line in f]
        self.assertTrue(any("rtcm" in e for e in entries), "no RTCM reached the receiver")
