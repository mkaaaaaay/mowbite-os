"""tron_gnss against fake_tron, whose receiver reports RTK fixed without any RTCM from the host, as with a coupled LoRa
base of another reference (G24). Without <data_dir>/ntrip.yaml such a fix counts; with it, only once the first NTRIP
corrections went to the receiver. The test plays mowbite_ntrip: it writes the access file and sends RTCM on gnss/rtcm.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_gnss_lora.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import GnssFix, Rtcm
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import SimProbe, fake_tron, rtcm_frame, sim_files, sim_links, write_json  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_lora_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
ACCESS = os.path.join(WORK, "ntrip.yaml")
FIXED = GnssFix.RTK_FIXED
FLOAT = GnssFix.RTK_FLOAT

write_json(FLAGS, {})


def generate_test_description():
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0,
                             "data_dir": WORK}])
    # The pty has to exist before the driver looks for it.
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
        TimerAction(period=1.0, actions=[gnss, launch_testing.actions.ReadyToTest()]),
    ]), {}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("lora_probe")
        # (time, solution) of every pose.
        self.flags = []
        self.node.create_subscription(GnssFix, "gps/absolute_pose", self._pose, 10)
        self.node.create_subscription(Log, "/rosout", self._log, 100)
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=50,
                                                                              reliability=ReliabilityPolicy.RELIABLE))
        self.start()

    def _pose(self, msg):
        with self.lock:
            self.flags.append((time.time(), msg.solution))

    def _log(self, msg):
        if msg.name == "tron_gnss":
            with self.lock:
                self.logs.append(msg.msg)

    def latest(self):
        return self.flags[-1][1] if self.flags else None

    def flags_since(self, since):
        with self.lock:
            return [f for t, f in self.flags if t >= since]

    def send_rtcm(self):
        msg = Rtcm()
        msg.data = list(rtcm_frame(1005))
        self.rtcm_pub.publish(msg)


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
