"""GNSS of an OpenMower v2 mower: xcore_driver turns the solutions of fake_xcore's GPS service into gps/absolute_pose,
gives mowbite_ntrip a GGA of its own for a test caster that streams only after one (like a VRS) and passes the RTCM to
the board in pieces of one datagram. Checks the datum, the position, RTK fixed only on corrections the driver passed
within max_correction_age, float and no RTK as the board reports them, a silent GPS service and that every RTCM byte
reaches the board unchanged.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_gnss.py
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import random
import shutil
import tempfile
import threading
import time
import unittest
import zlib

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import AbsolutePose, HardwareCapabilities, Rtcm, SensorValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_gnss_")
FLAGS = os.path.join(WORK, "flags.json")
STATE = os.path.join(WORK, "state.json")
ACCESS = os.path.join(WORK, "ntrip.yaml")
GROUP_PORT = random.randint(42000, 48000)
# An address of its own for the driver: it does not start beside another host at its bind_ip.
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAX_AGE = 3.0
DATUM = (52.0, 9.0, 80.0)
# Where fake_xcore puts the mower: about 11 m north and 14 m east of the datum, at the datum's height.
ORIGIN = (52.0001, 9.0002)
FIXED = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FIXED
FLOAT = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FLOAT
MAX_PIECE = 1448

# The caster of test_ntrip and the CRC of fake_tron, so that there is one of each in the sim.
_here = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    loader = importlib.machinery.SourceFileLoader(name, path)
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader))
    loader.exec_module(module)
    return module


tn = _load("test_ntrip_caster", os.path.join(_here, "test_ntrip.py"))
# Its access file and flags are not used here.
shutil.rmtree(tn.WORK, ignore_errors=True)
CASTER = tn.CASTER
# Down until the first test has seen the driver without corrections.
CASTER.down = True
with open(ACCESS + ".tmp", "w") as f:
    f.write(f"host: 127.0.0.1\nport: {CASTER.port}\nmountpoint: V2\nuser: {tn.USER}\npassword: '{tn.PASSWORD}'\n"
            "version: 2\n")
os.chmod(ACCESS + ".tmp", 0o600)
os.replace(ACCESS + ".tmp", ACCESS)


def set_flags(**flags):
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


set_flags(gps_fix="fix")


def generate_test_description():
    sim = FindPackageShare("mowbite_sim").find("mowbite_sim")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim, "config", "xcore_nx60i_services.json"), "--group-port", str(GROUP_PORT),
             "--origin", str(ORIGIN[0]), str(ORIGIN[1]), "--flags", FLAGS, "--state", STATE],
        output="screen")
    config = os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config", "nx60i.yaml")
    driver = Node(package="xcore_driver", executable="xcore_driver_node", output="screen",
                  parameters=[config, {"interface": "lo", "bind_ip": BIND_IP, "announcement_port": GROUP_PORT,
                                       "datum_lat": DATUM[0], "datum_long": DATUM[1], "datum_height": DATUM[2],
                                       "max_correction_age": MAX_AGE}])
    ntrip = Node(package="mowbite_ntrip", executable="mowbite_ntrip_node", output="screen",
                 parameters=[{"data_dir": WORK, "config_check": 1.0, "first_retry": 1.0, "max_retry": 2.0,
                              "data_timeout": 3.0, "gga_interval": 1.0, "max_correction_age": MAX_AGE}])
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, ntrip, launch_testing.actions.ReadyToTest()]),
    ]), {}


def board():
    with open(STATE) as f:
        return json.load(f)


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.poses = []
        self.datum = None
        self.params = []
        self.gga = []
        self.capabilities = None
        self.age = None
        self.rtcm_bytes = 0
        self.rtcm_crc = 0
        self.node = rclpy.create_node("xcore_gnss_probe")
        self.node.create_subscription(AbsolutePose, "gps/absolute_pose", self._pose, 50)
        self.node.create_subscription(NavSatFix, "gps/datum", self._datum, LATCHED)
        self.node.create_subscription(String, "bridge/params", self._params, HISTORY)
        self.node.create_subscription(String, "gnss/gga", self._gga, 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._capabilities, LATCHED)
        self.node.create_subscription(SensorValue, "bridge/sensor_value", self._sensor, 100)
        self.node.create_subscription(Rtcm, "gnss/rtcm", self._rtcm, QoSProfile(depth=50))
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=50))
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _pose(self, msg):
        with self.lock:
            self.poses = (self.poses + [(time.time(), msg)])[-200:]

    def _datum(self, msg):
        with self.lock:
            self.datum = msg

    def _params(self, msg):
        with self.lock:
            self.params.append(json.loads(msg.data))

    def _gga(self, msg):
        with self.lock:
            self.gga = (self.gga + [msg.data])[-20:]

    def _capabilities(self, msg):
        with self.lock:
            self.capabilities = msg

    def _sensor(self, msg):
        if msg.sensor_id == "os_correction_age":
            with self.lock:
                self.age = msg.value

    def _rtcm(self, msg):
        data = bytes(msg.data)
        with self.lock:
            self.rtcm_bytes += len(data)
            self.rtcm_crc = zlib.crc32(data, self.rtcm_crc)

    def wait(self, pred, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = pred(self)
            if value:
                return value
            time.sleep(0.02)
        return None

    def poses_since(self, since):
        with self.lock:
            return [m for t, m in self.poses if t >= since]

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


def latest_flags(p):
    return p.poses[-1][1].flags if p.poses else None


class TestXcoreGnss(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_datum_and_source(self):
        p = self.probe
        datum = p.wait(lambda p: p.datum, 10)
        self.assertTrue(datum, "no datum")
        self.assertEqual((datum.latitude, datum.longitude, datum.altitude), DATUM)
        params = p.wait(lambda p: [e for e in p.params if e["node"] == "/xcore_driver"], 5)
        self.assertTrue(params, p.params)
        self.assertEqual(params[0]["params"], {"datum_lat": DATUM[0], "datum_long": DATUM[1]})
        caps = p.wait(lambda p: p.capabilities, 10)
        self.assertEqual(caps.gnss_source, "xcore_driver")

    def test_2_fixed_counts_as_float_without_corrections(self):
        p = self.probe
        self.assertTrue(p.wait(lambda p: len(p.poses) >= 5, 20), "no poses")
        # The board says FIX, but the driver has passed no corrections yet.
        since = time.time()
        time.sleep(1.0)
        poses = p.poses_since(since)
        self.assertGreaterEqual(len(poses), 3)
        self.assertTrue(all(m.flags == FLOAT for m in poses), [m.flags for m in poses])
        self.assertTrue(all(m.position_accuracy >= 0.3 - 1e-6 for m in poses))
        # The caster gets a GGA anyway: VRS mountpoints stream only after one.
        gga = p.wait(lambda p: p.gga[-1] if p.gga else None, 5)
        self.assertTrue(gga.startswith("$GPGGA,"), gga)
        body, _, checksum = gga[1:].partition("*")
        crc = 0
        for c in body.encode():
            crc ^= c
        self.assertEqual(int(checksum, 16), crc)
        fields = body.split(",")
        self.assertEqual(fields[6], "4")
        self.assertAlmostEqual(int(fields[2][:2]) + float(fields[2][2:]) / 60.0, ORIGIN[0], places=7)
        self.assertEqual(fields[3], "N")
        self.assertAlmostEqual(int(fields[4][:3]) + float(fields[4][3:]) / 60.0, ORIGIN[1], places=7)
        self.assertEqual(fields[5], "E")
        self.assertAlmostEqual(float(fields[9]), 80.0, places=2)

    def test_3_position_in_the_frame_of_the_datum(self):
        p = self.probe
        pose = p.wait(lambda p: p.poses[-1][1] if p.poses else None, 5)
        north = math.radians(ORIGIN[0] - DATUM[0]) * 6378137.0
        east = math.radians(ORIGIN[1] - DATUM[1]) * 6378137.0 * math.cos(math.radians(ORIGIN[0]))
        # fake_xcore uses a sphere, the driver the ellipsoid: a few dm over this distance.
        self.assertAlmostEqual(pose.pose.pose.position.y, north, delta=0.15)
        self.assertAlmostEqual(pose.pose.pose.position.x, east, delta=0.15)
        self.assertAlmostEqual(pose.pose.pose.position.z, 0.0, delta=0.05)
        self.assertEqual(pose.source, AbsolutePose.SOURCE_GPS)
        self.assertEqual(pose.header.frame_id, "map")
        self.assertEqual(pose.motion_vector_valid, 1)

    def test_4_corrections_from_the_caster_give_fixed(self):
        p = self.probe
        CASTER.down = False
        self.assertTrue(p.wait(lambda p: latest_flags(p) == FIXED, 30), f"flags {latest_flags(p)}")
        pose = p.poses[-1][1]
        # The board's accuracy, not the table.
        self.assertAlmostEqual(pose.position_accuracy, 0.012, places=5)
        with CASTER.lock:
            gga = list(CASTER.gga)
        self.assertTrue(gga and gga[0].startswith("$GPGGA,"), gga)
        # fake_xcore writes its state every 0.1 s, it may lag behind the pose.
        b = p.wait(lambda p: (lambda b: b if b["rtcm_bytes"] > 0 else None)(board()), 2)
        self.assertTrue(b, "no RTCM at the board")
        self.assertLessEqual(b["rtcm_max_datagram"], MAX_PIECE)
        self.assertTrue(p.wait(lambda p: p.age is not None and 0 <= p.age < MAX_AGE, 5), p.age)
        self.assertEqual(b["gps_restarts"], 0)

    def test_5_large_rtcm_in_pieces_and_unchanged(self):
        p = self.probe
        frames = b"".join(tn.rtcm_frame(1077, 1000) for _ in range(5))
        with p.lock:
            before = p.rtcm_bytes
        p.rtcm_pub.publish(Rtcm(data=list(frames)))
        CASTER.down = True
        self.assertTrue(p.wait(lambda p: p.rtcm_bytes >= before + len(frames), 5), "own RTCM not seen")
        # Every byte the driver got reached the board, in the same order.
        ok = p.wait(lambda p: board()["rtcm_bytes"] == p.rtcm_bytes and board()["rtcm_crc"] == p.rtcm_crc, 10)
        b = board()
        self.assertTrue(ok, f"board {b['rtcm_bytes']} bytes, sent {p.rtcm_bytes}")
        self.assertGreaterEqual(b["rtcm_bytes"], len(frames))
        self.assertEqual(b["rtcm_max_datagram"], MAX_PIECE)

    def test_6_old_corrections_count_as_float(self):
        p = self.probe
        CASTER.down = True
        self.assertTrue(p.wait(lambda p: latest_flags(p) == FLOAT, MAX_AGE + 5), f"flags {latest_flags(p)}")
        self.assertTrue(p.wait(lambda p: p.age is not None and p.age > MAX_AGE, 3), p.age)
        CASTER.down = False
        self.assertTrue(p.wait(lambda p: latest_flags(p) == FIXED, 30), f"flags {latest_flags(p)}")

    def test_7_float_and_no_rtk_as_reported(self):
        p = self.probe
        set_flags(gps_fix="float")
        self.assertTrue(p.wait(lambda p: latest_flags(p) == FLOAT, 5))
        self.assertAlmostEqual(p.poses[-1][1].position_accuracy, 0.25, places=5)
        set_flags(gps_fix="none")
        self.assertTrue(p.wait(lambda p: latest_flags(p) == 0, 5))
        self.assertAlmostEqual(p.poses[-1][1].position_accuracy, 1.5, places=5)
        gga = p.wait(lambda p: p.gga[-1].split(",")[6] == "1" and p.gga[-1], 5)
        self.assertTrue(gga, p.gga[-1:])

    def test_8_silent_gps_gives_no_pose(self):
        p = self.probe
        set_flags(gps_fix="off")
        time.sleep(0.5)
        since = time.time()
        time.sleep(1.5)
        self.assertEqual(p.poses_since(since), [])
        set_flags(gps_fix="fix")
        self.assertTrue(p.wait(lambda p: latest_flags(p) == FIXED, 10), f"flags {latest_flags(p)}")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
