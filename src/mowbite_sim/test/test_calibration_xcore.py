"""The calibration drive on an OpenMower v2 mower (NX60i): xcore_driver against fake_xcore and the nodes of the container
with the geometry of config/geometry/nx60i.yaml, the board off its configuration on purpose: ticks 3 % more than the
2025 per m of the driver, 0.37 m between the wheels against 0.3556, the antenna at -0.10/0.02 where the geometry says
0/0, the IMU at 2 deg roll and 3 deg pitch (the platform's mounting is 4.5 deg pitch) on ground that rises 3 deg.

Checks that a level-ish mower with the IMU of the NX60i passes the start (the slope counts from the stored mounting),
a drive through its first line and turn, the head of the recording with the basis of the xcore_driver and the
mounting of the platform, calibration.abort, and the blade never on.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_calibration_xcore.py
"""
import gzip
import json
import math
import os
import random
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import (HardwareStatus, MapStatus, MqttPublish, RobotPose, Rtcm, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_calibration_xcore_")
FLAGS = os.path.join(WORK, "flags.json")
STATE = os.path.join(WORK, "state.json")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "extra.yaml")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DATUM = (52.0, 9.0, 80.0)
LAWN = [(-3.0, -3.0), (8.0, -3.0), (8.0, 3.0), (-3.0, 3.0)]

with open(CONFIG, "w") as f:
    f.write("mowbite_logic:\n  ros__parameters:\n    geometry:\n      antenna_x: 0.0\n      antenna_y: 0.0\n")
with open(FLAGS, "w") as f:
    json.dump({"gps_fix": "fix"}, f)


def generate_test_description():
    sim = FindPackageShare("mowbite_sim").find("mowbite_sim")
    bringup = FindPackageShare("mowbite_bringup").find("mowbite_bringup")
    config = os.path.join(bringup, "config", "tron.yaml")
    geometry = os.path.join(bringup, "config", "geometry", "nx60i.yaml")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim, "config", "xcore_nx60i_services.json"), "--group-port", str(GROUP_PORT),
             "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
             "--ticks-scale", "1.03", "--track", "0.37", "--antenna-offset", "-0.10", "0.02",
             "--imu-error-rpy-deg", "2.0", "3.0", "0.0", "--slope", "3.0", "40.0", "--gnss-noise-m", "0.01",
             "--gnss-delay", "0.1"],
        output="screen")
    driver_config = os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config", "nx60i.yaml")
    driver = Node(package="xcore_driver", executable="xcore_driver_node", name="xcore_driver", namespace="/",
                  output="screen",
                  parameters=[driver_config, {"interface": "lo", "bind_ip": BIND_IP, "announcement_port": GROUP_PORT,
                                              "datum_lat": DATUM[0], "datum_long": DATUM[1], "datum_height": DATUM[2],
                                              "safety_measurement_date": "simulation"}])

    def component(package, plugin, name, extra=None):
        return ComposableNode(package=package, plugin=plugin, name=name,
                              parameters=[geometry, config, CONFIG, *([extra] if extra else [])])

    container = ComposableNodeContainer(
        name="mowbite", namespace="", package="rclcpp_components", executable="component_container", output="screen",
        composable_node_descriptions=[
            component("mowbite_localization", "mowbite_localization::LocalizationNode", "mowbite_localization"),
            component("mowbite_safety", "mowbite_safety::SafetyNode", "mowbite_safety"),
            component("mowbite_logic", "mowbite_logic::LogicNode", "mowbite_logic", {"data_dir": DATA}),
            component("mowbite_bridge", "mowbite_bridge::BridgeNode", "mowbite_bridge",
                      {"mqtt_port": 18838, "data_dir": DATA}),
            component("mowbite_map", "mowbite_map::MapNode", "mowbite_map", {"data_dir": DATA}),
            component("mowbite_calibration", "mowbite_calibration::CalibrationNode", "mowbite_calibration",
                      {"data_dir": DATA}),
        ])
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, container, launch_testing.actions.ReadyToTest()]),
    ]), {}


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def station_frame():
    """An RTCM 3 frame of message 1005 (reference station), all fields 0: the driver only has to pass corrections on."""
    payload = bytes([1005 >> 4, (1005 & 0xF) << 4]) + bytes(17)
    head = bytes([0xD3, len(payload) >> 8, len(payload) & 0xFF]) + payload
    return head + crc24q(head).to_bytes(3, "big")


def board():
    with open(STATE) as f:
        return json.load(f)


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_status = None
        self.map = None
        self.pose = None
        self.hw = None
        self.blade_on = False
        self.events = []
        self.responses = {}
        self.errors = {}
        self.calls = 0
        self.node = rclpy.create_node("calibration_xcore_probe")
        self.node.create_subscription(MapStatus, "map/status", self._map_status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        # Corrections once a second, as a caster would send them: RTK fixed counts only with them (YF-c).
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=10))
        self.node.create_timer(1.0, lambda: self.rtcm_pub.publish(Rtcm(data=list(station_frame()))))
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _map_status(self, msg):
        with self.lock:
            self.map_status = msg

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _hw(self, msg):
        with self.lock:
            self.hw = msg
            self.blade_on = self.blade_on or msg.blade_on

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _response(self, msg):
        with self.lock:
            self.responses[msg.id] = json.loads(msg.result)

    def _error(self, msg):
        with self.lock:
            self.errors[msg.id] = (msg.code, msg.message, json.loads(msg.data) if msg.data else None)

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def service(self, client, request, timeout=60.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def rpc(self, method, params=None, timeout=5.0):
        self.calls += 1
        rid = f"calib_xcore_{self.calls}"
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params) if params is not None else "",
                                        id=rid))
        return self.wait(lambda: (("ok", self.responses[rid]) if rid in self.responses else None)
                         or (("error", self.errors[rid]) if rid in self.errors else None), timeout)

    def event(self, since, type_, timeout, **match):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestCalibrationXcore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_ready_with_the_imu_of_the_nx60i(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.map_status is not None
                               and p.map_status.datum_state != MapStatus.DATUM_UNKNOWN, 30), "no datum")
        request = ReplaceMap.Request()
        request.map = json.dumps({"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                                             "outline": [{"x": x, "y": y} for x, y in LAWN]}],
                                  "docking_stations": []})
        self.assertTrue(p.service(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.map_status.mowing_allowed, 10), "the map does not fit the datum")
        truth = board()
        request = SetPose.Request()
        request.pose.position.x = truth["x"]
        request.pose.position.y = truth["y"]
        request.pose.orientation.z = math.sin(truth["yaw"] / 2)
        request.pose.orientation.w = math.cos(truth["yaw"] / 2)
        p.service(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03, 40),
                        "no RTK pose with a heading")
        end = time.time() + 60
        check = None
        while time.time() < end:
            check = p.rpc("calibration.check")
            if check and check[0] == "ok" and check[1]["ok"]:
                break
            time.sleep(1.0)
        self.assertTrue(check and check[0] == "ok" and check[1]["ok"], check)
        self.assertAlmostEqual(check[1]["space"]["lane_length"], 3.0)

    def test_02_first_line_and_turn_then_stopped(self):
        p = self.probe
        since = len(p.events)
        self.assertEqual(p.rpc("calibration.start"), ("ok", {"state": "CALIBRATING"}))
        spin = p.event(since, "CALIBRATION", 120, phase="spin_left")
        self.assertIsNotNone(spin, [e for e in p.events[since:] if e["type"].startswith("CALIBRAT")])
        time.sleep(3.0)
        self.assertEqual(p.rpc("calibration.abort"), ("ok", {"ok": True}))
        failed = p.event(since, "CALIBRATION_FAILED", 5)
        self.assertEqual(failed["reason"], "STOPPED", failed)
        self.assertIsNotNone(p.event(since, "CALIBRATION", 5, phase="end"))
        a = board()
        time.sleep(1.0)
        b = board()
        self.assertLess(math.hypot(a["x"] - b["x"], a["y"] - b["y"]), 0.01)
        self.assertFalse(p.blade_on)
        self.assertEqual(b["mower_rpm"], 0.0)
        folder = os.path.join(DATA, "calibration", "recordings")
        files = sorted(os.path.join(folder, n) for n in os.listdir(folder) if n.endswith(".jsonl.gz"))
        self.assertEqual(len(files), 1)
        with gzip.open(files[0], "rt") as f:
            lines = [json.loads(line) for line in f]
        head = lines[0]
        self.assertEqual(head["platform"], "yardforce_nx60i")
        # The board's clock wraps, so no receiver time (gnss_receiver_time false).
        gps = [m for m in lines if m.get("k") == "gps"]
        self.assertTrue(gps)
        self.assertFalse([m for m in gps if "rt" in m])
        self.assertEqual(head["basis"], {"ticks_per_m": 2025.0, "wheel_distance_m": 0.3556})
        self.assertEqual(head["params"]["ticks_per_m"], 2025.0)
        self.assertEqual(head["imu"], {"roll": 0.0, "pitch": 0.079})
        self.assertEqual((lines[-1]["k"], lines[-1]["why"]), ("end", "stop"))
        marks = [(m["name"], m["edge"]) for m in lines if m.get("k") == "phase"]
        self.assertIn(("line", "end"), marks)
        self.assertIn(("spin_left", "start"), marks)
        self.assertEqual(p.rpc("calibration.result")[1]["status"], "failed")
