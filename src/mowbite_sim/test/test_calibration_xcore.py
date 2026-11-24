"""The calibration drive on an OpenMower v2 mower (NX60i): xcore_driver against fake_xcore and the nodes of the container
with geometry.yaml of the NX60i, the board off its configuration on purpose: ticks 3 % more than the
1815.1 per m of the driver, 0.37 m between the wheels against 0.3014, the antenna at -0.10/0.02 where the geometry says
0/0, the IMU at 2 deg roll and 3 deg pitch (the platform's mounting is 4.5 deg pitch) on ground that rises 3 deg.

Checks that a level-ish mower with the IMU of the NX60i passes the start (the slope counts from the stored mounting),
a drive through its first line and turn, the head of the recording with the basis of the xcore_driver and the
mounting of the platform, calibration.abort, and the blade never on.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_calibration_xcore.py
"""
import functools
import gzip
import json
import math
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from mowbite_msgs.msg import (HardwareStatus, MapStatus, MqttPublish, RobotPose, Rtcm, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import QoSProfile
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, fake_xcore, layers, loopback_group, read_json, sim_files,  # noqa: E402
                        station_frame, xcore_driver)

WORK = tempfile.mkdtemp(prefix="mowbite_calibration_xcore_")
STATE, FLAGS, _, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "extra.yaml")
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
LAWN = [(-3.0, -3.0), (8.0, -3.0), (8.0, 3.0), (-3.0, 3.0)]

with open(CONFIG, "w") as f:
    f.write("mowbite_logic:\n  ros__parameters:\n    geometry:\n      antenna_x: 0.0\n      antenna_y: 0.0\n")
with open(FLAGS, "w") as f:
    json.dump({"gps_fix": "fix"}, f)


def generate_test_description():
    config = layers.files("tron")
    geometry = layers.geometry("nx60i")
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--ticks-scale", "1.03", "--track", "0.37", "--antenna-offset", "-0.10", "0.02",
                       "--imu-error-rpy-deg", "2.0", "3.0", "0.0", "--slope", "3.0", "40.0", "--gnss-noise-m", "0.01",
                       "--gnss-delay", "0.1")
    driver = xcore_driver(BIND_IP, GROUP_PORT, node={"name": "xcore_driver", "namespace": "/"},
                          datum_lat=DATUM[0], datum_long=DATUM[1], datum_height=DATUM[2],
                          safety_measurement_date="simulation")

    def component(package, plugin, name, extra=None):
        return ComposableNode(package=package, plugin=plugin, name=name,
                              parameters=[geometry, *config, CONFIG, *([extra] if extra else [])])

    container = ComposableNodeContainer(
        name="mowbite", namespace="", package="rclcpp_components", executable="component_container_isolated",
        output="screen",
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
        board,
        TimerAction(period=0.5, actions=[driver, container, launch_testing.actions.ReadyToTest()]),
    ]), {}


board = functools.partial(read_json, STATE)


class Probe(SimProbe):
    service = SimProbe.call
    rpc = SimProbe.rpc_outcome

    def __init__(self):
        super().__init__("calibration_xcore_probe")
        self.map_status = None
        self.pose = None
        self.node.create_subscription(MapStatus, "map/status", self._set("map_status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._result, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._failure, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        # Corrections once a second, as a caster would send them: RTK fixed counts only with them (YF-c).
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=10))
        self.node.create_timer(1.0, lambda: self.rtcm_pub.publish(Rtcm(data=list(station_frame()))))
        self.start()


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
        self.assertEqual(head["basis"], {"ticks_per_m": 1815.1, "wheel_distance_m": 0.3014})
        self.assertEqual(head["params"]["ticks_per_m"], 1815.1)
        self.assertEqual(head["imu"], {"roll": 0.0, "pitch": 0.079})
        self.assertEqual((lines[-1]["k"], lines[-1]["why"]), ("end", "stop"))
        marks = [(m["name"], m["edge"]) for m in lines if m.get("k") == "phase"]
        self.assertIn(("line", "end"), marks)
        self.assertIn(("spin_left", "start"), marks)
        self.assertEqual(p.rpc("calibration.result")[1]["status"], "failed")
