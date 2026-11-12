"""Going home on the NX60i (O11) as nx60i.launch.py starts it, against fake_xcore, with a safety measurement given in
mower_params (test_nx60i_stack.py runs without one and cannot mow). The mower docks with the contacts in front, which
the logic does not build yet: it has no way home. A start with the blade motor at motor_cold_temperature or more is
refused, the default temperature of fake_xcore lies below it; while mowing a hot blade motor pauses the run with
MOTOR_HOT until the user resumes it (question 47 iii); a low battery ends the run, also from that pause, with
BATTERY_LOW and IDLE where the mower stands, without DOCKING. rain_mode other than 0 without the rain sensor of hw/capabilities is told once in the log.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_nx60i_going_home.py
"""
import functools
import json
import math
import os
import random
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import (HardwareCapabilities, MapStatus, MqttPublish, RobotPose, RobotState, Rtcm, RpcError,
                              RpcRequest, RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, ROSOUT, SimProbe, fake_xcore, loopback_group, read_json, sim_files,  # noqa: E402
                        station_frame, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_nx60i_home_")
STATE, FLAGS, _, DATA = sim_files(WORK)
MOWER = os.path.join(WORK, "mower.yaml")
EXTRA = os.path.join(WORK, "extra.yaml")
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
START = "mower_logic:idle/start_mowing"
P = "/mower_logic/"
LAWN = [(-2.0, -2.5), (6.0, -2.5), (6.0, 2.5), (-2.0, 2.5)]
# nx60i.yaml: empty 24.5 V (the highest voltage of 20 s); motor_hot/cold_temperature 70/40 of the logic.
EMPTY, LOW_WINDOW = 24.5, 20.0
MOTOR_COLD = 40.0

with open(FLAGS, "w") as f:
    json.dump({"gps_fix": "fix"}, f)
with open(MOWER, "w") as f:
    f.write("xcore_driver:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n")
with open(EXTRA, "w") as f:
    f.write("mowbite_logic:\n  ros__parameters:\n    gps_wait_time: 2.0\n")


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE)
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("mowbite_bringup").find("mowbite_bringup"),
                                                   "launch", "nx60i.launch.py")),
        launch_arguments={"interface": "lo", "bind_ip": BIND_IP, "announcement_port": str(GROUP_PORT),
                          "data_dir": DATA, "mower_params": MOWER, "extra_config": EXTRA,
                          "datum_lat": str(DATUM[0]), "datum_long": str(DATUM[1]), "datum_height": str(DATUM[2]),
                          "mqtt_port": str(random.randint(18000, 18999))}.items())
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[stack, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    write_json(FLAGS, {"gps_fix": "fix", **flags})


board = functools.partial(read_json, STATE, 5)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("nx60i_home_probe")
        self.status = None
        self.caps = None
        self.robot = None
        self.pose = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("caps"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=10))
        self.node.create_timer(1.0, lambda: self.rtcm_pub.publish(Rtcm(data=list(station_frame()))))
        self.start()

    def logged(self, text):
        with self.lock:
            return [line for line in self.logs if text in line]


class TestNx60iGoingHome(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_ready_on_the_lawn(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 60), p.robot)
        caps = p.wait(lambda: p.caps, 10)
        self.assertEqual(caps.platform, "yardforce_nx60i")
        self.assertFalse(caps.rain_sensor)
        self.assertEqual(caps.safety_measurement_date, "simulation")
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "no datum")
        request = ReplaceMap.Request()
        request.map = json.dumps({"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                                             "outline": [{"x": x, "y": y} for x, y in LAWN]}],
                                  "docking_stations": []})
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        truth = board()
        request = SetPose.Request()
        request.pose.position.x = truth["x"]
        request.pose.position.y = truth["y"]
        request.pose.orientation.z = math.sin(truth["yaw"] / 2)
        request.pose.orientation.w = math.cos(truth["yaw"] / 2)
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03, 40),
                        "no RTK pose with a heading")

    def test_2_a_hot_blade_motor_refuses_the_start(self):
        p = self.probe
        set_flags(blade_temperature=45.0)
        time.sleep(1.5)
        since = len(p.events)
        p.act(START)
        refused = p.event(since, "MOWING_REFUSED", 10)
        self.assertIsNotNone(refused, p.events[since:])
        self.assertEqual((refused["reason"], refused["severity"]), ("MOTOR_HOT", "warning"), refused)
        self.assertAlmostEqual(refused["temperature"], 45)
        self.assertAlmostEqual(refused["limit"], MOTOR_COLD)
        # At the default temperature of fake_xcore it starts.
        set_flags()
        time.sleep(1.5)
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 30, state="MOWING"), p.events[since:])

    def test_3_while_mowing_heat_pauses_without_a_way_home(self):
        p = self.probe
        since = len(p.events)
        set_flags(blade_temperature=75.0)
        paused = p.event(since, "STATE", 10, state="PAUSED")
        self.assertIsNotNone(paused, p.events[since:])
        self.assertEqual((paused["reason"], paused["resume"]), ("MOTOR_HOT", "user"), paused)
        self.assertTrue(paused["message"].startswith("It cannot go home: the blade motor at 75"), paused)
        time.sleep(3.0)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING", types)
        self.assertEqual(types.count("STATE"), 1, p.events[since:])
        self.assertEqual(p.robot.current_state, "PAUSED")
        set_flags()

    def test_4_a_low_battery_ends_the_run_where_it_stands(self):
        p = self.probe
        since = len(p.events)
        set_flags(battery_voltage=24.0)
        low = p.event(since, "BATTERY_LOW", LOW_WINDOW + 15)
        self.assertIsNotNone(low, p.events[since:])
        self.assertAlmostEqual(low["battery_voltage"], 24.0, places=3)
        self.assertAlmostEqual(low["limit"], EMPTY, places=3)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        self.assertNotIn("DOCKING", [e["type"] for e in p.events[since:]])
        before = board()
        time.sleep(2.0)
        after = board()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.01, (before, after))
        set_flags()

    def test_5_rain_mode_without_a_rain_sensor_is_told_once(self):
        p = self.probe
        text = "has no effect: this platform has no rain sensor"
        self.assertEqual(p.logged(text), [])
        for mode in (1, 2):
            answer = p.rpc("params.set", {"params": {P + "rain_mode": mode}})
            self.assertEqual(answer, {"result": {P + "rain_mode": mode}})
        self.assertTrue(p.wait(lambda: p.logged(text), 5))
        time.sleep(1.0)
        self.assertEqual(len(p.logged(text)), 1, p.logged(text))
        self.assertIn("rain_mode 1", p.logged(text)[0])
        answer = p.rpc("params.set", {"params": {P + "rain_mode": 0}})
        self.assertEqual(answer, {"result": {P + "rain_mode": 0}})
