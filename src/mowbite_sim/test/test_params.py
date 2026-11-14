"""params.set of mowbite_logic: the test plays the bridge (bridge/rpc/request) and the driver, and reads what the logic
answers, reports on bridge/params and saves, and what reaches the localization and the follower. The logic starts
with a file saved before.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_params.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from mowbite_msgs.msg import ActionList, GpsLimits, HardwareStatus, RobotPose, RpcError, RpcMethods, RpcRequest
from mowbite_msgs.msg import RpcResponse
from rcl_interfaces.msg import Log
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, LATCHED, ROSOUT, SimProbe, TestShutdown  # noqa: E402,F401

DATUM = (52.0, 9.0, 50.0)
WORK = tempfile.mkdtemp(prefix="mowbite_params_")
SAVED = os.path.join(WORK, "params", "mower_logic.json")
P = "/mower_logic/"


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "geometry", "tron.yaml")
# The names of the settings of params.set, as the catalog of the interface gives them.
with open(os.path.join(get_package_share_directory("mowbite_api"), "api.schema.json")) as schema:
    SETTINGS = json.load(schema)["x-settings"]


def generate_test_description():
    os.makedirs(os.path.dirname(SAVED))
    with open(SAVED, "w") as f:
        json.dump({"gps_timeout": 7.0, "rain_mode": 1}, f)
    nodes = [
        Node(package="mowbite_map", executable="mowbite_map_node", output="screen", parameters=[{"data_dir": WORK}]),
        Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen",
             parameters=[GEOMETRY, {"data_dir": WORK}]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", output="screen"),
        Node(package="mowbite_follower", executable="mowbite_follower_node", output="screen"),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("params_probe")
        self.limits = None
        self.idle = []
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._result, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(String, "bridge/params", self._params, HISTORY)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(GpsLimits, "logic/gps_limits", self._limits, LATCHED)
        self.node.create_subscription(ActionList, "bridge/actions", self._actions, HISTORY)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(Bool, "logic/idle", self._idle, LATCHED)
        self.start()
        self.spawn(self._keepalive)

    def _error(self, msg):
        """bridge/rpc/error as (code, message), as the tests compare it."""
        with self.lock:
            self.errors[msg.id] = (msg.code, msg.message)

    def _limits(self, msg):
        with self.lock:
            self.limits = (msg.max_position_accuracy, msg.gps_timeout)

    def _idle(self, msg):
        with self.lock:
            self.idle.append(msg.data)

    def set_params(self, rid, params, raw=None):
        """params.set as the bridge hands it on; the answer or the error, None without one."""
        self._send("params.set", params, raw, rid)
        return self.wait(lambda: self.responses.get(rid) or self.errors.get(rid), 5.0)


def saved():
    with open(SAVED) as f:
        return json.load(f)


class TestParams(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_starts_with_the_saved_file(self):
        self.assertTrue(self.probe.wait(lambda: "params.set" in self.probe.methods.get("mowbite_logic", [])),
                        str(self.probe.methods))
        self.assertTrue(self.probe.wait(lambda: self.probe.limits == (0.1, 7.0)), str(self.probe.limits))
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(P + "gps_timeout") == 7.0), str(self.probe.params))
        logic = sorted(k[len(P):] for k in self.probe.params if k.startswith(P))
        self.assertEqual(logic, sorted(SETTINGS))
        self.assertEqual(self.probe.params[P + "rain_mode"], 1)
        self.assertEqual(self.probe.params["/ll/board"], "MowBite")
        self.assertTrue(self.probe.logged("GPS limits: accuracy 0.100 m, timeout 7.0 s"))
        self.assertTrue(self.probe.logged("Accuracy limit 0.100 m"))

    def test_2_set_takes_the_bound_saves_and_reaches_the_nodes(self):
        answer = self.probe.set_params("set", {"params": {P + "max_position_accuracy": 1.0, P + "gps_timeout": 4}})
        self.assertEqual(answer, {P + "max_position_accuracy": 0.3, P + "gps_timeout": 4.0})
        self.assertTrue(self.probe.wait(lambda: self.probe.limits == (0.3, 4.0)), str(self.probe.limits))
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(P + "max_position_accuracy") == 0.3))
        self.assertEqual(saved()["max_position_accuracy"], 0.3)
        self.assertEqual(saved()["gps_timeout"], 4.0)
        self.assertEqual(saved()["rain_mode"], 1)
        self.assertTrue(self.probe.logged("GPS limits: accuracy 0.300 m, timeout 4.0 s"))
        self.assertTrue(self.probe.logged("Accuracy limit 0.300 m"))

    def test_3_a_bad_call_changes_nothing(self):
        before = saved()
        cases = {
            "unknown": ({"params": {P + "gps_wait_time": 5, P + "tool_width": 0.2}}, None,
                        "/mower_logic/tool_width is not a setting of MowBite OS"),
            "other_node": ({"params": {"/xbot_positioning/antenna_offset_x": 0.1}}, None,
                           "No dynamic_reconfigure server found for /xbot_positioning/antenna_offset_x"),
            "cold": ({"params": {P + "motor_cold_temperature": 80}}, None,
                     "motor_cold_temperature must be below motor_hot_temperature"),
            "empty": ({}, None, 'Expected {"params": {name: value, ...}}'),
            "broken": (None, "{broken", "The parameters are not valid JSON"),
        }
        for rid, (params, raw, message) in cases.items():
            answer = self.probe.set_params(rid, params, raw)
            self.assertEqual(answer, (RpcError.ERROR_INVALID_PARAMS, message), rid)
        self.assertEqual(saved(), before)
        self.assertEqual(self.probe.params[P + "gps_wait_time"], before["gps_wait_time"])

    def test_4_only_in_idle(self):
        self.probe.datum_pub.publish(NavSatFix(latitude=DATUM[0], longitude=DATUM[1], altitude=DATUM[2]))
        start = "mower_logic:idle/start_area_recording"
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get(start)), "start_area_recording stays off")
        # logic/idle tells the other nodes with settings (the NTRIP access) the same, once per change.
        self.assertTrue(self.probe.wait(lambda: self.probe.idle == [True]), self.probe.idle)
        self.probe.act(start)
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get("mower_logic:area_recording/start_recording")))
        self.assertTrue(self.probe.wait(lambda: self.probe.idle == [True, False]), self.probe.idle)
        answer = self.probe.set_params("recording", {"params": {P + "gps_wait_time": 5}})
        self.assertEqual(answer, (RpcError.ERROR_INVALID_REQUEST,
                                  "Settings change only in IDLE, not while recording, driving or mowing"))
        self.assertNotEqual(saved()["gps_wait_time"], 5)
        self.probe.act("mower_logic:area_recording/exit_recording_mode")
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get(start)))
        self.assertTrue(self.probe.wait(lambda: self.probe.idle == [True, False, True]), self.probe.idle)
        self.assertEqual(self.probe.set_params("idle", {"params": {P + "gps_wait_time": 5}}), {P + "gps_wait_time": 5.0})
        self.assertEqual(saved()["gps_wait_time"], 5.0)
