"""params.set of mowbite_logic: the test plays the bridge (bridge/rpc/request) and the driver, and reads what the logic
answers, reports on bridge/params and saves, and what reaches the localization and the follower. The logic starts
with a file saved before.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_params.py
"""
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
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from mowbite_msgs.msg import ActionList, GpsLimits, HardwareStatus, RobotPose, RpcError, RpcMethods, RpcRequest
from mowbite_msgs.msg import RpcResponse
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

DATUM = (52.0, 9.0, 50.0)
WORK = tempfile.mkdtemp(prefix="mowbite_params_")
SAVED = os.path.join(WORK, "params", "mower_logic.json")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROSOUT = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
P = "/mower_logic/"


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "geometry", "tron.yaml")


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


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.responses = {}
        self.errors = {}
        self.params = {}
        self.methods = {}
        self.limits = None
        self.actions = {}
        self.logs = []
        self.idle = []
        self.node = rclpy.create_node("params_probe")
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(String, "bridge/params", self._params, HISTORY)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(GpsLimits, "logic/gps_limits", self._limits, LATCHED)
        self.node.create_subscription(ActionList, "bridge/actions", self._actions, HISTORY)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(Bool, "logic/idle", self._idle, LATCHED)
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()
        threading.Thread(target=self._keepalive, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _keepalive(self):
        while self.running:
            hw = HardwareStatus()
            hw.telemetry_ok = True
            hw.battery_source = HardwareStatus.BATTERY_SOURCE_BMS
            hw.battery_voltage = 24.5
            hw.battery_percentage = 80
            self.hw_pub.publish(hw)
            pose = RobotPose()
            pose.position_accuracy = 0.02
            pose.heading_accuracy = 0.05
            pose.heading_valid = True
            self.pose_pub.publish(pose)
            time.sleep(0.2)

    def _response(self, msg):
        with self.lock:
            self.responses[msg.id] = json.loads(msg.result)

    def _error(self, msg):
        with self.lock:
            self.errors[msg.id] = (msg.code, msg.message)

    def _params(self, msg):
        with self.lock:
            data = json.loads(msg.data)
            for name, value in data["params"].items():
                self.params[data["node"] + "/" + name] = value

    def _methods(self, msg):
        with self.lock:
            self.methods[msg.node_id] = list(msg.methods)

    def _limits(self, msg):
        with self.lock:
            self.limits = (msg.max_position_accuracy, msg.gps_timeout)

    def _actions(self, msg):
        with self.lock:
            for a in msg.actions:
                self.actions[msg.node_prefix + "/" + a.id] = a.enabled

    def _log(self, msg):
        with self.lock:
            self.logs.append(msg.msg)

    def _idle(self, msg):
        with self.lock:
            self.idle.append(msg.data)

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def logged(self, text, timeout=10.0):
        return self.wait(lambda: any(text in line for line in self.logs), timeout)

    def set_params(self, rid, params, raw=None):
        """params.set as the bridge hands it on; the answer or the error, None without one."""
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_pub.publish(RpcRequest(method="params.set", params=raw if raw is not None else json.dumps(params),
                                        id=rid))
        return self.wait(lambda: self.responses.get(rid) or self.errors.get(rid), 5.0)

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


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
        logic = {k for k in self.probe.params if k.startswith(P)}
        self.assertEqual(len(logic), 13, sorted(logic))
        self.assertEqual(self.probe.params[P + "rain_mode"], 1)
        self.assertEqual(self.probe.params["/ll/board"], "Tron")
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


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
