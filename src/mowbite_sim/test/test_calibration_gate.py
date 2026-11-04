"""The calibration drive without a safety measurement of the platform: the stack of sim.launch.py with a Tron that has
none. calibration.check names NO_SAFETY_MEASUREMENT, calibration.start is refused with it, and nothing moves.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_calibration_gate.py
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
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import MqttPublish, RpcError, RpcRequest, RpcResponse

WORK = tempfile.mkdtemp(prefix="mowbite_calibration_gate_")
CONFIG = os.path.join(WORK, "gate.yaml")

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"\"\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": os.path.join(WORK, "state.json"),
                          "flags": os.path.join(WORK, "flags.json"), "log": os.path.join(WORK, "rx.jsonl"),
                          "data_dir": os.path.join(WORK, "data"), "mqtt_port": "18839",
                          "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class TestCalibrationGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("calibration_gate_probe")
        cls.lock = threading.Lock()
        cls.responses, cls.errors, cls.events, cls.moves = {}, {}, [], []
        cls.node.create_subscription(RpcResponse, "bridge/rpc/response",
                                     lambda m: cls.responses.__setitem__(m.id, json.loads(m.result)), 10)
        cls.node.create_subscription(RpcError, "bridge/rpc/error",
                                     lambda m: cls.errors.__setitem__(m.id, (m.code, json.loads(m.data or "null"))),
                                     10)
        cls.node.create_subscription(MqttPublish, "bridge/publish", lambda m: cls.events.append(json.loads(m.payload))
                                     if m.topic == "events/json" else None, 100)
        cls.node.create_subscription(Twist, "hw/cmd_vel",
                                     lambda m: cls.moves.append(m) if m.linear.x or m.angular.z else None, 10)
        cls.pub = cls.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        cls.running = True
        threading.Thread(target=cls._spin, daemon=True).start()

    @classmethod
    def _spin(cls):
        while cls.running:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        time.sleep(0.3)
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def rpc(self, rid, method):
        end = time.time() + 20
        while self.pub.get_subscription_count() == 0 and time.time() < end:
            time.sleep(0.05)
        self.pub.publish(RpcRequest(method=method, params="", id=rid))
        end = time.time() + 5
        while time.time() < end:
            if rid in self.responses:
                return "ok", self.responses[rid]
            if rid in self.errors:
                return "error", self.errors[rid]
            time.sleep(0.05)
        return None

    def test_refused_without_a_safety_measurement(self):
        answer = None
        end = time.time() + 60
        # Once the program reports, the gate is the first reason.
        while time.time() < end:
            answer = self.rpc(f"check_{time.time()}", "calibration.check")
            if answer and answer[0] == "ok" and answer[1]["problems"] and \
                    answer[1]["problems"][0]["reason"] == "NO_SAFETY_MEASUREMENT":
                break
            time.sleep(1.0)
        self.assertEqual(answer[1]["problems"][0]["reason"], "NO_SAFETY_MEASUREMENT", answer)
        since = len(self.events)
        answer = self.rpc("start", "calibration.start")
        self.assertEqual(answer[0], "error", answer)
        self.assertEqual(answer[1], (RpcError.ERROR_INVALID_REQUEST, answer[1][1]))
        self.assertEqual(answer[1][1]["reason"], "NO_SAFETY_MEASUREMENT")
        end = time.time() + 5
        while time.time() < end and not any(e["type"] == "CALIBRATION_REFUSED" for e in self.events[since:]):
            time.sleep(0.05)
        refused = [e for e in self.events[since:] if e["type"] == "CALIBRATION_REFUSED"]
        self.assertTrue(refused)
        self.assertEqual(refused[0]["reason"], "NO_SAFETY_MEASUREMENT")
        time.sleep(2.0)
        self.assertEqual(self.moves, [])
