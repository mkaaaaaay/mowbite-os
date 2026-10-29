"""A follower that stops answering in the middle of a mow path (G22): the whole stack of sim.launch.py with the blade
allowed, as in test_mowing.py. Once the blade turns, the follower node leaves the running container with its goal
open: no feedback and no result of its own any more. The logic has to end the path by itself: PAUSED with
FOLLOWER_MISSING, the blade off, the mower standing.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_follower_stall.py
"""
import json
import math
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from composition_interfaces.srv import ListNodes, UnloadNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, MapStatus, MqttPublish, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_stall_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "stall.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The lawn of test_mowing.py, one loop around it.
LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18833", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.robot = None
        self.events = []
        self.node = rclpy.create_node("stall_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.list_nodes = self.node.create_client(ListNodes, "/mowbite/_container/list_nodes")
        self.unload_node = self.node.create_client(UnloadNode, "/mowbite/_container/unload_node")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _robot(self, msg):
        with self.lock:
            self.robot = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def truth(self):
        try:
            with open(STATE) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def call(self, client, request, timeout=60.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def act(self, action):
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def standing(self):
        a = self.truth()
        time.sleep(1.0)
        b = self.truth()
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.02, (a, b)

    def cutter_frames(self):
        frames = []
        try:
            with open(RX) as f:
                for line in f:
                    entry = json.loads(line)
                    if "cutter" in entry:
                        frames.append(entry)
        except OSError:
            pass
        return frames

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestFollowerStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_lawn_and_an_aligned_heading(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.orientation_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def test_02_the_blade_turns_on_the_first_mow_path(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        # Well on the path, between two feedbacks of the follower.
        time.sleep(2.2)

    def test_03_without_an_answer_of_the_follower_the_logic_stops_mower_and_blade(self):
        p = self.probe
        nodes = p.call(p.list_nodes, ListNodes.Request())
        self.assertIn("/mowbite_follower", nodes.full_node_names)
        unload = UnloadNode.Request()
        unload.unique_id = nodes.unique_ids[nodes.full_node_names.index("/mowbite_follower")]
        since = len(p.events)
        t0 = time.time()
        answer = p.call(p.unload_node, unload)
        self.assertTrue(answer.success, answer.error_message)
        paused = p.event(since, "STATE", 6, state="PAUSED")
        took = time.time() - t0
        self.assertIsNotNone(paused, [e["type"] for e in p.events[since:]])
        self.assertEqual(paused["reason"], "FOLLOWER_MISSING", paused)
        self.assertEqual((paused["severity"], paused["resume"]), ("error", "user"))
        self.assertLess(took, 4.0)
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False), "the blade kept running")
        self.assertEqual(p.cutter_frames()[-1]["cutter"], 0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "PAUSED", 3))
        # Nothing starts again by itself.
        time.sleep(3.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "BLADES", 0.1, enabled=True))
