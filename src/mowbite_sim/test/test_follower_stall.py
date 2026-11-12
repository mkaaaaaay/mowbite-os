"""A follower that stops answering in the middle of a mow path (G22): the whole stack of sim.launch.py with the blade
allowed, as in test_mowing.py. Once the blade turns, the follower node leaves the running container with its goal
open: no feedback and no result of its own any more. The logic has to end the path by itself: PAUSED with
FOLLOWER_MISSING, the blade off, the mower standing.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_follower_stall.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from composition_interfaces.srv import ListNodes, UnloadNode
from mowbite_msgs.msg import MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_stall_")
STATE, FLAGS, RX, _ = sim_files(WORK)
CONFIG = os.path.join(WORK, "stall.yaml")

# The lawn of test_mowing.py, one loop around it.
LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18833, extra_config=CONFIG)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("stall_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.list_nodes = self.node.create_client(ListNodes, "/mowbite/_container/list_nodes")
        self.unload_node = self.node.create_client(UnloadNode, "/mowbite/_container/unload_node")
        self.start()


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
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
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
