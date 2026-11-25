"""An area the planner cannot plan as it is, in the simulation: a strip narrower than the mower comes first in the map,
the lawn of test_mowing after it. start_mowing skips the strip with AREA_SKIPPED (the code of the planner as its
reason, severity warning) and mows the lawn, without a pause (masterplan 3.4).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_planner_skip.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

import rclpy
from mowbite_msgs.msg import MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_planner_skip_")
STATE, FLAGS, RX, DATA = sim_files(WORK)

# A small lawn at the mower (0, 0); the strip 10 cm wide beside it.
LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
STRIP = [(-0.41, 2.5), (2.59, 2.5), (2.59, 2.6), (-0.41, 2.6)]
GARDEN = {
    "areas": [{"id": "strip", "properties": {"type": "mow", "name": "Strip", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in STRIP]},
              {"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}
SKIP_CODES = {"BODY_OUT", "NO_PLAN", "START_UNREACHABLE"}


def generate_test_description():
    return sim_description(WORK, 18833)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("planner_skip_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.start()


class TestPlannerSkip(unittest.TestCase):
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

    def test_01_the_strip_is_skipped_and_the_lawn_mowed(self):
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
        since, _ = p.started(lambda: p.act("mower_logic:idle/start_mowing"))
        skipped = p.event(since, "AREA_SKIPPED", 60, area_id="strip")
        self.assertIsNotNone(skipped, [e["type"] for e in p.events[since:]])
        self.assertIn(skipped["reason"], SKIP_CODES)
        self.assertEqual(skipped["severity"], "warning")
        area = p.event(since, "AREA", 60, area_id="lawn")
        self.assertIsNotNone(area, [e["type"] for e in p.events[since:]])
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "MOWING", 30))
        paused = [e for e in p.events[since:] if e["type"] == "STATE" and e.get("state") == "PAUSED"]
        self.assertEqual(paused, [])


if __name__ == "__main__":
    unittest.main()
