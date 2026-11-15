"""Driving between two lawns over a path in the simulation: the whole stack of sim.launch.py (fake_tron, driver,
localization, map, logic, follower, planner). logic/drive_to plans the way with ~/plan_transit and the follower drives
it; the test checks where the simulated mower really went (the state file of fake_tron).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_drive.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import MapStatus, MqttPublish, RobotPose
from mowbite_msgs.srv import DriveTo, ReplaceMap, SetPose
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, distance_to, follower_max_linear, inside, sim_description,  # noqa: E402
                        sim_files)

WORK = tempfile.mkdtemp(prefix="mowbite_drive_")
STATE, FLAGS, _, _ = sim_files(WORK)

# Two lawns with a path between them; the mower starts at (0, 0) facing +x, on the first lawn.
FRONT = [(-2, -2), (6, -2), (6, 5), (-2, 5)]
BACK = [(10, -2), (18, -2), (18, 5), (10, 5)]
PATH = [(5.5, 0.5), (10.5, 0.5), (10.5, 3.5), (5.5, 3.5)]
BED = [(14, 3), (16, 3), (16, 4.5), (14, 4.5)]
GARDEN = {
    "areas": [
        {"id": "front", "properties": {"type": "mow"}, "outline": [{"x": x, "y": y} for x, y in FRONT]},
        {"id": "back", "properties": {"type": "mow"}, "outline": [{"x": x, "y": y} for x, y in BACK]},
        {"id": "path", "properties": {"type": "nav"}, "outline": [{"x": x, "y": y} for x, y in PATH]},
        {"id": "bed", "properties": {"type": "obstacle"}, "outline": [{"x": x, "y": y} for x, y in BED]},
    ],
    "docking_stations": [],
}


def generate_test_description():
    return sim_description(WORK, 18831)


def outside_by(x, y):
    """m the point lies outside of the lawns and the path, 0 inside."""
    if any(inside(r, x, y) for r in (FRONT, BACK, PATH)):
        return 0.0
    return min(distance_to(r, x, y) for r in (FRONT, BACK, PATH))


class Probe(SimProbe):
    def __init__(self):
        super().__init__("drive_probe", state=STATE, flags=FLAGS)
        self.status = None
        self.pose = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.stop_drive = self.node.create_client(Trigger, "logic/stop_drive")
        self.start()
        self.spawn(self._record)

    def drive(self, x, y, heading=0.0, use_heading=False):
        request = DriveTo.Request()
        request.x, request.y, request.heading, request.use_heading = float(x), float(y), float(heading), use_heading
        return self.call(self.drive_to, request)

    def drive_ended(self, since, timeout):
        return self.event(since, "DRIVE", timeout, phase="end")


class TestDrive(unittest.TestCase):
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

    def test_01_garden_and_an_aligned_heading(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        # fake_tron starts at (0, 0) facing +x: tell the localization, so the heading counts as aligned.
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def test_02_drive_from_the_front_lawn_to_the_back_over_the_path(self):
        p = self.probe
        since = len(p.events)
        track_from = len(p.track)
        answer = p.drive(15, 1.0, 0.0, True)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(answer.message.startswith("driving "), answer.message)
        ended = p.drive_ended(since, 200)
        self.assertIsNotNone(ended, "the drive never ended")
        self.assertTrue(ended["arrived"], ended.get("reason"))
        self.assertIn(ended["reason"], ("DONE", "ALREADY_THERE"))
        truth = p.truth()
        self.assertLess(math.hypot(truth["x"] - 15, truth["y"] - 1.0), 0.25, truth)
        self.assertLess(abs(math.remainder(truth["yaw"], 2 * math.pi)), math.radians(25), truth)
        with p.lock:
            track = p.track[track_from:]
        # base_link stayed on the lawns and the path, never in the bed.
        worst = max(outside_by(t["x"], t["y"]) for t in track)
        self.assertLess(worst, 0.1, f"{worst:.2f} m outside")
        self.assertFalse(any(inside(BED, t["x"], t["y"]) for t in track))
        self.assertTrue(any(inside(PATH, t["x"], t["y"]) and 6.5 < t["x"] < 9.5 for t in track), "not over the path")
        self.assertLessEqual(max(abs(t["linear"]) for t in track), follower_max_linear() + 1e-3)
        types = [e["type"] for e in p.events[since:]]
        self.assertIn("STATE", types)

    def test_03_a_goal_off_the_map_is_refused(self):
        since = len(self.probe.events)
        answer = self.probe.drive(30, 30)
        self.assertFalse(answer.success)
        self.assertIn("Transit planning failed", answer.message)
        # Its end event has to be in before the next drive, which looks for the first end after its start.
        ended = self.probe.drive_ended(since, 5)
        self.assertIsNotNone(ended, "no DRIVE end for the refused goal")
        self.assertEqual(ended["reason"], "TRANSIT_FAILED")

    def test_04_stop_drive_stops_the_mower(self):
        p = self.probe
        since = len(p.events)
        answer = p.drive(0, 1.0)
        self.assertTrue(answer.success, answer.message)
        time.sleep(8.0)
        stop = p.call(p.stop_drive, Trigger.Request())
        self.assertTrue(stop.success, stop.message)
        ended = p.drive_ended(since, 10)
        self.assertIsNotNone(ended)
        self.assertFalse(ended["arrived"])
        self.assertEqual(ended["reason"], "STOPPED")
        self.assertIn("logic/stop_drive", ended["message"])
        self.assertEqual(ended["severity"], "info")
        time.sleep(1.5)
        a = p.truth()
        time.sleep(1.0)
        b = p.truth()
        self.assertLess(math.hypot(a["x"] - b["x"], a["y"] - b["y"]), 0.02, (a, b))
        self.assertEqual(p.call(p.stop_drive, Trigger.Request()).message, "not driving")

    def test_05_stop_button_ends_the_drive(self):
        p = self.probe
        since = len(p.events)
        answer = p.drive(0, 1.0)
        self.assertTrue(answer.success, answer.message)
        time.sleep(4.0)
        p.flags(stop=1)
        ended = p.drive_ended(since, 10)
        self.assertIsNotNone(ended)
        self.assertFalse(ended["arrived"])
        time.sleep(1.0)
        a = p.truth()
        time.sleep(1.0)
        b = p.truth()
        self.assertLess(math.hypot(a["x"] - b["x"], a["y"] - b["y"]), 0.02, (a, b))
        p.flags()
        # Still in emergency (latched): no new drive.
        answer = p.drive(0, 1.0)
        self.assertFalse(answer.success)
        self.assertEqual(answer.message, "Emergency: reset it first")
