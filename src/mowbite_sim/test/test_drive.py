"""Driving between two lawns over a path in the simulation: the whole stack of sim.launch.py (fake_tron, driver,
localization, map, logic, follower, planner). logic/drive_to plans the way with ~/plan_transit and the follower drives
it; the test checks where the simulated mower really went (the state file of fake_tron).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_drive.py
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, MapStatus, MqttPublish
from mowbite_msgs.srv import DriveTo, ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

WORK = tempfile.mkdtemp(prefix="mowbite_drive_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

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
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS,
                          "log": os.path.join(WORK, "rx.jsonl"), "data_dir": os.path.join(WORK, "data"),
                          "mqtt_port": "18831"}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def inside(ring, x, y):
    result = False
    for i in range(len(ring)):
        (ax, ay), (bx, by) = ring[i], ring[i - 1]
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            result = not result
    return result


def distance_to(ring, x, y):
    """m from (x, y) to the closest edge of the ring."""
    def to_edge(p, q):
        ex, ey = q[0] - p[0], q[1] - p[1]
        along = min(1.0, max(0.0, ((x - p[0]) * ex + (y - p[1]) * ey) / (ex * ex + ey * ey)))
        return math.dist((x, y), (p[0] + along * ex, p[1] + along * ey))
    return min(to_edge(ring[i - 1], ring[i]) for i in range(len(ring)))


def outside_by(x, y):
    """m the point lies outside of the lawns and the path, 0 inside."""
    if any(inside(r, x, y) for r in (FRONT, BACK, PATH)):
        return 0.0
    return min(distance_to(r, x, y) for r in (FRONT, BACK, PATH))


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.events = []
        self.track = []
        self.node = rclpy.create_node("drive_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.stop_drive = self.node.create_client(Trigger, "logic/stop_drive")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()
        threading.Thread(target=self._record, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _record(self):
        # The true pose of the simulated mower, five times a second.
        while self.running:
            truth = self.truth()
            if truth:
                with self.lock:
                    self.track.append(truth)
            time.sleep(0.2)

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

    def drive(self, x, y, heading=0.0, use_heading=False):
        request = DriveTo.Request()
        request.x, request.y, request.heading, request.use_heading = float(x), float(y), float(heading), use_heading
        return self.call(self.drive_to, request)

    def drive_ended(self, since, timeout):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == "DRIVE"
                                       and e.get("phase") == "end"), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


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
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.orientation_valid
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
        self.assertLessEqual(max(abs(t["linear"]) for t in track), 0.3 + 1e-3)
        types = [e["type"] for e in p.events[since:]]
        self.assertIn("STATE", types)

    def test_03_a_goal_off_the_map_is_refused(self):
        answer = self.probe.drive(30, 30)
        self.assertFalse(answer.success)
        self.assertIn("transit planning failed", answer.message)

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
        self.assertIn("stopped by logic/stop_drive", ended["reason"])
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
