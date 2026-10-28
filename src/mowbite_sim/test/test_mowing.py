"""Mowing a small lawn in the simulation: the whole stack of sim.launch.py with the blade allowed (allow_blade of
tron_mcu, mow_with_blade of the logic). start_mowing plans the lawn, the follower drives its paths, the blade turns
only on mow segments; pause, the stop button and continue, then the job completes. The test checks where the
simulated mower really went (the state file of fake_tron) and what reached the MCU (its log).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_mowing.py
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
from mowbite_msgs.msg import AbsolutePose, MapStatus, MqttPublish, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_mowing_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "mowing.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# One small lawn, one loop around it. Placed so that the loop starts at the mower (0, 0): plan_transit of planner
# 8ff8f73 refuses the start of the loop as a goal (the body would stick out there), so no approach is needed.
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
                          "data_dir": DATA, "mqtt_port": "18832", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def inside(ring, x, y):
    result = False
    for i in range(len(ring)):
        (ax, ay), (bx, by) = ring[i], ring[i - 1]
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            result = not result
    return result


def outside_by(ring, x, y):
    """m the point lies outside of the ring, 0 inside."""
    if inside(ring, x, y):
        return 0.0

    def to_edge(p, q):
        ex, ey = q[0] - p[0], q[1] - p[1]
        along = min(1.0, max(0.0, ((x - p[0]) * ex + (y - p[1]) * ey) / (ex * ex + ey * ey)))
        return math.dist((x, y), (p[0] + along * ex, p[1] + along * ey))
    return min(to_edge(ring[i - 1], ring[i]) for i in range(len(ring)))


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.robot = None
        self.events = []
        self.track = []
        # (time, position accuracy) of every localization/pose, for the report of a failure
        self.accuracy = []
        self.node = rclpy.create_node("mowing_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
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
            self.accuracy.append((time.time(), msg.position_accuracy))

    def accuracy_report(self, since):
        """The worst accuracy since that time and when it was above 0.1 m."""
        with self.lock:
            values = [(t, a) for t, a in self.accuracy if t >= since]
        bad = [(round(t - since, 1), round(a, 3)) for t, a in values if a > 0.1]
        return f"{len(values)} poses, worst {max((a for _, a in values), default=0):.3f} m, above 0.1 m: {bad[:20]}"

    def _robot(self, msg):
        with self.lock:
            self.robot = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _record(self):
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


class TestMowing(unittest.TestCase):
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
        # Before the start no cutter frame turns the blade on.
        self.assertFalse(any(f["cutter"] > 0 for f in p.cutter_frames()))

    def test_02_start_opens_a_job_on_the_lawn(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        state = p.event(since, "STATE", 10, state="MOWING")
        self.assertIsNotNone(state, [e["type"] for e in p.events[since:]])
        self.assertEqual(len(state["job_id"]), 32)
        self.assertEqual(len(state["session_id"]), 32)
        type(self).job = state["job_id"]
        area = p.event(since, "AREA", 10)
        self.assertIsNotNone(area)
        self.assertEqual(area["area_id"], "lawn")
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "MOWING"
                               and p.robot.current_area == 0 and p.robot.current_path >= 0, 30))

    def test_03_the_blade_turns_once_a_mow_segment_is_reached(self):
        p = self.probe
        blades = p.event(0, "BLADES", 90, enabled=True)
        self.assertIsNotNone(blades, "the blade never ran")
        self.assertEqual(blades["job_id"], self.job)
        self.assertTrue(any(f["cutter"] > 0 for f in p.cutter_frames()))

    def test_04_pause_stops_wheels_and_blade_and_continue_goes_on(self):
        p = self.probe
        time.sleep(3.0)
        since = len(p.events)
        p.act("mower_logic:mowing/pause")
        paused = p.event(since, "STATE", 5, state="PAUSED")
        self.assertIsNotNone(paused)
        self.assertEqual(paused["reason"], "Paused by the app")
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False), "the blade kept running")
        time.sleep(1.5)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual(p.robot.current_state, "PAUSED")
        with open(os.path.join(DATA, "checkpoint.json")) as f:
            checkpoint = json.load(f)
        self.assertEqual(checkpoint["job_id"], self.job)
        self.assertEqual(checkpoint["area_id"], "lawn")
        since = len(p.events)
        t0 = time.time() - 10.0
        p.act("mower_logic:mowing/continue")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"))
        self.assertIsNotNone(p.event(since, "BLADES", 60, enabled=True),
                             "the blade did not come back: " + p.accuracy_report(t0))

    def test_05_stop_button_pauses_until_reset_and_continue(self):
        p = self.probe
        time.sleep(3.0)
        since = len(p.events)
        p.flags(stop=1)
        paused = p.event(since, "STATE", 5, state="PAUSED")
        self.assertIsNotNone(paused)
        self.assertTrue(paused["reason"].startswith("Emergency"), paused["reason"])
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False))
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        p.flags()
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 10), "the emergency stayed")
        # Reset, but it stays where it is until continue.
        time.sleep(2.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual(p.robot.current_state, "PAUSED")
        since = len(p.events)
        p.act("mower_logic:mowing/continue")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"))

    def test_06_the_job_completes_on_the_lawn(self):
        p = self.probe
        done = p.event(0, "JOB_COMPLETE", 600)
        self.assertIsNotNone(done, "the job did not complete")
        self.assertEqual(done["job_id"], self.job)
        idle = p.wait(lambda: next((e for e in reversed(p.events) if e["type"] == "STATE"
                                    and e["state"] == "IDLE"), None), 5)
        self.assertIsNotNone(idle)
        self.assertNotIn("job_id", idle)
        self.assertNotIn("session_id", idle)
        self.assertIsNotNone(p.event(0, "BLADES", 5, enabled=False))
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.robot.current_area == -1, 5))
        with p.lock:
            track = list(p.track)
        # Within the lawn and the margin of the follower's geofence, never faster than 0.3 m/s.
        worst = max(outside_by(LAWN, t["x"], t["y"]) for t in track)
        self.assertLess(worst, 0.5, f"{worst:.2f} m outside")
        self.assertLessEqual(max(abs(t["linear"]) for t in track), 0.3 + 1e-3)
        # The last cutter frame switches the blade off.
        frames = p.cutter_frames()
        self.assertEqual(frames[-1]["cutter"], 0)
        with open(os.path.join(DATA, "checkpoint.json")) as f:
            checkpoint = json.load(f)
        self.assertEqual(checkpoint["job_id"], "")
        self.assertEqual(checkpoint["area"], 0)
