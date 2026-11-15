"""Mowing a small lawn in the simulation: the whole stack of sim.launch.py with the blade allowed (allow_blade of
tron_mcu, mow_with_blade of the logic). start_mowing plans the lawn, the follower drives its paths, the blade turns
only on mow segments; pause, the stop button and continue, then the job completes. Pause, the reset of the emergency,
continue and skipping a path go over their RPCs as well (AS2), with client. The test checks where the
simulated mower really went (the state file of fake_tron) and what reached the MCU (its log).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_mowing.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from mowbite_msgs.msg import (DriveCommand, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, distance_to, follower_max_linear, inside, read_json,  # noqa: E402
                        sim_description, sim_files)

WORK = tempfile.mkdtemp(prefix="mowbite_mowing_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "mowing.yaml")

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
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18832, extra_config=CONFIG)


def outside_by(ring, x, y):
    """m the point lies outside of the ring, 0 inside."""
    return 0.0 if inside(ring, x, y) else distance_to(ring, x, y)


# checkpoint.json once pred holds for it, else the last one read: the logic writes it in a thread of its own.
def read_checkpoint(pred, timeout=5.0):
    end = time.time() + timeout
    while True:
        checkpoint = read_json(os.path.join(DATA, "checkpoint.json")) or {}
        if (checkpoint and pred(checkpoint)) or time.time() > end:
            return checkpoint
        time.sleep(0.05)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("mowing_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        # (time, position accuracy) of every localization/pose, for the report of a failure
        self.accuracy = []
        # When each follow_path goal first showed as succeeded, by its id, and the times of the stops of the logic.
        self.succeeded = {}
        self.stops = []
        self.node.create_subscription(
            GoalStatusArray, "follow_path/_action/status", self._goals,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(DriveCommand, "safety/drive", self._drive, 100)
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()
        self.spawn(self._record)

    def _pose(self, msg):
        with self.lock:
            self.pose = msg
            self.accuracy.append((time.time(), msg.position_accuracy))

    def _goals(self, msg):
        now = time.time()
        with self.lock:
            for goal in msg.status_list:
                if goal.status == GoalStatus.STATUS_SUCCEEDED:
                    self.succeeded.setdefault(bytes(goal.goal_info.goal_id.uuid), now)

    def _drive(self, msg):
        if msg.source == DriveCommand.SOURCE_STOP:
            with self.lock:
                self.stops.append(time.time())

    def accuracy_report(self, since):
        """The worst accuracy since that time and when it was above 0.1 m."""
        with self.lock:
            values = [(t, a) for t, a in self.accuracy if t >= since]
        bad = [(round(t - since, 1), round(a, 3)) for t, a in values if a > 0.1]
        return f"{len(values)} poses, worst {max((a for _, a in values), default=0):.3f} m, above 0.1 m: {bad[:20]}"


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
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
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
        answer = p.rpc("mowing.pause", {"client": "test"})
        self.assertEqual(answer, {"result": {"state": "PAUSED", "sub_state": ""}})
        paused = p.event(since, "STATE", 5, state="PAUSED")
        self.assertIsNotNone(paused)
        self.assertEqual(paused["client"], "test")
        self.assertEqual(paused["reason"], "MANUAL")
        self.assertEqual(paused["resume"], "user")
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False), "the blade kept running")
        time.sleep(1.5)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual(p.robot.current_state, "PAUSED")
        checkpoint = read_checkpoint(lambda c: c["job_id"] == self.job)
        self.assertEqual(checkpoint["job_id"], self.job)
        self.assertEqual(checkpoint["area_id"], "lawn")
        # Paused already: a second pause is refused, without an event.
        since = len(p.events)
        error = (p.rpc("mowing.pause", {"client": "test"}) or {}).get("error") or {}
        self.assertEqual(error.get("code"), -32600)
        self.assertEqual(error["data"]["reason"], "WRONG_STATE")
        self.assertEqual(error["data"]["state"], "PAUSED")
        self.assertEqual([e["type"] for e in p.events[since:]], [])
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
        self.assertEqual(paused["reason"], "EMERGENCY", paused)
        self.assertEqual(paused["severity"], "error")
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False))
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        # In the emergency continue is refused without a second event.
        error = (p.rpc("mowing.continue", {}) or {}).get("error") or {}
        self.assertEqual(error["data"]["reason"], "EMERGENCY")
        self.assertEqual(error["data"]["event"], "MOWING_REFUSED")
        self.assertNotIn("event_id", error["data"])
        p.flags()
        since = len(p.events)
        answer = p.rpc("emergency.reset", {"client": "test"})
        self.assertEqual((answer or {}).get("result", {}).get("released"), True, answer)
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 10), "the emergency stayed")
        released = p.event(since, "EMERGENCY", 5, emergency=False)
        self.assertIsNotNone(released)
        self.assertEqual(released.get("source"), "rpc")
        self.assertEqual(released.get("client"), "test")
        # Reset, but it stays where it is until continue.
        time.sleep(2.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual(p.robot.current_state, "PAUSED")
        since = len(p.events)
        answer = p.rpc("mowing.continue", {"client": "test"})
        self.assertEqual(answer, {"result": {"state": "MOWING", "sub_state": ""}})
        self.assertEqual(p.event(since, "STATE", 5, state="MOWING").get("client"), "test")
        # A path the app saw before is not skipped; the one mowed now is.
        path = p.robot.current_path
        error = (p.rpc("mowing.skip_path", {"path": path + 7}) or {}).get("error") or {}
        self.assertEqual(error["data"]["reason"], "CHANGED")
        path = error["data"]["path"]
        # The path may end between the answer and the next call: then the answer names the new one.
        for _ in range(3):
            answer = p.rpc("mowing.skip_path", {"path": path, "client": "test"}) or {}
            if "result" in answer:
                break
            path = answer["error"]["data"]["path"]
        self.assertEqual(answer.get("result", {}).get("path"), path, answer)

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
        self.assertLessEqual(max(abs(t["linear"]) for t in track), follower_max_linear() + 1e-3)
        # Every path that ended done is followed by a stop of the logic, whether the last zero of the follower got
        # through or not, also where the next path follows in the same state.
        with p.lock:
            ended, stops = sorted(p.succeeded.values()), list(p.stops)
        self.assertGreater(len(ended), 1, ended)
        unstopped = [round(t - ended[0], 2) for t in ended if not any(t - 0.5 <= s <= t + 1.5 for s in stops)]
        self.assertEqual(unstopped, [], f"{len(ended)} paths done, stops at {[round(s - ended[0], 2) for s in stops]}")
        # The last cutter frame switches the blade off.
        frames = p.cutter_frames()
        self.assertEqual(frames[-1]["cutter"], 0)
        checkpoint = read_checkpoint(lambda c: c["job_id"] == "")
        self.assertEqual(checkpoint["job_id"], "")
        self.assertEqual(checkpoint["area"], 0)
