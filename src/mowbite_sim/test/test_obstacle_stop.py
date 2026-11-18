"""The hold for obstacles in the whole stack of sim.launch.py (FP30 stop): fake_obstacles reports the things of a scene
the test moves, mowbite_obstacles tracks them, the logic holds the mower and the follower stands by its gate. The
simulated Tron reports the camera stereo and obstacle_mode_max 2, obstacle_mode is stop. Checked on the true pose of
fake_tron: an object stops the mower with the blade off before it and it goes on once the place is seen free; a person
slows it, stops it farther off and lets it go on after it walked away; an animal beside the mower out of view holds it
until the user continues; after a thing right at the nose it waits 30 s; without sight nothing stops it.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_obstacle_stop.py
"""
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import MapStatus, MqttPublish, ObstacleStatus, RobotPose, RobotState, RpcError, RpcResponse
from mowbite_msgs.msg import RpcRequest
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, sim_description, sim_files, sim_program, stop_process,  # noqa: E402
                        write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_obstacle_stop_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "obstacle_stop.yaml")
SCENE = os.path.join(WORK, "scene.json")

# A square lawn of 10 m with the mower in its corner: every edge of the loop around it is long enough for a run at a
# thing ahead.
LAWN_MIN = (-0.41, -0.11)
LAWN_MAX = (9.59, 9.89)
LAWN = [LAWN_MIN, (LAWN_MAX[0], LAWN_MIN[1]), LAWN_MAX, (LAWN_MIN[0], LAWN_MAX[1])]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}
# The nose of the Tron ahead of base_link (m); speed_slow of the follower (m/s).
FRONT = 0.605
SPEED_SLOW = 0.3
# The hold: seen free this long before it goes on (s), longer after a thing right at the nose; the tracker ends a track
# once its place is seen free for 1 s.
FREE_WAIT = 10.0
NEAR_WAIT = 30.0
NEAR_AHEAD = 0.5
SEEN_FREE = 1.0

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "    obstacle_sources: [\"stereo\"]\n    obstacle_mode_max: 2\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    os.makedirs(os.path.join(DATA, "params"), exist_ok=True)
    # The settings of params.set: stop, and no sunset (sim_stack leaves this file alone with sunset).
    write_json(os.path.join(DATA, "params", "mower_logic.json"), {"dock_at_sunset": 0, "obstacle_mode": 2})
    write_json(SCENE, {"obstacles": []})
    return sim_description(WORK, 18836, extra_config=CONFIG, sunset=True)


def room_ahead(t):
    """m from base_link along the heading to the edge of the lawn."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    room = math.inf
    for lo, hi, p, d in ((LAWN_MIN[0], LAWN_MAX[0], t["x"], c), (LAWN_MIN[1], LAWN_MAX[1], t["y"], s)):
        if d > 1e-6:
            room = min(room, (hi - p) / d)
        elif d < -1e-6:
            room = min(room, (lo - p) / d)
    return room


def point(t, ahead, left=0.0):
    """The map point ahead and left of base_link."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    return t["x"] + ahead * c - left * s, t["y"] + ahead * s + left * c


def gap(t, x, y, radius):
    """m from the nose to the edge of a thing at x, y along the heading."""
    return (x - t["x"]) * math.cos(t["yaw"]) + (y - t["y"]) * math.sin(t["yaw"]) - FRONT - radius


def place(*things):
    write_json(SCENE, {"obstacles": list(things)})


def thing(kind, name, x, y, radius, height):
    return {"kind": kind, "class_name": name, "score": 0.9, "x": x, "y": y, "radius": radius, "height": height}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("obstacle_stop_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.obstacles = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(ObstacleStatus, "obstacles/status", self._set("obstacles"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()
        self.spawn(self._record)

    def speed(self, span=1.0):
        """m/s of the true pose over the last span s (five poses a second)."""
        with self.lock:
            track = list(self.track[-int(span * 5) - 1:])
        if len(track) < 2:
            return 0.0
        return math.hypot(track[-1]["x"] - track[0]["x"], track[-1]["y"] - track[0]["y"]) / (0.2 * (len(track) - 1))

    def straight_run(self, room, timeout=120.0):
        """The true pose once the mower mows straight ahead with that much lawn before it, None if it never does."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                track = list(self.track[-6:])
            turned = abs(math.remainder(track[-1]["yaw"] - track[0]["yaw"], 2 * math.pi)) if len(track) == 6 else 1
            if (turned < 0.03 and self.robot is not None and self.robot.current_state == "MOWING"
                    and self.speed() > 0.25 and room_ahead(track[-1]) >= room):
                return track[-1]
            time.sleep(0.2)
        return None

    def gaps_since(self, since_index, x, y, radius):
        """(gap, speed between the poses) along the true track since that index."""
        with self.lock:
            track = list(self.track[since_index:])
        return [(gap(b, x, y, radius), math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / 0.2)
                for a, b in zip(track, track[1:])]


class TestObstacleStop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()
        cls.fake = subprocess.Popen([sim_program("fake_obstacles"), "--ros-args", "-p", f"state:={STATE}", "-p",
                                     f"flags:={FLAGS}", "-p", f"scene:={SCENE}"])

    @classmethod
    def tearDownClass(cls):
        if cls.fake.poll() is None:
            stop_process(cls.fake, signal.SIGINT)
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def paused_for(self, since, kind, timeout=10.0):
        state = self.probe.event(since, "STATE", timeout, state="PAUSED")
        self.assertIsNotNone(state, [e["type"] for e in self.probe.events[since:]])
        self.assertEqual(state["reason"], "OBSTACLE", state)
        self.assertEqual(state["kind"], kind, state)
        self.assertEqual(state["resume"], "auto", state)
        return state

    def test_01_mowing_on_the_lawn(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30))
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
        self.assertTrue(p.wait(lambda: p.obstacles is not None and p.obstacles.sight, 20), "the camera sees nothing")
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(0, "BLADES", 120, enabled=True), "the blade never ran")

    def test_02_an_object_ahead_stops_it_until_its_place_is_seen_free(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + 0.1 + 1.6)
        since, index = len(p.events), len(p.track)
        place(thing("OBJECT", "ball", x, y, 0.1, 0.2))
        self.paused_for(since, "OBJECT")
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False), "the blade kept running")
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        left = gap(poses[1], x, y, 0.1)
        self.assertGreater(left, 0.25, f"stopped {left:.2f} m before the ball")
        self.assertLess(left, 0.8 + 0.05)
        # It stays while the ball lies there.
        time.sleep(12.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "STATE", 0.1, state="MOWING"))
        wait = NEAR_WAIT if left < NEAR_AHEAD else FREE_WAIT
        removed = time.time()
        since = len(p.events)
        place()
        on = p.event(since, "STATE", wait + SEEN_FREE + 6.0, state="MOWING")
        self.assertIsNotNone(on, "it did not go on")
        took = time.time() - removed
        self.assertGreater(took, wait, f"went on after {took:.1f} s, {left:.2f} m left")
        self.assertIsNotNone(p.event(since, "BLADES", 60, enabled=True), "the blade did not come back")
        print(f"object: {left:.2f} m left before the ball, on {took:.1f} s after it went, {len(p.track) - index} poses")

    def test_03_a_person_slows_it_stops_it_farther_off_and_lets_it_go_after_walking_away(self):
        p = self.probe
        start = p.straight_run(5.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + 0.25 + 2.9)
        since, index = len(p.events), len(p.track)
        place(thing("PERSON", "person", x, y, 0.25, 1.7))
        self.paused_for(since, "PERSON", 20)
        time.sleep(1.5)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        left = gap(poses[1], x, y, 0.25)
        self.assertGreater(left, 0.6, f"stopped {left:.2f} m before the person")
        # In the zone to go slow (2 m) at most speed_slow, after a second to brake.
        slowed = [v for g, v in p.gaps_since(index, x, y, 0.25) if left + 0.1 < g < 1.6]
        self.assertTrue(slowed, "no pose in the slow zone")
        self.assertLessEqual(max(slowed), SPEED_SLOW + 0.05, slowed)
        # The person walks away to the side, out of the corridor.
        since = len(p.events)
        gone = time.time()
        px, py = point(poses[1], FRONT + 1.5, 3.0)
        place(thing("PERSON", "person", px, py, 0.25, 1.7))
        on = p.event(since, "STATE", FREE_WAIT + SEEN_FREE + 6.0, state="MOWING")
        self.assertIsNotNone(on, "it did not go on")
        took = time.time() - gone
        self.assertGreater(took, FREE_WAIT)
        place()
        print(f"person: {left:.2f} m left, slow zone at most {max(slowed):.2f} m/s, on after {took:.1f} s")

    def test_04_an_animal_beside_it_out_of_view_holds_it_until_the_user_continues(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, 2.6, 1.05)
        since = len(p.events)
        place(thing("ANIMAL", "dog", x, y, 0.2, 0.5))
        self.paused_for(since, "ANIMAL")
        # Out of view a place is never free: it waits past every wait for a free place.
        time.sleep(NEAR_WAIT + 5.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "STATE", 0.1, state="MOWING"))
        tracks = [t for t in p.obstacles.tracks if t.zone == 2]
        self.assertTrue(tracks and not tracks[0].in_view, p.obstacles.tracks)
        since = len(p.events)
        answer = p.rpc("mowing.continue", {"client": "test"})
        self.assertEqual(answer, {"result": {"state": "MOWING", "sub_state": ""}}, answer)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"))
        place()

    def test_05_after_a_thing_right_at_the_nose_it_waits_thirty_seconds(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + 0.1 + 1.6)
        since = len(p.events)
        place(thing("OBJECT", "ball", x, y, 0.1, 0.2))
        self.paused_for(since, "OBJECT")
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        # A cat walks in right before the nose, still in view, and goes again with the ball.
        cx, cy = point(poses[1], FRONT + 0.42 + 0.15, 0.4)
        place(thing("OBJECT", "ball", x, y, 0.1, 0.2), thing("ANIMAL", "cat", cx, cy, 0.15, 0.3))
        self.assertTrue(p.wait(lambda: any(t.kind == "ANIMAL" and t.confirmed and t.ahead < NEAR_AHEAD
                                           for t in p.obstacles.tracks), 5), p.obstacles.tracks)
        since = len(p.events)
        gone = time.time()
        place()
        on = p.event(since, "STATE", NEAR_WAIT + SEEN_FREE + 6.0, state="MOWING")
        self.assertIsNotNone(on, "it did not go on")
        took = time.time() - gone
        self.assertGreater(took, NEAR_WAIT)
        print(f"near: on after {took:.1f} s")

    def test_06_without_sight_nothing_stops_it(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        since = len(p.events)
        p.flags(obstacles_dark=1)
        self.assertIsNotNone(p.event(since, "OBSTACLE_SIGHT", 6), "no word that the camera sees nothing")
        now = p.truth()
        x, y = point(now, FRONT + 0.1 + 1.6)
        place(thing("OBJECT", "ball", x, y, 0.1, 0.2))
        time.sleep(6.0)
        self.assertIsNone(p.event(since, "STATE", 0.1, state="PAUSED"))
        self.assertGreater(p.speed(), 0.1)
        place()
        p.flags()
        self.assertTrue(p.wait(lambda: p.obstacles is not None and p.obstacles.sight, 10))
