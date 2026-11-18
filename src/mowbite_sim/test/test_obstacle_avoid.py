"""Driving around objects in the whole stack of sim.launch.py (FP30 avoid): fake_obstacles reports the things of a
scene the test moves, mowbite_obstacles tracks them, the logic holds the mower, has the planner cut the plan by a circle
around a ball that stands and drives around it within the lawn with the blade on. The simulated Tron reports the camera
stereo and obstacle_mode_max 3, obstacle_mode is avoid. Checked on the true pose of fake_tron: the body keeps the
clearance to the ball all along, the blade runs on the way around, the list entry has the circle and OBSTACLE_AVOIDED
tells of it; around a ball amid the lanes the blade leaves little unmowed outside the circle; a person in the way is
waited for and never driven around.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_obstacle_avoid.py
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
from mowbite_msgs.msg import MapStatus, MqttPublish, ObstacleStatus, RobotPose, RobotState, RpcError, RpcRequest
from mowbite_msgs.msg import RpcResponse
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, sim_description, sim_files, sim_program, stop_process,  # noqa: E402
                        tron_geometry, wrap, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_obstacle_avoid_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "obstacle_avoid.yaml")
SCENE = os.path.join(WORK, "scene.json")

LAWN_MIN = (-0.41, -0.11)
LAWN_MAX = (6.59, 6.89)
LAWN = [LAWN_MIN, (LAWN_MAX[0], LAWN_MIN[1]), LAWN_MAX, (LAWN_MIN[0], LAWN_MAX[1])]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}
# The body around base_link (m).
FRONT = float(tron_geometry("robot_front"))
REAR = float(tron_geometry("robot_rear"))
HALF_WIDTH = float(tron_geometry("robot_width")) / 2
# Where the blade cuts: its centre ahead of base_link and its width (m).
BLADE_AHEAD = float(tron_geometry("blade_ahead"))
BLADE_RADIUS = float(tron_geometry("mower_width")) / 2
BALL_RADIUS = 0.1
BALL_HEIGHT = 0.2
BALL_GAP = 1.6
# Between the body and the ball the clearance of the logic on the way around and on a lane passing it straight, less
# what the follower may stray (m).
CLEARANCE = 0.30
PASS_CLEARANCE = 0.10
STRAY = 0.06
# Around a ball amid the lanes at most this much of the lawn outside its circle stays unmowed (m²): the stretch of each
# lane the body cannot drive while it would touch the circle, less what the way around cuts.
UNMOWED_MAX = 1.2
# Cells of the unmowed count (m) and how far around the ball it counts.
CELL = 0.05
AROUND = 1.5

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "    obstacle_sources: [\"stereo\"]\n    obstacle_mode_max: 3\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    write_json(SCENE, {"obstacles": []})
    return sim_description(WORK, 18837, extra_config=CONFIG, settings={"obstacle_mode": 3})


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
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    return t["x"] + ahead * c - left * s, t["y"] + ahead * s + left * c


def clearance(t, x, y, radius):
    """m between the body at the true pose t and the edge of a thing at x, y (negative: they overlap)."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    dx, dy = x - t["x"], y - t["y"]
    along, side = dx * c + dy * s, -dx * s + dy * c
    out_along = max(-REAR - along, 0.0, along - FRONT)
    out_side = max(abs(side) - HALF_WIDTH, 0.0)
    return math.hypot(out_along, out_side) - radius


def inner(x, y):
    """m from x, y to the nearest edge of the lawn."""
    return min(x - LAWN_MIN[0], LAWN_MAX[0] - x, y - LAWN_MIN[1], LAWN_MAX[1] - y)


def unmowed(track, x, y, keep_out, picture=None):
    """m² within AROUND of x, y, outside the circle keep_out around it, that the blade never went over on the track;
    picture gets a line of text per row (north up): # unmowed, . mowed, o the circle, blank outside."""
    n = int(round(2 * AROUND / CELL))
    cut = [[False] * n for _ in range(n)]
    for a, b in zip(track, track[1:]):
        if not (a.get("cutter") and b.get("cutter")):
            continue
        ax, ay = a["x"] + BLADE_AHEAD * math.cos(a["yaw"]), a["y"] + BLADE_AHEAD * math.sin(a["yaw"])
        bx, by = b["x"] + BLADE_AHEAD * math.cos(b["yaw"]), b["y"] + BLADE_AHEAD * math.sin(b["yaw"])
        lo_i = max(0, int((min(ax, bx) - BLADE_RADIUS - x + AROUND) / CELL))
        hi_i = min(n - 1, int((max(ax, bx) + BLADE_RADIUS - x + AROUND) / CELL))
        lo_j = max(0, int((min(ay, by) - BLADE_RADIUS - y + AROUND) / CELL))
        hi_j = min(n - 1, int((max(ay, by) + BLADE_RADIUS - y + AROUND) / CELL))
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        for i in range(lo_i, hi_i + 1):
            cx = x - AROUND + (i + 0.5) * CELL
            for j in range(lo_j, hi_j + 1):
                cy = y - AROUND + (j + 0.5) * CELL
                t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / length2))
                if math.hypot(ax + t * dx - cx, ay + t * dy - cy) <= BLADE_RADIUS:
                    cut[i][j] = True
    left = 0
    rows = [[" "] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            cx, cy = x - AROUND + (i + 0.5) * CELL, y - AROUND + (j + 0.5) * CELL
            r = math.hypot(cx - x, cy - y)
            if r <= keep_out:
                rows[n - 1 - j][i] = "o"
            elif r <= AROUND:
                rows[n - 1 - j][i] = "." if cut[i][j] else "#"
                left += 0 if cut[i][j] else 1
    if picture is not None:
        picture.extend("".join(row[::2]) for row in rows[::2])
    return left * CELL * CELL


def around(track, x, y, yaw):
    """The poses of track until base_link got 1 m past x, y along yaw: the way around a ball ahead."""
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for t in track:
        out.append(t)
        if (t["x"] - x) * c + (t["y"] - y) * s > 1.0:
            break
    return out


def place(*things):
    write_json(SCENE, {"obstacles": list(things)})


def thing(kind, name, x, y, radius, height):
    return {"kind": kind, "class_name": name, "score": 0.9, "x": x, "y": y, "radius": radius, "height": height}


def ball(x, y):
    return thing("OBJECT", "ball", x, y, BALL_RADIUS, BALL_HEIGHT)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("obstacle_avoid_probe", state=STATE, flags=FLAGS, log=RX)
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
        with self.lock:
            track = list(self.track[-int(span * 5) - 1:])
        if len(track) < 2:
            return 0.0
        return math.hypot(track[-1]["x"] - track[0]["x"], track[-1]["y"] - track[0]["y"]) / (0.2 * (len(track) - 1))

    def straight_run(self, room, timeout=120.0, ahead_inner=0.0):
        """The true pose once the mower mows straight ahead with that much lawn before it, and the point room/2
        ahead at least ahead_inner from every edge; None if it never does."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                track = list(self.track[-6:])
            turned = abs(wrap(track[-1]["yaw"] - track[0]["yaw"])) if len(track) == 6 else 1
            if (turned < 0.03 and self.robot is not None and self.robot.current_state == "MOWING"
                    and self.speed() > 0.25 and room_ahead(track[-1]) >= room
                    and inner(*point(track[-1], room / 2)) >= ahead_inner):
                return track[-1]
            time.sleep(0.2)
        return None


class TestObstacleAvoid(unittest.TestCase):
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

    def test_02_a_ball_that_stays_is_driven_around_with_the_clearance_and_the_blade_on(self):
        p = self.probe
        start = p.straight_run(5.0)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + BALL_RADIUS + BALL_GAP)
        since = len(p.events)
        place(ball(x, y))
        held = time.time()
        self.paused_for(since, "OBJECT")
        avoided = p.event(since, "OBSTACLE_AVOIDED", 30)
        self.assertIsNotNone(avoided, [e["type"] for e in p.events[since:]])
        self.assertGreater(time.time() - held, 3.0)
        self.assertGreaterEqual(avoided["keep_out_r"], BALL_RADIUS + CLEARANCE)
        self.assertGreaterEqual(avoided["left_out"], 0.0)
        self.assertEqual(avoided["area_id"], "lawn")
        on = p.event(since, "STATE", 10, state="MOWING")
        self.assertIsNotNone(on, "it did not go on")
        index = len(p.track)
        stood = p.truth()
        # The blade runs before the mower got far on its way around.
        self.assertIsNotNone(p.event(since, "BLADES", 30, enabled=True), "the blade stayed off")
        self.assertLess(math.hypot(p.truth()["x"] - stood["x"], p.truth()["y"] - stood["y"]), 0.6)
        # The way around and the next lanes keep the body out of the clearance while the ball lies there.
        time.sleep(60.0)
        self.assertEqual(p.robot.current_state, "MOWING")
        with p.lock:
            track = list(p.track[index:])
        way = min(clearance(t, x, y, BALL_RADIUS) for t in around(track, x, y, start["yaw"]))
        nearest = min(clearance(t, x, y, BALL_RADIUS) for t in track)
        self.assertGreater(way, CLEARANCE - STRAY, f"the body came {way:.2f} m near the ball on the way around")
        self.assertGreater(nearest, PASS_CLEARANCE - STRAY, f"the body came {nearest:.2f} m near the ball")
        moved = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(track, track[1:]))
        self.assertGreater(moved, 5.0, "it hardly moved after going around")
        entries = p.rpc("vision.obstacles.list", {"client": "test"})["result"]["obstacles"]
        entry = next(e for e in entries if e["id"] == avoided["obstacle_id"])
        self.assertEqual(entry["action"], "avoided", entry)
        self.assertEqual(entry["keep_out_r"], avoided["keep_out_r"], entry)
        print(f"ball: around {way:.2f} m, nearest {nearest:.2f} m, {avoided['left_out']:.1f} m left out")
        place()

    def mow_anew(self):
        """Ends mowing and starts it again: the circles go with it, the job goes on."""
        p = self.probe
        p.act("mower_logic:mowing/abort_mowing")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 30), "mowing did not end")
        p.act("mower_logic:idle/start_mowing")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "MOWING", 60), "mowing did not start")

    def test_03_around_a_ball_amid_the_lanes_little_stays_unmowed(self):
        p = self.probe
        self.mow_anew()
        start = p.straight_run(5.0, timeout=600.0, ahead_inner=2.2)
        self.assertIsNotNone(start, "no straight run amid the lanes")
        x, y = point(start, FRONT + BALL_RADIUS + BALL_GAP)
        index = len(p.track)
        since = len(p.events)
        place(ball(x, y))
        avoided = p.event(since, "OBSTACLE_AVOIDED", 30)
        self.assertIsNotNone(avoided, [e["type"] for e in p.events[since:]])
        # The planner does not mow the lanes in their order: the patch is judged once the job is done.
        done = p.event(since, "JOB_COMPLETE", 1500)
        place()
        self.assertIsNotNone(done, "the job did not complete")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 30), "mowing did not end")
        with p.lock:
            whole = list(p.track)
        # The lanes before the ball count as mowed too.
        picture = []
        left = unmowed(whole, x, y, avoided["keep_out_r"], picture)
        cutting = sum(1 for t in whole if t.get("cutter"))
        print(f"blade on in {cutting} of {len(whole)} poses, lanes along {math.degrees(start['yaw']):.0f} deg")
        print("\n".join(picture))
        way = min(clearance(t, x, y, BALL_RADIUS) for t in around(whole[index:], x, y, start["yaw"]))
        nearest = min(clearance(t, x, y, BALL_RADIUS) for t in whole[index:])
        print(f"amid the lanes: {left:.2f} m² unmowed outside the circle of {avoided['keep_out_r']:.2f} m, "
              f"{avoided['left_out']:.1f} m of lanes left out, around {way:.2f} m, nearest {nearest:.2f} m")
        self.assertGreater(way, CLEARANCE - STRAY, f"the body came {way:.2f} m near the ball on the way around")
        self.assertGreater(nearest, PASS_CLEARANCE - STRAY, f"the body came {nearest:.2f} m near the ball")
        self.assertLessEqual(left, UNMOWED_MAX)

    def test_04_a_person_in_the_way_is_waited_for(self):
        p = self.probe
        # No circle left from the balls: the mower mows lanes, no way around.
        if p.robot.current_state != "MOWING":
            p.act("mower_logic:idle/start_mowing")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "MOWING", 120), "no mowing")
        entries = p.rpc("vision.obstacles.list", {"client": "test"})["result"]["obstacles"]
        self.assertEqual([e for e in entries if e["keep_out_r"] is not None], [], "a circle stays")
        start = p.straight_run(5.0)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + 0.25 + 2.0)
        since = len(p.events)
        place(thing("PERSON", "person", x, y, 0.25, 1.7))
        self.paused_for(since, "PERSON", 20)
        time.sleep(15.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "OBSTACLE_AVOIDED", 0.1))
        self.assertIsNone(p.event(since, "STATE", 0.1, state="MOWING"))
        place()
        self.assertIsNotNone(p.event(since, "STATE", 20, state="MOWING"), "it did not go on")
