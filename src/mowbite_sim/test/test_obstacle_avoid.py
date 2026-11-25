"""Driving around objects in the whole stack of sim.launch.py (FP30 avoid): fake_obstacles reports the things of a
scene the test moves, mowbite_obstacles tracks them, the logic holds the mower, has the planner cut the plan by a circle
around a ball that stands and drives around it within the lawn with the blade on. The simulated Tron reports the camera
stereo and obstacle_mode_max 3, obstacle_mode is avoid. Checked on the true pose of fake_tron: the body keeps the
clearance to the ball on the way around (the pass clearance while it turns away from closer), the blade runs on the way
around, the list entry has the circle and OBSTACLE_AVOIDED
tells of it; around a ball amid the lanes the blade leaves little unmowed outside the circle; a person in the way is
waited for and never driven around.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_obstacle_avoid.py
"""
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from obstacle_scene import BALL_GAP, BALL_RADIUS, Lawn, ObstacleSim, Stage, ball, person, point  # noqa: E402
from sim_common import sim_description, tron_geometry  # noqa: E402

STAGE = Stage("obstacle_avoid", mode_max=3)
LAWN = Lawn((-0.41, -0.11), (6.59, 6.89))
# The body around base_link (m).
FRONT = float(tron_geometry("robot_front"))
REAR = float(tron_geometry("robot_rear"))
HALF_WIDTH = float(tron_geometry("robot_width")) / 2
# Where the blade cuts: its centre ahead of base_link and its width (m).
BLADE_AHEAD = float(tron_geometry("blade_ahead"))
BLADE_RADIUS = float(tron_geometry("mower_width")) / 2
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


def generate_test_description():
    return sim_description(STAGE.work, 18837, extra_config=STAGE.config, settings={"obstacle_mode": 3})


def clearance(t, x, y, radius):
    """m between the body at the true pose t and the edge of a thing at x, y (negative: they overlap)."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    dx, dy = x - t["x"], y - t["y"]
    along, side = dx * c + dy * s, -dx * s + dy * c
    out_along = max(-REAR - along, 0.0, along - FRONT)
    out_side = max(abs(side) - HALF_WIDTH, 0.0)
    return math.hypot(out_along, out_side) - radius


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
    """The poses of track until base_link got 1 m past x, y along yaw: the way around a ball ahead, without the turn
    away of a mower that stopped closer to it than CLEARANCE (that keeps PASS_CLEARANCE, G46)."""
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for t in track:
        if out or clearance(t, x, y, BALL_RADIUS) >= CLEARANCE - STRAY:
            out.append(t)
        if (t["x"] - x) * c + (t["y"] - y) * s > 1.0:
            break
    return out


class TestObstacleAvoid(ObstacleSim, unittest.TestCase):
    STAGE = STAGE
    LAWN = LAWN
    PROBE_NAME = "obstacle_avoid_probe"

    def test_01_mowing_on_the_lawn(self):
        self.mow_on_the_lawn()

    def test_02_a_ball_that_stays_is_driven_around_with_the_clearance_and_the_blade_on(self):
        p = self.probe
        start = p.straight_run(5.0)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + BALL_RADIUS + BALL_GAP)
        since = len(p.events)
        STAGE.place(ball(x, y))
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
        STAGE.place()

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
        STAGE.place(ball(x, y))
        avoided = p.event(since, "OBSTACLE_AVOIDED", 30)
        self.assertIsNotNone(avoided, [e["type"] for e in p.events[since:]])
        # The planner does not mow the lanes in their order: the patch is judged once the job is done.
        done = p.event(since, "JOB_COMPLETE", 1500)
        STAGE.place()
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
        STAGE.place(person(x, y))
        self.paused_for(since, "PERSON", 20)
        time.sleep(15.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "OBSTACLE_AVOIDED", 0.1))
        self.assertIsNone(p.event(since, "STATE", 0.1, state="MOWING"))
        STAGE.place()
        self.assertIsNotNone(p.event(since, "STATE", 20, state="MOWING"), "it did not go on")
