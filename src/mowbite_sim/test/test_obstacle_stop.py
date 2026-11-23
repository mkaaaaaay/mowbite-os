"""The hold for obstacles in the whole stack of sim.launch.py (FP30 stop): fake_obstacles reports the things of a scene
the test moves, mowbite_obstacles tracks them, the logic holds the mower and the follower stands by its gate. The
simulated Tron reports the camera stereo and obstacle_mode_max 2, obstacle_mode is stop. Checked on the true pose of
fake_tron: an object stops the mower with the blade off before it and it goes on once the place is seen free; a person
slows it, stops it farther off and lets it go on after it walked away; an animal beside the mower out of view holds it
until the user continues; after a person right at the nose it waits 30 s; without sight nothing stops it.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_obstacle_stop.py
"""
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from obstacle_scene import (BALL_GAP, BALL_RADIUS, Lawn, ObstacleSim, Stage, ball, dog, person,  # noqa: E402
                            point)
from sim_common import follower_param, sim_description, tron_geometry  # noqa: E402

STAGE = Stage("obstacle_stop", mode_max=2)
# A square lawn of 10 m with the mower in its corner: every edge of the loop around it is long enough for a run at a
# thing ahead.
LAWN = Lawn((-0.41, -0.11), (9.59, 9.89))
# The front of the body ahead of base_link that the zones count from (m); speed_slow of the follower (m/s).
FRONT = float(tron_geometry("robot_front"))
SPEED_SLOW = float(follower_param("speed_slow"))
# The hold: seen free this long before it goes on (s), longer after a thing right at the nose; the tracker ends a track
# once its place is seen free for 1 s.
FREE_WAIT = 10.0
NEAR_WAIT = 30.0
NEAR_AHEAD = 0.5
SEEN_FREE = 1.0


def generate_test_description():
    return sim_description(STAGE.work, 18836, extra_config=STAGE.config, settings={"obstacle_mode": 2})


def gap(t, x, y, radius):
    """m from the nose to the edge of a thing at x, y along the heading."""
    return (x - t["x"]) * math.cos(t["yaw"]) + (y - t["y"]) * math.sin(t["yaw"]) - FRONT - radius


def gaps_since(probe, since_index, x, y, radius):
    """(gap, speed between the poses) along the true track since that index, by the times of the simulated board: a
    pose read twice is no stop and the next step no jump."""
    with probe.lock:
        track = list(probe.track[since_index:])
    return [(gap(b, x, y, radius), math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / (b["t"] - a["t"]))
            for a, b in zip(track, track[1:]) if b["t"] > a["t"]]


class TestObstacleStop(ObstacleSim, unittest.TestCase):
    STAGE = STAGE
    LAWN = LAWN
    PROBE_NAME = "obstacle_stop_probe"

    def stop_at_ball(self):
        """A ball ahead on a straight run, the mower stopped for it: (x, y of the ball, the true pose, the index of the
        events before)."""
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + BALL_RADIUS + BALL_GAP)
        since = len(p.events)
        STAGE.place(ball(x, y))
        self.paused_for(since, "OBJECT", resume="auto")
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        return x, y, poses[1], since

    def goes_on_after(self, wait, *things):
        """The scene changed to things: it goes on after wait plus the second to see the place free, not before.
        Returns how long it took and the index of the events before."""
        p = self.probe
        since = len(p.events)
        changed = time.time()
        STAGE.place(*things)
        self.assertIsNotNone(p.event(since, "STATE", wait + SEEN_FREE + 6.0, state="MOWING"),
                             f"it did not go on: {p.robot and p.robot.pause_reason} {p.obstacles and p.obstacles.tracks}")
        took = time.time() - changed
        self.assertGreater(took, wait)
        return took, since

    def test_01_mowing_on_the_lawn(self):
        self.mow_on_the_lawn()

    def test_02_an_object_ahead_stops_it_until_its_place_is_seen_free(self):
        p = self.probe
        x, y, stood, since = self.stop_at_ball()
        self.assertIsNotNone(p.event(since, "BLADES", 5, enabled=False), "the blade kept running")
        left = gap(stood, x, y, BALL_RADIUS)
        self.assertGreater(left, 0.25, f"stopped {left:.2f} m before the ball")
        self.assertLess(left, 0.8 + 0.05)
        # It stays while the ball lies there, longer than the wait for a free place.
        time.sleep(FREE_WAIT + 2.0)
        self.assertEqual(p.robot.current_state, "PAUSED")
        self.assertIsNone(p.event(since, "STATE", 0.1, state="MOWING"))
        took, since = self.goes_on_after(NEAR_WAIT if left < NEAR_AHEAD else FREE_WAIT)
        self.assertIsNotNone(p.event(since, "BLADES", 60, enabled=True), "the blade did not come back")
        print(f"object: {left:.2f} m left before the ball, on {took:.1f} s after it went")

    def test_03_a_person_slows_it_stops_it_farther_off_and_lets_it_go_after_walking_away(self):
        p = self.probe
        start = p.straight_run(5.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, FRONT + 0.25 + 2.9)
        since, index = len(p.events), len(p.track)
        STAGE.place(person(x, y))
        self.addCleanup(STAGE.place)
        self.paused_for(since, "PERSON", 20, resume="auto")
        time.sleep(1.5)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        left = gap(poses[1], x, y, 0.25)
        self.assertGreater(left, 0.6, f"stopped {left:.2f} m before the person")
        # In the zone to go slow (2 m) at most speed_slow, after a second to brake.
        slowed = [v for g, v in gaps_since(p, index, x, y, 0.25) if left + 0.1 < g < 1.6]
        self.assertTrue(slowed, "no pose in the slow zone")
        self.assertLessEqual(max(slowed), SPEED_SLOW + 0.05, slowed)
        # The person walks away to the side, out of the corridor.
        px, py = point(poses[1], FRONT + 1.5, 3.0)
        took, _ = self.goes_on_after(FREE_WAIT, person(px, py))
        STAGE.place()
        print(f"person: {left:.2f} m left, slow zone at most {max(slowed):.2f} m/s, on after {took:.1f} s")

    def test_04_an_animal_beside_it_out_of_view_holds_it_until_the_user_continues(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        x, y = point(start, 2.6, 1.2)
        since = len(p.events)
        STAGE.place(dog(x, y))
        self.paused_for(since, "ANIMAL", resume="auto")
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
        STAGE.place()

    def test_05_after_a_thing_right_at_the_nose_it_waits_thirty_seconds(self):
        p = self.probe
        x, y, stood, _ = self.stop_at_ball()
        # A person steps in right before the nose, still in view, and goes again with the ball. Nearer than about 0.44 m
        # the sensor cannot see the place free at all; the track counts its spread nearer, so it stands at 0.53 m.
        cx, cy = point(stood, FRONT + 0.53 + 0.25)
        STAGE.place(ball(x, y), person(cx, cy))
        self.assertTrue(p.wait(lambda: any(t.kind == "PERSON" and t.zone == 2 and t.in_view and 0 < t.ahead < NEAR_AHEAD
                                           for t in p.obstacles.tracks), 5), p.obstacles.tracks)
        # The track of a person follows its last find: with the spread of the sensor it ends up to a few cm nearer,
        # where its place cannot be seen free. Sharp last finds put it where the person stood; the pose adds a spread
        # of about 0.03 m of its own.
        try:
            p.flags(obstacles_sigma=[0.005, 0.0])
            self.assertTrue(p.wait(lambda: any(t.kind == "PERSON" and t.in_view and t.sigma < 0.04
                                               for t in p.obstacles.tracks), 5), p.obstacles.tracks)
            took, _ = self.goes_on_after(NEAR_WAIT)
        finally:
            p.flags()
        print(f"near: on after {took:.1f} s")

    def test_06_without_sight_nothing_stops_it(self):
        p = self.probe
        start = p.straight_run(4.5)
        self.assertIsNotNone(start, "no straight run")
        since = len(p.events)
        p.flags(obstacles_dark=1)
        self.assertIsNotNone(p.event(since, "OBSTACLE_SIGHT", 6), "no word that the camera sees nothing")
        now = p.truth()
        STAGE.place(ball(*point(now, FRONT + BALL_RADIUS + BALL_GAP)))
        time.sleep(6.0)
        self.assertIsNone(p.event(since, "STATE", 0.1, state="PAUSED"))
        self.assertGreater(p.speed(), 0.1)
        STAGE.place()
        p.flags()
        self.assertTrue(p.wait(lambda: p.obstacles is not None and p.obstacles.sight, 10))
