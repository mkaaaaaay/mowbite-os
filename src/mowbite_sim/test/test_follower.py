"""mowbite_follower on its own: the test plays the driver (hw/status, hw/capabilities) and the localization, which
moves a unicycle by what the follower sends on follower/cmd_vel, and sends paths on follow_path. In the station frame
the unicycle moves by follower/dock_cmd_vel and is seen on localization/dock_pose, while localization/pose shows it
somewhere else.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_follower.py
"""
import math
import os
import sys
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Point32, Polygon, Pose, Twist
from launch_ros.actions import Node
from mowbite_msgs.action import FollowPath
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus, RobotPose
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe  # noqa: E402

LAWN = [(-1, -2), (12, -2), (12, 4), (-1, 4)]
Result = FollowPath.Result


def generate_test_description():
    follower = Node(package="mowbite_follower", executable="mowbite_follower_node", output="screen",
                    parameters=[{"max_turn_slide": 0.15}])
    return launch.LaunchDescription([follower, launch_testing.actions.ReadyToTest()]), {}


def polygon(points):
    return Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in points])


def facing(poses, yaw):
    """The same poses facing yaw, as a path driven backwards has them."""
    for p in poses:
        p.orientation.z = math.sin(yaw / 2)
        p.orientation.w = math.cos(yaw / 2)
    return poses


def line_poses(x0, y0, yaw, length, spacing=0.1):
    poses = []
    for i in range(int(round(length / spacing)) + 1):
        p = Pose()
        p.position.x = x0 + i * spacing * math.cos(yaw)
        p.position.y = y0 + i * spacing * math.sin(yaw)
        p.orientation.z = math.sin(yaw / 2)
        p.orientation.w = math.cos(yaw / 2)
        poses.append(p)
    return poses


class World(SimProbe):
    """Driver and localization: a mower that moves the way it is told on follower/cmd_vel."""
    spin_timeout = 0.02

    def __init__(self):
        super().__init__("follower_world")
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.cmds = []
        self.emergency = False
        self.accuracy = 0.02
        self.publish_pose = True
        # The pose in the station frame: the unicycle itself while dock is set; the map pose then lies 5 m away.
        self.dock = False
        self.dock_accuracy = 0.02
        self.dock_cmds = []
        # s the last fix behind the map pose is old, and m/s the mower slides towards +y while it turns.
        self.fix_age = 0.0
        self.slide = 0.0
        self.caps_pub = self.node.create_publisher(HardwareCapabilities, "hw/capabilities", LATCHED)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.dock_pub = self.node.create_publisher(RobotPose, "localization/dock_pose", 10)
        self.node.create_subscription(Twist, "follower/cmd_vel", self._cmd, 50)
        self.node.create_subscription(Twist, "follower/dock_cmd_vel", self._dock_cmd, 50)
        self.client = ActionClient(self.node, FollowPath, "follow_path")
        self.start()
        threading.Thread(target=self._world, daemon=True).start()

    def capabilities(self, date, max_linear=0.3, max_angular=0.3):
        caps = HardwareCapabilities()
        caps.platform = "test"
        caps.max_linear_speed = max_linear
        caps.max_angular_speed = max_angular
        caps.safety_measurement_date = date
        self.caps_pub.publish(caps)

    def place(self, x, y, yaw):
        with self.lock:
            self.x, self.y, self.yaw = x, y, yaw
            self.v = self.w = 0.0
        # The follower checks the next goal against the pose it has; under load the new one takes a while.
        time.sleep(0.3)

    def _cmd(self, msg):
        with self.lock:
            self.v, self.w = msg.linear.x, msg.angular.z
            self.last_cmd = time.time()
            self.cmds.append((time.time(), msg.linear.x, msg.angular.z))

    def _dock_cmd(self, msg):
        with self.lock:
            self.dock_cmds.append((time.time(), msg.linear.x, msg.angular.z))
            if self.dock:
                self.v, self.w = msg.linear.x, msg.angular.z
                self.last_cmd = time.time()

    def _world(self):
        dt = 0.02
        while self.running:
            with self.lock:
                # Like the driver's watchdog: without a command for 0.5 s the mower stands.
                if time.time() - self.last_cmd > 0.5:
                    self.v = self.w = 0.0
                self.yaw += self.w * dt
                self.x += self.v * dt * math.cos(self.yaw)
                self.y += self.v * dt * math.sin(self.yaw) + (self.slide * dt if self.w else 0.0)
                hw = HardwareStatus()
                hw.telemetry_ok = True
                hw.emergency = self.emergency
                pose = RobotPose()
                pose.x = self.x + (5.0 if self.dock else 0.0)
                pose.y = self.y
                pose.heading = self.yaw
                pose.position_accuracy = self.accuracy
                pose.heading_valid = True
                pose.fix_age = self.fix_age
                publish_pose = self.publish_pose
                dock = None
                if self.dock:
                    dock = RobotPose()
                    dock.header.frame_id = "dock"
                    dock.x = self.x
                    dock.y = self.y
                    dock.heading = self.yaw
                    dock.position_accuracy = self.dock_accuracy
                    dock.heading_valid = True
            self.hw_pub.publish(hw)
            if publish_pose:
                self.pose_pub.publish(pose)
            if dock is not None:
                self.dock_pub.publish(dock)
            time.sleep(dt)

    def follow(self, poses, start_index=0, allowed=None, obstacles=(), wait=True, frame=FollowPath.Goal.FRAME_MAP,
               reverse=False, body=()):
        goal = FollowPath.Goal()
        goal.frame = frame
        goal.reverse = reverse
        goal.poses = poses
        goal.start_index = start_index
        goal.allowed_areas = [polygon(allowed or LAWN)]
        goal.obstacles = [polygon(o) for o in obstacles]
        goal.body = polygon(body)
        assert self.client.wait_for_server(timeout_sec=10.0), "follow_path not offered"
        future = self.client.send_goal_async(goal)
        end = time.time() + 10
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        handle = future.result()
        if not wait or not handle.accepted:
            return handle
        return self.result(handle)

    def result(self, handle, timeout=90.0):
        future = handle.get_result_async()
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), "no result in time"
        return future.result().result

    def commands_since(self, t):
        with self.lock:
            return [c for c in self.cmds if c[0] >= t]

    def dock_commands_since(self, t):
        with self.lock:
            return [c for c in self.dock_cmds if c[0] >= t]


class TestFollower(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.world = World()

    @classmethod
    def tearDownClass(cls):
        cls.world.stop()
        rclpy.shutdown()

    def setUp(self):
        w = self.world
        with w.lock:
            w.emergency = False
            w.accuracy = 0.02
            w.publish_pose = True
            w.dock = False
            w.dock_accuracy = 0.02
            w.fix_age = 0.0
            w.slide = 0.0
        w.capabilities("2026-01-01")
        w.place(0.0, 0.0, 0.0)
        time.sleep(0.3)

    def test_01_no_driving_without_a_safety_measurement(self):
        self.world.capabilities("")
        time.sleep(0.3)
        t = time.time()
        result = self.world.follow(line_poses(0, 0, 0, 2.0))
        self.assertEqual(result.reason, Result.REASON_NOT_ALLOWED, result.message)
        self.assertTrue(all(v == 0 and w == 0 for _, v, w in self.world.commands_since(t)))

    def test_01b_no_driving_when_the_driver_allows_no_speed(self):
        self.world.capabilities("2026-01-01", max_linear=0.0, max_angular=0.0)
        time.sleep(0.3)
        t = time.time()
        result = self.world.follow(line_poses(0, 0, 0, 2.0))
        self.assertEqual(result.reason, Result.REASON_NOT_ALLOWED, result.message)
        self.assertIn("no speed", result.message)
        self.assertTrue(all(v == 0 and w == 0 for _, v, w in self.world.commands_since(t)))

    def test_02_follows_a_line_and_stops_sending(self):
        result = self.world.follow(line_poses(0, 0, 0, 3.0))
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)
        self.assertEqual(result.index, 30)
        with self.world.lock:
            x, y = self.world.x, self.world.y
        self.assertAlmostEqual(x, 3.0, delta=0.15)
        self.assertAlmostEqual(y, 0.0, delta=0.05)
        cmds = self.world.commands_since(0)
        self.assertLessEqual(max(abs(v) for _, v, _ in cmds), 0.3 + 1e-6)
        self.assertLessEqual(max(abs(w) for _, _, w in cmds), 0.3 + 1e-6)
        # The last command is a zero, then silence.
        self.assertEqual(cmds[-1][1:], (0.0, 0.0))
        t = time.time()
        time.sleep(0.5)
        self.assertEqual(self.world.commands_since(t), [])

    def test_03_emergency_stops(self):
        handle = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        self.assertTrue(handle.accepted)
        time.sleep(2.0)
        with self.world.lock:
            self.world.emergency = True
        result = self.world.result(handle)
        self.assertEqual(result.reason, Result.REASON_EMERGENCY, result.message)
        self.assertGreater(result.index, 0)
        t = time.time()
        time.sleep(0.5)
        self.assertEqual(self.world.commands_since(t), [])

    def test_04_bad_accuracy_stops(self):
        handle = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        time.sleep(1.5)
        with self.world.lock:
            self.world.accuracy = 0.5
        result = self.world.result(handle)
        self.assertEqual(result.reason, Result.REASON_POSITION_ACCURACY, result.message)

    def test_05_lost_pose_stops(self):
        handle = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        time.sleep(1.5)
        with self.world.lock:
            self.world.publish_pose = False
        result = self.world.result(handle)
        self.assertEqual(result.reason, Result.REASON_POSE_LOST, result.message)

    def test_06_leaving_the_allowed_area_stops(self):
        # The path runs out of the lawn: the follower stops 0.5 m beyond its edge.
        result = self.world.follow(line_poses(0, 0, 0, 6.0), allowed=[(-1, -2), (3, -2), (3, 4), (-1, 4)])
        self.assertEqual(result.reason, Result.REASON_OUTSIDE, result.message)
        with self.world.lock:
            x = self.world.x
        self.assertGreater(x, 3.4)
        self.assertLess(x, 3.7)

    def test_07_cancel(self):
        handle = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        time.sleep(1.5)
        handle.cancel_goal_async()
        result = self.world.result(handle)
        self.assertEqual(result.reason, Result.REASON_CANCELED, result.message)

    def test_08_one_path_at_a_time(self):
        first = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        self.assertTrue(first.accepted)
        second = self.world.follow(line_poses(0, 0, 0, 6.0), wait=False)
        self.assertFalse(second.accepted)
        first.cancel_goal_async()
        self.world.result(first)

    def test_09_invalid_paths(self):
        result = self.world.follow(line_poses(0, 0, 0, 0.1))
        self.assertEqual(result.reason, Result.REASON_INVALID)
        self.assertIn("at least 3 poses", result.message)
        result = self.world.follow(line_poses(0, 0, 0, 2.0), allowed=[(0, 0), (1, 1)])
        self.assertEqual(result.reason, Result.REASON_INVALID)
        self.assertIn("fewer than 3 points", result.message)
        result = self.world.follow(line_poses(0, 3, 0, 2.0))
        self.assertEqual(result.reason, Result.REASON_INVALID)
        self.assertIn("too far from the start", result.message)

    def test_10_goes_on_from_a_start_index(self):
        self.world.place(1.5, 0.0, 0.0)
        result = self.world.follow(line_poses(0, 0, 0, 3.0), start_index=15)
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)
        with self.world.lock:
            self.assertAlmostEqual(self.world.x, 3.0, delta=0.15)

    def test_11_follows_a_path_in_the_station_frame(self):
        # The station frame: the docked pose at the origin, the way in along +x; backwards onto it from 0.8 m.
        w = self.world
        with w.lock:
            w.dock = True
        w.place(-0.8, 0.0, math.pi)
        t = time.time()
        corridor = [(-1.0, -0.1), (0.1, -0.1), (0.1, 0.1), (-1.0, 0.1)]
        result = w.follow(facing(line_poses(-0.8, 0.0, 0.0, 0.7), math.pi), allowed=corridor,
                          frame=FollowPath.Goal.FRAME_DOCK, reverse=True)
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)
        with w.lock:
            x, y = w.x, w.y
        self.assertAlmostEqual(x, -0.1, delta=0.12)
        self.assertAlmostEqual(y, 0.0, delta=0.05)
        # Only on follower/dock_cmd_vel, backwards, ending with a zero; nothing on follower/cmd_vel.
        dock_cmds = w.dock_commands_since(t)
        self.assertGreater(len(dock_cmds), 10)
        self.assertTrue(all(v <= 0 for _, v, _ in dock_cmds))
        self.assertEqual(dock_cmds[-1][1:], (0.0, 0.0))
        self.assertEqual(w.commands_since(t), [])

    def test_12_the_station_frame_needs_the_accuracy_of_the_dock(self):
        # From the marker 0.03 m at the docked place and 0.09 m worse per m from it, whatever the map allows: 0.11 m
        # is refused 0.8 m out (limit 0.102 m), 0.04 m drives until about 0.11 m before the station.
        w = self.world
        with w.lock:
            w.dock = True
            w.accuracy = 0.04
            w.dock_accuracy = 0.11
        w.place(-0.8, 0.0, math.pi)
        result = w.follow(facing(line_poses(-0.8, 0.0, 0.0, 0.7), math.pi), frame=FollowPath.Goal.FRAME_DOCK,
                          reverse=True)
        self.assertEqual(result.reason, Result.REASON_POSITION_ACCURACY, result.message)
        self.assertIn("worse than 0.10 m", result.message)
        with w.lock:
            self.assertAlmostEqual(w.x, -0.8, delta=0.02)
            w.dock_accuracy = 0.04
        # As in place: under load the follower takes a while to see the new accuracy.
        time.sleep(0.3)
        result = w.follow(facing(line_poses(-0.8, 0.0, 0.0, 0.7), math.pi), frame=FollowPath.Goal.FRAME_DOCK,
                          reverse=True)
        self.assertEqual(result.reason, Result.REASON_POSITION_ACCURACY, result.message)
        with w.lock:
            self.assertLess(w.x, -0.08)
            self.assertGreater(w.x, -0.2)
        # The map pose is 5 m away: a path there in the map frame is fine, its accuracy is the map's.
        with w.lock:
            w.dock_accuracy = 0.02
        w.place(-0.8, 0.0, math.pi)
        result = w.follow(facing(line_poses(4.2, 0.0, 0.0, 0.7), math.pi), frame=FollowPath.Goal.FRAME_MAP,
                          reverse=True)
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)

    def test_13_without_a_dock_pose_or_with_an_unknown_frame(self):
        w = self.world
        result = w.follow(line_poses(0, 0, 0, 1.0), frame=FollowPath.Goal.FRAME_DOCK)
        self.assertEqual(result.reason, Result.REASON_POSE_LOST, result.message)
        result = w.follow(line_poses(0, 0, 0, 1.0), frame=2)
        self.assertEqual(result.reason, Result.REASON_INVALID)
        self.assertIn("unknown frame 2", result.message)

    def test_14_no_turn_on_the_spot_without_a_fresh_fix(self):
        w = self.world
        with w.lock:
            w.fix_age = 3.0
        # The follower turns by the pose it has; the old one first.
        time.sleep(0.3)
        began = time.time()
        handle = w.follow(line_poses(0, 0, math.pi / 2, 1.0), wait=False)
        time.sleep(3.0)
        # It stands and keeps saying so; the path goes on.
        commands = w.commands_since(began)
        self.assertGreater(len(commands), 20)
        self.assertFalse([c for c in commands if c[1] != 0.0 or c[2] != 0.0], commands[:5])
        with w.lock:
            self.assertEqual(w.yaw, 0.0)
            w.fix_age = 0.1
        result = w.result(handle)
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)
        # A fix that old does not stop a drive along the path.
        w.place(0.0, 0.0, 0.0)
        with w.lock:
            w.fix_age = 3.0
        time.sleep(0.3)
        result = w.follow(line_poses(0, 0, 0, 1.0))
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)

    def test_15_a_turn_that_slides_away_ends_the_path(self):
        w = self.world
        with w.lock:
            w.slide = 0.12
        result = w.follow(line_poses(0, 0, math.pi, 1.0))
        self.assertEqual(result.reason, Result.REASON_TURN_SLIDE, result.message)
        self.assertIn("from the place of the turn on the spot (limit 0.15 m)", result.message)
        time.sleep(0.7)
        with w.lock:
            y, turned = w.y, w.yaw
        # Stopped soon after the limit, in the middle of the turn.
        self.assertGreater(y, 0.15)
        self.assertLess(y, 0.25)
        self.assertLess(abs(turned), 2.5)

    def test_16_a_turn_without_room_for_the_body_ends_the_path_before_it(self):
        w = self.world
        # The box of the Tron, 0.3 m beside the upper edge of the lawn, facing along it.
        body = [(0.6, 0.235), (-0.12, 0.235), (-0.12, -0.235), (0.6, -0.235)]
        w.place(2.0, 3.7, 0.0)
        result = w.follow(line_poses(2.0, 3.7, math.pi / 2, 1.0), body=body)
        self.assertEqual(result.reason, Result.REASON_OUTSIDE, result.message)
        self.assertIn("by 90 deg, the body would get 0.34 m further out of the allowed areas", result.message)
        with w.lock:
            self.assertEqual(w.yaw, 0.0)
        # Turning the other way it has the room.
        result = w.follow(line_poses(2.0, 3.7, -math.pi / 2, 1.0), body=body)
        self.assertEqual(result.reason, Result.REASON_DONE, result.message)
