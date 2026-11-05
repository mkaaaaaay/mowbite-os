"""mowbite_follower on its own: the test plays the driver (hw/status, hw/capabilities) and the localization, which
moves a unicycle by what the follower sends on follower/cmd_vel, and sends paths on follow_path.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_follower.py
"""
import math
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LAWN = [(-1, -2), (12, -2), (12, 4), (-1, 4)]
Result = FollowPath.Result


def generate_test_description():
    follower = Node(package="mowbite_follower", executable="mowbite_follower_node", output="screen")
    return launch.LaunchDescription([follower, launch_testing.actions.ReadyToTest()]), {}


def polygon(points):
    return Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in points])


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


class World:
    """Driver and localization: a mower that moves the way it is told on follower/cmd_vel."""

    def __init__(self):
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.cmds = []
        self.emergency = False
        self.accuracy = 0.02
        self.publish_pose = True
        self.node = rclpy.create_node("follower_world")
        self.caps_pub = self.node.create_publisher(HardwareCapabilities, "hw/capabilities", LATCHED)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.node.create_subscription(Twist, "follower/cmd_vel", self._cmd, 50)
        self.client = ActionClient(self.node, FollowPath, "follow_path")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()
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

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _world(self):
        dt = 0.02
        while self.running:
            with self.lock:
                # Like the driver's watchdog: without a command for 0.5 s the mower stands.
                if time.time() - self.last_cmd > 0.5:
                    self.v = self.w = 0.0
                self.yaw += self.w * dt
                self.x += self.v * dt * math.cos(self.yaw)
                self.y += self.v * dt * math.sin(self.yaw)
                hw = HardwareStatus()
                hw.telemetry_ok = True
                hw.emergency = self.emergency
                pose = RobotPose()
                pose.x = self.x
                pose.y = self.y
                pose.heading = self.yaw
                pose.position_accuracy = self.accuracy
                pose.heading_valid = True
                publish_pose = self.publish_pose
            self.hw_pub.publish(hw)
            if publish_pose:
                self.pose_pub.publish(pose)
            time.sleep(dt)

    def follow(self, poses, start_index=0, allowed=None, obstacles=(), wait=True):
        goal = FollowPath.Goal()
        goal.poses = poses
        goal.start_index = start_index
        goal.allowed_areas = [polygon(allowed or LAWN)]
        goal.obstacles = [polygon(o) for o in obstacles]
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

    def stop(self):
        self.running = False
        time.sleep(0.2)
        self.node.destroy_node()


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
