"""Wheels that slip while the mower turns on the spot: the pose follows the fixes, and without them its accuracy
grows with what the gyro stays behind the wheels.

The chain is fake_tron, tron_mcu, tron_gnss, tron_imu and the localization with the values of the Tron; the test
plays the logic (the geometry on bridge/params) and drives on hw/cmd_vel. The left wheel loses 0.8 of its speed on
the ground: the base swings around it and moves some 5 cm/s, which the wheels do not report.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_spot_slip.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import RobotPose
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, REAL_FAULTS, ROSOUT, SimProbe, TestShutdown, fake_tron, layers,  # noqa: E402, F401
                        sim_files, sim_links)

ANTENNA_X = 0.098
WORK = tempfile.mkdtemp(prefix="mowbite_spot_slip_")
LINKS = sim_links(WORK)
STATE, FLAGS, _, _ = sim_files(WORK)
TRON = layers.files("tron")
LOCAL = os.path.join(WORK, "local.yaml")
SLIP = [0.8, 0.0]
TURN = 0.6
JUMP = "Position moved"
# m: max_position_accuracy of the follower and the logic.
LIMIT = 0.10


def generate_test_description():
    with open(LOCAL, "w") as f:
        f.write(f"tron_mcu:\n  ros__parameters:\n    port: {LINKS['mcu']}\n    safety_measurement_date: simulation\n"
                f"tron_gnss:\n  ros__parameters:\n    port: {LINKS['gnss']}\n    data_dir: {WORK}\n"
                f"tron_imu:\n  ros__parameters:\n    port: {LINKS['imu']}\n")
    nodes = [
        Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="tron_gnss", executable="tron_gnss_node", name="tron_gnss", output="screen",
             parameters=[*TRON, LOCAL, {"datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0}]),
        Node(package="tron_imu", executable="tron_imu_node", name="tron_imu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", name="mowbite_localization",
             output="screen", parameters=[*TRON, LOCAL]),
    ]
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--state", STATE, "--antenna-offset", str(ANTENNA_X), "0.0"),
        TimerAction(period=1.5, actions=nodes + [launch_testing.actions.ReadyToTest()])]), {}


class Probe(SimProbe):
    """Drives on hw/cmd_vel and keeps the last localization/pose."""
    faults = REAL_FAULTS

    def __init__(self):
        super().__init__("spot_slip_probe", state=STATE, flags=FLAGS)
        self.pose = None
        self.setpoint = (0.0, 0.0)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.cmd_pub = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.geometry = self.node.create_publisher(String, "bridge/params", LATCHED)
        self.geometry.publish(String(data=json.dumps(
            {"node": "/mowbite_geometry", "params": {"antenna_x": ANTENNA_X, "antenna_y": 0.0, "version": 0}})))
        self.start()
        self.spawn(self._drive)

    def _drive(self):
        while self.running:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.setpoint
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def error(self):
        """m between the pose and the true place of base_link, and the accuracy the pose reports."""
        truth = self.truth()
        with self.lock:
            return math.hypot(self.pose.x - truth["x"], self.pose.y - truth["y"]), self.pose.position_accuracy

    def spin(self, seconds, until=None):
        """Turns on the spot for seconds or until until() says so; returns the (error, accuracy) of every tenth of a
        second and the true way of base_link."""
        start = self.truth()
        samples = []
        self.setpoint = (0.0, TURN)
        end = time.time() + seconds
        while time.time() < end and not (until and until()):
            samples.append(self.error())
            time.sleep(0.1)
        self.setpoint = (0.0, 0.0)
        time.sleep(1.0)
        now = self.truth()
        return samples, math.hypot(now["x"] - start["x"], now["y"] - start["y"])


class TestSpotSlip(unittest.TestCase):
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

    def test_1_with_fixes_the_pose_follows_the_sliding_mower(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.gnss_recent and p.pose.position_accuracy < 0.05,
                               60), p.pose)
        p.setpoint = (0.3, 0.0)
        self.assertTrue(p.wait(lambda: p.pose.heading_valid, 6), p.pose)
        p.setpoint = (0.0, 0.0)
        time.sleep(1.5)
        logs = len(p.logs)
        p.flags(spot_slip=SLIP)
        samples, way = p.spin(8.0)
        # The base swings around the wheel that grips.
        self.assertGreater(way, 0.15)
        self.assertLess(max(error for error, _ in samples), 0.07, samples)
        self.assertLess(max(accuracy for _, accuracy in samples), LIMIT, samples)
        self.assertFalse([line for line in p.logs[logs:] if JUMP in line])

    def test_2_without_fixes_the_accuracy_stays_ahead_of_the_error(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy < 0.05 and p.pose.fix_age < 0.5, 10), p.pose)
        p.flags(spot_slip=SLIP, gnss_quality=5)
        self.assertTrue(p.wait(lambda: p.pose.fix_age > 0.5, 5), p.pose)
        began = time.time()
        samples, _ = p.spin(8.0, lambda: p.pose.position_accuracy > LIMIT)
        # 0.2 m * (0.6 - 0.36) rad/s on top of the time: the limit comes within 2 s, not after 8.
        self.assertGreater(p.pose.position_accuracy, LIMIT, samples)
        self.assertLess(time.time() - began, 4.0, samples)
        for error, accuracy in samples:
            self.assertLess(error, accuracy + 0.03, samples)
        error, _ = p.error()
        self.assertLess(error, LIMIT + 0.03)
        p.flags()
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy < 0.05, 15), p.pose)
