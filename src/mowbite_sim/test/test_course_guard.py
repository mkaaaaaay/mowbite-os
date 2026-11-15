"""A receiver that reports a wrong course at the right speed: the localization keeps to the track of the fixes.

The chain is fake_tron, tron_mcu, tron_gnss (motion vector from the course of the receiver, not from the positions),
tron_imu and the localization with the values of tron.yaml; the test plays the logic (the geometry on bridge/params)
and drives straight on hw/cmd_vel. With every course 160 deg off (as the LG290P 0.9.50 sent it) the heading stays
invalid and the localization warns; with the right course it aligns.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_course_guard.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import RobotPose
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, REAL_FAULTS, ROSOUT, SimProbe, TestShutdown, fake_tron, sim_files,  # noqa: E402, F401
                        sim_links)

ANTENNA_X = 0.098
WORK = tempfile.mkdtemp(prefix="mowbite_course_guard_")
LINKS = sim_links(WORK)
STATE, FLAGS, _, _ = sim_files(WORK)
TRON = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "tron.yaml")
LOCAL = os.path.join(WORK, "local.yaml")
SPEED = 0.3
WARNING = "GPS course against the track of the fixes"


def generate_test_description():
    with open(LOCAL, "w") as f:
        f.write(f"tron_mcu:\n  ros__parameters:\n    port: {LINKS['mcu']}\n    safety_measurement_date: simulation\n"
                f"tron_gnss:\n  ros__parameters:\n    port: {LINKS['gnss']}\n    data_dir: {WORK}\n"
                f"    motion_from_fixes: false\n"
                f"tron_imu:\n  ros__parameters:\n    port: {LINKS['imu']}\n")
    nodes = [
        Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", output="screen",
             parameters=[TRON, LOCAL]),
        Node(package="tron_gnss", executable="tron_gnss_node", name="tron_gnss", output="screen",
             parameters=[TRON, LOCAL, {"datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0}]),
        Node(package="tron_imu", executable="tron_imu_node", name="tron_imu", output="screen",
             parameters=[TRON, LOCAL]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", name="mowbite_localization",
             output="screen", parameters=[TRON, LOCAL]),
    ]
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--state", STATE, "--antenna-offset", str(ANTENNA_X), "0.0"),
        TimerAction(period=1.5, actions=nodes + [launch_testing.actions.ReadyToTest()])]), {}


class Probe(SimProbe):
    """Drives on hw/cmd_vel and keeps the last localization/pose."""
    faults = REAL_FAULTS

    def __init__(self):
        super().__init__("course_guard_probe", state=STATE, flags=FLAGS)
        self.pose = None
        self.setpoint = (0.0, 0.0)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 10)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.cmd_pub = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.geometry = self.node.create_publisher(String, "bridge/params", LATCHED)
        self.geometry.publish(String(data=json.dumps(
            {"node": "/mowbite_geometry", "params": {"antenna_x": ANTENNA_X, "antenna_y": 0.0, "version": 0}})))
        self.start()
        self.spawn(self._drive)

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _drive(self):
        while self.running:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.setpoint
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def heading_valid(self):
        with self.lock:
            return self.pose is not None and self.pose.heading_valid

    def drive(self, seconds, speed, until=None):
        """Drives straight for seconds, back and forth in the free space; until() ends it early. Returns its value."""
        self.setpoint = (speed, 0.0)
        found = self.wait(until, seconds) if until else time.sleep(seconds)
        self.setpoint = (0.0, 0.0)
        time.sleep(1.0)
        return found


class TestCourseGuard(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags(gnss_course_error=160.0)

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_a_wrong_course_aligns_nothing(self):
        p = self.probe
        # Calibrated gyro and GNSS set: a pose comes, the heading is not valid at rest.
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.gnss_recent and p.pose.position_accuracy < 0.05,
                               60), p.pose)
        self.assertFalse(p.heading_valid())
        logs = len(p.logs)
        # Forward and back again: 8 s at 0.3 m/s give some 40 courses, each one 160 deg off.
        p.drive(4.0, SPEED)
        p.drive(4.0, -SPEED)
        self.assertFalse(p.heading_valid())
        self.assertTrue(p.logged(WARNING, 12, logs), p.logs[logs:])

    def test_2_the_right_course_aligns(self):
        p = self.probe
        p.flags(gnss_course_error=0.0)
        self.assertTrue(p.drive(4.0, SPEED, p.heading_valid))

    def test_3_a_lift_makes_the_heading_uncertain(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.pose.heading_accuracy < 0.1, 10), p.pose)
        logs = len(p.logs)
        p.flags(lift=1)
        time.sleep(1.0)
        p.flags()
        self.assertTrue(p.logged("Lift ended", 10, logs), p.logs[logs:])
        self.assertTrue(p.wait(lambda: p.pose.heading_valid and p.pose.heading_accuracy > 0.3, 5), p.pose)
