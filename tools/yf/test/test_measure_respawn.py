"""measure.launch.py with respawn:=true against fake_xcore: a killed driver comes back latched from its first frame
(start_latched), so that a hw/cmd_vel still going on does not move the wheels before a release.

Run in a sourced workspace: launch_test tools/yf/test/test_measure_respawn.py
"""
import json
import os
import random
import shutil
import signal
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareStatus

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import measure as m  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_measure_respawn_")
FLAGS = os.path.join(WORK, "flags.json")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
SERVICES = os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "config", "xcore_nx60i_services.json")
FAKE = os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore")
BATTERY_PARAMS = os.path.join(FindPackageShare("platform_nx60i").find("platform_nx60i"), "platform.yaml")


def generate_test_description():
    with open(FLAGS, "w") as f:
        json.dump({}, f)
    fake_xcore = ExecuteProcess(cmd=[FAKE, "--services", SERVICES, "--group-port", str(GROUP_PORT), "--flags", FLAGS],
                                output="screen")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "launch",
                                                   "measure.launch.py")),
        launch_arguments={"battery_params": BATTERY_PARAMS, "interface": "lo", "bind_ip": BIND_IP,
                          "announcement_port": str(GROUP_PORT), "respawn": "true", "respawn_delay": "1.0"}.items())
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ])


class TestMeasureRespawn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("measure_respawn_probe")
        cls.status = []
        cls.node.create_subscription(HardwareStatus, "hw/status", cls.status.append, 10)
        cls.cmd = cls.node.create_publisher(Twist, "hw/cmd_vel", 1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def spin(self, seconds, until=None, drive=False):
        end = time.monotonic() + seconds
        twist = Twist()
        twist.linear.x = 0.1
        while time.monotonic() < end:
            if drive:
                self.cmd.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if until and until():
                return True
        return False

    def test_killed_driver_comes_back_latched(self):
        self.assertTrue(self.spin(20.0, until=lambda: self.status and self.status[-1].telemetry_ok),
                        "no healthy hw/status from the first driver")
        pids = m.driver_pids()
        self.assertEqual(len(pids), 1, pids)
        os.kill(pids[0], signal.SIGKILL)
        self.assertTrue(self.spin(30.0, until=lambda: m.driver_pids() and m.driver_pids() != pids),
                        "the launch started no new driver")
        self.status.clear()
        new = lambda: [s for s in self.status if s.telemetry_ok]  # noqa: E731
        self.assertTrue(self.spin(20.0, until=new, drive=True), "no healthy hw/status from the new driver")
        # The command goes on the whole time: the new driver holds the latch and never moves.
        self.spin(3.0, drive=True)
        for s in new():
            self.assertTrue(s.emergency_latched, s)
            self.assertTrue(s.emergency_reason & HardwareStatus.REASON_HIGH_LEVEL, s)
            self.assertFalse(s.moving, s)
