"""xcore_capture against a capture of fake_xcore with xcore_driver as the host: the presses, tilts and the turn made
here must come out with the fake's known values (STOP 10 ms, lift 2.5 s, collision 0.5 s, both sensors at once without
delay, latches after the release, IMU at 100 Hz).

The fake sends the input mask on every change, so the lower bound of each delay is the fake's real delay.
Run in a sourced workspace: launch_test tools/yf/test/test_capture_fake.py
"""
import json
import os
import random
import shutil
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, Vector3
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import xcore_capture as xc  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_capture_fake_")
FLAGS = os.path.join(WORK, "flags.json")
PCAP = os.path.join(WORK, "inner.pcap")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
SERVICES = os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "config", "xcore_nx60i_services.json")
# What the test does, in this order: (flag, seconds held); None for the tilts and the turn.
PRESSES = [("stop", 1.0), ("rear_stop", 1.0), ("lift_left", 3.2), ("lift_right", 1.0), ("lift", 0.6),
           ("bumper_left", 1.0), ("bumper_right", 0.3), ("bumper", 0.6)]


def generate_test_description():
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", SERVICES, "--group-port", str(GROUP_PORT), "--flags", FLAGS, "--pcap", PCAP,
             "--pcap-max-mb", "40"],
        output="screen")
    config = os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config", "nx60i.yaml")
    driver = Node(package="xcore_driver", executable="xcore_driver_node", output="screen",
                  parameters=[config, {"interface": "lo", "bind_ip": BIND_IP, "announcement_port": GROUP_PORT}])
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ])


def set_flags(**flags):
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.node = rclpy.create_node("capture_fake_probe")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def healthy(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.status is not None and self.status.telemetry_ok and not self.status.emergency:
                    return True
            time.sleep(0.02)
        return False

    def release(self):
        assert self.emergency.wait_for_service(timeout_sec=5)
        future = self.emergency.call_async(SetEmergency.Request(latch=False, reason="capture test"))
        end = time.time() + 5
        while not future.done() and time.time() < end:
            time.sleep(0.01)
        return future.result()

    def turn(self, w, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(Twist(angular=Vector3(z=w)))
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


class TestCaptureOfFake(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        assert cls.probe.healthy(30), "driver never ready"
        time.sleep(1.5)  # still and level for the rest values
        for flag, seconds in PRESSES:
            set_flags(**{flag: True})
            time.sleep(seconds)
            set_flags()
            time.sleep(0.6)
            cls.settle()
        set_flags(roll=45)
        time.sleep(1.5)
        set_flags()
        time.sleep(0.6)
        cls.settle()
        set_flags(pitch=35)
        time.sleep(1.2)
        set_flags()
        time.sleep(0.6)
        cls.settle()
        cls.probe.turn(0.25, 2.0)
        time.sleep(1.0)
        cls.probe.stop()
        rclpy.shutdown()
        cap = xc.load_capture(PCAP, SERVICES)
        cls.report = xc.analyse(cap, (xc.NX60I_INPUTS, "test"), {"driver": (40.0, 0.5), "safety": (31.0, 1.0)},
                                15.0, 0.1)
        print(xc.text_report(cls.report))

    @classmethod
    def settle(cls):
        if not cls.probe.healthy(0.3):
            answer = cls.probe.release()
            assert answer is not None and answer.success, answer
            assert cls.probe.healthy(5), "no release"

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(WORK, ignore_errors=True)

    def presses(self):
        boot, = self.report["boots"]
        return boot["presses"]

    def test_a_capture_is_clean(self):
        c = self.report["capture"]
        self.assertEqual(c["board_found_by"], "announcements")
        self.assertEqual(c["malformed"], {})
        self.assertEqual(c["dangerous_ports"], {})
        self.assertFalse(c["cut_at_end"])
        self.assertEqual(self.report["inputs_from"], "the input configuration in the capture")
        self.assertEqual(len(self.presses()), len(PRESSES) + 2)  # lift and bumper "both" are two presses each

    def test_b_delays_and_latches(self):
        p = self.presses()
        names = [x["input"] for x in p]
        self.assertEqual(names, ["gpio GPIO1", "gpio GPIO5", "gpio GPIO10", "gpio GPIO11", "gpio GPIO10",
                                 "gpio GPIO11", "gpio GPIO13", "gpio GPIO12", "gpio GPIO12", "gpio GPIO13"])
        expected = [("STOP", 10), ("STOP", 10), ("LIFT", 2500), (None, None), ("LIFT_MULTIPLE", 10),
                    ("LIFT_MULTIPLE", 10), ("COLLISION", 500), (None, None), ("COLLISION_MULTIPLE", 10),
                    ("COLLISION_MULTIPLE", 10)]
        for press, (reason, delay) in zip(p, expected):
            with self.subTest(press["input"], at=press["pressed_at"]):
                self.assertEqual(press["reason"], reason)
                if reason is None:
                    self.assertGreater(press["held_s"], 0.2)
                    continue
                self.assertGreaterEqual(press["delay_min_ms"], delay - 1)
                self.assertLessEqual(press["delay_min_ms"], delay + 25)
                self.assertLessEqual(press["delay_min_ms"], press["delay_max_ms"])
                self.assertTrue(press["latched"])
                self.assertFalse(press["reason_after_release"])
                self.assertIsNotNone(press["host_clear_at"])
                self.assertGreater(press["host_clear_at"], press["released_at"])

    def test_c_imu(self):
        imu = self.report["boots"][0]["imu"]
        self.assertAlmostEqual(imu["rate_hz"], 100.0, delta=3.0)
        self.assertAlmostEqual(imu["rest"]["a_z"], xc.GRAVITY, delta=0.05)
        self.assertLess(imu["rest"]["a_z_std"], 0.05)
        roll, pitch = imu["tilt_episodes"]
        self.assertAlmostEqual(roll["max_tilt"], 45, delta=1.0)
        self.assertAlmostEqual(roll["roll_at_max"], 45, delta=1.0)
        self.assertAlmostEqual(roll["checks"]["driver"]["after_crossing_s"], 0.5, delta=0.03)
        self.assertAlmostEqual(roll["checks"]["safety"]["after_crossing_s"], 1.0, delta=0.03)
        self.assertAlmostEqual(pitch["pitch_at_max"], 35, delta=1.0)
        self.assertIsNone(pitch["checks"]["driver"]["latched_at"])
        self.assertAlmostEqual(pitch["checks"]["safety"]["after_crossing_s"], 1.0, delta=0.03)
        turn, = imu["yaw_episodes"]
        self.assertEqual(turn["sign"], "+")
        self.assertGreater(turn["angle_deg"], 15)
