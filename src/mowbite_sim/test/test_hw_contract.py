"""What tron_mcu owes the board of the Tron beyond the driver contract (test_driver_contract.py), against fake_tron on
a pty: the values of its capabilities and status, the rate of the drive frames and the frames of the blade.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hw_contract.py
"""
import functools
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, Vector3
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import HardwareStatus

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import TRON_CUTTER as CUTTER, TRON_SPEED as SPEED  # noqa: E402
from sim_common import (HwProbe, TestShutdown, fake_tron, last_time, sim_files, sim_links,  # noqa: E402,F401
                        sub_packets, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_hw_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)


def generate_test_description():
    driver = Node(package="tron_mcu", executable="tron_mcu_node", output="screen",
                  parameters=[{"port": LINKS["mcu"], "send_imu": False, "allow_blade": True, "blade_lease_max": 2.0,
                               "mow_height_mm": 50, "cutter_firmware": ["0.6.36"],
                               "chassis_firmware": ["0.6.34"]}])
    # The pty has to exist before the driver looks for it.
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
        TimerAction(period=1.0, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {}


received = functools.partial(sub_packets, LOG)


sim_time = functools.partial(last_time, LOG)


class Probe(HwProbe):
    def __init__(self):
        super().__init__("hw_contract_probe")
        self.start()


class TestHardwareContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        write_json(FLAGS, {})
        cls.probe = Probe()
        # Free once telemetry and a matching version frame arrived.
        ready = cls.probe.wait_for(
            lambda p: p.status is not None and p.status.telemetry_ok and not p.status.emergency, 15)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_a_capabilities_latched_with_firmware(self):
        caps = self.probe.wait_for(
            lambda p: p.capabilities if p.capabilities and p.capabilities.firmware[0].value else None)
        self.assertIsNotNone(caps, "no hw/capabilities with firmware")
        self.assertEqual(caps.platform, "airseekers_tron")
        firmware = {kv.key: kv.value for kv in caps.firmware}
        self.assertEqual(firmware["cutter"], "0.6.36")
        self.assertEqual(firmware["chassis"], "0.6.34")
        self.assertFalse(caps.blade_speed_control)
        self.assertTrue(caps.blade_height_control)
        self.assertEqual((caps.blade_height_min_mm, caps.blade_height_max_mm), (30, 90))
        self.assertAlmostEqual(caps.blade_lease_max_s, 2.0)
        self.assertFalse(caps.host_charge_control)
        self.assertTrue(caps.charge_stop)
        self.assertAlmostEqual(caps.wheel_ticks_per_m, 1000.0)
        self.assertAlmostEqual(caps.wheel_distance, 0.40)
        self.assertAlmostEqual(caps.max_linear_speed, 0.55, places=5)
        self.assertAlmostEqual(caps.max_angular_speed, 0.7, places=5)
        self.assertTrue(caps.gnss_receiver_time)
        self.assertLess(caps.failsafe_without_host_s, 0)
        self.assertEqual(caps.safety_measurement_date, "")

    def test_b_battery_from_the_bms(self):
        # Ready comes with the first sensor frame, the battery report only once a second.
        s = self.probe.wait_for(
            lambda p: p.status if p.status.battery_source == HardwareStatus.BATTERY_SOURCE_BMS else None, 3)
        self.assertIsNotNone(s, "no battery report from the BMS")
        self.assertAlmostEqual(s.battery_voltage, 24.5, places=3)

    def test_c_drive_frames_at_most_every_100ms(self):
        # At 20 SpeedData/s the MCU of the Tron kept a setpoint up to 3.6 s without host (08.11.).
        t0 = sim_time()
        for i in range(40):
            self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.1 + 0.001 * i)))
            time.sleep(0.05)
        self.probe.cmd_vel.publish(Twist())
        time.sleep(0.5)
        moving = [e for e in received(t0, SPEED) if e["speed"] != [0.0, 0.0]]
        self.assertGreaterEqual(len(moving), 15)
        # fake_tron notes its times in ms: the difference of two of them is a whole number of ms.
        gaps = [round(b["t"] - a["t"], 3) for a, b in zip(moving, moving[1:])]
        self.assertGreaterEqual(min(gaps), 0.095, str(gaps))
        self.assertAlmostEqual(moving[-1]["speed"][0], 0.138, delta=0.0025)

    def test_c_drive_watchdog_three_zeros(self):
        # Tighter than the driver contract: measured from the last frame the MCU got, and the limit by its number.
        t0 = sim_time()
        for _ in range(5):
            self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.7)))
            time.sleep(0.1)
        time.sleep(1.2)
        speeds = [(e["t"], e["speed"]) for e in received(t0, SPEED)]
        moving = [t for t, v in speeds if v != [0.0, 0.0]]
        zeros = [t for t, v in speeds if v == [0.0, 0.0]]
        self.assertGreaterEqual(len(moving), 4)
        self.assertTrue(all(abs(v[0] - 0.55) < 1e-6 for t, v in speeds if v != [0.0, 0.0]), "not clamped to 0.55")
        self.assertEqual(len(zeros), 3, str(speeds))
        self.assertAlmostEqual(zeros[0] - moving[-1], 0.5, delta=0.1)
        self.assertAlmostEqual(zeros[2] - zeros[0], 0.2, delta=0.05)

    def test_d_blade_lease_runs_out(self):
        t0 = sim_time()
        r = self.probe.set_blade(True, lease_s=0.5)
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.probe.wait_for(lambda p: p.status.blade_on, 2), "blade_on not reported")
        off = self.probe.wait_for(lambda p: not p.status.blade_on and p.status, 3)
        self.assertTrue(off, "blade did not go off")
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_LEASE_EXPIRED)
        time.sleep(0.4)
        frames = received(t0, CUTTER)
        on = [e for e in frames if e["hex"][:2] == "01"]
        self.assertEqual(len(on), 1, str(frames))
        # Default height 50 mm at bytes 10-11, speed 100 % at bytes 2-3.
        self.assertEqual(int.from_bytes(bytes.fromhex(on[0]["hex"])[10:12], "little"), 50)
        self.assertEqual(int.from_bytes(bytes.fromhex(on[0]["hex"])[2:4], "little"), 100)
        offs = [e["t"] for e in frames if e["hex"][:2] == "00" and e["t"] > on[0]["t"]]
        self.assertEqual(len(offs), 3, str(frames))
        self.assertAlmostEqual(offs[0] - on[0]["t"], 0.5, delta=0.1)

    def test_e_blade_renewed_stays_on(self):
        t0 = sim_time()
        for _ in range(8):
            r = self.probe.set_blade(True, lease_s=0.6, speed=80, height=40)
            self.assertTrue(r.success, r.message)
            time.sleep(0.25)
        self.assertTrue(self.probe.status.blade_on)
        self.assertTrue(self.probe.set_blade(False).success)
        off = self.probe.wait_for(lambda p: not p.status.blade_on and p.status, 2)
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_REQUEST)
        time.sleep(0.4)
        frames = received(t0, CUTTER)
        self.assertEqual(len([e for e in frames if e["hex"][:2] == "01"]), 1, str(frames))
        self.assertEqual(len([e for e in frames if e["hex"][:2] == "00"]), 3, str(frames))
