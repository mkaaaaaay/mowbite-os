"""What xcore_driver owes the board of the NX60i beyond the driver contract (test_driver_contract.py), against
fake_xcore on UDP over the loopback: configuration of the board, the values of its capabilities and status, IMU, the
datagrams of drive, blade, emergency and hold, charging left to the board, a restart of the board and incompatible
firmware.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_contract.py
"""
import functools
import json
import os
import shutil
import struct
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from launch.actions import TimerAction
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetCharge, SetEmergency
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import XCORE_DATA as DATA, XCORE_DIFF_DRIVE as DIFF_DRIVE  # noqa: E402
from sim_common import XCORE_EMERGENCY as EMERGENCY, XCORE_MOWER as MOWER  # noqa: E402
from sim_common import (HwProbe, TestShutdown, fake_xcore, loopback_group, read_json, sim_files,  # noqa: E402,F401
                        write_json, xcore_datagrams, xcore_driver)

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_")
STATE, FLAGS, LOG, _ = sim_files(WORK)
HOLD = os.path.join(WORK, "hold.json")
GROUP_PORT, BIND_IP = loopback_group()
LATCH, LIFT = HardwareStatus.REASON_LATCH, HardwareStatus.REASON_LIFT


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "20", "--state", STATE)
    driver = xcore_driver(BIND_IP, GROUP_PORT, allow_blade=True, hold_file=HOLD)
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    write_json(FLAGS, flags)


board = functools.partial(read_json, STATE)


received = functools.partial(xcore_datagrams, LOG)


def board_time():
    return board()["t"]


class Probe(HwProbe):
    emergency_reason = "xcore contract test"

    def __init__(self):
        super().__init__("xcore_contract_probe")
        self.odom = []
        self.imu = []
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 10)
        self.node.create_subscription(Imu, "imu/data", self._imu, qos_profile_sensor_data)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.charge = self.node.create_client(SetCharge, "hw/charge")
        self.start()

    def _odom(self, msg):
        with self.lock:
            self.odom = (self.odom + [msg])[-200:]

    def _imu(self, msg):
        with self.lock:
            self.imu = (self.imu + [msg])[-200:]


def healthy(p):
    return p.status is not None and p.status.telemetry_ok and not p.status.emergency


class TestXcoreContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        ready = cls.probe.wait_for(healthy, 20)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def release(self):
        self.probe.set_emergency(False)
        return self.probe.wait_for(lambda p: not p.status.emergency, 3)

    def test_a_board_configured_as_the_mower_always_was(self):
        b = board()
        self.assertEqual(b["meta_name"], "YardForce")
        self.assertEqual(b["gps"], [[460800], [1], [6]])
        self.assertEqual(b["imu_remap"], [1, -2, -3])
        self.assertEqual(b["diff_drive"], [[2025.0], [0.3556]])
        power = {k: round(v[0], 3) for k, v in b["power"].items() if v}
        # No termination current (7) and never the override of the charge current limit (10); the charge voltage at
        # 29.4 V (G29), below the 29.8 V of OpenMower.
        self.assertEqual(power, {"1": 24.5, "2": 28.2, "3": 23.0, "4": 29.9, "6": 29.4})
        self.assertIsNone(b["power_override"])
        inputs = json.loads(b["inputs"])
        self.assertEqual([i["line"] for i in inputs["gpio"]],
                         ["GPIO1", "GPIO5", "GPIO10", "GPIO11", "GPIO12", "GPIO13"])
        self.assertEqual(inputs["yf_cover_ui"], [{"id": "hall_mux", "value": "om"}])
        self.assertEqual(b["restarts"], 0)
        self.assertEqual(b["gps_restarts"], 0)
        self.assertEqual(b["rejected_configs"], 0)
        # Only the services the driver needs; file, sound and high level stay unclaimed.
        self.assertEqual(sorted(int(k) for k in b["claims"]), [1, 2, 3, 4, 5, 6, 7, 11])

    def test_b_capabilities_latched_with_firmware(self):
        caps = self.probe.wait_for(
            lambda p: p.capabilities if p.capabilities and p.capabilities.firmware[0].value else None)
        self.assertIsNotNone(caps, "no hw/capabilities with firmware")
        self.assertEqual(caps.platform, "yardforce_nx60i")
        self.assertEqual({kv.key: kv.value for kv in caps.firmware}, {"xcore": "v1.4.3"})
        self.assertTrue(caps.blade_speed_control)
        self.assertFalse(caps.blade_height_control)
        self.assertAlmostEqual(caps.blade_lease_max_s, 2.0)
        self.assertFalse(caps.host_charge_control)
        self.assertFalse(caps.charge_stop)
        self.assertTrue(caps.wheel_ticks)
        self.assertAlmostEqual(caps.wheel_ticks_per_m, 2025.0)
        self.assertAlmostEqual(caps.wheel_distance, 0.3556)
        self.assertTrue(caps.separate_bumpers)
        self.assertEqual(caps.imu_source, "xcore_driver")
        # max_duty 0.3 reaches 0.58 x (0.3 - 0.019) = 0.163 m/s per wheel; turning on the spot stays at its cap.
        self.assertAlmostEqual(caps.max_linear_speed, 0.16298, places=4)
        self.assertAlmostEqual(caps.max_angular_speed, 0.3, places=5)
        self.assertLess(caps.failsafe_without_host_s, 0)
        self.assertEqual(caps.safety_measurement_date, "")

    def test_c_status_imu_and_odometry(self):
        s = self.probe.status
        self.assertEqual(s.emergency_reason, 0)
        self.assertEqual(s.battery_source, HardwareStatus.BATTERY_SOURCE_ESTIMATE)
        self.assertAlmostEqual(s.battery_voltage, 26.5, places=3)
        self.assertEqual(s.battery_percentage, round((26.5 - 24.5) / (28.2 - 24.5) * 100))
        self.assertTrue(s.left_motor.ok and s.right_motor.ok and s.blade_motor.ok)
        self.assertEqual(s.left_motor.status, 0)
        self.assertFalse(s.dock_contact)
        self.assertGreater(s.link_messages_ok, 0)
        imu = self.probe.wait_for(lambda p: list(p.imu) if len(p.imu) > 50 else None)
        self.assertIsNotNone(imu, "no imu/data")
        # The remap turns the sensor that lies upside down: gravity along +z of the body.
        az = sum(m.linear_acceleration.z for m in imu) / len(imu)
        self.assertAlmostEqual(az, 9.80665, delta=0.05)
        self.assertLess(imu[-1].orientation_covariance[0], 0)
        self.assertTrue(self.probe.wait_for(lambda p: len(p.odom) > 10), "no hw/wheel_odom")

    def test_d_emergency_heartbeat_always_four_bytes(self):
        t0 = board_time()
        time.sleep(2.0)
        inputs = received(t0, EMERGENCY, DATA, 0)
        self.assertGreaterEqual(len(inputs), 6)
        self.assertTrue(all(e["len"] == 4 for e in inputs))
        gaps = [b["t"] - a["t"] for a, b in zip(inputs, inputs[1:])]
        self.assertLess(max(gaps), 0.4, gaps)
        self.assertEqual(board()["short_emergency_inputs"], 0)

    def test_e_drive_limited_and_three_zeros(self):
        t0 = board_time()
        x0 = board()["x"]
        for _ in range(10):
            self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.5)))
            time.sleep(0.1)
        moving_odom = self.probe.odom[-1].twist.linear.x
        time.sleep(1.2)
        twists = [(e["t"], struct.unpack("<6d", bytes.fromhex(e["hex"])))
                  for e in received(t0, DIFF_DRIVE, DATA, 0)]
        moving = [(t, v) for t, v in twists if v[0] != 0 or v[5] != 0]
        zeros = [t for t, v in twists if v[0] == 0 and v[5] == 0]
        self.assertGreaterEqual(len(moving), 8)
        # 0.5 m/s is cut to the 0.163 m/s that max_duty 0.3 reaches: duty 0.163 / 0.58 + 0.019 = 0.3.
        self.assertTrue(all(abs(v[0] - 0.3) < 1e-6 and abs(v[5]) < 1e-6 for _, v in moving), str(moving[:3]))
        self.assertEqual(len(zeros), 3, str(twists[-5:]))
        # The setpoint goes out every 0.1 s until cmd_vel has been silent for 0.5 s: commands for 1.0 s, then 0.5 s.
        self.assertAlmostEqual(zeros[0] - moving[0][0], 1.4, delta=0.2)
        self.assertLess(zeros[0] - moving[-1][0], 0.15)
        self.assertAlmostEqual(zeros[2] - zeros[0], 0.2, delta=0.06)
        # The fake's wheels follow the same model: the ticks show 0.163 m/s forward, also on the falling counter.
        self.assertAlmostEqual(moving_odom, 0.163, delta=0.02)
        self.assertGreater(board()["x"] - x0, 0.15)
        self.assertEqual(board()["duty"], [0.0, 0.0])

    def test_f_blade_lease_runs_out(self):
        t0 = board_time()
        r = self.probe.set_blade(True, lease_s=0.5, height=40)
        self.assertFalse(r.success)
        self.assertEqual(r.emergency_reason, 0)
        r = self.probe.set_blade(True, lease_s=0.5)
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.probe.wait_for(lambda p: p.status.blade_on, 2), "blade_on not reported")
        off = self.probe.wait_for(lambda p: not p.status.blade_on and p.status, 3)
        self.assertTrue(off, "blade did not go off")
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_LEASE_EXPIRED)
        time.sleep(0.5)
        speeds = [(e["t"], struct.unpack("<f", bytes.fromhex(e["hex"]))[0]) for e in received(t0, MOWER, DATA, 0)]
        on = [t for t, v in speeds if v != 0]
        self.assertEqual(len(on), 1, str(speeds))
        self.assertEqual([v for t, v in speeds if v != 0], [1.0])
        offs = [t for t, v in speeds if v == 0 and t > on[0]]
        self.assertGreaterEqual(len(offs), 3, str(speeds))
        self.assertAlmostEqual(offs[0] - on[0], 0.5, delta=0.1)
        self.assertTrue(self.probe.wait_for(lambda p: p.status.blade_motor.rpm == 0, 3))

    def test_g_host_latch_reaches_the_board(self):
        self.assertTrue(self.probe.set_emergency(True).success)
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status, 2)
        self.assertEqual(s.emergency_reason & (LATCH | HardwareStatus.REASON_HIGH_LEVEL),
                         LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        time.sleep(0.4)
        self.assertEqual(board()["host_bits"], LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        r = self.probe.set_blade(True, lease_s=1.0)
        self.assertFalse(r.success)
        self.assertIn("blocked", r.message)
        # The refusal names the emergency, so the logic need not wait for the status.
        self.assertEqual(r.emergency_reason & (LATCH | HardwareStatus.REASON_HIGH_LEVEL),
                         LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)
        time.sleep(0.4)
        self.assertEqual(board()["host_bits"], 0)

    def test_h_stop_latches_until_release(self):
        set_flags(stop=1)
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status.stop and p.status, 2)
        self.assertTrue(s, "stop gave no emergency")
        self.assertEqual(s.emergency_reason, LATCH | HardwareStatus.REASON_STOP)
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.stop, 2))
        time.sleep(0.5)
        # The board keeps only its latch, the driver the cause as well.
        self.assertEqual(board()["reason"], LATCH)
        self.assertEqual(self.probe.status.emergency_reason, LATCH | HardwareStatus.REASON_STOP)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)

    def test_i_lift_after_its_delay(self):
        set_flags(lift_left=1)
        self.assertTrue(self.probe.wait_for(lambda p: p.status.lift, 1), "lift flag not shown")
        start = time.time()
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status, 4)
        self.assertTrue(s, "lift gave no emergency")
        # 2500 ms in the configuration of the NX60i.
        self.assertGreater(time.time() - start, 2.0)
        self.assertEqual(s.emergency_reason, LATCH | LIFT)
        set_flags(bumper_left=1)
        s = self.probe.wait_for(lambda p: p.status.bumper_left and p.status.emergency_reason & 32 and p.status, 2)
        self.assertTrue(s, "bumper gave no collision")
        self.assertTrue(s.bumper and not s.bumper_right)
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.bumper, 2))
        self.assertTrue(self.release(), "release failed")

    def test_k_charging_is_left_to_the_board(self):
        for on in (False, True):
            r = self.probe.call(self.probe.charge, SetCharge.Request(on=on))
            self.assertFalse(r.success)
            self.assertEqual(r.message, "charging is controlled by the board")
        set_flags(dock=1)
        s = self.probe.wait_for(lambda p: p.status.dock_contact and p.status, 4)
        self.assertTrue(s, "no dock contact")
        self.assertLess(s.battery_current, 0)
        self.assertFalse(self.probe.set_blade(True, lease_s=1.0).success)
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.dock_contact_raw, 3))

    def test_ka_hold_is_no_latch_for_the_board(self):
        now = int(time.time())
        write_json(HOLD, {"until": now + 600, "set_at": now})
        s = self.probe.wait_for(lambda p: p.status.hold and p.status, 2)
        self.assertTrue(s, "hold.json not seen")
        self.assertFalse(s.emergency)
        self.assertEqual(s.emergency_reason, 0)
        t0 = board_time()
        for _ in range(10):
            self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.2)))
            time.sleep(0.1)
        time.sleep(0.3)
        # No latch for the board: the emergency input stays clear.
        sets = [struct.unpack("<HH", bytes.fromhex(e["hex"]))[0] for e in received(t0, EMERGENCY, DATA, 0)]
        self.assertTrue(sets and all(v == 0 for v in sets), str(sets))
        os.remove(HOLD)
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.hold, 2), "still held after the file went")

    def test_l_board_restart(self):
        port = board()["port"]
        self.assertTrue(self.probe.set_blade(True, lease_s=2.0).success)
        set_flags(restart=1)
        # The board falls silent: blocked, so the blade goes off before its lease ends.
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.blade_on and p.status.emergency, 2.5),
                        "blade stayed on")
        self.assertTrue(self.probe.wait_for(healthy, 15), "driver did not recover")
        b = board()
        self.assertNotEqual(b["port"], port)
        self.assertEqual(b["restarts"], 1)
        self.assertEqual(b["gps_restarts"], 0)
        self.assertEqual(b["meta_name"], "YardForce")
        self.assertTrue(all(n == 2 for n in b["claims"].values()), b["claims"])

    def test_m_incompatible_firmware_blocks(self):
        set_flags(restart=2, major_version=2)
        s = self.probe.wait_for(
            lambda p: p.status.emergency_reason & HardwareStatus.REASON_FIRMWARE_INCOMPATIBLE and p.status, 15)
        self.assertTrue(s, "incompatible firmware not reported")
        self.assertTrue(s.emergency)
        time.sleep(1.0)
        self.assertTrue(self.probe.status.emergency)
        set_flags(restart=3)
        self.assertTrue(self.probe.wait_for(healthy, 15), "driver did not recover with major version 1")
