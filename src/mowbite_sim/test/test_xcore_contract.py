"""Duties of a platform driver behind the hardware contract hw/*, here xcore_driver against fake_xcore on UDP over the
loopback: configuration of the board, capabilities, status, IMU, drive watchdog, blade lease, emergency with its reason
and release, charging left to the board, a restart of the board and incompatible firmware.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_contract.py
"""
import json
import os
import random
import shutil
import struct
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetCharge, SetEmergency
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_")
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
STATE = os.path.join(WORK, "state.json")
# A port of its own, so that tests running side by side do not hear each other's board.
GROUP_PORT = random.randint(42000, 48000)
# An address of its own for the driver: it does not start beside another host at its bind_ip.
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
EMERGENCY, DIFF_DRIVE, MOWER = 1, 2, 3
DATA = 1
LATCH, LIFT, LIFT_MULTIPLE = HardwareStatus.REASON_LATCH, HardwareStatus.REASON_LIFT, 16


def generate_test_description():
    sim = FindPackageShare("mowbite_sim").find("mowbite_sim")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim, "config", "xcore_nx60i_services.json"), "--group-port", str(GROUP_PORT),
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "20", "--state", STATE],
        output="screen")
    config = os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config", "nx60i.yaml")
    driver = Node(package="xcore_driver", executable="xcore_driver_node", output="screen",
                  parameters=[config, {"interface": "lo", "bind_ip": BIND_IP, "announcement_port": GROUP_PORT,
                                       "allow_blade": True}])
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


def board():
    with open(STATE) as f:
        return json.load(f)


def received(since, sid=None, msg_type=None, arg2=None):
    """Datagrams fake_xcore got from the driver since board time `since`."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if "sid" not in entry or entry["t"] < since:
                continue
            if sid is not None and entry["sid"] != sid:
                continue
            if msg_type is not None and entry["type"] != msg_type:
                continue
            if arg2 is not None and entry["arg2"] != arg2:
                continue
            out.append(entry)
    return out


def board_time():
    return board()["t"]


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.capabilities = None
        self.odom = []
        self.imu = []
        self.node = rclpy.create_node("xcore_contract_probe")
        caps_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._capabilities, caps_qos)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 10)
        self.node.create_subscription(Imu, "imu/data", self._imu, qos_profile_sensor_data)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.charge = self.node.create_client(SetCharge, "hw/charge")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _capabilities(self, msg):
        with self.lock:
            self.capabilities = msg

    def _odom(self, msg):
        with self.lock:
            self.odom = (self.odom + [msg])[-200:]

    def _imu(self, msg):
        with self.lock:
            self.imu = (self.imu + [msg])[-200:]

    def wait(self, pred, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = pred(self)
            if value:
                return value
            time.sleep(0.02)
        return None

    def call(self, client, request, timeout=5.0):
        assert client.wait_for_service(timeout_sec=timeout), "service missing"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.01)
        return future.result()

    def set_blade(self, on, lease_s=0.0, speed=0, height=0):
        return self.call(self.blade, SetBlade.Request(on=on, lease_s=lease_s, speed_percent=speed, height_mm=height))

    def set_emergency(self, latch, bits=0):
        return self.call(self.emergency, SetEmergency.Request(latch=latch, bits=bits, reason="xcore contract test"))

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


def healthy(p):
    return p.status is not None and p.status.telemetry_ok and not p.status.emergency


class TestXcoreContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        ready = cls.probe.wait(healthy, 20)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def release(self):
        self.probe.set_emergency(False)
        return self.probe.wait(lambda p: not p.status.emergency, 3)

    def test_a_board_configured_as_the_mower_always_was(self):
        b = board()
        self.assertEqual(b["meta_name"], "YardForce")
        self.assertEqual(b["gps"], [[460800], [1], [6]])
        self.assertEqual(b["imu_remap"], [1, -2, -3])
        self.assertEqual(b["diff_drive"], [[2025.0], [0.3556]])
        power = {k: round(v[0], 3) for k, v in b["power"].items() if v}
        # No termination current (7) and never the override of the charge current limit (10).
        self.assertEqual(power, {"1": 24.5, "2": 28.2, "3": 23.0, "4": 29.9, "6": 29.8})
        self.assertIsNone(b["power_override"])
        inputs = json.loads(b["inputs"])
        self.assertEqual([i["line"] for i in inputs["gpio"]], ["GPIO1", "GPIO5", "GPIO10", "GPIO11", "GPIO12", "GPIO13"])
        self.assertEqual(inputs["yf_cover_ui"], [{"id": "hall_mux", "value": "om"}])
        self.assertEqual(b["restarts"], 0)
        self.assertEqual(b["gps_restarts"], 0)
        self.assertEqual(b["rejected_configs"], 0)
        # Only the services the driver needs; file, sound and high level stay unclaimed.
        self.assertEqual(sorted(int(k) for k in b["claims"]), [1, 2, 3, 4, 5, 6, 7, 11])

    def test_b_capabilities_latched_with_firmware(self):
        caps = self.probe.wait(
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
        self.assertTrue(caps.separate_bumpers)
        self.assertEqual(caps.imu_source, "xcore_driver")
        self.assertAlmostEqual(caps.max_linear_speed, 0.3, places=5)
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
        imu = self.probe.wait(lambda p: list(p.imu) if len(p.imu) > 50 else None)
        self.assertIsNotNone(imu, "no imu/data")
        # The remap turns the sensor that lies upside down: gravity along +z of the body.
        az = sum(m.linear_acceleration.z for m in imu) / len(imu)
        self.assertAlmostEqual(az, 9.80665, delta=0.05)
        self.assertLess(imu[-1].orientation_covariance[0], 0)
        self.assertTrue(self.probe.wait(lambda p: len(p.odom) > 10), "no hw/wheel_odom")

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
        # 0.3 m/s at full_duty_speed 1.0 is a duty of 0.3, the most the driver gives.
        self.assertTrue(all(abs(v[0] - 0.3) < 1e-6 and abs(v[5]) < 1e-6 for _, v in moving), str(moving[:3]))
        self.assertEqual(len(zeros), 3, str(twists[-5:]))
        # The setpoint goes out every 0.1 s until cmd_vel has been silent for 0.5 s: commands for 1.0 s, then 0.5 s.
        self.assertAlmostEqual(zeros[0] - moving[0][0], 1.4, delta=0.2)
        self.assertLess(zeros[0] - moving[-1][0], 0.15)
        self.assertAlmostEqual(zeros[2] - zeros[0], 0.2, delta=0.06)
        # The fake's wheels reach 0.9 m/s at full duty: the ticks show 0.27 m/s forward, also on the falling counter.
        self.assertAlmostEqual(moving_odom, 0.27, delta=0.03)
        self.assertGreater(board()["x"] - x0, 0.2)
        self.assertEqual(board()["duty"], [0.0, 0.0])

    def test_f_blade_lease_runs_out(self):
        t0 = board_time()
        self.assertFalse(self.probe.set_blade(True, lease_s=0.5, height=40).success)
        r = self.probe.set_blade(True, lease_s=0.5)
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.probe.wait(lambda p: p.status.blade_on, 2), "blade_on not reported")
        off = self.probe.wait(lambda p: not p.status.blade_on and p.status, 3)
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
        self.assertTrue(self.probe.wait(lambda p: p.status.blade_motor.rpm == 0, 3))

    def test_g_host_latch_reaches_the_board(self):
        self.assertTrue(self.probe.set_emergency(True).success)
        s = self.probe.wait(lambda p: p.status.emergency and p.status, 2)
        self.assertEqual(s.emergency_reason & (LATCH | HardwareStatus.REASON_HIGH_LEVEL),
                         LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        time.sleep(0.4)
        self.assertEqual(board()["host_bits"], LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        r = self.probe.set_blade(True, lease_s=1.0)
        self.assertFalse(r.success)
        self.assertIn("blocked", r.message)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)
        time.sleep(0.4)
        self.assertEqual(board()["host_bits"], 0)

    def test_h_stop_latches_until_release(self):
        set_flags(stop=1)
        s = self.probe.wait(lambda p: p.status.emergency and p.status.stop and p.status, 2)
        self.assertTrue(s, "stop gave no emergency")
        self.assertEqual(s.emergency_reason, LATCH | HardwareStatus.REASON_STOP)
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: not p.status.stop, 2))
        time.sleep(0.5)
        # The board keeps only its latch, the driver the cause as well.
        self.assertEqual(board()["reason"], LATCH)
        self.assertEqual(self.probe.status.emergency_reason, LATCH | HardwareStatus.REASON_STOP)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)

    def test_i_lift_after_its_delay(self):
        set_flags(lift_left=1)
        self.assertTrue(self.probe.wait(lambda p: p.status.lift, 1), "lift flag not shown")
        start = time.time()
        s = self.probe.wait(lambda p: p.status.emergency and p.status, 4)
        self.assertTrue(s, "lift gave no emergency")
        # 2500 ms in the configuration of the NX60i.
        self.assertGreater(time.time() - start, 2.0)
        self.assertEqual(s.emergency_reason, LATCH | LIFT)
        set_flags(bumper_left=1)
        s = self.probe.wait(lambda p: p.status.bumper_left and p.status.emergency_reason & 32 and p.status, 2)
        self.assertTrue(s, "bumper gave no collision")
        self.assertTrue(s.bumper and not s.bumper_right)
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: not p.status.bumper, 2))
        self.assertTrue(self.release(), "release failed")

    def test_j_host_latches_with_its_own_bits_only(self):
        tilt = SetEmergency.Request(latch=True, bits=HardwareStatus.REASON_TILT, reason="xcore contract test")
        self.assertTrue(self.probe.call(self.probe.emergency, tilt).success)
        self.assertTrue(self.probe.wait(lambda p: p.status.emergency_reason & HardwareStatus.REASON_TILT, 2))
        self.assertTrue(self.release(), "release failed")
        answer = self.probe.set_emergency(True, HardwareStatus.REASON_LIFT)
        self.assertFalse(answer.success)
        self.assertIn("only HIGH_LEVEL, TILT, FENCE and STUCK", answer.message)
        time.sleep(0.3)
        self.assertFalse(self.probe.status.emergency)

    def test_k_charging_is_left_to_the_board(self):
        for on in (False, True):
            r = self.probe.call(self.probe.charge, SetCharge.Request(on=on))
            self.assertFalse(r.success)
            self.assertEqual(r.message, "charging is controlled by the board")
        set_flags(dock=1)
        s = self.probe.wait(lambda p: p.status.dock_contact and p.status, 4)
        self.assertTrue(s, "no dock contact")
        self.assertLess(s.battery_current, 0)
        self.assertFalse(self.probe.set_blade(True, lease_s=1.0).success)
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: not p.status.dock_contact_raw, 3))

    def test_l_board_restart(self):
        port = board()["port"]
        self.assertTrue(self.probe.set_blade(True, lease_s=2.0).success)
        set_flags(restart=1)
        # The board falls silent: blocked, so the blade goes off before its lease ends.
        self.assertTrue(self.probe.wait(lambda p: not p.status.blade_on and p.status.emergency, 2.5), "blade stayed on")
        self.assertTrue(self.probe.wait(healthy, 15), "driver did not recover")
        b = board()
        self.assertNotEqual(b["port"], port)
        self.assertEqual(b["restarts"], 1)
        self.assertEqual(b["gps_restarts"], 0)
        self.assertEqual(b["meta_name"], "YardForce")
        self.assertTrue(all(n == 2 for n in b["claims"].values()), b["claims"])

    def test_m_incompatible_firmware_blocks(self):
        set_flags(restart=2, major_version=2)
        s = self.probe.wait(
            lambda p: p.status.emergency_reason & HardwareStatus.REASON_FIRMWARE_INCOMPATIBLE and p.status, 15)
        self.assertTrue(s, "incompatible firmware not reported")
        self.assertTrue(s.emergency)
        time.sleep(1.0)
        self.assertTrue(self.probe.status.emergency)
        set_flags(restart=3)
        self.assertTrue(self.probe.wait(healthy, 15), "driver did not recover with major version 1")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
