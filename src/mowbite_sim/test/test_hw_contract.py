"""Duties of a platform driver behind the hardware contract hw/*, here tron_mcu against fake_tron on a pty:
capabilities, status, drive watchdog, blade lease, emergency with its reason and release.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_hw_contract.py
"""
import json
import os
import shutil
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
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency
from rclpy.qos import DurabilityPolicy, QoSProfile

WORK = tempfile.mkdtemp(prefix="mowbite_hw_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
CUTTER = 11
SPEED = 4


def generate_test_description():
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"],
        output="screen")
    driver = Node(package="tron_mcu", executable="tron_mcu_node", output="screen",
                  parameters=[{"port": LINKS["mcu"], "send_imu": False, "allow_blade": True, "blade_lease_max": 2.0,
                               "mow_height_mm": 50, "expected_cutter_version": "0.6.36",
                               "expected_chassis_version": "0.6.34"}])
    # The pty has to exist before the driver looks for it.
    return launch.LaunchDescription([
        fake_tron,
        TimerAction(period=1.0, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {}


def received(since, module=None):
    """Sub packets fake_tron got from the driver since sim time `since`."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if entry.get("type") == 0 and entry["t"] >= since and (module is None or entry["mod"] == module):
                out.append(entry)
    return out


def sim_time():
    with open(LOG) as f:
        lines = f.readlines()
    return json.loads(lines[-1])["t"] if lines else 0.0


def set_flags(**flags):
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.capabilities = None
        self.odom = 0
        self.node = rclpy.create_node("hw_contract_probe")
        # Reliable by default; transient local gets the capabilities published before the probe started.
        caps_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._capabilities, caps_qos)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 10)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
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

    def _capabilities(self, msg):
        with self.lock:
            self.capabilities = msg

    def _odom(self, _msg):
        with self.lock:
            self.odom += 1

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

    def set_emergency(self, latch):
        return self.call(self.emergency, SetEmergency.Request(latch=latch, reason="hw contract test"))

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


class TestHardwareContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        # Free once telemetry and a matching version frame arrived.
        ready = cls.probe.wait(lambda p: p.status is not None and p.status.telemetry_ok and not p.status.emergency, 15)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def release(self):
        self.probe.set_emergency(False)
        return self.probe.wait(lambda p: not p.status.emergency, 3)

    def test_a_capabilities_latched_with_firmware(self):
        caps = self.probe.wait(
            lambda p: p.capabilities if p.capabilities and p.capabilities.firmware[0].value else None)
        self.assertIsNotNone(caps, "no hw/capabilities with firmware")
        self.assertEqual(caps.platform, "airseekers_tron")
        firmware = {kv.key: kv.value for kv in caps.firmware}
        self.assertEqual(firmware["cutter"], "0.6.36")
        self.assertEqual(firmware["chassis"], "0.6.34")
        self.assertTrue(caps.blade_height_control)
        self.assertEqual((caps.blade_height_min_mm, caps.blade_height_max_mm), (30, 90))
        self.assertAlmostEqual(caps.blade_lease_max_s, 2.0)
        self.assertFalse(caps.host_charge_control)
        self.assertTrue(caps.charge_stop)
        self.assertAlmostEqual(caps.wheel_ticks_per_m, 1000.0)
        self.assertAlmostEqual(caps.wheel_distance, 0.40)
        self.assertAlmostEqual(caps.max_linear_speed, 0.3, places=5)
        self.assertLess(caps.failsafe_without_host_s, 0)
        self.assertEqual(caps.safety_measurement_date, "")

    def test_b_status_healthy(self):
        s = self.probe.status
        self.assertTrue(s.telemetry_ok)
        self.assertFalse(s.emergency)
        self.assertEqual(s.emergency_reason, 0)
        self.assertEqual(s.battery_source, HardwareStatus.BATTERY_SOURCE_BMS)
        self.assertAlmostEqual(s.battery_voltage, 24.5, places=3)
        self.assertTrue(s.blade_motor.ok)
        self.assertGreater(s.link_messages_ok, 0)
        self.assertTrue(self.probe.wait(lambda p: p.odom > 10), "no hw/wheel_odom")

    def test_c_drive_watchdog_three_zeros(self):
        t0 = sim_time()
        for _ in range(5):
            self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.5)))
            time.sleep(0.1)
        time.sleep(1.2)
        speeds = [(e["t"], e["speed"]) for e in received(t0, SPEED)]
        moving = [t for t, v in speeds if v != [0.0, 0.0]]
        zeros = [t for t, v in speeds if v == [0.0, 0.0]]
        self.assertGreaterEqual(len(moving), 4)
        self.assertTrue(all(v[0] == 0.3 for t, v in speeds if v != [0.0, 0.0]), "not clamped to 0.3")
        self.assertEqual(len(zeros), 3, str(speeds))
        self.assertAlmostEqual(zeros[0] - moving[-1], 0.5, delta=0.1)
        self.assertAlmostEqual(zeros[2] - zeros[0], 0.2, delta=0.05)

    def test_d_blade_lease_runs_out(self):
        t0 = sim_time()
        r = self.probe.set_blade(True, lease_s=0.5)
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.probe.wait(lambda p: p.status.blade_on, 2), "blade_on not reported")
        off = self.probe.wait(lambda p: not p.status.blade_on and p.status, 3)
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
        off = self.probe.wait(lambda p: not p.status.blade_on and p.status, 2)
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_REQUEST)
        time.sleep(0.4)
        frames = received(t0, CUTTER)
        self.assertEqual(len([e for e in frames if e["hex"][:2] == "01"]), 1, str(frames))
        self.assertEqual(len([e for e in frames if e["hex"][:2] == "00"]), 3, str(frames))

    def test_f_blade_requests_out_of_range_refused(self):
        r = self.probe.set_blade(True, lease_s=2.5)
        self.assertFalse(r.success)
        self.assertEqual(r.emergency_reason, 0)
        self.assertFalse(self.probe.set_blade(True, lease_s=0.0).success)
        self.assertFalse(self.probe.set_blade(True, lease_s=1.0, height=20).success)
        self.assertFalse(self.probe.set_blade(True, lease_s=1.0, speed=150).success)
        self.assertFalse(self.probe.status.blade_on)

    def test_g_host_emergency_blocks_blade(self):
        self.assertTrue(self.probe.set_emergency(True).success)
        s = self.probe.wait(lambda p: p.status.emergency and p.status, 2)
        self.assertTrue(s, "no emergency")
        self.assertEqual(s.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        r = self.probe.set_blade(True, lease_s=1.0)
        self.assertFalse(r.success)
        self.assertIn("blocked", r.message)
        # The refusal names the emergency, so the logic need not wait for the status.
        self.assertEqual(r.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_HIGH_LEVEL)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)

    def test_h_lift_reason_stays_until_release(self):
        set_flags(lift=1)
        s = self.probe.wait(lambda p: p.status.emergency and p.status, 2)
        self.assertTrue(s, "lift gave no emergency")
        self.assertEqual(s.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_LIFT)
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: not p.status.lift, 2))
        time.sleep(0.3)
        s = self.probe.status
        self.assertTrue(s.emergency)
        self.assertEqual(s.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_LIFT)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)

    def test_i_host_latches_with_its_own_bits_only(self):
        # The bits of the safety filter stay until the release; a node above the driver cannot fake a lift.
        tilt = SetEmergency.Request(latch=True, bits=HardwareStatus.REASON_TILT, reason="hw contract test")
        self.assertTrue(self.probe.call(self.probe.emergency, tilt).success)
        fence = SetEmergency.Request(latch=True, bits=HardwareStatus.REASON_FENCE, reason="hw contract test")
        self.assertTrue(self.probe.call(self.probe.emergency, fence).success)
        both = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_TILT | HardwareStatus.REASON_FENCE
        self.assertTrue(self.probe.wait(lambda p: p.status.emergency_reason == both, 2), "bits not shown")
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)
        lift = SetEmergency.Request(latch=True, bits=HardwareStatus.REASON_LIFT, reason="hw contract test")
        answer = self.probe.call(self.probe.emergency, lift)
        self.assertFalse(answer.success)
        self.assertIn("only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART", answer.message)
        time.sleep(0.3)
        self.assertFalse(self.probe.status.emergency)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
