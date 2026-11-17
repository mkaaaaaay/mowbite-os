"""Duties of every platform driver behind the hardware contract hw/*: capabilities, status, the limit and the watchdog
of the drive, the blade lease, the emergency with its reason and release, the hold. One case per platform
(contract_<platform>.py) brings the fake board, the start of the driver and a few grips at the board; the checks speak
only hw/*. What only one board has (frames, configuration, restarts, firmware) stays in the test of its driver.

A case is the class Case of its file:
  Case(work, hold_file)   files of the run go to work; the driver is started with hold_file as its hold file
  driver_delay            s between the start of the board and the start of the driver
  cmd_vel_timeout         s without hw/cmd_vel after which the driver of this platform stops the wheels, as shipped
  board(), driver()       the launch actions of the fake board and of the driver with the values of the platform and
                          the blade allowed
  inputs(stop, lift)      sets these inputs of the board, all others off
  time()                  the time of the board, the clock of the two lists below
  drive(since)            (t, linear m/s, angular rad/s) of every drive command the board got since then
  blade(since)            (t, running) of every blade command the board got since then

Run in a sourced workspace: MOWBITE_PLATFORM=<platform> launch_test /ws/src/mowbite_sim/test/test_driver_contract.py
"""
import importlib
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from launch.actions import TimerAction
from mowbite_msgs.msg import HardwareCapabilities, HardwareInputs, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, DriverProbe, TestShutdown, layers, write_json  # noqa: E402,F401

PLATFORM = os.environ["MOWBITE_PLATFORM"]
WORK = tempfile.mkdtemp(prefix="mowbite_contract_")
HOLD = os.path.join(WORK, "hold.json")
CASE = importlib.import_module(f"contract_{PLATFORM}").Case(WORK, HOLD)
LATCH = HardwareStatus.REASON_LATCH
HOST = LATCH | HardwareStatus.REASON_HIGH_LEVEL
CMD_VEL_TIMEOUT = CASE.cmd_vel_timeout
# The longest time without hw/cmd_vel a driver may keep the wheels turning.
CMD_VEL_TIMEOUT_MAX = 0.5


def generate_test_description():
    CASE.inputs()
    return launch.LaunchDescription([
        CASE.board(),
        TimerAction(period=CASE.driver_delay, actions=[CASE.driver(), launch_testing.actions.ReadyToTest()]),
    ]), {}


def moving(commands):
    return [c for c in commands if c[1] or c[2]]


def blade_on(commands):
    return [t for t, running in commands if running]


class Probe(DriverProbe):
    emergency_reason = "driver contract test"

    def __init__(self):
        super().__init__("driver_contract_probe")
        self.status = None
        self.capabilities = None
        self.inputs = None
        self.odom = 0
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("capabilities"), LATCHED)
        self.node.create_subscription(HardwareInputs, "hw/inputs", self._set("inputs"), LATCHED)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 10)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def _odom(self, _msg):
        with self.lock:
            self.odom += 1

    def drive(self, linear, seconds):
        """cmd_vel for that long; the time of the board at the last one."""
        self.cmd_vel_for(linear, seconds)
        self.cmd_vel.publish(Twist(linear=Vector3(x=linear)))
        return CASE.time()


class TestDriverContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        ready = cls.probe.wait_for(
            lambda p: p.status is not None and p.status.telemetry_ok and not p.status.emergency, 20)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def release(self):
        self.probe.set_emergency(False)
        return self.probe.wait_for(lambda p: not p.status.emergency, 3)

    def nothing_went_out(self, since):
        self.assertEqual(moving(CASE.drive(since)), [])
        self.assertEqual(blade_on(CASE.blade(since)), [])

    def latched_until_release(self, flag, reason, timeout):
        """The input gives the emergency with its reason, which stays after the input went until the release."""
        p = self.probe
        CASE.inputs(**{flag: True})
        s = p.wait_for(lambda p: p.status.emergency and getattr(p.status, flag) and p.status, timeout)
        self.assertTrue(s, f"{flag} gave no emergency")
        self.assertEqual(s.emergency_reason, LATCH | reason)
        CASE.inputs()
        self.assertTrue(p.wait_for(lambda p: not getattr(p.status, flag), 2), f"{flag} stays shown")
        time.sleep(0.5)
        self.assertTrue(p.status.emergency)
        self.assertEqual(p.status.emergency_reason, LATCH | reason)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(p.status.emergency_reason, 0)

    def test_a_capabilities_latched_and_complete(self):
        # The driver publishes them again once a board named its firmware; one that did not keeps an empty value.
        caps = self.probe.wait_for(lambda p: p.capabilities if p.capabilities
                                   and any(kv.value for kv in p.capabilities.firmware) else None)
        self.assertIsNotNone(caps, "no hw/capabilities with firmware")
        self.assertTrue(all(kv.key for kv in caps.firmware), caps.firmware)
        geometry = layers.parameters([layers.geometry(PLATFORM)], "mowbite_logic")
        self.assertEqual(caps.platform, geometry["geometry.platform"])
        self.assertGreater(caps.max_linear_speed, 0)
        self.assertGreater(caps.max_angular_speed, 0)
        self.assertGreater(caps.wheel_distance, 0)
        self.assertGreater(caps.blade_lease_max_s, 0)
        heights = (caps.blade_height_min_mm, caps.blade_height_default_mm, caps.blade_height_max_mm)
        if caps.blade_height_control:
            self.assertTrue(0 < heights[0] <= heights[1] <= heights[2], heights)
        else:
            self.assertEqual(heights, (0, 0, 0))
        # Without the release at the mower all three stay 0.
        free, hold, max_hold = caps.stop_release_free_s, caps.stop_release_hold_s, caps.stop_release_max_hold_s
        self.assertTrue((free, hold, max_hold) == (0, 0, 0) or (free > 0 and 0 < hold < max_hold),
                        (free, hold, max_hold))
        # Without a safety measurement neither its date nor its time.
        self.assertEqual(caps.failsafe_without_host_s > 0, caps.safety_measurement_date != "")

    def test_b_status_healthy(self):
        # The battery may come after the first telemetry.
        s = self.probe.wait_for(lambda p: p.status.battery_source != HardwareStatus.BATTERY_SOURCE_NONE and p.status, 5)
        self.assertTrue(s, self.probe.status)
        self.assertTrue(s.telemetry_ok)
        self.assertFalse(s.emergency or s.hold)
        self.assertEqual(s.emergency_reason, 0)
        self.assertGreater(s.battery_voltage, 0)
        self.assertTrue(s.blade_motor.ok)
        self.assertFalse(s.blade_on)
        self.assertGreater(s.link_messages_ok, 0)
        self.assertTrue(self.probe.wait_for(lambda p: p.odom > 10), "no hw/wheel_odom")

    def test_c_drive_limited_and_three_zeros(self):
        p = self.probe
        limit = p.capabilities.max_linear_speed
        self.assertLessEqual(CMD_VEL_TIMEOUT, CMD_VEL_TIMEOUT_MAX)
        t0 = CASE.time()
        last = p.drive(2 * limit, 1.0)
        time.sleep(CMD_VEL_TIMEOUT + 1.0)
        commands = CASE.drive(t0)
        speeds = moving(commands)
        zeros = [t for t, v, w in commands if not v and not w]
        self.assertGreaterEqual(len(speeds), 8, commands)
        self.assertTrue(all(abs(v - limit) < 1e-3 and abs(w) < 1e-3 for _, v, w in speeds), speeds[:3])
        # Three stops 100 ms apart once cmd_vel has been silent for its time, then nothing.
        self.assertEqual(len(zeros), 3, commands[-6:])
        self.assertGreater(zeros[0], speeds[-1][0])
        self.assertAlmostEqual(zeros[0] - last, CMD_VEL_TIMEOUT, delta=0.2)
        self.assertAlmostEqual(zeros[2] - zeros[0], 0.2, delta=0.06)

    def test_d_blade_lease_runs_out(self):
        p = self.probe
        t0 = CASE.time()
        r = p.set_blade(True, lease_s=0.5)
        self.assertTrue(r.success, r.message)
        self.assertTrue(p.wait_for(lambda p: p.status.blade_on, 2), "blade_on not reported")
        off = p.wait_for(lambda p: not p.status.blade_on and p.status, 3)
        self.assertTrue(off, "blade did not go off")
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_LEASE_EXPIRED)
        time.sleep(0.5)
        commands = CASE.blade(t0)
        on = blade_on(commands)
        self.assertEqual(len(on), 1, commands)
        offs = [t for t, running in commands if not running and t > on[0]]
        self.assertGreaterEqual(len(offs), 3, commands)
        self.assertAlmostEqual(offs[0] - on[0], 0.5, delta=0.1)

    def test_e_blade_renewed_stays_on(self):
        p = self.probe
        t0 = CASE.time()
        for _ in range(8):
            r = p.set_blade(True, lease_s=0.6)
            self.assertTrue(r.success, r.message)
            time.sleep(0.25)
        self.assertTrue(p.status.blade_on)
        asked = CASE.time()
        self.assertTrue(p.set_blade(False).success)
        off = p.wait_for(lambda p: not p.status.blade_on and p.status, 2)
        self.assertTrue(off, "blade did not go off")
        self.assertEqual(off.blade_off_reason, HardwareStatus.BLADE_OFF_REQUEST)
        commands = CASE.blade(t0)
        on = blade_on(commands)
        self.assertTrue(on, commands)
        self.assertEqual([t for t, running in commands if not running and on[0] < t < asked], [], commands)

    def test_f_blade_requests_out_of_range_refused(self):
        p = self.probe
        caps = p.capabilities
        r = p.set_blade(True, lease_s=caps.blade_lease_max_s + 0.5)
        self.assertFalse(r.success)
        self.assertEqual(r.emergency_reason, 0)
        self.assertFalse(p.set_blade(True, lease_s=0.0).success)
        # Without blade_height_control any height is refused.
        heights = (caps.blade_height_min_mm - 10, caps.blade_height_max_mm + 10) if caps.blade_height_control else (40,)
        for height in heights:
            self.assertFalse(p.set_blade(True, lease_s=1.0, height=height).success, height)
        self.assertFalse(p.set_blade(True, lease_s=1.0, speed=150).success)
        time.sleep(0.3)
        self.assertFalse(p.status.blade_on)

    def test_g_host_emergency_blocks_blade_and_drive(self):
        p = self.probe
        self.assertTrue(p.set_emergency(True).success)
        s = p.wait_for(lambda p: p.status.emergency and p.status, 2)
        self.assertTrue(s, "no emergency")
        self.assertEqual(s.emergency_reason, HOST)
        t0 = CASE.time()
        r = p.set_blade(True, lease_s=1.0)
        self.assertFalse(r.success)
        self.assertIn("blocked", r.message)
        # The refusal names the emergency, so the logic need not wait for the status.
        self.assertEqual(r.emergency_reason, HOST)
        p.drive(p.capabilities.max_linear_speed / 2, 0.5)
        time.sleep(0.3)
        self.nothing_went_out(t0)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(p.status.emergency_reason, 0)

    def test_h_stop_stays_until_release(self):
        self.latched_until_release("stop", HardwareStatus.REASON_STOP, 2)

    def test_i_lift_stays_until_release(self):
        # A board may wait before raised wheels count as a lift.
        self.latched_until_release("lift", HardwareStatus.REASON_LIFT, 5)

    def test_ia_release_with_an_old_count_is_refused(self):
        # A driver that counts its latches refuses a release that carries the count of before another reason.
        p = self.probe
        CASE.inputs(stop=True)
        s = p.wait_for(lambda p: p.status.emergency and p.status.stop and p.status, 2)
        self.assertTrue(s, "stop gave no emergency")
        CASE.inputs()
        self.assertTrue(p.wait_for(lambda p: not p.status.stop, 2), "stop stays shown")
        count = p.status.latch_count
        if count == 0:
            self.assertTrue(self.release(), "release failed")
            self.skipTest("the driver does not count its latches")
        self.assertTrue(p.set_emergency(True, HardwareStatus.REASON_TILT).success)
        self.assertTrue(p.wait_for(lambda p: p.status.latch_count != count, 2), "the count stays")
        answer = p.set_emergency(False, latch_count=count)
        self.assertFalse(answer.success)
        self.assertEqual(answer.code, "LATCH_CHANGED")
        time.sleep(0.3)
        self.assertEqual(p.status.emergency_reason, LATCH | HardwareStatus.REASON_STOP | HardwareStatus.REASON_TILT)
        self.assertTrue(p.set_emergency(False, latch_count=p.status.latch_count).success)
        self.assertTrue(p.wait_for(lambda p: not p.status.emergency, 3), "release failed")

    def test_ib_inputs_one_by_one(self):
        # hw/inputs, where the driver has it: every input of the table, the stop ones switched with the button.
        p = self.probe
        if not p.wait_for(lambda p: p.inputs is not None, 2):
            self.skipTest("the driver has no hw/inputs")
        self.assertFalse(p.inputs.learning)
        roles = [i.role for i in p.inputs.inputs]
        for role in ("stop", "lift"):
            self.assertIn(role, roles)
        self.assertTrue(all(i.assigned and not i.active for i in p.inputs.inputs), p.inputs.inputs)
        CASE.inputs(stop=True)
        switched = p.wait_for(lambda p: [i.name for i in p.inputs.inputs if i.active], 2)
        CASE.inputs()
        self.assertTrue(switched, "no input shows the stop button")
        by_name = {i.name: i for i in p.inputs.inputs}
        self.assertTrue(all(by_name[name].role == "stop" for name in switched), switched)
        self.assertTrue(p.wait_for(lambda p: not p.status.stop, 2), "stop stays shown")
        self.assertTrue(self.release(), "release failed")
        self.assertTrue(p.wait_for(lambda p: not any(i.active for i in p.inputs.inputs), 2), "an input stays switched")

    def test_j_host_latches_with_its_own_bits_only(self):
        # The bits of the safety filter stay until the release; a node above the driver cannot fake a lift.
        p = self.probe
        for bit in (HardwareStatus.REASON_TILT, HardwareStatus.REASON_FENCE):
            self.assertTrue(p.set_emergency(True, bit).success)
        both = LATCH | HardwareStatus.REASON_TILT | HardwareStatus.REASON_FENCE
        self.assertTrue(p.wait_for(lambda p: p.status.emergency_reason == both, 2), p.status.emergency_reason)
        self.assertTrue(self.release(), "release failed")
        self.assertEqual(p.status.emergency_reason, 0)
        answer = p.set_emergency(True, HardwareStatus.REASON_LIFT)
        self.assertFalse(answer.success)
        self.assertIn("only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART", answer.message)
        time.sleep(0.3)
        self.assertFalse(p.status.emergency)

    def test_k_hold_blocks_without_emergency(self):
        p = self.probe
        speed = p.capabilities.max_linear_speed / 2
        now = int(time.time())
        write_json(HOLD, {"until": now + 600, "set_at": now})
        s = p.wait_for(lambda p: p.status.hold and p.status, 2)
        self.assertTrue(s, "hold.json not seen")
        self.assertFalse(s.emergency)
        self.assertEqual(s.emergency_reason, 0)
        t0 = CASE.time()
        p.drive(speed, 1.0)
        r = p.set_blade(True, lease_s=1.0)
        self.assertFalse(r.success)
        self.assertIn("held", r.message)
        self.assertEqual(r.emergency_reason, 0)
        time.sleep(0.3)
        self.nothing_went_out(t0)
        os.remove(HOLD)
        self.assertTrue(p.wait_for(lambda p: not p.status.hold, 2), "still held after the file went")
        t1 = CASE.time()
        p.drive(speed, 0.5)
        self.assertTrue(p.wait(lambda: len(moving(CASE.drive(t1))) > 2, CMD_VEL_TIMEOUT + 0.3), CASE.drive(t1))
