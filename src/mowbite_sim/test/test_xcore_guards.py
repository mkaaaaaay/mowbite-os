"""Checks of xcore_driver itself against fake_xcore on UDP over the loopback: no start beside another host's socket,
tilt from the board's IMU, wheels turning without or against the command, another host claiming the services, and a
driver frozen while wheels and blade run (the board stores setpoints sent during its emergency and runs them once the
reason clears, so nothing but a latch may go out first).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_guards.py
"""
import functools
import os
import shutil
import signal
import struct
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
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import XCORE_CLAIM as CLAIM, XCORE_DATA as DATA, XCORE_DIFF_DRIVE as DIFF_DRIVE  # noqa: E402
from sim_common import XCORE_EMERGENCY as EMERGENCY, XCORE_MOWER as MOWER  # noqa: E402
from sim_common import (DriverProbe, SimProbe, TestShutdown, fake_xcore, loopback_group, read_json,  # noqa: E402,F401
                        sim_files, write_json, xcore_datagrams, xcore_driver)

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_guards_")
STATE, FLAGS, LOG, _ = sim_files(WORK)
GROUP_PORT, BIND_IP = loopback_group()
# Another program holds a socket at the driver's address for this long after the launch.
SQUATTER_S = 4.0
LATCH = HardwareStatus.REASON_LATCH
HIGH_LEVEL = HardwareStatus.REASON_HIGH_LEVEL
TILT = HardwareStatus.REASON_TILT
WHEEL_RUNAWAY = HardwareStatus.REASON_WHEEL_RUNAWAY
FOREIGN_HOST = HardwareStatus.REASON_FOREIGN_HOST
DRIVER_STALL = HardwareStatus.REASON_DRIVER_STALL
TIMEOUT_HIGH_LEVEL, SERVICE_NOT_READY = 64, 256


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "40", "--state", STATE)
    squatter = ExecuteProcess(
        cmd=["python3", "-c", "import socket, sys, time; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); "
             f"s.bind(('{BIND_IP}', 0)); time.sleep({SQUATTER_S})"],
        output="screen")
    driver = xcore_driver(BIND_IP, GROUP_PORT, allow_blade=True)
    return launch.LaunchDescription([
        board,
        squatter,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {"driver": driver}


def set_flags(**flags):
    write_json(FLAGS, flags)


board = functools.partial(read_json, STATE)


received = functools.partial(xcore_datagrams, LOG)


def setpoints(entries):
    """(t, kind, value) for drive and blade inputs: kind "drive" with v, w, or "blade" with its speed."""
    out = []
    for e in entries:
        if e.get("type") != DATA or e.get("arg2") != 0 or "hex" not in e:
            continue
        raw = bytes.fromhex(e["hex"])
        if e["sid"] == DIFF_DRIVE and len(raw) == 48:
            v = struct.unpack("<6d", raw)
            out.append((e["t"], "drive", (v[0], v[5])))
        elif e["sid"] == MOWER and len(raw) == 4:
            out.append((e["t"], "blade", struct.unpack("<f", raw)[0]))
    return out


class Probe(DriverProbe):
    emergency_reason = "xcore guards test"

    def __init__(self):
        super().__init__("xcore_guards_probe")
        self.status = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def wait_for(self, pred, timeout=10.0):
        """As in SimProbe, asked only once a status came."""
        return super().wait_for(lambda p: p.status is not None and pred(p), timeout)

    drive = SimProbe.cmd_vel_for


def healthy(p):
    return p.status.telemetry_ok and not p.status.emergency


class TestXcoreGuards(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        assert cls.probe.wait_for(healthy, 30), "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def release(self):
        answer = self.probe.set_emergency(False)
        self.assertTrue(answer.success, answer.message)
        return self.probe.wait_for(healthy, 5)

    def test_a_no_claim_beside_another_programs_socket(self, proc_output, driver):
        proc_output.assertWaitFor(f"another program holds UDP sockets at {BIND_IP}", process=driver, timeout=1)
        claims = received(sid=None, msg_type=CLAIM)
        self.assertTrue(claims)
        # The fake started with the squatter; the driver claims only once its socket is gone.
        self.assertGreater(claims[0]["t"], SQUATTER_S - 0.5)

    def test_b_tilt_latches_and_holds_the_release(self):
        self.assertTrue(self.probe.set_blade(True, lease_s=2.0).success)
        set_flags(roll=45)
        s = self.probe.wait_for(lambda p: p.status.emergency_reason & TILT and p.status, 3)
        self.assertTrue(s, "no TILT")
        self.assertEqual(s.emergency_reason & (TILT | LATCH), TILT | LATCH)
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.blade_on, 1), "blade stayed on")
        time.sleep(0.4)
        self.assertEqual(board()["host_bits"] & (LATCH | HIGH_LEVEL), LATCH | HIGH_LEVEL)
        self.assertEqual(board()["mower_speed"], 0.0)
        set_flags(roll=30)
        time.sleep(0.3)
        answer = self.probe.set_emergency(False)
        self.assertFalse(answer.success)
        self.assertIn("still tilted 30 deg", answer.message)
        self.assertEqual(answer.code, "STILL_TILTED")
        self.assertAlmostEqual(answer.value, 30.0, delta=1.0)
        self.assertEqual(answer.limit, 26.0)
        self.assertTrue(self.probe.status.emergency_reason & TILT)
        set_flags()
        time.sleep(0.3)
        self.assertTrue(self.release(), "release failed level")

    def test_c_wheels_turning_after_the_stop_latch(self):
        self.probe.drive(0.1, 1.0)
        # Setpoints go on for 0.5 s after the last cmd_vel; then past the first second after them, within five.
        time.sleep(2.0)
        set_flags(wheel_speed=0.2)
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status, 2)
        set_flags()
        self.assertTrue(s, "no latch")
        self.assertEqual(s.emergency_reason & (WHEEL_RUNAWAY | LATCH | HIGH_LEVEL), WHEEL_RUNAWAY | LATCH)
        time.sleep(0.3)
        self.assertTrue(self.release(), "release failed")

    def test_d_pushing_by_hand_later_latches_nothing(self, proc_output, driver):
        proc_output.assertWaitFor("wheel turns at", process=driver, timeout=1)
        # More than 5 s after the last setpoint of test c.
        time.sleep(3.5)
        set_flags(wheel_speed=0.8)
        time.sleep(1.5)
        self.assertFalse(self.probe.status.emergency, self.probe.status.emergency_reason)
        set_flags()
        time.sleep(0.3)

    def test_e_wheels_faster_than_any_command_latch(self, proc_output, driver):
        set_flags(wheel_speed=0.6)
        self.probe.drive(0.3, 1.5)
        s = self.probe.status
        set_flags()
        self.assertTrue(s.emergency, "no latch")
        self.assertEqual(s.emergency_reason & (WHEEL_RUNAWAY | LATCH | HIGH_LEVEL), WHEEL_RUNAWAY | LATCH)
        # 1.3 x (0.163 + 0.3 x 0.1778): the limits that max_duty 0.3 reaches.
        proc_output.assertWaitFor("above 0.28 m/s", process=driver, timeout=1)
        self.assertTrue(self.release(), "release failed")

    def test_f_another_host_claiming_latches(self, proc_output, driver):
        claims_before = board()["claims"]["1"]
        set_flags(foreign_host=2.0)
        s = self.probe.wait_for(lambda p: p.status.emergency_reason & FOREIGN_HOST and p.status, 15)
        self.assertTrue(s, "no latch")
        self.assertTrue(s.emergency_latched)
        proc_output.assertWaitFor("another host claims it", process=driver, timeout=1)
        # The driver keeps claiming; the fake's other host takes over again 2 s after each claim, every 3.5 s or so.
        time.sleep(5.0)
        self.assertGreater(board()["claims"]["1"], claims_before + 2)
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: p.status.telemetry_ok and
                                            not p.status.emergency_reason & SERVICE_NOT_READY, 10))
        time.sleep(1.0)
        self.assertTrue(self.probe.status.emergency_reason & FOREIGN_HOST, "latch did not hold")
        self.assertTrue(self.release(), "release failed")

    def test_g_frozen_driver_sends_no_setpoint_before_its_latch(self, proc_output, driver, proc_info):
        pid = proc_info[driver].pid
        resumed_before = board()["resumed_setpoints"]
        running = [True]

        def keep_running():
            # Wheels at 0.1 m/s and the blade on, as a node above the driver keeps them.
            next_blade = 0.0
            while running[0]:
                self.probe.cmd_vel.publish(Twist(linear=Vector3(x=0.1)))
                if time.time() >= next_blade:
                    next_blade = time.time() + 0.5
                    self.probe.blade.call_async(SetBlade.Request(on=True, lease_s=2.0))
                time.sleep(0.1)

        keeper = threading.Thread(target=keep_running, daemon=True)
        keeper.start()
        try:
            self.assertTrue(self.probe.wait_for(lambda p: p.status.blade_on and p.status.moving, 3), "not running")
            time.sleep(1.5)
            b = board()
            self.assertNotEqual(b["applied_duty"], [0.0, 0.0])
            self.assertGreater(abs(b["mower_rpm"]), 1000)
            stopped_at = board()["t"]
            os.kill(pid, signal.SIGSTOP)
            time.sleep(1.2)
            os.kill(pid, signal.SIGCONT)
            time.sleep(3.0)
            during = board()
        finally:
            running[0] = False
            keeper.join(2.0)
        proc_output.assertWaitFor("I/O thread stalled", process=driver, timeout=1)
        inputs = received(stopped_at - 0.5, sid=EMERGENCY, msg_type=DATA)
        gaps = [(a["t"], b["t"]) for a, b in zip(inputs, inputs[1:]) if b["t"] - a["t"] > 1.0]
        self.assertEqual(len(gaps), 1, gaps)
        resume = gaps[0][1]
        first = bytes.fromhex(inputs[[e["t"] for e in inputs].index(resume)]["hex"])
        self.assertEqual(struct.unpack("<HH", first)[0], HIGH_LEVEL | LATCH)
        after = [s for s in setpoints(received(resume)) if s[2] != 0.0 and s[2] != (0.0, 0.0)]
        self.assertEqual(after, [])
        # The board stopped by itself and did not start again.
        self.assertEqual(during["resumed_setpoints"], resumed_before)
        self.assertEqual(during["applied_duty"], [0.0, 0.0])
        self.assertLess(abs(during["mower_rpm"]), 100)
        s = self.probe.status
        self.assertTrue(s.emergency_latched)
        self.assertTrue(s.emergency_reason & DRIVER_STALL)
        self.assertFalse(s.blade_on)
        self.assertTrue(self.release(), "release failed")
        time.sleep(1.0)
        b = board()
        self.assertEqual(b["applied_duty"], [0.0, 0.0])
        self.assertEqual(b["mower_speed"], 0.0)
        self.assertEqual(b["resumed_setpoints"], resumed_before)
