"""hw/silence and hw/end of tron_mcu against fake_tron on a pty: silent, the board gets no frame, afterwards the driver
latches with HIGH_LEVEL and sends the blade off and three stops; after hw/end the process ends with exit code 3 without
another frame. Read only it offers neither.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_driver_silence.py
"""
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from mowbite_msgs.msg import HardwareStatus
from mowbite_msgs.srv import EndDriver, SetEmergency, SilenceDriver

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HwProbe, TestShutdown, fake_tron, host_packets, last_time, moving,  # noqa: E402,F401
                        package_program, sim_files, sim_links, stop_process, sub_packets)

WORK = tempfile.mkdtemp(prefix="mowbite_driver_silence_")
LINKS = sim_links(WORK)
_, _, LOG, _ = sim_files(WORK)
SPEED, CUTTER = 4, 11
SILENCE_MAX = 3.0


def generate_test_description():
    return launch.LaunchDescription([fake_tron(LINKS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


class Driver:
    """tron_mcu as a process of its own, with the silence of the Tron."""

    def __init__(self, read_only=False):
        exe = package_program("tron_mcu", "tron_mcu_node")
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", f"silence_max:={SILENCE_MAX}",
                                      "-p", f"read_only:={str(read_only).lower()}"], start_new_session=True)

    def stop(self):
        return stop_process(self.proc, group=True)


class Probe(HwProbe):
    def __init__(self):
        super().__init__("driver_silence_probe", log=LOG)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.silence = self.node.create_client(SilenceDriver, "hw/silence")
        self.end = self.node.create_client(EndDriver, "hw/end")
        self.start()

    def latest(self, pred, timeout=10.0):
        return self.wait(lambda: self.status if self.status and pred(self.status) else None, timeout)

    def drive(self, seconds):
        """Commands motion at 20 Hz for that long."""
        msg = Twist()
        msg.linear.x = 0.2
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(msg)
            time.sleep(0.05)


class TestDriverSilence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.driver = Driver()

    @classmethod
    def tearDownClass(cls):
        if cls.driver:
            cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def restart(self, **kwargs):
        self.driver.stop()
        with self.probe.lock:
            self.probe.status = None
            self.probe.capabilities = None
        type(self).driver = Driver(**kwargs)
        return self.probe.latest(lambda s: s.telemetry_ok, 15)

    def test_a_capabilities_offer_both(self):
        caps = self.probe.wait(lambda: self.probe.capabilities, 15)
        self.assertTrue(caps, "no hw/capabilities")
        self.assertAlmostEqual(caps.silence_max_s, SILENCE_MAX, places=3)
        self.assertTrue(caps.driver_end)

    def test_b_silent_then_latched_with_blade_off_and_three_stops(self):
        s = self.probe.latest(lambda s: s.telemetry_ok and not s.emergency, 15)
        self.assertTrue(s, "no free status")
        count = s.latch_count
        self.probe.drive(0.5)
        answer = self.probe.call(self.probe.silence, SilenceDriver.Request(seconds=1.0))
        self.assertTrue(answer.success, answer)
        time.sleep(0.1)
        since = last_time(LOG)
        # Commands meanwhile are dropped.
        self.probe.drive(1.4)
        time.sleep(0.5)
        got = host_packets(LOG, since + 0.001)
        self.assertTrue(got, "nothing after the silence")
        # The silence began before since: what follows comes from its end.
        self.assertGreaterEqual(got[0]["t"] - since, 0.8, got[0])
        end = got[0]["t"]
        speeds = sub_packets(LOG, end - 0.001, SPEED)
        self.assertEqual(len(speeds), 3, speeds)
        self.assertFalse(any(moving(e) for e in speeds), speeds)
        self.assertLessEqual(speeds[0]["t"] - end, 0.05)
        self.assertEqual([e for e in sub_packets(LOG, since + 0.001, SPEED) if moving(e)], [])
        blade = sub_packets(LOG, end - 0.001, CUTTER)
        self.assertGreaterEqual(len(blade), 1)
        self.assertTrue(all(bytes.fromhex(e["hex"])[0] == 0 for e in blade), blade)
        s = self.probe.latest(lambda s: s.emergency_latched, 3.0)
        self.assertTrue(s, "not latched after the silence")
        self.assertTrue(s.emergency_reason & HardwareStatus.REASON_HIGH_LEVEL, s.emergency_reason)
        self.assertEqual(s.latch_count, count + 1)
        answer = self.probe.call(self.probe.emergency, SetEmergency.Request(latch=False, reason="test",
                                                                            latch_count=s.latch_count))
        self.assertTrue(answer.success, answer)
        self.assertTrue(self.probe.latest(lambda s: not s.emergency, 3.0), "the release did not end the emergency")

    def test_c_refusals(self):
        answer = self.probe.call(self.probe.silence, SilenceDriver.Request(seconds=0.1))
        self.assertEqual((answer.success, answer.code), (False, "OUT_OF_RANGE"))
        answer = self.probe.call(self.probe.silence, SilenceDriver.Request(seconds=SILENCE_MAX + 0.5))
        self.assertEqual((answer.success, answer.code), (False, "OUT_OF_RANGE"))
        self.assertTrue(self.probe.call(self.probe.silence, SilenceDriver.Request(seconds=0.5)).success)
        answer = self.probe.call(self.probe.silence, SilenceDriver.Request(seconds=0.5))
        self.assertEqual((answer.success, answer.code), (False, "BUSY"))
        answer = self.probe.call(self.probe.emergency, SetEmergency.Request(latch=False, reason="test"))
        self.assertEqual((answer.success, answer.code), (False, "SILENT"))
        time.sleep(0.8)

    def test_d_read_only_offers_neither(self):
        self.assertTrue(self.restart(read_only=True), "no telemetry read only")
        caps = self.probe.wait(lambda: self.probe.capabilities, 5)
        self.assertEqual((caps.silence_max_s, caps.driver_end), (0.0, False))
        for client, request in ((self.probe.silence, SilenceDriver.Request(seconds=1.0)),
                                (self.probe.end, EndDriver.Request())):
            answer = self.probe.call(client, request)
            self.assertEqual((answer.success, answer.code), (False, "READ_ONLY"))
        self.assertIsNone(self.driver.proc.poll())

    def test_e_end_exits_with_code_3_without_another_frame(self):
        self.assertTrue(self.restart(), "no telemetry after the restart")
        time.sleep(0.5)
        answer = self.probe.call(self.probe.end, EndDriver.Request())
        self.assertTrue(answer.success, answer)
        since = last_time(LOG)
        self.assertEqual(self.driver.proc.wait(5.0), 3)
        time.sleep(0.3)
        self.assertEqual(host_packets(LOG, since + 0.001), [])
        type(self).driver = None


if __name__ == "__main__":
    unittest.main()
