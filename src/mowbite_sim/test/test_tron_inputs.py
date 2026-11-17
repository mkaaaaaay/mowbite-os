"""hw/inputs and the latch count of tron_mcu, here against fake_tron on a pty with STOP kept by the board as on the
Tron: the four fixed inputs are listed for a late subscriber and with every change, hw/status carries the count of the
driver, and a release with that count is the probe for a STOP the board keeps.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_tron_inputs.py
"""
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from mowbite_msgs.msg import HardwareInputs, HardwareStatus
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, DriverProbe, TestShutdown, fake_tron, sim_files, sim_links,  # noqa: E402,F401
                        wait_until, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_tron_inputs_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
BOARD = {"stop_latches": 1}
NAMES = ["stop", "lift", "bumper_left", "bumper_right"]


def generate_test_description():
    write_json(FLAGS, BOARD)
    driver = launch_ros.actions.Node(package="tron_mcu", executable="tron_mcu_node", output="screen",
                                     parameters=[{"port": LINKS["mcu"], "send_imu": False}])
    return launch.LaunchDescription([fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"), driver,
                                     launch_testing.actions.ReadyToTest()]), {}


class Probe(DriverProbe):
    def __init__(self):
        super().__init__("tron_inputs_probe", log=LOG)
        self.status = None
        self.inputs = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 50)
        self.node.create_subscription(HardwareInputs, "hw/inputs", self._inputs, LATCHED)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _inputs(self, msg):
        with self.lock:
            self.inputs.append(msg)

    def wait_status(self, pred, timeout=5.0):
        """The latest hw/status once pred(status) holds, else None."""
        def fits():
            s = self.status
            return s if s is not None and pred(s) else None
        return wait_until(fits, timeout, lock=self.lock, poll=self.poll)

    def active(self, since=0):
        """The names of the active inputs of every hw/inputs from index since on."""
        with self.lock:
            return [[line.name for line in msg.inputs if line.active] for msg in self.inputs[since:]]

    def wait_active(self, names, timeout=3.0):
        return wait_until(lambda: (self.active() or [None])[-1] == names or None, timeout, poll=self.poll)

    def release(self, count):
        return self.call(self.emergency, SetEmergency.Request(latch=False, reason="test", latch_count=count))


class TestTronInputs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # The driver is up before the probe subscribes, so the first hw/inputs has to be a latched one.
        time.sleep(2.0)
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_a_the_fixed_inputs_are_listed_for_a_late_subscriber(self):
        self.assertTrue(self.probe.wait_status(lambda s: s.telemetry_ok, 15), "no telemetry")
        self.assertTrue(self.probe.wait_active([]), "no hw/inputs with every input free")
        with self.probe.lock:
            msg = self.probe.inputs[-1]
            status = self.probe.status
        self.assertFalse(msg.learning)
        self.assertEqual([line.name for line in msg.inputs], NAMES)
        self.assertEqual([line.role for line in msg.inputs], NAMES)
        self.assertEqual([line.reason for line in msg.inputs], ["stop", "lift", "collision", "collision"])
        self.assertEqual([line.held for line in msg.inputs], [True, True, False, False])
        self.assertTrue(all(line.assigned and line.active_high for line in msg.inputs))
        self.assertEqual([line.delay_ms for line in msg.inputs], [0, 0, 0, 0])
        self.assertFalse(status.emergency, status.emergency_reason)
        self.assertEqual(status.latch_count, 0)

    def test_b_a_bumper_shows_its_side_and_counts(self):
        seen = len(self.probe.active())
        write_json(FLAGS, {**BOARD, "bumper_left": 1})
        self.assertTrue(self.probe.wait_active(["bumper_left"]), self.probe.active(seen))
        write_json(FLAGS, BOARD)
        self.assertTrue(self.probe.wait_active([]), self.probe.active(seen))
        self.assertEqual(self.probe.active(seen), [["bumper_left"], []])
        # The short push stays in the reason until the release, with a count that is not 0.
        s = self.probe.wait_status(lambda s: s.emergency_latched and not s.bumper)
        self.assertTrue(s, "the bumper did not latch")
        self.assertEqual(s.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_COLLISION)
        self.assertNotEqual(s.latch_count, 0)
        refused = self.probe.release(s.latch_count + 1)
        self.assertFalse(refused.success)
        self.assertEqual(refused.code, "LATCH_CHANGED")
        self.assertTrue(self.probe.release(s.latch_count).success)
        self.assertTrue(self.probe.wait_status(lambda s: not s.emergency), "the release with the count did not release")

    def test_c_a_release_with_the_count_is_the_probe_for_a_kept_stop(self):
        before = self.probe.wait_status(lambda s: not s.emergency).latch_count
        write_json(FLAGS, {**BOARD, "stop": 1})
        s = self.probe.wait_status(lambda s: s.emergency_latched and s.stop)
        self.assertTrue(s, "STOP did not latch")
        self.assertEqual(s.emergency_reason, HardwareStatus.REASON_LATCH | HardwareStatus.REASON_STOP)
        self.assertNotIn(s.latch_count, (0, before))
        self.assertTrue(self.probe.wait_active(["stop"]), self.probe.active())
        # Held: the release is taken, STOP and the latch stay.
        self.assertTrue(self.probe.release(s.latch_count).success)
        time.sleep(1.5)
        held = self.probe.wait_status(lambda s: True)
        self.assertTrue(held.emergency_latched and held.stop, held)
        self.assertEqual(held.latch_count, s.latch_count)
        # Let go: the board keeps showing it, the next try with the same count releases.
        write_json(FLAGS, BOARD)
        time.sleep(0.5)
        self.assertEqual(self.probe.active()[-1], ["stop"])
        self.assertTrue(self.probe.release(s.latch_count).success)
        self.assertTrue(self.probe.wait_status(lambda s: not s.emergency), "STOP let go was not released")
        self.assertTrue(self.probe.wait_active([]), self.probe.active())


if __name__ == "__main__":
    unittest.main()
