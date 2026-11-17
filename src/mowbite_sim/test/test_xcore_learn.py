"""xcore_driver started with inputs.learn against fake_xcore: it watches the candidate lines of the platform instead of
its table, none of them stops anything, and INPUTS_LEARNING holds the mower whatever is asked.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_learn.py
"""
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from launch.actions import TimerAction
from mowbite_msgs.msg import HardwareInputs, HardwareStatus
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, DriverProbe, TestShutdown, fake_xcore, layers, log_entries,  # noqa: E402,F401
                        loopback_group, sim_files, write_json, xcore_config, xcore_driver)

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_learn_")
STATE, FLAGS, LOG, _ = sim_files(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DIFF_DRIVE, DATA = 2, 1
LEARNING = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_INPUTS_LEARNING
CANDIDATES = layers.parameters([xcore_config()], "xcore_driver")["inputs.candidates"]


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "20", "--state", STATE)
    driver = xcore_driver(BIND_IP, GROUP_PORT, **{"inputs.learn": True})
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {"driver": driver}


class Probe(DriverProbe):
    emergency_reason = "learn test"

    def __init__(self):
        super().__init__("xcore_learn_probe", log=LOG)
        self.status = None
        self.inputs = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareInputs, "hw/inputs", self._set("inputs"), LATCHED)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()


class TestXcoreLearn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        assert cls.probe.wait_for(lambda p: p.status is not None and p.status.telemetry_ok, 20), "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def active(self):
        return sorted(i.name for i in self.probe.inputs.inputs if i.active)

    def test_a_candidates_without_a_role(self):
        p = self.probe
        self.assertTrue(p.wait_for(lambda p: p.status.emergency_reason == LEARNING, 5), p.status.emergency_reason)
        # Without the levels of the board the driver names no line.
        self.assertTrue(p.wait_for(lambda p: p.inputs is not None and p.inputs.inputs, 2), "no hw/inputs")
        self.assertTrue(p.inputs.learning)
        self.assertEqual([i.name for i in p.inputs.inputs], CANDIDATES)
        for line in p.inputs.inputs:
            self.assertEqual((line.role, line.reason, line.assigned, line.active), ("", "", False, False), line.name)

    def test_b_a_sensor_shows_on_its_line_and_stops_nothing(self):
        p = self.probe
        write_json(FLAGS, {"rear_stop": 1})
        self.assertTrue(p.wait_for(lambda p: p.inputs is not None and [i for i in p.inputs.inputs if i.active], 3),
                        "the line does not show")
        self.assertEqual(self.active(), ["GPIO5"])
        time.sleep(0.5)
        self.assertFalse(p.status.stop)
        self.assertEqual(p.status.emergency_reason, LEARNING)
        write_json(FLAGS, {"lift_left": 1, "bumper_right": 1})
        self.assertTrue(p.wait_for(lambda p: sorted(i.name for i in p.inputs.inputs if i.active)
                                   == ["GPIO10", "GPIO12"], 3), self.active())
        write_json(FLAGS, {})
        self.assertTrue(p.wait_for(lambda p: not any(i.active for i in p.inputs.inputs), 3), self.active())
        self.assertEqual(p.status.emergency_reason, LEARNING)

    def test_c_no_release_and_no_motion(self):
        p = self.probe
        answer = p.set_emergency(False)
        self.assertFalse(answer.success)
        self.assertEqual(answer.code, "LEARNING")
        # A latch from above goes with a release as always, the one of learning stays.
        self.assertTrue(p.set_emergency(True).success)
        both = LEARNING | HardwareStatus.REASON_HIGH_LEVEL
        self.assertTrue(p.wait_for(lambda p: p.status.emergency_reason == both, 3), p.status.emergency_reason)
        self.assertEqual(p.set_emergency(False).code, "LEARNING")
        self.assertTrue(p.wait_for(lambda p: p.status.emergency_reason == LEARNING, 3), p.status.emergency_reason)
        began = p.cmd_vel_for(0.2, 1.0)
        self.assertEqual(p.status.emergency_reason, LEARNING)
        moved = [e for e in log_entries(LOG, began or 0.0) if e.get("sid") == DIFF_DRIVE and e["type"] == DATA
                 and e["arg2"] == 0 and any(bytes.fromhex(e["hex"]))]
        self.assertEqual(moved, [])
