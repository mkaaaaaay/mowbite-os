"""docking.test in the stack of sim.launch.py: docking by the dock marker alone, without map and without GNSS.

The mower is carried in front of the station of sim.yaml (the docked mower at (-1, 0), the way in going west) and
docks from there: DOCKING with the reason TEST, out to the approach point, the line, the gate, the close part, the
blind contact and the charge, DOCKED with guide marker; every drive command reaches safety/drive as SOURCE_DOCK. A start
beside the line is refused where the mower stands, a marker of the wrong size is caught at the gate (scale against the
wheels), a marker hidden for a moment on the line stops the mower until it is back, one hidden for good ends the try
with MARKER_LOST, a dark image with MARKER_NOT_SEEN and a camera that is off with MARKER_UNAVAILABLE.

The camera here gives the direction of the marker to 1 deg: with the 3 to 5 deg of M3b the pose in the station frame
stays above dock_max_position_accuracy farther than about 0.4 m from the station, and nothing drives there.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_test.py
"""
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import (DriveCommand, HardwareStatus, MqttPublish, RobotState, RpcError, RpcMethods, RpcRequest,
                              RpcResponse)
from rcl_interfaces.msg import Log

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, SimProbe, sim_description, sim_files, wrap  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_dock_test_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "dock_test.yaml")
# No fix at all: the map pose is never good, only the marker guides.
NO_GNSS = {"gnss_quality": 0}
# Carried onto the line 0.5 m in front of the station, or 0.3 m beside it, facing east: the contacts towards it.
ON_THE_LINE = [-0.5, 0.0, 0.0]
BESIDE_THE_LINE = [-0.5, 0.3, 0.0]

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            "mowbite_logic:\n  ros__parameters:\n    docking_retry_count: 0\n    geometry:\n"
            "      antenna_x: 0.098\n      antenna_y: 0.0\n"
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n"
            "fake_dock_marker:\n  ros__parameters:\n    yaw_bias_deg: 0.5\n    yaw_sigma_deg: [1.0, 1.0]\n")


def generate_test_description():
    return sim_description(WORK, 18897, extra_config=CONFIG)


class Probe(SimProbe):
    truth_tries = 5
    faults = NO_GNSS

    def __init__(self):
        super().__init__("dock_test_probe", state=STATE, flags=FLAGS, log=RX)
        self.robot = None
        # The sources of the drive commands that moved, with their time.
        self.drives = []
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(Log, "/rosout", self._log, 100)
        self.node.create_subscription(DriveCommand, "safety/drive", self._drive, 100)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.start()

    def _drive(self, msg):
        if msg.twist.linear.x != 0.0 or msg.twist.angular.z != 0.0:
            with self.lock:
                self.drives.append((time.time(), msg.source))

    def sources_since(self, since):
        with self.lock:
            return {source for t, source in self.drives if t >= since}

    def carry(self, place, **flags):
        """Carried to place [x, y, yaw_deg] and left standing; waits until the logic is idle off the contacts."""
        self.flags(**flags)
        time.sleep(0.3)
        self.flags(place=place, **flags)
        time.sleep(0.5)
        self.flags(**flags)
        assert self.wait(lambda: self.hw and not self.hw.dock_contact, 10), "still on the contacts"
        assert self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 30), "not idle"
        # The estimate of the station frame starts only at rest.
        time.sleep(2.0)

    def station(self):
        """The true pose in the station frame: x the way in (west), y left of it."""
        truth = self.truth()
        return -(truth["x"] + 1.0), -truth["y"], wrap(truth["yaw"] + math.pi)

    def test(self):
        answer = self.rpc("docking.test", {"client": "test"})
        assert answer and "result" in answer, answer
        return answer["result"]


class TestDockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def failed(self, since, timeout):
        failed = self.probe.event(since, "DOCKING_FAILED", timeout)
        self.assertIsNotNone(failed, self.probe.events[since:])
        self.assertEqual(failed["guide"], "marker", failed)
        return failed

    def test_01_offered_without_map_and_gnss(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: "docking.test" in p.methods.get("mowbite_logic", []), 30), p.methods)
        self.assertTrue(p.wait(lambda: p.robot is not None, 30), "no robot_state")
        p.carry(ON_THE_LINE)
        self.assertGreater(p.robot.pos_accuracy, 0.1, "the map pose is good without GNSS")

    def test_02_from_the_hand_start_to_docked(self):
        p = self.probe
        p.carry(ON_THE_LINE)
        since, began = len(p.events), time.time()
        self.assertEqual(p.test()["state"], "DOCKING")
        docking = p.event(since, "DOCKING", 5)
        self.assertIsNotNone(docking, p.events[since:])
        self.assertEqual(docking["reason"], "TEST")
        docked = p.event(since, "DOCKED", 150)
        self.assertIsNotNone(docked, p.events[since:])
        self.assertEqual(docked["guide"], "marker")
        self.assertIsNone(next((e for e in p.events[since:] if e["type"] == "DOCKING_RETRY"), None))
        truth = p.truth()
        self.assertTrue(truth["dock_ok"], truth)
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.02)
        self.assertLess(abs(truth["y"]), 0.02, truth)
        # Only the follower in the station frame moved the mower.
        self.assertEqual(p.sources_since(began), {DriveCommand.SOURCE_DOCK})

    def test_03_beside_the_line_refused(self):
        p = self.probe
        p.carry(BESIDE_THE_LINE)
        since, began = len(p.events), time.time()
        p.test()
        failed = self.failed(since, 40)
        self.assertIn("beside the line", failed["message"])
        self.assertEqual(p.sources_since(began), set())
        standing, states = p.standing()
        self.assertTrue(standing, states)

    def test_04_wrong_marker_size_caught_at_the_gate(self):
        p = self.probe
        p.carry(ON_THE_LINE, marker_scale=1.3)
        since = len(p.events)
        p.test()
        failed = self.failed(since, 120)
        self.assertIn("right size", failed["message"])
        self.assertFalse(p.truth()["dock_ok"])
        p.flags()

    def wait_on_the_line(self, timeout):
        """Until the mower backs along the line past 0.35 m from the approach point; True if it did."""
        p = self.probe
        out = p.wait(lambda: p.station()[0] < -0.75, timeout)
        return bool(out) and bool(p.wait(lambda: p.station()[0] > -0.45, timeout))

    def test_05_hidden_on_the_line_waits_and_goes_on(self):
        p = self.probe
        p.carry(ON_THE_LINE)
        since = len(p.events)
        p.test()
        self.assertTrue(self.wait_on_the_line(60), p.station())
        logs = len(p.logs)
        p.flags(marker_hidden=True)
        self.assertTrue(p.logged("Docking waits for the marker", 10, logs), p.logs[logs:])
        standing, states = p.standing()
        self.assertTrue(standing, states)
        time.sleep(3.0)
        p.flags()
        self.assertTrue(p.logged("Docking goes on: the marker is back", 30, logs), p.logs[logs:])
        docked = p.event(since, "DOCKED", 150)
        self.assertIsNotNone(docked, p.events[since:])

    def test_06_hidden_for_good_is_marker_lost(self):
        p = self.probe
        p.carry(ON_THE_LINE)
        since = len(p.events)
        p.test()
        self.assertTrue(self.wait_on_the_line(60), p.station())
        p.flags(marker_hidden=True)
        failed = self.failed(since, 60)
        self.assertEqual(failed["cause"], "MARKER_LOST", failed)
        self.assertEqual(failed["detail"], "NO_MARKER", failed)
        p.flags()

    def test_07_dark_is_not_seen(self):
        p = self.probe
        p.carry(ON_THE_LINE, marker_dark=True)
        since, began = len(p.events), time.time()
        p.test()
        failed = self.failed(since, 60)
        self.assertEqual(failed["cause"], "MARKER_NOT_SEEN", failed)
        self.assertEqual(failed["detail"], "TOO_DARK", failed)
        self.assertEqual(p.sources_since(began), set())
        p.flags()

    def test_08_camera_off_is_unavailable(self):
        p = self.probe
        p.carry(ON_THE_LINE, camera_off=True)
        since, began = len(p.events), time.time()
        p.test()
        failed = self.failed(since, 30)
        self.assertEqual(failed["cause"], "MARKER_UNAVAILABLE", failed)
        self.assertEqual(failed["detail"], "NO_SIGNAL", failed)
        self.assertEqual(p.sources_since(began), set())
        p.flags()
