"""Docking by the dock marker in normal operation (dock_guide) in the stack of sim.launch.py, with RTK and the map.

From the station the mower drives into the yard and is sent home with docking.start: the approach by the map
(SOURCE_FOLLOWER), at the approach point the sight of the marker, then the line, the gate, the close part and the
contact in the station frame (SOURCE_DOCK), DOCKED with guide marker. A station that stands 5 cm off its place in the
map is docked onto by the marker; 20 cm off it is a MARKER_MISMATCH and nothing drives by the marker. A dark image is
MARKER_NOT_SEEN and never docks by the position; a marker hidden for good on the line drives out by the map and fails
with MARKER_LOST. A camera that is off fails the try with MARKER_UNAVAILABLE under require, and docks by the position
with DOCKING_WITHOUT_MARKER under prefer (MOWBITE_DOCK_GUIDE=prefer).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_guide.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import (DriveCommand, HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError,
                              RpcRequest, RpcResponse)
from mowbite_msgs.srv import DriveTo, ReplaceMap
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

GUIDE = os.environ.get("MOWBITE_DOCK_GUIDE") or "require"
WORK = tempfile.mkdtemp(prefix="mowbite_dock_guide_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "dock_guide.yaml")

# The yard and the station of test_dock: the docked mower at (-1, 0), backing in going west; the line onto it begins
# at (-0.2, 0). The yard begins 0.5 m in front of the station.
YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}
DOCKED = [-1.0, 0.0, 0.0]
FIELD = (1.5, 0.5)

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            f"mowbite_logic:\n  ros__parameters:\n    dock_guide: {GUIDE}\n    docking_retry_count: 0\n"
            "    gps_wait_time: 2.0\n    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18899 if GUIDE == "prefer" else 18898, extra_config=CONFIG)


class Probe(SimProbe):
    truth_tries = 5

    def __init__(self):
        super().__init__("dock_guide_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.drives = []
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(Log, "/rosout", self._log, 100)
        self.node.create_subscription(DriveCommand, "safety/drive", self._drive, 100)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.start()

    def _drive(self, msg):
        if msg.source != DriveCommand.SOURCE_STOP and (msg.twist.linear.x != 0.0 or msg.twist.angular.z != 0.0):
            with self.lock:
                self.drives.append((time.time(), msg.source))

    def sources_since(self, since):
        with self.lock:
            return [source for t, source in self.drives if t >= since]

    def replace(self):
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = self.call(self.replace_map, request)
        assert answer.success, answer.message
        assert self.wait(lambda: self.map and self.map["docking_stations"], 10), "the map did not come"
        time.sleep(0.5)

    def put_on_the_station(self):
        """Carried onto the contacts; waits for the contact, an RTK position fit for the check and DOCKED."""
        since = len(self.events)
        self.flags()
        time.sleep(0.3)
        self.flags(place=DOCKED)
        assert self.wait(lambda: self.hw and self.hw.dock_contact, 10), "no contact on the station"
        assert self.wait(lambda: self.pose and self.pose.position_accuracy <= 0.03
                         and abs(self.pose.x + 1.0) < 0.05, 30), "no RTK position there"
        assert self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 10)
        self.flags()
        assert self.event(since, "DOCKED", 20), "no DOCKED on the station"

    def into_the_field(self, **flags):
        """From the station to FIELD by logic/drive_to, then the flags; True once it arrived and stands idle."""
        self.put_on_the_station()
        request = DriveTo.Request()
        request.x, request.y = FIELD
        since = len(self.events)
        for _ in range(10):
            answer = self.call(self.drive_to, request)
            if answer.success or "still stopping" not in answer.message:
                break
            time.sleep(0.3)
        assert answer.success, answer.message
        end = self.event(since, "DRIVE", 90, phase="end")
        assert end and end.get("arrived"), self.events[since:]
        self.flags(**flags)
        return self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 10)

    def home(self):
        answer = self.rpc("docking.start")
        assert answer == {"result": {"state": "DOCKING", "sub_state": ""}}, answer


class TestDockGuide(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def failed(self, since, timeout, cause):
        failed = self.probe.event(since, "DOCKING_FAILED", timeout)
        self.assertIsNotNone(failed, self.probe.events[since:])
        self.assertEqual(failed["cause"], cause, failed)
        self.assertEqual(failed["guide"], "marker", failed)
        return failed

    def test_01_ready(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        p.replace()
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")

    def docks_by_the_marker(self, shift):
        p = self.probe
        self.assertTrue(p.into_the_field(dock_shift_y=shift))
        since, began = len(p.events), time.time()
        p.home()
        docked = p.event(since, "DOCKED", 150)
        self.assertIsNotNone(docked, p.events[since:])
        self.assertEqual(docked["guide"], "marker")
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_RETRY", types)
        self.assertNotIn("DOCKING_WITHOUT_MARKER", types)
        truth = p.truth()
        self.assertTrue(truth["dock_ok"], truth)
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.02)
        self.assertAlmostEqual(truth["y"], shift, delta=0.02)
        # The approach by the map, then only the follower in the station frame.
        sources = p.sources_since(began)
        self.assertEqual(sources[0], DriveCommand.SOURCE_FOLLOWER, sources[:3])
        last_map = max(i for i, s in enumerate(sources) if s != DriveCommand.SOURCE_DOCK)
        self.assertEqual(set(sources[last_map + 1:]), {DriveCommand.SOURCE_DOCK})
        self.assertGreater(len(sources) - last_map, 20)

    def test_02_docking_start_docks_by_the_marker(self):
        self.docks_by_the_marker(0.0)

    def test_03_station_5_cm_off_the_map(self):
        self.docks_by_the_marker(0.05)

    def test_04_station_20_cm_off_is_a_mismatch(self):
        p = self.probe
        self.assertTrue(p.into_the_field(dock_shift_y=0.20))
        since, began = len(p.events), time.time()
        p.home()
        self.failed(since, 90, "MARKER_MISMATCH")
        self.assertNotIn(DriveCommand.SOURCE_DOCK, p.sources_since(began))
        self.assertNotIn("DOCKING_WITHOUT_MARKER", [e["type"] for e in p.events[since:]])
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_05_dark_never_docks_by_the_position(self):
        p = self.probe
        self.assertTrue(p.into_the_field(marker_dark=True))
        since, began = len(p.events), time.time()
        p.home()
        failed = self.failed(since, 120, "MARKER_NOT_SEEN")
        self.assertEqual(failed["detail"], "TOO_DARK", failed)
        self.assertNotIn(DriveCommand.SOURCE_DOCK, p.sources_since(began))
        self.assertNotIn("DOCKING_WITHOUT_MARKER", [e["type"] for e in p.events[since:]])
        self.assertFalse(p.truth()["dock_ok"])

    def test_06_hidden_for_good_on_the_line(self):
        p = self.probe
        self.assertTrue(p.into_the_field())
        since = len(p.events)
        p.home()
        # On the line by the marker, then the marker is gone.
        self.assertTrue(p.wait(lambda: p.truth()["x"] < -0.4, 120), p.events[since:])
        p.flags(marker_hidden=True)
        hidden = time.time()
        self.failed(since, 90, "MARKER_LOST")
        # Out by the map to the approach point.
        self.assertIn(DriveCommand.SOURCE_FOLLOWER, p.sources_since(hidden))
        self.assertAlmostEqual(p.truth()["x"], -0.2, delta=0.1)
        p.flags()

    def test_07_camera_off(self):
        p = self.probe
        self.assertTrue(p.into_the_field(camera_off=True))
        since, began = len(p.events), time.time()
        p.home()
        if GUIDE == "require":
            failed = self.failed(since, 90, "MARKER_UNAVAILABLE")
            self.assertEqual(failed["detail"], "NO_SIGNAL", failed)
            self.assertNotIn(DriveCommand.SOURCE_DOCK, p.sources_since(began))
            self.assertFalse(p.truth()["dock_ok"])
            return
        docked = p.event(since, "DOCKED", 150)
        self.assertIsNotNone(docked, p.events[since:])
        self.assertEqual(docked["guide"], "position")
        without = p.event(since, "DOCKING_WITHOUT_MARKER", 1)
        self.assertIsNotNone(without, p.events[since:])
        self.assertEqual((without["cause"], without["detail"]), ("MARKER_UNAVAILABLE", "NO_SIGNAL"), without)
        self.assertEqual((without["severity"], without["resume"]), ("warning", "auto"), without)
        self.assertNotIn(DriveCommand.SOURCE_DOCK, p.sources_since(began))
