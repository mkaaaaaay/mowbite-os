"""The docking station in the simulation (stage 4, step 1): the whole stack of sim.launch.py with the station of
fake_tron 1 m behind the start. The test records the station like a user with the remote control (a point in front,
then backwards onto the contacts until the driver reports them), then starts mowing on the contacts: the logic sets the
heading of the station in the localization and compares the position with the recorded one (G24). A check that passes
leads to undocking (test_undock.py); a moved station or a poor position refuses the start, without the mower moving.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_reference.py
"""
import functools
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import DriveTo, ReplaceMap
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, host_packets, last_time, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_dock_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "dock.yaml")
REC = "mower_logic:area_recording/"
START = "mower_logic:idle/start_mowing"
ABORT = "mower_logic:undocking/abort_undocking"

# A lawn in front of the station that the way out reaches; the station itself is not in the map until the test records
# it.
LAWN = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]


def garden(docks):
    return {"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                       "outline": [{"x": x, "y": y} for x, y in LAWN]}],
            "docking_stations": docks}


with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_logic:\n  ros__parameters:\n    dock_guide: none\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml: the docked mower at (-1, 0), backing in going west.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18835, extra_config=CONFIG)


received = functools.partial(host_packets, RX)
sim_time = functools.partial(last_time, RX)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("dock_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.raw_since = None
        self.contact_since = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.teleop_pub = self.node.create_publisher(Twist, "bridge/teleop", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.start()

    def _hw(self, msg):
        with self.lock:
            self.hw = msg
            now = time.time()
            if msg.dock_contact_raw and self.raw_since is None:
                self.raw_since = now
            if msg.dock_contact and self.contact_since is None:
                self.contact_since = now

    def teleop(self, linear, seconds, until=None):
        """Remote control at 10 Hz like the app's joystick, then one zero; stops early once until() is true."""
        end = time.time() + seconds
        while time.time() < end:
            msg = Twist()
            msg.linear.x = linear
            self.teleop_pub.publish(msg)
            time.sleep(0.1)
            if until:
                with self.lock:
                    if until():
                        break
        self.teleop_pub.publish(Twist())
        time.sleep(0.3)

    def replace(self, docks):
        request = ReplaceMap.Request()
        request.map = json.dumps(garden(docks))
        return self.call(self.replace_map, request)


class TestDockReference(unittest.TestCase):
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

    def refused(self, since, start=True):
        """The refusal of a start (or drive) since index since, without the mower moving meanwhile."""
        event = self.probe.event(since, "MOWING_REFUSED" if start else "DRIVE_REFUSED", 10)
        self.assertIsNotNone(event, self.probe.events[since:])
        return event

    def assert_still(self, before, sim_since):
        after = self.probe.truth()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.005, (before, after))
        moving = [e for e in received(sim_since) if e.get("speed") and any(v != 0 for v in e["speed"])]
        self.assertEqual(moving, [])
        self.assertEqual(self.probe.robot.current_state, "IDLE")

    def test_01_lawn_and_the_localization(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        answer = p.replace([])
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        # Away from the station there is no contact.
        self.assertFalse(p.hw.dock_contact_raw)

    def test_02_record_the_station_backing_onto_the_contacts(self):
        p = self.probe
        p.act("mower_logic:idle/start_area_recording")
        self.assertTrue(p.wait(lambda: p.robot and p.robot.current_state == "AREA_RECORDING", 10))
        # Forwards until the GPS course aligns the heading, which both points need, then back.
        p.teleop(0.2, 4.0, until=lambda: p.pose is not None and p.pose.heading_valid)
        self.assertTrue(p.wait(lambda: p.pose.heading_valid, 10), "the heading did not align")
        p.teleop(-0.2, 2.5)
        since = len(p.events)
        p.act(REC + "record_dock")
        self.assertTrue(p.wait(lambda: p.robot.current_sub_state == "RECORD_DOCKING_POSITION", 5),
                        p.events[since:])
        # Not on the contacts yet.
        p.act(REC + "record_dock")
        refused = p.event(since, "RECORDING_REFUSED", 5, reason="NO_DOCK_CONTACT")
        self.assertIsNotNone(refused, p.events[since:])
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
        # Backwards until the driver reports the debounced contact; the stop of the station holds the mower.
        p.teleop(-0.1, 25.0, until=lambda: p.hw and p.hw.dock_contact)
        self.assertTrue(p.hw.dock_contact, "no contact")
        truth = p.truth()
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.01)
        self.assertAlmostEqual(truth["y"], 0.0, delta=0.03)
        self.assertTrue(truth["dock_ok"])
        # The raw contact came first, the debounced one after three battery reports at 1 Hz.
        self.assertGreater(p.contact_since - p.raw_since, 1.5)
        p.act(REC + "record_dock")
        recorded = p.event(since, "DOCK_RECORDED", 10)
        self.assertIsNotNone(recorded, p.events[since:])
        dock = p.wait(lambda: p.map and p.map["docking_stations"] and p.map["docking_stations"][0], 10)
        self.assertAlmostEqual(dock["position"]["x"], -1.0, delta=0.05)
        self.assertAlmostEqual(dock["position"]["y"], 0.0, delta=0.05)
        self.assertLess(abs(math.remainder(dock["heading"] - math.pi, 2 * math.pi)), math.radians(6))
        p.act(REC + "exit_recording_mode")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 10))
        # The driver switches charging on, the station gives current only then.
        self.assertTrue(p.wait(lambda: p.hw.battery_current < 0, 5), p.hw.battery_current)
        self.assertTrue(any(e.get("charge") == 1 for e in received(0)))

    def test_03_a_check_that_passes_leads_to_undocking(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy <= 0.03, 10))
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 10, state="UNDOCKING"), p.events[since:])
        self.assertFalse([e for e in p.events[since:] if e["type"] == "MOWING_REFUSED"])
        # The heading of the station: the mower backed in going west and faces east.
        self.assertTrue(p.wait(lambda: p.pose.heading_valid and abs(p.pose.heading) < math.radians(3), 5),
                        p.pose.heading)
        p.act(ABORT)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        # Back onto the contacts by hand for the checks that follow.
        p.flags(place=[-1.0, 0.0, 0.0])
        self.assertTrue(p.wait(lambda: p.hw.dock_contact and abs(p.truth()["x"] + 1.0) < 0.001, 10), p.truth())
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy <= 0.03
                               and abs(p.pose.x + 1.0) < 0.05, 20), (p.pose.x, p.pose.y))

    def test_04_a_moved_station_is_a_reference_mismatch(self):
        p = self.probe
        # Somebody moves the station with the mower on it, or the corrections changed their reference.
        p.flags(dock_shift_y=0.2)
        self.assertTrue(p.wait(lambda: abs(p.truth()["y"] - 0.2) < 0.01, 5), p.truth())
        time.sleep(1.0)
        self.assertTrue(p.hw.dock_contact)
        before, sim_since, since = p.truth(), sim_time(), len(p.events)
        p.act(START)
        refused = self.refused(since)
        self.assertEqual(refused["reason"], "REFERENCE_MISMATCH", refused)
        self.assertAlmostEqual(refused["distance"], 0.2, delta=0.03)
        self.assertEqual(refused["limit"], 0.1)
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
        # A drive from the station is checked the same way.
        since = len(p.events)
        request = DriveTo.Request()
        request.x, request.y = 2.0, 0.0
        answer = p.call(p.drive_to, request)
        self.assertFalse(answer.success)
        self.assertIn("from the recorded one", answer.message)
        self.assertEqual(self.refused(since, start=False)["reason"], "REFERENCE_MISMATCH")
        time.sleep(1.0)
        self.assert_still(before, sim_since)
        p.flags()
        self.assertTrue(p.wait(lambda: abs(p.truth()["y"]) < 0.01, 5), p.truth())

    def test_05_a_poor_position_on_the_station(self):
        p = self.probe
        # Float fixes do not count: the accuracy grows from the last fixed one.
        p.flags(gnss_quality=5)
        self.assertTrue(p.wait(lambda: 0.03 < p.pose.position_accuracy < 0.1, 10), p.pose.position_accuracy)
        since = len(p.events)
        p.act(START)
        refused = self.refused(since)
        self.assertEqual(refused["reason"], "POSITION_ACCURACY", refused)
        self.assertEqual(refused["limit"], 0.03)
        p.flags()
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy <= 0.03, 30), p.pose.position_accuracy)

    def test_06_contact_without_a_station_in_the_map(self):
        p = self.probe
        answer = p.replace([])
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.map and not p.map["docking_stations"], 10))
        since = len(p.events)
        p.act(START)
        refused = self.refused(since)
        self.assertEqual(refused["reason"], "NO_DOCK", refused)
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
