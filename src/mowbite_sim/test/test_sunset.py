"""Going home at sunset in the simulation: the whole stack of sim.launch.py with the station of fake_tron and in the
map, the datum chosen so that the sun sets a few minutes after the stack starts. No hand on the clock: the test waits
for time/clock ok (the machine is synchronized), mowing starts before the sunset and goes on until it, then exactly
one DOCKING with reason DARK, the height of the sun near its limit, and the mower docks.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_sunset.py
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import ClockStatus, HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, package_program, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_sunset_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "sunset.yaml")
START = "mower_logic:idle/start_mowing"
LAT = 52.0
# s from the import of this file to the sunset: the start of the stack, the station and the way onto the patch.
LEAD = 240.0
# The limit of the logic (kSunsetElevation).
DARK = -0.833

# The yard, the patch and the station of test_going_home.py.
YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
PATCH = [(0.8, -1.5), (3.6, -1.5), (3.6, 1.5), (0.8, 1.5)]
DOCKED = [-1.0, 0.0, 0.0]
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]},
                    {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                     "outline": [{"x": x, "y": y} for x, y in PATCH]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}


def sun(latitude, longitude, unix_s):
    """Sunrise, sunset and the height of the sun of mowbite_sun_times."""
    out = subprocess.run([package_program("mowbite_time", "mowbite_sun_times"), str(latitude), str(longitude),
                          str(unix_s)], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def longitude_for_sunset(latitude, at):
    """The longitude where the sun sets at unix s at: the sunset moves 240 s earlier per degree east."""
    longitude = 0.0
    for _ in range(6):
        longitude += (sun(latitude, longitude, at)["set"] - at) / 240.0
        longitude = (longitude + 180.0) % 360.0 - 180.0
    return longitude


SUNSET = time.time() + LEAD
LON = longitude_for_sunset(LAT, SUNSET)
SUNSET = sun(LAT, LON, SUNSET)["set"]

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_logic:\n  ros__parameters:\n    dock_guide: none\n    gps_wait_time: 2.0\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18874, sunset=True, extra_config=CONFIG, datum_lat=str(LAT), datum_long=f"{LON:.6f}")


class Probe(SimProbe):
    truth_tries = 5

    def __init__(self):
        super().__init__("sunset_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.clock = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(ClockStatus, "time/clock", self._set("clock"), LATCHED)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.start()


class TestSunset(unittest.TestCase):
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

    def test_1_on_the_station_with_a_safe_clock(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.clock is not None, 30), "no time/clock")
        self.assertTrue(p.clock.ok, p.clock.reason)
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.map and p.map["docking_stations"], 10), "the map did not come")
        time.sleep(0.5)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        since = len(p.events)
        p.flags(place=DOCKED)
        self.assertTrue(p.wait(lambda: p.hw and p.hw.dock_contact, 10), "no contact on the station")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy <= 0.03 and abs(p.pose.x + 1.0) < 0.05,
                               30), "no RTK position there")
        p.flags()
        self.assertIsNotNone(p.event(since, "DOCKED", 30), p.events[since:])

    def test_2_home_at_sunset(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 10), p.robot.current_state)
        self.assertGreater(SUNSET - time.time(), 60, "the stack started too late for this sunset")
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 60, state="MOWING"), p.events[since:])
        # It mows until the sunset.
        self.assertIsNone(p.event(since, "DOCKING", max(SUNSET - time.time() - 3, 0)), p.events[since:])
        docking = p.event(since, "DOCKING", SUNSET - time.time() + 10)
        self.assertIsNotNone(docking, p.events[since:])
        self.assertGreaterEqual(time.time(), SUNSET - 1)
        self.assertEqual((docking["reason"], docking["severity"], docking["resume"]), ("DARK", "info", "user"),
                         docking)
        self.assertTrue(docking["message"].startswith("Going home: "), docking)
        self.assertAlmostEqual(docking["limit"], DARK, places=3)
        self.assertLess(abs(docking["sun_elevation"] - DARK), 0.2, docking)
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        events = p.events[since:]
        self.assertEqual([e["type"] for e in events].count("DOCKING"), 1, events)
        self.assertTrue(p.truth()["dock_ok"], p.truth())
