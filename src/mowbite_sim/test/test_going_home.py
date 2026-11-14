"""Going home while mowing (O11) in the simulation: the whole stack of sim.launch.py with the station of fake_tron and
in the map. A job started on the station undocks and mows; a low battery (the highest voltage of battery_low_window s
below battery_empty_voltage), a critical one (battery_critical_time s below battery_critical_voltage), a hot blade
motor and, with rain_mode 1, rain send it home with exactly one event DOCKING and its reason, and it docks with the
charge current confirmed; rain with rain_mode 0 changes nothing. A start below the empty voltage or with the blade
motor at motor_cold_temperature or more is refused. From a pause that waits for the user (the app's pause, the
emergency stop) it never goes home by itself: the battery ends the run there with BATTERY_LOW and IDLE, the mower
stays where it is. Docking without a GNSS fix fails after gps_timeout with DOCKING_FAILED and the reason of the
position (the localization keeps its pose and calls it lost by its accuracy: POSITION_ACCURACY; NO_POSITION and
NO_HEADING are tested in the gtests of the core).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_going_home.py
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
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest
from mowbite_msgs.msg import RpcResponse
from mowbite_msgs.srv import ReplaceMap
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_going_home_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "going_home.yaml")
START = "mower_logic:idle/start_mowing"
ABORT_MOWING = "mower_logic:mowing/abort_mowing"
P = "/mower_logic/"

# The yard and the station of test_dock.py, with a patch large enough for several starts of the same job.
YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
PATCH = [(0.8, -1.5), (3.6, -1.5), (3.6, 1.5), (0.8, 1.5)]
DOCKED = [-1.0, 0.0, 0.0]
# tron.yaml: empty 21.0 V (the highest voltage of 20 s), critical 19.7 V for 2 s; motor_hot/cold_temperature 70/40.
EMPTY, CRITICAL, LOW_WINDOW = 21.0, 19.7, 20.0
MOTOR_HOT, MOTOR_COLD = 70.0, 40.0
RAIN_CHECK = 20
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]},
                    {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                     "outline": [{"x": x, "y": y} for x, y in PATCH]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_logic:\n  ros__parameters:\n    dock_guide: none\n    gps_wait_time: 2.0\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18843, extra_config=CONFIG)


class Probe(SimProbe):
    truth_tries = 5

    def __init__(self):
        super().__init__("going_home_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.start()

    def set_params(self, **values):
        answer = self.rpc("params.set", {"params": {P + k: v for k, v in values.items()}})
        assert answer == {"result": {P + k: v for k, v in values.items()}}, answer


class TestGoingHome(unittest.TestCase):
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

    def assert_still(self, seconds=1.0):
        before = self.probe.truth()
        time.sleep(seconds)
        after = self.probe.truth()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.005, (before, after))

    def mowing(self):
        """Started from where it is: the index of the events from the start on, once it mows on the patch. The flags
        go back to the defaults after the test, whatever its outcome."""
        p = self.probe
        self.addCleanup(p.flags)
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 10), p.robot.current_state)
        # The defaults of fake_tron arrived with the last battery and motor report (1 Hz).
        self.assertTrue(p.wait(lambda: p.hw.battery_voltage > EMPTY + 1.0 and p.hw.blade_motor.temperature < MOTOR_COLD
                               and not p.hw.rain, 5), p.hw)
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 60, state="MOWING"), p.events[since:])
        self.assertTrue(p.wait(lambda: p.truth()["x"] > 0.5, 60), p.truth())
        return since

    def home(self, since, reason, timeout, **fields):
        """Exactly one DOCKING with that reason, before STATE DOCKING, then DOCKED with the charge current."""
        p = self.probe
        docking = p.event(since, "DOCKING", timeout)
        self.assertIsNotNone(docking, p.events[since:])
        self.assertEqual(docking["reason"], reason, docking)
        self.assertEqual(docking["resume"], "user", docking)
        self.assertTrue(docking["message"].startswith("Going home: "), docking)
        for key, value in fields.items():
            self.assertAlmostEqual(docking[key], value, places=3, msg=(key, docking))
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 5), p.robot.current_state)
        events = p.events[since:]
        going = [e for e in events if e["type"] == "DOCKING" or (e["type"] == "STATE" and e["state"] == "DOCKING")]
        self.assertEqual([e["type"] for e in going[:2]], ["DOCKING", "STATE"], going)
        types = [e["type"] for e in events]
        self.assertEqual(types.count("DOCKING"), 1, events)
        for absent in ("BATTERY_LOW", "JOB_COMPLETE", "DOCKING_RETRY", "DOCKING_FAILED"):
            self.assertNotIn(absent, types, events)
        truth = p.truth()
        self.assertTrue(truth["dock_ok"], truth)
        self.assertTrue(p.wait(lambda: p.hw.battery_current < 0, 5), p.hw.battery_current)
        return docking

    def rain_mode_0(self):
        # params.set only in IDLE: after a failure the mower may still be on its way.
        if self.probe.wait(lambda: self.probe.robot.current_state == "IDLE", 180):
            self.probe.set_params(rain_mode=0)

    def refused(self, reason, **fields):
        p = self.probe
        since = len(p.events)
        p.act(START)
        event = p.event(since, "MOWING_REFUSED", 10)
        self.assertIsNotNone(event, p.events[since:])
        self.assertEqual(event["reason"], reason, event)
        for key, value in fields.items():
            self.assertAlmostEqual(event[key], value, places=3, msg=(key, event))
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 5), p.robot.current_state)
        return event

    def test_01_on_the_station(self):
        p = self.probe
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

    def test_02_a_low_battery_sends_it_home(self):
        p = self.probe
        since = self.mowing()
        p.flags(battery_voltage=20.5)
        # Not before the whole window lay below the empty voltage.
        self.assertIsNone(p.event(since, "DOCKING", LOW_WINDOW - 3), p.events[since:])
        docking = self.home(since, "BATTERY_LOW", 18, battery_voltage=20.5, limit=EMPTY)
        self.assertEqual(docking["severity"], "info")
        # Below the empty voltage no start.
        self.refused("BATTERY_LOW")
        p.flags()

    def test_03_a_critical_battery_sends_it_home_at_once(self):
        p = self.probe
        since = self.mowing()
        p.flags(battery_voltage=19.0)
        docking = self.home(since, "BATTERY_CRITICAL", 8, battery_voltage=19.0, limit=CRITICAL)
        self.assertEqual(docking["severity"], "warning")
        p.flags()

    def test_04_a_hot_blade_motor_sends_it_home(self):
        p = self.probe
        since = self.mowing()
        p.flags(blade_temperature=75)
        docking = self.home(since, "MOTOR_HOT", 10, temperature=75, limit=MOTOR_HOT)
        self.assertEqual(docking["severity"], "warning")
        # It starts again only below motor_cold_temperature.
        p.flags(blade_temperature=50)
        self.assertTrue(p.wait(lambda: p.hw.blade_motor.temperature == 50, 5), p.hw.blade_motor)
        refused = self.refused("MOTOR_HOT", temperature=50, limit=MOTOR_COLD)
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
        p.flags()
        time.sleep(1.5)

    def test_05_rain_with_rain_mode_0_changes_nothing(self):
        p = self.probe
        p.set_params(rain_check_seconds=RAIN_CHECK)
        since = self.mowing()
        p.flags(rain=1)
        time.sleep(RAIN_CHECK + 6)
        self.assertNotIn("DOCKING", [e["type"] for e in p.events[since:]])
        self.assertEqual(p.robot.current_state, "MOWING")
        p.flags()
        # Sent home by hand: DOCKING MANUAL.
        since = len(p.events)
        p.act(ABORT_MOWING)
        docking = self.home(since, "MANUAL", 5)
        self.assertEqual(docking["severity"], "info")

    def test_06_rain_with_rain_mode_1_sends_it_home(self):
        p = self.probe
        p.set_params(rain_mode=1)
        self.addCleanup(self.rain_mode_0)
        since = self.mowing()
        p.flags(rain=1)
        self.assertIsNone(p.event(since, "DOCKING", RAIN_CHECK - 3), p.events[since:])
        docking = self.home(since, "RAIN", 18)
        self.assertEqual(docking["severity"], "info")

    def test_07_from_the_pause_of_the_app_never_home(self):
        p = self.probe
        since = self.mowing()
        answer = p.rpc("mowing.pause", {"client": "test"})
        self.assertEqual(answer, {"result": {"state": "PAUSED", "sub_state": ""}})
        # Heat changes nothing in the pause, a low battery ends the run where the mower stands.
        p.flags(battery_voltage=20.5, blade_temperature=75)
        low = p.event(since, "BATTERY_LOW", LOW_WINDOW + 15)
        self.assertIsNotNone(low, p.events[since:])
        self.assertAlmostEqual(low["battery_voltage"], 20.5, places=3)
        self.assertAlmostEqual(low["limit"], EMPTY, places=3)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        self.assertNotIn("DOCKING", [e["type"] for e in p.events[since:]])
        self.assert_still(2.0)
        self.assertGreater(p.truth()["x"], 0.5)
        p.flags()
        time.sleep(1.5)

    def test_08_from_the_emergency_stop_never_home(self):
        p = self.probe
        since = self.mowing()
        p.flags(stop=1)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="PAUSED", reason="EMERGENCY"), p.events[since:])
        p.flags(stop=1, battery_voltage=20.5)
        self.assertIsNotNone(p.event(since, "BATTERY_LOW", LOW_WINDOW + 15), p.events[since:])
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        p.flags()
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5))
        # Released, it stays where it is.
        self.assert_still(3.0)
        self.assertEqual(p.robot.current_state, "IDLE")
        self.assertNotIn("DOCKING", [e["type"] for e in p.events[since:]])

    def test_09_docking_without_a_fix_fails_on_the_position(self):
        p = self.probe
        self.addCleanup(p.flags)
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        self.assertIsNotNone(p.event(since, "DOCKING", 5, reason="MANUAL"), p.events[since:])
        time.sleep(2.0)
        p.flags(gnss_quality=0)
        # Lost after gps_timeout (10 s) without a fix, failed after another gps_timeout of waiting.
        failed = p.event(since, "DOCKING_FAILED", 60)
        self.assertIsNotNone(failed, p.events[since:])
        self.assertEqual((failed["reason"], failed["severity"], failed["resume"]),
                         ("POSITION_ACCURACY", "error", "user"), failed)
        self.assertGreater(failed["accuracy"], failed["limit"], failed)
        self.assertIn("no good position for 10 s", failed["message"])
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        self.assertNotIn("DOCKED", [e["type"] for e in p.events[since:]])
        self.assert_still(1.0)
        p.flags()
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy < 0.03, 30), "no RTK position again")
