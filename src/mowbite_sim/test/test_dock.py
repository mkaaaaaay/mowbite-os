"""Docking in the simulation (stage 4, step 3): the whole stack of sim.launch.py with the station of fake_tron and in
the map. A job started on the station undocks, mows a small patch and docks after JOB_COMPLETE; abort_mowing docks as
well, and so does docking.start from IDLE in the field. On the way the mower backs along the line of the station, stops
at the gate, creeps onto the contacts and stops at the first raw contact: after it the driver sends three zero frames
and then nothing while it stands (G18). docking.start is refused with the reason in the data of the error and as
DRIVE_REFUSED; abort_docking and an emergency stop it without DOCKING_FAILED; a station 4 cm beside its place in the
map gets DOCKING_RETRY and DOCKING_FAILED (docking_retry_count of params.set), the mower waits at the approach point.
DOCKED comes only once the charge current held on the contacts (G19): contacts that do not charge fail the try with
NO_CHARGE_CURRENT, the way out begins after Charge 0, and the last try leaves the mower on them; a current that comes later
docks it after all, as does a nearly full battery with the contact alone. Taken off the station by hand it tells
CHARGING_LOST.

MOWBITE_DOCK_RATE=20 instead counts how many of 20 tries dock with the faults of a real Tron in fake_tron (RTK noise, a
late fix, wheels reporting too little, a dead band, slip): a first answer to open question 10, for the simulation only.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock.py
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
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest
from mowbite_msgs.msg import RpcResponse
from mowbite_msgs.srv import DriveTo, ReplaceMap
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, REAL_FAULTS, SimProbe, host_packets, last_time, moving, sim_description,  # noqa: E402
                        sim_files)

RATE = int(os.environ.get("MOWBITE_DOCK_RATE") or 0)
FAULTS = REAL_FAULTS if RATE else {}

WORK = tempfile.mkdtemp(prefix="mowbite_dock_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "dock.yaml")
START = "mower_logic:idle/start_mowing"
ABORT_MOWING = "mower_logic:mowing/abort_mowing"
ABORT = "mower_logic:docking/abort_docking"

# A yard to drive in from 0.5 m in front of the station, and a small patch in it to mow. The station: the docked mower
# at (-1, 0), backing in going west; the line onto it begins at (-0.2, 0), the gate at (-0.75, 0).
YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
PATCH = [(1.0, -0.5), (2.6, -0.5), (2.6, 0.5), (1.0, 0.5)]
DOCKED = [-1.0, 0.0, 0.0]


def garden(station=True, yard=YARD):
    return {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                       "outline": [{"x": x, "y": y} for x, y in yard]},
                      {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                       "outline": [{"x": x, "y": y} for x, y in PATCH]}],
            "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]
            if station else []}


with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_logic:\n  ros__parameters:\n    dock_guide: none\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18838, extra_config=CONFIG)


received = functools.partial(host_packets, RX)
sim_time = functools.partial(last_time, RX)


class Probe(SimProbe):
    truth_tries = 5
    faults = FAULTS

    def __init__(self):
        super().__init__("dock_probe", state=STATE, flags=FLAGS, log=RX)
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
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.start()

    def replace(self, station=True, yard=YARD):
        request = ReplaceMap.Request()
        request.map = json.dumps(garden(station, yard))
        answer = self.call(self.replace_map, request)
        assert answer.success, answer.message
        assert self.wait(lambda: self.map and bool(self.map["docking_stations"]) == station
                         and self.map["areas"][0]["outline"][0]["x"] == yard[0][0], 10), "the map did not come"
        # The logic gets map/json at the same time as the probe, give it a moment.
        time.sleep(0.5)

    def put_on_the_station(self, **flags):
        """Carried onto the contacts; waits for the debounced contact, an RTK position fit for the check and the DOCKED
        of the charge confirmed there."""
        since = len(self.events)
        self.flags()
        time.sleep(0.3)
        self.flags(place=DOCKED, **flags)
        assert self.wait(lambda: self.hw and self.hw.dock_contact, 10), "no contact on the station"
        assert self.wait(lambda: self.pose and self.pose.position_accuracy_at_heading <= 0.03
                         and abs(self.pose.x + 1.0) < 0.05, 30), "no RTK position there"
        assert self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 10)
        self.flags(**flags)
        assert self.event(since, "DOCKED", 20), "no DOCKED on the station"

    def drive(self, x, y):
        """logic/drive_to (from the station after undocking): the end of the drive once it arrived, else None."""
        request = DriveTo.Request()
        request.x, request.y = x, y
        since = len(self.events)
        # Right after docking the follower may still be letting go of its last path: the logic refuses for a moment.
        for _ in range(10):
            answer = self.call(self.drive_to, request)
            if answer.success or "still stopping" not in answer.message:
                break
            time.sleep(0.3)
        if not answer.success:
            return None
        end = self.event(since, "DRIVE", 90, phase="end")
        return end if end and end.get("arrived") else None


def g18(test, rx, docked_at, stood_until):
    """After the last wheel command onto the contacts three zero frames, then no SpeedData while the mower stands."""
    last = max(i for i, e in enumerate(rx) if moving(e))
    test.assertLess(rx[last]["speed"][0], 0.0, rx[last])
    after = [e for e in rx[last + 1:] if "speed" in e]
    test.assertEqual(len(after), 3, after)
    test.assertTrue(all(e["speed"] == [0.0, 0.0] for e in after), after)
    test.assertLess(after[-1]["t"], docked_at, (after, docked_at))
    test.assertGreater(stood_until - docked_at, 4.0)


def going_home(test, events):
    """Sent home by hand: exactly one event DOCKING MANUAL (api 5), before STATE DOCKING."""
    going = [e for e in events if e["type"] == "DOCKING" or (e["type"] == "STATE" and e["state"] == "DOCKING")]
    test.assertEqual([e["type"] for e in going[:2]], ["DOCKING", "STATE"], going)
    test.assertEqual((going[0]["reason"], going[0]["severity"], going[0]["resume"]), ("MANUAL", "info", "user"))
    test.assertEqual(len([e for e in events if e["type"] == "DOCKING"]), 1, events)


class Base(unittest.TestCase):
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
        # The state file comes 5 times a second: only a sample written after this call shows the mower now.
        p = self.probe
        written = p.truth()["t"]
        self.assertTrue(p.wait(lambda: p.truth()["t"] != written, 2))
        before = p.truth()
        time.sleep(seconds)
        after = self.probe.truth()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.005, (before, after))

    def on_the_station(self):
        truth = self.probe.truth()
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.01)
        self.assertLess(abs(truth["y"]), 0.02, truth)
        self.assertTrue(truth["dock_ok"], truth)

    def ready(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        p.replace()
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        p.put_on_the_station()


@unittest.skipIf(RATE, "the rate only")
class TestDock(Base):
    def test_01_on_the_station(self):
        self.ready()

    def test_02_a_job_from_the_station_ends_docked(self):
        p = self.probe
        sim_since, since = sim_time(), len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "UNDOCKED", 30), p.events[since:])
        self.assertIsNotNone(p.event(since, "JOB_COMPLETE", 600), p.events[since:])
        docking_from = len(p.events)
        docked = p.event(since, "DOCKED", 120)
        self.assertIsNotNone(docked, p.events[docking_from:])
        docked_at = sim_time()
        self.assertFalse(docked.get("reason"), docked)
        self.assertIsNotNone(p.event(docking_from, "STATE", 5, state="IDLE"))
        after = p.events[since:]
        states = [e["state"] for e in after if e["type"] == "STATE"]
        self.assertEqual(states[:2], ["UNDOCKING", "MOWING"], states)
        self.assertEqual(states[-2:], ["DOCKING", "IDLE"], states)
        self.assertFalse([e for e in after if e["type"] in ("DOCKING_RETRY", "DOCKING_FAILED", "UNDOCKING_FAILED")],
                         after)
        # Home after JOB_COMPLETE without an event DOCKING, as OpenMower.
        self.assertNotIn("DOCKING", [e["type"] for e in after])
        self.on_the_station()
        # Backwards onto the station: never faster than docking_speed, the last stretch at contact_speed.
        time.sleep(5.0)
        rx = received(sim_since)
        start = next(e["t"] for e in rx if e["t"] >= docked_at - 60 and moving(e) and e["speed"][0] < 0)
        back = [e for e in rx if e["t"] >= start and moving(e)]
        self.assertTrue(all(e["speed"][0] >= -0.1 - 1e-3 for e in back), back[:3])
        last = [e["speed"][0] for e in back if e["t"] >= back[-1]["t"] - 2.0]
        self.assertTrue(all(-0.06 - 1e-3 <= v <= 0 for v in last), last)
        g18(self, rx, docked_at, sim_time())
        # Charging since the debounced contact; DOCKED once the charge current held 5 s.
        charge_on = next((e["t"] for e in rx if e.get("charge") == 1 and e["t"] >= start), None)
        self.assertIsNotNone(charge_on, rx[-20:])
        self.assertGreaterEqual(docked_at - charge_on, 5.0, (charge_on, docked_at))
        self.assertTrue(p.wait(lambda: p.hw.battery_current < 0, 5), p.hw.battery_current)

    def test_03_abort_mowing_docks(self):
        p = self.probe
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 30, state="MOWING"), p.events[since:])
        time.sleep(3.0)
        p.act(ABORT_MOWING)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="DOCKING"), p.events[since:])
        self.assertIsNotNone(p.event(since, "DOCKED", 120), p.events[since:])
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("JOB_COMPLETE", types)
        going_home(self, p.events[since:])
        self.on_the_station()

    def test_04_docking_start_from_the_field(self):
        p = self.probe
        answer = p.rpc("docking.start")
        self.assertEqual(answer["error"]["code"], -32600, answer)
        self.assertEqual(answer["error"]["data"]["reason"], "ALREADY_THERE", answer)
        self.assertIsNotNone(p.drive(1.5, 0.8), "the drive did not end")
        # Refused: no station in the map, then the line beginning outside of the areas.
        p.replace(station=False)
        since = len(p.events)
        answer = p.rpc("docking.start")
        self.assertEqual(answer["error"]["data"]["reason"], "NO_DOCK", answer)
        refused = p.event(since, "DRIVE_REFUSED", 5, reason="NO_DOCK")
        self.assertIsNotNone(refused, p.events[since:])
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
        p.replace(yard=[(0.0, -2.0), (4.0, -2.0), (4.0, 2.0), (0.0, 2.0)])
        answer = p.rpc("docking.start")
        self.assertEqual(answer["error"]["data"]["reason"], "DOCK_UNREACHABLE", answer)
        self.assertEqual(answer["error"]["data"]["distance"], 0.8, answer)
        self.assertEqual(p.robot.current_state, "IDLE")
        p.replace()
        since = len(p.events)
        answer = p.rpc("docking.start")
        self.assertEqual(answer, {"result": {"state": "DOCKING", "sub_state": ""}})
        self.assertIsNotNone(p.event(since, "STATE", 5, state="DOCKING"))
        self.assertIsNotNone(p.event(since, "DOCKED", 120), p.events[since:])
        going_home(self, p.events[since:])
        self.on_the_station()

    def test_05_abort_docking(self):
        p = self.probe
        before = len(p.events)
        self.assertIsNotNone(p.drive(1.5, -0.8), "the drive did not end")
        # A request right after DOCKED is refused for a moment, it does not fail the undocking.
        self.assertNotIn("UNDOCKING_FAILED", [e["type"] for e in p.events[before:]])
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        # On the line, backwards.
        self.assertTrue(p.wait(lambda: p.truth()["x"] < -0.3, 90), p.events[since:])
        p.act(ABORT)
        self.assertIsNotNone(p.event(since, "STATE", 3, state="IDLE"), p.events[since:])
        time.sleep(0.5)
        self.assert_still()
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_FAILED", types)
        self.assertNotIn("DOCKED", types)

    def test_06_an_emergency_while_docking(self):
        p = self.probe
        since = len(p.events)
        # From where abort_docking left it, on the line: out to the approach point first.
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        self.assertTrue(p.wait(lambda: p.truth()["x"] < -0.4 and p.truth()["linear"] < 0, 60), p.events[since:])
        p.flags(stop=1)
        self.assertIsNotNone(p.event(since, "EMERGENCY", 3, emergency=True), p.events[since:])
        self.assertIsNotNone(p.event(since, "STATE", 3, state="IDLE"), p.events[since:])
        self.assert_still()
        p.flags()
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5))
        # It never goes on by itself.
        self.assert_still(3.0)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_FAILED", types)
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_07_a_station_beside_its_place_in_the_map(self):
        p = self.probe
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 1}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 1}})
        p.flags(dock_shift_y=0.04)
        sim_since, since = sim_time(), len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        retry = p.event(since, "DOCKING_RETRY", 120)
        self.assertIsNotNone(retry, p.events[since:])
        self.assertEqual((retry["reason"], retry["attempts"]), ("dock_failed", 1), retry)
        self.assertEqual((retry["severity"], retry["resume"]), ("warning", "auto"))
        self.assertIn("does not move", retry["message"])
        failed = p.event(since, "DOCKING_FAILED", 120)
        self.assertIsNotNone(failed, p.events[since:])
        self.assertEqual((failed["reason"], failed["attempts"]), ("dock_failed", 2), failed)
        self.assertEqual((failed["severity"], failed["resume"]), ("error", "user"))
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"))
        # It waits at the approach point, without contact; it pushed no faster than contact_speed at the station.
        truth = p.truth()
        self.assertLess(math.hypot(truth["x"] + 0.2, truth["y"]), 0.15, truth)
        self.assertFalse(truth["dock_ok"], truth)
        rx = received(sim_since)
        self.assertTrue(all(e["speed"][0] >= -0.1 - 1e-3 for e in rx if moving(e)))
        self.assertNotIn("DOCKED", [e["type"] for e in p.events[since:]])
        p.flags()
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 4}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 4}})

    def test_08_contacts_that_do_not_charge(self):
        p = self.probe
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 1}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 1}})
        p.flags(charge_current=0.0)
        sim_since, since = sim_time(), len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        retry = p.event(since, "DOCKING_RETRY", 150)
        self.assertIsNotNone(retry, p.events[since:])
        self.assertEqual((retry["reason"], retry["attempts"]), ("NO_CHARGE_CURRENT", 1), retry)
        self.assertEqual((retry["severity"], retry["resume"]), ("warning", "auto"))
        self.assertEqual(retry["limit"], 0.5, retry)
        self.assertIn("no charge current on the contacts within 20 s", retry["message"])
        failed = p.event(since, "DOCKING_FAILED", 150)
        self.assertIsNotNone(failed, p.events[since:])
        self.assertEqual((failed["reason"], failed["attempts"]), ("NO_CHARGE_CURRENT", 2), failed)
        self.assertEqual((failed["severity"], failed["resume"]), ("error", "user"))
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"))
        # The last try left it on the contacts.
        self.on_the_station()
        self.assertNotIn("DOCKED", [e["type"] for e in p.events[since:]])
        # The way out of the first try began only after Charge 0 (G19): by the order in the log, both can come
        # within the same ms.
        rx = received(sim_since)
        charge_on = next(i for i, e in enumerate(rx) if e.get("charge") == 1)
        out = next(i for i, e in enumerate(rx) if i > charge_on and moving(e) and e["speed"][0] > 0)
        self.assertTrue([e for e in rx[charge_on:out] if e.get("charge") == 0], rx[max(charge_on, out - 40):out + 1])
        # The charge comes after all: DOCKED, and it stays IDLE.
        since = len(p.events)
        p.flags()
        self.assertIsNotNone(p.event(since, "DOCKED", 20), p.events[since:])
        # Taken off the station by hand: CHARGING_LOST after 10 s, it does not drive.
        since = len(p.events)
        p.flags(place=[0.5, 0.0, 0.0])
        lost = p.event(since, "CHARGING_LOST", 20, reason="NO_DOCK_CONTACT")
        self.assertIsNotNone(lost, p.events[since:])
        self.assertEqual((lost["severity"], lost["resume"]), ("warning", "user"))
        self.assertEqual(lost["battery_percentage"], 80)
        self.assertEqual(p.robot.current_state, "IDLE")
        p.flags()
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 4}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 4}})

    def test_09_a_nearly_full_battery_needs_no_charge_current(self):
        p = self.probe
        p.flags(battery_percentage=97, charge_current=0.0)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy_at_heading < 0.03
                               and abs(p.pose.x - 0.5) < 0.05, 30), p.pose)
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        self.assertFalse([e for e in p.events[since:] if e["type"] in ("DOCKING_RETRY", "DOCKING_FAILED")])
        self.on_the_station()
        p.flags()


@unittest.skipUnless(RATE, "MOWBITE_DOCK_RATE only")
class TestDockRate(Base):
    # Where each try starts: in turn north, south and east of the line, facing different ways after the drive.
    STARTS = [(1.5, 1.0), (1.5, -1.0), (2.5, 0.0), (0.8, 1.2), (0.8, -1.2)]

    def test_rate(self):
        p = self.probe
        self.ready()
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 0}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 0}})
        results = []
        for run in range(RATE):
            x, y = self.STARTS[run % len(self.STARTS)]
            for _ in range(3):
                if p.drive(x, y):
                    break
            else:
                self.fail(f"run {run}: no drive to ({x}, {y})")
            since = len(p.events)
            self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}}, f"run {run}")
            end = p.wait(lambda: next((e for e in p.events[since:] if e["type"] in ("DOCKED", "DOCKING_FAILED")),
                                      None), 180)
            self.assertIsNotNone(end, f"run {run}: {p.events[since:]}")
            self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 10))
            truth = p.truth()
            results.append({"run": run, "start": [x, y], "docked": end["type"] == "DOCKED",
                            "message": end.get("message", ""), "x": round(truth["x"], 3), "y": round(truth["y"], 3),
                            "yaw_deg": round(math.degrees(truth["yaw"]), 1), "dock_ok": truth["dock_ok"]})
            print(f"docking run {run}: {json.dumps(results[-1])}", flush=True)
        docked = sum(r["docked"] for r in results)
        print(f"docking rate: {docked} of {len(results)} tries docked (one try each, faults {json.dumps(FAULTS)})",
              flush=True)
        # Each docked try ends on the contacts.
        self.assertTrue(all(r["dock_ok"] for r in results if r["docked"]), results)
        self.assertGreaterEqual(docked, 0.7 * len(results), results)
