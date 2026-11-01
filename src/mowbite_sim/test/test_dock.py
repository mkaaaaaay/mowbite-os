"""Docking in the simulation (stage 4, step 3): the whole stack of sim.launch.py with the station of fake_tron and in
the map. A job started on the station undocks, mows a small patch and docks after JOB_COMPLETE; abort_mowing docks as
well, and so does docking.start from IDLE in the field. On the way the mower backs along the line of the station, stops
at the gate, creeps onto the contacts and stops at the first raw contact: after it the driver sends three zero frames
and then nothing while it stands (G18). docking.start is refused with the reason in the data of the error and as
DRIVE_REFUSED; abort_docking and an emergency stop it without DOCKING_FAILED; a station 4 cm beside its place in the
map gets DOCKING_RETRY and DOCKING_FAILED (docking_retry_count of params.set), the mower waits at the approach point.

MOWBITE_DOCK_RATE=20 instead counts how many of 20 tries dock with the faults of a real Tron in fake_tron (RTK noise, a
late fix, wheels reporting too little, a dead band, slip): a first answer to open question 10, for the simulation only.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock.py
"""
import json
import math
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, HardwareStatus, MapStatus, MqttPublish, RobotState, RpcError, RpcRequest
from mowbite_msgs.msg import RpcResponse
from mowbite_msgs.srv import DriveTo, ReplaceMap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

RATE = int(os.environ.get("MOWBITE_DOCK_RATE") or 0)
# fake_tron flags of a real Tron as in test_full_run.py: RTK about 1 cm rms, the fix 0.1 s late, wheels report 6 % too
# little, a wheel below 0.05 m/s stands, the base turns on 0.395 m while the MCU counts 0.40 m, slip of the omni wheels.
FAULTS = {"gnss_noise": 0.01, "gnss_delay": 0.1, "odom_scale": 0.94, "wheel_deadband": 0.05, "track": 0.395,
          "yaw_drift": 0.01} if RATE else {}

WORK = tempfile.mkdtemp(prefix="mowbite_dock_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "dock.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
            "mowbite_logic:\n  ros__parameters:\n    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18838", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def received(since):
    """Sub packets and charge frames fake_tron got from the host since its time since."""
    out = []
    try:
        with open(RX) as f:
            for line in f:
                entry = json.loads(line)
                if entry.get("t", 0) >= since and "mod" in entry:
                    out.append(entry)
    except OSError:
        pass
    return out


def sim_time():
    try:
        with open(RX) as f:
            lines = f.readlines()
    except OSError:
        return 0.0
    return json.loads(lines[-1])["t"] if lines else 0.0


def moving(entry):
    return bool(entry.get("speed")) and any(v != 0 for v in entry["speed"])


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.map = None
        self.pose = None
        self.robot = None
        self.hw = None
        self.events = []
        self.answers = {}
        self.node = rclpy.create_node("dock_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.rpc_count = 0
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _robot(self, msg):
        with self.lock:
            self.robot = msg

    def _hw(self, msg):
        with self.lock:
            self.hw = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _response(self, msg):
        with self.lock:
            self.answers[msg.id] = {"result": json.loads(msg.result)}

    def _error(self, msg):
        with self.lock:
            self.answers[msg.id] = {"error": {"code": msg.code, "message": msg.message,
                                              "data": json.loads(msg.data) if msg.data else None}}

    def truth(self):
        for _ in range(5):
            try:
                with open(STATE) as f:
                    return json.load(f)
            except (OSError, ValueError):
                time.sleep(0.05)
        return None

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def call(self, client, request, timeout=60.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def rpc(self, method, params=None, timeout=10.0):
        """The answer of the logic as the bridge would pass it on: {"result": ...} or {"error": {...}}."""
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_count += 1
        rid = f"dock-{self.rpc_count}"
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params) if params is not None else "",
                                        id=rid))
        return self.wait(lambda: self.answers.get(rid), timeout)

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(dict(FAULTS, **values), f)
        os.replace(FLAGS + ".tmp", FLAGS)

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
        """Carried onto the contacts; waits for the debounced contact and an RTK position fit for the check."""
        self.flags()
        time.sleep(0.3)
        self.flags(place=DOCKED, **flags)
        assert self.wait(lambda: self.hw and self.hw.dock_contact, 10), "no contact on the station"
        assert self.wait(lambda: self.pose and self.pose.position_accuracy <= 0.03
                         and abs(self.pose.pose.pose.position.x + 1.0) < 0.05, 30), "no RTK position there"
        assert self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 10)
        self.flags(**flags)

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

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


def g18(test, rx, docked_at, stood_until):
    """After the last wheel command onto the contacts three zero frames, then no SpeedData while the mower stands."""
    last = max(i for i, e in enumerate(rx) if moving(e))
    test.assertLess(rx[last]["speed"][0], 0.0, rx[last])
    after = [e for e in rx[last + 1:] if "speed" in e]
    test.assertEqual(len(after), 3, after)
    test.assertTrue(all(e["speed"] == [0.0, 0.0] for e in after), after)
    test.assertLess(after[-1]["t"], docked_at, (after, docked_at))
    test.assertGreater(stood_until - docked_at, 4.0)


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

    def assert_still(self, before, seconds=1.0):
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
        # Charging after the debounced contact.
        self.assertTrue(p.wait(lambda: any(e.get("charge") == 1 and e["t"] >= docked_at - 1.0
                                           for e in received(sim_since)), 5))
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
        self.assertEqual(answer, {"result": {"state": "DOCKING"}})
        self.assertIsNotNone(p.event(since, "STATE", 5, state="DOCKING"))
        self.assertIsNotNone(p.event(since, "DOCKED", 120), p.events[since:])
        self.on_the_station()

    def test_05_abort_docking(self):
        p = self.probe
        before = len(p.events)
        self.assertIsNotNone(p.drive(1.5, -0.8), "the drive did not end")
        # A request right after DOCKED is refused for a moment, it does not fail the undocking.
        self.assertNotIn("UNDOCKING_FAILED", [e["type"] for e in p.events[before:]])
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING"}})
        # On the line, backwards.
        self.assertTrue(p.wait(lambda: p.truth()["x"] < -0.3, 90), p.events[since:])
        p.act(ABORT)
        self.assertIsNotNone(p.event(since, "STATE", 3, state="IDLE"), p.events[since:])
        time.sleep(0.5)
        self.assert_still(p.truth())
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_FAILED", types)
        self.assertNotIn("DOCKED", types)

    def test_06_an_emergency_while_docking(self):
        p = self.probe
        since = len(p.events)
        # From where abort_docking left it, on the line: out to the approach point first.
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING"}})
        self.assertTrue(p.wait(lambda: p.truth()["x"] < -0.4 and p.truth()["linear"] < 0, 60), p.events[since:])
        p.flags(stop=1)
        self.assertIsNotNone(p.event(since, "EMERGENCY", 3, emergency=True), p.events[since:])
        self.assertIsNotNone(p.event(since, "STATE", 3, state="IDLE"), p.events[since:])
        self.assert_still(p.truth())
        p.flags()
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5))
        # It never goes on by itself.
        self.assert_still(p.truth(), 3.0)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_FAILED", types)
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_07_a_station_beside_its_place_in_the_map(self):
        p = self.probe
        answer = p.rpc("params.set", {"params": {"/mower_logic/docking_retry_count": 1}})
        self.assertEqual(answer, {"result": {"/mower_logic/docking_retry_count": 1}})
        p.flags(dock_shift_y=0.04)
        sim_since, since = sim_time(), len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING"}})
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
            self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING"}}, f"run {run}")
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
