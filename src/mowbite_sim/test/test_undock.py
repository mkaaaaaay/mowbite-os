"""Undocking in the simulation (stage 4, step 2): the whole stack of sim.launch.py with the station of fake_tron 1 m
behind the start and in the map. The mower is put on the station by hand (flag place). A start there checks the
reference, switches charging off, leaves the station 0.8 m straight ahead at no more than 0.15 m/s and then mows; a
drive from the station goes on after it. Refused when the way out ends outside of the areas (DOCK_UNREACHABLE), stopped
by abort_undocking, UNDOCKING_FAILED when the wheels cannot climb out of the hollows (charging goes on again) and when
the mower leaves at an angle to the station in the map (the course probe).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_undock.py
"""
import json
import math
import os
import re
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
from mowbite_msgs.msg import AbsolutePose, HardwareStatus, MapStatus, MqttPublish, RobotState
from mowbite_msgs.srv import DriveTo, ReplaceMap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_undock_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "undock.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
START = "mower_logic:idle/start_mowing"
ABORT = "mower_logic:undocking/abort_undocking"

# The lawn reaches to 0.5 m in front of the station, where the way out ends 0.2 m inside it; the far one begins 1 m in
# front of the station.
LAWN = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
FAR_LAWN = [(0.0, -2.0), (4.0, -2.0), (4.0, 2.0), (0.0, 2.0)]
DOCKED = [-1.0, 0.0, 0.0]


def garden(lawn, dock_heading=math.pi):
    return {"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                       "outline": [{"x": x, "y": y} for x, y in lawn]}],
            "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": dock_heading}]}


with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_localization:\n  ros__parameters:\n    antenna_offset_x: 0.098\n    antenna_offset_y: 0.0\n"
            # The station of sim.yaml: the docked mower at (-1, 0), backing in going west.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18836", "extra_config": CONFIG}.items())
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
        self.node = rclpy.create_node("undock_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
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

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def replace(self, lawn, dock_heading=math.pi):
        request = ReplaceMap.Request()
        request.map = json.dumps(garden(lawn, dock_heading))
        answer = self.call(self.replace_map, request)
        assert answer.success, answer.message
        assert self.wait(lambda: self.map and self.map["docking_stations"]
                         and abs(self.map["docking_stations"][0]["heading"] - dock_heading) < 1e-6
                         and self.map["areas"][0]["outline"][0]["x"] == lawn[0][0], 10), "the map did not come"

    def put_on_the_station(self, **flags):
        """Carried onto the contacts; waits for the debounced contact and an RTK position fit for the check."""
        self.flags()
        time.sleep(0.3)
        self.flags(place=DOCKED, **flags)
        assert self.wait(lambda: self.hw and self.hw.dock_contact, 10), "no contact on the station"
        assert self.wait(lambda: self.pose and self.pose.position_accuracy <= 0.03
                         and abs(self.pose.pose.pose.position.x + 1.0) < 0.05, 30), "no RTK position there"
        assert self.wait(lambda: self.robot and self.robot.current_state == "IDLE", 10)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestUndock(unittest.TestCase):
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

    def assert_still(self, before):
        after = self.probe.truth()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.005, (before, after))

    def test_01_on_the_station(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        p.replace(LAWN)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        sim_since = sim_time()
        p.put_on_the_station()
        # The driver charges on the contacts.
        self.assertTrue(p.wait(lambda: any(e.get("charge") == 1 for e in received(sim_since)), 5))

    def test_02_a_way_out_that_ends_outside_of_the_areas(self):
        p = self.probe
        p.replace(FAR_LAWN)
        before, since = p.truth(), len(p.events)
        p.act(START)
        refused = p.event(since, "MOWING_REFUSED", 10)
        self.assertIsNotNone(refused, p.events[since:])
        self.assertEqual(refused["reason"], "DOCK_UNREACHABLE", refused)
        self.assertEqual(refused["distance"], 0.8)
        self.assertEqual((refused["severity"], refused["resume"]), ("warning", "user"))
        time.sleep(1.0)
        self.assert_still(before)
        self.assertEqual(p.robot.current_state, "IDLE")
        p.replace(LAWN)

    def test_03_start_on_the_station_undocks_then_mows(self):
        p = self.probe
        sim_since, since = sim_time(), len(p.events)
        p.act(START)
        undocked = p.event(since, "UNDOCKED", 30)
        self.assertIsNotNone(undocked, p.events[since:])
        undocked_at, truth = sim_time(), p.truth()
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), p.events[since:])
        after = p.events[since:]
        self.assertEqual([e["state"] for e in after if e["type"] == "STATE"][:2], ["UNDOCKING", "MOWING"], after)
        self.assertFalse([e for e in after if e["type"] in ("MOWING_REFUSED", "UNDOCKING_FAILED")], after)
        self.assertFalse(undocked.get("reason"), undocked)
        # Charging went off before the first wheel command, three times, and not on again.
        rx = received(sim_since)
        first = next(i for i, e in enumerate(rx) if moving(e))
        offs = [i for i, e in enumerate(rx) if e.get("charge") == 0]
        self.assertEqual(len(offs), 3, offs)
        self.assertLess(offs[0], first)
        self.assertFalse([e for e in rx[offs[0]:] if e.get("charge") == 1])
        # The way out: straight ahead without steering, never faster than undock_speed.
        out = [e["speed"] for e in rx[first:] if e["t"] <= undocked_at and e.get("speed")]
        self.assertTrue(out)
        self.assertLessEqual(max(v[0] for v in out), 0.15 + 1e-3, out)
        self.assertEqual(max(abs(v[1]) for v in out), 0.0, out)
        self.assertGreater(max(v[0] for v in out), 0.14, out)
        # Out by 0.7 to 0.8 m along the line, off the contacts.
        self.assertGreater(truth["x"], -0.32, truth)
        self.assertLess(truth["x"], -0.18, truth)
        self.assertLess(abs(truth["y"]), 0.01, truth)
        self.assertFalse(truth["dock_ok"], truth)
        # Stopping the job docks again (test_dock.py); here it is stopped on the way.
        since = len(p.events)
        p.act("mower_logic:mowing/abort_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="DOCKING"), p.events[since:])
        p.act("mower_logic:docking/abort_docking")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="IDLE"), p.events[since:])

    def test_04_a_drive_from_the_station_goes_on_after_undocking(self):
        p = self.probe
        p.put_on_the_station()
        since = len(p.events)
        request = DriveTo.Request()
        request.x, request.y = 1.0, 0.5
        answer = p.call(p.drive_to, request)
        self.assertTrue(answer.success, answer.message)
        self.assertIn("driving", answer.message)
        types = [e["type"] for e in p.events[since:]]
        self.assertIn("UNDOCKED", types)
        self.assertLess(types.index("UNDOCKED"), types.index("DRIVE"))
        end = p.event(since, "DRIVE", 60, phase="end")
        self.assertIsNotNone(end, p.events[since:])
        self.assertTrue(end["arrived"], end)
        truth = p.truth()
        self.assertLess(math.hypot(truth["x"] - 1.0, truth["y"] - 0.5), 0.2, truth)

    def test_05_abort_on_the_way_out(self):
        p = self.probe
        p.put_on_the_station()
        since = len(p.events)
        p.act(START)
        self.assertTrue(p.wait(lambda: p.truth()["x"] > -0.9, 20), p.events[since:])
        p.act(ABORT)
        self.assertIsNotNone(p.event(since, "STATE", 3, state="IDLE"), p.events[since:])
        time.sleep(0.5)
        stopped = p.truth()
        time.sleep(1.0)
        self.assert_still(stopped)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("UNDOCKING_FAILED", types)
        self.assertNotIn("UNDOCKED", types)
        self.assertFalse([e for e in p.events[since:] if e["type"] == "STATE" and e["state"] == "MOWING"])

    def test_06_stuck_in_the_hollows(self):
        p = self.probe
        p.put_on_the_station(dock_hollow_speed=9.0)
        sim_since, since = sim_time(), len(p.events)
        p.act(START)
        failed = p.event(since, "UNDOCKING_FAILED", 20)
        self.assertIsNotNone(failed, p.events[since:])
        failed_at = sim_time()
        self.assertEqual(failed["reason"], "path_failed", failed)
        self.assertIn("does not move", failed["message"])
        self.assertEqual((failed["severity"], failed["resume"]), ("error", "user"))
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 3))
        # It never pushed harder than undock_speed and stands on the contacts, which charge again.
        rx = received(sim_since)
        first = next(e["t"] for e in rx if moving(e))
        self.assertLess(failed_at - first, 4.0)
        self.assertLessEqual(max(e["speed"][0] for e in rx if e.get("speed")), 0.15 + 1e-3)
        truth = p.truth()
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.005)
        self.assertTrue(truth["dock_ok"])
        self.assertTrue(p.wait(lambda: any(e.get("charge") == 1 and e["t"] > first for e in received(sim_since)), 5))
        self.assertTrue(p.wait(lambda: p.hw.battery_current < 0, 5), p.hw.battery_current)

    def test_07_a_station_turned_against_the_map(self):
        p = self.probe
        p.put_on_the_station()
        # The map says the way in is 15 deg further round than the real station: the mower leaves at that angle to
        # the line it expects, which the reference check alone does not see (3 cm with the antenna 0.1 m ahead).
        p.replace(LAWN, math.pi + math.radians(15.0))
        since = len(p.events)
        p.act(START)
        failed = p.event(since, "UNDOCKING_FAILED", 20)
        self.assertIsNotNone(failed, p.events[since:])
        self.assertEqual(failed["reason"], "path_failed", failed)
        drove = re.search(r"drove (\d+) deg off the line of the docking station", failed["message"])
        self.assertIsNotNone(drove, failed["message"])
        self.assertAlmostEqual(int(drove.group(1)), 15, delta=2)
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 3))
        truth = p.truth()
        # Stopped soon after the probe began, straight on the line of the real station.
        self.assertLess(truth["x"], -0.5, truth)
        self.assertLess(abs(truth["y"]), 0.01, truth)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("UNDOCKED", types)
        p.replace(LAWN)
