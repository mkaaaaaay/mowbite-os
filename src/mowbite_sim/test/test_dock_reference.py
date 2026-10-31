"""The docking station in the simulation (stage 4, step 1): the whole stack of sim.launch.py with the station of
fake_tron 1 m behind the start. The test records the station like a user with the remote control (a point in front,
then backwards onto the contacts until the driver reports them), then starts mowing on the contacts: the logic sets the
heading of the station in the localization and compares the position with the recorded one (G24). A check that passes
leads to undocking (test_undock.py); a moved station or a poor position refuses the start, without the mower moving.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_reference.py
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
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, HardwareStatus, MapStatus, MqttPublish, RobotState
from mowbite_msgs.srv import DriveTo, ReplaceMap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_dock_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "dock.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
            "mowbite_localization:\n  ros__parameters:\n    antenna_offset_x: 0.098\n    antenna_offset_y: 0.0\n"
            # The station of sim.yaml: the docked mower at (-1, 0), backing in going west.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18835", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def received(since):
    """Sub packets fake_tron got from the host since its time since."""
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


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.map = None
        self.pose = None
        self.robot = None
        self.hw = None
        self.raw_since = None
        self.contact_since = None
        self.events = []
        self.node = rclpy.create_node("dock_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.teleop_pub = self.node.create_publisher(Twist, "bridge/teleop", 10)
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
            now = time.time()
            if msg.dock_contact_raw and self.raw_since is None:
                self.raw_since = now
            if msg.dock_contact and self.contact_since is None:
                self.contact_since = now

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def truth(self):
        try:
            with open(STATE) as f:
                return json.load(f)
        except (OSError, ValueError):
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

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def replace(self, docks):
        request = ReplaceMap.Request()
        request.map = json.dumps(garden(docks))
        return self.call(self.replace_map, request)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


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
        # Forwards and back: the GPS course aligns the heading, which both points need.
        p.teleop(0.2, 2.5)
        self.assertTrue(p.wait(lambda: p.pose.orientation_valid, 10), "the heading did not align")
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
        self.assertTrue(p.wait(lambda: p.pose.orientation_valid and abs(p.pose.vehicle_heading) < math.radians(3), 5),
                        p.pose.vehicle_heading)
        p.act(ABORT)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        # Back onto the contacts by hand for the checks that follow.
        p.flags(place=[-1.0, 0.0, 0.0])
        self.assertTrue(p.wait(lambda: p.hw.dock_contact and abs(p.truth()["x"] + 1.0) < 0.001, 10), p.truth())
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy <= 0.03
                               and abs(p.pose.pose.pose.position.x + 1.0) < 0.05, 20), p.pose.pose.pose.position)

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
