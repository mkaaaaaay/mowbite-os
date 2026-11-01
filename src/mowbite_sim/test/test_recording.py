"""Area recording of mowbite_logic against mowbite_map: the test plays the driver (hw/status), the localization
(localization/pose), tron_gnss (gps/datum) and the app (actions on bridge/action), and reads map/json and what the
logic sends to MQTT through bridge/publish.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_recording.py
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
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from mowbite_msgs.msg import AbsolutePose, ActionList, HardwareStatus, MapStatus, MqttPublish
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

DATUM = (52.0, 9.0, 50.0)
WORK = tempfile.mkdtemp(prefix="mowbite_recording_")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ACTIONS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REC = "mower_logic:area_recording/"


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "geometry", "tron.yaml")


def generate_test_description():
    map_node = Node(package="mowbite_map", executable="mowbite_map_node", output="screen",
                    parameters=[{"data_dir": WORK}])
    logic = Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen",
                 parameters=[GEOMETRY, {"map_edit_timeout": 3.0}])
    return launch.LaunchDescription([map_node, logic, launch_testing.actions.ReadyToTest()]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.map = None
        self.status = None
        self.actions = {}
        self.events = []
        self.overlay = None
        self.x = 0.0
        self.y = 0.0
        self.contact = False
        self.node = rclpy.create_node("recording_probe")
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(AbsolutePose, "localization/pose", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(ActionList, "bridge/actions", self._actions, ACTIONS_QOS)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()
        threading.Thread(target=self._keepalive, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _keepalive(self):
        # A healthy driver and a standing mower, so nothing times out between the steps.
        while self.running:
            hw = HardwareStatus()
            hw.telemetry_ok = True
            hw.battery_source = HardwareStatus.BATTERY_SOURCE_BMS
            hw.battery_voltage = 24.5
            hw.battery_percentage = 80
            hw.dock_contact = self.contact
            self.hw_pub.publish(hw)
            self.pose(self.x, self.y)
            time.sleep(0.2)

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _actions(self, msg):
        with self.lock:
            for a in msg.actions:
                self.actions[msg.node_prefix + "/" + a.action_id] = a.enabled

    def _publish(self, msg):
        with self.lock:
            if msg.topic == "events/json":
                self.events.append(json.loads(msg.payload))
            elif msg.topic == "map_overlay/json":
                self.overlay = json.loads(msg.payload)

    def datum(self, datum):
        msg = NavSatFix()
        msg.latitude, msg.longitude, msg.altitude = datum
        self.datum_pub.publish(msg)

    def pose(self, x, y, accuracy=0.02):
        msg = AbsolutePose()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.position_accuracy = accuracy
        msg.orientation_accuracy = 0.05
        msg.orientation_valid = 1
        self.pose_pub.publish(msg)

    def drive(self, corners):
        """Along the corners in steps of 5 cm, a pose at every step."""
        for (ax, ay), (bx, by) in zip(corners, corners[1:]):
            steps = max(1, math.ceil(math.hypot(bx - ax, by - ay) / 0.05))
            for s in range(steps + 1):
                self.x = ax + (bx - ax) * s / steps
                self.y = ay + (by - ay) * s / steps
                self.pose(self.x, self.y)
                time.sleep(0.005)
        time.sleep(0.2)

    def act(self, action):
        # The first action may come before the logic has subscribed; it would be lost.
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def wait(self, getter, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def events_of(self, kind):
        return [e for e in self.events if e["type"] == kind]

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestRecording(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def enabled(self, action, value=True, timeout=10.0):
        return self.probe.wait(lambda: self.probe.actions.get(action) == value, timeout)

    def test_01_no_recording_before_the_map_knows_its_datum(self):
        self.assertTrue(self.probe.wait(lambda: self.probe.status is not None
                                        and self.probe.status.datum_state == MapStatus.DATUM_UNKNOWN, 15))
        self.assertTrue(self.enabled("mower_logic:idle/start_area_recording", False))
        # Until its first hw/status the logic may be in the emergency of the start, which refuses without an event.
        refused = None
        for _ in range(10):
            self.probe.act("mower_logic:idle/start_area_recording")
            refused = self.probe.wait(lambda: self.probe.events_of("RECORDING_REFUSED"), 1.0)
            if refused:
                break
        self.assertIsNotNone(refused, "no RECORDING_REFUSED")
        self.assertEqual(refused[-1]["action"], "start_area_recording")

    def test_02_records_an_area_with_an_obstacle(self):
        self.probe.datum(DATUM)
        self.assertTrue(self.enabled("mower_logic:idle/start_area_recording"), "start_area_recording stays off")
        self.probe.act("mower_logic:idle/start_area_recording")
        self.assertTrue(self.enabled(REC + "start_recording"))
        self.probe.act(REC + "start_recording")
        self.assertTrue(self.enabled(REC + "stop_recording"))
        self.probe.drive([(0, 0), (4, 0), (4, 3), (0, 3), (0, 0.1)])
        line = self.probe.wait(lambda: [p for p in (self.probe.overlay or {}).get("polygons", [])
                                        if p["color"] == "blue" and len(p["poly"]) > 50])
        self.assertTrue(line, "no blue line on map_overlay/json")
        self.probe.act(REC + "stop_recording")
        self.assertTrue(self.enabled(REC + "finish_mowing_area"))
        self.probe.act(REC + "start_recording")
        self.assertTrue(self.enabled(REC + "stop_recording"))
        self.probe.drive([(1, 1), (2, 1), (2, 2), (1, 2), (1, 1.1)])
        self.probe.act(REC + "finish_mowing_area")

        saved = self.probe.wait(lambda: self.probe.map if self.probe.map and len(self.probe.map["areas"]) == 2
                                else None)
        self.assertIsNotNone(saved, f"map/json: {self.probe.map}")
        self.assertEqual(saved["areas"][0]["properties"]["type"], "mow")
        self.assertEqual(saved["areas"][1]["properties"]["type"], "obstacle")
        recorded = self.probe.wait(lambda: self.probe.events_of("AREA_RECORDED"))
        self.assertIsNotNone(recorded, "no AREA_RECORDED")
        self.assertEqual(recorded[-1]["area_id"], saved["areas"][0]["id"])
        self.assertEqual(recorded[-1]["obstacles"], 1)
        self.assertAlmostEqual(recorded[-1]["area_m2"], 12.0, delta=0.5)
        self.assertTrue(self.probe.wait(lambda: self.probe.overlay == {"polygons": []}), str(self.probe.overlay)[:200])
        self.assertTrue(self.enabled(REC + "start_recording"))
        # The first area ties the map to the datum.
        self.assertTrue(self.probe.wait(lambda: self.probe.status.datum_state == MapStatus.DATUM_MATCH))

    def test_03_records_the_docking_station(self):
        self.probe.drive([(self.probe.x, self.probe.y), (-1, 2.0)])
        self.probe.act(REC + "record_dock")
        self.assertTrue(self.enabled(REC + "start_recording", False))
        self.probe.drive([(-1, 2.0), (-1, 1.0)])
        # The point on the contacts needs the contact.
        self.probe.act(REC + "record_dock")
        refused = self.probe.wait(lambda: [e for e in self.probe.events_of("RECORDING_REFUSED")
                                           if e.get("reason") == "NO_DOCK_CONTACT"])
        self.assertTrue(refused, self.probe.events_of("RECORDING_REFUSED"))
        self.probe.contact = True
        time.sleep(0.5)
        self.probe.act(REC + "record_dock")
        saved = self.probe.wait(lambda: self.probe.map if self.probe.map and self.probe.map["docking_stations"]
                                else None)
        self.assertIsNotNone(saved, "no docking station in map/json")
        dock = saved["docking_stations"][0]
        self.assertAlmostEqual(dock["position"]["x"], -1.0, delta=0.01)
        self.assertAlmostEqual(dock["position"]["y"], 1.0, delta=0.01)
        self.assertAlmostEqual(dock["heading"], -math.pi / 2, delta=0.01)
        recorded = self.probe.wait(lambda: self.probe.events_of("DOCK_RECORDED"))
        self.assertIsNotNone(recorded)
        self.assertEqual(recorded[-1]["dock_id"], dock["id"])
        self.assertTrue(self.enabled(REC + "start_recording"))
        self.probe.contact = False

    def test_04_another_datum_keeps_the_area_open(self):
        self.probe.act(REC + "start_recording")
        self.assertTrue(self.enabled(REC + "stop_recording"))
        self.probe.drive([(5, 0), (7, 0), (7, 2), (5, 2), (5, 0.1)])
        self.probe.datum((DATUM[0] + 0.001, DATUM[1], DATUM[2]))
        self.assertTrue(self.probe.wait(lambda: self.probe.status.datum_state == MapStatus.DATUM_MISMATCH))
        before = len(self.probe.events_of("RECORDING_REFUSED"))
        self.probe.act(REC + "finish_mowing_area")
        refused = self.probe.wait(lambda: self.probe.events_of("RECORDING_REFUSED")[before:])
        self.assertIsNotNone(refused, "finish_mowing_area was not refused")
        self.assertEqual(refused[0]["reason"], "MAP_MISMATCH")
        self.assertIn("datum", refused[0]["message"])
        self.assertEqual(refused[0]["resume"], "user")
        time.sleep(0.5)
        self.assertEqual(len(self.probe.map["areas"]), 2)
        # Still recording: the outline can be saved once the datum is back.
        self.assertTrue(self.enabled(REC + "stop_recording"))
        self.probe.act(REC + "exit_recording_mode")
        self.assertTrue(self.enabled("mower_logic:idle/start_area_recording", False))
