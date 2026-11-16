"""Area recording of mowbite_logic against mowbite_map: the test plays the driver (hw/status, hw/wheel_odom), the
localization (localization/pose), tron_gnss (gps/datum) and the app (actions on bridge/action), and reads map/json and
what the logic sends to MQTT through bridge/publish.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_recording.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import TwistStamped
from launch_ros.actions import Node
from mowbite_msgs.msg import (ActionList, HardwareStatus, MapStatus, MqttPublish, RobotPose, RpcError, RpcRequest,
                              RpcResponse)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, LATCHED, SimProbe, layers, read_json  # noqa: E402

DATUM = (52.0, 9.0, 50.0)
WORK = tempfile.mkdtemp(prefix="mowbite_recording_")
REC = "mower_logic:area_recording/"


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = layers.geometry("tron")


def generate_test_description():
    map_node = Node(package="mowbite_map", executable="mowbite_map_node", output="screen",
                    parameters=[{"data_dir": WORK}])
    logic = Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen",
                 parameters=[GEOMETRY, {"map_edit_timeout": 3.0}])
    return launch.LaunchDescription([map_node, logic, launch_testing.actions.ReadyToTest()]), {}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("recording_probe")
        self.status = None
        self.overlay = None
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        # m/s of the wheels, negative backwards
        self.speed = 0.0
        self.contact = False
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.wheel_pub = self.node.create_publisher(TwistStamped, "hw/wheel_odom", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(ActionList, "bridge/actions", self._actions, HISTORY)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.start()
        self.spawn(self._keepalive)

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
        # The wheels with every pose: without them no ring keeps its body.
        wheels = TwistStamped()
        wheels.twist.linear.x = self.speed
        self.wheel_pub.publish(wheels)
        msg = RobotPose()
        msg.x = x
        msg.y = y
        msg.heading = self.heading
        msg.position_accuracy = accuracy
        msg.heading_accuracy = 0.05
        msg.heading_valid = True
        self.pose_pub.publish(msg)

    def drive(self, corners):
        """Forward along the corners in steps of 5 cm, a pose at every step, turning on the spot at each corner."""
        self.speed = 10.0
        for (ax, ay), (bx, by) in zip(corners, corners[1:]):
            self.heading = math.atan2(by - ay, bx - ax)
            steps = max(1, math.ceil(math.hypot(bx - ax, by - ay) / 0.05))
            for s in range(steps + 1):
                self.x = ax + (bx - ax) * s / steps
                self.y = ay + (by - ay) * s / steps
                self.pose(self.x, self.y)
                time.sleep(0.005)
        self.speed = 0.0
        time.sleep(0.2)

    def back(self, x, y):
        """Straight back to the point in steps of 1 cm at 0.5 m/s, the heading kept."""
        ax, ay = self.x, self.y
        steps = max(1, math.ceil(math.hypot(x - ax, y - ay) / 0.01))
        self.speed = -0.5
        for s in range(1, steps + 1):
            self.x = ax + (x - ax) * s / steps
            self.y = ay + (y - ay) * s / steps
            self.pose(self.x, self.y)
            time.sleep(0.02)
        self.speed = 0.0
        time.sleep(0.2)

    def events_of(self, kind):
        return [e for e in self.events if e["type"] == kind]


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
        # Over the start at the end, crossing the first side flatly: only so the closing edge was driven.
        self.probe.drive([(0, 0), (4, 0), (4, 3), (0, 3), (0, 0.15), (0.6, -0.05)])
        line = self.probe.wait(lambda: [p for p in (self.probe.overlay or {}).get("polygons", [])
                                        if p["color"] == "blue" and len(p["poly"]) > 50])
        self.assertTrue(line, "no blue line on map_overlay/json")
        self.probe.act(REC + "stop_recording")
        self.assertTrue(self.enabled(REC + "finish_mowing_area"))
        # To the obstacle first: a ring begins where the mower stands.
        self.probe.drive([(0.6, -0.05), (1, 1)])
        self.probe.act(REC + "start_recording")
        self.assertTrue(self.enabled(REC + "stop_recording"))
        self.probe.drive([(1, 1), (2, 1), (2, 2), (1, 2), (1, 1.15), (1.6, 0.95)])
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
        # Both rings were driven forward without a break: they carry the body of the platform and the id of this
        # installation.
        self.assertTrue(recorded[-1]["driven"])
        self.assertNotIn("driven_reason", recorded[-1])
        self.assertEqual(recorded[-1]["obstacles_driven"], 1)
        bodies = [area["properties"].get("recorded_body") for area in saved["areas"]]
        self.assertEqual(bodies[0], bodies[1])
        self.assertEqual(set(bodies[0]), {"robot_width", "robot_front", "robot_rear", "robot_outline", "by"})
        self.assertRegex(bodies[0]["by"], "^[0-9a-f]{32}$")
        geometry = self.probe.rpc("robot.geometry.get")["result"]
        for key in ("robot_width", "robot_front", "robot_rear"):
            self.assertAlmostEqual(bodies[0][key], geometry["settings"][key]["value"], places=6)
        on_disk = read_json(os.path.join(WORK, "map.json"))
        self.assertEqual(on_disk["areas"][0]["properties"]["recorded_body"], bodies[0])
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

    def test_05_a_new_body_follows_on_the_areas_recorded_here(self):
        self.probe.datum(DATUM)
        self.assertTrue(self.probe.wait(lambda: self.probe.status.datum_state == MapStatus.DATUM_MATCH))
        before = self.probe.map["areas"][0]["properties"]["recorded_body"]
        geometry = self.probe.rpc("robot.geometry.get")["result"]
        width = round(geometry["settings"]["robot_width"]["value"] + 0.02, 3)
        answer = self.probe.rpc("robot.geometry.set", {"version": geometry["version"],
                                                       "settings": {"robot_width": width}})
        self.assertIn("result", answer, answer)
        moved = self.probe.wait(lambda: self.probe.map if all(
            area["properties"]["recorded_body"]["robot_width"] == width for area in self.probe.map["areas"])
            else None)
        self.assertIsNotNone(moved, self.probe.map)
        for area in moved["areas"]:
            self.assertEqual(area["properties"]["recorded_body"]["by"], before["by"])
            self.assertEqual(area["properties"]["recorded_body"]["robot_front"], before["robot_front"])
        self.assertEqual(self.probe.events_of("RECORDED_BODY_KEPT"), [])

    def test_06_points_by_hand_give_no_body(self):
        self.probe.act("mower_logic:idle/start_area_recording")
        self.assertTrue(self.enabled(REC + "start_recording"))
        self.probe.act(REC + "start_recording")
        self.assertTrue(self.enabled(REC + "auto_point_collecting_disable"))
        self.probe.act(REC + "auto_point_collecting_disable")
        self.assertTrue(self.enabled(REC + "collect_point"))
        corners = [(10, 0), (12, 0), (12, 2), (10, 2)]
        for corner in corners:
            self.probe.drive([(self.probe.x, self.probe.y), corner])
            self.probe.act(REC + "collect_point")
            time.sleep(0.5)
        since = len(self.probe.events_of("AREA_RECORDED"))
        self.probe.act(REC + "finish_mowing_area")
        recorded = self.probe.wait(lambda: self.probe.events_of("AREA_RECORDED")[since:])
        self.assertIsNotNone(recorded, "no AREA_RECORDED")
        self.assertFalse(recorded[0]["driven"])
        self.assertEqual(recorded[0]["driven_reason"], "MANUAL_POINT")
        saved = self.probe.wait(lambda: next((a for a in self.probe.map["areas"] if a["id"] == recorded[0]["area_id"]),
                                             None))
        self.assertIsNotNone(saved)
        self.assertNotIn("recorded_body", saved["properties"])

    def test_07_reversing_gives_no_body(self):
        self.probe.act("mower_logic:idle/start_area_recording")
        self.assertTrue(self.enabled(REC + "start_recording"))
        self.probe.drive([(self.probe.x, self.probe.y), (15, 0)])
        self.probe.act(REC + "start_recording")
        # test_06 turned them off; only automatic points are taken back.
        self.assertTrue(self.enabled(REC + "auto_point_collecting_enable"))
        self.probe.act(REC + "auto_point_collecting_enable")
        self.assertTrue(self.enabled(REC + "stop_recording"))
        since = len(self.probe.events)
        self.probe.drive([(15, 0), (17, 0)])
        # 5 cm, under the two steps the pose needs: the wheels show it.
        self.probe.back(16.95, 0)
        self.probe.drive([(16.95, 0), (18, 0), (18, 2), (15, 2), (15, 0.15), (15.6, -0.05)])
        hint = self.probe.wait(lambda: [e for e in self.probe.events[since:]
                                        if e["type"] == "RECORDING_REFUSED" and e["reason"] == "REVERSING"])
        self.assertIsNotNone(hint, "no hint while reversing")
        recorded_since = len(self.probe.events_of("AREA_RECORDED"))
        self.probe.act(REC + "finish_mowing_area")
        recorded = self.probe.wait(lambda: self.probe.events_of("AREA_RECORDED")[recorded_since:])
        self.assertIsNotNone(recorded, "no AREA_RECORDED")
        self.assertFalse(recorded[0]["driven"])
        self.assertEqual(recorded[0]["driven_reason"], "REVERSED")
        saved = self.probe.wait(lambda: next((a for a in self.probe.map["areas"] if a["id"] == recorded[0]["area_id"]),
                                             None))
        self.assertIsNotNone(saved)
        self.assertNotIn("recorded_body", saved["properties"])
