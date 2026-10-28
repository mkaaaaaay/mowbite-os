"""The map of MowBite OS behind the app interface: mowbite_map and mowbite_bridge on a private mosquitto. The test plays
the app (MQTT), the logic (the services) and tron_gnss (the datum).

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_map.py
"""
import hashlib
import json
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Point
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import MapStatus, RpcMethods
from mowbite_msgs.srv import AddArea, ReplaceMap, SetDock
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

MQTT_PORT = 18832
DATUM = (52.0, 9.0, 50.0)
METRES_PER_DEGREE = 111320.0
WORK = tempfile.mkdtemp(prefix="mowbite_map_")
MAP_DIR = os.path.join(WORK, "map")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# The map the app sends: open rings are fine, the mower closes them.
APP_MAP = {
    "areas": [
        {"id": "lawn", "properties": {"name": "Front", "type": "mow", "angle": 0.35, "planner": {"pattern": "stripes"}},
         "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}]},
        {"id": "tree", "properties": {"type": "obstacle"},
         "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}]},
    ],
    "docking_stations": [{"id": "dock", "position": {"x": -1.5, "y": 3.0}, "heading": 3.14}],
}


def generate_test_description():
    os.makedirs(MAP_DIR)
    with open(os.path.join(WORK, "mosquitto.conf"), "w") as f:
        f.write(f"listener {MQTT_PORT} 127.0.0.1\nallow_anonymous true\n")
    mosquitto = ExecuteProcess(cmd=["mosquitto", "-c", os.path.join(WORK, "mosquitto.conf")], output="screen")
    bridge = Node(package="mowbite_bridge", executable="mowbite_bridge_node", output="screen",
                  parameters=[{"mqtt_host": "127.0.0.1", "mqtt_port": MQTT_PORT, "data_dir": os.path.join(WORK, "bridge")}])
    return launch.LaunchDescription([
        mosquitto,
        TimerAction(period=1.0, actions=[bridge, launch_testing.actions.ReadyToTest()]),
    ]), {}


def area_request(kind, name, points):
    request = AddArea.Request()
    request.type = kind
    request.name = name
    request.outline = [Point(x=float(x), y=float(y)) for x, y in points]
    return request


def replace_request(map_json, expected_sha256=""):
    request = ReplaceMap.Request()
    request.map = map_json if isinstance(map_json, str) else json.dumps(map_json)
    request.expected_sha256 = expected_sha256
    return request


def moved(north_m):
    return (DATUM[0] + north_m / METRES_PER_DEGREE, DATUM[1], DATUM[2])


class Probe:
    """The ROS and MQTT sides of the test."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ros = {}
        self.mqtt = {}
        self.events = []
        self.node = rclpy.create_node("map_probe")
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.methods_pub = self.node.create_publisher(RpcMethods, "bridge/rpc/methods", QoSProfile(
            depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, "map/json", lambda m: self._ros("map", m), LATCHED)
        self.node.create_subscription(MapStatus, "map/status", lambda m: self._ros("status", m), LATCHED)
        self.add_area = self.node.create_client(AddArea, "map/add_area")
        self.set_dock = self.node.create_client(SetDock, "map/set_dock")
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "map_probe")
        except AttributeError:
            self.client = mqtt.Client("map_probe")
        self.client.on_message = self._mqtt
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def connect(self, timeout=15.0):
        end = time.time() + timeout
        while True:
            try:
                self.client.connect("127.0.0.1", MQTT_PORT)
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(0.2)
        self.client.subscribe("#")
        self.client.loop_start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _ros(self, key, msg):
        with self.lock:
            self.ros[key] = msg

    def _mqtt(self, _c, _u, msg):
        with self.lock:
            self.mqtt[msg.topic] = msg.payload
            if msg.topic == "events/json":
                self.events.append(json.loads(msg.payload))

    def publish_datum(self, datum):
        msg = NavSatFix()
        msg.latitude, msg.longitude, msg.altitude = datum
        self.datum_pub.publish(msg)

    def wait(self, getter, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value:
                return value
            time.sleep(0.05)
        return None

    def rpc(self, method, params=None, timeout=10.0):
        rid = f"{method}-{time.time_ns()}"
        request = {"jsonrpc": "2.0", "method": method, "id": rid}
        if params is not None:
            request["params"] = params
        with self.lock:
            self.mqtt.pop("rpc/response", None)
        self.client.publish("rpc/request", json.dumps(request))
        # Answers of other calls overwrite each other, so a test asks one thing at a time.
        answer = self.wait(lambda: self.mqtt.get("rpc/response") if b'"%s"' % rid.encode() in self.mqtt.get("rpc/response", b"") else None, timeout)
        return json.loads(answer) if answer else None

    def call(self, client, request, timeout=10.0):
        self.node.get_logger().info("calling " + client.srv_name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.client.loop_stop()
        self.node.destroy_node()


def map_node_path():
    return os.path.join(FindPackagePrefix("mowbite_map").find("mowbite_map"), "lib", "mowbite_map", "mowbite_map_node")


class TestMap(unittest.TestCase):
    process = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.connect()
        cls.probe.publish_datum(DATUM)

    @classmethod
    def tearDownClass(cls):
        cls.stop_map()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def start_map(cls):
        cls.probe.ros.pop("map", None)
        cls.probe.ros.pop("status", None)
        cls.process = subprocess.Popen(
            [map_node_path(), "--ros-args", "-p", f"data_dir:={MAP_DIR}"], stdout=subprocess.DEVNULL)

    @classmethod
    def stop_map(cls):
        if cls.process is not None:
            cls.process.send_signal(signal.SIGTERM)
            try:
                cls.process.wait(10)
            except subprocess.TimeoutExpired:
                cls.process.kill()
            cls.process = None

    def status(self, state=None):
        return self.probe.wait(lambda: self.probe.ros.get("status")
                               if self.probe.ros.get("status") is not None
                               and (state is None or self.probe.ros["status"].datum_state == state) else None)

    def map_json(self, areas):
        """The published map once it has this many areas."""
        message = self.probe.wait(lambda: self.probe.ros.get("map")
                                  if self.probe.ros.get("map") is not None
                                  and len(json.loads(self.probe.ros["map"].data)["areas"]) == areas else None)
        self.assertIsNotNone(message, f"no map/json with {areas} areas")
        return json.loads(message.data)

    def test_01_registers_its_methods_at_the_bridge(self):
        self.start_map()
        methods = None
        end = time.time() + 20
        while time.time() < end and methods is None:
            answer = self.probe.rpc("rpc.methods")
            if answer and "map.replace" in answer["result"]:
                methods = answer["result"]
        self.assertIsNotNone(methods, "map.replace never registered")
        self.assertIn("map.area_properties", methods)
        answer = self.probe.rpc("map.area_properties")
        self.assertEqual(answer["result"], ["name", "type", "active", "mowable", "angle", "outline_count",
                                            "outline_overlap_count", "outline_offset"])

    def test_02_replace_stores_closes_the_rings_and_publishes(self):
        self.assertEqual(self.status(MapStatus.DATUM_EMPTY).mowing_allowed, False)
        answer = self.probe.rpc("map.replace", [APP_MAP])
        self.assertEqual(answer["result"], "Successfully stored map (2 areas)")
        published = self.map_json(2)
        self.assertEqual(len(published["areas"][0]["outline"]), 5)
        self.assertEqual(published["areas"][0]["outline"][0], published["areas"][0]["outline"][-1])
        # What the planner writes into the properties comes back.
        self.assertEqual(published["areas"][0]["properties"]["planner"], {"pattern": "stripes"})
        status = self.status(MapStatus.DATUM_MATCH)
        self.assertIsNotNone(status)
        self.assertTrue(status.mowing_allowed)
        self.assertEqual((status.areas, status.docking_stations), (2, 1))
        # The app gets the same through the bridge, retained for the next phone.
        retained = self.probe.wait(lambda: self.probe.mqtt.get("map/json")
                                   if b'"planner"' in self.probe.mqtt.get("map/json", b"") else None)
        self.assertIsNotNone(retained, "no retained map/json")
        self.assertEqual(len(json.loads(retained)["areas"]), 2)

    def test_03_an_invalid_map_is_refused_with_the_reason(self):
        bad = json.loads(json.dumps(APP_MAP))
        bad["areas"][1]["outline"] = bad["areas"][1]["outline"][:2]
        answer = self.probe.rpc("map.replace", [bad])
        self.assertEqual(answer["error"]["code"], -32602)
        self.assertEqual(answer["error"]["message"], "Invalid map: areas[1] 'tree': outline needs at least 3 points")
        answer = self.probe.rpc("map.replace", [])
        self.assertEqual(answer["error"]["message"], "Missing map parameter")
        self.assertEqual(len(json.loads(self.probe.ros["map"].data)["areas"]), 2)

    def test_04_the_logic_adds_an_area_and_sets_the_dock(self):
        response = self.probe.call(self.probe.add_area, area_request("nav", "Path", [(0, 0), (3, 0), (3, 1)]))
        self.assertIsNotNone(response, "no answer from map/add_area")
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.id), 32)
        published = self.map_json(3)
        self.assertEqual(published["areas"][2]["id"], response.id)
        self.assertEqual(published["areas"][2]["properties"], {"name": "Path", "type": "nav"})

        response = self.probe.call(self.probe.add_area, area_request("garden", "Odd", [(0, 0), (3, 0), (3, 1)]))
        self.assertFalse(response.success)
        self.assertIn("unknown type 'garden'", response.message)

        dock = SetDock.Request()
        dock.x, dock.y, dock.heading = 2.5, -1.0, 1.57
        response = self.probe.call(self.probe.set_dock, dock)
        self.assertTrue(response.success, response.message)
        self.map_json(3)
        status = self.status(MapStatus.DATUM_MATCH)
        self.assertEqual((status.areas, status.docking_stations), (3, 1))

    def test_05_a_restart_brings_the_map_back_and_says_nothing_about_the_datum(self):
        before = json.loads(self.probe.ros["map"].data)
        events = len(self.probe.events)
        self.stop_map()
        self.start_map()
        self.assertEqual(self.map_json(3), before)
        self.assertEqual(self.status(MapStatus.DATUM_MATCH).mowing_allowed, True)
        time.sleep(2.5)
        self.assertEqual([e for e in self.probe.events[events:] if e.get("type") == "MAP_DATUM"], [])

    def test_06_another_datum_stops_mowing_and_every_change_but_emptying(self):
        self.stop_map()
        self.probe.publish_datum(moved(1000.0))
        self.start_map()
        status = self.status(MapStatus.DATUM_MISMATCH)
        self.assertIsNotNone(status, "no mismatch")
        self.assertFalse(status.mowing_allowed)
        self.assertIn("m apart", status.reason)
        event = self.probe.wait(lambda: next((e for e in self.probe.events if e.get("type") == "MAP_DATUM"), None))
        self.assertIsNotNone(event, "no MAP_DATUM event")
        self.assertEqual(event["state"], "mismatch")
        self.assertEqual(event["reason"], "MAP_MISMATCH")
        self.assertEqual(event["severity"], "error")
        # The map stays as it was, but nothing may change it.
        self.assertEqual(len(self.map_json(3)["areas"]), 3)
        answer = self.probe.rpc("map.replace", [APP_MAP])
        self.assertEqual(answer["error"]["code"], -32000)
        self.assertIn("send an empty map to start over", answer["error"]["message"])
        response = self.probe.call(self.probe.add_area, area_request("mow", "More", [(0, 0), (2, 0), (2, 2)]))
        self.assertFalse(response.success)
        self.assertIn("recorded for the datum", response.message)
        # The way out.
        answer = self.probe.rpc("map.replace", [{"areas": [], "docking_stations": []}])
        self.assertIn("Successfully", answer["result"])
        self.assertIsNotNone(self.status(MapStatus.DATUM_EMPTY))
        response = self.probe.call(self.probe.add_area, area_request("mow", "New", [(0, 0), (2, 0), (2, 2)]))
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.status(MapStatus.DATUM_MATCH).mowing_allowed)

    def test_07_the_properties_of_the_planner_are_reported_while_it_offers_its_settings(self):
        def stored():
            return self.probe.rpc("map.area_properties")["result"]

        self.assertEqual(len(stored()), 8)
        offer = RpcMethods()
        offer.node_id = "mowbite_planner"
        offer.methods = ["planner.settings", "planner.plan"]
        self.probe.methods_pub.publish(offer)
        end = time.time() + 10
        while time.time() < end and len(stored()) != 11:
            time.sleep(0.2)
        self.assertEqual(stored()[8:], ["mow_around", "angle_min", "angle_max"])
        offer.methods = []
        self.probe.methods_pub.publish(offer)
        end = time.time() + 10
        while time.time() < end and len(stored()) != 8:
            time.sleep(0.2)
        self.assertEqual(len(stored()), 8)

    def test_08_the_planner_replaces_the_map_through_map_replace_with_a_version_check(self):
        read = self.probe.ros["map"].data
        version = hashlib.sha256(read.encode()).hexdigest()
        planned = json.loads(read)
        planned["areas"][0]["properties"]["planner"] = {"pattern": "stripes"}
        response = self.probe.call(self.probe.replace_map, replace_request(planned, version))
        self.assertIsNotNone(response, "no answer from map/replace")
        self.assertTrue(response.success, response.message)
        self.assertEqual((response.code, response.message), (0, "Successfully stored map (1 areas)"))
        written = self.probe.wait(lambda: self.probe.ros["map"].data
                                  if '"planner"' in self.probe.ros["map"].data else None)
        self.assertIsNotNone(written, "map/json without the planner settings")
        retained = self.probe.wait(lambda: self.probe.mqtt.get("map/json")
                                   if b'"planner"' in self.probe.mqtt.get("map/json", b"") else None)
        self.assertIsNotNone(retained, "retained map/json without the planner settings")

        # A second writer with the version of before is too late and changes nothing.
        late = json.loads(read)
        late["areas"][0]["properties"]["name"] = "Late"
        response = self.probe.call(self.probe.replace_map, replace_request(late, version))
        self.assertFalse(response.success)
        self.assertEqual((response.code, response.message), (-32001, "map changed meanwhile"))
        self.assertEqual(self.probe.ros["map"].data, written)

        # With the version of the map as it is now it goes through.
        response = self.probe.call(self.probe.replace_map,
                                   replace_request(late, hashlib.sha256(written.encode()).hexdigest()))
        self.assertTrue(response.success, response.message)
        self.assertIsNotNone(self.probe.wait(lambda: '"Late"' in self.probe.ros["map"].data or None))

        response = self.probe.call(self.probe.replace_map, replace_request("{not json"))
        self.assertEqual((response.code, response.message), (-32602, "Invalid map: not valid JSON"))
        response = self.probe.call(self.probe.replace_map, replace_request(late, "abc"))
        self.assertEqual(response.code, -32602)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
