"""Smoke tests of the sensor chain on ptys: fake_tron feeds tron_gnss and tron_imu, mowbite_localization fuses them with
standing wheels on hw/wheel_odom and mowbite_bridge publishes the result on a private mosquitto.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_stack_smoke.py
"""
import json
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import paho.mqtt.client as mqtt
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import GnssFix, RobotPose
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

MQTT_PORT = 18831
DATUM = (52.0, 9.0, 50.0)
ANTENNA_X = 0.098
WORK = tempfile.mkdtemp(prefix="mowbite_smoke_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
BRIDGE_START_S = 6.0
RETAINED_ID = "retained-1"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def generate_test_description():
    with open(os.path.join(WORK, "mosquitto.conf"), "w") as f:
        f.write(f"listener {MQTT_PORT} 127.0.0.1\nallow_anonymous true\n")
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--datum", *map(str, DATUM), "--antenna-offset", str(ANTENNA_X), "0.0"],
        output="screen")
    mosquitto = ExecuteProcess(cmd=["mosquitto", "-c", os.path.join(WORK, "mosquitto.conf")], output="screen")
    nodes = [
        Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
             parameters=[{"port": LINKS["gnss"], "datum_lat": DATUM[0], "datum_long": DATUM[1],
                          "datum_height": DATUM[2]}]),
        Node(package="tron_imu", executable="tron_imu_node", output="screen",
             parameters=[{"port": LINKS["imu"]}]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", output="screen",
             parameters=[{"gyro_calibration_time": 2.0}]),
        # Standing wheels, as a driver reports them: without a wheel report the gyro does not calibrate.
        ExecuteProcess(cmd=["ros2", "topic", "pub", "-r", "20", "hw/wheel_odom", "geometry_msgs/msg/TwistStamped",
                            "{}"], output="log"),
    ]
    bridge = Node(package="mowbite_bridge", executable="mowbite_bridge_node", output="screen",
                  parameters=[{"mqtt_host": "127.0.0.1", "mqtt_port": MQTT_PORT, "data_dir": WORK}])
    # The ptys and the broker have to exist before the drivers look for them. The bridge comes last, after the probe
    # has left a retained request on the broker.
    return launch.LaunchDescription([
        fake_tron, mosquitto,
        TimerAction(period=2.0, actions=nodes + [launch_testing.actions.ReadyToTest()]),
        TimerAction(period=BRIDGE_START_S, actions=[bridge]),
    ]), {}


class Collector:
    """Keeps the newest ROS messages and MQTT payloads and waits for them."""

    def __init__(self):
        self.ros = {}
        self.mqtt = {}
        self.rpc_ids = []
        self.lock = threading.Lock()
        self.node = rclpy.create_node("stack_smoke_probe")
        self.node.create_subscription(GnssFix, "gps/absolute_pose", lambda m: self._ros("gps", m), 10)
        self.node.create_subscription(NavSatFix, "gps/datum", lambda m: self._ros("datum", m), LATCHED)
        self.node.create_subscription(Imu, "imu/data", lambda m: self._ros("imu", m), qos_profile_sensor_data)
        self.node.create_subscription(RobotPose, "localization/pose", lambda m: self._ros("pose", m), 10)
        self.node.create_subscription(Odometry, "odom", lambda m: self._ros("odom", m), 10)
        # The logic is not part of this launch: the probe stands in for it with the antenna in the dimensions of the
        # mower, which the localization waits for before it takes a fix.
        self.geometry = self.node.create_publisher(String, "bridge/params", LATCHED)
        self.geometry.publish(String(data=json.dumps(
            {"node": "/mowbite_geometry", "params": {"antenna_x": ANTENNA_X, "antenna_y": 0.0, "version": 0}})))
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "stack_smoke")
        except AttributeError:
            self.client = mqtt.Client("stack_smoke")
        self.client.on_message = self._mqtt
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.running = True
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
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _ros(self, key, msg):
        with self.lock:
            self.ros[key] = msg

    def _mqtt(self, _c, _u, msg):
        with self.lock:
            self.mqtt[msg.topic] = msg.payload
            if msg.topic == "rpc/response":
                self.rpc_ids.append(json.loads(msg.payload).get("id"))

    def wait(self, getter, timeout=30.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = getter()
            if value is not None:
                return value
            time.sleep(0.05)
        return None

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.client.loop_stop()
        self.node.destroy_node()


class TestSensorChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Collector()
        cls.probe.connect()
        # Left on the broker before the bridge connects: it must not run when the bridge subscribes.
        request = {"jsonrpc": "2.0", "method": "rpc.ping", "id": RETAINED_ID}
        cls.probe.client.publish("rpc/request", json.dumps(request), retain=True).wait_for_publish(5.0)

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_gnss_fix_at_antenna(self):
        msg = self.probe.wait(lambda: self.probe.ros.get("gps"))
        self.assertIsNotNone(msg, "no gps/absolute_pose")
        self.assertEqual(msg.solution, GnssFix.RTK_FIXED)
        self.assertLess(msg.accuracy, 0.1)
        self.assertAlmostEqual(msg.x, ANTENNA_X, delta=0.05)
        self.assertAlmostEqual(msg.y, 0.0, delta=0.05)

    def test_gnss_publishes_its_datum_latched(self):
        # Latched, so the map finds it however late it starts.
        msg = self.probe.wait(lambda: self.probe.ros.get("datum"))
        self.assertIsNotNone(msg, "no gps/datum")
        self.assertEqual((msg.latitude, msg.longitude, msg.altitude), DATUM)

    def test_imu_is_upright_after_mount_rotation(self):
        msg = self.probe.wait(lambda: self.probe.ros.get("imu"))
        self.assertIsNotNone(msg, "no imu/data")
        self.assertAlmostEqual(msg.linear_acceleration.z, 9.81, delta=0.5)
        self.assertAlmostEqual(msg.angular_velocity.z, 0.0, delta=0.05)

    def test_localization_fuses_gps_after_calibration(self):
        # Before the gyro calibration and the first fix the pose carries accuracy 999.
        msg = self.probe.wait(lambda: self.probe.ros.get("pose") if self.probe.ros.get("pose") is not None
                              and self.probe.ros["pose"].position_accuracy < 0.1 else None, timeout=40.0)
        self.assertIsNotNone(msg, "no localization/pose with a fix")
        self.assertLess(msg.position_accuracy, 0.1)
        self.assertAlmostEqual(msg.x, 0.0, delta=0.05)
        self.assertAlmostEqual(msg.y, 0.0, delta=0.05)
        # Standing still gives no course: the heading stays unaligned.
        self.assertFalse(msg.heading_valid)
        odom = self.probe.wait(lambda: self.probe.ros.get("odom"))
        self.assertIsNotNone(odom, "no odom")
        self.assertEqual(odom.child_frame_id, "base_link")

    def test_bridge_publishes_antenna_offset_and_answers_ping(self):
        # The first params/json can come before the latched parameters of the localization, the next one has them.
        params = self.probe.wait(lambda: self.probe.mqtt.get("params/json")
                                 if b"antenna_offset_x" in self.probe.mqtt.get("params/json", b"") else None, timeout=40.0)
        self.assertIsNotNone(params, "no params/json")
        self.assertAlmostEqual(json.loads(params)["/xbot_positioning/antenna_offset_x"], ANTENNA_X, places=6)
        self.probe.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "method": "rpc.ping", "id": "p1"}))
        answer = self.probe.wait(lambda: "p1" in self.probe.rpc_ids or None)
        self.assertIsNotNone(answer, "no rpc/response")
        self.assertEqual(json.loads(self.probe.mqtt["rpc/response"])["result"], "pong")
        # The retained request reached the bridge on subscribe, before the live one, and was dropped.
        self.assertNotIn(RETAINED_ID, self.probe.rpc_ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        # SIGINT from launch is a normal stop for every process of the chain; ros2 topic pub ends on it with 2.
        for action in proc_info.processes():
            name = action.process_details["name"]
            allowed = [0, -2, -15, 130, 143] + ([2] if name.startswith("ros2") else [])
            self.assertIn(proc_info[action].returncode, allowed, name)
