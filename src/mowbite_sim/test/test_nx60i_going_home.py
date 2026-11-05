"""Going home on the NX60i (O11) as nx60i.launch.py starts it, against fake_xcore, with a safety measurement given in
mower_params (test_nx60i_stack.py runs without one and cannot mow). The mower docks with the contacts in front, which
the logic does not build yet: it has no way home. A start with the blade motor at motor_cold_temperature or more is
refused, the default temperature of fake_xcore lies below it; while mowing a hot blade motor changes nothing (question
47 iii, told once in the log), a low battery ends the run with BATTERY_LOW and IDLE where the mower stands, without
DOCKING. rain_mode other than 0 without the rain sensor of hw/capabilities is told once in the log.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_nx60i_going_home.py
"""
import json
import math
import os
import random
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import (HardwareCapabilities, MapStatus, MqttPublish, RobotPose, RobotState, Rtcm, RpcError,
                              RpcRequest, RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_nx60i_home_")
FLAGS = os.path.join(WORK, "flags.json")
STATE = os.path.join(WORK, "state.json")
DATA = os.path.join(WORK, "data")
MOWER = os.path.join(WORK, "mower.yaml")
EXTRA = os.path.join(WORK, "extra.yaml")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
DATUM = (52.0, 9.0, 80.0)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
START = "mower_logic:idle/start_mowing"
P = "/mower_logic/"
LAWN = [(-2.0, -2.5), (6.0, -2.5), (6.0, 2.5), (-2.0, 2.5)]
# nx60i.yaml: empty 24.5 V (the highest voltage of 20 s); motor_hot/cold_temperature 70/40 of the logic.
EMPTY, LOW_WINDOW = 24.5, 20.0
MOTOR_COLD = 40.0

with open(FLAGS, "w") as f:
    json.dump({"gps_fix": "fix"}, f)
with open(MOWER, "w") as f:
    f.write("xcore_driver:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n")
with open(EXTRA, "w") as f:
    f.write("mowbite_logic:\n  ros__parameters:\n    gps_wait_time: 2.0\n")


def generate_test_description():
    sim = FindPackageShare("mowbite_sim").find("mowbite_sim")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim, "config", "xcore_nx60i_services.json"), "--group-port", str(GROUP_PORT),
             "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE],
        output="screen")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("mowbite_bringup").find("mowbite_bringup"),
                                                   "launch", "nx60i.launch.py")),
        launch_arguments={"interface": "lo", "bind_ip": BIND_IP, "announcement_port": str(GROUP_PORT),
                          "data_dir": DATA, "mower_params": MOWER, "extra_config": EXTRA,
                          "datum_lat": str(DATUM[0]), "datum_long": str(DATUM[1]), "datum_height": str(DATUM[2]),
                          "mqtt_port": str(random.randint(18000, 18999))}.items())
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[stack, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    flags.setdefault("gps_fix", "fix")
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


def board():
    for _ in range(5):
        try:
            with open(STATE) as f:
                return json.load(f)
        except (OSError, ValueError):
            time.sleep(0.05)
    return None


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def station_frame():
    """An RTCM 3 frame of message 1005, all fields 0: RTK fixed counts on the board only with corrections (YF-c)."""
    payload = bytes([1005 >> 4, (1005 & 0xF) << 4]) + bytes(17)
    head = bytes([0xD3, len(payload) >> 8, len(payload) & 0xFF]) + payload
    return head + crc24q(head).to_bytes(3, "big")


class Probe:
    def __init__(self):
        # Reentrant: logged() takes it again inside wait().
        self.lock = threading.RLock()
        self.status = None
        self.caps = None
        self.robot = None
        self.pose = None
        self.events = []
        self.logs = []
        self.answers = {}
        self.node = rclpy.create_node("nx60i_home_probe")
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("caps"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(Log, "/rosout", self._log, QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", QoSProfile(depth=10))
        self.node.create_timer(1.0, lambda: self.rtcm_pub.publish(Rtcm(data=list(station_frame()))))
        self.rpc_count = 0
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _set(self, name):
        def store(msg):
            with self.lock:
                setattr(self, name, msg)
        return store

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

    def _log(self, msg):
        with self.lock:
            self.logs.append(msg.msg)

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
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_count += 1
        rid = f"nx60i-home-{self.rpc_count}"
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params) if params is not None else "",
                                        id=rid))
        return self.wait(lambda: self.answers.get(rid), timeout)

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def logged(self, text):
        with self.lock:
            return [line for line in self.logs if text in line]

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestNx60iGoingHome(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_ready_on_the_lawn(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 60), p.robot)
        caps = p.wait(lambda: p.caps, 10)
        self.assertEqual(caps.platform, "yardforce_nx60i")
        self.assertFalse(caps.rain_sensor)
        self.assertEqual(caps.safety_measurement_date, "simulation")
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "no datum")
        request = ReplaceMap.Request()
        request.map = json.dumps({"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                                             "outline": [{"x": x, "y": y} for x, y in LAWN]}],
                                  "docking_stations": []})
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        truth = board()
        request = SetPose.Request()
        request.pose.position.x = truth["x"]
        request.pose.position.y = truth["y"]
        request.pose.orientation.z = math.sin(truth["yaw"] / 2)
        request.pose.orientation.w = math.cos(truth["yaw"] / 2)
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03, 40),
                        "no RTK pose with a heading")

    def test_2_a_hot_blade_motor_refuses_the_start(self):
        p = self.probe
        set_flags(blade_temperature=45.0)
        time.sleep(1.5)
        since = len(p.events)
        p.act(START)
        refused = p.event(since, "MOWING_REFUSED", 10)
        self.assertIsNotNone(refused, p.events[since:])
        self.assertEqual((refused["reason"], refused["severity"]), ("MOTOR_HOT", "warning"), refused)
        self.assertAlmostEqual(refused["temperature"], 45)
        self.assertAlmostEqual(refused["limit"], MOTOR_COLD)
        # At the default temperature of fake_xcore it starts.
        set_flags()
        time.sleep(1.5)
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 30, state="MOWING"), p.events[since:])

    def test_3_while_mowing_heat_changes_nothing_without_a_way_home(self):
        p = self.probe
        since = len(p.events)
        set_flags(blade_temperature=75.0)
        time.sleep(8.0)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING", types)
        self.assertNotIn("STATE", types)
        self.assertEqual(p.robot.current_state, "MOWING")
        self.assertEqual(len(p.logged("Mowing goes on, it cannot go home: the blade motor at 75")), 1,
                         [line for line in p.logs if "home" in line])
        set_flags()

    def test_4_a_low_battery_ends_the_run_where_it_stands(self):
        p = self.probe
        since = len(p.events)
        set_flags(battery_voltage=24.0)
        low = p.event(since, "BATTERY_LOW", LOW_WINDOW + 15)
        self.assertIsNotNone(low, p.events[since:])
        self.assertAlmostEqual(low["battery_voltage"], 24.0, places=3)
        self.assertAlmostEqual(low["limit"], EMPTY, places=3)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"), p.events[since:])
        self.assertNotIn("DOCKING", [e["type"] for e in p.events[since:]])
        before = board()
        time.sleep(2.0)
        after = board()
        self.assertLess(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 0.01, (before, after))
        set_flags()

    def test_5_rain_mode_without_a_rain_sensor_is_told_once(self):
        p = self.probe
        text = "has no effect: this platform has no rain sensor"
        self.assertEqual(p.logged(text), [])
        for mode in (1, 2):
            answer = p.rpc("params.set", {"params": {P + "rain_mode": mode}})
            self.assertEqual(answer, {"result": {P + "rain_mode": mode}})
        self.assertTrue(p.wait(lambda: p.logged(text), 5))
        time.sleep(1.0)
        self.assertEqual(len(p.logged(text)), 1, p.logged(text))
        self.assertIn("rain_mode 1", p.logged(text)[0])
        answer = p.rpc("params.set", {"params": {P + "rain_mode": 0}})
        self.assertEqual(answer, {"result": {P + "rain_mode": 0}})
