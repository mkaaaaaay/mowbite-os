"""The sounds of MowBite OS behind the app interface: mowbite_sound with its file output and mowbite_bridge on a private
mosquitto. The test plays the app (MQTT), the driver (hw/status) and the logic (bridge/robot_state, events/json), and
reads the start and end of every sound from the file. Uploads go through the real converter.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_sound.py
"""
import base64
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
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareStatus, MqttPublish, RobotState

MQTT_PORT = 18835
WORK = tempfile.mkdtemp(prefix="mowbite_sound_")
DATA_DIR = os.path.join(WORK, "data")
VENDOR = os.path.join(WORK, "vendor")
PLAYED = os.path.join(WORK, "played")
CODES = {"MOWING_START": "900001", "MOWING_COMPLETE": "900102", "EMERGENCY": "900017", "LIFT_TRIGGERED": "900018"}
LIFT = 1 << 3


def generate_test_description():
    os.makedirs(DATA_DIR)
    os.makedirs(os.path.join(VENDOR, "v1.3.32"))
    for code in CODES.values():
        shutil.copy(tone_path(), os.path.join(VENDOR, "v1.3.32", f"{code}.mp3"))
    with open(os.path.join(WORK, "mosquitto.conf"), "w") as f:
        f.write(f"listener {MQTT_PORT} 127.0.0.1\nallow_anonymous true\n")
    mosquitto = ExecuteProcess(cmd=["mosquitto", "-c", os.path.join(WORK, "mosquitto.conf")], output="screen")
    bridge = Node(package="mowbite_bridge", executable="mowbite_bridge_node", output="screen",
                  parameters=[{"mqtt_host": "127.0.0.1", "mqtt_port": MQTT_PORT, "data_dir": os.path.join(WORK, "bridge")}])
    return launch.LaunchDescription([
        mosquitto,
        TimerAction(period=1.0, actions=[bridge, launch_testing.actions.ReadyToTest()]),
    ]), {}


def tone_path():
    return os.path.join(FindPackageShare("mowbite_sound").find("mowbite_sound"), "test_data", "tone.mp3")


def sound_node_path():
    return os.path.join(FindPackagePrefix("mowbite_sound").find("mowbite_sound"), "lib", "mowbite_sound",
                        "mowbite_sound_node")


class TestSound(unittest.TestCase):
    process = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sound_probe")
        cls.state_pub = cls.node.create_publisher(RobotState, "bridge/robot_state", 10)
        cls.status_pub = cls.node.create_publisher(HardwareStatus, "hw/status", 10)
        cls.event_pub = cls.node.create_publisher(MqttPublish, "bridge/publish", 100)
        cls.lock = threading.Lock()
        cls.answers = {}
        try:
            cls.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "sound_probe")
        except AttributeError:
            cls.client = mqtt.Client("sound_probe")
        cls.client.on_message = cls.on_message
        end = time.time() + 15
        while True:
            try:
                cls.client.connect("127.0.0.1", MQTT_PORT)
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(0.2)
        cls.client.subscribe("rpc/response")
        cls.client.loop_start()
        codes = [f"{situation}={code}" for situation, code in CODES.items()]
        cls.process = subprocess.Popen(
            [sound_node_path(), "--ros-args", "-p", "output:=file", "-p", f"file:={PLAYED}", "-p",
             f"data_dir:={DATA_DIR}", "-p", f"vendor_dirs:=['{VENDOR}/*']", "-p",
             "vendor_codes:=[" + ",".join(f"'{c}'" for c in codes) + "]"], stdout=subprocess.DEVNULL)

    @classmethod
    def tearDownClass(cls):
        if cls.process is not None:
            cls.process.send_signal(signal.SIGTERM)
            try:
                cls.process.wait(10)
            except subprocess.TimeoutExpired:
                cls.process.kill()
        cls.client.loop_stop()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def on_message(cls, client, userdata, message):
        try:
            answer = json.loads(message.payload)
        except ValueError:
            return
        with cls.lock:
            cls.answers[answer.get("id")] = answer

    def rpc(self, method, params=None, timeout=10.0):
        rid = f"{method}-{time.time_ns()}"
        request = {"jsonrpc": "2.0", "method": method, "id": rid}
        if params is not None:
            request["params"] = params
        self.client.publish("rpc/request", json.dumps(request))
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if rid in self.answers:
                    return self.answers.pop(rid)
            time.sleep(0.05)
        return None

    def publish_state(self, emergency=False, bits=0, charging=False):
        msg = RobotState()
        msg.current_state = "IDLE"
        msg.emergency = emergency
        msg.emergency_reason = bits
        msg.is_charging = charging
        msg.battery_percentage = 0.8
        self.state_pub.publish(msg)

    def publish_status(self, emergency=False, bits=0):
        msg = HardwareStatus()
        msg.emergency = emergency
        msg.emergency_reason = bits
        self.status_pub.publish(msg)

    def publish_event(self, event_type, **details):
        msg = MqttPublish()
        msg.topic = "events/json"
        msg.payload = json.dumps({"id": str(time.time_ns()), "t": time.time(), "type": event_type, **details})
        self.event_pub.publish(msg)

    def played(self):
        try:
            with open(PLAYED) as f:
                return [json.loads(line) for line in f]
        except FileNotFoundError:
            return []

    def wait_line(self, predicate, timeout=5.0, after=0):
        end = time.time() + timeout
        while time.time() < end:
            lines = self.played()[after:]
            for line in lines:
                if predicate(line):
                    return line
            time.sleep(0.01)
        self.fail(f"no matching line in {self.played()[after:]}")

    def test_1_offers_its_methods_with_the_vendors_sounds(self):
        methods = None
        end = time.time() + 30
        while time.time() < end and methods is None:
            answer = self.rpc("rpc.methods")
            if answer and "sound.list" in answer.get("result", []):
                methods = answer["result"]
            time.sleep(0.2)
        self.assertIsNotNone(methods, "sound.list never registered")
        for method in ("sound.play", "sound.stop", "sound.volume", "sound.upload", "sound.delete"):
            self.assertIn(method, methods)
        listed = self.rpc("sound.list")["result"]
        self.assertEqual(len(listed["situations"]), 12)
        with_file = {s["name"] for s in listed["situations"] if s["file"]}
        self.assertEqual(with_file, set(CODES))
        self.assertEqual(listed["volume"], 60)
        self.assertTrue(os.path.isfile(os.path.join(DATA_DIR, "sounds", "MOWING_START.wav")))
        # The first values after the start only set the scene.
        for _ in range(3):
            self.publish_state()
            self.publish_status()
            time.sleep(0.1)

    def test_2_a_job_is_announced(self):
        before = len(self.played())
        self.publish_event("STATE", state="MOWING")
        self.wait_line(lambda l: l == {"event": "start", "situation": "MOWING_START", "gain": 0.36, "rate": 24000},
                       after=before)
        self.wait_line(lambda l: l["event"] == "done" and l["situation"] == "MOWING_START", after=before)
        refused = self.rpc("sound.play", {"situation": "DOCKING_FAILED"})
        self.assertEqual(refused["error"]["code"], -32600)
        self.assertEqual(refused["error"]["data"]["reason"], "NO_FILE")

    def test_3_the_emergency_ends_a_sound_and_speaks_alone(self):
        before = len(self.played())
        self.assertIn("result", self.rpc("sound.play", {"situation": "MOWING_COMPLETE"}))
        self.wait_line(lambda l: l["event"] == "start" and l["situation"] == "MOWING_COMPLETE", after=before)
        sent = time.time()
        self.publish_status(True, 1)
        self.wait_line(lambda l: l["event"] == "stopped" and l["situation"] == "MOWING_COMPLETE", after=before)
        self.assertLess(time.time() - sent, 0.1)
        self.wait_line(lambda l: l["event"] == "start" and l["situation"] == "EMERGENCY", after=before)
        refused = self.rpc("sound.stop")
        self.assertEqual(refused["error"]["data"]["reason"], "EMERGENCY")
        self.publish_event("JOB_COMPLETE")
        # A lift joins: the emergency's own sound gives way to LIFT_TRIGGERED.
        self.publish_status(True, 1 | LIFT)
        self.wait_line(lambda l: l["event"] == "start" and l["situation"] == "LIFT_TRIGGERED", after=before)
        time.sleep(1.5)
        self.assertFalse(any(l["situation"] == "MOWING_COMPLETE" and l["event"] == "start"
                             for l in self.played()[before + 1:]))
        self.publish_status()
        self.publish_state()

    def test_4_volume_zero_is_silent(self):
        self.assertEqual(self.rpc("sound.volume", {"volume": 0})["result"], {"volume": 0})
        self.assertEqual(self.rpc("sound.volume")["result"], {"volume": 0})
        time.sleep(0.5)
        before = len(self.played())
        self.assertIn("result", self.rpc("sound.play", {"situation": "MOWING_START"}))
        self.publish_status(True, 1)
        time.sleep(1.0)
        self.assertEqual(self.played()[before:], [])
        self.publish_status()
        with open(os.path.join(DATA_DIR, "sounds", "sounds.json")) as f:
            self.assertEqual(json.load(f)["volume"], 0)

    def upload(self, situation, data, parts=2):
        """All parts of data, the answer to the last one."""
        size = -(-len(data) // parts)
        sha256 = hashlib.sha256(data).hexdigest()
        for part in range(parts):
            answer = self.rpc("sound.upload", {"situation": situation, "part": part, "parts": parts, "sha256": sha256,
                                               "data": base64.b64encode(data[part * size:(part + 1) * size]).decode()},
                              timeout=40.0)
            if part + 1 < parts:
                self.assertEqual(answer.get("result"), {"received": part}, answer)
        return answer

    def test_5_an_upload_plays_until_it_is_deleted(self):
        self.assertEqual(self.rpc("sound.volume", {"volume": 60})["result"], {"volume": 60})
        with open(tone_path(), "rb") as f:
            tone = f.read()
        sha256 = hashlib.sha256(tone).hexdigest()
        answer = self.upload("RAIN_DETECTED", tone)
        self.assertEqual(answer["result"]["situation"], "RAIN_DETECTED")
        self.assertEqual(answer["result"]["source"], "upload")
        self.assertEqual(answer["result"]["sha256"], sha256)
        self.assertAlmostEqual(answer["result"]["duration_s"], 1.0, delta=0.15)
        rows = {row["name"]: row["file"] for row in self.rpc("sound.list")["result"]["situations"]}
        self.assertEqual(rows["RAIN_DETECTED"]["source"], "upload")
        self.assertEqual(rows["RAIN_DETECTED"]["size"], len(tone))

        before = len(self.played())
        self.publish_event("DOCKING", reason="RAIN")
        self.wait_line(lambda l: l == {"event": "start", "situation": "RAIN_DETECTED", "gain": 0.36, "rate": 24000},
                       after=before)

        # A file that is no MP3 leaves the sound before alone.
        refused = self.upload("RAIN_DETECTED", b"no mp3 at all" * 100)
        self.assertEqual(refused["error"]["code"], -32600)
        self.assertEqual(refused["error"]["data"]["reason"], "NOT_MP3")
        rows = {row["name"]: row["file"] for row in self.rpc("sound.list")["result"]["situations"]}
        self.assertEqual(rows["RAIN_DETECTED"]["sha256"], sha256)

        deleted = self.rpc("sound.delete", {"situation": "RAIN_DETECTED"})["result"]
        self.assertEqual(deleted, {"situation": "RAIN_DETECTED", "source": None, "file": None})
        self.assertEqual(self.rpc("sound.play", {"situation": "RAIN_DETECTED"})["error"]["data"]["reason"], "NO_FILE")
        self.assertEqual(os.listdir(os.path.join(DATA_DIR, "sounds", "upload")), [])
