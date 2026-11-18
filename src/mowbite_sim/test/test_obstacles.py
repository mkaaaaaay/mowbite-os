"""mowbite_obstacles behind the app interface: the node on its own with mowbite_bridge on a private mosquitto and
fake_obstacles reporting the things of a scene. The test plays the platform (hw/capabilities), the localization
(localization/pose and the state file of fake_tron that fake_obstacles reads) and the logic (its settings and the
geometry on bridge/params), and the app over MQTT.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_obstacles.py
"""
import base64
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import TimerAction
from mowbite_msgs.msg import HardwareCapabilities, ObstacleStatus, RobotPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (bridge_node, mosquitto, mqtt_connect, mqtt_rpc, package_program, sim_program,  # noqa: E402
                        stop_process, store_answer, wait_until, write_json)

MQTT_PORT = 18854
WORK = tempfile.mkdtemp(prefix="mowbite_obstacles_")
DATA_DIR = os.path.join(WORK, "data")
PHOTOS = os.path.join(DATA_DIR, "obstacles")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
SCENE = os.path.join(WORK, "scene.json")
METHODS = ["vision.obstacles.clear", "vision.obstacles.image", "vision.obstacles.list"]
# A ball 0.9 m ahead of the nose of the Tron at the origin, a person on the left, a dog behind: the mower sees one at
# a time by its heading.
BALL = (1.6, 0.0)
PERSON = (0.0, 3.0)
DOG = (-3.0, 0.0)
SCENE_THINGS = {"obstacles": [
    {"kind": "OBJECT", "class_name": "ball", "score": 0.9, "x": BALL[0], "y": BALL[1], "radius": 0.1, "height": 0.2},
    {"kind": "PERSON", "class_name": "person", "score": 0.9, "x": PERSON[0], "y": PERSON[1], "radius": 0.25,
     "height": 1.7},
    {"kind": "ANIMAL", "class_name": "dog", "score": 0.9, "x": DOG[0], "y": DOG[1], "radius": 0.2, "height": 0.5}]}
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def generate_test_description():
    os.makedirs(DATA_DIR)
    write_json(STATE, {"x": 0.0, "y": 0.0, "yaw": 0.0})
    write_json(FLAGS, {})
    write_json(SCENE, SCENE_THINGS)
    return launch.LaunchDescription([
        mosquitto(WORK, MQTT_PORT),
        TimerAction(period=1.0, actions=[bridge_node(WORK, MQTT_PORT), launch_testing.actions.ReadyToTest()]),
    ]), {}


def program(path, **params):
    args = [arg for name, value in params.items() for arg in ("-p", f"{name}:={value}")]
    return subprocess.Popen([path, "--ros-args", *args])


class TestObstacles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("obstacles_probe")
        cls.lock = threading.Lock()
        cls.answers = {}
        cls.events = []
        cls.status = None
        cls.photos_wanted = None
        cls.yaw = 0.0
        cls.capabilities_pub = cls.node.create_publisher(HardwareCapabilities, "hw/capabilities", LATCHED)
        cls.params_pub = cls.node.create_publisher(String, "bridge/params", LATCHED)
        cls.geometry_pub = cls.node.create_publisher(String, "bridge/params", LATCHED)
        cls.pose_pub = cls.node.create_publisher(RobotPose, "localization/pose", 10)
        cls.node.create_subscription(ObstacleStatus, "obstacles/status", cls.on_status, 10)
        cls.node.create_subscription(Bool, "obstacles/photos_wanted", cls.on_photos_wanted, LATCHED)
        cls.node.create_timer(0.05, cls.publish_pose)
        cls.stop = threading.Event()
        cls.spinner = threading.Thread(target=cls.spin)
        cls.spinner.start()
        cls.client = mqtt_connect("obstacles_probe", MQTT_PORT, cls.on_message,
                                  [("rpc/response", 0), ("events/json", 0)])
        cls.settings(mode=1, photos=1)
        geometry = {"robot_front": 0.6, "robot_rear": 0.12, "robot_width": 0.47}
        cls.geometry_pub.publish(String(data=json.dumps({"node": "/mowbite_geometry", "params": geometry})))
        capabilities = HardwareCapabilities()
        capabilities.platform = "sim"
        capabilities.obstacle_sources = ["stereo"]
        capabilities.obstacle_mode_max = 1
        cls.capabilities_pub.publish(capabilities)
        cls.obstacles = program(package_program("mowbite_obstacles", "mowbite_obstacles_node"), data_dir=DATA_DIR)
        cls.fake = program(sim_program("fake_obstacles"), state=STATE, flags=FLAGS, scene=SCENE)

    @classmethod
    def tearDownClass(cls):
        for process in (cls.fake, cls.obstacles):
            if process.poll() is None:
                stop_process(process, signal.SIGINT)
        cls.stop.set()
        cls.spinner.join()
        cls.client.loop_stop()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def spin(cls):
        while not cls.stop.is_set():
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def settings(cls, mode, photos):
        params = {"obstacle_mode": mode, "obstacle_photos": photos}
        cls.params_pub.publish(String(data=json.dumps({"node": "/mower_logic", "params": params})))

    @classmethod
    def publish_pose(cls):
        msg = RobotPose()
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.heading = cls.yaw
        msg.heading_valid = True
        msg.position_accuracy = 0.02
        cls.pose_pub.publish(msg)

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.status = msg

    @classmethod
    def on_photos_wanted(cls, msg):
        with cls.lock:
            cls.photos_wanted = msg.data

    @classmethod
    def on_message(cls, client, userdata, message):
        if message.topic != "events/json":
            store_answer(cls.answers, cls.lock, message.payload)
            return
        try:
            payload = json.loads(message.payload)
        except ValueError:
            return
        with cls.lock:
            cls.events.append(payload)

    def turn(self, yaw):
        """The mower turns on the spot at the origin: the pose and the true pose of fake_obstacles."""
        type(self).yaw = yaw
        write_json(STATE, {"x": 0.0, "y": 0.0, "yaw": yaw})

    def flags(self, **values):
        write_json(FLAGS, values)

    def rpc(self, method, params=None, timeout=10.0):
        return mqtt_rpc(self.client, self.answers, self.lock, method, params, timeout)

    def result(self, method, params=None):
        answer = self.rpc(method, params)
        self.assertIsNotNone(answer, f"no answer to {method}")
        self.assertIn("result", answer, answer)
        return answer["result"]

    def mark(self):
        with self.lock:
            return len(self.events)

    def wait_event(self, event_type, after, timeout=15.0, **having):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                found = [event for event in self.events[after:]
                         if event["type"] == event_type and having.items() <= event.items()]
            if found:
                return found[0]
            time.sleep(0.05)
        with self.lock:
            self.fail(f"no {event_type} with {having} in {self.events[after:]}")

    def status_where(self, predicate, timeout=10.0):
        def match():
            with self.lock:
                return self.status if self.status is not None and predicate(self.status) else None
        status = wait_until(match, timeout)
        if status is None:
            with self.lock:
                self.fail(f"no such obstacles/status, last {self.status}")
        return status

    def download(self, name):
        data, offset = b"", 0
        while True:
            piece = self.result("vision.obstacles.image", {"name": name, "offset": offset, "size": 500})
            chunk = base64.b64decode(piece["data"])
            if not chunk:
                return data, piece
            data += chunk
            offset += len(chunk)

    def test_1_a_ball_ahead_is_listed_with_its_photo_and_stops(self):
        methods = wait_until(lambda: set(METHODS) <= set((self.rpc("rpc.methods", timeout=1.0) or {})
                                                         .get("result", [])), 30.0)
        self.assertTrue(methods, "vision.obstacles.* never offered")
        self.assertTrue(wait_until(lambda: self.photos_wanted is True, 10.0))
        seen = self.wait_event("OBSTACLE_SEEN", 0, timeout=20.0, kind="OBJECT")
        self.assertEqual((seen["class"], seen["action"], seen["camera_id"], seen["image"]),
                         ("ball", "seen", "stereo", True))
        self.assertLess(math.hypot(seen["x"] - BALL[0], seen["y"] - BALL[1]), 0.2, seen)
        listed = self.result("vision.obstacles.list")
        self.assertEqual((listed["mode"], listed["photos"], listed["sight"]), ("show", True, {"ok": True,
                                                                                              "reason": None}))
        self.assertEqual(listed["limit"], {"entries": 200, "bytes": 20 * 1024 * 1024, "days": 14.0})
        [ball] = listed["obstacles"]
        self.assertEqual(ball["id"], seen["obstacle_id"])
        image = ball["image"]
        self.assertTrue(os.path.isfile(os.path.join(PHOTOS, image["name"])))
        data, last = self.download(image["name"])
        self.assertEqual(len(data), image["bytes"])
        self.assertEqual(hashlib.sha256(data).hexdigest(), image["sha256"])
        self.assertEqual((last["sha256"], last["type"]), (image["sha256"], "image/jpeg"))
        self.assertGreater(image["box"]["w"], 0)
        status = self.status_where(lambda s: s.stop)
        self.assertEqual((status.stop_kind, status.sight), ("OBJECT", True))
        self.assertFalse(status.slow)

    def test_2_turned_away_the_ball_stays_unseen_and_the_person_comes(self):
        before = self.mark()
        self.turn(math.pi / 2)
        person = self.wait_event("OBSTACLE_SEEN", before, kind="PERSON")
        self.assertLess(math.hypot(person["x"] - PERSON[0], person["y"] - PERSON[1]), 0.3, person)
        # The ball lies out of view now: it is not seen free, its track stays, and nothing stops ahead of the nose.
        status = self.status_where(lambda s: any(t.kind == "OBJECT" and not t.in_view for t in s.tracks))
        self.assertFalse(status.stop and status.stop_kind == "OBJECT")
        kinds = sorted(entry["kind"] for entry in self.result("vision.obstacles.list")["obstacles"])
        self.assertEqual(kinds, ["OBJECT", "PERSON"])
        # A person 2.5 m from the nose: not stopping, but slow.
        self.assertTrue(self.status_where(lambda s: s.slow).slow)

    def test_3_without_sight_it_says_why(self):
        before = self.mark()
        self.flags(obstacles_dark=True)
        status = self.status_where(lambda s: not s.sight)
        self.assertEqual((status.reason, status.source), ("TOO_DARK", "stereo"))
        sight = self.wait_event("OBSTACLE_SIGHT", before)
        self.assertEqual((sight["ok"], sight["reason"], sight["source"]), (False, "TOO_DARK", "stereo"))
        self.assertEqual(self.result("vision.obstacles.list")["sight"], {"ok": False, "reason": "TOO_DARK"})
        before = self.mark()
        self.flags(obstacles_off=True)
        self.assertEqual(self.status_where(lambda s: s.reason == "NO_SIGNAL").source, "stereo")
        self.flags()
        self.assertEqual(self.wait_event("OBSTACLE_SIGHT", before, ok=True)["ok"], True)

    def test_4_the_dog_behind_is_never_listed(self):
        kinds = [entry["kind"] for entry in self.result("vision.obstacles.list")["obstacles"]]
        self.assertNotIn("ANIMAL", kinds)

    def test_5_clear_takes_the_entries_and_their_photos(self):
        before = self.mark()
        listed = self.result("vision.obstacles.list")["obstacles"]
        person = next(entry for entry in listed if entry["kind"] == "PERSON")
        self.assertEqual(self.rpc("vision.obstacles.clear", {"id": 99999})["error"]["code"], -32602)
        self.assertEqual(self.result("vision.obstacles.clear", {"id": person["id"]}), {"removed": 1})
        cleared = self.wait_event("OBSTACLES_CLEARED", before)
        self.assertEqual(cleared["obstacle_id"], person["id"])
        self.assertEqual(self.result("vision.obstacles.clear"), {"removed": len(listed) - 1})
        self.assertIsNone(self.wait_event("OBSTACLES_CLEARED", before, obstacle_id=None)["obstacle_id"])
        self.assertEqual(self.result("vision.obstacles.list")["obstacles"], [])
        self.assertEqual([name for name in os.listdir(PHOTOS) if name.endswith(".jpg")], [])

    def test_6_off_lists_nothing_and_wants_no_photos(self):
        self.settings(mode=0, photos=1)
        self.assertTrue(wait_until(lambda: self.photos_wanted is False, 10.0))
        self.turn(0.0)
        # The ball is ahead again and confirmed by now, but nothing is listed.
        self.status_where(lambda s: s.stop)
        time.sleep(2.0)
        self.assertEqual(self.result("vision.obstacles.list")["obstacles"], [])
        self.assertEqual(self.result("vision.obstacles.list")["mode"], "off")


if __name__ == "__main__":
    unittest.main()
