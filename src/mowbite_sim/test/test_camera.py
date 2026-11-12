"""The cameras of MowBite OS behind the app interface: mowbite_camera in a process of its own and mowbite_bridge on a
private mosquitto. The test plays the app (MQTT and HTTP) and a camera (hw/cameras, camera/<id>/jpeg). The process the
camera names is a stand-in under stack.own_process, so the launch itself writes its exit note. With the stream server
in MOWBITE_MEDIAMTX a camera with webrtc is served through it, without it that camera is not set up.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_camera.py
"""
import json
import logging
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from mowbite_msgs.msg import Camera
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, os.path.join(get_package_share_directory("mowbite_bringup"), "launch"))
import stack  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (bridge_node, mosquitto, mqtt_connect, mqtt_rpc, package_program, stop_process,  # noqa: E402
                        store_answer, wait_until)

MQTT_PORT = 18844
MJPEG_PORT = 18845
WEBRTC_PORT = 18846
ICE_PORT = 18847
MEDIA_API_PORT = 18848
MEDIAMTX = os.environ.get("MOWBITE_MEDIAMTX", "")
WORK = tempfile.mkdtemp(prefix="mowbite_camera_")
DATA_DIR = os.path.join(WORK, "data")
PROCESS = "vision"
# The stand-in for the process of the camera ends with code 3 once this file is there, and takes it along.
END_FILE = os.path.join(WORK, "end")
STAND_IN = """
import os, signal, sys, time
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while not os.path.exists(sys.argv[1]):
    time.sleep(0.05)
os.remove(sys.argv[1])
sys.exit(3)
"""
# camera.list for the camera of this test while it can stream.
LISTED = {"id": "front", "name": "Front camera", "kind": "obstacle", "position": "front", "width": 640, "height": 480,
          "detections": True, "protocols": ["mjpeg"], "available": True}
METHODS = ["camera.list", "camera.stream"]
PICTURE_BYTES = 20000


def generate_test_description():
    os.makedirs(DATA_DIR)
    broker = mosquitto(WORK, MQTT_PORT)
    bridge = bridge_node(WORK, MQTT_PORT)
    vision = stack.own_process(
        lambda prefix: ExecuteProcess(cmd=[sys.executable, "-c", STAND_IN, END_FILE], prefix=prefix, output="screen"),
        PROCESS, DATA_DIR, logging.getLogger("test_camera"))
    return launch.LaunchDescription([
        broker, *vision,
        TimerAction(period=1.0, actions=[bridge, launch_testing.actions.ReadyToTest()]),
    ]), {}


def picture(number):
    """A picture of the stand-in camera: the marks of a JPEG around its number."""
    return b"\xff\xd8" + number.to_bytes(4, "big") + bytes(PICTURE_BYTES) + b"\xff\xd9"


class Viewer:
    """A client of the picture server on a connection of its own."""

    def __init__(self, url, method="GET"):
        host, port, self.target = re.fullmatch(r"http://([0-9.]+):(\d+)(/\S*)", url).groups()
        self.sock = socket.create_connection((host, int(port)), timeout=5)
        self.sock.sendall(f"{method} {self.target} HTTP/1.1\r\nHost: {host}:{port}\r\n\r\n".encode())
        self.data = b""
        self.closed = False

    def take(self, seconds=0.05):
        """What comes within seconds; notes when the server ended the connection."""
        self.sock.settimeout(seconds)
        try:
            chunk = self.sock.recv(1 << 16)
        except socket.timeout:
            return
        except OSError:
            chunk = b""
        self.closed = self.closed or not chunk
        self.data += chunk

    def head(self, timeout=5.0):
        """Status and fields of the answer; the rest stays for body() and pictures()."""
        end = time.time() + timeout
        while b"\r\n\r\n" not in self.data and not self.closed and time.time() < end:
            self.take()
        assert b"\r\n\r\n" in self.data, f"no answer to {self.target}: {self.data!r}"
        head, _, self.data = self.data.partition(b"\r\n\r\n")
        lines = head.decode().split("\r\n")
        fields = dict(line.split(": ", 1) for line in lines[1:])
        return int(lines[0].split(" ")[1]), {name.lower(): value for name, value in fields.items()}

    def pictures(self):
        """The numbers of the complete pictures that came since the last call."""
        numbers = []
        while True:
            part = re.match(rb"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: (\d+)\r\n\r\n", self.data)
            if not part:
                assert not self.data or b"--frame"[:len(self.data)] == self.data[:7], self.data[:80]
                return numbers
            end = part.end() + int(part.group(1))
            if len(self.data) < end + 2:
                return numbers
            body, tail, self.data = self.data[part.end():end], self.data[end:end + 2], self.data[end + 2:]
            assert tail == b"\r\n", tail
            number = int.from_bytes(body[2:6], "big")
            assert body == picture(number), f"picture {number} came damaged"
            numbers.append(number)

    def wait_pictures(self, count, timeout=10.0):
        numbers = []
        end = time.time() + timeout
        while len(numbers) < count and not self.closed and time.time() < end:
            self.take()
            numbers += self.pictures()
        assert len(numbers) >= count, f"{len(numbers)} of {count} pictures, closed {self.closed}"
        return numbers

    def wait_closed(self, timeout=5.0):
        end = time.time() + timeout
        while not self.closed and time.time() < end:
            self.take()
        return self.closed

    def close(self):
        self.sock.close()


def status_of(url, method="GET"):
    """The status a new connection gets for this url."""
    viewer = Viewer(url, method)
    try:
        return viewer.head()[0]
    finally:
        viewer.close()


class TestCamera(unittest.TestCase):
    process = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("camera_probe")
        # As tron_vision: the list latched, the pictures best effort and only for subscribers.
        cls.cameras_pub = cls.node.create_publisher(
            Camera, "hw/cameras", QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE,
                                             durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.jpeg_pub = cls.node.create_publisher(
            CompressedImage, "camera/front/jpeg", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.lock = threading.Lock()
        cls.answers = {}
        cls.events = []
        cls.subscribers = 0
        cls.stop = threading.Event()
        cls.camera = threading.Thread(target=cls.play_camera)
        cls.camera.start()
        cls.client = mqtt_connect("camera_probe", MQTT_PORT, cls.on_message,
                                  [("rpc/response", 0), ("events/json", 0)])
        cls.process = cls.start_node(MJPEG_PORT)

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            stop_process(cls.process, signal.SIGINT)
        cls.stop.set()
        cls.camera.join()
        cls.client.loop_stop()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def start_node(cls, port, **pipes):
        # public_host: a test container has no default route to take the address from.
        params = {"data_dir": DATA_DIR, "public_host": "127.0.0.1", "mjpeg_port": port, "webrtc_port": WEBRTC_PORT,
                  "ice_port": ICE_PORT, "media_api_port": MEDIA_API_PORT, "runtime_dir": WORK,
                  "media_server": MEDIAMTX or "/nowhere/mediamtx"}
        args = [arg for name, value in params.items() for arg in ("-p", f"{name}:={value}")]
        return subprocess.Popen([package_program("mowbite_camera", "mowbite_camera_node"), "--ros-args", *args],
                                **pipes)

    @classmethod
    def play_camera(cls):
        number = 0
        while not cls.stop.wait(0.1):
            cls.subscribers = cls.jpeg_pub.get_subscription_count()
            if cls.subscribers == 0:
                continue
            number += 1
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = picture(number)
            cls.jpeg_pub.publish(msg)

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

    def rpc(self, method, params=None, timeout=10.0):
        return mqtt_rpc(self.client, self.answers, self.lock, method, params, timeout)

    def stream(self, **params):
        """The url of camera.stream for the camera of this test."""
        answer = self.rpc("camera.stream", {"id": "front", **params})
        self.assertIn("result", answer, answer)
        return answer["result"]["url"]

    def refusal(self, **params):
        """The data of a refused camera.stream."""
        answer = self.rpc("camera.stream", {"id": "front", **params})
        self.assertEqual(answer.get("error", {}).get("code"), -32600, answer)
        return answer["error"]["data"]

    def mark(self):
        with self.lock:
            return len(self.events)

    def wait_event(self, kind, after, timeout=10.0, **having):
        """The first event of this type with these values since the mark, without id and t."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                found = [event for event in self.events[after:]
                         if event["type"] == kind and having.items() <= event.items()]
            if found:
                return {key: value for key, value in found[0].items() if key not in ("id", "t")}
            time.sleep(0.05)
        with self.lock:
            self.fail(f"no {kind} in {self.events[after:]}")

    def wait_until(self, predicate, timeout=5.0):
        return bool(wait_until(predicate, timeout))

    def report(self, available=True, reason="", detail=""):
        msg = Camera()
        msg.id = "front"
        msg.name = "Front camera"
        msg.process = PROCESS
        msg.kind = Camera.KIND_OBSTACLE
        msg.position = Camera.POSITION_FRONT
        msg.width = 640
        msg.height = 480
        msg.detections = True
        msg.protocols = [Camera.PROTOCOL_MJPEG]
        msg.available = available
        msg.reason = reason
        msg.detail = detail
        self.cameras_pub.publish(msg)

    def test_1_the_methods_come_with_the_first_camera(self):
        methods = None
        end = time.time() + 30
        while time.time() < end and methods is None:
            methods = (self.rpc("rpc.methods", timeout=1.0) or {}).get("result")
        self.assertIsNotNone(methods, "the bridge never answered")
        self.assertFalse(set(METHODS) & set(methods))
        before = self.mark()
        self.report()
        changed = self.wait_event("METHODS_CHANGED", before, timeout=20.0)
        self.assertEqual((sorted(changed["added"]), changed["removed"]), (METHODS, []))
        self.assertTrue(set(METHODS) <= set(self.rpc("rpc.methods")["result"]))
        self.assertEqual(self.wait_event("CAMERA_CHANGED", before),
                         {"type": "CAMERA_CHANGED", "camera_id": "front", "available": True, "severity": "info",
                          "resume": "auto"})
        self.assertEqual(self.rpc("camera.list")["result"], [LISTED])

    def test_2_a_viewer_gets_the_pictures_of_its_token(self):
        self.assertEqual(self.subscribers, 0)
        answer = self.rpc("camera.stream", {"id": "front"})["result"]
        self.assertEqual(sorted(answer), ["expires", "protocol", "url"])
        self.assertEqual(answer["protocol"], "mjpeg")
        self.assertIsInstance(answer["expires"], int)
        self.assertTrue(25 < answer["expires"] - time.time() <= 30, answer)
        url = answer["url"]
        self.assertRegex(url, rf"^http://127\.0\.0\.1:{MJPEG_PORT}/camera/front\.mjpg\?t=[0-9a-f]{{32}}$")
        # Neither another token nor another camera gets in, and neither uses the place up.
        wrong = url[:-1] + ("0" if url[-1] != "0" else "1")
        refused = Viewer(wrong)
        status, fields = refused.head()
        refused.close()
        self.assertEqual(status, 401)
        self.assertEqual(fields["access-control-allow-origin"], "*")
        self.assertEqual(status_of(url.replace("front.mjpg", "rear.mjpg")), 401)

        viewer = Viewer(url)
        status, fields = viewer.head()
        self.assertEqual(status, 200)
        self.assertEqual(fields["content-type"], "multipart/x-mixed-replace; boundary=frame")
        self.assertEqual(fields["access-control-allow-origin"], "*")
        numbers = viewer.wait_pictures(5)
        self.assertEqual(numbers, sorted(set(numbers)))
        self.assertEqual(self.subscribers, 1)
        # The token is good for this one connection.
        self.assertEqual(status_of(url), 401)
        self.assertGreater(viewer.wait_pictures(2)[0], numbers[-1])
        viewer.close()
        self.assertTrue(self.wait_until(lambda: self.subscribers == 0), "the picture topic is still subscribed")
        self.assertEqual(status_of(url), 401)

    def test_3_the_third_viewer_is_refused(self):
        first = Viewer(self.stream())
        self.assertEqual(first.head()[0], 200)
        second = self.stream(protocol="mjpeg")
        self.assertEqual(self.refusal(), {"event": "CAMERA_REFUSED", "reason": "CAMERA_BUSY",
                                          "message": "Two viewers are watching already", "severity": "info",
                                          "resume": "user"})
        # What the camera does not have is a wrong call, not a refusal.
        for params in ({"id": "front", "protocol": "webrtc"}, {"id": "rear"}, {}):
            self.assertEqual(self.rpc("camera.stream", params)["error"]["code"], -32602, params)
        # The place of a viewer that left is free at once; the token that still waits keeps its own.
        first.close()
        self.assertTrue(self.wait_until(lambda: "result" in (self.rpc("camera.stream", {"id": "front"}) or {})))
        self.assertEqual(self.refusal()["reason"], "CAMERA_BUSY")
        waiting = Viewer(second)
        self.assertEqual(waiting.head()[0], 200)
        waiting.wait_pictures(1)
        type(self).viewer = waiting

    def test_4_a_camera_that_cannot_stream_loses_its_viewers(self):
        before = self.mark()
        self.report(False, Camera.REASON_TOO_HOT, "SoC 81 C")
        self.assertEqual(self.wait_event("CAMERA_CHANGED", before),
                         {"type": "CAMERA_CHANGED", "camera_id": "front", "available": False, "reason": "TOO_HOT",
                          "message": "SoC 81 C", "severity": "info", "resume": "auto"})
        self.assertTrue(self.viewer.wait_closed(), "the viewer stayed connected")
        self.assertTrue(self.wait_until(lambda: self.subscribers == 0), "the picture topic is still subscribed")
        self.assertEqual(self.rpc("camera.list")["result"],
                         [{**LISTED, "available": False, "reason": "TOO_HOT", "detail": "SoC 81 C"}])
        refused = self.refusal()
        self.assertEqual((refused["reason"], refused["severity"], refused["resume"]), ("TOO_HOT", "warning", "auto"))

        before = self.mark()
        self.report()
        self.assertEqual(self.wait_event("CAMERA_CHANGED", before)["available"], True)
        # Both places are free again: also the one whose token was still waiting.
        viewers = [Viewer(self.stream()), Viewer(self.stream())]
        self.assertEqual([viewer.head()[0] for viewer in viewers], [200, 200])
        viewers[1].close()
        type(self).viewer = viewers[0]

    def test_5_the_end_of_its_process_takes_the_camera(self):
        self.viewer.wait_pictures(1)
        before = self.mark()
        started = time.monotonic()
        open(END_FILE, "w").close()
        self.assertEqual(self.wait_event("CAMERA_CHANGED", before),
                         {"type": "CAMERA_CHANGED", "camera_id": "front", "available": False, "reason": "NO_SIGNAL",
                          "message": f"process {PROCESS} ended", "severity": "info", "resume": "auto"})
        self.assertTrue(self.viewer.wait_closed(), "the viewer stayed connected")
        with open(os.path.join(DATA_DIR, f"{PROCESS}_exit.json")) as f:
            note = json.load(f)
        self.assertEqual((note["exit_code"], note["restart"], note["down"]), (3, 1, False))
        self.assertTrue(started < note["mono"] < time.monotonic(), note)
        refused = self.refusal()
        self.assertEqual((refused["reason"], refused["severity"], refused["resume"]), ("NO_SIGNAL", "warning", "user"))

        # The process is back and says so; its old note stays in data_dir and no longer counts.
        before = self.mark()
        self.report()
        self.assertEqual(self.wait_event("CAMERA_CHANGED", before)["available"], True)
        time.sleep(2.5)
        self.assertEqual(self.rpc("camera.list")["result"], [LISTED])
        viewer = Viewer(self.stream())
        self.assertEqual(viewer.head()[0], 200)
        viewer.wait_pictures(1)
        viewer.close()

    def test_6_a_camera_with_webrtc_goes_through_the_stream_server(self):
        before = self.mark()
        msg = Camera()
        msg.id = "stereo"
        msg.name = "Stereo camera"
        msg.process = "stereo"
        msg.kind = Camera.KIND_STEREO
        msg.width = 1280
        msg.height = 720
        msg.protocols = [Camera.PROTOCOL_WEBRTC]
        msg.source = "rtsp://127.0.0.1:18849/stereo"
        msg.available = True
        self.cameras_pub.publish(msg)
        listed = {"id": "stereo", "name": "Stereo camera", "kind": "stereo", "position": None, "width": 1280,
                  "height": 720, "detections": False, "protocols": ["webrtc"]}
        if not MEDIAMTX:
            changed = self.wait_event("CAMERA_CHANGED", before, camera_id="stereo", reason="NOT_SET_UP")
            self.assertEqual(changed["message"], "the stream server is not installed")
            self.assertEqual(self.rpc("camera.list")["result"],
                             [LISTED, {**listed, "available": False, "reason": "NOT_SET_UP",
                                       "detail": "the stream server is not installed"}])
            refused = self.rpc("camera.stream", {"id": "stereo"})["error"]["data"]
            self.assertEqual((refused["reason"], refused["severity"], refused["resume"]),
                             ("NOT_SET_UP", "warning", "user"))
            return
        # Not before its API answers: until then the camera waits with NO_SIGNAL.
        waiting = self.wait_event("CAMERA_CHANGED", before, camera_id="stereo")
        self.assertEqual((waiting["available"], waiting["reason"]), (False, "NO_SIGNAL"))
        self.wait_event("CAMERA_CHANGED", before, timeout=30.0, camera_id="stereo", available=True)
        self.assertEqual(self.rpc("camera.list")["result"], [LISTED, {**listed, "available": True}])
        answer = self.rpc("camera.stream", {"id": "stereo"})["result"]
        self.assertEqual(answer["protocol"], "webrtc")
        self.assertRegex(answer["url"], rf"^http://127\.0\.0\.1:{WEBRTC_PORT}/stereo/whep\?t=[0-9a-f]{{32}}$")
        # The stream server asks the node: the token that waits may ask how to connect, another one may not.
        self.assertEqual(status_of(answer["url"], "OPTIONS"), 204)
        self.assertEqual(status_of(answer["url"][:-32] + "0" * 32, "OPTIONS"), 401)
        with open(os.path.join(WORK, "mowbite_camera", "mediamtx.yml")) as f:
            self.assertIn('"stereo":\n    source: "rtsp://127.0.0.1:18849/stereo"\n', f.read())
        # The token that waits holds one place, so one more viewer fits and no third.
        self.stream()
        self.assertEqual(self.refusal()["reason"], "CAMERA_BUSY")

    def test_7_a_node_that_cannot_start_ends_with_code_1(self):
        for port, said in ((MJPEG_PORT, "cannot listen for viewers"), (70000, "mjpeg_port is 0 to 65535")):
            other = self.start_node(port, stderr=subprocess.PIPE, text=True)
            _, errors = other.communicate(timeout=30)
            self.assertEqual(other.returncode, 1, errors)
            self.assertIn(said, errors)
        self.assertEqual(self.rpc("camera.list")["result"][0], LISTED)

    def test_8_the_methods_go_with_the_node(self):
        before = self.mark()
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(10), 0)
        # The stream server went with its node.
        with self.assertRaises(OSError):
            socket.create_connection(("127.0.0.1", WEBRTC_PORT), timeout=2).close()
        changed = self.wait_event("METHODS_CHANGED", before, timeout=20.0)
        self.assertEqual((changed["added"], sorted(changed["removed"])), ([], METHODS))
        self.assertEqual(self.rpc("camera.list")["error"]["code"], -32601)
