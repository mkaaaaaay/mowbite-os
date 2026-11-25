"""What the sims of the obstacle hold on the whole stack share: a square lawn, the things of the scene fake_obstacles
reports, the platform file of the simulated Tron with the camera stereo, and a probe that records the true track."""
import json
import math
import os
import shutil
import signal
import subprocess
import tempfile
import time

import rclpy
from mowbite_msgs.msg import MapStatus, MqttPublish, ObstacleStatus, RobotPose, RobotState, RpcError, RpcResponse
from mowbite_msgs.msg import RpcRequest
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

from sim_common import LATCHED, SimProbe, sim_files, sim_program, stop_process, wrap, write_json

# The ball of the runs: its radius and height, and how far its edge lies ahead of the front when it is placed (m).
BALL_RADIUS = 0.1
BALL_HEIGHT = 0.2
BALL_GAP = 1.6


def thing(kind, name, x, y, radius, height):
    return {"kind": kind, "class_name": name, "score": 0.9, "x": x, "y": y, "radius": radius, "height": height}


def ball(x, y):
    return thing("OBJECT", "ball", x, y, BALL_RADIUS, BALL_HEIGHT)


def person(x, y):
    return thing("PERSON", "person", x, y, 0.25, 1.7)


def dog(x, y):
    return thing("ANIMAL", "dog", x, y, 0.2, 0.5)


def point(t, ahead, left=0.0):
    """The map point ahead and left of base_link."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    return t["x"] + ahead * c - left * s, t["y"] + ahead * s + left * c


class Lawn:
    """A lawn between two corners with the mower starting near the first."""

    def __init__(self, low, high):
        self.low, self.high = low, high
        self.ring = [low, (high[0], low[1]), high, (low[0], high[1])]

    def garden(self):
        return {"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
                           "outline": [{"x": x, "y": y} for x, y in self.ring]}],
                "docking_stations": []}

    def room_ahead(self, t):
        """m from base_link along the heading to the edge of the lawn."""
        c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
        room = math.inf
        for lo, hi, p, d in ((self.low[0], self.high[0], t["x"], c), (self.low[1], self.high[1], t["y"], s)):
            if d > 1e-6:
                room = min(room, (hi - p) / d)
            elif d < -1e-6:
                room = min(room, (lo - p) / d)
        return room

    def inner(self, x, y):
        """m from x, y to the nearest edge of the lawn."""
        return min(x - self.low[0], self.high[0] - x, y - self.low[1], self.high[1] - y)


class Stage:
    """The files of one sim: the simulation, the platform file with the camera stereo and obstacle_mode_max, and the
    scene of fake_obstacles, empty at first."""

    def __init__(self, name, mode_max):
        self.work = tempfile.mkdtemp(prefix=f"mowbite_{name}_")
        self.state, self.flags, self.rx, self.data = sim_files(self.work)
        self.config = os.path.join(self.work, f"{name}.yaml")
        self.scene = os.path.join(self.work, "scene.json")
        with open(self.config, "w") as f:
            f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
                    "    allow_blade: true\n"
                    f"    obstacle_sources: [\"stereo\"]\n    obstacle_mode_max: {mode_max}\n"
                    "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n"
                    "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")
        self.place()

    def place(self, *things):
        write_json(self.scene, {"obstacles": list(things)})

    def fake_obstacles(self):
        return subprocess.Popen([sim_program("fake_obstacles"), "--ros-args", "-p", f"state:={self.state}", "-p",
                                 f"flags:={self.flags}", "-p", f"scene:={self.scene}"])


class ObstacleProbe(SimProbe):
    """The probe on the stack: map, pose, robot state and obstacles status, actions, RPC, and the true track."""

    def __init__(self, name, stage, lawn):
        super().__init__(name, state=stage.state, flags=stage.flags, log=stage.rx)
        self.lawn = lawn
        self.status = None
        self.pose = None
        self.robot = None
        self.obstacles = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(ObstacleStatus, "obstacles/status", self._set("obstacles"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()
        self.spawn(self._record)

    def speed(self, span=1.0):
        """m/s of the true pose over the last span s (five poses a second)."""
        with self.lock:
            track = list(self.track[-int(span * 5) - 1:])
        if len(track) < 2:
            return 0.0
        return math.hypot(track[-1]["x"] - track[0]["x"], track[-1]["y"] - track[0]["y"]) / (0.2 * (len(track) - 1))

    def straight_run(self, room, timeout=120.0, ahead_inner=0.0):
        """The true pose once the mower mows straight ahead with that much lawn before it, and the point room/2 ahead
        at least ahead_inner from every edge; None if it never does."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                track = list(self.track[-6:])
            turned = abs(wrap(track[-1]["yaw"] - track[0]["yaw"])) if len(track) == 6 else 1
            if (turned < 0.03 and self.robot is not None and self.robot.current_state == "MOWING"
                    and self.speed() > 0.25 and self.lawn.room_ahead(track[-1]) >= room
                    and self.lawn.inner(*point(track[-1], room / 2)) >= ahead_inner):
                return track[-1]
            time.sleep(0.2)
        return None


class ObstacleSim:
    """Mixin of the TestCase of such a sim: STAGE, LAWN and PROBE_NAME set by the test; probe and fake_obstacles run
    for the class."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = ObstacleProbe(cls.PROBE_NAME, cls.STAGE, cls.LAWN)
        cls.probe.flags()
        cls.fake = cls.STAGE.fake_obstacles()

    @classmethod
    def tearDownClass(cls):
        if cls.fake.poll() is None:
            stop_process(cls.fake, signal.SIGINT)
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(cls.STAGE.work, ignore_errors=True)

    def paused_for(self, since, kind, timeout=10.0, resume=None):
        state = self.probe.event(since, "STATE", timeout, state="PAUSED")
        self.assertIsNotNone(state, [e["type"] for e in self.probe.events[since:]])
        self.assertEqual(state["reason"], "OBSTACLE", state)
        self.assertEqual(state["kind"], kind, state)
        if resume is not None:
            self.assertEqual(state["resume"], resume, state)
        return state

    def mow_on_the_lawn(self):
        """The garden as the map, a pose with a heading at the origin, sight, and mowing with the blade on."""
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30))
        request = ReplaceMap.Request()
        request.map = json.dumps(self.LAWN.garden())
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")
        self.assertTrue(p.wait(lambda: p.obstacles is not None and p.obstacles.sight, 20), "the camera sees nothing")
        p.started(lambda: p.act("mower_logic:idle/start_mowing"))
        self.assertIsNotNone(p.event(0, "BLADES", 120, enabled=True), "the blade never ran")
