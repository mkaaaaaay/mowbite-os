"""The calibration drive in the simulation: the whole stack of sim.launch.py with a Tron that is off its configuration on
purpose: wheel speeds reported 6 % short (ticks_per_m 940 against 1000), 0.395 m between the wheels against the MCU's
0.40, the antenna 0.098 m ahead of base_link where the geometry says 0, the IMU turned 2 deg in roll and -1.5 deg in
pitch on ground that rises 3 deg, RTK noise of 1 cm and 0.1 s of delay, wheels that stand below 0.05 m/s. The planner
evaluates the recording (planner.calib.evaluate): the drive ends done with its values.

Checks calibration.check, a whole drive (phase events, forward only, at most 0.3 m/s and 0.3 rad/s, the blade never on,
the recording in the format mowbite-calib/1 with its marks and rates, calibration.result) and the ends: calibration.abort,
RTK lost, no room and the bumper.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_calibration.py
"""
import gzip
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
from mowbite_msgs.msg import (CalibrationStatus, HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState,
                              RpcError, RpcMethods, RpcRequest, RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_calibration_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "calibration.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HISTORY = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

LAWN = [(-3.0, -3.0), (8.0, -3.0), (8.0, 3.0), (-3.0, 3.0)]
SMALL = [(-1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (-1.0, 1.0)]
# fake_tron off what the configuration says.
WRONG = ["--odom-scale", "0.94", "--track", "0.395", "--antenna-offset", "0.098", "0.0",
         "--imu-error-rpy-deg", "2.0", "-1.5", "0.0", "--slope", "3.0", "40.0", "--gnss-noise-m", "0.01",
         "--gnss-delay", "0.1", "--wheel-deadband", "0.05"]

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            "mowbite_logic:\n  ros__parameters:\n    geometry:\n      antenna_x: 0.0\n      antenna_y: 0.0\n"
            "fake_tron:\n  ros__parameters:\n    args: " + json.dumps(WRONG) + "\n")


def garden(lawn):
    return {"areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn"},
                       "outline": [{"x": x, "y": y} for x, y in lawn]}],
            "docking_stations": []}


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18837", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def cutter_on_since(since):
    """Cutter frames with a speed fake_tron got since its time since."""
    out = []
    try:
        with open(RX) as f:
            for line in f:
                entry = json.loads(line)
                if entry.get("t", 0) >= since and entry.get("cutter", 0) > 0:
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


def recordings():
    folder = os.path.join(DATA, "calibration", "recordings")
    try:
        return sorted(os.path.join(folder, n) for n in os.listdir(folder) if n.endswith(".jsonl.gz"))
    except OSError:
        return []


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_status = None
        self.map = None
        self.pose = None
        self.robot = None
        self.hw = None
        self.status = None
        self.events = []
        self.responses = {}
        self.errors = {}
        self.methods = {}
        # Every command on hw/cmd_vel and calibration/cmd_vel, and whether the blade showed on.
        self.commands = []
        self.blade_on = False
        self.node = rclpy.create_node("calibration_probe")
        self.node.create_subscription(MapStatus, "map/status", self._map_status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(CalibrationStatus, "calibration/status", self._status, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(Twist, "hw/cmd_vel", lambda m: self._command("hw", m), 50)
        self.node.create_subscription(Twist, "calibration/cmd_vel", lambda m: self._command("calibration", m), 50)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.calls = 0
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _map_status(self, msg):
        with self.lock:
            self.map_status = msg

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
            self.blade_on = self.blade_on or msg.blade_on

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _response(self, msg):
        with self.lock:
            self.responses[msg.id] = json.loads(msg.result)

    def _error(self, msg):
        with self.lock:
            self.errors[msg.id] = (msg.code, msg.message, json.loads(msg.data) if msg.data else None)

    def _methods(self, msg):
        with self.lock:
            self.methods[msg.node_id] = list(msg.methods)

    def _command(self, topic, msg):
        with self.lock:
            self.commands.append((time.time(), topic, msg.linear.x, msg.angular.z))

    def truth(self):
        for _ in range(5):
            try:
                with open(STATE) as f:
                    return json.load(f)
            except (OSError, ValueError):
                time.sleep(0.05)
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

    def service(self, client, request, timeout=60.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def rpc(self, method, params=None, timeout=5.0):
        """The answer, or the error as (code, message, data)."""
        self.calls += 1
        rid = f"calib_test_{self.calls}"
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params) if params is not None else "",
                                        id=rid))
        return self.wait(lambda: (("ok", self.responses[rid]) if rid in self.responses else None)
                         or (("error", self.errors[rid]) if rid in self.errors else None), timeout)

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def replace(self, lawn):
        request = ReplaceMap.Request()
        request.map = json.dumps(garden(lawn))
        answer = self.service(self.replace_map, request)
        assert answer.success, answer.message
        assert self.wait(lambda: self.map and self.map["areas"]
                         and self.map["areas"][0]["outline"][0]["x"] == lawn[0][0], 10), "the map did not come"

    def standing(self):
        a = self.truth()
        time.sleep(1.0)
        b = self.truth()
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.01, (a, b)

    def ready(self, timeout=60.0):
        """calibration.check without problems, polled."""
        end = time.time() + timeout
        last = None
        while time.time() < end:
            last = self.rpc("calibration.check")
            if last and last[0] == "ok" and last[1]["ok"]:
                return last[1]
            time.sleep(1.0)
        raise AssertionError(f"calibration.check never ok: {last}")

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestCalibration(unittest.TestCase):
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

    def start(self):
        p = self.probe
        p.ready()
        since = len(p.events)
        answer = p.rpc("calibration.start")
        self.assertEqual(answer, ("ok", {"state": "CALIBRATING"}), answer)
        self.assertIsNotNone(p.event(since, "CALIBRATION", 5, phase="start"))
        return since

    def ends_with(self, since, reason, timeout):
        p = self.probe
        failed = p.event(since, "CALIBRATION_FAILED", timeout)
        self.assertIsNotNone(failed, [e for e in p.events[since:] if e["type"].startswith("CALIBRAT")])
        self.assertEqual(failed["reason"], reason, failed)
        self.assertEqual(failed["resume"], "user")
        self.assertIsNotNone(p.event(since, "CALIBRATION", 5, phase="end"))
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"))
        time.sleep(0.5)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        return failed

    def test_01_lawn_pose_and_check(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.map_status is not None
                               and p.map_status.datum_state != MapStatus.DATUM_UNKNOWN, 30), "no datum")
        p.replace(LAWN)
        self.assertTrue(p.wait(lambda: p.map_status.mowing_allowed, 10), "the map does not fit the datum")
        truth = p.truth()
        request = SetPose.Request()
        request.pose.position.x = truth["x"]
        request.pose.position.y = truth["y"]
        request.pose.orientation.z = math.sin(truth["yaw"] / 2)
        request.pose.orientation.w = math.cos(truth["yaw"] / 2)
        p.service(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03, 40),
                        "no RTK pose with a heading")
        check = p.ready()
        self.assertEqual(check["problems"], [])
        self.assertAlmostEqual(check["space"]["lane_length"], 3.0)
        self.assertAlmostEqual(check["space"]["length"], 3.0 + 2 * (0.64 + 0.3), delta=0.05)
        self.assertLess(abs(check["space"]["heading"]), math.radians(10.5))
        self.assertIn("calibration.result", p.methods.get("mowbite_calibration", []))
        self.assertIn("calibration.start", p.methods.get("mowbite_logic", []))

    def test_02_a_whole_drive(self):
        p = self.probe
        sim_since = sim_time()
        with p.lock:
            p.commands.clear()
            p.blade_on = False
        since = self.start()
        t0 = time.time()
        done = p.event(since, "CALIBRATED", 480)
        if done is None:
            failed = [e for e in p.events[since:] if e["type"] == "CALIBRATION_FAILED"]
            self.fail(f"no CALIBRATED: {failed}")
        took = time.time() - t0
        self.assertEqual(done["status"], "done", done)
        self.assertEqual(done["severity"], "info")
        self.assertIsNotNone(p.event(since, "CALIBRATION", 5, phase="end"))
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"))
        phases = [(e["phase"], e["index"]) for e in p.events[since:] if e["type"] == "CALIBRATION"]
        for phase in [("line", 1), ("spin_left", 1), ("line", 2), ("spin_right", 2), ("line", 4), ("spin_right", 4)]:
            self.assertIn(phase, phases)
        self.assertGreater(took, 200)
        # Forward only, within 0.3 m/s and 0.3 rad/s, and the blade never on.
        with p.lock:
            commands = list(p.commands)
            blade = p.blade_on
        self.assertTrue(commands)
        self.assertTrue(all(c[2] >= 0.0 and c[2] <= 0.3 + 1e-6 and abs(c[3]) <= 0.3 + 1e-6 for c in commands),
                        [c for c in commands if c[2] < 0 or c[2] > 0.3 or abs(c[3]) > 0.3][:5])
        self.assertFalse(blade)
        self.assertEqual(cutter_on_since(sim_since), [])
        # The recording.
        files = recordings()
        self.assertEqual(len(files), 1, files)
        with gzip.open(files[0], "rt") as f:
            raw = f.read().splitlines()
        self.assertTrue(all(len(line.encode()) + 1 <= 4096 for line in raw))
        lines = [json.loads(line, parse_constant=lambda c: self.fail(f"{c} in the recording")) for line in raw]
        head = lines[0]
        self.assertEqual((head["format"], head["source"], head["mode"]), ("mowbite-calib/1", "mowbite-os", "program"))
        self.assertEqual(head["basis"], {"ticks_per_m": 1000.0, "wheel_distance_m": 0.4})
        self.assertEqual(head["params"]["ticks_per_m"], 1000.0)
        self.assertAlmostEqual(head["params"]["wheel_distance_m"], 0.395)
        self.assertEqual(head["params"]["antenna_offset_x"], 0.0)
        self.assertEqual(head["imu"], {"roll": 0.0, "pitch": 0.0})
        self.assertEqual((head["imu_frame"], head["platform"]), ("base_link", "airseekers_tron"))
        self.assertEqual(lines[-1]["k"], "end")
        self.assertEqual(lines[-1]["why"], "enough")
        data = lines[1:-1]
        self.assertTrue(all(not isinstance(v, (dict, list)) for line in data for v in line.values()))
        kinds = {}
        for line in data:
            kinds.setdefault(line["k"], []).append(line)
        span = data[-1]["t"] - data[0]["t"]
        self.assertGreater(len(kinds["imu"]) / span, 40)
        self.assertGreater(len(kinds["gps"]) / span, 4)
        self.assertGreater(len(kinds["twist"]) / span, 5)
        self.assertLess(len(kinds["pose"]) / span, 10.5)
        self.assertTrue(kinds["cmd"])
        self.assertTrue(all(g["fix"] == "fixed" for g in kinds["gps"][:10]))
        # The receiver time only where hw/capabilities says it is UTC (gnss_receiver_time).
        self.assertTrue(all("rt" in g for g in kinds["gps"]))
        starts = [(m["name"]) for m in kinds["phase"] if m["edge"] == "start"]
        self.assertEqual(starts.count("line"), 4)
        self.assertEqual(starts.count("spin_left"), 2)
        self.assertEqual(starts.count("spin_right"), 2)
        self.assertEqual(starts.count("stand"), 9)
        # calibration.result and its file.
        answer = p.rpc("calibration.result")
        self.assertEqual(answer[0], "ok", answer)
        result = answer[1]
        self.assertEqual((result["status"], result["reason"]), ("done", "DONE"))
        values = {value["key"]: value for value in result["values"]}
        self.assertEqual(set(values), {"ticks_per_m", "track", "antenna_x", "antenna_y", "imu_roll", "imu_pitch"})
        # What this Tron is off by is found: the wheel ticks, the antenna and the tilt of the IMU (2 and -1.5 deg).
        for key, truth, delta in (("ticks_per_m", 940.0, 5.0), ("antenna_x", 0.098, 0.01), ("antenna_y", 0.0, 0.01),
                                  ("imu_roll", 0.0349, 0.006), ("imu_pitch", -0.0262, 0.006)):
            self.assertTrue(values[key]["ok"], values[key])
            self.assertAlmostEqual(values[key]["value"], truth, delta=delta, msg=key)
        # The recording holds no wheel ticks, so the distance between the wheels stays open.
        self.assertFalse(values["track"]["ok"], values["track"])
        self.assertEqual(result["recording"], files[0])
        with open(os.path.join(DATA, "calibration", "result.json")) as f:
            self.assertEqual(json.load(f)["id"], result["id"])

    def test_03_stopped_by_the_app(self):
        p = self.probe
        since = self.start()
        self.assertIsNotNone(p.event(since, "CALIBRATION", 30, phase="line"))
        time.sleep(2.0)
        answer = p.rpc("calibration.abort")
        self.assertEqual(answer, ("ok", {"ok": True}))
        failed = self.ends_with(since, "STOPPED", 5)
        self.assertEqual(failed["severity"], "info")
        self.assertEqual(p.rpc("calibration.abort"), ("ok", {"ok": False}))
        self.assertEqual(p.rpc("calibration.result")[1]["status"], "failed")

    def test_04_rtk_lost(self):
        p = self.probe
        since = self.start()
        self.assertIsNotNone(p.event(since, "CALIBRATION", 30, phase="line"))
        time.sleep(1.0)
        p.flags(gnss_quality=5)
        self.ends_with(since, "NO_RTK", 5)
        p.flags()
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03, 40))

    def test_05_no_room(self):
        p = self.probe
        p.replace(SMALL)
        # Only NO_ROOM: the rate of RTK fixes counts over 2 s and may still be short after test_04.
        self.assertTrue(p.wait(lambda: p.status and p.status.lane_length == 0
                               and [x.reason for x in p.status.problems] == ["NO_ROOM"], 15))
        since = len(p.events)
        answer = p.rpc("calibration.start")
        self.assertEqual(answer[0], "error", answer)
        self.assertEqual((answer[1][0], answer[1][2]["reason"]), (RpcError.ERROR_INVALID_REQUEST, "NO_ROOM"), answer)
        refused = p.event(since, "CALIBRATION_REFUSED", 5)
        self.assertEqual(refused["reason"], "NO_ROOM", refused)
        self.assertIn("free_length", refused)
        p.replace(LAWN)

    def test_06_bumper(self):
        p = self.probe
        since = self.start()
        self.assertIsNotNone(p.event(since, "CALIBRATION", 30, phase="line"))
        time.sleep(1.0)
        p.flags(bumper=1)
        self.ends_with(since, "EMERGENCY", 5)
        p.flags()
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 10), "the emergency stayed")
