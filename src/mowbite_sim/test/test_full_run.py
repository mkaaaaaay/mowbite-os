"""The whole way of the app in the simulation: an area recorded by teleop, map/json, mowing.plan, start_mowing, the
blade, pause, abort and a new start from the checkpoint until the job completes. The test compares where the simulated
mower really went (the state file of fake_tron) with the paths mowing.plan answered, while the blade turns.

MOWBITE_FULL_RUN=real gives fake_tron the faults of a real Tron (assumptions until the measurements of phase 6 and 7):
RTK noise, a fix sent late, odometry 6 % short, a wheel deadband, the real track and the slip of the omni wheels.
Without it wheels and GNSS are ideal.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_full_run.py
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
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, MapStatus, MqttPublish, RpcRequest, RpcResponse
from mowbite_msgs.srv import SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

REAL = os.environ.get("MOWBITE_FULL_RUN") == "real"
# fake_tron flags of the real variant: RTK about 1 cm rms, the fix 0.1 s late, wheels report 6 % too little, a wheel
# below 0.05 m/s stands, the base turns on 0.395 m while the MCU counts 0.40 m, 0.01 rad of yaw per metre from slip.
FAULTS = {"gnss_noise": 0.01, "gnss_delay": 0.1, "odom_scale": 0.94, "wheel_deadband": 0.05, "track": 0.395,
          "yaw_drift": 0.01} if REAL else {}
# Limits of the cross track error (m) of the base while the blade turns: median, 95 %, worst.
LIMITS = (0.03, 0.08, 0.15) if REAL else (0.01, 0.03, 0.05)

WORK = tempfile.mkdtemp(prefix="mowbite_full_run_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "full_run.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REC = "mower_logic:area_recording/"
MOW = "mower_logic:mowing/"
# The lawn the teleop drives around.
CORNERS = [(2.6, 0.0), (2.6, 1.6), (0.0, 1.6), (0.0, 0.1)]

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            "mowbite_localization:\n  ros__parameters:\n    antenna_offset_x: 0.098\n    antenna_offset_y: 0.0\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18834", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def to_segment(x, y, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    length2 = ex * ex + ey * ey
    along = 0.0 if length2 == 0 else min(1.0, max(0.0, ((x - a[0]) * ex + (y - a[1]) * ey) / length2))
    return math.hypot(x - (a[0] + along * ex), y - (a[1] + along * ey))


def quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.map = None
        self.actions = {}
        self.answers = {}
        # (arrival time, event) and (read time, true pose) for the blade windows
        self.events = []
        self.track = []
        self.node = rclpy.create_node("full_run_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._answer, 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.teleop_pub = self.node.create_publisher(Twist, "bridge/teleop", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()
        threading.Thread(target=self._record, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

    def _pose(self, msg):
        # The localization turns about z only.
        yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        with self.lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.position_accuracy,
                         msg.orientation_valid)

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append((time.time(), json.loads(msg.payload)))

    def _answer(self, msg):
        with self.lock:
            self.answers[msg.id] = json.loads(msg.result)

    def _record(self):
        last = None
        while self.running:
            truth = self.truth()
            if truth and truth["t"] != last:
                last = truth["t"]
                with self.lock:
                    self.track.append((time.time(), truth))
            time.sleep(0.05)

    def truth(self):
        try:
            with open(STATE) as f:
                return json.load(f)
        except (OSError, ValueError):
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

    def call(self, client, request, timeout=30.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def rpc(self, method, params, id_, timeout=60.0):
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params), id=id_))
        return self.wait(lambda: self.answers.get(id_), timeout)

    def act(self, action):
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for _, e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump({**FAULTS, **values}, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def twist(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.teleop_pub.publish(msg)

    def teleop_to(self, x, y, timeout=60.0):
        """Like a thumb on the joystick: turn towards the point on the spot, then drive and steer, 10 times a second."""
        end = time.time() + timeout
        while time.time() < end:
            px, py, yaw, _, _ = self.pose
            distance = math.hypot(x - px, y - py)
            if distance < 0.03:
                break
            error = math.atan2(math.sin(math.atan2(y - py, x - px) - yaw), math.cos(math.atan2(y - py, x - px) - yaw))
            if abs(error) > 0.15:
                self.twist(0.0, math.copysign(0.3, error))
            else:
                self.twist(min(0.25, max(0.08, distance)), max(-0.3, min(0.3, 1.5 * error)))
            time.sleep(0.1)
        self.twist(0.0, 0.0)
        time.sleep(0.5)
        return math.hypot(x - self.pose[0], y - self.pose[1])

    def blade_off(self):
        """True once the last BLADES event says off (the blade may have stopped before, on a stretch without mowing)."""
        last = next((e for _, e in reversed(self.events) if e["type"] == "BLADES"), None)
        return last is not None and not last["enabled"]

    def standing(self):
        a = self.truth()
        time.sleep(1.0)
        b = self.truth()
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.02, (a, b)

    def blade_windows(self, end):
        """(on, off) times of the blade from the BLADES events, the last one open until end."""
        windows, on = [], None
        with self.lock:
            events = list(self.events)
        for t, e in events:
            if e["type"] != "BLADES":
                continue
            if e.get("enabled") and on is None:
                on = t
            elif not e.get("enabled") and on is not None:
                windows.append((on, t))
                on = None
        if on is not None:
            windows.append((on, end))
        return windows

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestFullRun(unittest.TestCase):
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

    def test_01_a_good_pose_with_a_heading(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose[4] and p.pose[3] < 0.1, 30),
                        "no good pose with a heading")

    def test_02_record_the_lawn_by_teleop(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_area_recording")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="AREA_RECORDING"))
        p.act(REC + "start_recording")
        # Like the app on a mode change: one zero, then the joystick.
        p.twist(0.0, 0.0)
        time.sleep(1.2)
        for x, y in CORNERS:
            self.assertLess(p.teleop_to(x, y), 0.1, f"teleop did not reach {x, y}")
        p.act(REC + "stop_recording")
        time.sleep(1.0)
        p.act(REC + "finish_mowing_area")
        recorded = p.event(since, "AREA_RECORDED", 10)
        self.assertIsNotNone(recorded, [e["type"] for _, e in p.events[since:]])
        self.assertAlmostEqual(recorded["area_m2"], 2.6 * 1.5, delta=0.5)
        saved = p.wait(lambda: p.map if p.map and len(p.map["areas"]) == 1 else None)
        self.assertIsNotNone(saved, f"map/json: {p.map}")
        self.assertEqual(saved["areas"][0]["id"], recorded["area_id"])
        self.assertEqual(saved["areas"][0]["properties"]["type"], "mow")
        type(self).area_id = recorded["area_id"]

    def test_03_mowing_plan_of_the_lawn_and_to_its_start(self):
        p = self.probe
        answer = p.rpc("mowing.plan", {"area_id": self.area_id}, "full_run_plan")
        self.assertIsNotNone(answer, "no answer to mowing.plan")
        self.assertEqual(answer["area_id"], self.area_id)
        paths = answer["paths"]
        self.assertTrue(paths and paths[0]["is_outline"] and not paths[-1]["is_outline"], json.dumps(answer)[:300])
        type(self).segments = [(a, b) for path in paths for a, b in zip(path["points"], path["points"][1:])]
        # plan_transit of planner 5093454 finds no way from inside the lawn to the start of the loop by the edge (the
        # body would stick out turning there), so teleop puts the mower at the start, within approach_distance.
        self.assertLess(p.teleop_to(*paths[0]["points"][0]), 0.1)
        since = len(p.events)
        p.act(REC + "exit_recording_mode")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="IDLE"))

    def test_04_start_and_the_blade(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        state = p.event(since, "STATE", 10, state="MOWING")
        self.assertIsNotNone(state, [e["type"] for _, e in p.events[since:]])
        type(self).job = state["job_id"]
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")

    def test_05_pause_and_continue(self):
        p = self.probe
        time.sleep(4.0)
        since = len(p.events)
        p.act(MOW + "pause")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="PAUSED"))
        self.assertTrue(p.wait(p.blade_off, 5), "the blade kept running")
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        since = len(p.events)
        p.act(MOW + "continue")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"))
        # The blade comes back on the next mow segment, which may lie after a turn or a transit.
        self.assertIsNotNone(p.event(since, "BLADES", 120, enabled=True), "the blade did not come back")

    def test_06_abort_stops_and_a_new_start_goes_on_with_the_job(self):
        p = self.probe
        time.sleep(6.0)
        since = len(p.events)
        p.act(MOW + "abort_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="IDLE"))
        self.assertTrue(p.wait(p.blade_off, 5), "the blade kept running")
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        with open(os.path.join(DATA, "checkpoint.json")) as f:
            self.assertEqual(json.load(f)["job_id"], self.job)
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        state = p.event(since, "STATE", 10, state="MOWING")
        self.assertIsNotNone(state)
        self.assertEqual(state["job_id"], self.job)

    def test_07_the_job_completes_on_the_plan(self):
        p = self.probe
        done = p.event(0, "JOB_COMPLETE", 900)
        self.assertIsNotNone(done, "the job did not complete")
        self.assertEqual(done["job_id"], self.job)
        self.assertTrue(p.wait(p.blade_off, 5), "the blade kept running")
        end = time.time()
        with p.lock:
            track = list(p.track)
        # The first second after the blade comes on and the last before it goes off are left out (event delay).
        windows = [(a + 1.0, b - 1.0) for a, b in p.blade_windows(end)]
        mowing = [t for when, t in track if any(a <= when <= b for a, b in windows)]
        self.assertGreater(len(mowing), 100, f"{len(mowing)} poses while the blade turned")
        errors = [min(to_segment(t["x"], t["y"], a, b) for a, b in self.segments) for t in mowing]
        median, p95, worst = quantile(errors, 0.5), quantile(errors, 0.95), max(errors)
        print(f"\nfull run {'real' if REAL else 'ideal'}: {len(mowing)} poses with the blade, cross track median "
              f"{median:.3f} m, 95 % {p95:.3f} m, worst {worst:.3f} m", flush=True)
        self.assertLess(median, LIMITS[0])
        self.assertLess(p95, LIMITS[1])
        self.assertLess(worst, LIMITS[2])
        self.assertLessEqual(max(abs(t["linear"]) for _, t in track), 0.3 + 1e-3)
