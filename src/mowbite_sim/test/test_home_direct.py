"""The way home without the planner in the simulation: the whole stack of sim.launch.py with the station of fake_tron
in a yard that reaches around it. The planner process is stopped (SIGSTOP), so its services stay offered and never
answer. docking.start from the yard then waits plan_timeout for the way, tells DOCKING_DIRECT and drives straight to
the point in front of the station, base_link no farther than direct_margin off that line, and docks with the charge
confirmed. Where the straight way is not free (the mower stands too close to the end of the yard) the try fails with
DOCKING_RETRY, cause and direct_blocked, and the next one docks over the way of the planner once it answers again. A
job whose approach gets no answer asks again three times, standing in MOWING, then ends with DOCKING and the reason
PLANNER_TIMEOUT and goes home the straight way.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_home_direct.py
"""
import json
import math
import os
import shutil
import signal
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest
from mowbite_msgs.msg import RpcResponse
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_home_direct_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "home_direct.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
START = "mower_logic:idle/start_mowing"
PAUSE = "mower_logic:mowing/pause"
CONTINUE = "mower_logic:mowing/continue"

# The station of test_dock.py in a yard that reaches 1.2 m behind it: around the point in front of it, 0.8 m east of
# the station, the 0.9 m the straight way needs are free (turning circle 0.644 m, direct_margin 0.15 m, accuracy limit
# 0.1 m, 1 cm of the check).
YARD = [(-2.2, -2.6), (4.2, -2.6), (4.2, 2.6), (-2.2, 2.6)]
PATCH = [(0.8, -1.5), (3.6, -1.5), (3.6, 1.5), (0.8, 1.5)]
DOCKED = [-1.0, 0.0, 0.0]
APPROACH = (-0.2, 0.0)
# Short, so that the questions of a job all go out before DDS forgets the stopped planner (20 s): from then on the
# logic finds no service and a job pauses with APPROACH_FAILED. Planning the patch takes less than 0.1 s. Half a
# second off the step of 1 s in which the logic looks at its questions, so each one takes the same time.
PLAN_TIMEOUT = 3.5
MARGIN = 0.15
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]},
                    {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                     "outline": [{"x": x, "y": y} for x, y in PATCH]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            f"mowbite_logic:\n  ros__parameters:\n    gps_wait_time: 2.0\n    plan_timeout: {PLAN_TIMEOUT}\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            # The station of sim.yaml.
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18846", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def parent(pid):
    """The parent of a process, 0 once it is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return int(f.read().rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def planner_pids():
    """The processes of the planner node this test launched, its workers included."""
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/cmdline", "rb") as f:
                if b"mowbite_planner_node" not in f.read():
                    continue
        except OSError:
            continue
        up = int(name)
        while up > 1 and up != os.getpid():
            up = parent(up)
        if up == os.getpid():
            pids.append(int(name))
    return pids


def signal_planner(sig):
    pids = planner_pids()
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    return pids


def off_the_line(point, a, b):
    """m from point to the stretch from a to b."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length < 1e-9:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = ((point[0] - a[0]) * (b[0] - a[0]) + (point[1] - a[1]) * (b[1] - a[1])) / length ** 2
    t = min(1.0, max(0.0, t))
    return math.hypot(point[0] - a[0] - t * (b[0] - a[0]), point[1] - a[1] - t * (b[1] - a[1]))


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.map = None
        self.pose = None
        self.robot = None
        self.hw = None
        self.events = []
        self.answers = {}
        self.node = rclpy.create_node("home_direct_probe")
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
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

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

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
        rid = f"direct-{self.rpc_count}"
        self.rpc_pub.publish(RpcRequest(method=method, params=json.dumps(params) if params is not None else "",
                                        id=rid))
        return self.wait(lambda: self.answers.get(rid), timeout)

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

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestHomeDirect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        signal_planner(signal.SIGCONT)
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def silence(self):
        """Stops the planner process until the end of the test: it keeps its services and answers nothing."""
        self.addCleanup(signal_planner, signal.SIGCONT)
        self.assertTrue(signal_planner(signal.SIGSTOP), "no planner process")

    def carry(self, x, y, yaw_deg):
        """Carried to (x, y) in the yard, off the station, and the localization told, as the gyro of the simulation
        does not see the turn; waits for the RTK position there."""
        p = self.probe
        p.flags(place=[x, y, yaw_deg])
        request = SetPose.Request()
        request.pose.position.x, request.pose.position.y = float(x), float(y)
        request.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        request.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.heading_valid and p.pose.position_accuracy <= 0.03
                               and math.hypot(p.pose.x - x, p.pose.y - y) < 0.05, 30), "no RTK position there")
        p.flags()
        self.assertTrue(p.wait(lambda: p.hw and not p.hw.dock_contact, 5), "still on the contacts")

    def docked(self, since, timeout=150):
        p = self.probe
        self.assertIsNotNone(p.event(since, "DOCKED", timeout), p.events[since:])
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 5), p.robot.current_state)
        truth = p.truth()
        self.assertTrue(truth["dock_ok"], truth)
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.01)
        self.assertLess(abs(truth["y"]), 0.02, truth)

    def straight(self, since, direct):
        """Follows the mower from where DOCKING_DIRECT was told to the point in front of the station: base_link never
        farther off that line than the margin."""
        p = self.probe
        start = (direct["x"], direct["y"])
        worst = 0.0
        end = time.time() + 120
        arrived = False
        while time.time() < end and not arrived:
            truth = p.truth()
            worst = max(worst, off_the_line((truth["x"], truth["y"]), start, APPROACH))
            arrived = math.hypot(truth["x"] - APPROACH[0], truth["y"] - APPROACH[1]) < 0.1
            time.sleep(0.1)
        self.assertTrue(arrived, (p.truth(), p.events[since:]))
        self.assertLess(worst, MARGIN, p.events[since:])

    def test_01_on_the_station(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.map and p.map["docking_stations"], 10), "the map did not come")
        time.sleep(0.5)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        since = len(p.events)
        p.flags(place=DOCKED)
        self.assertTrue(p.wait(lambda: p.hw and p.hw.dock_contact, 10), "no contact on the station")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy <= 0.03 and abs(p.pose.x + 1.0) < 0.05,
                               30), "no RTK position there")
        p.flags()
        self.assertIsNotNone(p.event(since, "DOCKED", 30), p.events[since:])

    def test_02_without_an_answer_it_drives_straight_home(self):
        p = self.probe
        self.carry(2.0, 0.8, 90.0)
        self.silence()
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        docking = p.event(since, "DOCKING", 5)
        self.assertIsNotNone(docking, p.events[since:])
        self.assertEqual(docking["reason"], "MANUAL", docking)
        # The planner has its time first.
        self.assertIsNone(p.event(since, "DOCKING_DIRECT", PLAN_TIMEOUT - 2), p.events[since:])
        direct = p.event(since, "DOCKING_DIRECT", 8)
        self.assertIsNotNone(direct, p.events[since:])
        self.assertEqual(direct["cause"], "PLANNER_TIMEOUT", direct)
        self.assertEqual((direct["severity"], direct["resume"]), ("warning", "auto"), direct)
        self.assertAlmostEqual(direct["distance"], math.hypot(2.2, 0.8), delta=0.05, msg=direct)
        self.straight(since, direct)
        self.docked(since)
        types = [e["type"] for e in p.events[since:]]
        self.assertEqual(types.count("DOCKING_DIRECT"), 1, p.events[since:])
        for absent in ("DOCKING_RETRY", "DOCKING_FAILED"):
            self.assertNotIn(absent, types, p.events[since:])

    def test_03_a_way_that_is_not_free_waits_for_the_planner(self):
        p = self.probe
        # 0.6 m from the end of the yard: no room to turn there with the margin.
        self.carry(3.2, 2.0, 180.0)
        self.silence()
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        retry = p.event(since, "DOCKING_RETRY", PLAN_TIMEOUT + 8)
        self.assertIsNotNone(retry, p.events[since:])
        self.assertEqual(retry["reason"], "approach_failed", retry)
        self.assertEqual(retry["cause"], "PLANNER_TIMEOUT", retry)
        self.assertIs(retry["direct_blocked"], True, retry)
        self.assertEqual(retry["attempts"], 1, retry)
        self.assertEqual((retry["severity"], retry["resume"]), ("warning", "auto"), retry)
        self.assertIn("the straight way is not free: the areas end", retry["message"])
        truth = p.truth()
        self.assertLess(math.hypot(truth["x"] - 3.2, truth["y"] - 2.0), 0.02, truth)
        # The planner answers again: the try that began meanwhile takes its way.
        signal_planner(signal.SIGCONT)
        self.docked(since)
        types = [e["type"] for e in p.events[since:]]
        self.assertNotIn("DOCKING_DIRECT", types, p.events[since:])
        self.assertNotIn("DOCKING_FAILED", types, p.events[since:])

    def test_04_a_job_without_an_answer_ends_at_home(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 10), p.robot.current_state)
        since = len(p.events)
        p.act(START)
        self.assertIsNotNone(p.event(since, "STATE", 60, state="MOWING"), p.events[since:])
        self.assertTrue(p.wait(lambda: p.truth()["x"] > 0.9, 90), p.truth())
        p.act(PAUSE)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="PAUSED"), p.events[since:])
        time.sleep(2.0)
        # Carried off its path while it is paused: to go on it needs a way back, which the planner does not give.
        self.carry(0.0, -1.2, 0.0)
        self.silence()
        since = len(p.events)
        p.act(CONTINUE)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"), p.events[since:])
        # Four questions of plan_timeout each, standing in MOWING, then home.
        self.assertIsNone(p.event(since, "DOCKING", 3 * PLAN_TIMEOUT), p.events[since:])
        self.assertEqual(p.robot.current_state, "MOWING")
        truth = p.truth()
        self.assertLess(math.hypot(truth["x"], truth["y"] + 1.2), 0.02, truth)
        docking = p.event(since, "DOCKING", PLAN_TIMEOUT + 8)
        self.assertIsNotNone(docking, p.events[since:])
        self.assertEqual(docking["reason"], "PLANNER_TIMEOUT", docking)
        self.assertEqual((docking["severity"], docking["resume"]), ("warning", "user"), docking)
        self.assertEqual(docking["message"], "Going home: the planner does not answer", docking)
        # By now DDS may have dropped the stopped planner: no answer in time, or no service at all.
        direct = p.event(since, "DOCKING_DIRECT", PLAN_TIMEOUT + 8)
        self.assertIsNotNone(direct, p.events[since:])
        self.assertIn(direct["cause"], ("PLANNER_TIMEOUT", "PLANNER_MISSING"), direct)
        self.straight(since, direct)
        self.docked(since)
        events = p.events[since:]
        types = [e["type"] for e in events]
        self.assertFalse([e for e in events if e["type"] == "STATE" and e["state"] == "PAUSED"], events)
        for absent in ("JOB_COMPLETE", "DOCKING_RETRY", "DOCKING_FAILED"):
            self.assertNotIn(absent, types, events)
        # The job is kept: the next start goes on with it.
        self.assertEqual(docking["job_id"], next(e for e in events if e["type"] == "STATE")["job_id"], events)
