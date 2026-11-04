"""mowbite_safety (S2) in the whole stack of sim.launch.py: it is the only writer of hw/cmd_vel; a tilt beyond the limit
latches TILT for remote control and drives alike and keeps it until the mower is level; an autonomous command outside
of the map latches FENCE, remote control has no fence; a bad position refuses autonomous commands without a latch; one
source drives at a time and a source moves after an emergency only after a zero. Some commands go straight to
safety/drive, as from a source that misbehaves; the logic would not send them.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_safety_filter.py
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
from mowbite_msgs.msg import DriveCommand, HardwareStatus, MapStatus, MqttPublish, RobotPose, SafetyStatus
from mowbite_msgs.srv import DriveTo, ReplaceMap, SetPose
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_s2_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TILT = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_TILT
FENCE = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_FENCE
RESET = "mower_logic/reset_emergency"

# One lawn around the start; the mower starts at (0, 0) facing +x.
LAWN = [(-2, -2), (8, -2), (8, 5), (-2, 5)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow"}, "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS,
                          "log": os.path.join(WORK, "rx.jsonl"), "data_dir": os.path.join(WORK, "data"),
                          "mqtt_port": "18832"}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_status = None
        self.pose = None
        self.hw = None
        self.safety = None
        self.events = []
        self.logs = []
        self.node = rclpy.create_node("safety_filter_probe")
        self.node.create_subscription(MapStatus, "map/status", self._map_status, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(SafetyStatus, "safety/status", self._safety, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Log, "/rosout", self._log, rosout_qos)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.teleop_pub = self.node.create_publisher(Twist, "bridge/teleop", 10)
        self.drive_pub = self.node.create_publisher(DriveCommand, "safety/drive", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _map_status(self, msg):
        with self.lock:
            self.map_status = msg

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _hw(self, msg):
        with self.lock:
            self.hw = msg

    def _safety(self, msg):
        with self.lock:
            self.safety = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

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

    def logged(self, text, since=0, timeout=10.0):
        return self.wait(lambda: any(text in line for line in self.logs[since:]), timeout)

    def event(self, since, pred, timeout=10.0):
        return self.wait(lambda: next((e for e in self.events[since:] if pred(e)), None), timeout)

    def call(self, client, request, timeout=60.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def act(self, action):
        self.action_pub.publish(String(data=action))

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def truth(self):
        try:
            with open(STATE) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def moved(self, seconds):
        """m the simulated mower moved in that time."""
        a = self.truth()
        time.sleep(seconds)
        b = self.truth()
        return math.hypot(b["x"] - a["x"], b["y"] - a["y"])

    def send(self, source, linear, seconds, period=0.05, teleop=False):
        """Commands of one source for that long, like a follower (20 Hz) or the app (teleop over the bridge)."""
        end = time.time() + seconds
        while time.time() < end:
            if teleop:
                msg = Twist()
                msg.linear.x = float(linear)
                self.teleop_pub.publish(msg)
            else:
                msg = DriveCommand(source=source)
                msg.twist.linear.x = float(linear)
                self.drive_pub.publish(msg)
            time.sleep(period)

    def put(self, x, y, yaw_deg=0.0):
        """Places the mower by hand and tells the localization, so pose and heading hold at once."""
        self.flags(place=[x, y, yaw_deg])
        request = SetPose.Request()
        request.pose.position.x, request.pose.position.y = float(x), float(y)
        request.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        request.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        self.call(self.set_pose, request)
        return self.wait(lambda: self.pose is not None and self.pose.heading_valid
                         and self.pose.position_accuracy < 0.1
                         and math.hypot(self.pose.x - x, self.pose.y - y) < 0.1,
                         30)

    def reset(self):
        """reset_emergency as the app sends it, until hw/status shows no emergency."""
        for _ in range(5):
            self.act(RESET)
            if self.wait(lambda: self.hw is not None and not self.hw.emergency, 3):
                return True
        return False

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestSafetyFilter(unittest.TestCase):
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

    def test_01_garden_and_one_writer_of_hw_cmd_vel(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.map_status is not None
                               and p.map_status.datum_state != MapStatus.DATUM_UNKNOWN, 30), "no datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.map_status.mowing_allowed, 10), "the map does not fit the datum")
        self.assertTrue(p.put(0.0, 0.0), "no good pose with a heading")
        self.assertTrue(p.wait(lambda: p.hw is not None and not p.hw.emergency, 20), "the driver blocks")
        self.assertTrue(p.wait(lambda: p.safety is not None and p.safety.autonomous_ok, 20), "S2 refuses drives")
        writers = p.node.get_publishers_info_by_topic("/hw/cmd_vel")
        self.assertEqual([w.node_name for w in writers], ["mowbite_safety"])
        self.assertLess(abs(p.safety.tilt_deg), 1.0)
        self.assertEqual(p.safety.fence_distance, 0.0)

    def test_02_tilt_while_recording_latches_until_the_mower_is_level(self):
        p = self.probe
        since, logs = len(p.events), len(p.logs)
        p.act("mower_logic:idle/start_area_recording")
        self.assertIsNotNone(p.event(since, lambda e: e["type"] == "STATE" and e["state"] == "AREA_RECORDING"))
        teleop = threading.Thread(target=p.send, args=(None, 0.1, 8.0), kwargs={"period": 0.1, "teleop": True},
                                  daemon=True)
        teleop.start()
        time.sleep(1.0)
        self.assertGreater(p.moved(1.0), 0.05, "remote control does not move the mower")
        p.flags(tilt_roll=35.0)
        tilted = p.event(since, lambda e: e["type"] == "EMERGENCY" and e["emergency"], 5)
        self.assertIsNotNone(tilted, "no emergency at 35 degrees")
        self.assertEqual(tilted["reason"], TILT)
        self.assertTrue(p.logged("Emergency reason changed from: '' to: 'LATCH, TILT'", logs))
        self.assertGreater(p.safety.tilt_deg, 34.0)
        # The joystick is still held.
        time.sleep(0.5)
        self.assertLess(p.moved(1.0), 0.01, "the tilted mower moves")
        teleop.join()
        # Still tilted: the release is refused and says why.
        p.act(RESET)
        self.assertTrue(p.logged("Emergency stays: still tilted by 35 deg, below 26 deg needed", logs))
        refused = p.event(since, lambda e: e["type"] == "EMERGENCY_RESET_REFUSED", 3)
        self.assertIsNotNone(refused, "no EMERGENCY_RESET_REFUSED")
        self.assertEqual((refused["reason"], refused["limit"], refused["severity"]), ("STILL_TILTED", 26.0, "warning"))
        self.assertAlmostEqual(refused["angle"], 35.0, delta=1.0)
        time.sleep(1.0)
        self.assertTrue(p.hw.emergency)
        p.flags(tilt_roll=10.0)
        self.assertTrue(p.reset(), "no release once level")
        self.assertIsNotNone(p.event(since, lambda e: e["type"] == "EMERGENCY" and not e["emergency"], 5))
        p.act("mower_logic:area_recording/exit_recording_mode")
        p.flags()

    def test_03_tilt_ends_a_drive(self):
        p = self.probe
        self.assertTrue(p.put(0.0, 0.0))
        since = len(p.events)
        answer = p.call(p.drive_to, DriveTo.Request(x=6.0, y=0.0))
        self.assertTrue(answer.success, answer.message)
        time.sleep(3.0)
        p.flags(tilt_pitch=-35.0)
        ended = p.event(since, lambda e: e["type"] == "DRIVE" and e.get("phase") == "end", 10)
        self.assertIsNotNone(ended, "the drive did not end")
        self.assertEqual(ended["reason"], "EMERGENCY")
        self.assertIn("TILT", ended["message"])
        time.sleep(1.0)
        self.assertLess(p.moved(1.0), 0.01)
        p.flags()
        self.assertTrue(p.reset())

    def test_04_fence_for_autonomous_commands_only(self):
        p = self.probe
        # 1.5 m west of the lawn, away from the station of fake_tron.
        self.assertTrue(p.put(-3.5, 3.0), "no pose at the new place")
        self.assertTrue(p.wait(lambda: p.safety is not None and abs(p.safety.fence_distance - 1.5) < 0.1, 5))
        since, logs = len(p.events), len(p.logs)
        p.send(DriveCommand.SOURCE_FOLLOWER, 0.1, 1.0)
        fence = p.event(since, lambda e: e["type"] == "EMERGENCY" and e["emergency"], 5)
        self.assertIsNotNone(fence, "no emergency for a drive outside of the map")
        self.assertEqual(fence["reason"], FENCE)
        self.assertTrue(p.logged("Emergency FENCE: autonomous drive with base_link is 1.", logs))
        self.assertLess(p.moved(1.0), 0.01)
        # Standing outside, the release goes through; remote control brings the mower back without a fence.
        self.assertTrue(p.reset())
        p.act("mower_logic:idle/start_area_recording")
        self.assertIsNotNone(p.event(since, lambda e: e["type"] == "STATE" and e["state"] == "AREA_RECORDING"))
        threading.Thread(target=p.send, args=(None, 0.1, 3.0), kwargs={"period": 0.1, "teleop": True},
                         daemon=True).start()
        time.sleep(1.0)
        self.assertGreater(p.moved(1.0), 0.05, "remote control is fenced")
        time.sleep(1.5)
        p.act("mower_logic:area_recording/exit_recording_mode")
        self.assertTrue(p.put(0.0, 0.0))

    def test_05_bad_position_refuses_autonomous_commands_without_latch(self):
        p = self.probe
        p.flags(gnss_quality=5)
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy > 0.1, 30), "the accuracy stays good")
        p.send(DriveCommand.SOURCE_FOLLOWER, 0.1, 1.0)
        self.assertTrue(p.wait(lambda: p.safety.last_refusal.startswith("follower: position accuracy"), 5),
                        p.safety.last_refusal)
        self.assertLess(p.moved(0.5), 0.01)
        self.assertFalse(p.hw.emergency)
        self.assertEqual(p.safety.latched, 0)
        p.flags()
        self.assertTrue(p.put(0.0, 0.0))

    def test_06_one_source_at_a_time_and_a_zero_after_an_emergency(self):
        p = self.probe
        logs = len(p.logs)
        follower = threading.Thread(target=p.send, args=(DriveCommand.SOURCE_FOLLOWER, 0.1, 2.0), daemon=True)
        follower.start()
        time.sleep(0.5)
        p.send(DriveCommand.SOURCE_TELEOP, 0.2, 0.5)
        self.assertTrue(p.logged("Refused the remote control: the follower drives (one source at a time)", logs))
        self.assertEqual(p.safety.owner, DriveCommand.SOURCE_FOLLOWER)
        follower.join()
        p.send(DriveCommand.SOURCE_FOLLOWER, 0.0, 0.1)
        time.sleep(1.5)
        # A follower that keeps sending through an emergency moves only after a zero.
        stream = threading.Thread(target=p.send, args=(DriveCommand.SOURCE_FOLLOWER, 0.1, 9.0), daemon=True)
        stream.start()
        time.sleep(1.0)
        self.assertGreater(p.moved(0.5), 0.02)
        p.flags(stop=1)
        self.assertTrue(p.wait(lambda: p.hw.emergency, 5))
        p.flags()
        time.sleep(0.5)
        self.assertTrue(p.reset())
        self.assertLess(p.moved(2.0), 0.01, "a stream through the emergency moves again")
        self.assertEqual(p.safety.last_refusal, "follower: waits for a zero or 1.0 s without commands")
        stream.join()
        p.send(DriveCommand.SOURCE_FOLLOWER, 0.0, 0.1)
        threading.Thread(target=p.send, args=(DriveCommand.SOURCE_FOLLOWER, 0.1, 1.5), daemon=True).start()
        time.sleep(0.5)
        self.assertGreater(p.moved(0.5), 0.02, "a zero does not arm the follower")
        time.sleep(1.0)
        p.send(DriveCommand.SOURCE_FOLLOWER, 0.0, 0.1)

