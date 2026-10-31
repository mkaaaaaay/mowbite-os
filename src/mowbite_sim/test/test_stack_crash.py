"""The rest of MowBite OS dies while the mower mows (G25): the whole stack of sim.launch.py with the blade allowed, as
in test_follower_stall.py. Once the blade turns and the mower drives on a path, the container with every node but
tron_mcu is killed (SIGKILL: no node shuts down on its own). fake_tron goes on getting the heartbeat, so like the MCU it
holds the last setpoint and the blade. tron_mcu runs in a process of its own and has to stop both by itself: speed 0
after cmd_vel_timeout, the blade off once its lease ran out or the missing IMU latched IMU_TIMEOUT.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_stack_crash.py
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
import launch_testing.asserts
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.events.process import ProcessStarted
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import AbsolutePose, HardwareStatus, MapStatus, MqttPublish
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_crash_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
CONFIG = os.path.join(WORK, "crash.yaml")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# tron.yaml: cmd_vel_timeout of tron_mcu and blade_lease of the logic. fake_tron stops on its own only after 2 s
# without any frame of the host.
CMD_VEL_TIMEOUT = 0.5
BLADE_LEASE = 1.0
HOST_TIMEOUT = 2.0
HEARTBEAT = 0xFF
# SpeedData 0, 0 as sent.
ZERO = "00" * 8

# The lawn of test_mowing.py, one loop around it.
LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "mowbite_localization:\n  ros__parameters:\n    antenna_offset_x: 0.098\n    antenna_offset_y: 0.0\n")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18833", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.hw = None
        self.events = []
        self.node = rclpy.create_node("crash_probe")
        self.node.create_subscription(MapStatus, "map/status", self._status, LATCHED)
        self.node.create_subscription(AbsolutePose, "localization/pose", self._pose, 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _hw(self, msg):
        with self.lock:
            self.hw = msg

    def _publish(self, msg):
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

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

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def flags(self, **values):
        with open(FLAGS + ".tmp", "w") as f:
            json.dump(values, f)
        os.replace(FLAGS + ".tmp", FLAGS)

    def standing(self):
        a = self.truth()
        time.sleep(1.0)
        b = self.truth()
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < 0.02, (a, b)

    def received(self, since=0.0):
        """What fake_tron got from the driver from its time since on."""
        out = []
        with open(RX) as f:
            for line in f:
                entry = json.loads(line)
                if "mod" in entry and entry["t"] >= since:
                    out.append(entry)
        return out

    def recent(self, size=65536):
        """The last entries of the fake_tron log, without reading all of it."""
        with open(RX, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - size))
            lines = f.read().decode().splitlines()[1:]
        return [json.loads(line) for line in lines]

    def sim_time(self):
        return self.recent()[-1]["t"]

    def driving_now(self):
        """fake_tron time of its newest entry if the newest setpoint it got moves the mower."""
        entries = self.recent()
        speeds = [e for e in entries if "speed" in e and e["t"] > entries[-1]["t"] - 0.2]
        return entries[-1]["t"] if speeds and speeds[-1]["hex"] != ZERO else None

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


class TestStackCrash(unittest.TestCase):
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

    def test_01_lawn_and_an_aligned_heading(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.orientation_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def test_02_the_blade_turns_and_the_mower_drives(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.blade_motor.rpm > 1000, 5), "no rpm from fake_tron")
        self.assertTrue(p.wait(lambda: (p.truth() or {}).get("linear", 0.0) > 0.05, 30), "the mower does not drive")

    def test_03_without_the_container_the_driver_stops_mower_and_blade(self, proc_info):
        p = self.probe
        self.assertIsInstance(proc_info["tron_mcu_node"], ProcessStarted)
        container = proc_info["component_container"].pid
        # Not in a stop at a corner: fake_tron holds a setpoint that moves the mower.
        t0 = p.wait(p.driving_now, 30)
        self.assertTrue(t0, "the mower does not drive")
        os.kill(container, signal.SIGKILL)
        # Long enough for fake_tron to drop setpoint and blade by itself, were the heartbeat gone.
        time.sleep(HOST_TIMEOUT + 1.0)
        rx = p.received(t0)
        speeds = [e for e in rx if "speed" in e]
        zeros = [e for e in speeds if e["hex"] == ZERO]
        self.assertTrue(zeros, f"no stop after the crash: {speeds[-5:]}")
        stop = zeros[0]["t"]
        self.assertLess(stop - t0, CMD_VEL_TIMEOUT + 0.5, f"stop {stop - t0:.2f} s after the crash")
        after = [e for e in speeds if e["t"] >= stop]
        self.assertTrue(all(e["hex"] == ZERO for e in after), after)
        # The stop burst and no stream of zero setpoints.
        self.assertLessEqual(len(after), 3, after)
        cutter = [e for e in rx if "cutter" in e]
        self.assertTrue(cutter, "no blade frame after the crash")
        off = next((e["t"] for e in cutter if e["cutter"] == 0), None)
        self.assertIsNotNone(off, cutter)
        self.assertLess(off - t0, BLADE_LEASE + 0.5, f"blade off {off - t0:.2f} s after the crash")
        self.assertFalse([e for e in cutter if e["t"] >= off and e["cutter"] != 0], cutter)
        # The driver went on talking, so it was its own frames that stopped the setpoint and the blade.
        beats = [e for e in rx if e["type"] == HEARTBEAT and e["t"] > t0 + HOST_TIMEOUT]
        self.assertGreaterEqual(len(beats), 5, "the heartbeat stopped")
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual((poses[1]["linear"], poses[1]["angular"]), (0.0, 0.0), poses)
        hw = p.wait(lambda: p.hw is not None and p.hw.blade_motor.ok and p.hw.blade_motor.rpm == 0 and p.hw, 3)
        self.assertTrue(hw, "fake_tron still reports blade rpm")
        self.assertTrue(hw.emergency and hw.emergency_latched, hw)
        self.assertTrue(hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, hw.emergency_reason)
        self.assertFalse(hw.blade_on or hw.moving, hw)
        print(f"after the crash: stop {stop - t0:.2f} s, blade off {off - t0:.2f} s, "
              f"{len(after)} zero setpoints, reason {hw.emergency_reason:#x}", flush=True)

    def test_04_the_driver_keeps_the_mower_blocked(self, proc_info):
        p = self.probe
        t1 = p.sim_time()
        time.sleep(3.0)
        rx = p.received(t1)
        self.assertFalse([e for e in rx if "speed" in e], "a setpoint after the stop")
        self.assertFalse([e for e in rx if e.get("cutter", 0) != 0], "the blade on again")
        self.assertGreaterEqual(len([e for e in rx if e["type"] == HEARTBEAT]), 20, "the heartbeat stopped")
        self.assertTrue(p.hw.emergency and p.hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, p.hw)
        self.assertIsInstance(proc_info["tron_mcu_node"], ProcessStarted)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_only_the_container_was_killed(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[-signal.SIGKILL],
                                               process="component_container")
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143],
                                               process="tron_mcu_node")
