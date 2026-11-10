"""A node of the container stalls while the mower mows (G39): the whole stack of sim.launch.py with the blade allowed,
as in test_stack_crash.py. Once the blade turns and the mower drives, one thread of the container is stopped (ptrace,
only that thread): first the one of the bridge, then the one of S2. The watch of the container (mowbite_watch) has to
end the process once the node gave no sign for its limit (bridge 10 s, S2 2 s), not before; then the way of a crash:
tron_mcu stops the wheels and the blade runs out, the container starts again, the restart hold latches and the event
UNEXPECTED_RESTART names the node with reason NODE_STALLED (cause stays OS_CRASH). With the bridge stalled the mower
mows on without remote control until the limit; with S2 stalled the wheels stop at once and the blade with the end.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_watch_stall.py
"""
import ctypes
import json
import os
import shutil
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
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_watch_")
STATE = os.path.join(WORK, "state.json")
FLAGS = os.path.join(WORK, "flags.json")
RX = os.path.join(WORK, "rx.jsonl")
DATA = os.path.join(WORK, "data")
RESTART_HOLD = os.path.join(DATA, "restart-hold.json")
EXIT_NOTE = os.path.join(DATA, "container_exit.json")
WATCH_NOTE = os.path.join(DATA, "watch_exit.json")
CONFIG = os.path.join(WORK, "watch.yaml")
UNEXPECTED_RESTART = HardwareStatus.REASON_UNEXPECTED_RESTART
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# tron.yaml: cmd_vel_timeout of tron_mcu and blade_lease of the logic.
CMD_VEL_TIMEOUT = 0.5
BLADE_LEASE = 1.0
# mowbite_watch: limits of the bridge and of S2, how often it judges, and its exit code.
BRIDGE_LIMIT = 10.0
SAFETY_LIMIT = 2.0
PERIOD = 0.5
EXIT_CODE = 86
RESTART_DELAY = 2.0
ZERO = "00" * 8
# Slack for the times below: a loaded machine, the sign period of the node.
SLACK = 0.5

LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")

LIBC = ctypes.CDLL(None, use_errno=True)
LIBC.ptrace.argtypes = [ctypes.c_long, ctypes.c_long, ctypes.c_void_p, ctypes.c_void_p]
PTRACE_SEIZE = 0x4206
PTRACE_INTERRUPT = 0x4207
# waitpid: also for threads that are not children (tracees).
WALL = 0x40000000


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": STATE, "flags": FLAGS, "log": RX,
                          "data_dir": DATA, "mqtt_port": "18834", "extra_config": CONFIG}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.hw = None
        self.robot = None
        self.events = []
        self.logs = []
        self.node = rclpy.create_node("watch_probe")
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Log, "/rosout", self._log, rosout_qos)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
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

    def _log(self, msg):
        with self.lock:
            self.logs.append(msg.msg)

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

    def logged(self, text, timeout):
        return self.wait(lambda: any(text in line for line in self.logs), timeout)

    def act(self, action):
        self.action_pub.publish(String(data=action))

    def event(self, since, type_, timeout, **match):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def received(self, since=0.0):
        out = []
        with open(RX) as f:
            for line in f:
                entry = json.loads(line)
                if "mod" in entry and entry["t"] >= since:
                    out.append(entry)
        return out

    def recent(self, size=65536):
        with open(RX, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - size))
            lines = f.read().decode().splitlines()[1:]
        return [json.loads(line) for line in lines]

    def clock_offset(self, reads=40):
        """time.monotonic() minus the time of fake_tron, from the read that found its newest entry the freshest."""
        offsets = []
        for _ in range(reads):
            now = time.monotonic()
            offsets.append(now - self.recent(4096)[-1]["t"])
            time.sleep(0.01)
        return min(offsets)

    def driving_now(self):
        entries = self.recent()
        speeds = [e for e in entries if "speed" in e and e["t"] > entries[-1]["t"] - 0.2]
        return entries[-1]["t"] if speeds and speeds[-1]["hex"] != ZERO else None

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


def container_pid(proc_info):
    for action in proc_info.processes():
        event = proc_info[action]
        if "component_container_isolated" in action.process_details["name"] and isinstance(event, ProcessStarted):
            return event.pid
    return None


def restarted(proc_info, pid, timeout):
    end = time.time() + timeout
    while time.time() < end:
        now = container_pid(proc_info)
        if now is not None and now != pid:
            return now
        time.sleep(0.1)
    return None


def stop_thread(pid, name):
    """Stops the thread of pid with that name (the watch names it after its node) and only it."""
    for tid in os.listdir(f"/proc/{pid}/task"):
        with open(f"/proc/{pid}/task/{tid}/comm") as f:
            if f.read().strip() != name:
                continue
        for request in (PTRACE_SEIZE, PTRACE_INTERRUPT):
            if LIBC.ptrace(request, int(tid), None, None) != 0:
                raise OSError(ctypes.get_errno(), f"ptrace {request:#x} of {name} ({tid})")
        return int(tid)
    raise LookupError(f"no thread {name} in {pid}")


def reap(tid, timeout):
    """Waits for the end of a thread this process traces: until it is collected here, its process cannot end."""
    end = time.time() + timeout
    while time.time() < end:
        done, status = os.waitpid(tid, os.WNOHANG | WALL)
        if done and (os.WIFEXITED(status) or os.WIFSIGNALED(status)):
            return True
        time.sleep(0.02)
    return False


class TestWatchStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        with open(FLAGS, "w") as f:
            json.dump({}, f)

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def mow(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.blade_motor.rpm > 1000, 5), "no rpm from fake_tron")

    def stall(self, proc_info, thread, limit):
        """Stops that thread while the mower drives; the times of fake_tron of the stall, of the last setpoint that
        moved the mower, of the final stop and of the last time the blade went off."""
        p = self.probe
        pid = container_pid(proc_info)
        offset = p.clock_offset()
        self.assertTrue(p.wait(p.driving_now, 30), "the mower does not drive")
        t0 = time.monotonic() - offset
        tid = stop_thread(pid, thread)
        self.assertTrue(reap(tid, limit + PERIOD + 10), f"{thread} did not end")
        self.assertTrue(restarted(proc_info, pid, RESTART_DELAY + 10), "no new container")
        rx = p.received(t0)
        speeds = [e for e in rx if "speed" in e]
        moving = [e["t"] for e in speeds if e["hex"] != ZERO]
        self.assertTrue(moving, speeds[:5])
        # The stop for good: the first zero setpoint after the last one that moved.
        stop = next((e["t"] for e in speeds if e["hex"] == ZERO and e["t"] > moving[-1]), None)
        self.assertIsNotNone(stop, speeds[-5:])
        cutter = [e for e in rx if "cutter" in e]
        off = next((e["t"] for e in reversed(cutter) if e["cutter"] == 0), None)
        self.assertIsNotNone(off, "the blade never went off")
        self.assertFalse([e for e in cutter if e["t"] > off and e["cutter"] != 0], "the blade on again")
        print(f"{thread} stalled: last drive {moving[-1] - t0:.2f} s, stop {stop - t0:.2f} s, blade off "
              f"{off - t0:.2f} s after the stall", flush=True)
        return t0, moving[-1], stop, off

    def restart_event(self, since, node, limit, state):
        p = self.probe
        event = p.event(since, "UNEXPECTED_RESTART", 30)
        self.assertTrue(event, [e["type"] for e in p.events[since:]])
        self.assertEqual((event["cause"], event["reason"], event["state_before"], event["hold"]),
                         ("OS_CRASH", "NODE_STALLED", state, True), event)
        self.assertEqual((event["severity"], event["resume"]), ("error", "user"), event)
        stalled = event["stalled"]
        self.assertEqual((stalled["node"], stalled["limit"]), (node, limit), event)
        # One decimal as the watch writes it.
        self.assertTrue(limit <= stalled["seconds"] <= limit + PERIOD + 0.2, event)
        self.assertIn(f"The watch ended MowBite OS: {node} gave no sign", event["detail"])
        with open(EXIT_NOTE) as f:
            note = json.load(f)
        self.assertEqual((note["exit_code"], note["stalled"]["node"]), (EXIT_CODE, node), note)
        self.assertFalse(os.path.exists(WATCH_NOTE))
        self.assertTrue(p.logged(f"ended by the watch: {node} gave no sign", 10))
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & UNEXPECTED_RESTART, 10),
                        p.robot)
        self.assertTrue(os.path.exists(RESTART_HOLD))

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
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def test_02_a_stalled_bridge_ends_the_container_after_its_limit(self, proc_info):
        self.mow()
        since = len(self.probe.events)
        t0, moved, stop, off = self.stall(proc_info, "bridge", BRIDGE_LIMIT)
        # Without remote control the mower mows on until the limit, then the way of a crash.
        self.assertGreater(moved - t0, BRIDGE_LIMIT - 1.0)
        self.assertLess(stop - t0, BRIDGE_LIMIT + PERIOD + CMD_VEL_TIMEOUT + SLACK)
        self.assertLess(off - t0, BRIDGE_LIMIT + PERIOD + BLADE_LEASE + SLACK)
        self.restart_event(since, "mowbite_bridge", BRIDGE_LIMIT, "MOWING")

    def test_03_a_stalled_s2_stops_the_wheels_at_once_and_the_blade_with_the_end(self, proc_info):
        p = self.probe
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.hw.emergency, 10), p.hw)
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5), p.robot)
        self.mow()
        since = len(p.events)
        t0, _, stop, off = self.stall(proc_info, "safety", SAFETY_LIMIT)
        # No hw/cmd_vel any more: the driver stops the wheels; the blade lease of the logic ends with the process.
        self.assertLess(stop - t0, CMD_VEL_TIMEOUT + SLACK)
        self.assertLess(off - t0, SAFETY_LIMIT + PERIOD + BLADE_LEASE + SLACK)
        event = p.event(since, "UNEXPECTED_RESTART", 30)
        self.restart_event(since, "mowbite_safety", SAFETY_LIMIT, event["state_before"] if event else "MOWING")
        self.assertNotEqual(event["state_before"], "IDLE", event)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_the_watch_ended_two_containers(self, proc_info):
        codes = [proc_info[a].returncode for a in proc_info.processes()
                 if "component_container_isolated" in a.process_details["name"]]
        self.assertEqual(codes[:2], [EXIT_CODE, EXIT_CODE], codes)
        self.assertEqual(len(codes), 3, codes)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143],
                                               process="tron_mcu_node")
