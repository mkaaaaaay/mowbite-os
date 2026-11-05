"""MowBite OS on the NX60i as nx60i.launch.py starts it, against fake_xcore on UDP over the loopback: the stack comes up
IDLE with the capabilities of the speed model and the speeds for the planner on bridge/params, the gyro calibrates on
the wheel reports of the driver; without a safety measurement mowing and drives are refused; the emergency stop latches
and is released; a killed driver stands the wheels (the board's own timeout) and its successor holds the board from its
first frame until a release; a crashed container comes back beside the running driver; after three restarts the driver
stays down. Prints the RSS of driver, container and planner.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_nx60i_stack.py
"""
import json
import os
import random
import shutil
import signal
import struct
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from geometry_msgs.msg import Twist, Vector3
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.events.process import ProcessStarted
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import DriveTo, SetEmergency
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_nx60i_")
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
STATE = os.path.join(WORK, "state.json")
DATA = os.path.join(WORK, "data")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
DATUM = (52.0, 9.0, 80.0)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EMERGENCY = 1
CLAIM, DATA_MSG = 3, 1
# The bits of the board as the driver sends them.
BOARD_LATCH, BOARD_HIGH_LEVEL = 1, 128
# nx60i.launch.py and stack.py.
DRIVER_RESTARTS = 3
# max_duty 0.3 of nx60i.yaml in the speed model 0.58 x (duty - 0.019).
REACHED = 0.58 * (0.3 - 0.019)

with open(FLAGS, "w") as f:
    json.dump({"gps_fix": "fix"}, f)


def generate_test_description():
    sim = FindPackageShare("mowbite_sim").find("mowbite_sim")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim, "config", "xcore_nx60i_services.json"), "--group-port", str(GROUP_PORT),
             "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE, "--log", LOG,
             "--log-max-mb", "60"],
        output="screen")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("mowbite_bringup").find("mowbite_bringup"),
                                                   "launch", "nx60i.launch.py")),
        launch_arguments={"interface": "lo", "bind_ip": BIND_IP, "announcement_port": str(GROUP_PORT),
                          "data_dir": DATA, "datum_lat": str(DATUM[0]), "datum_long": str(DATUM[1]),
                          "datum_height": str(DATUM[2]), "mqtt_port": str(random.randint(18000, 18999))}.items())
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[stack, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    flags.setdefault("gps_fix", "fix")
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


def board():
    with open(STATE) as f:
        return json.load(f)


def received(since, sid, msg_type):
    """Datagrams fake_xcore got from a driver since board time `since`."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if entry["t"] >= since and entry.get("sid") == sid and entry.get("type") == msg_type:
                out.append(entry)
    return out


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.caps = None
        self.robot = None
        self.pose = None
        self.params = []
        self.events = []
        self.logs = []
        self.node = rclpy.create_node("nx60i_probe")
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("caps"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(
            String, "bridge/params", self._param,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
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

    def _param(self, msg):
        with self.lock:
            self.params.append(json.loads(msg.data))

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

    def call(self, client, request, timeout=20.0):
        assert client.wait_for_service(timeout_sec=20.0), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    def event(self, since, type_, timeout, **match):
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def release(self):
        return self.call(self.emergency, SetEmergency.Request(latch=False, reason="test"))

    def drive(self, linear, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(Twist(linear=Vector3(x=linear)))
            time.sleep(0.1)

    def stop(self):
        self.running = False
        time.sleep(0.3)
        self.node.destroy_node()


def pids(proc_info, name):
    """Pids of the processes with that name that started, oldest first."""
    out = []
    for action in proc_info.processes():
        event = proc_info[action]
        if name in action.process_details["name"] and isinstance(event, ProcessStarted):
            out.append(event.pid)
    return out


def alive(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            return "State:\tZ" not in f.read()
    except OSError:
        return False


def running(proc_info, name):
    return [pid for pid in pids(proc_info, name) if alive(pid)]


def rss_mb(pid):
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024
    return None


def note(name):
    try:
        with open(os.path.join(DATA, name)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class TestNx60iStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_idle_with_the_speeds_of_the_model(self, proc_info):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 60), p.robot)
        caps = p.wait(lambda: p.caps, 10)
        self.assertEqual(caps.platform, "yardforce_nx60i")
        self.assertAlmostEqual(caps.max_linear_speed, REACHED, places=4)
        self.assertAlmostEqual(caps.max_angular_speed, 0.3, places=5)
        self.assertEqual(caps.safety_measurement_date, "")
        # The follower under the driver's limits: speed_fast cut to what max_duty reaches.
        follower = p.wait(lambda: [m["params"] for m in p.params if m["node"] == "/mowbite_follower"
                                   and abs(m["params"]["speed_fast"] - REACHED) < 1e-4], 10)
        self.assertTrue(follower, [m for m in p.params if m["node"] == "/mowbite_follower"])
        self.assertAlmostEqual(follower[-1]["speed_slow"], 0.15)
        self.assertAlmostEqual(follower[-1]["max_angular"], 0.3)
        # The gyro calibrates on the wheel reports of the driver: poses come.
        self.assertTrue(p.wait(lambda: p.pose, 30), "no pose: the gyro never calibrated")
        self.assertTrue(p.wait(lambda: any("Gyro offset" in line for line in p.logs), 5))
        rss = {name: [round(rss_mb(pid), 1) for pid in running(proc_info, name)]
               for name in ("xcore_driver_node", "component_container", "mowbite_planner_node")}
        print(f"RSS MB: {rss}", flush=True)
        self.assertEqual([len(v) for v in rss.values()], [1, 1, 1], rss)

    def test_2_mowing_and_drives_wait_for_the_safety_measurement(self):
        p = self.probe
        since = len(p.events)
        p.action_pub.publish(String(data="mower_logic:idle/start_mowing"))
        refused = p.event(since, "MOWING_REFUSED", 10, reason="NO_SAFETY_MEASUREMENT")
        self.assertTrue(refused, [e for e in p.events[since:]])
        answer = p.call(p.drive_to, DriveTo.Request(x=1.0, y=0.0))
        self.assertFalse(answer.success)
        self.assertIn("safety measurement", answer.message)
        self.assertTrue(p.event(since, "DRIVE_REFUSED", 10, reason="NO_SAFETY_MEASUREMENT"))
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_3_the_emergency_stop_latches_until_a_release(self):
        p = self.probe
        set_flags(stop=True)
        s = p.wait(lambda: p.status.emergency_reason & HardwareStatus.REASON_STOP and p.status, 3)
        self.assertTrue(s, "no STOP")
        self.assertTrue(p.wait(lambda: p.robot.emergency, 3))
        set_flags()
        time.sleep(0.5)
        self.assertTrue(p.status.emergency_latched)
        self.assertTrue(p.release().success)
        self.assertTrue(p.wait(lambda: not p.status.emergency, 3), p.status)

    def test_4_a_killed_driver_leaves_a_latched_successor(self, proc_info):
        p = self.probe
        driver, = running(proc_info, "xcore_driver_node")
        p.drive(0.1, 1.0)
        self.assertTrue(any(board()["applied_duty"]), board())
        t_kill = board()["t"]
        stop_driving = threading.Event()

        def keep_driving():
            while not stop_driving.is_set():
                p.cmd_vel.publish(Twist(linear=Vector3(x=0.1)))
                time.sleep(0.1)
        thread = threading.Thread(target=keep_driving, daemon=True)
        thread.start()
        try:
            os.kill(driver, signal.SIGKILL)
            # The board stops on its own 1 s after the last message of the host.
            stood = p.wait(lambda: not any(board()["applied_duty"]) and board()["t"], 3)
            self.assertTrue(stood, board())
            self.assertLess(stood - t_kill, 1.4)
            new = p.wait(lambda: [pid for pid in running(proc_info, "xcore_driver_node") if pid != driver], 10)
            self.assertTrue(new, "no new driver")
            claims = p.wait(lambda: received(t_kill, EMERGENCY, CLAIM), 10)
            self.assertTrue(claims, "the new driver never claimed the emergency service")
            # Its first emergency input already holds the board.
            first = p.wait(lambda: received(claims[-1]["t"], EMERGENCY, DATA_MSG), 5)
            self.assertTrue(first)
            bits = struct.unpack("<H", bytes.fromhex(first[0]["hex"])[:2])[0]
            self.assertEqual(bits & (BOARD_LATCH | BOARD_HIGH_LEVEL), BOARD_LATCH | BOARD_HIGH_LEVEL, first[0])
            s = p.wait(lambda: p.status.emergency_latched and p.status.emergency_reason
                       & HardwareStatus.REASON_HIGH_LEVEL and p.status, 10)
            self.assertTrue(s, p.status)
            # cmd_vel goes on all the time: the wheels stay still under the new driver.
            time.sleep(2.0)
            self.assertFalse(any(board()["applied_duty"]), board())
        finally:
            stop_driving.set()
            thread.join()
        self.assertEqual(note("driver_exit.json")["restart"], 1)
        self.assertEqual(note("driver_exit.json")["exit_code"], -signal.SIGKILL)
        self.assertTrue(p.release().success)
        self.assertTrue(p.wait(lambda: not p.status.emergency, 3), p.status)
        # Released, the new driver drives again.
        p.drive(0.1, 1.0)
        self.assertTrue(any(board()["applied_duty"]), board())

    def test_5_a_crashed_container_comes_back_beside_the_driver(self, proc_info):
        p = self.probe
        time.sleep(1.5)
        driver = running(proc_info, "xcore_driver_node")
        container, = running(proc_info, "component_container")
        os.kill(container, signal.SIGKILL)
        new = p.wait(lambda: [pid for pid in running(proc_info, "component_container") if pid != container], 15)
        self.assertTrue(new, "no new container")
        self.assertEqual(note("container_exit.json")["restart"], 1)
        self.assertEqual(running(proc_info, "xcore_driver_node"), driver)
        p.robot = None
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 60), p.robot)

    def test_6_after_three_restarts_the_driver_stays_down(self, proc_info):
        p = self.probe
        for restart in (2, 3):
            driver, = running(proc_info, "xcore_driver_node")
            os.kill(driver, signal.SIGKILL)
            self.assertTrue(p.wait(lambda: [pid for pid in running(proc_info, "xcore_driver_node")
                                            if pid != driver], 10), f"no driver for restart {restart}")
            self.assertEqual(note("driver_exit.json")["restart"], restart)
        driver, = running(proc_info, "xcore_driver_node")
        os.kill(driver, signal.SIGKILL)
        time.sleep(4.0)
        self.assertEqual(running(proc_info, "xcore_driver_node"), [])
        started = [a for a in proc_info.processes() if "xcore_driver_node" in a.process_details["name"]]
        self.assertEqual(len(started), 1 + DRIVER_RESTARTS)
        self.assertTrue(p.wait(lambda: board()["reason"] & 64, 3), board())


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly_or_were_killed(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -9, -15, 130, 143])
