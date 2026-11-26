"""MowBite OS on the NX60i as mowbite.launch.py starts it, against fake_xcore on UDP over the loopback: the stack
comes up IDLE with the capabilities of the speed model and the speeds for the planner on bridge/params, the gyro
calibrates on the wheel reports of the driver; without a safety measurement mowing and drives are refused; the
emergency stop latches and is released; on the station the charge of the board is confirmed (DOCKED) and its loss
told (CHARGING_LOST), with the values of the NX60i; a killed driver stands the wheels (the board's own timeout) and
its successor holds the board from its first frame until a release; a crashed container comes back beside the running
driver; after three restarts the driver stays down. Prints the RSS of driver, container, planner and mowbite_net.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_nx60i_stack.py
"""
import functools
import json
import os
import random
import shutil
import signal
import struct
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from geometry_msgs.msg import Twist, Vector3
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from mowbite_msgs.msg import (HardwareCapabilities, HardwareStatus, MqttPublish, RobotPose, RobotState, RpcError,
                              RpcMethods, RpcRequest, RpcResponse)
from mowbite_msgs.srv import DriveTo, SetEmergency
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import XCORE_CLAIM as CLAIM, XCORE_DATA as DATA_MSG, XCORE_EMERGENCY as EMERGENCY  # noqa: E402
from sim_common import (HISTORY, LATCHED, ROSOUT, SimProbe, fake_xcore, journal_dir, loopback_group,  # noqa: E402
                        read_json, running, sim_files, wait_until, write_json, xcore_datagrams)

# stack.py of mowbite_bringup; sim_common put its folder on the path.
import stack as bringup  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_nx60i_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
# The incidents need a journal.
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
# The bits of the board as the driver sends them.
BOARD_LATCH, BOARD_HIGH_LEVEL = 1, 128
DRIVER_RESTARTS = bringup.platform_module("nx60i").DRIVER_RESTARTS
# max_duty 0.3 of nx60i.yaml in the speed model 0.58 x (duty - 0.019).
REACHED = 0.58 * (0.3 - 0.019)
SETUP_METHODS = ["setup.get", "setup.start", "setup.answer", "setup.skip", "setup.abort", "setup.keep",
                 "setup.reset", "setup.platforms", "setup.platform.set", "setup.image"]
# Given as user and password of an NTRIP access: neither may reach the state or the record of the setup program.
ACCESS_MARKERS = {"user": "marker-user-5b1f", "password": "marker-password-93ce"}

# ntrip.set, which the stack leaves off, for the test that an access never reaches the setup program.
CONFIG = os.path.join(WORK, "extra.yaml")

write_json(FLAGS, {"gps_fix": "fix"})
with open(CONFIG, "w") as f:
    f.write("mowbite_ntrip:\n  ros__parameters:\n    access_over_rpc: true\n")


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--log", LOG, "--log-max-mb", "60")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("mowbite_bringup").find("mowbite_bringup"),
                                                   "launch", "mowbite.launch.py")),
        launch_arguments={"platform": "nx60i", "interface": "lo", "bind_ip": BIND_IP,
                          "announcement_port": str(GROUP_PORT), "data_dir": DATA, "journal_dir": JOURNAL,
                          "extra_config": CONFIG,
                          "datum_lat": str(DATUM[0]),
                          "datum_long": str(DATUM[1]), "datum_height": str(DATUM[2]),
                          "mqtt_port": str(random.randint(18000, 18999))}.items())
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[stack, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(**flags):
    flags.setdefault("gps_fix", "fix")
    write_json(FLAGS, flags)


board = functools.partial(read_json, STATE)


received = functools.partial(xcore_datagrams, LOG)


def saved_record(version):
    """The text of setup.json once it has the version, else None."""
    try:
        with open(os.path.join(DATA, "setup.json")) as f:
            text = f.read()
        return text if json.loads(text).get("version", -1) >= version else None
    except (OSError, ValueError):
        return None


class Probe(SimProbe):
    offer_timeout = 20.0
    call_timeout = 20.0

    def __init__(self):
        super().__init__("nx60i_probe")
        self.status = None
        self.caps = None
        self.robot = None
        self.pose = None
        self.params = []
        # Every setup/json as (payload, retained).
        self.setup_states = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("caps"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(String, "bridge/params", self._param, HISTORY)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.start()

    def _param(self, msg):
        with self.lock:
            self.params.append(json.loads(msg.data))

    def _publish(self, msg):
        if msg.topic == "setup/json":
            with self.lock:
                self.setup_states.append((msg.payload, msg.retain))
        super()._publish(msg)

    def release(self):
        return self.set_emergency(False)

    def setup_state(self):
        answer = self.rpc("setup.get")
        assert answer and "result" in answer, f"setup.get: {answer}"
        return answer["result"]

    def setup_change(self, method, **params):
        """A setup.* call with the version of now, once more if the state moved by itself in between; its answer."""
        for _ in range(2):
            answer = self.rpc(method, {**params, "version": self.setup_state()["version"]})
            data = ((answer or {}).get("error") or {}).get("data") or {}
            if data.get("reason") != "VERSION":
                break
        return answer

    drive = SimProbe.cmd_vel_for


def rss_mb(pid):
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024
    return None


def note(name):
    return read_json(os.path.join(DATA, name))


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
               for name in ("xcore_driver_node", "component_container_isolated", "mowbite_planner_os_node",
                            "mowbite_net")}
        print(f"RSS MB: {rss}", flush=True)
        self.assertEqual([len(v) for v in rss.values()], [1, 1, 1, 1], rss)

    def test_2_mowing_and_drives_wait_for_the_safety_measurement(self):
        p = self.probe
        since = len(p.events)
        p.action_pub.publish(String(data="mowing.start"))
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

    def test_3b_charging_on_the_station_is_confirmed_and_its_loss_told(self):
        p = self.probe
        # On the station at 26.5 V (54 %) the board charges with 0.88 A by itself: DOCKED once it held 5 s.
        since = len(p.events)
        set_flags(dock=1)
        self.assertTrue(p.wait(lambda: p.status.dock_contact and p.status.battery_current < -0.8, 5), p.status)
        self.assertTrue(p.event(since, "DOCKED", 15), p.events[since:])
        # The current stops below 75 %: CHARGING_LOST after 10 s, the mower stays where it is.
        since = len(p.events)
        set_flags(dock=1, charge_current=0.0)
        lost = p.event(since, "CHARGING_LOST", 20, reason="NO_CHARGE_CURRENT")
        self.assertTrue(lost, p.events[since:])
        self.assertEqual((lost["severity"], lost["resume"]), ("warning", "user"))
        self.assertEqual(lost["battery_percentage"], 54)
        # Charging again: DOCKED; off the station: CHARGING_LOST NO_DOCK_CONTACT.
        since = len(p.events)
        set_flags(dock=1)
        self.assertTrue(p.event(since, "DOCKED", 15), p.events[since:])
        since = len(p.events)
        set_flags()
        self.assertTrue(p.event(since, "CHARGING_LOST", 20, reason="NO_DOCK_CONTACT"), p.events[since:])
        # Full, the charger done without a current: the contact alone, then off again.
        since = len(p.events)
        set_flags(dock=1, battery_voltage=28.3)
        self.assertTrue(p.event(since, "DOCKED", 15), p.events[since:])
        since = len(p.events)
        set_flags()
        self.assertTrue(p.event(since, "CHARGING_LOST", 20, reason="NO_DOCK_CONTACT"), p.events[since:])
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_3c_the_setup_program_answers_and_sends_its_state_retained(self):
        p = self.probe
        self.assertEqual(p.wait(lambda: p.methods.get("mowbite_setup"), 10), SETUP_METHODS)
        state = p.setup_state()
        self.assertEqual(state["platform"], p.caps.platform)
        self.assertTrue(state["steps"])
        # Sent on every change and every 30 s, always retained for an app that comes later.
        self.assertTrue(p.wait(lambda: p.setup_states, 35), "no setup/json")
        with p.lock:
            sent = list(p.setup_states)
        self.assertTrue(all(retained for _, retained in sent), [retained for _, retained in sent])
        last = json.loads(sent[-1][0])
        self.assertEqual((last["platform"], last["catalog"]), (state["platform"], state["catalog"]))
        self.assertEqual([step["id"] for step in last["steps"]], [step["id"] for step in state["steps"]])

    def test_3d_no_access_data_in_the_state_or_the_record_of_the_setup_program(self):
        p = self.probe
        self.assertIn("ntrip.set", p.wait(lambda: p.methods.get("mowbite_ntrip"), 10) or [])
        self.assertEqual(p.rpc("ntrip.get"), {"result": {"configured": False}})
        access = p.rpc("ntrip.set", {"host": "127.0.0.1", "port": 9, "mountpoint": "CHECK", **ACCESS_MARKERS})
        self.assertIn("result", access or {}, access)
        try:
            step = next((s for s in p.setup_state()["steps"] if s["skippable"] and s["state"] in ("open", "locked")
                         and "later" in [reason["id"] for reason in s["skip_reasons"]]), None)
            self.assertIsNotNone(step, "no step to skip with later")
            # The skip writes the record and sends the state.
            answer = p.setup_change("setup.skip", step=step["id"], reason="later")
            self.assertIn("result", answer or {}, answer)
            version = answer["result"]["version"]
            self.assertTrue(p.wait(lambda: p.setup_states and json.loads(p.setup_states[-1][0])["version"] >= version,
                                   5), "no setup/json after the skip")
            # The record is written on a thread of its own (FileWriter): it may come a moment after the state.
            record = p.wait(lambda: saved_record(version), 5)
            self.assertTrue(record, "no setup.json of the version after the skip")
            with p.lock:
                sent = p.setup_states[-1][0]
            texts = {"setup.get": json.dumps(p.setup_state()), "setup/json": sent, "setup.json": record}
            for where, text in texts.items():
                for marker in ACCESS_MARKERS.values():
                    self.assertNotIn(marker, text, where)
        finally:
            deleted = p.rpc("ntrip.delete")
        self.assertEqual(deleted, {"result": {"configured": False}})

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
        since = len(p.events)
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
        self.assertEqual(note(bringup.DRIVER_EXIT_NOTE)["restart"], 1)
        self.assertEqual(note(bringup.DRIVER_EXIT_NOTE)["exit_code"], -signal.SIGKILL)
        # The app sees an emergency with a reason it knows. The end of the driver is a trigger of an incident, its own
        # or the one that emergency opened before, though xcore_driver keeps no journal of its own.
        self.assertTrue(p.robot.emergency and p.robot.emergency_reason
                        & (HardwareStatus.REASON_TIMEOUT_INPUTS | HardwareStatus.REASON_HIGH_LEVEL), p.robot)
        trigger = wait_until(lambda: p.crash_trigger(since, "xcore_driver"), 20, poll=1.0)
        self.assertTrue(trigger, [(e["type"], e.get("reason")) for e in p.events[since:]])
        self.assertEqual((trigger["safety"], trigger["detail"]),
                         (True, {"exit_code": -signal.SIGKILL, "restart": 1, "down": False}), trigger)
        self.assertTrue(p.release().success)
        self.assertTrue(p.wait(lambda: not p.status.emergency, 3), p.status)
        # Released, the new driver drives again.
        p.drive(0.1, 1.0)
        self.assertTrue(any(board()["applied_duty"]), board())

    def test_5_a_crashed_container_comes_back_beside_the_driver(self, proc_info):
        p = self.probe
        time.sleep(1.5)
        driver = running(proc_info, "xcore_driver_node")
        container, = running(proc_info, "component_container_isolated")
        os.kill(container, signal.SIGKILL)
        new = p.wait(lambda: [pid for pid in running(proc_info, "component_container_isolated") if pid != container],
                     15)
        self.assertTrue(new, "no new container")
        self.assertEqual(note(bringup.EXIT_NOTE)["restart"], 1)
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
            self.assertEqual(note(bringup.DRIVER_EXIT_NOTE)["restart"], restart)
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
