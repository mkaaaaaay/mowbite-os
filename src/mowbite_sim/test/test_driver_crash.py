"""The driver dies while the mower mows, in the whole stack of sim.launch.py with the blade allowed: tron_mcu is killed
(SIGKILL) on a path with the blade on. fake_tron, like the MCU, drops the setpoint and the blade by itself once the
host is silent. The start notes the end in driver_exit.json and starts the driver again, latched from its first frame
(start_latched): it sends no setpoint that moves a wheel and no blade frame that turns the blade until the emergency
is released in the app, and a release alone starts neither. The app sees the emergency with a reason it knows, the
stats keep one CRASH incident named after the driver. After DRIVER_RESTARTS restarts the driver stays down, and a
reset of the emergency is refused while no driver answers.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_driver_crash.py
"""
import json
import os
import shutil
import signal
import sys
import tempfile
import time
import unittest

import launch_testing
import rclpy
from mowbite_msgs.msg import (HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetEmergency, SetPose
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, LATCHED, ROSOUT, SimProbe, alive, container_pid, journal_dir,  # noqa: E402
                        read_json, running, sim_description, sim_files, wait_until)

# stack.py of mowbite_bringup; sim_common put its folder on the path.
import stack  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_driver_crash_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
NOTE = os.path.join(DATA, stack.DRIVER_EXIT_NOTE)
CONFIG = os.path.join(WORK, "crash.yaml")
TIMEOUT_INPUTS = HardwareStatus.REASON_TIMEOUT_INPUTS
HIGH_LEVEL = HardwareStatus.REASON_HIGH_LEVEL
DRIVER = "tron_mcu_node"
RESTARTS = stack.platform_module("tron").DRIVER_RESTARTS
# fake_tron drops the setpoint this long after the last SpeedData (its --host-timeout), with room for a slow machine.
STOP_WITHIN = 1.0
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
    # Short segments, so that an incident is sealed soon after its window.
    f.write("/**:\n  ros__parameters:\n    journal_segment_s: 20.0\n"
            "tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18847, extra_config=CONFIG, journal_dir=JOURNAL)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("driver_crash_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def driving_now(self):
        """fake_tron time of its newest entry if the newest setpoint it got moves the mower."""
        entries = self.recent()
        speeds = [e for e in entries if "speed" in e and e["t"] > entries[-1]["t"] - 0.2]
        return entries[-1]["t"] if speeds and speeds[-1]["hex"] != ZERO else None

    def commands(self, since):
        """What the host sent since that time of fake_tron: the setpoints that move a wheel and the blade frames that
        turn the blade."""
        rx = self.received(since)
        return ([e for e in rx if "speed" in e and e["hex"] != ZERO], [e for e in rx if e.get("cutter", 0) != 0])


def crashes(events):
    return [e for e in events if e["type"] == "INCIDENT" and e["reason"] == "CRASH"]


def note():
    return read_json(NOTE) if os.path.exists(NOTE) else None


def successor(proc_info, pid, timeout):
    """The pid of the driver that runs after pid, else None."""
    return wait_until(lambda: next((p for p in running(proc_info, DRIVER) if p != pid), None), timeout)


class TestDriverCrash(unittest.TestCase):
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

    def test_1_lawn_and_an_aligned_heading(self):
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
        status = wait_until(lambda: (p.rpc("journal.status", timeout=2) or {}).get("result"), 30)
        self.assertTrue(status and status["enabled"] and status["problem"] == "", status)
        self.assertFalse(os.path.exists(NOTE))

    def test_2_the_blade_turns_and_the_mower_drives(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.blade_motor.rpm > 1000, 5), "no rpm from fake_tron")
        self.assertTrue(p.wait(lambda: (p.truth() or {}).get("linear", 0.0) > 0.05, 30), "the mower does not drive")

    def test_3_the_mcu_stops_and_the_successor_is_latched(self, proc_info):
        p = self.probe
        driver, = running(proc_info, DRIVER)
        container = container_pid(proc_info)
        offset = p.clock_offset()
        # Not in a stop at a corner: fake_tron holds a setpoint that moves the mower.
        t0 = p.wait(p.driving_now, 30)
        self.assertTrue(t0, "the mower does not drive")
        TestDriverCrash.since = len(p.events)
        kill = time.monotonic() - offset
        os.kill(driver, signal.SIGKILL)
        # No host any more: the MCU drops the setpoint and the blade by itself.
        stood = p.wait(lambda: (p.truth() or {}).get("linear") == 0.0 and p.truth()["clock"] - offset, 5)
        self.assertTrue(stood, p.truth())
        self.assertLess(stood - kill, STOP_WITHIN, f"wheels stopped {stood - kill:.2f} s after the kill")
        # Frames the old driver wrote before its end can reach fake_tron after the kill; once the wheels stand,
        # fake_tron has read them all, so frames after its newest entry are the successor's.
        self.assertTrue(wait_until(lambda: not alive(driver), 2), "the old driver still runs")
        time.sleep(0.2)
        quiet = p.recent(4096)[-1]["t"]
        self.assertEqual(running(proc_info, DRIVER), [], "the successor started before the old frames were read")
        TestDriverCrash.quiet = quiet
        new = successor(proc_info, driver, stack.RESTART_DELAY + 10)
        self.assertTrue(new, "no new driver")
        TestDriverCrash.driver = new
        hw = p.wait(lambda: p.hw.emergency_latched and p.hw.emergency_reason & HIGH_LEVEL and p.hw, 10)
        self.assertTrue(hw, p.hw)
        self.assertFalse(hw.blade_on or hw.moving, hw)
        self.assertTrue(p.wait(lambda: p.hw.blade_motor.ok and p.hw.blade_motor.rpm == 0, 5), p.hw)
        written = note()
        self.assertTrue(written, "no driver_exit.json")
        self.assertEqual((written["name"], written["exit_code"], written["restart"], written["max_restarts"],
                          written["down"]), ("tron_mcu", -signal.SIGKILL, 1, RESTARTS, False), written)
        self.assertTrue(os.path.exists(f"/proc/{container}"), "the container went along")
        self.assertEqual(container_pid(proc_info), container)
        # The app sees an emergency with a reason it knows.
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & (TIMEOUT_INPUTS | HIGH_LEVEL),
                               10), p.robot)
        moving, cutting = p.commands(quiet)
        self.assertEqual((moving, cutting), ([], []), "the new driver moved the mower or turned the blade")
        print(f"driver killed: wheels stopped {stood - kill:.2f} s after it, successor latched "
              f"{p.truth()['clock'] - offset - kill:.2f} s after it, reason {hw.emergency_reason:#x}, "
              f"state {p.robot.current_state}", flush=True)

    def test_4_nothing_moves_until_a_release_and_none_by_it(self):
        p = self.probe
        quiet = TestDriverCrash.quiet
        time.sleep(3.0)
        self.assertEqual(p.commands(quiet), ([], []))
        self.assertTrue(p.hw.emergency_latched and p.hw.emergency_reason & HIGH_LEVEL, p.hw)
        still, poses = p.standing()
        self.assertTrue(still and poses[1]["linear"] == 0.0, poses)
        since = len(p.events)
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.robot.emergency and not p.hw.emergency, 10),
                        (p.robot, p.hw, [(e["type"], e.get("reason")) for e in p.events[since:]]))
        # The release alone starts neither the wheels nor the blade: the job waits paused until the user continues.
        time.sleep(5.0)
        self.assertEqual(p.commands(quiet), ([], []))
        self.assertEqual(p.robot.current_state, "PAUSED", p.robot)
        self.assertEqual(p.hw.blade_motor.rpm, 0, p.hw)

    def test_5_the_end_of_the_driver_is_one_incident(self):
        p = self.probe
        since = TestDriverCrash.since
        trigger = wait_until(lambda: p.crash_trigger(since, "tron_mcu"), 30, poll=1.0)
        self.assertTrue(trigger, [(e["type"], e.get("reason")) for e in p.events[since:]])
        self.assertEqual((trigger["safety"], trigger["detail"]),
                         (True, {"exit_code": -signal.SIGKILL, "restart": 1, "down": False}), trigger)
        time.sleep(22.0)
        self.assertLessEqual(len(crashes(p.events[since:])), 1, crashes(p.events[since:]))

    def test_6_after_its_restarts_the_driver_stays_down(self, proc_info):
        p = self.probe
        driver = TestDriverCrash.driver
        for restart in range(2, RESTARTS + 1):
            self.assertEqual(running(proc_info, DRIVER), [driver])
            os.kill(driver, signal.SIGKILL)
            driver = successor(proc_info, driver, stack.RESTART_DELAY + 10)
            self.assertTrue(driver, f"no driver for restart {restart}")
            self.assertEqual(note()["restart"], restart)
            # Every one of them comes up latched.
            self.assertTrue(p.wait(lambda: p.hw.emergency_latched and p.hw.emergency_reason & HIGH_LEVEL, 10), p.hw)
        os.kill(driver, signal.SIGKILL)
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & TIMEOUT_INPUTS, 10), p.robot)
        time.sleep(stack.RESTART_DELAY + 3.0)
        self.assertEqual(running(proc_info, DRIVER), [])
        written = note()
        self.assertEqual((written["restart"], written["max_restarts"], written["down"]), (RESTARTS, RESTARTS, True),
                         written)
        started = [a for a in proc_info.processes() if DRIVER in a.process_details["name"]]
        self.assertEqual(len(started), 1 + RESTARTS)
        self.assertEqual(p.commands(TestDriverCrash.quiet), ([], []))

    def test_7_a_reset_is_refused_without_the_driver(self):
        p = self.probe
        # Once the graph has dropped the killed driver, mowbite_safety refuses without asking it. Its own view of
        # the graph can lag the probe's: then it asks, gets no answer and refuses too, and the next reset is the check.
        self.assertTrue(p.wait(lambda: not p.emergency.service_is_ready(), 30))
        for _ in range(2):
            since = len(p.events)
            p.act("mower_logic/reset_emergency")
            refused = p.event(since, "EMERGENCY_RESET_REFUSED", 10)
            self.assertTrue(refused, [(e["type"], e.get("reason")) for e in p.events[since:]])
            if refused["reason"] != "NO_ANSWER":
                break
        self.assertEqual(refused["reason"], "DRIVER_MISSING", refused)
        self.assertTrue(p.robot.emergency and p.robot.emergency_reason & TIMEOUT_INPUTS, p.robot)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_only_the_drivers_were_killed(self, proc_info):
        drivers = [a for a in proc_info.processes() if DRIVER in a.process_details["name"]]
        self.assertEqual([proc_info[a].returncode for a in drivers], [-signal.SIGKILL] * (1 + RESTARTS))
        others = [a for a in proc_info.processes() if a not in drivers]
        self.assertTrue(others)
        for action in others:
            self.assertIn(proc_info[action].returncode, EXIT_CODES, action.process_details["name"])
