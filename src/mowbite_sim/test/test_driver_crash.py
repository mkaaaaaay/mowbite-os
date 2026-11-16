"""The driver dies while the rest of MowBite OS runs, in the whole stack of sim.launch.py: tron_mcu is killed (SIGKILL)
in IDLE. The app sees the emergency with the reason of the missing hardware (TIMEOUT_INPUTS), the start notes the end
in driver_exit.json and does not start the driver of the Tron again, the stats keep one CRASH incident named after the
driver, and a reset of the emergency stays refused while no driver answers.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_driver_crash.py
"""
import os
import shutil
import signal
import sys
import tempfile
import time
import unittest

import launch_testing
import launch_testing.asserts
import rclpy
from mowbite_msgs.msg import HardwareStatus, MqttPublish, RobotState, RpcError, RpcRequest, RpcResponse
from mowbite_msgs.srv import SetEmergency
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, ROSOUT, SimProbe, container_pid, journal_dir, read_json,  # noqa: E402
                        running, sim_description, sim_files, wait_until)

# stack.py of mowbite_bringup; sim_common put its folder on the path.
import stack  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_driver_crash_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
NOTE = os.path.join(DATA, stack.DRIVER_EXIT_NOTE)
TIMEOUT_INPUTS = HardwareStatus.REASON_TIMEOUT_INPUTS
DRIVER = "tron_mcu_node"


def generate_test_description():
    return sim_description(WORK, 18847, journal_dir=JOURNAL)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("driver_crash_probe", state=STATE, flags=FLAGS, log=RX)
        self.robot = None
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()


def crashes(events):
    return [e for e in events if e["type"] == "INCIDENT" and e["reason"] == "CRASH"]


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

    def test_1_idle_with_the_journal(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 30), p.robot)
        self.assertFalse(p.robot.emergency, p.robot)
        status = wait_until(lambda: (p.rpc("journal.status", timeout=2) or {}).get("result"), 30)
        self.assertTrue(status and status["enabled"] and status["problem"] == "", status)
        self.assertFalse(os.path.exists(NOTE))

    def test_2_the_app_sees_why_and_the_driver_stays_down(self, proc_info):
        p = self.probe
        driver, = running(proc_info, DRIVER)
        container = container_pid(proc_info)
        TestDriverCrash.since = len(p.events)
        os.kill(driver, signal.SIGKILL)
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & TIMEOUT_INPUTS, 10), p.robot)
        note = wait_until(lambda: os.path.exists(NOTE) and read_json(NOTE), 5)
        self.assertTrue(note, "no driver_exit.json")
        self.assertEqual((note["name"], note["exit_code"], note["restart"], note["max_restarts"], note["down"]),
                         ("tron_mcu", -signal.SIGKILL, 0, 0, True), note)
        time.sleep(stack.RESTART_DELAY + 3.0)
        self.assertEqual(running(proc_info, DRIVER), [])
        self.assertTrue(os.path.exists(f"/proc/{container}"), "the container went along")
        self.assertEqual(p.robot.current_state, "IDLE")

    def test_3_the_end_of_the_driver_is_one_incident(self):
        p = self.probe
        since = TestDriverCrash.since
        event = p.event(since, "INCIDENT", 30, reason="CRASH")
        self.assertTrue(event, [(e["type"], e.get("reason")) for e in p.events[since:]])
        # Named after the note of the launch or after the run of tron_mcu without its end, whichever the stats found
        # first; both belong to the same incident.
        self.assertIn(event["detail"], ("tron_mcu", DRIVER), event)
        self.assertTrue(event["safety"], event)
        time.sleep(22.0)
        self.assertEqual(len(crashes(p.events[since:])), 1, crashes(p.events[since:]))
        trigger = p.crash_trigger(since, "tron_mcu")
        self.assertTrue(trigger, event)
        self.assertEqual(trigger["detail"], {"exit_code": -signal.SIGKILL, "restart": 0, "down": True}, trigger)

    def test_4_a_reset_is_refused_without_the_driver(self):
        p = self.probe
        # Once the graph has dropped the killed driver, mowbite_safety refuses without asking it.
        self.assertTrue(p.wait(lambda: not p.emergency.service_is_ready(), 30))
        since = len(p.events)
        p.act("mower_logic/reset_emergency")
        refused = p.event(since, "EMERGENCY_RESET_REFUSED", 10)
        self.assertTrue(refused, [(e["type"], e.get("reason")) for e in p.events[since:]])
        self.assertEqual(refused["reason"], "DRIVER_MISSING", refused)
        self.assertTrue(p.robot.emergency and p.robot.emergency_reason & TIMEOUT_INPUTS, p.robot)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_only_the_driver_was_killed(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[-signal.SIGKILL],
                                               process=DRIVER)
        others = [a for a in proc_info.processes() if DRIVER not in a.process_details["name"]]
        self.assertTrue(others)
        for action in others:
            self.assertIn(proc_info[action].returncode, EXIT_CODES, action.process_details["name"])
