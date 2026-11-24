"""The choice of the mower (FP27 S1): mowbite.launch.py without MOWBITE_PLATFORM and without platform.json starts only
the bridge and the setup program, robot_state SETUP with sub state PLATFORM, no driver, no hw/status; the step
platform is the only one and open. setup.platform.set nx60i writes platform.json and ends the launch. Started again
it runs as the NX60i, here without its board: the driver has heard nothing and reports BOARD_NOT_HEARD, not latched,
the step platform is ok and setup.platforms names nx60i, not fixed. Another mower, chosen while it is idle, takes
every result back, closes the lock with a reset and ends the launch again.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_platform_choice.py
"""
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from launch.actions import ExecuteProcess
from mowbite_msgs.msg import HardwareStatus, RobotState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import SimProbe, journal_dir, loopback_group, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_platform_choice_")
_, _, _, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
PLATFORM_FILE = os.path.join(DATA, "platform.json")
LOCK = os.path.join(DATA, "safety_lock.json")
NOT_HEARD = HardwareStatus.REASON_BOARD_NOT_HEARD
# Without MOWBITE_PLATFORM, as an installation that leaves the choice to the app.
ENV = {key: value for key, value in os.environ.items() if key != "MOWBITE_PLATFORM"}
COMMAND = ["ros2", "launch", "mowbite_bringup", "mowbite.launch.py", f"data_dir:={DATA}", f"journal_dir:={JOURNAL}",
           f"mqtt_port:={random.randint(18000, 18999)}", "interface:=lo", f"bind_ip:={BIND_IP}",
           f"announcement_port:={GROUP_PORT}", "datum_lat:=52.0", "datum_long:=9.0", "datum_height:=80.0"]

os.makedirs(DATA, exist_ok=True)


def generate_test_description():
    system = ExecuteProcess(cmd=COMMAND, env=ENV, output="screen", sigterm_timeout="30", sigkill_timeout="30")
    return launch.LaunchDescription([system, launch_testing.actions.ReadyToTest()]), {"system": system}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("platform_choice_probe")
        self.robot = None
        self.status = None
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.start()

    def setup(self, method, **params):
        """The answer of a method setup.* with the version of the state now."""
        state = (self.rpc("setup.get", {}, 10) or {}).get("result", {})
        return self.rpc(method, {**params, "version": state.get("version")}, 10) or {}

    def step(self, step):
        state = (self.rpc("setup.get", {}, 10) or {}).get("result", {})
        return next((entry for entry in state.get("steps", []) if entry["id"] == step), None)


class TestPlatformChoice(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_the_stage_of_the_choice(self, proc_info, system):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot and (p.robot.current_state, p.robot.current_sub_state)
                               == ("SETUP", "PLATFORM"), 60), p.robot)
        self.assertFalse(p.robot.hardware)
        mowers = p.rpc("setup.platforms", {}, 10)["result"]
        self.assertEqual((mowers["current"], mowers["fixed"]), ("", False), mowers)
        names = [entry["name"] for entry in mowers["platforms"]]
        self.assertIn("nx60i", names)
        self.assertIn("tron", names)
        self.assertEqual(p.step("platform")["state"], "open")
        # No driver: nothing reports hw/status.
        self.assertIsNone(p.status)
        self.assertEqual(p.setup("setup.platform.set", platform="yf500")["error"]["data"]["reason"],
                         "UNKNOWN_PLATFORM")
        since = len(p.events)
        self.assertEqual(p.setup("setup.platform.set", platform="nx60i")["result"], {"restart": True})
        event = p.event(since, "SETUP", 10, what="platform")
        self.assertTrue(event, p.events[since:])
        self.assertEqual((event["platform"], event["from"]), ("nx60i", ""))
        proc_info.assertWaitForShutdown(process=system, timeout=60)
        with open(PLATFORM_FILE) as f:
            self.assertEqual(json.load(f)["platform"], "nx60i")

    def test_02_started_again_as_the_mower_chosen(self):
        p = self.probe
        p.robot = p.status = None
        with open(os.path.join(WORK, "second.log"), "w") as log:
            second = subprocess.Popen(COMMAND, env=ENV, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            # Its board never answers: the driver sent nothing and says so, without a latch.
            self.assertTrue(p.wait(lambda: p.status and p.status.emergency_reason & NOT_HEARD, 60), p.status)
            self.assertFalse(p.status.telemetry_ok)
            self.assertFalse(p.status.emergency_latched)
            self.assertTrue(p.wait(lambda: p.robot and p.robot.current_state == "IDLE", 60), p.robot)
            self.assertFalse(p.robot.hardware)
            mowers = p.rpc("setup.platforms", {}, 10)["result"]
            self.assertEqual((mowers["current"], mowers["fixed"]), ("nx60i", False), mowers)
            self.assertEqual(p.step("platform")["state"], "ok")
            # The wrong mower: another one, chosen while it stands, closes the lock until its summary passed.
            since = len(p.events)
            self.assertEqual(p.setup("setup.platform.set", platform="tron")["result"], {"restart": True})
            event = p.event(since, "SETUP", 10, what="platform")
            self.assertTrue(event, p.events[since:])
            self.assertEqual((event["platform"], event["from"]), ("tron", "nx60i"))
            self.assertEqual(second.wait(60), 0)
        finally:
            if second.poll() is None:
                os.killpg(second.pid, signal.SIGINT)
                second.wait(30)
        with open(PLATFORM_FILE) as f:
            self.assertEqual(json.load(f)["platform"], "tron")
        with open(LOCK) as f:
            self.assertTrue(any(cause.get("reset") for cause in json.load(f)["causes"]))


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_the_choice_ended_the_launch_in_order(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
