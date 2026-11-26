"""A node of the container stalls while the mower mows (G39): the whole stack of sim.launch.py with the blade allowed,
as in test_stack_crash.py. Once the blade turns and the mower drives, one thread of the container is stopped (ptrace,
only that thread): first the one of the bridge, then the one of S2. The watch of the container (mowbite_watch) has to
end the process once the node gave no sign for its limit (bridge 10 s, S2 2 s), not before; then the way of a crash:
tron_mcu stops the wheels and the blade runs out, the container starts again, the restart hold latches and the event
UNEXPECTED_RESTART names the node with reason NODE_STALLED (cause stays OS_CRASH). With the bridge stalled the mower
mows on without remote control until the limit; with S2 stalled the wheels stop at once and the blade with the end.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_watch_stall.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch_testing.asserts
import rclpy
from mowbite_msgs.msg import HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, LATCHED, ROSOUT, WALL, SimProbe, container_pid, restarted,  # noqa: E402
                        sim_description, sim_files, stop_thread)

WORK = tempfile.mkdtemp(prefix="mowbite_watch_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
RESTART_HOLD = os.path.join(DATA, "restart-hold.json")
EXIT_NOTE = os.path.join(DATA, "container_exit.json")
WATCH_NOTE = os.path.join(DATA, "watch_exit.json")
CONFIG = os.path.join(WORK, "watch.yaml")
UNEXPECTED_RESTART = HardwareStatus.REASON_UNEXPECTED_RESTART
# The Tron: cmd_vel_timeout of tron_mcu and blade_lease of the logic.
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


def generate_test_description():
    return sim_description(WORK, 18834, extra_config=CONFIG)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("watch_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()

    def driving_now(self):
        """The time of the newest setpoint when it moves the mower and is no older than 0.2 s."""
        entries = self.recent()
        speeds = [e for e in entries if "speed" in e and e["t"] > entries[-1]["t"] - 0.2]
        return speeds[-1]["t"] if speeds and speeds[-1]["hex"] != ZERO else None


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

    def align(self):
        """The heading of fake_tron for the localization, then a good pose with it."""
        p = self.probe
        yaw = p.truth()["yaw"]
        request = SetPose.Request()
        request.pose.orientation.z = math.sin(yaw / 2)
        request.pose.orientation.w = math.cos(yaw / 2)
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def mow(self):
        p = self.probe
        since, _ = p.started(lambda: p.act("mowing.start"))
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.blade_motor.rpm > 1000, 5), "no rpm from fake_tron")

    def stall(self, proc_info, thread, limit):
        """Stops that thread while the mower drives; the times of fake_tron of the stall, of the last setpoint that
        moved the mower, of the final stop and of the last time the blade went off."""
        p = self.probe
        pid = container_pid(proc_info)
        offset = p.clock_offset()
        drove = p.wait(p.driving_now, 30)
        self.assertTrue(drove, "the mower does not drive")
        t0 = time.monotonic() - offset
        tid = stop_thread(pid, thread)
        self.assertTrue(reap(tid, limit + PERIOD + 10), f"{thread} did not end")
        self.assertTrue(restarted(proc_info, pid, RESTART_DELAY + 10), "no new container")
        rx = p.received(t0)
        # From the setpoint that showed the drive on: a stalled S2 may let no other follow it.
        speeds = [e for e in p.received(drove) if "speed" in e]
        moving = [e["t"] for e in speeds if e["hex"] != ZERO]
        self.assertTrue(moving, speeds[:5])
        # The stop for good: the first zero setpoint after the last one that moved.
        stop = next((e["t"] for e in speeds if e["hex"] == ZERO and e["t"] > moving[-1]), None)
        self.assertIsNotNone(stop, speeds[-5:])
        self.assertGreaterEqual(stop, t0, "the mower stood before the stall")
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
        self.align()

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
        p.act("emergency.reset")
        self.assertTrue(p.wait(lambda: not p.hw.emergency, 10), p.hw)
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5), p.robot)
        # The new container begins without a heading.
        self.align()
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
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES,
                                               process="tron_mcu_node")
