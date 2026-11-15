"""Releasing the emergency with STOP at the mower (G37) in the simulation: the whole stack of sim.launch.py with the
gesture on (stop_release of tron_mcu) and fake_tron keeping STOP until a clear frame as the Tron does, so tron_mcu clears
it during the emergency (stop_clear_period). While mowing, STOP pauses; the press that latched never releases, however
long; after 2 s free a short press (0.1 to 2 s) releases once let go, with source button, and mowing stays paused until
continue. Too long, too quick and an IMU gap release nothing. The app asking while STOP is still held gets
STILL_ACTIVE, a new short press releases afterwards. The flags file of fake_tron plays the button.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_stop_release.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import (ButtonEvent, HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_stop_")
STATE, FLAGS, _, _ = sim_files(WORK)
CONFIG = os.path.join(WORK, "stop.yaml")
# hw/button is reliable and never latched.
BUTTON = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
# Free and held with a margin to the limits of 2, 0.1 and 2 s, for a loaded machine.
FREE = 3.0
HOLD = 0.8

LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    stop_release: true\n"
            "    stop_clear_period: 0.25\n"
            "mowbite_logic:\n  ros__parameters:\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18852, extra_config=CONFIG)


class Probe(SimProbe):
    # The board keeps STOP until a clear frame, as the Tron's MCU.
    faults = {"stop_latches": 1}

    def __init__(self):
        super().__init__("stop_release_probe", state=STATE, flags=FLAGS)
        self.status = None
        self.pose = None
        self.robot = None
        self.hw = None
        self.presses = []
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(ButtonEvent, "hw/button", self._button, BUTTON)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()

    def _button(self, msg):
        with self.lock:
            self.presses.append(msg)

    def press(self, since, kind, timeout):
        return self.wait(lambda: next((m for m in self.presses[since:] if m.press == kind), None), timeout)

    def free(self):
        """STOP free for FREE, counted from the clear frame that lets the board drop it."""
        if not self.wait(lambda: self.hw is not None and not self.hw.stop, 10):
            raise AssertionError("the board kept STOP, no clear frame")
        time.sleep(FREE)

    def hold(self, seconds, **during):
        """STOP held for seconds, then let go."""
        self.flags(stop=1, **during)
        time.sleep(seconds)
        self.flags()


class TestStopRelease(unittest.TestCase):
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

    def assertNoRelease(self, since, presses_since):
        p = self.probe
        time.sleep(1.0)
        self.assertTrue(p.robot.emergency)
        self.assertEqual([e for e in p.events[since:] if e["type"] == "EMERGENCY" and not e["emergency"]], [])
        self.assertIsNone(p.press(presses_since, ButtonEvent.PRESS_LONG, 0.1))

    def test_01_mowing_on_the_lawn(self):
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
        p.act("mower_logic:idle/start_mowing")
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "MOWING"
                               and p.robot.current_path >= 0, 60), "mowing did not start")

    def test_02_the_press_that_latched_never_releases(self):
        p = self.probe
        time.sleep(2.0)
        since, presses = len(p.events), len(p.presses)
        p.flags(stop=1)
        paused = p.event(since, "STATE", 5, state="PAUSED")
        self.assertIsNotNone(paused)
        self.assertEqual(paused["reason"], "EMERGENCY")
        time.sleep(6.0)
        p.flags()
        self.assertNoRelease(since, presses)
        self.assertEqual(p.presses[presses:], [])

    def test_03_free_then_held_releases_and_mowing_stays_paused(self):
        p = self.probe
        p.free()
        since, presses = len(p.events), len(p.presses)
        p.flags(stop=1)
        held = p.press(presses, ButtonEvent.PRESS_HELD, HOLD)
        self.assertIsNotNone(held, "no HELD after 0.1 s")
        self.assertTrue(p.robot.emergency)
        time.sleep(0.3)
        p.flags()
        released = p.press(presses, ButtonEvent.PRESS_LONG, 2.0)
        self.assertIsNotNone(released)
        self.assertNotEqual(released.latch_count, 0)
        end = p.event(since, "EMERGENCY", 5, emergency=False)
        self.assertIsNotNone(end, [e["type"] for e in p.events[since:]])
        self.assertEqual(end.get("source"), "button")
        self.assertNotIn("client", end)
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5))
        # Released, but nothing starts: it stays paused and standing until continue.
        time.sleep(1.0)
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual(p.robot.current_state, "PAUSED")
        since = len(p.events)
        p.act("mower_logic:mowing/continue")
        self.assertIsNotNone(p.event(since, "STATE", 5, state="MOWING"))

    def test_04_too_long_too_quick_and_an_imu_gap_release_nothing(self):
        p = self.probe
        time.sleep(2.0)
        since = len(p.events)
        p.hold(0.5)
        self.assertIsNotNone(p.event(since, "STATE", 5, state="PAUSED"))
        self.assertIsNotNone(p.event(since, "EMERGENCY", 5, emergency=True))
        since = len(p.events)
        # Too long: held at 0.1 s, canceled at 2 s, letting go does nothing.
        p.free()
        presses = len(p.presses)
        p.hold(3.5)
        self.assertNoRelease(since, presses)
        self.assertEqual([m.press for m in p.presses[presses:]], [ButtonEvent.PRESS_HELD, ButtonEvent.PRESS_CANCELED])
        # Too quick after the last press.
        time.sleep(0.8)
        presses = len(p.presses)
        p.hold(HOLD)
        self.assertNoRelease(since, presses)
        self.assertEqual(p.presses[presses:], [])
        # The IMU stops while it is held.
        p.free()
        presses = len(p.presses)
        p.flags(stop=1)
        self.assertIsNotNone(p.press(presses, ButtonEvent.PRESS_HELD, HOLD))
        p.flags(stop=1, imu_off=1)
        self.assertIsNotNone(p.press(presses, ButtonEvent.PRESS_CANCELED, 2.0))
        p.flags(stop=1)
        time.sleep(1.0)
        p.flags()
        self.assertNoRelease(since, presses)

    def test_05_the_app_while_held_then_the_gesture(self):
        p = self.probe
        p.free()
        since, presses = len(p.events), len(p.presses)
        p.flags(stop=1)
        self.assertIsNotNone(p.press(presses, ButtonEvent.PRESS_HELD, HOLD))
        # The driver takes the request, but STOP is held beyond its window: the emergency stays, and the answer says so
        # with the same event, which goes out as well.
        asked = time.time()
        answer = p.rpc("emergency.reset", {"client": "test"})
        self.assertLess(time.time() - asked, 5.0)
        data = ((answer or {}).get("error") or {}).get("data") or {}
        self.assertEqual((data.get("event"), data.get("reason")), ("EMERGENCY_RESET_REFUSED", "STILL_ACTIVE"), answer)
        self.assertIn("STOP", data.get("bits", []))
        refused = p.event(since, "EMERGENCY_RESET_REFUSED", 5, reason="STILL_ACTIVE")
        self.assertIsNotNone(refused, [e["type"] for e in p.events[since:]])
        self.assertEqual(data.get("event_id"), refused.get("id"))
        self.assertEqual(refused["source"], "rpc")
        self.assertEqual(refused["client"], "test")
        self.assertIn("STOP", refused["bits"])
        self.assertEqual(refused["severity"], "warning")
        self.assertTrue(p.robot.emergency)
        # Held past 2 s by now: letting go does nothing, a new short press after the free time releases.
        p.flags()
        p.free()
        presses = len(p.presses)
        p.hold(HOLD)
        self.assertIsNotNone(p.press(presses, ButtonEvent.PRESS_LONG, 2.0))
        end = p.event(since, "EMERGENCY", 5, emergency=False)
        self.assertIsNotNone(end)
        self.assertEqual(end.get("source"), "button")
        self.assertEqual(p.robot.current_state, "PAUSED")

    def test_06_a_new_press_after_the_release_latches_again(self):
        p = self.probe
        time.sleep(0.5)
        since = len(p.events)
        p.hold(0.3)
        self.assertIsNotNone(p.event(since, "EMERGENCY", 5, emergency=True))
        time.sleep(1.0)
        self.assertTrue(p.robot.emergency)
        # Leave it clean for the next run.
        answer = p.rpc("emergency.reset", {"client": "test"})
        self.assertEqual((answer or {}).get("result", {}).get("released"), True, answer)
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 5))
