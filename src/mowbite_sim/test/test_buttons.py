"""The buttons of the mower in the simulation, sent on hw/button beside the driver: START held starts mowing, START
short pauses, START held goes on, POWER held pauses, HOME short ends mowing and goes home; each step tells the client
button, and a press of a button the logic does not take (HOME held) changes nothing.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_buttons.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import ButtonEvent, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_buttons_")
STATE, FLAGS, RX, DATA = sim_files(WORK)

# A lawn around the mower at (0, 0), large enough to mow a while.
LAWN = [(-0.41, -0.11), (5.59, -0.11), (5.59, 4.89), (-0.41, 4.89)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}


def generate_test_description():
    return sim_description(WORK, 18839)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("buttons_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.button_pub = self.node.create_publisher(ButtonEvent, "hw/button", 10)
        self.start()

    def press(self, button, press):
        msg = ButtonEvent()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.button = button
        msg.press = press
        self.button_pub.publish(msg)


class TestButtons(unittest.TestCase):
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

    def state_by_button(self, since, state):
        """The STATE event with state after since; an event of the step names the client button."""
        p = self.probe
        event = p.event(since, "STATE", 60, state=state)
        seen = [(e["type"], e.get("state"), e.get("client")) for e in p.events[since:]]
        self.assertIsNotNone(event, seen)
        self.assertTrue(any(e.get("client") == "button" for e in p.events[since:]), seen)
        return event

    def test_01_the_buttons_start_pause_go_on_and_go_home(self):
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
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 30))

        # HOME held stands for nothing: no step.
        since = len(p.events)
        p.press(ButtonEvent.BUTTON_HOME, ButtonEvent.PRESS_HELD)
        time.sleep(2.0)
        self.assertEqual([e for e in p.events[since:] if e["type"] == "STATE"], [])

        since, _ = p.started(lambda: p.press(ButtonEvent.BUTTON_START, ButtonEvent.PRESS_HELD))
        self.state_by_button(since, "MOWING")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "MOWING", 60), "no mowing")

        since = len(p.events)
        p.press(ButtonEvent.BUTTON_START, ButtonEvent.PRESS_SHORT)
        paused = self.state_by_button(since, "PAUSED")
        self.assertEqual(paused.get("message"), "Paused at the mower")

        since = len(p.events)
        p.press(ButtonEvent.BUTTON_START, ButtonEvent.PRESS_HELD)
        self.state_by_button(since, "MOWING")

        since = len(p.events)
        p.press(ButtonEvent.BUTTON_POWER, ButtonEvent.PRESS_HELD)
        self.state_by_button(since, "PAUSED")

        since = len(p.events)
        p.press(ButtonEvent.BUTTON_HOME, ButtonEvent.PRESS_SHORT)
        self.assertTrue(p.wait(lambda: p.robot.current_state != "MOWING", 30), "mowing did not end")
        ended = [e for e in p.events[since:] if e.get("client") == "button"]
        self.assertTrue(ended, [(e["type"], e.get("state")) for e in p.events[since:]])


if __name__ == "__main__":
    unittest.main()
