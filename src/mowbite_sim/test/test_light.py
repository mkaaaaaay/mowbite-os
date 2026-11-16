"""The lights of MowBite OS behind the app interface: mowbite_light with its file output and mowbite_bridge on a private
mosquitto. The test plays the app (MQTT) and the logic (bridge/robot_state) and reads the WS2812 frames from the file.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_light.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import TimerAction
from mowbite_msgs.msg import RobotState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (bridge_node, layers, mosquitto, mqtt_connect, mqtt_rpc, package_program,  # noqa: E402
                        stop_process, store_answer)

MQTT_PORT = 18834
WORK = tempfile.mkdtemp(prefix="mowbite_light_")
DATA_DIR = os.path.join(WORK, "data")
FRAME = os.path.join(WORK, "frame")
LEDS = 55
BUFFER_LEDS = 60
# 120 of 255 at full brightness of an effect.
MAX_BRIGHTNESS = 0.47
RESET_BYTES = 100
FRONT = 14


def generate_test_description():
    os.makedirs(DATA_DIR)
    broker = mosquitto(WORK, MQTT_PORT)
    bridge = bridge_node(WORK, MQTT_PORT)
    return launch.LaunchDescription([
        broker,
        TimerAction(period=1.0, actions=[bridge, launch_testing.actions.ReadyToTest()]),
    ]), {}


def decode(data):
    """The colours (r, g, b) of a WS2812 frame: GRB per LED, one byte per bit (0xFC 1, 0xC0 0), resets around."""
    body = data[RESET_BYTES:len(data) - RESET_BYTES]
    if len(data) < 2 * RESET_BYTES or any(data[:RESET_BYTES]) or any(data[len(data) - RESET_BYTES:]) or len(body) % 24:
        return None
    bits = []
    for b in body:
        if b not in (0xFC, 0xC0):
            return None
        bits.append(1 if b == 0xFC else 0)
    leds = []
    for i in range(0, len(bits), 24):
        g, r, b = (int("".join(map(str, bits[i + k * 8:i + k * 8 + 8])), 2) for k in range(3))
        leds.append((r, g, b))
    return leds


def tron_strips():
    return layers.parameters(layers.files("tron"), "mowbite_light")["strips"]


class TestLight(unittest.TestCase):
    process = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("light_probe")
        cls.state_pub = cls.node.create_publisher(RobotState, "bridge/robot_state", 10)
        cls.lock = threading.Lock()
        cls.answers = {}
        cls.client = mqtt_connect("light_probe", MQTT_PORT, cls.on_message, "rpc/response")
        cls.process = subprocess.Popen(
            [package_program("mowbite_light", "mowbite_light_node"), "--ros-args", "-p", "output:=file", "-p",
             f"file:={FRAME}", "-p", f"data_dir:={DATA_DIR}", "-p", f"leds:={LEDS}", "-p",
             f"buffer_leds:={BUFFER_LEDS}", "-p", f"max_brightness:={MAX_BRIGHTNESS}", "-p", "max_current_ma:=100000.0",
             "-p", f"strips:={json.dumps(tron_strips())}"], stdout=subprocess.DEVNULL)

    @classmethod
    def tearDownClass(cls):
        cls.stop_node()
        cls.client.loop_stop()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def stop_node(cls):
        if cls.process is not None:
            stop_process(cls.process)
            cls.process = None

    @classmethod
    def on_message(cls, client, userdata, message):
        store_answer(cls.answers, cls.lock, message.payload)

    def rpc(self, method, params=None, timeout=10.0):
        return mqtt_rpc(self.client, self.answers, self.lock, method, params, timeout)

    def publish_state(self, state, emergency=False, charging=False, battery=0.8):
        msg = RobotState()
        msg.current_state = state
        msg.emergency = emergency
        msg.is_charging = charging
        msg.battery_percentage = battery
        self.state_pub.publish(msg)

    def frame_with(self, predicate, timeout=10.0, state=None):
        """The first frame from the file that predicate accepts; state is published again meanwhile."""
        end = time.time() + timeout
        leds = None
        while time.time() < end:
            if state is not None:
                self.publish_state(*state)
            try:
                with open(FRAME, "rb") as f:
                    leds = decode(f.read())
            except FileNotFoundError:
                leds = None
            if leds is not None and predicate(leds):
                return leds
            time.sleep(0.05)
        self.fail(f"no matching frame, last {leds[:3] if leds else leds}")

    def test_1_registers_its_methods(self):
        methods = None
        end = time.time() + 20
        while time.time() < end and methods is None:
            answer = self.rpc("rpc.methods")
            if answer and "light.get_profile" in answer.get("result", []):
                methods = answer["result"]
            time.sleep(0.2)
        self.assertIsNotNone(methods, "light.get_profile never registered")
        for method in ("light.set_profile", "light.preview", "light.reset"):
            self.assertIn(method, methods)
        profile = self.rpc("light.get_profile")["result"]
        self.assertIn("MOWING", profile["states"])
        self.assertEqual(profile["leds"], LEDS)
        self.assertEqual(profile["emergency"]["colors"], ["#ff0000"])

    def test_2_the_frame_shows_the_state_within_the_limits(self):
        answer = self.rpc("light.set_profile",
                          {"profile": {"MOWING": {"effect": "solid", "colors": ["#ffffff"]}}})
        self.assertIn("result", answer, answer)
        self.assertTrue(os.path.isfile(os.path.join(DATA_DIR, "light_profile.json")))
        leds = self.frame_with(lambda leds: leds[0] == (120, 120, 120), state=("MOWING",))
        self.assertEqual(len(leds), BUFFER_LEDS)
        self.assertEqual(leds[LEDS - 1], (120, 120, 120))
        self.assertEqual(leds[LEDS], (0, 0, 0))

    def test_3_the_emergency_goes_over_a_preview(self):
        answer = self.rpc("light.preview", {"effect": {"effect": "solid", "colors": ["#0000ff"]}, "duration": 3})
        self.assertIn("result", answer, answer)
        self.frame_with(lambda leds: leds[0] == (0, 0, 120), state=("MOWING",))
        self.frame_with(lambda leds: leds[0] == (120, 0, 0), state=("MOWING", True))
        self.assertIn("error", self.rpc("light.preview", {"effect": {"effect": "solid", "colors": ["#0000ff"]},
                                                         "duration": 11}))
        # Back to MOWING once the emergency and the preview are over.
        self.frame_with(lambda leds: leds[0] == (120, 120, 120), state=("MOWING",))

    def test_3a_a_chase_runs_alike_on_both_fronts(self):
        answer = self.rpc("light.preview", {"effect": {"effect": "chase", "colors": ["#00ff00"], "period": 2},
                                            "duration": 3})
        self.assertIn("result", answer, answer)
        self.frame_with(lambda leds: leds[:FRONT] == leds[FRONT:2 * FRONT] and leds[0] == (0, 120, 0)
                        and leds[FRONT - 1] == (0, 0, 0), state=("MOWING",))
        self.frame_with(lambda leds: leds[:FRONT] == leds[FRONT:2 * FRONT] and leds[0] == (0, 0, 0)
                        and (0, 120, 0) in leds[:FRONT], state=("MOWING",))

    def test_4_reset_and_a_dark_chain_at_the_end(self):
        answer = self.rpc("light.reset")
        self.assertEqual(answer["result"]["profile"]["MOWING"]["effect"], "solid")
        self.assertFalse(os.path.exists(os.path.join(DATA_DIR, "light_profile.json")))
        self.stop_node()
        with open(FRAME, "rb") as f:
            self.assertEqual(decode(f.read()), [(0, 0, 0)] * BUFFER_LEDS)
