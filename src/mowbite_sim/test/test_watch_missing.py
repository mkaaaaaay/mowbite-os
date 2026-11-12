"""A node of the container does not load (G39): the stack of sim.launch.py with a parameter of mowbite_light of the
wrong type, so its constructor throws and the container runs on without it. The watch of the container expects every
node of stack.py and tron.launch.py and ends the process 60 s after its start; the container starts again and, from
IDLE, UNEXPECTED_RESTART names the node with reason NODE_STALLED and holds nothing.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_watch_missing.py
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from mowbite_msgs.msg import MqttPublish
from rcl_interfaces.msg import Log

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import ROSOUT, sim_description, sim_files, wait_until  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_missing_")
DATA = sim_files(WORK)[3]
EXIT_NOTE = os.path.join(DATA, "container_exit.json")
CONFIG = os.path.join(WORK, "missing.yaml")
# mowbite_watch: the start limit and the exit code.
START_LIMIT = 60.0
EXIT_CODE = 86

with open(CONFIG, "w") as f:
    f.write("mowbite_light:\n  ros__parameters:\n    output: file\n    file: " + os.path.join(WORK, "light.bin") +
            "\n    leds: \"many\"\n")


def generate_test_description():
    return sim_description(WORK, 18835, extra_config=CONFIG)


class TestWatchMissing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.lock = threading.Lock()
        cls.events = []
        cls.logs = []
        cls.node = rclpy.create_node("missing_probe")
        cls.node.create_subscription(MqttPublish, "bridge/publish", cls._publish, 100)
        cls.node.create_subscription(Log, "/rosout", cls._log, ROSOUT)
        cls.running = True
        threading.Thread(target=cls._spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        time.sleep(0.3)
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def _spin(cls):
        while cls.running:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _publish(cls, msg):
        if msg.topic == "events/json":
            with cls.lock:
                cls.events.append(json.loads(msg.payload))

    @classmethod
    def _log(cls, msg):
        with cls.lock:
            cls.logs.append(msg.msg)

    def wait(self, getter, timeout):
        return wait_until(getter, timeout, self.lock, 0.1)

    def test_a_node_that_does_not_load_ends_the_container_after_the_start_limit(self, proc_output):
        proc_output.assertWaitFor("mowbite_watch: mowbite_light did not start within 60.0 s", timeout=START_LIMIT + 60,
                                  stream="stderr")
        event = self.wait(lambda: next((e for e in self.events if e["type"] == "UNEXPECTED_RESTART"), None), 60)
        self.assertTrue(event, [e["type"] for e in self.events])
        self.assertEqual((event["cause"], event["reason"], event["state_before"], event["hold"]),
                         ("OS_CRASH", "NODE_STALLED", "IDLE", False), event)
        self.assertEqual((event["severity"], event["resume"]), ("warning", "auto"), event)
        self.assertEqual((event["stalled"]["node"], event["stalled"]["limit"]), ("mowbite_light", START_LIMIT), event)
        with open(EXIT_NOTE) as f:
            note = json.load(f)
        self.assertEqual((note["exit_code"], note["restart"], note["stalled"]["node"]), (EXIT_CODE, 1, "mowbite_light"),
                         note)
        self.assertTrue(self.wait(lambda: any("ended by the watch: mowbite_light" in line for line in self.logs), 10))
