"""H4: the schedule only asks the logic. What schedule_node.cpp publishes, calls and offers is this list; it subscribes
to nothing that drives, and nothing it names lies under hw/ or safety/, nor is cmd_vel or follow_path. On MQTT it
sends only mowbite/schedule/ready.

Run: python3 test_topics.py
"""
import os
import re
import unittest

HERE = os.path.dirname(__file__)
NODE = os.path.join(HERE, "..", "src", "schedule_node.cpp")
RPC = os.path.join(HERE, "..", "include", "mowbite_schedule", "schedule_rpc.h")
OUT = {"schedule/next_start", "net/weather/request", "schedule/notice", "bridge/rpc/methods", "bridge/rpc/response",
       "bridge/rpc/error", "bridge/publish", "logic/start_run", "logic/drop_tasks"}


class Topics(unittest.TestCase):
    def test_what_the_node_sends_and_calls(self):
        with open(NODE) as f:
            text = f.read()
        made = re.findall(r'create_(publisher|client|service|subscription)<[^;]*?>\(\s*"([^"]+)"', text)
        self.assertGreater(len(made), 10)
        out = {name for kind, name in made if kind in ("publisher", "client", "service")}
        self.assertEqual(out, OUT)
        self.assertNotIn("rclcpp_action", text)
        for kind, name in made:
            self.assertFalse(name.startswith(("hw/", "safety/")) and kind != "subscription", name)
            self.assertNotIn("cmd_vel", name)
            self.assertNotIn("follow_path", name)

    def test_what_the_node_sends_on_mqtt(self):
        with open(NODE) as f:
            text = f.read()
        self.assertEqual(re.findall(r"msg\.topic = ([^;]+);", text), ["kReadyTopic"])
        with open(RPC) as f:
            self.assertIn('kReadyTopic[] = "mowbite/schedule/ready";', f.read())


if __name__ == "__main__":
    unittest.main()
