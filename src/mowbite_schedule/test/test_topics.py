"""H4: the schedule only asks the logic. What schedule_node.cpp publishes, calls and offers is this list; it subscribes
to nothing that drives, and nothing it names lies under hw/ or safety/, nor is cmd_vel or follow_path.

Run: python3 test_topics.py
"""
import os
import re
import unittest

NODE = os.path.join(os.path.dirname(__file__), "..", "src", "schedule_node.cpp")
OUT = {"schedule/next_start", "net/weather/request", "bridge/rpc/methods", "bridge/rpc/response", "bridge/rpc/error",
       "logic/start_run", "logic/drop_tasks"}


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


if __name__ == "__main__":
    unittest.main()
