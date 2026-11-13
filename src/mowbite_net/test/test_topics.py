"""mowbite_net is the only node that talks to the internet. What net_node.cpp publishes and reads is this list, nothing
under hw/, safety/ or logic/ is written, and no log line names the token, the topic or an address of a phone.

Run: python3 test_topics.py
"""
import os
import re
import unittest

HERE = os.path.dirname(__file__)
NODE = os.path.join(HERE, "..", "src", "net_node.cpp")
OUT = {"bridge/rpc/response", "bridge/rpc/error", "bridge/rpc/methods", "net/weather"}
IN = {"bridge/rpc/request", "bridge/publish", "bridge/robot_state", "stats/parts", "schedule/notice",
      "net/weather/request"}


class Topics(unittest.TestCase):
    def setUp(self):
        with open(NODE) as f:
            self.text = f.read()

    def test_topics(self):
        made = re.findall(r'create_(publisher|client|service|subscription)<[^;]*?>\(\s*"([^"]+)"', self.text)
        self.assertEqual({name for kind, name in made if kind != "subscription"}, OUT)
        self.assertEqual({name for kind, name in made if kind == "subscription"}, IN)
        self.assertNotIn("create_client", self.text)
        self.assertNotIn("rclcpp_action", self.text)

    def test_no_secret_in_a_log_line(self):
        calls = re.findall(r"RCLCPP_[A-Z]+\((.*?)\);", self.text, re.S)
        self.assertGreater(len(calls), 3)
        for call in calls:
            for secret in ("token", ".topic.c_str", "address", "pushes[", "server"):
                self.assertNotIn(secret, call, call)


if __name__ == "__main__":
    unittest.main()
