"""mowbite_net and mowbite_stats in the simulation (FP22 B4): the whole stack of sim.launch.py on a private mosquitto,
and one HTTP server on the loopback that plays the ntfy server, two phones (one no longer signed up) and open-meteo.
notify.* over MQTT as the app does, a push of a change to the targets of before (G13), a test push that drops the
phone gone, a push of an event, the weather for the schedule, stats.*; at the end the token is in no output of any
process and in no file but notify.json, which only the owner may read.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_net.py
"""
import json
import os
import stat
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import launch
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from mowbite_msgs.msg import MqttPublish, Weather, WeatherRequest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import mosquitto, mqtt_connect, mqtt_rpc, sim_files, sim_stack, store_answer, wait_until  # noqa: E402

MQTT_PORT = 18879
HTTP_PORT = 18891
BASE = f"http://127.0.0.1:{HTTP_PORT}"
TOKEN = "tk_sim_7f3a9c"
PHONE = BASE + "/up/phone"
GONE = BASE + "/up/gone"
WORK = tempfile.mkdtemp(prefix="mowbite_net_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "net.yaml")
WEATHER = {"current": {"precipitation": 0.4, "temperature_2m": 6.5},
           "minutely_15": {"precipitation": [0.1] * 12}}

REQUESTS = []
LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def _record(self, body):
        with LOCK:
            REQUESTS.append({"method": self.command, "path": self.path, "auth": self.headers.get("Authorization"),
                             "body": body})

    def do_GET(self):  # noqa: N802
        self._record(None)
        data = json.dumps(WEATHER).encode() if self.path.startswith("/v1/forecast") else b"{}"
        self.send_response(200 if self.path.startswith("/v1/forecast") else 404)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0"))
        self._record(json.loads(self.rfile.read(length) or b"null"))
        self.send_response(410 if self.path == "/up/gone" else 200)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def log_message(self, *args):
        pass


def generate_test_description():
    with open(os.path.join(get_package_share_directory("mowbite_sim"), "config", "sim.yaml")) as f:
        text = f.read()
    with open(CONFIG, "w") as f:
        f.write(text + f"\nmowbite_net:\n  ros__parameters:\n    weather_server: {BASE}\n")
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT,
                                                                                extra_config=CONFIG)]),
                                     launch_testing.actions.ReadyToTest()]), {}


def posts(since, path=None):
    with LOCK:
        return [r for r in REQUESTS[since:] if r["method"] == "POST" and (path is None or r["path"] == path)]


class TestNet(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        cls.answers, cls.lock = {}, threading.Lock()
        cls.client = mqtt_connect("test_net", MQTT_PORT,
                                  lambda c, u, m: store_answer(cls.answers, cls.lock, m.payload), "rpc/response")
        rclpy.init()
        cls.node = rclpy.create_node("test_net_probe")
        cls.weather = []
        cls.node.create_subscription(Weather, "net/weather", cls.weather.append, 10)
        cls.ask_weather = cls.node.create_publisher(WeatherRequest, "net/weather/request", 10)
        cls.publish = cls.node.create_publisher(MqttPublish, "bridge/publish", 100)
        cls.spin = threading.Thread(target=rclpy.spin, args=(cls.node,), daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.client.loop_stop()
        cls.client.disconnect()
        cls.server.shutdown()
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def rpc(self, method, params=None):
        answer = mqtt_rpc(self.client, self.answers, self.lock, method, params)
        self.assertIsNotNone(answer, method)
        self.assertNotIn(TOKEN, json.dumps(answer))
        return answer

    def result(self, method, params=None):
        answer = self.rpc(method, params)
        self.assertIn("result", answer, answer)
        return answer["result"]

    def test_net(self):
        def offered():
            answer = mqtt_rpc(self.client, self.answers, self.lock, "rpc.methods") or {}
            return {"notify.get", "stats.get"} <= set(answer.get("result", []))

        self.assertTrue(wait_until(offered, 60), "notify.* and stats.* in rpc.methods")
        first = self.result("notify.get")
        self.assertFalse(first["stored"])
        self.assertFalse(first["has_token"])

        # Saved for the first time: no push, there was no target before.
        mark = len(REQUESTS)
        got = self.result("notify.set", {"server": BASE, "topic": "sim_topic", "token": TOKEN, "lang": "en",
                                         "name": "Simi", "url": "http://mower.sim",
                                         "events": ["done", "emergency"]})
        self.assertTrue(got["has_token"])
        self.assertTrue(got["stored"])
        bad = self.rpc("notify.set", {"topic": "a/b"})
        self.assertEqual(bad["error"]["code"], -32602)
        self.assertEqual(bad["error"]["data"], {"field": "topic", "reason": "format"})

        # A phone added: the targets of before (the topic) hear of it, with the token.
        self.result("notify.push_add", {"address": PHONE})
        sent = wait_until(lambda: posts(mark, "/"), 20)
        self.assertTrue(sent, "push of the change to the topic")
        self.assertEqual(sent[0]["auth"], "Bearer " + TOKEN)
        self.assertEqual(sent[0]["body"]["topic"], "sim_topic")
        self.assertEqual(sent[0]["body"]["title"], "Notifications changed")
        self.assertEqual(posts(mark, "/up/phone"), [], "the new phone is not a target of before")
        mark = len(REQUESTS)
        self.result("notify.push_add", {"address": GONE})
        self.assertTrue(wait_until(lambda: posts(mark, "/up/phone"), 20), "the phone hears of the second")

        # The test push to all; the phone no longer signed up drops out.
        mark = len(REQUESTS)
        self.assertEqual(self.result("notify.test"), {"result": "sent"})
        phone = posts(mark, "/up/phone")
        self.assertEqual(len(phone), 1)
        self.assertEqual(phone[0]["body"]["kind"], "test")
        self.assertIsNone(phone[0]["auth"])
        self.assertNotIn(TOKEN, json.dumps(phone[0]["body"]))
        self.assertEqual(self.result("notify.get")["pushes"], [PHONE])
        log = self.result("notify.log", {"limit": 5})
        self.assertEqual(log[0]["kind"], "test")
        self.assertTrue(log[0]["sent"])

        # An event of the logic: done is picked.
        mark = len(REQUESTS)
        event = MqttPublish(topic="events/json", payload=json.dumps({"type": "JOB_COMPLETE", "t": time.time()}))
        self.publish.publish(event)
        done = wait_until(lambda: posts(mark, "/up/phone"), 20)
        self.assertTrue(done)
        self.assertEqual(done[0]["body"]["title"], "Done mowing")
        self.assertEqual(done[0]["body"]["api"], "http://mower.sim")

        # The weather for the schedule, twice: the second from the store.
        for i in (1, 2):
            self.ask_weather.publish(WeatherRequest(id=i, latitude=52.12, longitude=9.99, hours=2))
            self.assertTrue(wait_until(lambda: any(w.id == i for w in self.weather), 20), f"weather {i}")
        first, second = [next(w for w in self.weather if w.id == i) for i in (1, 2)]
        self.assertTrue(first.ok)
        self.assertTrue(first.has_rain)
        self.assertAlmostEqual(first.rain_hour_mm[1], 0.4)
        self.assertAlmostEqual(first.temp_c, 6.5)
        self.assertGreater(second.age_s, 0)
        with LOCK:
            self.assertEqual(sum(r["path"].startswith("/v1/forecast") for r in REQUESTS), 1)

        # stats.*
        stats = self.result("stats.get")
        self.assertEqual(stats["schema"], 1)
        self.assertEqual([p["key"] for p in stats["parts"]], ["blade", "clean"])
        due = self.result("stats.part_due", {"part": "blade", "hours": 5})
        self.assertEqual(due["parts"][0]["due_h"], 5)
        self.assertEqual(self.rpc("stats.part_done", {"part": "wheels"})["error"]["data"]["reason"], "unknown")

        # The files: the token only in notify.json, readable by the owner alone.
        path = os.path.join(DATA, "notify.json")
        self.assertTrue(wait_until(lambda: os.path.exists(path), 5))
        self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)
        for root, _, files in os.walk(DATA):
            for name in files:
                if name == "notify.json":
                    continue
                with open(os.path.join(root, name), "rb") as f:
                    self.assertNotIn(TOKEN.encode(), f.read(), name)


@launch_testing.post_shutdown_test()
class TestOutput(unittest.TestCase):
    def test_the_token_is_in_no_output(self, proc_output):
        for process in proc_output.processes():
            text = "".join(o.text.decode(errors="replace") for o in proc_output[process])
            self.assertNotIn(TOKEN, text, process)
            self.assertNotIn("/up/phone", text, process)
