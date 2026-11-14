"""mowbite_bridge with MQTT 5 (AS4): answers to the response topic of their request, a listener on rpc/response sees
none of them; a client on 3.1.1 next to it as before; a broker of 3.1.1 makes the bridge fall back.

Three bridges on three private mosquittos: A straight on its broker, B behind mqtt311_gate in the mode connack, C behind
it in the mode close. The probe stands in for a node with the method test.echo.
Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_bridge_mqtt5.py
"""
import json
import os
import re
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import paho.mqtt.client as mqtt
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

from mowbite_msgs.msg import RpcMethods, RpcRequest, RpcResponse

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, SimProbe, mosquitto, mqtt_connect, sim_program, wait_until  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_mqtt5_")
BROKERS = {"a": 18851, "b": 18852, "c": 18853}
GATES = {"b": (18862, "connack"), "c": (18863, "close")}
WARN = "the bridge uses 3.1.1 until it restarts"
# The api version and break the bridge reports: kApi and kApiBreak of the source tree next to this test.
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "mowbite_api", "include", "mowbite_api",
                       "catalog.h")) as f:
    CATALOG = f.read()
API = int(re.search(r"constexpr int kApi = (\d+);", CATALOG).group(1))
API_BREAK = int(re.search(r"constexpr int kApiBreak = (\d+);", CATALOG).group(1))


def generate_test_description():
    actions = []
    for name, port in BROKERS.items():
        os.makedirs(os.path.join(WORK, name))
        actions.append(mosquitto(os.path.join(WORK, name), port))
    gate = sim_program("mqtt311_gate")
    for name, (port, mode) in GATES.items():
        actions.append(ExecuteProcess(cmd=[gate, "--port", str(port), "--upstream", str(BROKERS[name]), "--mode", mode],
                                      output="screen"))
    bridges = {
        "a": BROKERS["a"],
        "b": GATES["b"][0],
        "c": GATES["c"][0],
    }
    nodes = [Node(package="mowbite_bridge", executable="mowbite_bridge_node", namespace=name, output="screen",
                  parameters=[{"mqtt_host": "127.0.0.1", "mqtt_port": port, "software_version": "mqtt5-test"}])
             for name, port in bridges.items()]
    test = {"bridge_" + name: node for name, node in zip(bridges, nodes)}
    return launch.LaunchDescription(actions + [
        TimerAction(period=1.0, actions=nodes + [launch_testing.actions.ReadyToTest()]),
    ]), test


def client(name, port, version, topic):
    """A started client of MQTT 5 or 3.1.1 on topic that keeps what it gets: (topic, payload, correlation or None)."""
    got = []
    lock = threading.Lock()

    def on_message(_c, _u, m):
        props = getattr(m, "properties", None)
        correlation = getattr(props, "CorrelationData", None) if props else None
        with lock:
            got.append((m.topic, m.payload, correlation))
    c = mqtt_connect(name, port, on_message, topic, protocol=mqtt.MQTTv5 if version == 5 else mqtt.MQTTv311)
    c.got = got
    c.lock = lock
    return c


def ask(c, method, rid, response_topic=None, correlation=None, params=None):
    request = {"jsonrpc": "2.0", "method": method, "id": rid}
    if params is not None:
        request["params"] = params
    props = None
    if response_topic is not None:
        props = Properties(PacketTypes.PUBLISH)
        props.ResponseTopic = response_topic
        if correlation is not None:
            props.CorrelationData = correlation
    c.publish("rpc/request", json.dumps(request), qos=1, properties=props)


def answers(c, rid, topic=None):
    with c.lock:
        out = []
        for t, p, corr in c.got:
            if not t.startswith("rpc/response") or (topic and t != topic):
                continue
            try:
                j = json.loads(p)
            except ValueError:
                continue
            if j.get("id") == rid:
                out.append((t, j, corr))
        return out


def wait_for(pred, timeout=10.0):
    return wait_until(pred, timeout) or pred()


class EchoNode(SimProbe):
    """Stands in for a node of bridge A: test.echo answers with its params, the answers in the order release() says."""
    spin_timeout = 0.1

    def __init__(self):
        super().__init__("mqtt5_probe")
        self.requests = []
        self.methods = self.node.create_publisher(RpcMethods, "/a/bridge/rpc/methods", HISTORY)
        self.methods.publish(RpcMethods(node_id="mqtt5_probe", methods=["test.echo"]))
        self.response = self.node.create_publisher(RpcResponse, "/a/bridge/rpc/response", 100)
        self.node.create_subscription(RpcRequest, "/a/bridge/rpc/request", self._request, 100)
        self.start()

    def _request(self, m):
        with self.lock:
            self.requests.append(m)

    def answer(self, request):
        self.response.publish(RpcResponse(id=request.id, result=request.params or "null"))


class TestBridgeMqtt5(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.echo = EchoNode()
        cls.five = client("five", BROKERS["a"], 5, "rpc/response/five")
        cls.old = client("old", BROKERS["a"], 3, "rpc/response")
        # Listens to everything with 3.1.1, the way an app of before MQTT 5 does.
        cls.listener = client("listener", BROKERS["a"], 3, "#")
        time.sleep(0.5)
        # The bridge is there once it answers.

        def pinged():
            ask(cls.old, "rpc.ping", "up")
            time.sleep(0.3)
            return answers(cls.old, "up")
        assert wait_for(pinged, 20), "bridge A does not answer"
        assert wait_for(lambda: "test.echo" in json.dumps(cls.methods_of(cls.old)), 10), "test.echo not registered"

    @classmethod
    def methods_of(cls, c):
        rid = f"m{time.time()}"
        ask(c, "rpc.methods", rid)
        got = wait_for(lambda: answers(c, rid), 3)
        return got[0][1].get("result") if got else []

    @classmethod
    def tearDownClass(cls):
        for c in (cls.five, cls.old, cls.listener):
            c.loop_stop()
            c.disconnect()
        cls.echo.stop()
        rclpy.shutdown()

    def test_1_system_info_says_5(self):
        ask(self.five, "system.info", "info", "rpc/response/five", b"i")
        got = wait_for(lambda: answers(self.five, "info"))
        self.assertTrue(got, "no answer on rpc/response/five")
        self.assertEqual(got[0][1]["result"]["mqtt"], 5)
        self.assertEqual(got[0][1]["result"]["api"], API)
        self.assertEqual(got[0][1]["result"]["api_break"], API_BREAK)

    def test_1_system_stats_of_the_machine(self):
        self.assertIn("system.stats", self.methods_of(self.old))
        ask(self.old, "system.stats", "stats")
        got = wait_for(lambda: answers(self.old, "stats"))
        self.assertTrue(got, "no answer to system.stats")
        stats = got[0][1]["result"]
        self.assertGreater(stats["mem_total"], 0)
        self.assertLessEqual(stats["disk_free"], stats["disk_total"])
        self.assertGreaterEqual(stats["cpus"], 1)

    def test_2_answer_only_to_the_response_topic_with_correlation(self):
        correlation = b"\x00\x01corr\xff"
        ask(self.five, "rpc.ping", "same", "rpc/response/five", correlation)
        ask(self.old, "rpc.ping", "same")
        got = wait_for(lambda: answers(self.five, "same", "rpc/response/five"))
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][1]["result"], "pong")
        self.assertEqual(got[0][2], correlation)
        old = wait_for(lambda: answers(self.old, "same", "rpc/response"))
        self.assertEqual(len(old), 1)
        time.sleep(0.5)
        # The listener on rpc/response sees the answer of the 3.1.1 client only, not the one of the 5 client.
        self.assertEqual(len(answers(self.listener, "same", "rpc/response")), 1)
        self.assertEqual(len(answers(self.old, "same")), 1)

    def test_3_answers_of_a_node_find_their_client(self):
        start = len(self.echo.requests)
        ask(self.five, "test.echo", 7, "rpc/response/five", b"n5", params={"from": "five"})
        ask(self.old, "test.echo", 7, params={"from": "old"})
        self.assertTrue(wait_for(lambda: len(self.echo.requests) >= start + 2))
        with self.echo.lock:
            requests = self.echo.requests[start:start + 2]
        # Answered in the other order; the node never sees the ids of the clients.
        for r in reversed(requests):
            self.assertNotEqual(r.id, "7")
            self.echo.answer(r)
        five = wait_for(lambda: answers(self.five, 7, "rpc/response/five"))
        old = wait_for(lambda: answers(self.old, 7, "rpc/response"))
        self.assertEqual(len(five), 1)
        self.assertEqual(five[0][1]["result"], {"from": "five"})
        self.assertEqual(five[0][2], b"n5")
        self.assertEqual(len(old), 1)
        self.assertEqual(old[0][1]["result"], {"from": "old"})
        time.sleep(0.5)
        self.assertEqual([a[1]["result"] for a in answers(self.listener, 7, "rpc/response")], [{"from": "old"}])

    def test_4_an_invalid_response_topic_is_refused_and_nothing_done(self):
        start = len(self.echo.requests)
        for i, topic in enumerate(["map/json", "robot_state/json", "rpc/response/a/b", "rpc/response/+", "x"]):
            rid = f"bad{i}"
            ask(self.five, "test.echo", rid, topic, b"b", params={"n": i})
            got = wait_for(lambda: answers(self.listener, rid, "rpc/response"))
            self.assertEqual(len(got), 1, topic)
            error = got[0][1]["error"]
            self.assertEqual(error["code"], -32600)
            self.assertEqual(error["data"]["event"], "REQUEST_REFUSED")
            self.assertEqual(error["data"]["reason"], "RESPONSE_TOPIC")
            self.assertEqual(error["data"]["severity"], "error")
        time.sleep(1.0)
        self.assertEqual(len(self.echo.requests), start, "a refused request reached the node")
        with self.listener.lock:
            on_map = [t for t, _, _ in self.listener.got if t in ("map/json", "robot_state/json", "x")]
        self.assertEqual(on_map, [])

    def check_fallback(self, name):
        five = client("five_" + name, BROKERS[name], 5, "rpc/response/#")
        try:
            time.sleep(0.3)
            rid = "info_" + name

            def asked():
                ask(five, "system.info", rid, "rpc/response/five", b"f")
                time.sleep(0.5)
                return answers(five, rid)
            got = wait_for(asked, 40)
            self.assertTrue(got, f"bridge {name} does not answer")
            # On 3.1.1 the bridge cannot see the response topic: the answer goes to rpc/response, without correlation.
            self.assertEqual(got[0][0], "rpc/response")
            self.assertIsNone(got[0][2])
            self.assertEqual(got[0][1]["result"]["mqtt"], 3)
        finally:
            five.loop_stop()
            five.disconnect()

    def test_5_fallback_when_the_broker_refuses_5(self):
        self.check_fallback("b")

    def test_6_fallback_when_the_broker_drops_5(self):
        self.check_fallback("c")


@launch_testing.post_shutdown_test()
class TestWarnings(unittest.TestCase):

    def test_one_warning_per_fallback(self, proc_output, bridge_a, bridge_b, bridge_c):
        lines = {name: "".join(o.text.decode(errors="replace") for o in proc_output[proc])
                 for name, proc in (("a", bridge_a), ("b", bridge_b), ("c", bridge_c))}
        self.assertEqual(lines["a"].count(WARN), 0)
        self.assertEqual(lines["b"].count(WARN), 1)
        self.assertIn("refused MQTT 5", lines["b"])
        self.assertEqual(lines["c"].count(WARN), 1)
        self.assertIn("twice dropped the connect of MQTT 5", lines["c"])
