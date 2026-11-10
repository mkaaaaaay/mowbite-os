"""Tests of the service of mowbite-host: its RPC, the state of the OS, the health probe, and over a private mosquitto
(when installed) the statuses, the last will and an answer to a response topic.

  python3 -I host/test/test_service.py
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from mowbite_host import files, installer, os_state, probe, service  # noqa: E402
from mowbite_host.broker import Message  # noqa: E402
import support  # noqa: E402
from test_updater import A, B, Base  # noqa: E402


class Published:
    def __init__(self):
        self.sent = []

    def __call__(self, topic, text, retain, correlation=None):
        self.sent.append((topic, json.loads(text), retain, correlation))
        return True

    def last(self, topic):
        for sent in reversed(self.sent):
            if sent[0] == topic:
                return sent
        return None


class HostBase(Base):
    def setUp(self):
        super().setUp()
        self.published = Published()
        self.watch = os_state.OsWatch(self.paths, self.fake, self.updater)
        self.host = service.Host(self.paths, self.updater, self.watch, self.fake, self.published)
        self.updater.on_change = self.host.update_changed

    def ask(self, method, params=None, call_id="1", response_topic=None, correlation=None, retained=False):
        request = {"jsonrpc": "2.0", "id": call_id, "method": method}
        if params is not None:
            request["params"] = params
        self.published.sent.clear()
        self.host.on_request(Message("rpc/request", json.dumps(request).encode(), retained, response_topic,
                                     correlation))
        while not self.host.requests.empty():
            self.host.handle(*self.host.requests.get())
        answers = [s for s in self.published.sent if s[0].startswith("rpc/response")]
        return answers

    def answer(self, method, params=None):
        answers = self.ask(method, params)
        self.assertEqual(len(answers), 1)
        return answers[0][1]

    def install(self, rev=A):
        self.bundle(rev)
        self.updater.check()
        self.assertEqual(self.run_update("os")["result"], "installed")


class RequestTest(HostBase):
    def test_only_its_methods_and_each_once(self):
        self.assertEqual(self.ask("rpc.methods"), [])
        self.assertEqual(self.ask("update.statusx"), [])
        self.assertEqual(len(self.ask("update.status")), 1)
        # Kept by the broker from earlier: never.
        self.assertEqual(self.ask("update.status", retained=True), [])

    def test_answer_goes_where_the_request_wants_it(self):
        topic, answer, retain, correlation = self.ask("update.status", response_topic="rpc/response/app-1",
                                                      correlation=b"c1")[0]
        self.assertEqual((topic, retain, correlation), ("rpc/response/app-1", False, b"c1"))
        self.assertEqual(answer["id"], "1")
        # The bridge refuses these: the host stays silent.
        self.assertEqual(self.ask("update.status", response_topic="robot_state/json"), [])
        self.assertEqual(self.ask("update.status", response_topic="rpc/response/app-1", correlation=b"x" * 1025), [])

    def test_without_id_only_to_a_response_topic(self):
        self.assertEqual(self.ask("update.status", call_id=None), [])
        self.assertEqual(len(self.ask("update.status", call_id=None, response_topic="rpc/response/a")), 1)

    def test_status_and_app_version(self):
        answer = self.answer("update.status", {"app_version": "1.5.0"})
        self.assertEqual(answer["result"]["phase"], "idle")
        self.assertEqual(answer["result"]["components"][0]["id"], "os")
        self.assertEqual(self.updater.state["app_version"], "1.5.0")

    def test_refusals_name_their_reason(self):
        error = self.answer("update.start", {"component": "os"})["error"]
        self.assertEqual((error["code"], error["data"]), (-32600, {"event": "UPDATE_REFUSED", "reason": "NO_NEWER",
                                                                    "message": error["message"]}))
        self.bundle(A)
        self.updater.check()
        error = self.answer("update.start", {"component": "os", "revision": "fffffff"})["error"]
        self.assertEqual(error["data"]["reason"], "UNKNOWN_REVISION")
        self.assertEqual(self.answer("update.abort")["error"]["data"]["reason"], "NOT_UPDATING")
        self.assertEqual(self.answer("update.rollback", {"component": "os"})["error"]["data"]["reason"],
                         "NO_PREVIOUS")
        for method, params in (("update.start", {"component": "planner"}), ("update.start", {}),
                               ("update.start", [1]), ("update.set_auto", {"from": "25:00"}),
                               ("update.set_auto", {"enabled": "yes"})):
            error = self.answer(method, params)["error"]
            self.assertEqual((error["code"], error["data"]["reason"]), (-32602, "INVALID"), (method, params))

    def test_start_check_and_auto(self):
        self.bundle(A)
        result = self.answer("update.check")["result"]
        self.assertEqual(result["components"][0]["available"]["revision"], A)
        result = self.answer("update.set_auto", {"enabled": True, "from": "01:00"})["result"]
        self.assertEqual(result["auto"], {"enabled": True, "from": "01:00", "to": "05:00"})
        self.answer("update.start", {"component": "os", "app_version": "1.4.0"})
        self.assertEqual(self.updater.join()["result"], "installed")
        status = self.published.last(service.UPDATE_STATUS)
        self.assertEqual((status[1]["phase"], status[2]), ("idle", True))
        self.assertEqual(status[1]["last"]["revision"], A)


class RestartTest(HostBase):
    def restart(self):
        return self.answer("host.restart_os")

    def test_stopped_always(self):
        self.install()
        self.fake.stop()
        self.fake.session(emergency=True)
        self.assertEqual(self.restart()["result"]["state"], "running")
        self.assertTrue(self.fake.running)

    def test_running_only_in_idle_without_emergency(self):
        self.install()
        self.assertEqual(self.restart()["result"]["state"], "running")
        self.fake.session(state="MOWING")
        self.assertEqual(self.restart()["error"]["data"]["reason"], "BUSY")
        self.fake.session(emergency=True)
        self.assertEqual(self.restart()["error"]["data"]["reason"], "EMERGENCY")
        self.fake.session()
        files.write_json(self.paths.restart_hold, {"cause": "OS_CRASH"})
        self.assertEqual(self.restart()["error"]["data"]["reason"], "EMERGENCY")

    def test_down_or_halted_always(self):
        self.install()
        self.fake.session(emergency=True)
        files.write_json(self.paths.exit_note, {"t": time.time(), "restart": 3, "down": True})
        self.assertEqual(self.watch.read()["state"], "down")
        self.assertIsNone(self.restart().get("error"))
        # The start by hand removes the note; the fake does not.
        os.remove(self.paths.exit_note)
        self.fake.session(emergency=True)
        self.write(self.paths.halt, "MOWBITE_OS_HALT reason=SCHEMA_NEWER data_schema=3 image_schema=2 Newer.\n")
        self.assertEqual(self.watch.read()["state"], "halted")
        self.assertIsNone(self.restart().get("error"))

    def test_never_during_an_update(self):
        self.install()
        self.fake.grant = False
        self.updater.gate_wait = 30
        self.bundle(B)
        self.updater.check()
        self.updater.start("os")
        time.sleep(0.3)
        self.assertEqual(self.restart()["error"]["data"]["reason"], "HOLD")
        self.updater.abort()
        self.updater.join()


class OsStateTest(HostBase):
    def read(self):
        return self.watch.read()

    def test_states(self):
        self.assertEqual(self.read()["state"], "stopped")
        self.install()
        os_now = self.read()
        self.assertEqual((os_now["state"], os_now["reason"], os_now["revision"], os_now["restarts"]),
                         ("running", None, A, 0))
        # Written before this start of the service: still starting.
        self.fake.started = int(time.time()) + 5
        self.assertEqual(self.read()["state"], "starting")
        self.fake.started = int(time.time())
        self.write(self.paths.halt, "MOWBITE_OS_HALT reason=DATA_NO_SPACE data_schema=3 image_schema=3 No room.\n")
        self.assertEqual((self.read()["state"], self.read()["reason"]), ("halted", "DATA_NO_SPACE"))
        os.remove(self.paths.halt)
        self.fake.show = lambda: ("failed", 1, None)
        self.assertEqual((self.read()["state"], self.read()["reason"]), ("down", "CRASH_LOOP"))
        self.fake.show = lambda: ("activating", 1, None)
        self.assertEqual(self.read()["state"], "starting")

    def test_since_changes_with_the_state(self):
        self.fake.show = lambda: ("inactive", 1000, None)
        self.assertEqual(self.read()["since"], 1000)
        self.install()
        self.fake.show = type(self.fake).show.__get__(self.fake)
        self.assertGreater(self.read()["since"], 1000)

    def test_crashes_restarts_and_stalled(self):
        self.install()
        now = time.time()
        stalled = {"node": "mowbite_bridge", "seconds": 10.2, "limit": 10.0}
        files.write_json(self.paths.exit_note, {"t": now, "restart": 2, "exit_code": -9, "stalled": stalled,
                                                "down": False, "stable_s": 600})
        os_now = self.read()
        self.assertEqual((os_now["state"], os_now["restarts"], os_now["last_crash"], os_now["stalled"]),
                         ("running", 2, int(now), stalled))
        # The same end counts once.
        self.read()
        entry = self.updater.status()["components"][0]
        self.assertEqual(entry["crashes_since_install"], 1)
        # After a stable run the count is 0 again, the last crash stays.
        files.write_json(self.paths.exit_note, {"t": now - 700, "restart": 2, "stable_s": 600})
        self.assertEqual(self.watch.read(now)["restarts"], 0)
        files.write_json(self.paths.exit_note, {"t": now + 1, "restart": 3, "down": True, "stalled": None})
        os_now = self.read()
        self.assertEqual((os_now["state"], os_now["reason"], os_now["restarts"], os_now["stalled"]),
                         ("down", "CRASH_LOOP", 3, None))
        self.assertEqual(self.updater.status()["components"][0]["crashes_since_install"], 2)

    def test_updating_while_it_switches(self):
        self.install()
        self.updater._phase["phase"] = "checking"
        self.assertEqual(self.read()["state"], "updating")

    def test_host_status_on_change_and_connect(self):
        self.host.look()
        self.host.look()
        sent = [s for s in self.published.sent if s[0] == service.HOST_STATUS]
        self.assertEqual(len(sent), 1)
        status = sent[0][1]
        self.assertEqual((status["schema"], status["online"], status["methods"], sent[0][2]),
                         (1, True, list(service.METHODS), True))
        self.assertEqual(set(status["os"]), {"state", "reason", "since", "restarts", "last_crash", "revision",
                                             "stalled"})
        self.published.sent.clear()
        self.host.connected()
        self.assertEqual([s[0] for s in self.published.sent], [service.HOST_STATUS, service.UPDATE_STATUS])


class FakeRpc:
    def __init__(self):
        self.answers = {"rpc.methods": ["system.hold", "rpc.methods"], "system.info": {"api": 14}}

    def call(self, method, params=None):
        return self.answers.get(method)


class ProbeTest(unittest.TestCase):
    def test_answers_and_bits(self):
        rpc = FakeRpc()
        check = probe.Probe(rpc, api_of_running=lambda: 14)
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        check.begin()
        self.assertFalse(check()[0])
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        self.assertEqual(check(), (True, "api 14, IDLE"))
        # A bit that stood before does not fail it; a new one does.
        self.assertEqual(check.settled()[0], True)
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": (1 << 1) | (1 << 12)}))
        self.assertEqual(check.settled(), (False, "emergency since the update: FIRMWARE_INCOMPATIBLE"))
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 2}))
        self.assertEqual(check.settled()[0], True)
        rpc.answers["system.info"] = {"api": 13}
        self.assertEqual(check(), (False, "system.info says api 13, the bundle 14"))
        rpc.answers["rpc.methods"] = ["rpc.methods"]
        self.assertFalse(check()[0])

    def test_health_takes_the_last_word_of_settled(self):
        class Settles:
            def __init__(self):
                self.begun = 0

            def begin(self):
                self.begun += 1

            def __call__(self):
                return True, "ok"

            def settled(self):
                return False, "emergency since the update: SERVICE_NOT_READY"

        class Running:
            def active(self):
                return True

        root = tempfile.mkdtemp()
        try:
            from mowbite_host import paths
            where = paths.Paths(root)
            checker = Settles()
            health = installer.Health(where, Running(), checker, wait=0, stable=0, every=0)
            health.once = lambda since: (True, "ok")
            health.begin()
            self.assertEqual(health.wait_healthy(0), (False, "emergency since the update: SERVICE_NOT_READY"))
            self.assertEqual(checker.begun, 1)
        finally:
            shutil.rmtree(root)


@unittest.skipUnless(shutil.which("mosquitto"), "no mosquitto")
class BrokerTest(unittest.TestCase):
    """mowbite-host against a private mosquitto: answers with correlation, statuses, last will."""

    def setUp(self):
        from mowbite_host import broker
        self.broker = broker
        self.dir = tempfile.mkdtemp()
        self.port = support.free_port()
        self.mosquitto = support.mosquitto(self.dir, self.port)

    def tearDown(self):
        self.mosquitto.terminate()
        self.mosquitto.wait()
        shutil.rmtree(self.dir)

    def client(self, name, subscriptions, will=None):
        got = []
        arrived = threading.Condition()

        def on_message(message):
            with arrived:
                got.append(message)
                arrived.notify_all()
        c = self.broker.Broker("127.0.0.1", self.port, name, subscriptions, on_message, will=will)
        c.start()
        deadline = time.time() + 5
        while not c.connected and time.time() < deadline:
            time.sleep(0.02)
        self.assertTrue(c.connected)
        return c, got, arrived

    def wait_for(self, got, arrived, test):
        with arrived:
            arrived.wait_for(lambda: any(test(m) for m in got), timeout=5)
        return [m for m in got if test(m)]

    def test_answer_status_and_last_will(self):
        os_side, os_got, os_arrived = self.client("os", [("rpc/request", 1)])
        host, host_got, host_arrived = self.client(
            "host", [("rpc/request", 1)], will=(service.HOST_STATUS, service.OFFLINE))
        rpc = probe.OsRpc(host)
        host._on_message = lambda m: rpc.on_answer(m.payload) if m.topic == host.answers else host_got.append(m)
        # A call of the host to the OS goes out with its own response topic and comes back.
        def answer_os(message):
            request = json.loads(message.payload)
            os_side.publish(message.response_topic, json.dumps({"jsonrpc": "2.0", "id": request["id"],
                                                                "result": ["system.hold"]}))
        os_side._on_message = answer_os
        self.assertEqual(rpc.call("rpc.methods"), ["system.hold"])
        # Retained status for the bridge and the app, then the last will when the host dies.
        host.publish(service.HOST_STATUS, json.dumps({"schema": 1, "online": True}), True)
        watcher, got, arrived = self.client("app", [(service.HOST_STATUS, 1)])
        self.assertTrue(self.wait_for(got, arrived, lambda m: json.loads(m.payload).get("online") is True))
        host.client._sock.close()
        host.client.loop_stop()
        offline = self.wait_for(got, arrived, lambda m: json.loads(m.payload) == {"schema": 1, "online": False})
        self.assertTrue(offline)
        watcher.stop()
        os_side.stop()

    def test_request_with_correlation_answered_by_the_host(self):
        app, got, arrived = self.client("app", [("rpc/response/app-1", 1)])
        try:
            from paho.mqtt.packettypes import PacketTypes
            from paho.mqtt.properties import Properties

            class Answers:
                def status(self):
                    return {"phase": "idle"}

                def remember_app(self, version):
                    pass

            host = None

            def route(message):
                host.on_request(message)
            client = self.broker.Broker("127.0.0.1", self.port, "host", [("rpc/request", 1)], route)
            host = service.Host(None, Answers(), None, None, client.publish)
            client.start()
            deadline = time.time() + 5
            while not client.connected and time.time() < deadline:
                time.sleep(0.02)
            props = Properties(PacketTypes.PUBLISH)
            props.ResponseTopic = "rpc/response/app-1"
            props.CorrelationData = b"\x01corr"
            app.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "id": 7, "method": "update.status"}),
                               qos=1, properties=props)
            host.handle(*host.requests.get(timeout=5))
            answer = self.wait_for(got, arrived, lambda m: m.topic == "rpc/response/app-1")[0]
            self.assertEqual(json.loads(answer.payload), {"jsonrpc": "2.0", "id": 7, "result": {"phase": "idle"}})
            self.assertEqual(answer.correlation, b"\x01corr")
            client.stop()
        finally:
            app.stop()


if __name__ == "__main__":
    unittest.main(verbosity=1)
