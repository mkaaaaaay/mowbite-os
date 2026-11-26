"""Tests of the service of mowbite-host: its RPC, the state of the OS, the health probe, and over a private mosquitto
(when installed) the statuses, the last will and an answer to a response topic.

  python3 -I host/test/test_service.py
"""
import hashlib
import hmac
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from io import StringIO

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "boot"))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from mowbite_host import files, gesture, installer, os_state, probe, service, updater, wifi  # noqa: E402
from mowbite_host.command import nmcli_fields  # noqa: E402
from mowbite_host.broker import Message  # noqa: E402
import support  # noqa: E402
from test_components import P1, X, ComponentBase  # noqa: E402
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

    def test_unreadable_requests_are_dropped(self):
        for payload in (b"{", b"\xff", b"[" * 100000):
            self.host.on_request(Message("rpc/request", payload, False))
        self.assertTrue(self.host.requests.empty())

    def test_without_id_only_to_a_response_topic(self):
        self.assertEqual(self.ask("update.status", call_id=None), [])
        self.assertEqual(len(self.ask("update.status", call_id=None, response_topic="rpc/response/a")), 1)

    def test_status_and_app_version(self):
        answer = self.answer("update.status", {"app_version": "1.5.0"})
        self.assertEqual(answer["result"]["phase"], "idle")
        self.assertEqual(answer["result"]["components"][0]["id"], "os")
        self.assertEqual(self.updater.state["app_version"], "1.5.0")

    def test_only_an_app_version_that_compares_is_kept(self):
        self.answer("update.status", {"app_version": "1.5.0-beta.2"})
        for bad in ("latest", "", "1." * 40 + "0", "1.5.0-" + "x" * 60):
            self.assertIn("result", self.answer("update.status", {"app_version": bad}))
            self.updater.remember_app(bad)
        self.assertEqual(self.updater.state["app_version"], "1.5.0-beta.2")

    def test_a_stored_app_version_that_is_no_version_is_dropped(self):
        os.makedirs(self.paths.var, exist_ok=True)
        for stored, kept in (("1.5.0", "1.5.0"), ("x" * 65, None), ("latest", None), (5, None)):
            files.write_json(self.paths.state, {"app_version": stored})
            again = updater.Updater(self.paths, self.fake, None, None, self.fake)
            self.assertEqual(again.state["app_version"], kept, stored)

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
        for method, params in (("update.start", {"component": "camera"}), ("update.start", {}),
                               ("update.start", [1]), ("update.set_auto", {"from": "25:00"}),
                               ("update.set_auto", {"enabled": "yes"}),
                               ("update.start", {"component": "os", "reset_jobs": True}),
                               ("update.rollback", {"component": "app", "reset_jobs": True}),
                               ("update.start", {"component": "planner", "reset_jobs": "yes"})):
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


class ComponentsTest(ComponentBase, HostBase):
    def test_four_components(self):
        components = self.answer("update.status")["result"]["components"]
        self.assertEqual([(c["id"], c["name"]) for c in components],
                         [("os", "MowBite OS"), ("planner", "MowBite Planner"), ("app", "MowBite App"),
                          ("host", "MowBite Host")])
        self.assertEqual([c.get("source") for c in components], [None, "bundled", "bundled", None])

    def test_reset_jobs_only_when_asked_for(self):
        self.installed("os", A)
        self.planner_bundle(P1)
        self.checkpoint(index=3)
        self.answer("update.check")
        self.assertIn("result", self.answer("update.start", {"component": "planner", "reset_jobs": False}))
        self.assertEqual(self.updater.join()["reason"], "JOB_OPEN")
        self.assertIn("result", self.answer("update.start", {"component": "planner", "reset_jobs": True}))
        self.assertEqual(self.updater.join()["result"], "installed")
        planner = self.published.last(service.UPDATE_STATUS)[1]["components"][1]
        self.assertEqual((planner["source"], planner["running"]["revision"]), ("own", P1))
        # The way back of the planner waits for the job as well.
        self.assertIn("result", self.answer("update.rollback", {"component": "planner"}))
        self.assertEqual(self.updater.join()["reason"], "JOB_OPEN")
        self.assertIn("result", self.answer("update.rollback", {"component": "planner", "reset_jobs": True}))
        self.assertEqual(self.updater.join()["result"], "installed")
        self.assertEqual(self.published.last(service.UPDATE_STATUS)[1]["components"][1]["source"], "bundled")

    def test_rollback_of_each_component(self):
        self.installed("os", A)
        self.installed("app", X)
        self.assertEqual(self.answer("update.rollback", {"component": "planner"})["error"]["data"]["reason"],
                         "NO_PREVIOUS")
        self.assertIn("result", self.answer("update.rollback", {"component": "app"}))
        last = self.updater.join()
        self.assertEqual((last["result"], last["component"], last["revision"]), ("installed", "app", None))
        self.assertEqual(self.published.last(service.UPDATE_STATUS)[1]["components"][2]["source"], "bundled")


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


class HardwareTest(HostBase):
    def conf(self):
        with open(self.paths.env_files[-1]) as f:
            return f.read()

    def test_on_writes_os_conf_and_restarts(self):
        self.install()
        os.makedirs(os.path.dirname(self.paths.env_files[-1]), exist_ok=True)
        self.write(self.paths.env_files[0], "MOWBITE_HARDWARE=false\n")
        self.write(self.paths.env_files[-1], "# kept\n#MOWBITE_HARDWARE=true\nMOWBITE_REAR=on\n")
        self.fake.stop()
        answer = self.answer("host.hardware", {"on": True})
        self.assertEqual(answer["result"]["state"], "running", answer)
        self.assertTrue(self.fake.running)
        self.assertEqual(self.conf(), "# kept\n#MOWBITE_HARDWARE=true\nMOWBITE_REAR=on\nMOWBITE_HARDWARE=true\n")
        self.assertTrue(self.published.last(service.HOST_STATUS)[1]["hardware"])
        # Off again: the line takes the new value, a second one of the key goes.
        self.write(self.paths.env_files[-1], "MOWBITE_HARDWARE=true\nMOWBITE_HARDWARE=read\n")
        self.assertIn("result", self.answer("host.hardware", {"on": False}))
        self.assertEqual(self.conf(), "MOWBITE_HARDWARE=false\n")
        self.assertFalse(self.published.last(service.HOST_STATUS)[1]["hardware"])

    def test_the_rule_of_the_restart_and_its_params(self):
        self.install()
        os.makedirs(os.path.dirname(self.paths.env_files[-1]), exist_ok=True)
        self.assertEqual(self.answer("host.hardware", {"on": "yes"})["error"]["data"]["reason"], "INVALID")
        self.assertEqual(self.answer("host.hardware")["error"]["data"]["reason"], "INVALID")
        self.fake.session(state="MOWING")
        self.assertEqual(self.answer("host.hardware", {"on": True})["error"]["data"]["reason"], "BUSY")
        # Refused before anything is written.
        self.assertFalse(os.path.exists(self.paths.env_files[-1]))
        # With the drivers on an emergency refuses as for host.restart_os.
        self.fake.session(emergency=True)
        self.assertEqual(self.answer("host.hardware", {"on": False})["error"]["data"]["reason"], "EMERGENCY")
        self.assertEqual(self.answer("host.hardware", {"on": True})["error"]["data"]["reason"], "EMERGENCY")

    def test_without_drivers_their_missing_inputs_do_not_hold_it(self):
        self.install()
        self.write(self.paths.env_files[0], "MOWBITE_HARDWARE=false\n")
        os.makedirs(os.path.dirname(self.paths.env_files[-1]), exist_ok=True)
        # The OS without drivers: IDLE with the emergency of the inputs that never came.
        self.fake.session(emergency=True)
        self.assertEqual(self.answer("host.hardware", {"on": False})["error"]["data"]["reason"], "EMERGENCY")
        self.assertEqual(self.answer("host.hardware", {"on": True})["result"]["state"], "running")
        self.assertEqual(self.conf(), "MOWBITE_HARDWARE=true\n")
        # Not while it does anything else, and not past the restart hold.
        self.write(self.paths.env_files[-1], "MOWBITE_HARDWARE=false\n")
        self.fake.session(state="RECORDING", emergency=True)
        self.assertEqual(self.answer("host.hardware", {"on": True})["error"]["data"]["reason"], "EMERGENCY")
        self.fake.session(emergency=True)
        files.write_json(self.paths.restart_hold, {"cause": "OS_CRASH"})
        self.assertEqual(self.answer("host.hardware", {"on": True})["error"]["data"]["reason"], "EMERGENCY")


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
        self.fake.show = type(self.fake).show.__get__(self.fake)
        self.install()
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
        for component, state in (("os", "updating"), ("planner", "updating"), ("app", "running")):
            self.updater._phase = {**self.updater._phase, "phase": "checking", "component": component}
            self.assertEqual(self.read()["state"], state)

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

    # A disk that takes no writes: the step is logged once and the crash still counts, the host goes on.
    def test_failing_step_does_not_end_the_host(self):
        calls = []

        def fail():
            calls.append(1)
            raise OSError(30, "Read-only file system")
        self.host._step("check", fail)
        self.host._step("check", fail)
        self.assertEqual(len(calls), 2)
        self.assertEqual(self.host._complaints, {"check": "OSError: [Errno 30] Read-only file system"})
        self.host._step("check", lambda: None)
        self.assertEqual(self.host._complaints, {})
        self.updater._save = fail
        self.updater.note_crash("os", 5)
        self.assertEqual(self.updater.last_crash("os"), 5)


class FakeRpc:
    def __init__(self):
        self.answers = {"rpc.methods": ["system.hold", "rpc.methods"], "system.info": {"api": 14}}

    def call(self, method, params=None):
        return self.answers.get(method)


class Commands:
    """command.run of the tests: handlers by the first words of argv, each (code, stdout, stderr) or a callable of
    argv; every call is kept. A program without a handler is not there."""

    def __init__(self):
        self.calls = []
        self.inputs = []
        self.handlers = {}

    def on(self, words, answer):
        self.handlers[tuple(words)] = answer

    def __call__(self, argv, timeout=30, input=None):  # noqa: A002 - as command.run
        self.calls.append(list(argv))
        self.inputs.append(input)
        for length in range(len(argv), 0, -1):
            answer = self.handlers.get(tuple(argv[:length]))
            if answer is not None:
                return answer(argv) if callable(answer) else answer
        raise FileNotFoundError(argv[0])

    def ran(self, *words):
        return [call for call in self.calls if call[:len(words)] == list(words)]


class CallsBase(HostBase):
    def setUp(self):
        super().setUp()
        self.commands = Commands()
        self.host = service.Host(self.paths, self.updater, self.watch, self.fake, self.published, self.commands)


class ConfirmStartTest(CallsBase):
    def setUp(self):
        super().setUp()
        self.install()
        self.write(self.paths.trial, "")
        self.commands.on([self.paths.trial_tool, "ssh"], (0, "", ""))
        self.commands.on([self.paths.trial_tool, "end"], lambda argv: (os.remove(self.paths.trial), (0, "", ""))[1])
        self.commands.on(["systemctl", "--failed"], (0, "", ""))
        self.commands.on(["nmcli"], (0, "ethernet:unavailable\nwifi:connected\nloopback:unmanaged\n", ""))

    def confirm(self):
        return self.answer("host.confirm_start")

    def test_without_a_trial_period_nothing_to_do(self):
        os.remove(self.paths.trial)
        self.assertEqual(self.confirm()["result"], {"trial": False})
        self.assertEqual(self.commands.calls, [])

    def test_a_good_start_ends_it(self):
        self.assertTrue(self.host.host_status()["trial"])
        self.assertEqual(self.confirm()["result"], {"trial": False})
        self.assertEqual(len(self.commands.ran(self.paths.trial_tool, "ssh")), 1)
        self.assertEqual(len(self.commands.ran(self.paths.trial_tool, "end")), 1)
        self.assertFalse(self.published.last(service.HOST_STATUS)[1]["trial"])

    # A bad start does not count as reaching the mower: asked again and again it still goes back on its own.
    def test_a_bad_start_is_not_counted(self):
        self.commands.on(["systemctl", "--failed"], (0, "mowbite-unpack.service loaded failed failed Pack\n"
                                                        "tron-hwinit.service loaded failed failed Tron\n", ""))
        self.commands.on(["nmcli"], (0, "wifi:disconnected\n", ""))
        self.fake.stop()
        for _ in range(3):
            error = self.confirm()["error"]["data"]
        self.assertEqual(error["reason"], "START_NOT_OK")
        for part in ("mowbite-unpack.service", "tron-hwinit.service", "no WLAN connection", "MowBite OS is stopped"):
            self.assertIn(part, error["message"])
        self.assertEqual(self.commands.ran(self.paths.trial_tool, "ssh"), [])
        self.assertEqual(self.commands.ran(self.paths.trial_tool, "end"), [])
        self.assertTrue(os.path.exists(self.paths.trial))

    # Only the units that put the pack in place count; another failed one is a note.
    def test_a_failed_camera_unit_is_a_note(self):
        self.commands.on(["systemctl", "--failed"], (0, "mowbite-tron-stereo-gain.service loaded failed failed S\n",
                                                        ""))
        self.assertEqual(self.confirm()["result"],
                         {"trial": False, "notes": ["failed: mowbite-tron-stereo-gain.service"]})
        self.assertEqual(len(self.commands.ran(self.paths.trial_tool, "ssh")), 1)

    # The installer proves it talks to the mower it wrote: HMAC of its challenge under the id of the pack.
    def test_proof_of_the_mower(self):
        challenge = "ab" * 16
        self.assertNotIn("proof", self.answer("host.confirm_start", {"challenge": challenge})["result"])
        self.write(self.paths.install_id, "00112233445566778899aabbccddeeff\n")
        result = self.answer("host.confirm_start", {"challenge": challenge})["result"]
        want = hmac.new(b"00112233445566778899aabbccddeeff", challenge.encode(), hashlib.sha256).hexdigest()
        self.assertEqual(result, {"trial": False, "proof": want})
        self.assertEqual(self.confirm()["result"], {"trial": False})
        for bad in ("ab", "x" * 32, 5):
            reason = self.answer("host.confirm_start", {"challenge": bad})["error"]["data"]["reason"]
            self.assertEqual(reason, "INVALID", bad)

    # While the first installation runs the period stands still, but the boot counts as not confirmed.
    def test_waits_for_the_first_installation(self):
        self.commands.on([self.paths.trial_tool, "hold"], (0, "", ""))
        self.updater.state["first"] = True
        self.assertEqual(self.confirm()["error"]["data"]["reason"], "BUSY")
        self.assertEqual(len(self.commands.ran(self.paths.trial_tool, "hold")), 1)
        self.assertEqual(self.commands.ran(self.paths.trial_tool, "ssh"), [])

    def test_a_rootfs_without_the_tool(self):
        del self.commands.handlers[(self.paths.trial_tool, "ssh")]
        self.assertEqual(self.confirm()["error"]["data"]["reason"], "INTERNAL")


class TrialHoldTest(CallsBase):
    def setUp(self):
        super().setUp()
        self.install()
        self.write(self.paths.trial, "")
        for verb in ("hold", "again"):
            self.commands.on([self.paths.trial_tool, verb], (0, "", ""))

    def ran(self, verb):
        return len(self.commands.ran(self.paths.trial_tool, verb))

    # Also without an installer at the PC that asks host.confirm_start.
    def test_held_while_the_first_installation_runs(self):
        self.updater.state["first"] = True
        self.host._hold_trial()
        self.host._hold_trial()
        self.assertEqual(self.ran("hold"), 1)
        self.host._trial_held -= service.HOLD_EVERY
        self.host._hold_trial()
        self.assertEqual((self.ran("hold"), self.ran("again")), (2, 0))

    def test_runs_again_when_the_first_installation_ends_without_an_os(self):
        self.updater.state["first"] = True
        self.host._hold_trial()
        self.fake.stop()
        self.updater.state["first"] = False
        self.host._hold_trial()
        self.host._hold_trial()
        self.assertEqual(self.ran("again"), 1)

    # The installer confirms a running OS; until then the tool lets the hold go by itself.
    def test_stays_held_when_the_os_runs(self):
        self.updater.state["first"] = True
        self.host._hold_trial()
        self.updater.state["first"] = False
        self.host._hold_trial()
        self.assertEqual((self.ran("hold"), self.ran("again")), (1, 0))

    def test_nothing_without_a_trial_period_or_a_first_installation(self):
        self.host._hold_trial()
        os.remove(self.paths.trial)
        self.updater.state["first"] = True
        self.host._hold_trial()
        self.assertEqual(self.commands.calls, [])

    # A rootfs whose tool knows no hold: one complaint, the next try a minute later.
    def test_an_old_tool(self):
        self.commands.on([self.paths.trial_tool, "hold"], (2, "", "usage"))
        self.updater.state["first"] = True
        out = StringIO()
        with redirect_stdout(out):
            for _ in range(3):
                self.host._step("trial", self.host._hold_trial)
        self.assertEqual(self.ran("hold"), 1)
        self.assertIn("the trial period was not held", out.getvalue())


class UsbModeTest(CallsBase):
    def setUp(self):
        super().setUp()
        self.install()
        self.write(self.paths.device_tree, "airseekers,tron\0rockchip,rk3588\0")
        self.write(self.paths.env_files[1], "MOWBITE_PLATFORM=tron\n")
        self.commands.on(list(service.LOADER_REBOOT), (0, "", ""))
        # Holds of the update before.
        self.fake.holds.clear()
        self.after = service.REBOOT_AFTER
        service.REBOOT_AFTER = 0.05

    def tearDown(self):
        service.REBOOT_AFTER = self.after
        super().tearDown()

    def usb(self):
        return self.answer("host.usb_mode")

    def rebooted(self):
        time.sleep(0.3)
        return self.commands.ran(*service.LOADER_REBOOT) != []

    def test_idle_with_the_hold_also_off_the_contacts(self):
        self.fake.session(charging=False)
        self.assertEqual(self.usb()["result"], {"rebooting": True})
        self.assertTrue(self.rebooted())
        self.assertTrue(os.path.exists(self.paths.hold))
        self.assertEqual(self.answer("update.status")["error"]["data"]["reason"], "BUSY")

    def test_a_stopped_os_needs_no_hold(self):
        self.fake.stop()
        self.assertEqual(self.usb()["result"], {"rebooting": True})
        self.assertTrue(self.rebooted())
        self.assertEqual(self.fake.holds, [])

    def test_refused_while_it_works_or_without_the_hold(self):
        for session, reason in (({"state": "MOWING"}, "BUSY"), ({"emergency": True}, "EMERGENCY")):
            self.fake.session(**session)
            self.assertEqual(self.usb()["error"]["data"]["reason"], reason)
        self.fake.session()
        self.fake.grant = False
        self.assertEqual(self.usb()["error"]["data"]["reason"], "NO_HOLD")
        self.assertFalse(self.rebooted())
        self.assertFalse(self.host.rebooting)

    # An update that waits at its gate (here for the charging contacts) gives way; the owner wants the USB mode.
    def test_an_update_at_its_gate_gives_way(self):
        self.fake.session(charging=False)
        self.updater.gate_wait = 30
        self.bundle(B)
        self.updater.check()
        self.updater.start("os")
        time.sleep(0.3)
        self.assertEqual(self.usb()["result"], {"rebooting": True})
        self.assertEqual(self.updater.state["last"]["result"], "aborted")
        self.assertTrue(self.rebooted())

    def test_not_during_a_switch(self):
        self.updater._working = True
        self.updater._phase = {**self.updater._phase, "phase": "switching", "component": "os"}
        try:
            self.assertEqual(self.usb()["error"]["data"]["reason"], "HOLD")
        finally:
            self.updater._working = False
        self.assertFalse(self.rebooted())

    # An abort counts for its run only: later the hold is asked for as always.
    def test_after_an_aborted_update(self):
        self.fake.grant = False
        self.updater.gate_wait = 30
        self.bundle(B)
        self.updater.check()
        self.updater.start("os")
        time.sleep(0.3)
        self.answer("update.abort")
        self.updater.join()
        # Granted a little later, as by the OS.
        self.fake.grant = True
        updater.HOLD_WAIT = 2.0
        request_hold = self.fake.request_hold
        self.fake.request_hold = lambda seconds: threading.Timer(0.2, request_hold, (seconds,)).start()
        self.assertEqual(self.usb()["result"], {"rebooting": True})

    def test_only_a_tron(self):
        self.write(self.paths.env_files[1], "MOWBITE_PLATFORM=nx60i\n")
        self.assertEqual(self.usb()["error"]["data"]["reason"], "UNSUPPORTED")
        self.write(self.paths.env_files[1], "MOWBITE_PLATFORM=tron\n")
        os.remove(self.paths.device_tree)
        self.assertEqual(self.usb()["error"]["data"]["reason"], "UNSUPPORTED")
        self.assertEqual(self.fake.holds, [])

    def test_a_failed_reboot_gives_the_hold_back(self):
        self.commands.on(list(service.LOADER_REBOOT), (1, "", "Access denied"))
        with redirect_stdout(StringIO()):
            self.assertEqual(self.usb()["result"], {"rebooting": True})
            self.assertTrue(self.rebooted())
        self.assertFalse(os.path.exists(self.paths.hold))
        self.assertFalse(self.host.rebooting)
        self.host.look()
        failed = self.published.last(service.HOST_STATUS)[1]["usb_mode_failed"]
        self.assertEqual(failed["error"], "Access denied")
        self.assertLessEqual(abs(failed["t"] - time.time()), 5)
        # A new try clears it.
        self.commands.on(list(service.LOADER_REBOOT), (0, "", ""))
        self.assertEqual(self.usb()["result"], {"rebooting": True})
        self.assertIsNone(self.host.host_status()["usb_mode_failed"])

    def test_no_failure_without_a_try(self):
        self.assertIsNone(self.host.host_status()["usb_mode_failed"])


KEY = "Geheim:Schl\\uessel 42"
SCAN = ("*:Heim\\:netz:70:WPA2\n:Heim\\:netz:40:WPA2\n:Gast:90:--\n:::\n:Nachbar:55:WPA1 WPA2\n"
        ":Gast:20:--\n")


class WifiTest(CallsBase):
    def setUp(self):
        super().setUp()
        os.makedirs(self.paths.nm_connections)
        self.old = os.path.join(self.paths.nm_connections, wifi.PREFIX + "0ld.nmconnection")
        self.foreign = os.path.join(self.paths.nm_connections, "hersteller.nmconnection")
        self.write(self.old, "[connection]\n")
        self.write(self.foreign, "[connection]\n")
        self.commands.on(["nmcli", "-t", "-f", "DEVICE,TYPE", "device"],
                         (0, "wlan0:wifi\nwlan1:wifi\nlo:loopback\n", ""))
        self.commands.on(["nmcli", "-t", "-f", "UUID,DEVICE"], (0, "1111-old:wlan0\n2222:lo\n", ""))
        self.commands.on(["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY"], (0, SCAN, ""))
        self.commands.on(["nmcli", "connection", "load"], (0, "", ""))
        self.commands.on(["nmcli", "connection", "reload"], (0, "", ""))
        self.commands.on(["nmcli", "--wait"], (0, "", ""))
        self.out = StringIO()

    def set(self, params):
        with redirect_stdout(self.out):
            answer = self.answer("host.wifi_set", params)
            self.host.wifi.join()
            self.host.look()
        return answer

    def profiles(self):
        return sorted(os.listdir(self.paths.nm_connections))

    # Nowhere but in the profile: answers, statuses, log lines, the command lines of nmcli.
    def assert_key_kept(self):
        seen = json.dumps([sent[1] for sent in self.published.sent]) + self.out.getvalue() + json.dumps(
            self.commands.calls) + json.dumps(self.host.host_status())
        self.assertNotIn(KEY, seen)
        self.assertNotIn("Geheim", seen)

    def test_scan(self):
        networks = self.answer("host.wifi_scan")["result"]["networks"]
        self.assertEqual(networks, [
            {"ssid": "Heim:netz", "signal": 70, "secure": True, "in_use": True},
            {"ssid": "Gast", "signal": 90, "secure": False, "in_use": False},
            {"ssid": "Nachbar", "signal": 55, "secure": True, "in_use": False}])

    def test_scan_right_after_a_scan(self):
        self.commands.on(["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY"],
                         lambda argv: (0, SCAN, "") if argv[-1] == "no" else (1, "", "Error: Scanning not allowed"))
        self.assertEqual(len(self.answer("host.wifi_scan")["result"]["networks"]), 3)

    def own(self):
        return [name for name in self.profiles() if name.startswith(wifi.PREFIX)]

    def test_a_network_that_connects_is_kept_once_reached(self):
        answer = self.set({"ssid": "Heim:netz", "password": KEY})
        self.assertEqual(answer["result"]["state"], "trying")
        status = self.published.last(service.HOST_STATUS)[1]["wifi"]
        self.assertEqual((status["state"], status["ssid"], status["reason"]), ("connected", "Heim:netz", None))
        self.assertEqual(len(self.own()), 2)
        self.answer("host.wifi_scan")
        self.host.wifi.look()
        own = self.own()
        self.assertEqual(len(own), 1)
        self.assertNotIn(os.path.basename(self.old), own)
        self.assertIn(os.path.basename(self.foreign), self.profiles())
        path = os.path.join(self.paths.nm_connections, own[0])
        self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
        with open(path) as f:
            text = f.read()
        self.assertIn("psk=Geheim:Schl\\\\uessel 42\n", text)
        self.assertIn("ssid=" + "".join(f"{b};" for b in b"Heim:netz") + "\n", text)
        up = self.commands.ran("nmcli", "--wait")
        self.assertEqual(up[0][3:6], ["connection", "up", "uuid"])
        self.assertEqual(up[0][-2:], ["ifname", "wlan0"])
        self.assert_key_kept()

    def test_a_network_that_does_not_connect_goes_back(self):
        self.commands.on(["nmcli", "--wait"], lambda argv: (
            (4, "", "Error: Connection activation failed: Secrets were required") if argv[6] != "1111-old"
            else (0, "", "")))
        self.set({"ssid": "Heim", "password": KEY})
        status = self.published.last(service.HOST_STATUS)[1]["wifi"]
        self.assertEqual((status["state"], status["reason"]), ("failed", "WIFI_FAILED"))
        self.assertIn("the connection before is back", status["message"])
        self.assertEqual(self.profiles(), sorted([os.path.basename(self.old), os.path.basename(self.foreign)]))
        self.assertEqual(self.commands.ran("nmcli", "--wait")[-1][3:7], ["connection", "up", "uuid", "1111-old"])
        self.assert_key_kept()

    # Connected, but nothing reaches the mower over it (a guest WLAN): back to the network before.
    def test_a_network_not_reached_goes_back(self):
        self.set({"ssid": "Gast"})
        new = [name for name in self.own() if name != os.path.basename(self.old)]
        self.host.wifi.look(now=time.monotonic() + wifi.REACH_WAIT - 1)
        self.assertEqual(len(self.own()), 2)
        with redirect_stdout(self.out):
            self.host.wifi.look(now=time.monotonic() + wifi.REACH_WAIT + 1)
            self.host.wifi.join()
        self.assertEqual(self.own(), [os.path.basename(self.old)])
        self.assertNotIn(new[0], self.profiles())
        self.assertEqual(self.commands.ran("nmcli", "--wait")[-1][3:7], ["connection", "up", "uuid", "1111-old"])
        status = self.host.wifi.status()
        self.assertEqual((status["state"], status["reason"]), ("failed", "WIFI_FAILED"))
        self.assertIn("nothing reached the mower over Gast", status["message"])
        self.assertEqual(self.answer("host.wifi_set", {"ssid": "Gast"})["result"]["state"], "trying")
        with redirect_stdout(self.out):
            self.host.wifi.join()

    # Without a connection before there is nothing to go back to.
    def test_without_a_connection_before(self):
        self.commands.on(["nmcli", "-t", "-f", "UUID,DEVICE"], (0, "2222:lo\n", ""))
        self.set({"ssid": "Gast"})
        self.assertEqual(len(self.own()), 1)
        self.assertNotIn(os.path.basename(self.old), self.own())

    def test_one_change_until_reached(self):
        self.set({"ssid": "Gast"})
        self.host.wifi.reached()
        self.assertEqual(self.set({"ssid": "Heim", "password": KEY})["result"]["state"], "trying")

    def test_hidden_and_the_name_servers(self):
        self.set({"ssid": "Versteckt", "hidden": True})
        new = [name for name in self.own() if name != os.path.basename(self.old)]
        with open(os.path.join(self.paths.nm_connections, new[0])) as f:
            text = f.read()
        self.assertIn("hidden=true\n", text)
        self.assertIn("[ipv6]\nmethod=auto\nignore-auto-dns=true\n", text)
        self.assertEqual(self.set({"ssid": "Gast", "hidden": "ja"})["error"]["data"]["reason"], "INVALID")

    def test_an_open_network(self):
        self.set({"ssid": "Gast"})
        new = [name for name in self.own() if name != os.path.basename(self.old)]
        with open(os.path.join(self.paths.nm_connections, new[0])) as f:
            self.assertNotIn("wifi-security", f.read())

    def test_invalid(self):
        for params in ({"ssid": ""}, {"ssid": "x" * 33}, {"ssid": "a\nb"}, {"ssid": "Heim", "password": "kurz"},
                       {"ssid": "Heim", "password": "Geheim\nzeile"}, {"ssid": "Heim", "password": 12345678}):
            self.assertEqual(self.set(params)["error"]["data"]["reason"], "INVALID", params)
        self.assertEqual(self.commands.calls, [])
        self.assert_key_kept()

    def test_one_change_at_a_time(self):
        gate = threading.Event()
        self.commands.on(["nmcli", "--wait"], lambda argv: (gate.wait(5), (0, "", ""))[1])
        self.assertEqual(self.answer("host.wifi_set", {"ssid": "Heim", "password": KEY})["result"]["state"], "trying")
        self.assertEqual(self.answer("host.wifi_set", {"ssid": "Gast"})["error"]["data"]["reason"], "BUSY")
        gate.set()
        with redirect_stdout(self.out):
            self.host.wifi.join()

    def test_without_network_manager(self):
        self.commands.handlers.clear()
        self.assertEqual(self.answer("host.wifi_scan")["error"]["data"]["reason"], "UNSUPPORTED")
        self.assertEqual(self.set({"ssid": "Heim", "password": KEY})["error"]["data"]["reason"], "UNSUPPORTED")

    def test_nmcli_fields(self):
        self.assertEqual(nmcli_fields("a\\:b:c\\\\:"), ["a:b", "c\\", ""])


# Key events as the input device gives them: (code, value), type EV_KEY.
def key_events(*keys):
    return b"".join(gesture.EVENT.pack(0, 0, gesture.EV_KEY, code, value) for code, value in keys)


PRESS = ((25, 1), (25, 0))
DEVICES = """I: Bus=0019 Vendor=0001 Product=0001 Version=0100
N: Name="gpio-keys"
H: Handlers=kbd event2

I: Bus=0019 Vendor=0001 Product=0001 Version=0100
N: Name="key input"
H: Handlers=kbd event5
"""


class GestureTest(CallsBase):
    def setUp(self):
        super().setUp()
        self.profile = os.path.join(self.paths.nm_connections, "mowbite-setup.nmconnection")
        self.write(self.paths.wifi_gesture, f"# key\ndevice=key input\ncode=25\ntone_device=plughw:CARD=x\n"
                                            f"unit=ap.service\nprofile={self.profile}\nbutton=on_ok\n")
        self.write(self.paths.input_devices, DEVICES)
        self.write(self.profile, "[connection]\n")
        self.commands.on(["nmcli", "-t", "-f", "TYPE,STATE", "device"],
                         (0, "wifi:disconnected\nwifi:unavailable\n", ""))
        self.commands.on(["systemctl", "--no-block", "restart", "ap.service"], (0, "", ""))
        self.commands.on(["aplay"], (0, "", ""))
        self.host = service.Host(self.paths, self.updater, self.watch, self.fake, self.published, self.commands)
        # As start() reads it, without the thread that reads the device.
        self.host.gesture.config = gesture.read_config(self.paths.wifi_gesture)
        self.out = StringIO()

    def press(self, events, clock=lambda: 0.0):
        os.makedirs(self.paths.dev_input, exist_ok=True)
        with open(os.path.join(self.paths.dev_input, "event5"), "wb") as f:
            f.write(events)
        self.host.gesture.read(self.counter, clock)

    def gesture(self):
        self.counter = gesture.Counter()
        self.press(key_events(*PRESS * gesture.PRESSES))
        with redirect_stdout(self.out):
            self.host.look()
        self.host.gesture.player.join(5)

    def test_counts_short_presses_of_its_key_within_the_window(self):
        self.counter = gesture.Counter()
        # Four presses, other keys and the code of a long press do not count.
        self.press(key_events(*PRESS * 4, (38, 1), (38, 0), (31, 1), (31, 0)))
        self.assertFalse(self.host.gesture.take())
        self.press(key_events(*PRESS))
        self.assertTrue(self.host.gesture.take())
        self.assertFalse(self.host.gesture.take())
        # Spread over more than the window: never five within it.
        times = iter([0.0, 1.5, 3.0, 4.5, 6.0, 7.5])
        self.counter = gesture.Counter()
        self.press(key_events(*PRESS * 6), clock=lambda: next(times))
        self.assertFalse(self.host.gesture.take())

    def test_opens_the_setup_wlan_with_the_rising_tone(self):
        before = time.time()
        self.gesture()
        until = int(files.read_text(self.paths.setup_ap_until))
        self.assertTrue(before + gesture.OPEN_S - 1 <= until <= time.time() + gesture.OPEN_S)
        self.assertEqual(self.commands.ran("systemctl"), [["systemctl", "--no-block", "restart", "ap.service"]])
        self.assertEqual(self.host.host_status()["setup_wifi"], {"until": until})
        self.assertEqual(self.published.last(service.HOST_STATUS)[1]["setup_wifi"], {"until": until})
        self.assertEqual(self.commands.ran("aplay"), [["aplay", "-q", "-D", "plughw:CARD=x", self.paths.tone_opened]])
        with open(self.paths.tone_opened, "rb") as f:
            self.assertEqual(f.read(4), b"RIFF")
        self.assertIn("setup WLAN opened by the buttons", self.out.getvalue())

        # Again while open, where the access point counts as a connected WLAN: the deadline moves, no restart.
        self.commands.on(["nmcli", "-t", "-f", "TYPE,STATE", "device"], (0, "wifi:connected\n", ""))
        self.write(self.paths.setup_ap_until, f"{until - 600}\n")
        self.gesture()
        # The second may have turned since the first gesture.
        self.assertLessEqual(abs(int(files.read_text(self.paths.setup_ap_until)) - until), 1)
        self.assertEqual(len(self.commands.ran("systemctl")), 1)
        self.assertEqual(self.commands.ran("aplay")[-1][-1], self.paths.tone_opened)

        # The unit removed the file when it ended (wlan0 connected): closed.
        os.remove(self.paths.setup_ap_until)
        self.host.look()
        self.assertIsNone(self.host.host_status()["setup_wifi"])
        self.assertFalse(os.path.exists(self.paths.setup_ap_until))

    def test_a_deadline_that_passed_closes_it(self):
        self.gesture()
        self.write(self.paths.setup_ap_until, f"{int(time.time()) - 1}\n")
        self.host.look()
        self.assertIsNone(self.host.host_status()["setup_wifi"])

    def refused(self, why, restarts=0):
        self.gesture()
        self.assertEqual(len(self.commands.ran("systemctl")), restarts)
        self.assertFalse(os.path.exists(self.paths.setup_ap_until))
        self.assertIsNone(self.host.host_status()["setup_wifi"])
        self.assertEqual(self.commands.ran("aplay"), [["aplay", "-q", "-D", "plughw:CARD=x", self.paths.tone_refused]])
        self.assertIn(f"setup WLAN not opened: {why}", self.out.getvalue())

    def test_an_os_that_runs_has_to_idle(self):
        self.install()
        self.fake.session(state="MOWING")
        self.refused("the OS is not at rest: MowBite OS is MOWING")

    def test_an_idle_os_lets_it_open(self):
        self.install()
        self.gesture()
        self.assertEqual(len(self.commands.ran("systemctl")), 1)

    def test_not_next_to_a_connected_wlan(self):
        self.commands.on(["nmcli", "-t", "-f", "TYPE,STATE", "device"], (0, "wifi:connected\n", ""))
        self.refused("the mower has a WLAN connection")

    def test_not_without_the_setup_profile(self):
        os.remove(self.profile)
        self.refused("no setup profile")

    def test_a_failed_restart_leaves_it_closed(self):
        self.commands.on(["systemctl", "--no-block", "restart", "ap.service"], (1, "", "Unit not found"))
        self.refused("systemctl: Unit not found", restarts=1)

    def test_without_the_file_of_the_platform_no_gesture(self):
        os.remove(self.paths.wifi_gesture)
        host = service.Host(self.paths, self.updater, self.watch, self.fake, self.published, self.commands)
        host.gesture.start()
        self.assertIsNone(host.gesture._thread)
        self.assertIsNone(host.host_status()["setup_wifi"])
        self.assertIsNone(host.host_status()["setup_wifi_gesture"])

    def test_the_app_learns_the_gesture(self):
        self.assertEqual(self.host.host_status()["setup_wifi_gesture"],
                         {"button": "on_ok", "presses": 5, "within_s": 5.0, "open_s": 1800})
        self.host.gesture.config.pop("button")
        self.assertIsNone(self.host.host_status()["setup_wifi_gesture"]["button"])

    def test_the_device_is_found_by_its_name(self):
        self.assertEqual(gesture.find_device(self.paths, "key input"), os.path.join(self.paths.dev_input, "event5"))
        self.assertIsNone(gesture.find_device(self.paths, "key"))
        self.assertEqual(gesture.read_config(self.paths.wifi_gesture),
                         {"device": "key input", "code": 25, "tone_device": "plughw:CARD=x", "unit": "ap.service",
                          "profile": self.profile, "button": "on_ok"})
        for broken in ("code=x", "unit="):
            self.write(self.paths.wifi_gesture, files.read_text(self.paths.wifi_gesture) + broken + "\n")
            self.assertIsNone(gesture.read_config(self.paths.wifi_gesture))


class SshTest(CallsBase):
    def setUp(self):
        super().setUp()
        self.write(self.paths.sshd_mowbite, "PermitRootLogin prohibit-password\n")
        self.write(self.paths.shadow, "root:*:20000:0:99999:7:::\nmkay:!:20000::::::\n")
        self.enabled = "disabled"
        self.commands.on(["systemctl", "is-enabled", "ssh.socket"],
                         lambda argv: (0 if self.enabled == "enabled" else 1, self.enabled + "\n", ""))
        self.commands.on(["systemctl", "enable"], lambda argv: (setattr(self, "enabled", "enabled"), (0, "", ""))[1])
        self.commands.on(["systemctl", "disable"], lambda argv: (setattr(self, "enabled", "disabled"), (0, "", ""))[1])
        self.commands.on(["chpasswd"], lambda argv: (self.write(self.paths.shadow, "root:$6$x$y:20000::::::\n"),
                                                     (0, "", ""))[1])

    def test_off_from_the_start_and_on_with_a_password(self):
        self.assertEqual(self.host.host_status()["ssh"], {"enabled": False, "password": False})
        answer = self.answer("host.ssh", {"enabled": True, "password": "garden-mower"})
        self.assertEqual(answer["result"], {"enabled": True, "password": True})
        self.assertEqual(self.commands.ran("systemctl", "enable"), [["systemctl", "enable", "--now", "ssh.socket"]])
        # The password only over stdin, never in argv.
        self.assertIn("root:garden-mower\n", self.commands.inputs)
        self.assertFalse(any("garden-mower" in " ".join(call) for call in self.commands.calls))
        self.assertEqual(self.published.last(service.HOST_STATUS)[1]["ssh"], {"enabled": True, "password": True})

    def test_off_and_on_again_keeps_the_password_and_a_new_one_needs_no_old(self):
        self.answer("host.ssh", {"enabled": True, "password": "garden-mower"})
        off = self.answer("host.ssh", {"enabled": False})["result"]
        self.assertEqual(off, {"enabled": False, "password": True})
        self.assertEqual(self.commands.ran("systemctl", "disable"),
                         [["systemctl", "disable", "--now", "ssh.socket", "ssh.service"]])
        self.assertEqual(self.answer("host.ssh", {"enabled": True})["result"], {"enabled": True, "password": True})
        self.assertIn("result", self.answer("host.ssh", {"enabled": True, "password": "another-one"}))
        self.assertEqual(len(self.commands.ran("chpasswd")), 2)

    def test_on_without_any_password_is_refused(self):
        error = self.answer("host.ssh", {"enabled": True})["error"]
        self.assertEqual(error["data"]["reason"], "NO_PASSWORD")
        self.assertEqual(self.commands.ran("systemctl", "enable"), [])

    def test_bad_params(self):
        for params in ({}, {"enabled": "yes"}, {"enabled": True, "password": "short"},
                       {"enabled": True, "password": "x" * 129}, {"enabled": True, "password": "line\nbreak!"},
                       {"enabled": False, "password": "garden-mower"}):
            error = self.answer("host.ssh", params)["error"]
            self.assertEqual((error["code"], error["data"]["reason"]), (service.INVALID_PARAMS, "INVALID"), params)
        self.assertEqual(self.commands.ran("chpasswd"), [])

    def test_a_failure_is_told_and_the_status_stays_true(self):
        self.commands.on(["systemctl", "enable"], (1, "", "Unit ssh.socket not found."))
        error = self.answer("host.ssh", {"enabled": True, "password": "garden-mower"})["error"]
        self.assertEqual(error["data"]["reason"], "FAILED")
        self.assertEqual(self.host.host_status()["ssh"], {"enabled": False, "password": True})

    def test_not_on_a_mower_without_the_sshd_config_of_mowbite(self):
        os.remove(self.paths.sshd_mowbite)
        self.assertIsNone(self.host.host_status()["ssh"])
        self.assertEqual(self.answer("host.ssh", {"enabled": True})["error"]["data"]["reason"], "UNSUPPORTED")


class ProbeTest(unittest.TestCase):
    def test_answers_and_bits(self):
        rpc = FakeRpc()
        check = probe.Probe(rpc, api_of_running=lambda: 14)
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        check.begin()
        time.sleep(0.01)
        since = time.time()
        # The robot_state of the old OS does not count for the new one.
        self.assertEqual(check(since), (False, "no robot_state from the new OS"))
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        self.assertEqual(check(since), (True, "api 14, IDLE"))
        # A bit that stood before does not fail it; a new one does.
        self.assertEqual(check.settled()[0], True)
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": (1 << 1) | (1 << 12)}))
        self.assertEqual(check.settled(), (False, "emergency since the update: FIRMWARE_INCOMPATIBLE"))
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 2}))
        self.assertEqual(check.settled()[0], True)
        rpc.answers["system.info"] = {"api": 13}
        self.assertEqual(check(since), (False, "system.info says api 13, the bundle 14"))
        rpc.answers["rpc.methods"] = ["rpc.methods"]
        self.assertFalse(check(since)[0])

    # A first installation with hardware off: no driver, TIMEOUT_INPUTS and FIRMWARE_INCOMPATIBLE stand from the start.
    def test_without_hardware_only_service_not_ready_fails(self):
        hardware = [False]
        check = probe.Probe(FakeRpc(), hardware=lambda: hardware[0])
        check.begin()
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": (1 << 1) | (1 << 12)}))
        self.assertEqual(check.settled(), (True, ""))
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": (1 << 1) | (1 << 8)}))
        self.assertEqual(check.settled(), (False, "emergency since the update: SERVICE_NOT_READY"))
        hardware[0] = True
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        self.assertEqual(check.settled(), (False, "emergency since the update: TIMEOUT_INPUTS"))

    # The probe of the host reads MOWBITE_HARDWARE as run-os does: the last file wins, only false is off.
    def test_the_host_probe_follows_the_environment_files(self):
        from mowbite_host import broker
        from mowbite_host.paths import Paths
        prefix = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, prefix)
        paths = Paths(prefix)
        for path in paths.env_files:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(paths.env_files[0], "w") as f:
            f.write("MOWBITE_HARDWARE=false\n")
        check = broker.attach(paths, "probe-test", [])[2]
        self.assertFalse(check.hardware())
        with open(paths.env_files[1], "w") as f:
            f.write("MOWBITE_HARDWARE=true\n")
        self.assertTrue(check.hardware())
        os.remove(paths.env_files[0])
        os.remove(paths.env_files[1])
        self.assertTrue(check.hardware())

    def test_unreadable_payloads_are_dropped(self):
        check = probe.Probe(FakeRpc())
        check.on_robot_state(json.dumps({"current_state": "IDLE", "emergency_reason": 1 << 1}))
        check.on_robot_state(b"[" * 100000)
        self.assertEqual(check.bits(), 1 << 1)
        probe.OsRpc(None).on_answer(b"[" * 100000)

    def test_health_takes_the_last_word_of_settled(self):
        class Settles:
            def __init__(self):
                self.begun = 0

            def begin(self):
                self.begun += 1

            def __call__(self, since):
                return True, "ok"

            def settled(self):
                return False, "emergency since the update: SERVICE_NOT_READY"

        class Running:
            def active(self):
                return True

            def show(self):
                return "active", None, 1

        root = tempfile.mkdtemp()
        try:
            from mowbite_host import paths
            where = paths.Paths(root)
            checker = Settles()
            health = installer.Health(where, Running(), checker, wait=0, stable=0, every=0)
            health.once = lambda since, entered: (True, "ok", 1)
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

    def test_an_error_in_a_handler_does_not_end_the_loop(self):
        host, got, arrived = self.client("host", [("rpc/request", 1)])
        on_message = host._on_message

        def fails_once(message):
            if message.payload == b"first":
                raise RuntimeError("handler failed")
            on_message(message)
        host._on_message = fails_once
        sender, _, _ = self.client("app", [])
        sender.publish("rpc/request", b"first")
        sender.publish("rpc/request", b"second")
        self.assertTrue(self.wait_for(got, arrived, lambda m: m.payload == b"second"))
        sender.stop()
        host.stop()

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
