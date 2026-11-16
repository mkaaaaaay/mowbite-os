"""End to end: mowbite-host as a process under a prefix with the carrier process, a private mosquitto, signed bundles
whose OS is fake_os.py, and the app side on the broker. A -> B -> broken C -> back; kill -9 of the host in every phase
of an update of the OS and of its planner, then its next start recovers; a foreign origin, an old serial, swapped
files, a closed gate; an app installed while the OS mows.

  python3 -I host/test/test_e2e.py   (needs mosquitto, ssh-keygen and paho-mqtt)
"""
import fcntl
import io
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.realpath(__file__))
LIB = os.path.join(HERE, "..")
BOOT = os.path.join(LIB, "boot")
sys.path.insert(0, LIB)
sys.path.insert(0, BOOT)
sys.path.insert(0, HERE)
import support  # noqa: E402
REPO = os.path.join(HERE, "..", "..")
SBIN = os.path.join(REPO, "platform", "tron", "rootfs", "files", "usr", "sbin")
try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.packettypes import PacketTypes
    from paho.mqtt.properties import Properties

    from mowbite_host import broker
except ImportError:
    mqtt = None

A, B, C = "a000001", "b000002", "c000003"
PLANNER, APP = "opt/mowbite-planner", "opt/mowbite-app"
WAIT = 60.0
# dpkg-query: every package is installed; it takes 2 s while state/slow-verify is there.
DPKG_QUERY = """#!/bin/sh
[ -e "{P}/state/slow-verify" ] && sleep 2
printf 'install ok installed'
"""
# os-setup of the rootfs: sourced for the library check during the unpack; 2 s while state/slow-unpack is there.
OS_SETUP = """[ -e "{P}/state/slow-unpack" ] && sleep 2
true
"""
RUN_OS = """#!/bin/sh
exec python3 -I "$(readlink -f "{P}/opt/mowbite/current")/fake_os.py"
"""


def until(test, wait=WAIT, what="condition"):
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        value = test()
        if value:
            return value
        time.sleep(0.1)
    raise AssertionError(f"timed out waiting for {what}")


class App:
    """The app on the broker: the retained statuses and its calls with a response topic and correlation."""

    def __init__(self, port):
        self.statuses = {}
        self.updates = []
        # The states of the OS the host reported, in their order.
        self.os_states = []
        self.answers = {}
        self.cond = threading.Condition()
        self.client = broker._client("app-e2e")
        self.client.on_connect = lambda c, *_: c.subscribe(
            [("mowbite/host/status", 1), ("mowbite/update/status", 1), ("rpc/response/app-e2e", 1)])
        self.client.on_message = self._message
        self.client.connect("127.0.0.1", port)
        self.client.loop_start()
        self.next_id = 0

    def _message(self, client, userdata, m):
        payload = json.loads(m.payload) if m.payload else None
        with self.cond:
            if m.topic == "rpc/response/app-e2e":
                self.answers[payload.get("id")] = payload
            else:
                self.statuses[m.topic] = payload
                if m.topic == "mowbite/update/status" and payload:
                    self.updates.append(payload)
                elif m.topic == "mowbite/host/status" and payload and payload.get("os"):
                    self.os_states.append(payload["os"].get("state"))
            self.cond.notify_all()

    def call(self, method, **params):
        self.next_id += 1
        call_id = self.next_id
        props = Properties(PacketTypes.PUBLISH)
        props.ResponseTopic = "rpc/response/app-e2e"
        props.CorrelationData = b"e2e"
        self.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "id": call_id, "method": method,
                                                       "params": params}), qos=1, properties=props)
        with self.cond:
            self.cond.wait_for(lambda: call_id in self.answers, timeout=WAIT)
            return self.answers.pop(call_id)

    def wait(self, test, what):
        with self.cond:
            if not self.cond.wait_for(test, timeout=WAIT):
                raise AssertionError(f"timed out waiting for {what}")

    def update(self):
        return self.statuses.get("mowbite/update/status") or {}

    def os(self):
        return (self.statuses.get("mowbite/host/status") or {}).get("os") or {}

    def stop(self):
        self.client.disconnect()
        self.client.loop_stop()


@unittest.skipUnless(mqtt and shutil.which("mosquitto") and shutil.which("ssh-keygen"),
                     "needs mosquitto, ssh-keygen and paho-mqtt")
class EndToEnd(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-e2e-")
        P = self.P = os.path.join(self.root, "fs")
        for folder in ("opt/mowbite", "userdata/mowbite-update/inbox", "userdata/mowbite", "etc/mowbite",
                       "usr/lib/mowbite", "proc/sys/kernel/random", "state", "run"):
            os.makedirs(os.path.join(P, folder))
        self.port = support.free_port()
        self.write("proc/sys/kernel/random/boot_id", "boot-1\n")
        self.write("usr/lib/mowbite/base", "1\n")
        self.write("usr/lib/mowbite/os-setup", OS_SETUP.format(P=P))
        self.write("usr/lib/mowbite/run-os", RUN_OS.format(P=P), mode=0o755)
        self.write("etc/mowbite/os.conf",
                   f"MOWBITE_HOST_CARRIER=process\nMOWBITE_MQTT_PORT={self.port}\nMOWBITE_UPDATE_NET=off\n"
                   "MOWBITE_PLATFORM=tron\n")
        self.bin = os.path.join(self.root, "bin")
        os.makedirs(self.bin)
        with open(os.path.join(self.bin, "dpkg-query"), "w") as f:
            f.write(DPKG_QUERY.format(P=P))
        os.chmod(os.path.join(self.bin, "dpkg-query"), 0o755)
        self.key = support.keygen(os.path.join(self.root, "key"))
        self.foreign = support.keygen(os.path.join(self.root, "foreign"))
        self.write("etc/mowbite/allowed_signers", 'mowbite namespaces="mowbite-os,mowbite-planner,mowbite-app,'
                                                  f'mowbite-host,mowbite-index" {support.public(self.key)}\n')
        self.mosquitto = support.mosquitto(self.root, self.port)
        self.app = App(self.port)
        self.host = None
        self.starts = 0
        self.serial = 1_700_000_000

    def tearDown(self):
        self.kill_host(gone=False)
        if os.path.exists(self.pid_file()):
            with open(self.pid_file()) as f:
                pid = int(f.read().split()[0])
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for pid in self.fake_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.app.stop()
        self.mosquitto.terminate()
        self.mosquitto.wait()
        if os.environ.get("E2E_KEEP"):
            print(f"kept {self.root}")
        else:
            shutil.rmtree(self.root, ignore_errors=True)

    def write(self, rel, text, mode=0o644):
        path = os.path.join(self.P, rel)
        with open(path, "w") as f:
            f.write(text)
        os.chmod(path, mode)

    def sign(self, path, namespace="mowbite-os", key=None):
        support.sign(path, key or self.key, namespace)

    # A signed bundle of rev as make-bundle.sh lays it out, with fake_os.py as its OS and an ELF file for the check
    # of the libraries; in the inbox unless folder is given. Of the planner (broken: it does not answer) or the app:
    # what those bring.
    def bundle(self, rev, serial=None, broken=False, key=None, folder=None, name=None, component="os"):
        if serial is None:
            self.serial += 1
            serial = self.serial
        meta = {"format": 2, "component": component, "revision": rev, "commit": rev * 5 + "abcde", "serial": serial,
                "date": serial, "runtime_packages": ["fake-runtime"]}
        meta.update({"os": {"base": 1, "platforms": ["tron"], "api": 14, "min_app": "1.4.0", "planner_interface": 1,
                            "planner_files_version": 1},
                     "planner": {"version": "2.0.0", "interface": 1, "files_version": 1},
                     "app": {"version": "1.5.0", "api": {"min": 14, "max": 14}},
                     "host": {"base": 1}}[component])
        path = os.path.join(folder or os.path.join(self.P, "userdata/mowbite-update/inbox"),
                            name or f"mowbite-{component}-{rev}.tar.gz")
        with tarfile.open(path, "w:gz") as tar:
            def add(member, data, mode=0o644):
                info = tarfile.TarInfo(f"{rev}/{member}")
                info.size, info.mode = len(data), mode
                tar.addfile(info, io.BytesIO(data))
            add("bundle.json", json.dumps(meta).encode())
            add("revision", rev.encode())
            if component == "os":
                with open(os.path.join(HERE, "fake_os.py"), "rb") as f:
                    add("fake_os.py", f.read())
                with open(shutil.which("true"), "rb") as f:
                    add("bin/true", f.read(), 0o755)
                if broken:
                    add("broken", b"")
            elif component == "host":
                for name in sorted(os.listdir(os.path.join(LIB, "mowbite_host"))):
                    if name.endswith(".py"):
                        with open(os.path.join(LIB, "mowbite_host", name), "rb") as f:
                            data = f.read()
                        if broken and name == "main.py":
                            data += b"\nraise SystemExit('broken host')\n"
                        add(f"mowbite_host/{name}", data)
            elif component == "planner":
                add("planner/local_setup.bash", b"")
                if broken:
                    add("planner/silent", b"")
            else:
                add("app/index.html", b"<html></html>")
                add("app/version.json", json.dumps({"version": meta["version"], "level": 1}).encode())
        self.sign(path, namespace=f"mowbite-{component}", key=key)
        return path

    def pid_file(self):
        return os.path.join(self.P, "run/mowbite-host/os.pid")

    # seed: the library mowbite_boot chooses under the prefix (the seed or an installation) instead of the repo's;
    # wait: for its status online.
    def start_host(self, seed=False, wait=True, notify=None):
        self.starts += 1
        t = int(time.time())
        env = {**os.environ, "MOWBITE_UPDATE_PREFIX": self.P, "MOWBITE_BOOT_LIB": BOOT, "MOWBITE_HOST_LIB": LIB,
               "MOWBITE_UPDATE_HEALTH_STABLE": "1", "MOWBITE_HOST_SETTLE": "1",
               "PATH": self.bin + os.pathsep + os.environ.get("PATH", "")}
        if seed:
            del env["MOWBITE_HOST_LIB"]
        if notify:
            env["NOTIFY_SOCKET"] = notify
        log = open(os.path.join(self.root, f"host-{self.starts}.log"), "w")
        self.host = subprocess.Popen([sys.executable, "-I", os.path.join(SBIN, "mowbite-host")], env=env,
                                     stdout=log, stderr=subprocess.STDOUT)
        log.close()
        if wait:
            self.app.wait(lambda: (self.app.statuses.get("mowbite/host/status") or {}).get("online") is True and
                          self.app.statuses["mowbite/host/status"].get("t", 0) >= t, "host status")

    # gone: waits for the last will of the killed host, so that a status seen after it is one of the next host and
    # not the last of this one.
    def kill_host(self, gone=True):
        if self.host is not None and self.host.poll() is None:
            self.host.send_signal(signal.SIGKILL)
            self.host.wait()
            if gone:
                self.app.wait(lambda: (self.app.statuses.get("mowbite/host/status") or {}).get("online") is False,
                              "last will of the host")
        self.host = None

    # mowbite-update by hand under the prefix.
    def by_hand(self, *args):
        return subprocess.run([sys.executable, "-I", os.path.join(SBIN, "mowbite-update"), *args],
                              env={**os.environ, "MOWBITE_UPDATE_PREFIX": self.P, "MOWBITE_HOST_LIB": LIB,
                                   "MOWBITE_BOOT_LIB": BOOT,
                                   "PATH": self.bin + os.pathsep + os.environ.get("PATH", "")},
                              capture_output=True, text=True, timeout=WAIT)

    def fake_pids(self):
        pids = []
        for pid in filter(str.isdigit, os.listdir("/proc")):
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmd = f.read().split(b"\0")
            except OSError:
                continue
            if any(part.startswith(self.P.encode()) and part.endswith(b"/fake_os.py") for part in cmd):
                pids.append(int(pid))
        return pids

    # The revisions whose fake OS runs.
    def running(self):
        revisions = set()
        for pid in self.fake_pids():
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = [part for part in f.read().split(b"\0") if part.endswith(b"/fake_os.py")][0]
            revisions.add(os.path.basename(os.path.dirname(cmd.decode())))
        return revisions

    def current(self, opt="opt/mowbite", name="current"):
        try:
            return os.readlink(os.path.join(self.P, opt, name))
        except OSError:
            return None

    # running.json of the fake OS that runs.
    def fake(self):
        with open(os.path.join(self.P, "state/running.json")) as f:
            return json.load(f)

    def component(self, name):
        return next(c for c in self.app.update()["components"] if c["id"] == name)

    def state(self):
        with open(os.path.join(self.P, "var/lib/mowbite-host/update.json")) as f:
            return json.load(f)

    def check(self):
        answer = self.app.call("update.check")
        self.assertIn("result", answer, answer)
        return answer["result"]

    # update.start (or another call) and the last result once the update ended.
    def run_update(self, method="update.start", component="os", **params):
        before = self.app.update().get("last")
        answer = self.app.call(method, component=component, **params)
        self.assertIn("result", answer, answer)
        self.app.wait(lambda: self.app.update().get("phase") == "idle" and self.app.update().get("last") != before,
                      "end of the update")
        return self.app.update()["last"]

    def wait_os(self, revision):
        self.app.wait(lambda: self.app.os().get("state") == "running" and self.app.os().get("revision") == revision,
                      f"OS {revision} running")
        until(lambda: self.running() == {revision}, what=f"only {revision} runs")

    def install(self, rev, component="os", **kwargs):
        self.bundle(rev, component=component, **kwargs)
        self.check()
        return self.run_update(component=component, revision=rev)

    # The host is killed, the marker of the phase goes, the next host starts and takes the cut update back: its
    # result once it is the one of rev, None for a phase before the switch.
    def cut_and_restart(self, marker, rev, switched):
        self.kill_host()
        os.remove(os.path.join(self.P, "state", marker))
        self.start_host()
        self.wait_os(A)
        if not switched:
            return None
        self.app.wait(lambda: (self.app.update().get("last") or {}).get("revision") == rev, "result of the cut update")
        return self.app.update()["last"]

    # Starts the update of rev and waits until it is in phase, held there by marker.
    def start_into(self, phase, marker, rev, component="os"):
        with open(os.path.join(self.P, "state", marker), "w") as f:
            f.write("MOWING" if marker == "os-state" else "")
        self.bundle(rev, component=component)
        self.check()
        self.app.call("update.start", component=component, revision=rev)
        self.app.wait(lambda: self.app.update().get("phase") == phase and
                      self.app.update().get("revision") == rev, phase)
        if phase == "waiting_gate":
            self.app.wait(lambda: self.app.update().get("reason") == "BUSY", "gate closed BUSY")

    def test_a_b_broken_c_and_back(self):
        self.start_host()
        last = self.install(A)
        self.assertEqual((last["result"], last["revision"]), ("installed", A), last)
        self.wait_os(A)
        last = self.install(B)
        self.assertEqual((last["result"], last["revision"]), ("installed", B), last)
        self.wait_os(B)
        component = self.app.update()["components"][0]
        self.assertEqual((component["running"]["revision"], component["previous"]["revision"]), (B, A))
        # A hold was asked for and given back.
        self.assertFalse(os.path.exists(os.path.join(self.P, "userdata/mowbite/hold.json")))

        last = self.install(C, broken=True)
        self.assertEqual((last["result"], last["reason"], last["revision"]), ("rolled_back", "UNHEALTHY", C), last)
        self.wait_os(B)
        self.assertEqual(self.state()["components"]["os"]["blocked"][C]["result"], "rolled_back")
        # Blocked for the automatic, never offered again by itself.
        self.assertEqual(self.app.update()["components"][0]["available"]["blocked"]["reason"], "UNHEALTHY")

        last = self.run_update("update.rollback")
        self.assertEqual((last["result"], last["revision"]), ("installed", A), last)
        self.wait_os(A)
        self.assertEqual(self.state()["components"]["os"]["blocked"][B]["reason"], "ROLLED_BACK_BY_HAND")
        self.assertEqual(sorted(os.listdir(os.path.join(self.P, "opt/mowbite"))), sorted([A, B, "current", "previous"]))

        # An old serial leaves the inbox at the next check and cannot be started.
        old = self.bundle("d000004", serial=1)
        self.check()
        self.assertFalse(os.path.exists(old))
        answer = self.app.call("update.start", component="os", revision="d000004")
        self.assertEqual(answer["error"]["data"]["reason"], "UNKNOWN_REVISION")
        # By hand with mowbite-update it is refused for its serial.
        self.kill_host()
        old = self.bundle("d000004", serial=1, folder=self.root)
        tool = self.by_hand("install", old)
        self.assertEqual(tool.returncode, 1, tool.stdout + tool.stderr)
        self.assertIn("OLD_SERIAL", tool.stdout + tool.stderr)
        self.assertEqual(self.current(), A)

    def test_kill_in_every_phase(self):
        self.start_host()
        self.assertEqual(self.install(A)["result"], "installed")
        self.wait_os(A)
        phases = (("verifying", "slow-verify"), ("unpacking", "slow-unpack"), ("waiting_gate", "os-state"),
                  ("switching", "slow-stop"), ("checking", "slow-start"))
        for n, (phase, marker) in enumerate(phases):
            with self.subTest(phase=phase):
                rev = f"e00000{n}"
                self.start_into(phase, marker, rev)
                if phase == "switching":
                    # While the OS before it stops.
                    until(lambda: os.path.exists(os.path.join(self.P, "opt/mowbite/pending")), what="pending")
                last = self.cut_and_restart(marker, rev, phase in ("switching", "checking"))
                self.assertEqual(self.current(), A)
                self.assertFalse(os.path.lexists(os.path.join(self.P, "opt/mowbite/pending")))
                if last:
                    self.assertEqual((last["result"], last["reason"], last["revision"]),
                                     ("failed", "INTERRUPTED", rev), last)
                    # Once cut off is not blocked yet.
                    self.assertNotIn(rev, self.state()["components"]["os"]["blocked"])
                    self.assertEqual(self.state()["components"]["os"]["attempts"].get(rev), 1)
        # After all of it an update goes through, and the leftovers of the cut ones are gone.
        last = self.install("f000006")
        self.assertEqual(last["result"], "installed", last)
        self.wait_os("f000006")
        self.assertEqual(sorted(os.listdir(os.path.join(self.P, "opt/mowbite"))),
                         sorted([A, "f000006", "current", "previous"]))

    # The planner switches like the OS: a cut in every phase leaves the OS with the planner it had, here its own.
    def test_planner_kill_in_every_phase_and_back(self):
        self.start_host()
        self.assertEqual(self.install(A)["result"], "installed")
        self.wait_os(A)
        self.assertIsNone(self.fake()["planner"])
        phases = (("verifying", "slow-verify"), ("waiting_gate", "os-state"), ("switching", "slow-stop"),
                  ("checking", "slow-start"))
        for n, (phase, marker) in enumerate(phases):
            with self.subTest(phase=phase):
                rev = f"d00000{n}"
                self.start_into(phase, marker, rev, component="planner")
                if phase == "switching":
                    until(lambda: os.path.exists(os.path.join(self.P, PLANNER, "pending")), what="pending")
                last = self.cut_and_restart(marker, rev, phase in ("switching", "checking"))
                self.assertEqual((self.current(), self.current(PLANNER), self.current(PLANNER, "active")),
                                 (A, None, None))
                self.assertFalse(os.path.lexists(os.path.join(self.P, PLANNER, "pending")))
                self.assertIsNone(self.fake()["planner"])
                if last:
                    self.assertEqual((last["result"], last["reason"], last["component"]),
                                     ("failed", "INTERRUPTED", "planner"), last)
        good, silent = "d000007", "d000008"
        last = self.install(good, component="planner")
        self.assertEqual((last["result"], last["component"]), ("installed", "planner"), last)
        self.wait_os(A)
        self.assertEqual((self.current(PLANNER, "active"), self.fake()["planner"]), (good, good))
        self.assertEqual(self.component("planner")["source"], "own")
        # A planner that does not answer through the OS: back to the one before, and the OS runs with it.
        last = self.install(silent, component="planner", broken=True)
        self.assertEqual((last["result"], last["reason"]), ("rolled_back", "UNHEALTHY"), last)
        self.assertIn("planner.settings", last["message"])
        self.wait_os(A)
        self.assertEqual((self.current(PLANNER, "active"), self.fake()["planner"]), (good, good))
        last = self.run_update("update.rollback", component="planner")
        self.assertEqual((last["result"], last["revision"]), ("installed", None), last)
        self.wait_os(A)
        self.assertEqual((self.current(PLANNER, "active"), self.fake()["planner"]), (None, None))
        self.assertEqual(self.component("planner")["source"], "bundled")

    # Decision S1: the app is installed while the OS mows; the OS is neither stopped nor held nor reported updating.
    def test_app_while_the_os_mows(self):
        self.start_host()
        self.assertEqual(self.install(A)["result"], "installed")
        self.wait_os(A)
        with open(os.path.join(self.P, "state/os-state"), "w") as f:
            f.write("MOWING")
        session = os.path.join(self.P, "userdata/mowbite/os-session.json")

        def mowing():
            with open(session) as f:
                return json.load(f).get("state") == "MOWING"
        until(mowing, what="the OS mows")
        pid, seen = self.fake()["pid"], len(self.app.os_states)
        rev = "f000001"
        last = self.install(rev, component="app")
        self.assertEqual((last["result"], last["component"], last["revision"]), ("installed", "app", rev), last)
        self.assertEqual((self.current(APP), self.current(APP, "active")), (rev, rev))
        app = self.component("app")
        self.assertEqual((app["source"], app["running"]["version"]), ("own", "1.5.0"))
        self.assertEqual((self.fake()["pid"], self.running()), (pid, {A}))
        self.assertTrue(mowing())
        self.assertFalse(os.path.exists(os.path.join(self.P, "userdata/mowbite/hold.json")))
        self.assertNotIn("updating", self.app.os_states[seen:])
        # The planner waits for the OS to idle on its contacts.
        self.bundle("d000001", component="planner")
        self.check()
        self.app.call("update.start", component="planner", revision="d000001")
        self.app.wait(lambda: self.app.update().get("phase") == "waiting_gate" and
                      self.app.update().get("reason") == "BUSY", "gate closed")
        self.assertIn("result", self.app.call("update.abort"))
        self.app.wait(lambda: (self.app.update().get("last") or {}).get("result") == "aborted", "aborted")
        self.assertEqual((self.fake()["pid"], self.current(PLANNER)), (pid, None))

    def test_origin_swapped_files_and_gate(self):
        self.start_host()
        self.assertEqual(self.install(A)["result"], "installed")
        self.wait_os(A)
        component = lambda: self.app.update()["components"][0]  # noqa: E731
        # Signed by a key the mower does not know: never a candidate.
        self.bundle(B, key=self.foreign)
        self.check()
        self.assertIsNone(component()["available"])
        # An index signed by a foreign key hides even a good bundle.
        self.bundle(B)
        index = os.path.join(self.P, "userdata/mowbite-update/inbox/index.json")
        with open(index, "w") as f:
            json.dump({"bundles": []}, f)
        self.sign(index, "mowbite-index", key=self.foreign)
        self.check()
        self.assertIsNone(component()["available"])
        os.remove(index)
        os.remove(index + ".sig")
        # Swapped after the check for another signed bundle: refused, nothing switched.
        path = self.bundle(B)
        self.check()
        self.assertEqual(component()["available"]["revision"], B)
        self.bundle(C, folder=self.root)
        shutil.copy(os.path.join(self.root, f"mowbite-os-{C}.tar.gz"), path)
        shutil.copy(os.path.join(self.root, f"mowbite-os-{C}.tar.gz.sig"), path + ".sig")
        last = self.run_update(revision=B)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"), last)
        self.assertEqual(self.current(), A)
        # The gate stays closed while the OS mows; abort ends the wait, nothing switched.
        os.remove(path)
        os.remove(path + ".sig")
        self.bundle(B)
        self.check()
        with open(os.path.join(self.P, "state/os-state"), "w") as f:
            f.write("MOWING")
        self.app.call("update.start", component="os", revision=B)
        self.app.wait(lambda: self.app.update().get("phase") == "waiting_gate" and
                      self.app.update().get("reason") == "BUSY", "gate closed")
        self.assertIn("result", self.app.call("update.abort"))
        self.app.wait(lambda: (self.app.update().get("last") or {}).get("result") == "aborted", "aborted")
        self.assertEqual(self.current(), A)
        self.assertEqual(self.running(), {A})
        self.assertFalse(os.path.exists(os.path.join(self.P, "userdata/mowbite/hold.json")))

    # The host updates itself: the switch ends it, the next start (systemd) tries the new one, which confirms itself;
    # a broken one does not come up and the start after it goes back. The OS runs on all along.
    def test_host_updates_itself_and_goes_back(self):
        seed = os.path.join(self.P, "usr/lib/mowbite-host")
        shutil.copytree(os.path.join(LIB, "mowbite_host"), os.path.join(seed, "mowbite_host"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        with open(os.path.join(seed, "bundle.json"), "w") as f:
            json.dump({"format": 2, "component": "host", "revision": "5eed000", "serial": 1}, f)
        self.bundle(A)
        self.start_host(seed=True)
        self.check()
        self.assertEqual(self.run_update()["result"], "installed")
        os_pids = set(self.fake_pids())
        self.assertEqual(self.component("host")["running"]["revision"], "5eed000")

        self.bundle(B, component="host")
        self.check()
        self.app.call("update.start", component="host")
        self.assertEqual(self.host.wait(WAIT), 0)
        self.start_host(seed=True)
        self.app.wait(lambda: (self.app.update().get("last") or {}).get("component") == "host", "host confirmed")
        self.assertEqual(self.app.update()["last"]["result"], "installed")
        self.assertEqual(self.component("host")["running"]["revision"], B)
        self.assertEqual(self.current("opt/mowbite-host"), B)
        self.assertFalse(os.path.lexists(os.path.join(self.P, "opt/mowbite-host/pending")))

        self.bundle(C, component="host", broken=True)
        self.check()
        self.app.call("update.start", component="host")
        self.assertEqual(self.host.wait(WAIT), 0)
        self.start_host(seed=True, wait=False)
        self.assertNotEqual(self.host.wait(WAIT), 0)
        self.start_host(seed=True)
        self.app.wait(lambda: (self.app.update().get("last") or {}).get("revision") == C, "result of the way back")
        last = self.app.update()["last"]
        self.assertEqual((last["result"], last["reason"]), ("rolled_back", "UNHEALTHY"))
        self.assertEqual(self.current("opt/mowbite-host"), B)
        self.assertEqual(self.component("host")["running"]["revision"], B)
        self.assertEqual(set(self.fake_pids()), os_pids, "the OS ran on")

    # A tool that stopped the OS itself holds the lock of the host (mowbite-measure): a host started meanwhile waits
    # without the broker and takes nothing back, mowbite-update is refused, and unfinished answers under the lock.
    def test_the_notify_socket_stays_with_the_host(self):
        # Tools of systemd tell their exit status to a NOTIFY_SOCKET they inherit.
        told = []
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.addCleanup(sock.close)
        sock.bind(os.path.join(self.root, "notify"))
        sock.settimeout(0.2)

        def read():
            while sock.fileno() >= 0:
                try:
                    told.append(sock.recv(64))
                except OSError:
                    pass
        threading.Thread(target=read, daemon=True).start()
        self.start_host(notify=sock.getsockname())
        self.install(A)
        self.wait_os(A)
        with open(self.pid_file()) as f:
            pid = int(f.read().split()[0])
        with open(f"/proc/{pid}/environ", "rb") as f:
            self.assertNotIn(b"NOTIFY_SOCKET=", f.read())
        until(lambda: b"WATCHDOG=1" in told, what="the host telling systemd that it is alive")
        self.assertEqual(told[0], b"READY=1")

    def test_host_waits_while_its_lock_is_held(self):
        self.start_host()
        self.install(A)
        self.wait_os(A)
        self.kill_host()
        with open(self.pid_file()) as f:
            os.killpg(int(f.read().split()[0]), signal.SIGINT)
        until(lambda: not self.fake_pids(), what="the OS stopped")
        self.assertEqual(self.by_hand("unfinished").returncode, 0)
        pending = os.path.join(self.P, "opt/mowbite/pending")
        with open(pending, "w") as f:
            json.dump({"to": A, "from": A, "ran": True}, f)
        with open(os.path.join(self.P, "run/mowbite-host.lock"), "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.start_host(wait=False)
            log = os.path.join(self.root, f"host-{self.starts}.log")
            until(lambda: "waiting for it" in open(log).read(), what="the host waiting for its lock")
            time.sleep(1.5)
            self.assertIsNone(self.host.poll())
            self.assertIsNot((self.app.statuses.get("mowbite/host/status") or {}).get("online"), True)
            self.assertEqual((self.fake_pids(), os.path.lexists(pending)), ([], True))
            asked = self.by_hand("unfinished")
            self.assertEqual((asked.returncode, asked.stdout.startswith("os unfinished update: ")), (1, True), asked)
            self.assertEqual(self.by_hand("recover").returncode, 1)
            self.assertTrue(os.path.lexists(pending))
        # The lock is free: the host takes the cut switch back and starts the OS that ran before it.
        self.wait_os(A)
        self.assertEqual(self.by_hand("unfinished").returncode, 0)

if __name__ == "__main__":
    unittest.main(verbosity=1)
