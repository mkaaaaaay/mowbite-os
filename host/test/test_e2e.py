"""End to end: mowbite-host as a process under a prefix with the carrier process, a private mosquitto, signed bundles
whose OS is fake_os.py, and the app side on the broker. A -> B -> broken C -> back; kill -9 of the host in every phase
of an update, then its next start recovers; a foreign origin, an old serial, swapped files, a closed gate.

  python3 -I host/test/test_e2e.py   (needs mosquitto, ssh-keygen and paho-mqtt)
"""
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
REPO = os.path.join(HERE, "..", "..")
SBIN = os.path.join(REPO, "platform", "tron", "rootfs", "files", "usr", "sbin")
try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.packettypes import PacketTypes
    from paho.mqtt.properties import Properties
except ImportError:
    mqtt = None

A, B, C = "a000001", "b000002", "c000003"
WAIT = 60.0
# dpkg-query: every package is installed; it takes 4 s while state/slow-verify is there.
DPKG_QUERY = """#!/bin/sh
[ -e "{P}/state/slow-verify" ] && sleep 4
printf 'install ok installed'
"""
# os-setup of the rootfs: sourced for the library check during the unpack; 4 s while state/slow-unpack is there.
OS_SETUP = """[ -e "{P}/state/slow-unpack" ] && sleep 4
true
"""
RUN_OS = """#!/bin/sh
exec python3 -I "$(readlink -f "{P}/opt/mowbite/current")/fake_os.py"
"""


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


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
        self.answers = {}
        self.cond = threading.Condition()
        if hasattr(mqtt, "CallbackAPIVersion"):
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="app-e2e", protocol=mqtt.MQTTv5)
        else:
            self.client = mqtt.Client(client_id="app-e2e", protocol=mqtt.MQTTv5)
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
        self.port = free_port()
        self.write("proc/sys/kernel/random/boot_id", "boot-1\n")
        self.write("usr/lib/mowbite/base", "1\n")
        self.write("usr/lib/mowbite/os-setup", OS_SETUP.format(P=P))
        self.write("usr/lib/mowbite/run-os", RUN_OS.format(P=P), mode=0o755)
        self.write("etc/mowbite/os.conf", f"MOWBITE_HOST_CARRIER=process\nMOWBITE_MQTT_PORT={self.port}\n")
        self.bin = os.path.join(self.root, "bin")
        os.makedirs(self.bin)
        with open(os.path.join(self.bin, "dpkg-query"), "w") as f:
            f.write(DPKG_QUERY.format(P=P))
        os.chmod(os.path.join(self.bin, "dpkg-query"), 0o755)
        self.key, self.foreign = self.keygen("key"), self.keygen("foreign")
        with open(self.key + ".pub") as f:
            public = " ".join(f.read().split()[:2])
        self.write("etc/mowbite/allowed_signers", f'mowbite namespaces="mowbite-os,mowbite-index" {public}\n')
        conf = os.path.join(self.root, "mosquitto.conf")
        with open(conf, "w") as f:
            f.write(f"listener {self.port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
        self.mosquitto = subprocess.Popen(["mosquitto", "-c", conf], stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
        until(lambda: self.connectable(), 5, "mosquitto")
        self.app = App(self.port)
        self.host = None
        self.starts = 0
        self.serial = 1_700_000_000

    def tearDown(self):
        self.kill_host()
        if os.path.exists(self.pid_file()):
            with open(self.pid_file()) as f:
                pid = int(f.read().split()[0])
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for pid in self.fake_pids():
            os.kill(pid, signal.SIGKILL)
        self.app.stop()
        self.mosquitto.terminate()
        self.mosquitto.wait()
        if os.environ.get("E2E_KEEP"):
            print(f"kept {self.root}")
        else:
            shutil.rmtree(self.root, ignore_errors=True)

    def connectable(self):
        try:
            socket.create_connection(("127.0.0.1", self.port), 0.2).close()
            return True
        except OSError:
            return False

    def write(self, rel, text, mode=0o644):
        path = os.path.join(self.P, rel)
        with open(path, "w") as f:
            f.write(text)
        os.chmod(path, mode)

    def keygen(self, name):
        path = os.path.join(self.root, name)
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "", "-f", path], check=True)
        return path

    def sign(self, path, namespace="mowbite-os", key=None):
        if os.path.exists(path + ".sig"):
            os.remove(path + ".sig")
        subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key or self.key, "-n", namespace, path], check=True,
                       capture_output=True, stdin=subprocess.DEVNULL, timeout=30)

    # A signed bundle of rev as make-bundle.sh lays it out, with fake_os.py as its OS and an ELF file for the check
    # of the libraries; in the inbox unless folder is given.
    def bundle(self, rev, serial=None, broken=False, key=None, folder=None, name=None):
        if serial is None:
            self.serial += 1
            serial = self.serial
        meta = {"format": 2, "component": "os", "revision": rev, "commit": rev * 5 + "abcde", "serial": serial,
                "date": serial, "base": 1, "api": 14, "min_app": "1.4.0", "runtime_packages": ["fake-runtime"]}
        path = os.path.join(folder or os.path.join(self.P, "userdata/mowbite-update/inbox"),
                            name or f"mowbite-os-{rev}.tar.gz")
        with tarfile.open(path, "w:gz") as tar:
            def add(member, data, mode=0o644):
                info = tarfile.TarInfo(f"{rev}/{member}")
                info.size, info.mode = len(data), mode
                tar.addfile(info, io.BytesIO(data))
            add("bundle.json", json.dumps(meta).encode())
            add("revision", rev.encode())
            with open(os.path.join(HERE, "fake_os.py"), "rb") as f:
                add("fake_os.py", f.read())
            with open(shutil.which("true"), "rb") as f:
                add("bin/true", f.read(), 0o755)
            if broken:
                add("broken", b"")
        self.sign(path, key=key)
        return path

    def pid_file(self):
        return os.path.join(self.P, "run/mowbite-host/os.pid")

    def start_host(self):
        self.starts += 1
        t = int(time.time())
        env = {**os.environ, "MOWBITE_UPDATE_PREFIX": self.P, "MOWBITE_HOST_LIB": LIB,
               "MOWBITE_UPDATE_HEALTH_STABLE": "1", "PATH": self.bin + os.pathsep + os.environ.get("PATH", "")}
        log = open(os.path.join(self.root, f"host-{self.starts}.log"), "w")
        self.host = subprocess.Popen([sys.executable, "-I", os.path.join(SBIN, "mowbite-host")], env=env,
                                     stdout=log, stderr=subprocess.STDOUT)
        log.close()
        self.app.wait(lambda: (self.app.statuses.get("mowbite/host/status") or {}).get("online") is True and
                      self.app.statuses["mowbite/host/status"].get("t", 0) >= t, "host status")

    def kill_host(self):
        if self.host is not None and self.host.poll() is None:
            self.host.send_signal(signal.SIGKILL)
            self.host.wait()
        self.host = None

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

    def current(self):
        try:
            return os.readlink(os.path.join(self.P, "opt/mowbite/current"))
        except OSError:
            return None

    def state(self):
        with open(os.path.join(self.P, "var/lib/mowbite-host/update.json")) as f:
            return json.load(f)

    def check(self):
        answer = self.app.call("update.check")
        self.assertIn("result", answer, answer)
        return answer["result"]

    # update.start (or another call) and the last result once the update ended.
    def run_update(self, method="update.start", **params):
        before = self.app.update().get("last")
        answer = self.app.call(method, component="os", **params)
        self.assertIn("result", answer, answer)
        self.app.wait(lambda: self.app.update().get("phase") == "idle" and self.app.update().get("last") != before,
                      "end of the update")
        return self.app.update()["last"]

    def wait_os(self, revision):
        self.app.wait(lambda: self.app.os().get("state") == "running" and self.app.os().get("revision") == revision,
                      f"OS {revision} running")
        until(lambda: self.running() == {revision}, what=f"only {revision} runs")

    def install(self, rev, **kwargs):
        self.bundle(rev, **kwargs)
        self.check()
        return self.run_update(revision=rev)

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
        tool = subprocess.run([sys.executable, "-I", os.path.join(SBIN, "mowbite-update"), "install", old],
                              env={**os.environ, "MOWBITE_UPDATE_PREFIX": self.P, "MOWBITE_HOST_LIB": LIB,
                                   "PATH": self.bin + os.pathsep + os.environ.get("PATH", "")},
                              capture_output=True, text=True, timeout=WAIT)
        self.assertEqual(tool.returncode, 1, tool.stdout + tool.stderr)
        self.assertIn("OLD_SERIAL", tool.stdout + tool.stderr)
        self.assertEqual(self.current(), A)

    def test_kill_in_every_phase(self):
        self.start_host()
        self.assertEqual(self.install(A)["result"], "installed")
        self.wait_os(A)
        state_dir = os.path.join(self.P, "state")
        phases = (("verifying", "slow-verify"), ("unpacking", "slow-unpack"), ("waiting_gate", "os-state"),
                  ("switching", "slow-stop"), ("checking", "slow-start"))
        for n, (phase, marker) in enumerate(phases):
            with self.subTest(phase=phase):
                rev = f"e00000{n}"
                with open(os.path.join(state_dir, marker), "w") as f:
                    f.write("MOWING" if marker == "os-state" else "")
                self.bundle(rev)
                self.check()
                self.app.call("update.start", component="os", revision=rev)
                self.app.wait(lambda: self.app.update().get("phase") == phase and
                              self.app.update().get("revision") == rev, phase)
                if phase == "waiting_gate":
                    self.app.wait(lambda: self.app.update().get("reason") == "BUSY", "gate closed BUSY")
                if phase == "switching":
                    # While the OS before it stops.
                    until(lambda: os.path.exists(os.path.join(self.P, "opt/mowbite/pending")), what="pending")
                self.kill_host()
                os.remove(os.path.join(state_dir, marker))
                self.start_host()
                self.wait_os(A)
                self.assertEqual(self.current(), A)
                self.assertFalse(os.path.lexists(os.path.join(self.P, "opt/mowbite/pending")))
                if phase in ("switching", "checking"):
                    last = self.app.update()["last"]
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


if __name__ == "__main__":
    unittest.main(verbosity=1)
