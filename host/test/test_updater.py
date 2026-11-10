"""Tests of the core of mowbite-host without systemd and broker: an OS stand-in that writes its session file.

  python3 -I host/test/test_updater.py
"""
import hashlib
import io
import json
import os
import shutil
import sys
import tarfile
import tempfile
import threading
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import support  # noqa: E402
from mowbite_host import Refused, bundle, gate, installer, origin, paths, updater  # noqa: E402

A, B, C = "aaaaaaa", "bbbbbbb", "ccccccc"


class FakeOs:
    """The carrier and the broker side of the OS: a start writes os-session.json of this boot, IDLE on the contacts;
    a hold asked for is granted into hold.json unless grant is False."""

    def __init__(self, where):
        self.paths = where
        self.running = False
        self.grant = True
        self.broken = set()
        self.holds = []
        self.started = None

    def session(self, **fields):
        record = {"state": "IDLE", "job_id": "", "boot_id": "b1", "since": 0, "clean": False, "emergency": False,
                  "charging": True, **fields}
        os.makedirs(os.path.dirname(self.paths.session), exist_ok=True)
        with open(self.paths.session, "w") as f:
            json.dump(record, f)

    def active(self):
        return self.running

    def show(self):
        return ("active" if self.running else "inactive"), None, self.started

    def restart(self):
        self.stop()
        return self.start()

    def start(self):
        self.running = True
        self.started = int(time.time())
        if os.readlink(self.paths.current) not in self.broken:
            self.session()
        return True

    def stop(self):
        if self.running:
            self.session(clean=True)
        self.running = False

    def request_hold(self, seconds):
        self.holds.append(seconds)
        if self.grant:
            now = int(time.time())
            with open(self.paths.hold, "w") as f:
                json.dump({"until": now + seconds, "set_at": now}, f)

    def release_hold(self):
        self.holds.append(0)
        if os.path.exists(self.paths.hold):
            os.remove(self.paths.hold)


class Base(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-host-")
        self.paths = paths.Paths(os.path.join(self.root, "fs"))
        for folder in (self.paths.opt, self.paths.inbox, os.path.dirname(self.paths.signers),
                       os.path.dirname(self.paths.boot_id), os.path.dirname(self.paths.base)):
            os.makedirs(folder)
        self.write(self.paths.boot_id, "b1\n")
        self.write(self.paths.base, "1\n")
        self.write(self.paths.os_setup, "true\n")
        self.key = support.keygen(os.path.join(self.root, "key"))
        self.write(self.paths.signers, f'mowbite namespaces="mowbite-os,mowbite-index" {support.public(self.key)}\n')
        self.fake = FakeOs(self.paths)
        health = installer.Health(self.paths, self.fake, wait=0, stable=0, every=0)
        self.updater = updater.Updater(self.paths, self.fake, health, origin.Inbox(self.paths), self.fake)
        self.updater.gate_wait = 0
        self.serial = 1000
        # A hold not granted and a closed gate are judged again within a fraction of a second.
        self.waits = updater.HOLD_WAIT, updater.GATE_EVERY
        updater.HOLD_WAIT, updater.GATE_EVERY = 0.3, 0.05

    def tearDown(self):
        updater.HOLD_WAIT, updater.GATE_EVERY = self.waits
        shutil.rmtree(self.root)

    def write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)

    def sign(self, path, namespace):
        support.sign(path, self.key, namespace)

    # A signed bundle of rev in the inbox (or folder), with a newer serial unless given.
    def bundle(self, rev, serial=None, folder=None, **meta):
        self.serial += 1
        path = os.path.join(folder or self.paths.inbox, f"mowbite-os-{rev}.tar.gz")
        files = {f"{rev}/bundle.json": json.dumps({"format": 2, "component": "os", "revision": rev,
                                                   "serial": serial or self.serial, "base": 1, "min_app": "1.4.0",
                                                   "runtime_packages": [], **meta}).encode(),
                 f"{rev}/revision": rev.encode()}
        with tarfile.open(path, "w:gz") as tar:
            for name, data in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
        self.sign(path, "mowbite-os")
        return path

    def run_update(self, *args, **kwargs):
        self.updater.start(*args, **kwargs)
        return self.updater.join()

    def blocked(self):
        return self.updater.state["components"]["os"]["blocked"]


class InboxTest(Base):
    def test_found_and_newest_available(self):
        self.bundle(A)
        self.bundle(B)
        self.updater.check()
        self.assertEqual(self.updater.status()["components"][0]["available"]["revision"], B)

    def test_install_from_inbox_and_prune(self):
        self.bundle(A)
        self.updater.check()
        self.assertEqual(self.run_update("os")["result"], "installed")
        self.updater.check()
        # Installed and older ones leave the inbox.
        self.assertEqual(os.listdir(self.paths.inbox), [])
        self.assertIsNone(self.updater.status()["components"][0]["available"])

    def test_at_most_two_per_component(self):
        for rev in (A, B, C):
            self.bundle(rev)
        self.updater.check()
        self.assertEqual(sorted(os.listdir(self.paths.inbox)), sorted(
            [f"mowbite-os-{B}.tar.gz", f"mowbite-os-{B}.tar.gz.sig", f"mowbite-os-{C}.tar.gz",
             f"mowbite-os-{C}.tar.gz.sig"]))

    def test_bad_signature_left_out(self):
        path = self.bundle(A)
        with open(path, "ab") as f:
            f.write(b"\0")
        self.updater.check()
        self.assertIsNone(self.updater.status()["components"][0]["available"])

    def index(self, entries, namespace="mowbite-index"):
        path = os.path.join(self.paths.inbox, origin.INDEX)
        with open(path, "w") as f:
            json.dump({"bundles": entries}, f)
        self.sign(path, namespace)

    def entry(self, path, **fields):
        with open(path, "rb") as f:
            data = f.read()
        meta = json.loads(tarfile.open(path).extractfile(
            f"{os.path.basename(path)[11:-7]}/bundle.json").read())
        return {"component": "os", "revision": meta["revision"], "serial": meta["serial"], "date": meta["serial"],
                "size": len(data), "sha256": hashlib.sha256(data).hexdigest(), "file": os.path.basename(path),
                **fields}

    def test_index(self):
        a = self.bundle(A)
        self.index([self.entry(a)])
        self.updater.check()
        self.assertEqual(self.run_update("os")["result"], "installed")

    def test_index_with_another_file(self):
        a = self.bundle(A)
        entry = self.entry(a)
        self.bundle(A, serial=entry["serial"], min_app="9.9.9")
        self.index([entry])
        self.updater.check()
        last = self.run_update("os")
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"))

    def test_index_signed_for_os_is_no_index(self):
        a = self.bundle(A)
        self.index([self.entry(a)], namespace="mowbite-os")
        self.updater.check()
        self.assertIsNone(self.updater.status()["components"][0]["available"])

    def test_index_names_no_path(self):
        a = self.bundle(A)
        self.index([self.entry(a, file="../" + os.path.basename(a))])
        self.updater.check()
        self.assertIsNone(self.updater.status()["components"][0]["available"])


class StageTest(Base):
    # The file in the inbox changes after its signature was checked: only the copy is unpacked.
    def test_swapped_after_check(self):
        good = self.bundle(A)
        staged = bundle.stage(good, good + ".sig", "os", os.path.join(self.paths.stage, "t"), self.paths.signers)
        with open(good, "wb") as f:
            f.write(b"something else")
        partial = bundle.unpack(staged, self.paths.opt)
        self.assertTrue(os.path.isfile(os.path.join(partial, A, "bundle.json")))

    def test_fifo_refused(self):
        fifo = os.path.join(self.paths.inbox, "mowbite-os-fifo.tar.gz")
        os.mkfifo(fifo)
        with self.assertRaises(Refused) as caught:
            bundle.copy_in(fifo, os.path.join(self.root, "copy"), 1000)
        self.assertEqual(caught.exception.reason, "BAD_BUNDLE")

    def test_too_large(self):
        path = os.path.join(self.root, "big")
        with open(path, "wb") as f:
            f.write(b"x" * 2000)
        with self.assertRaises(Refused):
            bundle.copy_in(path, os.path.join(self.root, "copy"), 1000)

    def test_not_root_runs_itself(self):
        if os.geteuid() != 0:
            self.assertEqual(bundle.as_user(["ldd", "x"]), ["ldd", "x"])


class GateTest(Base):
    def test_hold_of_before_does_not_count(self):
        now = int(time.time())
        self.write(self.paths.hold, json.dumps({"until": now + 900, "set_at": now - 30}))
        self.assertFalse(gate.hold_granted(self.paths, now, now, 800))
        self.write(self.paths.hold, json.dumps({"until": now + 900, "set_at": now}))
        self.assertTrue(gate.hold_granted(self.paths, now, now, 800))
        self.assertFalse(gate.hold_granted(self.paths, now, now, 1000))

    def test_session_of_another_boot_is_starting(self):
        self.fake.session(boot_id="b0")
        self.assertEqual(gate.before_hold(self.paths, True)[0], "BUSY")
        self.assertIsNone(gate.before_hold(self.paths, False))

    def test_hold_given_back_when_the_gate_stays_closed(self):
        self.bundle(A)
        self.updater.check()
        self.run_update("os")
        self.bundle(B)
        self.updater.check()
        self.fake.grant = False
        self.assertEqual(self.run_update("os")["reason"], "NO_HOLD")
        self.assertEqual(self.fake.holds[-1], 0)

    def test_driving_is_busy(self):
        self.bundle(A)
        self.updater.check()
        self.run_update("os")
        self.fake.session(state="DRIVING")
        self.bundle(B)
        self.updater.check()
        self.assertEqual(self.run_update("os")["reason"], "BUSY")


class FlowTest(Base):
    def install(self, rev, **meta):
        self.bundle(rev, **meta)
        self.updater.check()
        return self.run_update("os", rev)

    def test_unhealthy_blocks_for_auto_not_by_hand(self):
        self.install(A)
        self.fake.broken.add(B)
        self.assertEqual(self.install(B)["result"], "rolled_back")
        self.assertEqual(self.blocked()[B]["result"], "rolled_back")
        self.updater.check()
        self.assertEqual(self.updater.status()["components"][0]["available"]["blocked"]["reason"], "UNHEALTHY")
        with self.assertRaises(Refused):
            self.updater.start("os", B, by_hand=False)
        self.fake.broken.clear()
        self.assertEqual(self.run_update("os", B)["result"], "installed")

    # The OS does not stop: nothing switched and no unfinished update left behind.
    def test_stop_refused_leaves_no_pending(self):
        self.install(A)
        stop = self.fake.stop

        def refuse():
            raise Refused("BUSY", "systemctl stop failed")
        self.fake.stop = refuse
        last = self.install(B)
        self.fake.stop = stop
        self.assertEqual((last["result"], last["reason"]), ("refused", "BUSY"))
        self.assertEqual(os.readlink(self.paths.current), A)
        self.assertFalse(os.path.lexists(self.paths.pending))

    def test_base_too_old_blocks(self):
        last = self.install(A, base=7)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BASE_TOO_OLD"))
        self.assertEqual(self.blocked()[A]["reason"], "BASE_TOO_OLD")
        self.assertNotIn("result", self.blocked()[A])

    def test_rollback_by_hand_blocks_the_newer(self):
        self.install(A)
        self.install(B)
        self.updater.rollback("os")
        self.assertEqual(self.updater.join()["revision"], A)
        self.assertEqual(self.blocked()[B]["reason"], "ROLLED_BACK_BY_HAND")
        status = self.updater.status()["components"][0]
        self.assertEqual((status["running"]["revision"], status["previous"]["revision"]), (A, B))

    def test_status_after_install(self):
        self.install(A)
        status = self.updater.status()
        component = status["components"][0]
        self.assertEqual(status["phase"], "idle")
        self.assertEqual(component["running"]["revision"], A)
        self.assertIsNone(component["previous"])
        self.assertIsInstance(component["installed_at"], int)
        self.assertEqual(component["crashes_since_install"], 0)
        self.assertEqual(status["last"]["result"], "installed")

    def test_abort_while_waiting_for_the_gate(self):
        self.install(A)
        self.fake.session(state="MOWING")
        self.bundle(B)
        self.updater.check()
        self.updater.gate_wait = 30
        phases = []
        seen = threading.Event()

        def changed(status):
            phases.append(status["phase"])
            if status["phase"] == "waiting_gate" and status["reason"] == "BUSY":
                seen.set()
        self.updater.on_change = changed
        updater.GATE_EVERY, every = 0.05, updater.GATE_EVERY
        try:
            self.updater.start("os", B)
            self.assertTrue(seen.wait(10))
            self.updater.abort()
            self.assertEqual(self.updater.join()["result"], "aborted")
        finally:
            updater.GATE_EVERY = every
        self.assertEqual(os.readlink(self.paths.current), A)

    def test_one_at_a_time(self):
        self.install(A)
        self.fake.session(state="MOWING")
        self.bundle(B)
        self.updater.check()
        self.updater.gate_wait = 30
        self.updater.start("os", B)
        with self.assertRaises(Refused) as caught:
            self.updater.start("os", B)
        self.assertEqual(caught.exception.reason, "BUSY")
        self.updater.abort()
        self.updater.join()

    def test_no_newer(self):
        with self.assertRaises(Refused) as caught:
            self.updater.start("os")
        self.assertEqual(caught.exception.reason, "NO_NEWER")


class AutoTest(Base):
    def setUp(self):
        super().setUp()
        self.write(self.paths.prefix + "/run/systemd/timesync/synchronized", "")
        self.updater.set_auto(True, "00:00", "23:59")
        self.updater.remember_app("1.5.0")

    def test_tick_installs(self):
        self.bundle(A)
        self.updater.tick()
        self.assertEqual(self.updater.join()["result"], "installed")

    def test_no_sure_clock(self):
        os.remove(self.paths.prefix + "/run/systemd/timesync/synchronized")
        self.bundle(A)
        self.updater.tick()
        self.assertFalse(self.updater.busy())
        self.assertIsNone(self.updater.state["last"])

    def test_app_too_old(self):
        self.updater.remember_app("1.3.9")
        self.bundle(A)
        self.updater.tick()
        self.assertIsNone(self.updater.state["last"])

    def test_blocked_not_again(self):
        self.fake.broken.add(A)
        self.bundle(A)
        self.updater.tick()
        self.assertEqual(self.updater.join()["result"], "failed")
        self.updater.state["last"] = None
        self.updater.tick()
        self.assertIsNone(self.updater.state["last"])

    def test_set_auto_checked(self):
        with self.assertRaises(Refused):
            self.updater.set_auto(True, "25:00", "05:00")
        with self.assertRaises(Refused):
            self.updater.set_auto("yes", "02:00", "05:00")


class HelpersTest(unittest.TestCase):
    def test_versions(self):
        self.assertTrue(updater.app_suffices("1.4.0", "1.4.0"))
        self.assertTrue(updater.app_suffices("1.10.0", "1.9.3"))
        self.assertFalse(updater.app_suffices("1.4.0-beta.1", "1.4.0"))
        self.assertTrue(updater.app_suffices("v1.4.1", "1.4.0"))
        self.assertFalse(updater.app_suffices(None, "1.4.0"))
        self.assertTrue(updater.app_suffices(None, None))

    def test_window(self):
        at = time.struct_time((2026, 11, 10, 3, 0, 0, 1, 314, 0))
        self.assertTrue(updater.in_window(at, "02:00", "05:00"))
        self.assertTrue(updater.in_window(at, "23:00", "04:00"))
        self.assertFalse(updater.in_window(at, "04:00", "23:00"))


if __name__ == "__main__":
    unittest.main()
