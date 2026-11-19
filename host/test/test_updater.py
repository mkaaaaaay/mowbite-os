"""Tests of the core of mowbite-host without systemd and broker: an OS stand-in that writes its session file.

  python3 -I host/test/test_updater.py
"""
import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "boot"))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import support  # noqa: E402
from mowbite_host import Refused, bundle, carrier, files, gate, installer, net, origin, paths, updater  # noqa: E402

A, B, C, D = "aaaaaaa", "bbbbbbb", "ccccccc", "ddddddd"


class FakeOs:
    """The carrier and the broker side of the OS: a start writes os-session.json of this boot, IDLE on the contacts;
    a hold asked for is granted into hold.json unless grant is False."""

    def __init__(self, where):
        self.paths = where
        self.running = False
        self.grant = True
        self.broken = set()
        self.holds = []
        # How many releases of the hold get lost on their way.
        self.lost_releases = 0
        self.started = None
        self.starts = self.stops = 0
        # The planner the last start took: the revision active named, None for the one of the OS.
        self.planner = None
        # The state of the service when it is neither active nor inactive.
        self.state = None

    def session(self, **fields):
        record = {"state": "IDLE", "job_id": "", "boot_id": "b1", "since": 0, "clean": False, "emergency": False,
                  "charging": True, **fields}
        os.makedirs(os.path.dirname(self.paths.session), exist_ok=True)
        with open(self.paths.session, "w") as f:
            json.dump(record, f)

    def active(self):
        return self.running

    def show(self):
        return self.state or ("active" if self.running else "inactive"), None, self.started

    def restart(self):
        self.stop()
        return self.start()

    # Like os-setup it takes the planner active names, else the one of the OS.
    def start(self):
        self.running, self.state = True, None
        self.started = time.time()
        self.planner = files.link_target(self.paths.places["planner"].active)
        self.starts += 1
        if not {os.readlink(self.paths.os.current), self.planner} & self.broken:
            self.session()
        return True

    def stop(self):
        if self.running:
            self.session(clean=True)
            self.stops += 1
        self.running = False

    def request_hold(self, seconds):
        self.holds.append(seconds)
        if self.grant:
            now = int(time.time())
            with open(self.paths.hold, "w") as f:
                json.dump({"until": now + seconds, "set_at": now}, f)

    def release_hold(self):
        self.holds.append(0)
        if self.lost_releases > 0:
            self.lost_releases -= 1
            return
        if os.path.exists(self.paths.hold):
            os.remove(self.paths.hold)


class Base(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-host-")
        self.paths = paths.Paths(os.path.join(self.root, "fs"))
        for folder in (*(place.opt for place in self.paths.places.values()), self.paths.inbox,
                       os.path.dirname(self.paths.signers), os.path.dirname(self.paths.boot_id),
                       os.path.dirname(self.paths.base)):
            os.makedirs(folder)
        self.write(self.paths.boot_id, "b1\n")
        self.write(self.paths.base, "1\n")
        self.write(self.paths.os_setup, "true\n")
        self.key = support.keygen(os.path.join(self.root, "key"))
        namespaces = "mowbite-os,mowbite-planner,mowbite-app,mowbite-host,mowbite-index"
        self.write(self.paths.signers, f'mowbite namespaces="{namespaces}" {support.public(self.key)}\n')
        self.fake = FakeOs(self.paths)
        health = installer.Health(self.paths, self.fake, wait=0, stable=0, every=0)
        self.updater = updater.Updater(self.paths, self.fake, health, origin.Inbox(self.paths), self.fake)
        self.updater.gate_wait = 0
        self.serial = 1000
        # A hold not granted and a closed gate are judged again within a fraction of a second.
        self.waits = updater.HOLD_WAIT, updater.GATE_EVERY, updater.RELEASE_WAIT
        updater.HOLD_WAIT, updater.GATE_EVERY, updater.RELEASE_WAIT = 0.3, 0.05, 0.2

    def tearDown(self):
        updater.HOLD_WAIT, updater.GATE_EVERY, updater.RELEASE_WAIT = self.waits
        shutil.rmtree(self.root)

    def write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)

    def sign(self, path, namespace):
        support.sign(path, self.key, namespace)

    # A signed bundle of rev in the inbox (or folder), with a newer serial unless given; extra: more files {name:
    # bytes}, modes: {name: mode} for files not 0644.
    def bundle(self, rev, serial=None, folder=None, extra=None, component="os", modes=None, **meta):
        self.serial += 1
        path = os.path.join(folder or self.paths.inbox, f"mowbite-{component}-{rev}.tar.gz")
        common = {"format": 2, "component": component, "revision": rev, "serial": serial or self.serial}
        if component == "os":
            meta = {"base": 1, "min_app": "1.4.0", "runtime_packages": [], **meta}
        members = {f"{rev}/bundle.json": json.dumps({**common, **meta}).encode(), f"{rev}/revision": rev.encode()}
        members.update({f"{rev}/{name}": data for name, data in (extra or {}).items()})
        with tarfile.open(path, "w:gz") as tar:
            for name, data in members.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = (modes or {}).get(name[len(rev) + 1:], 0o644)
                tar.addfile(info, io.BytesIO(data))
        self.sign(path, f"mowbite-{component}")
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

    # update.status names api and api_break of an OS on offer, null when its bundle.json does not.
    def test_available_os_names_its_api(self):
        self.bundle(A)
        self.updater.check()
        self.assertEqual({k: self.updater.status()["components"][0]["available"][k] for k in ("api", "api_break")},
                         {"api": None, "api_break": None})
        self.bundle(B, api=31, api_break=1)
        self.updater.check()
        self.assertEqual({k: self.updater.status()["components"][0]["available"][k] for k in ("api", "api_break")},
                         {"api": 31, "api_break": 1})

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

    # A bundle checked before its .sig arrived is read again with it.
    def test_signature_after_the_bundle(self):
        path = self.bundle(A)
        os.rename(path + ".sig", path + ".later")
        self.updater.check()
        self.assertIsNone(self.updater.status()["components"][0]["available"])
        os.rename(path + ".later", path + ".sig")
        self.updater.check()
        self.assertEqual(self.updater.status()["components"][0]["available"]["revision"], A)

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


class NetTest(Base):
    """The net as a folder of files: fetch copies BASE + <path> from self.served, the way download writes it."""

    def setUp(self):
        super().setUp()
        self.served = os.path.join(self.root, "served")
        os.makedirs(os.path.join(self.served, "index"))
        self.fetched = []
        self.net = net.Net(self.paths, fetch=self.fetch)
        self.updater = updater.Updater(self.paths, self.fake, self.updater.installers["os"].health,
                                       origin.Inbox(self.paths), self.fake, net=self.net)
        self.updater.gate_wait = 0
        self.index_serial = 100

    def fetch(self, url, path, limit, seconds):
        self.assertTrue(url.startswith(net.BASE))
        self.fetched.append(url[len(net.BASE):])
        source = os.path.join(self.served, url[len(net.BASE):])
        if not os.path.isfile(source):
            raise net.Failed("unreachable", f"{url}: 404")
        if os.path.getsize(source) > limit:
            raise net.Failed("too_large", f"{url} has more than {limit} bytes")
        shutil.copyfile(source, path)

    # A signed bundle of rev published as release os-<rev>: its index entry.
    def publish(self, rev, channel="preview", **fields):
        folder = os.path.join(self.served, f"os-{rev}")
        os.makedirs(folder)
        path = self.bundle(rev, folder=folder)
        with open(path, "rb") as f:
            data = f.read()
        meta = json.loads(tarfile.open(path).extractfile(f"{rev}/bundle.json").read())
        name = os.path.basename(path)
        return {"component": "os", "revision": rev, "serial": meta["serial"], "date": meta["serial"],
                "size": len(data), "sha256": hashlib.sha256(data).hexdigest(), "min_app": meta["min_app"],
                "file": name, "url": f"os-{rev}/{name}", "channel": channel, **fields}

    def index(self, entries, serial=None, namespace="mowbite-index"):
        self.index_serial += 1
        path = os.path.join(self.served, "index", origin.INDEX)
        with open(path, "w") as f:
            json.dump({"schema": 1, "serial": serial or self.index_serial, "date": 1, "bundles": entries}, f)
        self.sign(path, namespace)

    # update.check with the net, then what the main loop does once the refresh ended.
    def refresh(self):
        self.updater.check(fetch=True)
        self.assertTrue(self.net.done.wait(10))
        self.updater.poll()
        return self.updater.status()

    def available(self):
        return self.updater.status()["components"][0]["available"]

    def test_preview_only_in_preview(self):
        self.index([self.publish(A)])
        status = self.refresh()
        self.assertEqual((status["origin"]["channel"], status["origin"]["result"]), ("stable", "ok"))
        self.assertIsNone(self.available())
        self.assertNotIn(f"os-{A}/mowbite-os-{A}.tar.gz", self.fetched)
        self.updater.set_channel("preview")
        self.assertTrue(self.net.done.wait(10))
        self.updater.poll()
        self.assertEqual((self.available()["revision"], self.available()["from"]), (A, "net"))
        self.assertEqual(self.run_update("os")["result"], "installed")
        self.assertEqual(self.updater.state["origin"]["index_serial"], self.index_serial)

    def test_stable_is_taken_in_both(self):
        self.index([self.publish(A, channel="stable")])
        self.refresh()
        self.assertEqual(self.available()["revision"], A)

    def test_newest_only_and_nothing_twice(self):
        self.updater.set_channel("preview")
        self.assertTrue(self.net.done.wait(10))
        self.index([self.publish(A), self.publish(B)])
        self.fetched = []
        self.refresh()
        self.assertEqual(self.available()["revision"], B)
        self.assertNotIn(f"os-{A}/mowbite-os-{A}.tar.gz", self.fetched)
        self.fetched = []
        self.refresh()
        self.assertEqual(self.fetched, ["index/index.json", "index/index.json.sig"])

    def test_not_newer_than_installed(self):
        self.updater.set_channel("preview")
        self.assertTrue(self.net.done.wait(10))
        self.index([self.publish(A)])
        self.refresh()
        self.assertEqual(self.run_update("os")["result"], "installed")
        self.fetched = []
        self.refresh()
        self.assertEqual(self.fetched, ["index/index.json", "index/index.json.sig"])
        self.assertIsNone(self.available())

    def test_index_badly_signed(self):
        self.index([self.publish(A, channel="stable")], namespace="mowbite-os")
        self.assertEqual(self.refresh()["origin"]["result"], "bad_index")
        self.assertIsNone(self.available())
        self.assertEqual(os.listdir(self.paths.net), [])

    def test_older_index_refused(self):
        self.index([self.publish(A, channel="stable")])
        self.refresh()
        self.index([], serial=self.index_serial - 5)
        self.assertEqual(self.refresh()["origin"]["result"], "bad_index")
        self.assertEqual(self.available()["revision"], A)

    def test_bundle_not_the_one_named(self):
        self.index([self.publish(A, channel="stable", sha256="0" * 64)])
        self.assertEqual(self.refresh()["origin"]["result"], "bad_bundle")
        self.assertIsNone(self.available())
        self.assertNotIn(f"mowbite-os-{A}.tar.gz", os.listdir(self.paths.net))

    def test_larger_than_a_bundle_may_be(self):
        entry = self.publish(A, channel="stable")
        self.index([{**entry, "size": bundle.MAX_BUNDLE["os"] + 1}])
        self.assertEqual(self.refresh()["origin"]["result"], "too_large")
        self.assertNotIn(f"os-{A}/mowbite-os-{A}.tar.gz", self.fetched)

    def test_entries_with_a_path_left_out(self):
        entry = self.publish(A, channel="stable")
        bad = [{**entry, "url": "../os-a/" + entry["file"]}, {**entry, "url": "/etc/" + entry["file"]},
               {**entry, "url": "https://example.com/" + entry["file"]}, {**entry, "file": "other.tar.gz"},
               {**entry, "channel": "nightly"}]
        self.index(bad)
        self.assertEqual(self.refresh()["origin"]["result"], "ok")
        self.assertIsNone(self.available())
        self.assertEqual(sorted(os.listdir(self.paths.net)), [origin.INDEX, origin.INDEX + ".sig"])

    def test_files_the_index_drops_leave(self):
        self.index([self.publish(A, channel="stable")])
        self.refresh()
        self.index([])
        self.refresh()
        self.assertEqual(sorted(os.listdir(self.paths.net)), [origin.INDEX, origin.INDEX + ".sig"])
        self.assertIsNone(self.available())

    def test_unreachable_keeps_what_is_there(self):
        self.index([self.publish(A, channel="stable")])
        self.refresh()
        os.remove(os.path.join(self.served, "index", origin.INDEX))
        self.assertEqual(self.refresh()["origin"]["result"], "unreachable")
        self.assertEqual(self.available()["revision"], A)

    def test_channel_is_checked(self):
        with self.assertRaises(Refused):
            self.updater.set_channel("nightly")

    def test_addresses(self):
        for url in ("http://github.com/x", "https://example.com/x", "https://github.com.example.com/x",
                    "https://objects.githubusercontent.com.evil.org/x", "file:///etc/passwd"):
            with self.assertRaises(net.Failed, msg=url):
                net._allowed(url)
        for url in ("https://github.com/mkaaaaaay/mowbite-releases/releases/download/index/index.json",
                    "https://release-assets.githubusercontent.com/x?y=1"):
            net._allowed(url)
        with self.assertRaises(net.Failed):
            net._Redirect().redirect_request(None, None, 302, "Found", {}, "https://example.com/x")


class StageTest(Base):
    # The file in the inbox changes after its signature was checked: only the copy is unpacked.
    def test_swapped_after_check(self):
        good = self.bundle(A)
        staged = bundle.stage(good, good + ".sig", "os", os.path.join(self.paths.stage, "t"), self.paths.signers)
        with open(good, "wb") as f:
            f.write(b"something else")
        partial = bundle.unpack(staged, self.paths.os.opt)
        self.assertTrue(os.path.isfile(os.path.join(partial, A, "bundle.json")))

    def test_platforms_not_a_list_of_names(self):
        for platforms in ("tron", [], ["tron", 1], ["tron", ""], None):
            path = self.bundle(A, platforms=platforms)
            with self.assertRaises(Refused) as caught:
                bundle.stage(path, path + ".sig", "os", os.path.join(self.paths.stage, "t"), self.paths.signers)
            self.assertEqual(caught.exception.reason, "BAD_BUNDLE", platforms)

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

    # The release of the hold after a switch got lost: asked again until hold.json is gone.
    def test_lost_release_of_the_hold_is_asked_again(self):
        self.bundle(A)
        self.updater.check()
        self.run_update("os")
        self.bundle(B)
        self.updater.check()
        self.fake.lost_releases = 1
        self.assertEqual(self.run_update("os")["result"], "installed")
        self.assertFalse(os.path.lexists(self.paths.hold))
        self.assertEqual(self.fake.holds[-2:], [0, 0])

    # An OS that runs and never gives the hold back keeps its file: its own state would hold on without it.
    def test_hold_never_given_back_stays_with_the_running_os(self):
        self.bundle(A)
        self.updater.check()
        self.run_update("os")
        self.bundle(B)
        self.updater.check()
        self.fake.lost_releases = 100
        self.assertEqual(self.run_update("os")["result"], "installed")
        self.assertTrue(self.fake.running)
        self.assertTrue(os.path.lexists(self.paths.hold))
        self.assertEqual(self.fake.holds[-updater.RELEASE_TRIES:], [0] * updater.RELEASE_TRIES)

    def test_halted_and_down_count_as_stopped(self):
        self.fake.session()
        self.assertIsNone(gate.off(self.paths, None))
        self.write(self.paths.exit_note, json.dumps({"t": 1000.0, "down": True}))
        self.assertEqual(gate.off(self.paths, 1000), "down")
        # A note from before this start of the service says nothing about it.
        self.assertIsNone(gate.off(self.paths, 1002))
        os.remove(self.paths.exit_note)
        self.write(self.paths.halt, "MOWBITE_OS_HALT reason=SCHEMA_NEWER data_schema=9 image_schema=8\n")
        self.assertEqual(gate.off(self.paths, None), "halted")
        self.assertEqual(gate.halt_reason(self.paths), "SCHEMA_NEWER")
        self.fake.session(state="MOWING")
        self.write(self.paths.restart_hold, "{}")
        self.assertEqual(gate.before_hold(self.paths, True)[0], "EMERGENCY")
        self.assertEqual(gate.before_hold(self.paths, False)[0], "EMERGENCY")
        self.assertIsNone(gate.before_hold(self.paths, True, True))
        self.write(self.paths.os.pending, "{}")
        self.assertEqual(gate.before_hold(self.paths, True, True)[0], "UNFINISHED")

    # Files in the folders of the OS user: never read or written through a link, never more than a small text.
    def test_no_link_followed_in_the_folders_of_the_os(self):
        other = os.path.join(self.root, "of-root")
        self.write(other, json.dumps({"boot_id": "b1", "clean": False, "reason": "reason=SECRET"}))
        for path in (self.paths.session, self.paths.halt, self.paths.hold, self.paths.hold + ".tmp"):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            os.symlink(other, path)
        self.assertIsNone(gate.session_of_this_boot(self.paths))
        self.assertEqual(gate.halt_reason(self.paths), "DATA_ERROR")
        files.write_new(self.paths.hold, "{}\n")
        self.assertFalse(os.path.islink(self.paths.hold))
        self.assertIn("SECRET", files.read_text(other))
        self.write(other, "x" * (files.MAX_TEXT + 1))
        self.assertIsNone(files.read_text(other))

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
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertFalse(os.path.lexists(self.paths.os.pending))

    # An abort the caller got a yes for: no switch after it, although the hold arrives.
    def test_accepted_abort_is_never_switched(self):
        self.install(A)
        self.bundle(B)
        self.updater.check()
        updater.HOLD_WAIT = 5.0
        asked = threading.Event()
        request_hold = self.fake.request_hold
        self.fake.grant = False

        def ask(seconds):
            request_hold(seconds)
            asked.set()
        self.fake.request_hold = ask
        self.updater.start("os", B)
        self.assertTrue(asked.wait(10))
        self.updater.abort()
        self.fake.grant = True
        request_hold(900)
        self.assertEqual(self.updater.join()["result"], "aborted")
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertEqual(self.fake.holds[-1], 0)
        # Also when it comes between the gate and the switch.
        self.updater._abort.set()
        with self.assertRaises(updater.Aborted):
            self.updater._switch("os", B)
        self.assertEqual(self.updater.phase(), "idle")

    # An error after the stop (no memory for a fork, a full disk): back to the one before, no unfinished update.
    def test_error_in_the_switch_goes_back(self):
        self.install(A)
        once, calls = self.updater.installer.health.once, []

        def fail_first(since, entered=None):
            calls.append(since)
            if len(calls) == 1:
                raise OSError(12, "Cannot allocate memory")
            return once(since, entered)
        self.updater.installer.health.once = fail_first
        last = self.install(B)
        self.assertEqual((last["result"], last["reason"]), ("failed", "INTERNAL"))
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertTrue(self.fake.running)
        self.assertFalse(os.path.lexists(self.paths.os.pending))
        self.assertEqual(self.updater.state["components"]["os"]["attempts"], {})
        self.assertEqual(self.install(B)["result"], "installed")

    def test_stop_that_fails_half_starts_the_old_one_again(self):
        self.install(A)
        stop = self.fake.stop

        def half():
            stop()
            raise Refused("BUSY", "systemctl stop mowbite-os.service: timed out")
        self.fake.stop = half
        last = self.install(B)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BUSY"))
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertTrue(self.fake.running)
        self.assertFalse(os.path.lexists(self.paths.os.pending))

    def test_restart_between_two_looks_is_unhealthy(self):
        self.install(A)
        health = self.updater.installer.health
        health.stable, health.every = 0.3, 0.05
        show = self.fake.show
        self.fake.show = lambda: (*show()[:2], time.time())
        last = self.install(B)
        self.assertEqual(last["result"], "rolled_back")
        self.assertTrue(last["message"].startswith("the service started again"), last["message"])
        self.assertEqual(os.readlink(self.paths.os.current), A)

    def test_folder_of_the_revision_from_another_bundle_is_replaced(self):
        self.install(A)
        os.makedirs(self.paths.os.of(B))
        self.write(os.path.join(self.paths.os.of(B), "bundle.json"), "{}")
        self.assertEqual(self.install(B)["result"], "installed")
        with open(os.path.join(self.paths.os.of(B), "revision")) as f:
            self.assertEqual(f.read(), B)

    def test_installed_revision_from_another_bundle_is_refused(self):
        self.install(A)
        last = self.install(A)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"))
        self.assertEqual(os.readlink(self.paths.os.current), A)

    def test_nothing_half_unpacked_stays_after_an_error(self):
        def slow(component, tree):
            raise subprocess.TimeoutExpired("ldd", 600)
        self.updater._tree_problem = slow
        last = self.install(A)
        self.assertEqual((last["result"], last["reason"]), ("failed", "INTERNAL"))
        self.assertEqual(sorted(os.listdir(self.paths.os.opt)), [])

    def test_stop_that_fails_does_not_keep_the_unhealthy_one(self):
        self.install(A)
        self.fake.broken.add(B)
        stop, calls = self.fake.stop, []

        def refuse_second():
            calls.append(1)
            if len(calls) == 2:
                raise Refused("BUSY", "systemctl stop failed")
            stop()
        self.fake.stop = refuse_second
        self.assertEqual(self.install(B)["result"], "rolled_back")
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertFalse(os.path.lexists(self.paths.os.pending))

    # From its stop on the host holds the mower itself, also when the OS was off and nobody granted a hold.
    def test_switch_holds_the_os_it_starts(self):
        self.install(A)
        self.fake.stop()
        start, left = self.fake.start, []

        def start_and_look():
            left.append((files.read_json(self.paths.hold) or {}).get("until", 0) - time.time())
            return start()
        self.fake.start = start_and_look
        self.assertEqual(self.install(B)["result"], "installed")
        self.assertGreater(left[0], installer.HOLD_SECONDS - 60)
        self.assertNotIn(900, self.fake.holds)
        self.assertFalse(os.path.exists(self.paths.hold))

    def test_service_that_starts_or_stops_is_busy(self):
        self.install(A)
        self.bundle(B)
        self.updater.check()
        for state in ("activating", "deactivating", "unknown"):
            self.fake.state = state
            self.assertEqual(self.run_update("os", B)["reason"], "BUSY")
        self.assertNotIn(900, self.fake.holds)

    # The container stays down after its crashes and holds a restart hold nobody can release: the way back is open.
    def test_back_out_of_down(self):
        self.install(A)
        self.install(B)
        self.write(self.paths.exit_note, json.dumps({"t": time.time() - 0.5, "down": True, "restart": 3}))
        self.write(self.paths.restart_hold, "{}")
        self.fake.session(state="MOWING")
        self.fake.grant = False
        asked = len(self.fake.holds)
        self.updater.rollback("os")
        self.assertEqual(self.updater.join()["result"], "installed")
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertNotIn(900, self.fake.holds[asked:])
        self.assertTrue(os.path.exists(self.paths.restart_hold))

    # Given up by systemd after its crashes, with the restart hold no one can release.
    def test_back_out_of_failed(self):
        self.install(A)
        self.install(B)
        self.fake.stop()
        self.fake.state = "failed"
        self.write(self.paths.restart_hold, "{}")
        self.updater.rollback("os")
        self.assertEqual(self.updater.join()["result"], "installed")
        self.assertEqual(os.readlink(self.paths.os.current), A)
        self.assertTrue(os.path.exists(self.paths.restart_hold))

    # The gate opened after the end of the window: no switch.
    def test_no_switch_after_the_window(self):
        self.install(A)
        self.bundle(B)
        self.updater.check()
        last = self.run_update("os", B, by_hand=False, gate_until=time.monotonic() - 1)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BUSY"))
        self.assertEqual(os.readlink(self.paths.os.current), A)

    def test_library_missing_blocks(self):
        self.write(self.paths.os_setup, "return 1\n")
        self.bundle(A, extra={"lib/x.so": b"\x7fELF" + bytes(60)})
        self.updater.check()
        self.assertEqual(self.run_update("os", A)["reason"], "LIBRARY_MISSING")
        self.assertEqual(self.blocked()[A]["reason"], "LIBRARY_MISSING")
        with self.assertRaises(Refused):
            self.updater.start("os", A, by_hand=False)

    def test_base_too_old_blocks(self):
        last = self.install(A, base=7)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BASE_TOO_OLD"))
        self.assertEqual(self.blocked()[A]["reason"], "BASE_TOO_OLD")
        self.assertNotIn("result", self.blocked()[A])

    def test_other_platform_blocks(self):
        self.write(self.paths.env_files[0], "MOWBITE_PLATFORM=tron\n")
        last = self.install(A, platforms=["xcore", "yardforce"])
        self.assertEqual((last["result"], last["reason"]), ("refused", bundle.PLATFORM_MISMATCH))
        self.assertIn("xcore, yardforce", last["message"])
        self.assertIn("tron", last["message"])
        self.assertEqual(self.blocked()[A]["reason"], bundle.PLATFORM_MISMATCH)
        self.assertNotIn("result", self.blocked()[A])
        self.assertFalse(os.path.isdir(self.paths.os.of(A)))
        with self.assertRaises(Refused):
            self.updater.start("os", A, by_hand=False)

    def test_own_platform_installs(self):
        self.write(self.paths.env_files[0], "MOWBITE_PLATFORM=xcore\n")
        self.write(self.paths.env_files[1], "MOWBITE_PLATFORM=tron\n")
        self.assertEqual(self.install(A, platforms=["xcore", "tron"])["result"], "installed")

    # A mower without MOWBITE_PLATFORM is of the platform its running OS is built for alone.
    def test_platform_of_the_running_os(self):
        self.write(self.paths.env_files[0], "MOWBITE_HARDWARE=false\n")
        self.assertEqual(self.install(A, platforms=["tron"])["result"], "installed")
        last = self.install(B, platforms=["xcore"])
        self.assertEqual((last["result"], last["reason"]), ("refused", bundle.PLATFORM_MISMATCH))
        self.assertEqual(self.install(C, platforms=["xcore", "tron"])["result"], "installed")

    # Bundles built before platforms and mowers without MOWBITE_PLATFORM take any bundle.
    def test_platform_unknown_installs(self):
        self.write(self.paths.env_files[0], "MOWBITE_PLATFORM=tron\n")
        self.assertEqual(self.install(A)["result"], "installed")
        self.write(self.paths.env_files[0], "MOWBITE_HARDWARE=false\n")
        self.assertEqual(self.install(B, platforms=["xcore"])["result"], "installed")

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
        self.assertEqual(os.readlink(self.paths.os.current), A)

    # Left by an abort or a power cut: an unused installation and an unpack; current and previous stay.
    def left_behind(self):
        opt = self.paths.os.opt
        for name in (B, C, f".{C}.partial"):
            os.makedirs(os.path.join(opt, name, "x"))
        os.symlink(B, self.paths.os.previous)
        return opt

    def test_recover_removes_what_an_update_left(self):
        self.install(A)
        opt = self.left_behind()
        self.updater.recover()
        self.assertEqual(sorted(os.listdir(opt)), sorted([A, B, "current", "previous"]))

    def test_left_installations_go_before_the_space_check(self):
        self.install(A)
        self.left_behind()
        self.bundle(D)
        self.updater.check()
        seen, check_space = [], updater.bundle.check_space

        def space(staged, folder):
            seen.append(sorted(os.listdir(folder)))
            check_space(staged, folder)
        updater.bundle.check_space = space
        try:
            self.assertEqual(self.run_update("os", D)["result"], "installed")
        finally:
            updater.bundle.check_space = check_space
        self.assertEqual(seen, [sorted([A, B, "current", "previous"])])

    def test_nothing_is_pruned_while_pending_is_unreadable(self):
        self.install(A)
        opt = self.left_behind()
        self.write(self.paths.os.pending, "{")
        self.updater.installer.prune()
        self.assertIn(C, os.listdir(opt))
        self.assertIn(f".{C}.partial", os.listdir(opt))

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

    def test_gate_given_up_at_the_end_of_the_window(self):
        self.bundle(A)
        given = {}
        self.updater.start = lambda *args, **kwargs: given.update(kwargs)
        now = time.time()
        self.updater.tick(now)
        left = updater.window_left(time.localtime(now), "23:59")
        self.assertAlmostEqual(given["gate_until"] - time.monotonic(), left, delta=2)

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

    # A gate that stays closed is not tried again every few minutes with a copy of the bundle each time.
    def test_each_revision_once_a_night(self):
        self.bundle(A)
        self.updater.tick()
        self.updater.join()
        self.fake.session(charging=False)
        self.bundle(B)
        now = time.time()
        self.updater.tick(now)
        self.assertEqual(self.updater.join()["reason"], "NO_DOCK_CONTACT")
        self.updater.state["last"] = None
        self.updater.tick(now + 600)
        self.assertFalse(self.updater.busy())
        self.assertIsNone(self.updater.state["last"])
        self.fake.session()
        self.updater.set_auto(True, "00:00", "23:59")
        later = now + updater.AUTO_AGAIN + 1
        # 23:59 is outside the window of this test.
        later += 120 if time.localtime(later)[3:5] == (23, 59) else 0
        self.updater.tick(later)
        self.assertEqual(self.updater.join()["result"], "installed")

    def test_os_stopped_by_hand_is_left_alone(self):
        self.bundle(A)
        self.updater.tick()
        self.updater.join()
        self.fake.stop()
        self.bundle(B)
        self.updater.state["last"] = None
        self.updater.tick()
        self.assertFalse(self.updater.busy())
        self.assertIsNone(self.updater.state["last"])
        self.assertFalse(self.fake.running)
        # After a crash loop the update may be the way out.
        self.fake.state = "failed"
        self.updater.tick()
        self.assertEqual(self.updater.join()["result"], "installed")

    def test_set_auto_checked(self):
        with self.assertRaises(Refused):
            self.updater.set_auto(True, "25:00", "05:00")
        with self.assertRaises(Refused):
            self.updater.set_auto("yes", "02:00", "05:00")


class ProcessCarrierTest(unittest.TestCase):
    """The carrier for tests and compose: one session, found again by its pid file, stopped as a whole."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.ran = os.path.join(self.dir, "ran")
        self.pid_file = os.path.join(self.dir, "run/os.pid")

    def carrier(self, script):
        command = os.path.join(self.dir, "run-os")
        with open(command, "w") as f:
            f.write(f"#!/bin/sh\necho $$ >> {self.ran}\n{script}\n")
        os.chmod(command, 0o755)
        process = carrier.Process(command, self.pid_file, dict(os.environ), stop_wait=2.0)
        self.addCleanup(process.stop)
        return process

    def wait_for(self, check, seconds=5.0):
        deadline = time.monotonic() + seconds
        while not check() and time.monotonic() < deadline:
            time.sleep(0.02)
        return check()

    def test_found_by_its_pid_file_and_stopped(self):
        process = self.carrier("exec sleep 60")
        self.assertTrue(process.start())
        self.assertTrue(self.wait_for(lambda: os.path.exists(self.ran)))
        again = carrier.Process(process.command, self.pid_file, {})
        self.assertTrue(again.active())
        self.assertTrue(again.start())
        again.stop()
        self.assertFalse(process.active())
        self.assertFalse(os.path.exists(self.pid_file))
        with open(self.ran) as f:
            self.assertEqual(1, len(f.read().split()))

    def test_session_without_its_leader_still_runs(self):
        process = self.carrier("sleep 60 &\nexit 0")
        self.assertTrue(process.start())
        self.assertTrue(self.wait_for(lambda: process._child.poll() is not None))
        self.assertTrue(process.active())
        self.assertTrue(process.start())
        with open(self.ran) as f:
            self.assertEqual(1, len(f.read().split()), "a second OS beside the rest of the first")
        process.stop()
        self.assertFalse(process.active())

    def test_nothing_runs_when_the_pid_file_was_not_written(self):
        child = subprocess.Popen(["/bin/sh", "-c", carrier.Process.GO, "/bin/sh", "-c", f"echo x > {self.ran}"],
                                 stdin=subprocess.PIPE)
        child.stdin.close()
        self.assertEqual(1, child.wait(5))
        self.assertFalse(os.path.exists(self.ran))

    def test_stop_of_a_session_already_gone(self):
        process = self.carrier("exit 0")
        self.assertTrue(process.start())
        self.assertTrue(self.wait_for(lambda: not process.active()))
        process.stop()
        process.stop()

    def test_a_session_that_does_not_end_is_refused(self):
        process = self.carrier("exec sleep 60")
        self.assertTrue(process.start())
        self.assertTrue(self.wait_for(lambda: os.path.exists(self.ran)))
        process.stop_wait, process.KILL_WAIT = 0.1, 0.1
        stays = process._pid
        process._pid = lambda: stays() or 1
        with self.assertRaises(Refused):
            process.stop()
        process._pid = stays


class HelpersTest(unittest.TestCase):
    # Tools of the rootfs take the lock by this path (mowbite-measure); a bundle of the host does not renew them.
    def test_the_lock_keeps_its_path(self):
        self.assertEqual(paths.Paths().lock, "/run/mowbite-host.lock")

    # The environment files as systemd reads them: quotes, blanks and both kinds of comments.
    def test_env_as_systemd_reads_it(self):
        root = tempfile.mkdtemp(prefix="mowbite-host-")
        self.addCleanup(shutil.rmtree, root)
        where = paths.Paths(root)
        os.makedirs(os.path.dirname(where.env_files[1]))
        with open(where.env_files[1], "w") as f:
            f.write('MOWBITE_MQTT_PORT="1884"\n; note\n# note\n MOWBITE_PLATFORM = \'tron\'\nA="x\nB=""\n')
        self.assertEqual(where.env(), {"MOWBITE_MQTT_PORT": "1884", "MOWBITE_PLATFORM": "tron", "A": '"x', "B": ""})

    def test_window_left(self):
        at = time.struct_time((2026, 11, 11, 4, 59, 30, 2, 315, 0))
        self.assertEqual(updater.window_left(at, "05:00"), 30)
        at = time.struct_time((2026, 11, 11, 23, 0, 0, 2, 315, 0))
        self.assertEqual(updater.window_left(at, "02:00"), 3 * 3600)

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
