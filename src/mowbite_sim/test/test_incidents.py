"""Incidents of the journal (FP23 B2a) in the whole stack of sim.launch.py: secrets set before as decoys (NTRIP
password, notify token, push address), then the stop button. The rising emergency bit opens an incident that links the
segments of every journaled run, seals it after its window and serves it over incidents.*: the manifest and timeline,
a replay of its runs bit-identical, the archive in pieces with sha256, without location without the datum and with
the date moved. No decoy is in any segment of the journal or in any archive. incidents.mark is throttled per minute,
incidents.delete removes one or all.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_incidents.py
"""
import base64
import hashlib
import io
import os
import shutil
import struct
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from mowbite_msgs.msg import HardwareStatus, MqttPublish, RobotState, RpcError, RpcRequest, RpcResponse
from rcl_interfaces.msg import Log

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (ROSOUT, SimProbe, TestShutdown, journal_dir, sim_description, sim_files,  # noqa: E402,F401
                        wait_until)

WORK = tempfile.mkdtemp(prefix="mowbite_incidents_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
INCIDENTS = os.path.join(JOURNAL, "incidents")
REPLAY = os.path.join(get_package_prefix("mowbite_replay"), "bin", "mowbite_replay")
CONFIG = os.path.join(WORK, "incidents.yaml")
# Not 52.0/9.0, where the copy without location puts the mower.
DATUM = (47.61, 8.73, 50.0)
SEGMENT_S = 20
CHUNK = 256 * 1024
# The decoys: none may be written anywhere in the journal.
PASSWORD = "pw.decoy#4b1e"
TOKEN = "tk_decoy_9e2c"
PUSH = "https://push.invalid/up/decoy_c7d5"
DECOYS = [s.encode() for s in (PASSWORD, TOKEN, PUSH, "decoy_c7d5")]
STOP = HardwareStatus.REASON_STOP

with open(CONFIG, "w") as f:
    # sim.yaml, which this file replaces, and short segments so that a window closes soon.
    f.write("/**:\n  ros__parameters:\n    journal_platform: sim\n    journal_max_mb: 256.0\n"
            f"    journal_max_days: 3.0\n    journal_segment_s: {SEGMENT_S}.0\n"
            "tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            "mowbite_logic:\n  ros__parameters:\n    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n"
            "mowbite_ntrip:\n  ros__parameters:\n    access_over_rpc: true\n"
            "fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: -0.10\n")


def generate_test_description():
    return sim_description(WORK, 18891, extra_config=CONFIG, journal_dir=JOURNAL, datum_lat=str(DATUM[0]),
                           datum_long=str(DATUM[1]), datum_height=str(DATUM[2]))


class Probe(SimProbe):
    def __init__(self):
        super().__init__("incidents_probe", state=STATE, flags=FLAGS, log=RX)
        self.robot = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._hw, 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.start()

    def result(self, method, params=None, timeout=10.0):
        answer = self.rpc(method, params, timeout)
        assert answer is not None and "result" in answer, f"{method}: {answer}"
        return answer["result"]

    def incident(self, id_):
        return next((i for i in self.result("incidents.list") if i["id"] == id_), None)

    def done(self, id_, timeout):
        # Without the lock of the probe: its callbacks take the answers.
        return wait_until(lambda: (self.incident(id_) or {}).get("state") == "done", timeout, poll=0.5)

    def download(self, id_, location):
        """The whole archive in pieces, each checked against its sha256; (archive, its sha256 as told)."""
        params = {"id": id_, "location": location}
        def ready():
            answer = self.result("incidents.download", dict(params, size=1))
            return answer if answer["ready"] else None
        first = wait_until(ready, 60, poll=0.5)
        assert first, f"archive never ready ({location})"
        data, told = b"", first["archive_sha256"]
        while True:
            piece = self.result("incidents.download", dict(params, offset=len(data), size=CHUNK))
            raw = base64.b64decode(piece["data"])
            assert piece["sha256"] == hashlib.sha256(raw).hexdigest(), (len(data), piece["sha256"])
            assert piece["size"] == first["size"] and piece["archive_sha256"] == told, piece
            assert len(raw) <= CHUNK
            if not raw:
                break
            data += raw
        assert len(data) == first["size"], (len(data), first["size"])
        return data, told


def unpacked(path):
    """The unpacked bytes of every segment of the journal or run in path (mowbite_replay cat)."""
    out = subprocess.run([REPLAY, "cat", path], capture_output=True, timeout=120)
    assert out.returncode == 0, out.stderr.decode()
    return out.stdout


def coordinates(value):
    return [struct.pack("<d", value), struct.pack(">d", value), repr(value).encode()]


def extracted(archive, name):
    """The archive unpacked into a folder of WORK: its path and the unpacked bytes of every file in it."""
    folder = os.path.join(WORK, name)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        tar.extractall(folder, filter="data")
    blobs = []
    for root, _, files in os.walk(folder):
        blobs += [open(os.path.join(root, f), "rb").read() for f in files]
        if any(f.endswith(".mbj") for f in files):
            blobs.append(unpacked(root))
    return folder, blobs


class TestIncidents(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_decoys_are_set_and_the_journal_runs(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 30), p.robot)
        self.assertTrue(wait_until(lambda: p.rpc("journal.status", timeout=2), 30), "no journal.status")
        login = {"host": "127.0.0.1", "port": 9, "mountpoint": "DECOY", "user": "decoy.user", "password": PASSWORD}
        self.assertTrue(wait_until(lambda: "result" in (p.rpc("ntrip.set", login, 3) or {}), 30), "ntrip.set refused")
        p.result("notify.set", {"server": "http://127.0.0.1:9", "topic": "decoy_topic", "token": TOKEN, "lang": "en",
                                "events": ["emergency"]})
        p.result("notify.push_add", {"address": PUSH})
        status = p.result("journal.status")
        self.assertTrue(status["enabled"], status)
        self.assertEqual(status["problem"], "", status)
        self.assertGreater(status["active"], 0, status)
        self.assertEqual(status["incidents"]["count"], 0, status)
        self.assertEqual(p.result("incidents.list"), [])

    def test_2_the_stop_button_opens_an_incident(self):
        p = self.probe
        since = len(p.events)
        runs = sorted(n for n in os.listdir(JOURNAL) if n[:1].isdigit())
        p.flags(stop=1)
        event = p.event(since, "INCIDENT", 10)
        self.assertTrue(event, [e["type"] for e in p.events[since:]])
        self.assertEqual((event["reason"], event["safety"], event["severity"], event["resume"]),
                         ("EMERGENCY", True, "info", "auto"), event)
        id_ = event["incident"]
        TestIncidents.stop = (id_, runs)
        row = p.incident(id_)
        self.assertEqual((row["state"], row["safety"], row["triggers"]), ("collecting", True, ["EMERGENCY"]), row)
        # Not before it is sealed.
        early = p.rpc("incidents.download", {"id": id_})
        self.assertEqual(early["error"]["data"], {"field": "id", "reason": "state"}, early)
        refused = p.rpc("incidents.delete", {"id": id_})
        self.assertEqual(refused["error"]["data"], {"field": "id", "reason": "state"}, refused)
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.emergency_reason & STOP, 5), p.hw)
        p.flags()

    def test_3_sealed_with_every_run_and_a_timeline(self):
        p = self.probe
        id_, runs = TestIncidents.stop
        # 15 s after the trigger, then the segments open then close within SEGMENT_S.
        self.assertTrue(p.done(id_, 15 + SEGMENT_S + 30), p.incident(id_))
        got = p.result("incidents.get", {"id": id_})
        manifest = got["manifest"]
        self.assertEqual((manifest["state"], manifest["replay"]), ("done", "from_start"), manifest["replay"])
        self.assertEqual(manifest["triggers"][0]["kind"], "EMERGENCY")
        self.assertIn("STOP", manifest["triggers"][0]["reason"], manifest["triggers"])
        self.assertEqual(sorted(r["name"] for r in manifest["runs"]), runs)
        self.assertTrue(all(r["files"] for r in manifest["runs"]), manifest["runs"])
        self.assertTrue(got["timeline"], got)
        self.assertLessEqual(os.path.getsize(os.path.join(INCIDENTS, id_, "timeline.jsonl")), 64 * 1024)
        out = subprocess.run([REPLAY, "check", os.path.join(INCIDENTS, id_)], capture_output=True, text=True,
                             timeout=300)
        self.assertEqual(out.returncode, 0, out.stdout[-3000:] + out.stderr[-2000:])

    def test_4_downloads_with_and_without_location(self):
        p = self.probe
        id_, _ = TestIncidents.stop
        bad = p.rpc("incidents.download", {"id": id_, "size": CHUNK + 1})
        self.assertEqual(bad["error"]["data"], {"field": "size", "reason": "range"}, bad)
        unknown = p.rpc("incidents.download", {"id": "19700101T000000Z_0"})
        self.assertEqual(unknown["error"]["data"], {"field": "id", "reason": "unknown"}, unknown)
        datum = coordinates(DATUM[0]) + coordinates(DATUM[1])
        archives = {}
        for location in (True, False):
            data, told = p.download(id_, location)
            self.assertEqual(hashlib.sha256(data).hexdigest(), told)
            folder, blobs = extracted(data, f"download_{location}")
            archives[location] = (folder, blobs)
            for blob in blobs:
                for decoy in DECOYS:
                    self.assertNotIn(decoy, blob, f"decoy {decoy!r} in the archive (location {location})")
        # The search finds the datum where it has to be, and nowhere in the copy without location.
        with_place = archives[True][1]
        self.assertTrue(any(c in b for b in with_place for c in coordinates(DATUM[0])), "no datum with location")
        for blob in archives[False][1]:
            for c in datum:
                self.assertNotIn(c, blob, f"datum {c!r} without location")
        # Without location the names carry another date than the incident.
        names = os.listdir(archives[False][0])
        self.assertTrue(names and all(id_[:8] not in n for n in names), names)

    def test_5_no_decoy_in_the_journal(self):
        journal = unpacked(JOURNAL)
        self.assertGreater(len(journal), 0)
        for decoy in DECOYS:
            self.assertNotIn(decoy, journal, f"decoy {decoy!r} in the journal")
        with open(os.path.join(INCIDENTS, TestIncidents.stop[0], "manifest.json"), "rb") as f:
            manifest = f.read()
        for decoy in DECOYS:
            self.assertNotIn(decoy, manifest)

    def test_6_mark_is_throttled_and_delete_removes(self):
        p = self.probe
        since = len(p.events)
        t0 = time.monotonic()
        marked = p.result("incidents.mark", {"note": "the mower stood still for no reason"})
        event = p.event(since, "INCIDENT", 5, incident=marked["id"])
        self.assertEqual((event or {}).get("reason"), "MARK", event)
        # While it collects, another mark grows it.
        self.assertEqual(p.result("incidents.mark", {"note": "again"}), marked)
        long = p.rpc("incidents.mark", {"note": "x" * 501})
        self.assertEqual(long["error"]["data"], {"field": "note", "reason": "range"}, long)
        self.assertTrue(p.done(marked["id"], 15 + SEGMENT_S + 30), p.incident(marked["id"]))
        self.assertEqual(p.incident(marked["id"])["triggers"], ["MARK"])
        # Sealed within a minute of the first: the next mark is throttled.
        throttled = time.monotonic() - t0 < 55
        if throttled:
            again = p.rpc("incidents.mark", {"note": "and again"})
            self.assertEqual(again["error"]["data"], {"field": "note", "reason": "throttled"}, again)
        self.assertEqual(p.result("incidents.delete", {"id": marked["id"]}), {"deleted": 1})
        self.assertIsNone(p.incident(marked["id"]))
        self.assertFalse(os.path.exists(os.path.join(INCIDENTS, marked["id"])))
        self.assertEqual(p.result("incidents.delete"), {"deleted": 1})
        self.assertEqual(p.result("incidents.list"), [])
        status = p.result("journal.status")
        self.assertEqual(status["incidents"]["count"], 0, status)
        self.assertEqual(status["incidents"]["throttled"], [{"kind": "MARK", "count": 1}] if throttled else [], status)
        # The journal itself keeps its segments.
        self.assertTrue(unpacked(JOURNAL))
