"""Corrections from a caster to the receiver: mowbite_ntrip against a test caster on 127.0.0.1 (NTRIP 2.0 chunked and
1.0, a refused login, an outage), tron_gnss with the RTK board's handshake against fake_tron, whose receiver reports
RTK fixed only while the board passes RTCM. Checks the fix, the age limit, the CORRECTIONS events, the sensors, the
access over RPC (the test plays the bridge and the logic's logic/idle), SOURCE_CHANGED against the source used last and
that user and password show up nowhere.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_ntrip.py
"""
import base64
import json
import os
import shutil
import socket
import stat
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import GnssFix, MqttPublish, NtripStatus, RpcError, RpcMethods, RpcRequest, RpcResponse
from mowbite_msgs.msg import SensorValue
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HISTORY, LATCHED, ROSOUT, SimProbe, fake_tron, log_entries, read_json, rtcm_frame,  # noqa: E402
                        sim_files, sim_links, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_ntrip_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
ACCESS = os.path.join(WORK, "ntrip.yaml")
SOURCE = os.path.join(WORK, "ntrip_source.json")
# Test values only, they must never appear in a status, a log line, an event or an answer.
USER = "rover.of.the.test"
PASSWORD = "pw.of#the.test"
WRONG = "not.the#pw"
MAX_AGE = 5.0
FIXED = GnssFix.RTK_FIXED
FLOAT = GnssFix.RTK_FLOAT


class Caster:
    """NTRIP caster for the test. Mountpoint V2 answers NTRIP 2.0 (chunked) and streams only after the first GGA, like a
    VRS; V1 answers ICY 200 OK. Both want USER and PASSWORD. While down, every connection is closed at once."""

    def __init__(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(8)
        self.port = self.server.getsockname()[1]
        self.lock = threading.Lock()
        self.down = False
        # (time, mountpoint, version header, login ok) per connection, and the GGA lines received
        self.connections = []
        self.gga = []
        self.requests = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        conn.settimeout(5.0)
        try:
            header = b""
            while b"\r\n\r\n" not in header:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                header += chunk
            head, _, rest = header.partition(b"\r\n\r\n")
            lines = head.decode(errors="replace").split("\r\n")
            mount = lines[0].split(" ")[1].lstrip("/")
            fields = {k.strip().lower(): v.strip() for k, _, v in (line.partition(":") for line in lines[1:])}
            auth = fields.get("authorization", "")
            login = auth.startswith("Basic ") and base64.b64decode(auth[6:]).decode() == f"{USER}:{PASSWORD}"
            with self.lock:
                self.connections.append((time.time(), mount, fields.get("ntrip-version", ""), login))
                self.requests.append(head.decode(errors="replace"))
                down = self.down
            if down:
                return
            if not login:
                refusal = f'HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic realm="{mount}"\r\n\r\n'
                conn.sendall(refusal.encode())
                return
            if mount == "V2":
                conn.sendall(b"HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\nTransfer-Encoding: chunked\r\n"
                             b"Content-Type: gnss/data\r\n\r\n")
            elif mount == "V1":
                conn.sendall(b"ICY 200 OK\r\n")
            else:
                conn.sendall(b"SOURCETABLE 200 OK\r\n\r\nENDSOURCETABLE\r\n")
                return
            conn.settimeout(0.5)
            pending = rest
            seen_gga = mount != "V2"
            while True:
                with self.lock:
                    if self.down:
                        return
                try:
                    pending += conn.recv(1024)
                except socket.timeout:
                    pass
                while b"\r\n" in pending:
                    line, _, pending = pending.partition(b"\r\n")
                    if line.startswith(b"$") and line[3:6] == b"GGA":
                        seen_gga = True
                        with self.lock:
                            self.gga.append(line.decode())
                if not seen_gga:
                    continue
                data = rtcm_frame(1005, 19) + rtcm_frame(1077, 200)
                if mount == "V2":
                    data = f"{len(data):x}\r\n".encode() + data + b"\r\n"
                conn.sendall(data)
        except OSError:
            pass
        finally:
            conn.close()

    def attempts(self):
        with self.lock:
            return list(self.connections)


CASTER = Caster()


def write_access(mountpoint="V2", password=PASSWORD, version=2):
    with open(ACCESS + ".tmp", "w") as f:
        f.write(f"host: 127.0.0.1\nport: {CASTER.port}\nmountpoint: {mountpoint}\nuser: {USER}\n"
                f"password: '{password}'\nversion: {version}\n")
    os.chmod(ACCESS + ".tmp", 0o600)
    os.replace(ACCESS + ".tmp", ACCESS)


write_access()
write_json(FLAGS, {"gnss_rtcm": 1})


def generate_test_description():
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0,
                             "max_correction_age": MAX_AGE}])
    ntrip = Node(package="mowbite_ntrip", executable="mowbite_ntrip_node", output="screen",
                 parameters=[{"data_dir": WORK, "config_check": 1.0, "first_retry": 1.0, "max_retry": 4.0,
                              "rejected_retry": 8.0, "data_timeout": 3.0, "gga_interval": 2.0,
                              "max_correction_age": MAX_AGE, "access_over_rpc": True}])
    # The ptys have to exist before the driver looks for them.
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
        TimerAction(period=1.0, actions=[gnss, ntrip, launch_testing.actions.ReadyToTest()]),
    ]), {}


def board_log():
    """Entries of fake_tron about the board and the RTCM it passed."""
    return [e for e in log_entries(LOG) if "board" in e or "rtcm" in e]


def saved_source():
    """The source of the corrections used last, as mowbite_ntrip saved it; None without the file."""
    return read_json(SOURCE)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("ntrip_probe")
        self.status = None
        self.pose = None
        self.sensors = {}
        self.statuses = []
        self.node.create_subscription(NtripStatus, "gnss/ntrip_status", self._status, 10)
        self.node.create_subscription(GnssFix, "gps/absolute_pose", self._set("pose"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(SensorValue, "bridge/sensor_value", self._sensor, 100)
        self.node.create_subscription(String, "gnss/gga", lambda m: None, 10)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.idle_pub = self.node.create_publisher(Bool, "logic/idle", LATCHED)
        self.start()

    def _status(self, msg):
        with self.lock:
            self.status = msg
            self.statuses.append(msg)

    def _sensor(self, msg):
        with self.lock:
            self.sensors[msg.sensor_id] = msg

    def _error(self, msg):
        """An error without data: the tests compare the whole of it."""
        with self.lock:
            self.answers[msg.id] = {"error": {"code": msg.code, "message": msg.message}}

    def rpc(self, method, params=None, raw=None, timeout=5.0):
        """What the node answers, as the bridge would pass it on: {"result": ...} or {"error": {code, message}}."""
        return super().rpc(method, params, timeout, raw)

    def set_idle(self, idle):
        self.wait_for(lambda p: p.idle_pub.get_subscription_count() > 0)
        self.idle_pub.publish(Bool(data=idle))
        time.sleep(0.5)

    def answer_texts(self):
        with self.lock:
            return [json.dumps(a) for a in self.answers.values()]

    def corrections_events(self, since=0):
        with self.lock:
            return [e for e in self.events[since:] if e["type"] == "CORRECTIONS"]

    def source_changes(self, since=0):
        return [e for e in self.corrections_events(since) if e["reason"] == "SOURCE_CHANGED"]


class TestNtrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def connected_with_frames(self, timeout=30.0):
        return self.probe.wait_for(lambda p: p.status and p.status.state == "connected" and p.status.frames > 0 and
                                   0 <= p.status.last_frame_age < 2.0, timeout)

    def assert_source_changed(self, since, before, now):
        """One SOURCE_CHANGED since the index, from mountpoint before to now on the test caster, and now saved."""
        p = self.probe
        changed = p.wait_for(lambda p: p.source_changes(since), 5)
        self.assertTrue(changed, p.corrections_events(since))
        self.assertEqual(len(changed), 1, changed)
        event = changed[0]
        names = ("severity", "host", "port", "mountpoint", "previous_host", "previous_port", "previous_mountpoint")
        self.assertEqual({k: event.get(k) for k in names},
                         {"severity": "warning", "host": "127.0.0.1", "port": CASTER.port, "mountpoint": now,
                          "previous_host": "127.0.0.1", "previous_port": CASTER.port, "previous_mountpoint": before})
        # Holds nothing up.
        self.assertNotIn("resume", event)
        self.assertIn("may be offset", event["message"])
        self.assertTrue(p.wait_for(lambda p: (saved_source() or {}).get("mountpoint") == now, 5), saved_source())

    def test_1_ntrip2_gives_rtk_fixed(self):
        p = self.probe
        self.assertTrue(self.connected_with_frames(), f"status: {p.status}")
        self.assertTrue(p.wait_for(lambda p: p.pose and p.pose.solution == FIXED, 20),
                        f"pose solution {p.pose and p.pose.solution}")
        attempt = [a for a in CASTER.attempts() if a[3]][0]
        self.assertEqual(attempt[1:], ("V2", "Ntrip/2.0", True))
        self.assertIn("User-Agent: NTRIP ", CASTER.requests[0])
        # The VRS got the receiver's position before it streamed.
        self.assertTrue(CASTER.gga and CASTER.gga[0].startswith("$GNGGA,"))
        board = board_log()
        first_on = next(i for i, e in enumerate(board) if e.get("board") == "on_command")
        self.assertTrue(any("rtcm" in e for e in board[first_on:]))
        # No reset on the first handshake, and no RTCM reached the board before it was on.
        self.assertFalse(any(e.get("board") in ("reset", "rtcm_dropped") for e in board))
        self.assertTrue(p.wait_for(lambda p: p.sensors.get("os_ntrip_state") and
                                   p.sensors["os_ntrip_state"].text == "connected"))
        self.assertTrue(p.wait_for(lambda p: p.sensors.get("os_correction_age") and
                                   p.sensors["os_correction_age"].value < 2.0))
        # The first source of all is saved without the login and gives no event.
        self.assertTrue(p.wait_for(lambda p: saved_source(), 5), "no ntrip_source.json")
        self.assertEqual(saved_source(), {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V2"})
        self.assertEqual(p.corrections_events(), [])

    def test_2_outage_float_lost_then_restored(self):
        p = self.probe
        since = len(p.events)
        board_since = len(board_log())
        with CASTER.lock:
            CASTER.down = True
        start = time.time()
        # Past the age limit the fix counts as float, although the receiver still says fixed.
        self.assertTrue(p.wait_for(lambda p: p.pose and p.pose.solution == FLOAT, MAX_AGE + 6), "still fixed")
        lost = p.wait_for(lambda p: [e for e in p.corrections_events(since) if e["reason"] == "LOST"], MAX_AGE + 6)
        self.assertTrue(lost, p.corrections_events(since))
        self.assertEqual((lost[0]["severity"], lost[0]["resume"], lost[0]["limit"]), ("warning", "auto", MAX_AGE))
        # Without RTCM the keep-alive stops after its window (10 s) and the board falls back to LoRa 6 s later.
        self.assertTrue(p.wait_for(lambda p: any(e.get("board") == "off" for e in board_log()[board_since:]), 25),
                        "board stayed on")
        attempts = [a[0] for a in CASTER.attempts() if a[0] >= start]
        gaps = [b - a for a, b in zip(attempts, attempts[1:])]
        self.assertGreater(len(gaps), 2, attempts)
        # 1, 2, 4, 4, ... s apart (plus up to 3 s data timeout on the first), never a burst.
        self.assertTrue(all(g > 0.8 for g in gaps), gaps)
        self.assertTrue(any(g > 3.5 for g in gaps), gaps)
        with CASTER.lock:
            CASTER.down = False
        self.assertTrue(self.connected_with_frames(15), f"status: {p.status}")
        self.assertTrue(p.wait_for(lambda p: p.pose and p.pose.solution == FIXED, 15), "no fix after the outage")
        events = [e["reason"] for e in p.corrections_events(since)]
        self.assertEqual(events, ["LOST", "RESTORED"])
        # The board came back on with the next RTCM, without a reset (it fell back while nothing was wanted).
        self.assertFalse(any(e.get("board") == "reset" for e in board_log()))

    def test_3_refused_login_waits_and_reports_once(self):
        p = self.probe
        since = len(p.events)
        write_access(password=WRONG)
        rejected = p.wait_for(lambda p: p.status and p.status.state == "rejected", 10)
        self.assertTrue(rejected, f"status: {p.status}")
        self.assertEqual(p.status.last_error, "login refused (401)")
        count = len(CASTER.attempts())
        time.sleep(5.0)
        # No new attempt within rejected_retry (8 s here, 300 s on the Tron).
        self.assertEqual(len(CASTER.attempts()), count)
        events = p.corrections_events(since)
        self.assertEqual([e["reason"] for e in events], ["REJECTED"], events)
        self.assertEqual((events[0]["severity"], events[0]["resume"], events[0]["code"]), ("error", "user", 401))
        self.assertTrue(p.wait_for(lambda p: p.sensors["os_ntrip_state"].text == "rejected"))
        # The right password: the changed file ends the wait at once.
        write_access()
        self.assertTrue(self.connected_with_frames(6), f"status: {p.status}")

    def test_4_ntrip1(self):
        p = self.probe
        since = len(p.events)
        write_access(mountpoint="V1", version=1)
        self.assertTrue(p.wait_for(lambda p: any(a[1] == "V1" and a[3] for a in CASTER.attempts()), 10))
        self.assertTrue(self.connected_with_frames(10), f"status: {p.status}")
        v1 = [r for r in CASTER.requests if r.startswith("GET /V1 ")][-1]
        self.assertIn("HTTP/1.0", v1.split("\r\n")[0])
        self.assertNotIn("Ntrip-Version", v1)
        self.assertTrue(p.wait_for(lambda p: p.pose and p.pose.solution == FIXED, 15))
        # Another mountpoint in the file by hand.
        self.assert_source_changed(since, "V2", "V1")

    def test_5_without_access_file_off(self):
        p = self.probe
        since = len(p.events)
        os.remove(ACCESS)
        self.assertTrue(p.wait_for(lambda p: p.status and p.status.state == "off", 10), f"status: {p.status}")
        self.assertTrue(p.wait_for(lambda p: p.sensors["os_ntrip_state"].text == "off"))
        # No event for switching off, and the source used last stays for the next comparison.
        self.assertEqual(p.corrections_events(since), [])
        self.assertEqual(saved_source(), {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V1"})

    def test_6_access_over_rpc(self):
        p = self.probe
        self.assertTrue(p.wait_for(lambda p: p.methods.get("mowbite_ntrip"), 5), p.methods)
        self.assertEqual(sorted(p.methods["mowbite_ntrip"]), ["ntrip.delete", "ntrip.get", "ntrip.set"])
        self.assertEqual(p.rpc("ntrip.get"), {"result": {"configured": False}})
        login = {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V2", "user": USER, "password": PASSWORD}
        # Changes only in IDLE, and not before the logic said it is; reading always.
        r = p.rpc("ntrip.set", login)
        self.assertEqual(r and r["error"]["code"], -32600, r)
        self.assertIn("not known", r["error"]["message"])
        p.set_idle(False)
        for method, params in (("ntrip.set", login), ("ntrip.delete", None)):
            r = p.rpc(method, params)
            self.assertEqual(r and r["error"]["code"], -32600, r)
        self.assertEqual(p.rpc("ntrip.get"), {"result": {"configured": False}})
        self.assertFalse(os.path.exists(ACCESS))
        p.set_idle(True)
        r = p.rpc("ntrip.set", {k: v for k, v in login.items() if k not in ("user", "password")})
        self.assertEqual(r and r.get("error"),
                         {"code": -32602, "message": 'user: needed for a new caster ("" for none)'})
        self.assertFalse(os.path.exists(ACCESS))

        start = time.time()
        since = len(p.events)
        r = p.rpc("ntrip.set", login)
        self.assertEqual(r, {"result": {"configured": True, "host": "127.0.0.1", "port": CASTER.port,
                                        "mountpoint": "V2", "version": 2, "has_user": True, "has_password": True}})
        self.assertEqual(stat.S_IMODE(os.stat(ACCESS).st_mode), 0o600)
        self.assertTrue(self.connected_with_frames(10), f"status: {p.status}")
        self.assertTrue([a for a in CASTER.attempts() if a[0] >= start and a[1] == "V2" and a[3]])
        # Compared with the source used last (V1 in test_4), the removed access does not count.
        self.assert_source_changed(since, "V1", "V2")

        # On the same caster user and password stay when left out.
        start = time.time()
        since = len(p.events)
        r = p.rpc("ntrip.set", {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "/V1", "version": 1})
        self.assertEqual(r and (r["result"]["mountpoint"], r["result"]["version"], r["result"]["has_password"]),
                         ("V1", 1, True), r)
        self.assertTrue(p.wait_for(lambda p: [a for a in CASTER.attempts()
                                              if a[0] >= start and a[1] == "V1" and a[3]], 10))
        self.assertTrue(self.connected_with_frames(10), f"status: {p.status}")
        self.assert_source_changed(since, "V2", "V1")

        # Another caster gets no stored login; a refusal leaves the file as it is.
        with open(ACCESS) as f:
            saved = f.read()
        for params, raw, message in (
                ({"host": "localhost", "port": CASTER.port, "mountpoint": "V1"}, None, "user: needed for a new caster"),
                ({"host": "127.0.0.1", "port": str(CASTER.port), "mountpoint": "V1"}, None, "port: not a port number"),
                ({"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V1", "pass": PASSWORD}, None,
                 "unknown field"),
                (None, '{"host": "127.0.0.1", "password": "' + PASSWORD, "The parameters are not valid JSON")):
            r = p.rpc("ntrip.set", params, raw)
            self.assertEqual(r and r["error"]["code"], -32602, r)
            self.assertTrue(r["error"]["message"].startswith(message), r)
        with open(ACCESS) as f:
            self.assertEqual(f.read(), saved)

        # A wrong password: the caster refuses it once; the right one ends the wait at once.
        since = len(p.events)
        r = p.rpc("ntrip.set", {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V2", "password": WRONG})
        self.assertTrue(r and r["result"]["configured"], r)
        self.assertTrue(p.wait_for(lambda p: p.status and p.status.state == "rejected", 10), f"status: {p.status}")
        self.assertTrue(p.wait_for(lambda p: p.corrections_events(since), 5))
        self.assertEqual([e["reason"] for e in p.corrections_events(since)], ["REJECTED"])
        r = p.rpc("ntrip.set", {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V2", "password": PASSWORD})
        self.assertTrue(r and r["result"]["configured"], r)
        self.assertTrue(self.connected_with_frames(6), f"status: {p.status}")
        # The refused login used nothing: compared with V1, not with the refused V2.
        self.assert_source_changed(since, "V1", "V2")

        self.assertEqual(p.rpc("ntrip.delete"), {"result": {"configured": False}})
        self.assertFalse(os.path.exists(ACCESS))
        self.assertTrue(p.wait_for(lambda p: p.status and p.status.state == "off", 5), f"status: {p.status}")
        self.assertEqual(p.rpc("ntrip.get"), {"result": {"configured": False}})
        self.assertEqual(p.rpc("ntrip.delete"), {"result": {"configured": False}})

    def test_7_access_data_appear_nowhere(self):
        p = self.probe
        with p.lock:
            texts = list(p.logs) + [json.dumps(e) for e in p.events] + [s.last_error for s in p.statuses]
        texts += p.answer_texts()
        with open(SOURCE) as f:
            texts.append(f.read())
        token = base64.b64encode(f"{USER}:{PASSWORD}".encode()).decode()
        for secret in (USER, PASSWORD, WRONG, token):
            self.assertFalse([t for t in texts if secret in t], secret)

    def test_8_one_source_changed_per_change(self):
        p = self.probe
        # V2 first (none), V1 by hand, then over RPC V2, V1 and V2 after the refused login; switching off gave none.
        changes = [(e["previous_mountpoint"], e["mountpoint"]) for e in p.source_changes()]
        self.assertEqual(changes, [("V2", "V1"), ("V1", "V2"), ("V2", "V1"), ("V1", "V2")])
        self.assertEqual(saved_source(), {"host": "127.0.0.1", "port": CASTER.port, "mountpoint": "V2"})
