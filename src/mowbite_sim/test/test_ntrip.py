"""Corrections from a caster to the receiver: mowbite_ntrip against a test caster on 127.0.0.1 (NTRIP 2.0 chunked and
1.0, a refused login, an outage), tron_gnss with the RTK board's handshake against fake_tron, whose receiver reports
RTK fixed only while the board passes RTCM. Checks the fix, the age limit, the CORRECTIONS events, the sensors and that
user and password show up nowhere.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_ntrip.py
"""
import base64
import json
import os
import shutil
import socket
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import AbsolutePose, MqttPublish, NtripStatus, SensorValue
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WORK = tempfile.mkdtemp(prefix="mowbite_ntrip_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
ACCESS = os.path.join(WORK, "ntrip.yaml")
USER = "rover"
# Test values only, they must never appear in a status, a log line or an event.
PASSWORD = "pw.of#the.test"
WRONG = "not.the#pw"
MAX_AGE = 5.0
LOST_AFTER = 4.0
FIXED = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FIXED
FLOAT = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FLOAT


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm_frame(message, length):
    payload = bytes([message >> 4, (message & 0x0F) << 4]) + bytes((i * 7) & 0xFF for i in range(length - 2))
    frame = bytes([0xD3, length >> 8, length & 0xFF]) + payload
    return frame + crc24q(frame).to_bytes(3, "big")


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
                conn.sendall(b"HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic realm=\"/" + mount.encode() +
                             b"\"\r\n\r\n")
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
with open(FLAGS, "w") as f:
    json.dump({"gnss_rtcm": 1}, f)


def generate_test_description():
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"],
        output="screen")
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0,
                             "max_correction_age": MAX_AGE}])
    ntrip = Node(package="mowbite_ntrip", executable="mowbite_ntrip_node", output="screen",
                 parameters=[{"data_dir": WORK, "config_check": 1.0, "first_retry": 1.0, "max_retry": 4.0,
                              "rejected_retry": 8.0, "data_timeout": 3.0, "gga_interval": 2.0,
                              "lost_after": LOST_AFTER}])
    # The ptys have to exist before the driver looks for them.
    return launch.LaunchDescription([
        fake_tron,
        TimerAction(period=1.0, actions=[gnss, ntrip, launch_testing.actions.ReadyToTest()]),
    ]), {}


def board_log():
    """Entries of fake_tron about the board and the RTCM it passed."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if "board" in entry or "rtcm" in entry:
                out.append(entry)
    return out


class Probe:
    def __init__(self):
        # Reentrant: wait() holds it while a predicate may call corrections_events().
        self.lock = threading.RLock()
        self.status = None
        self.pose = None
        self.logs = []
        self.events = []
        self.sensors = {}
        self.statuses = []
        self.node = rclpy.create_node("ntrip_probe")
        self.node.create_subscription(NtripStatus, "gnss/ntrip_status", self._status, 10)
        self.node.create_subscription(AbsolutePose, "gps/absolute_pose", self._pose, 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(SensorValue, "bridge/sensor_value", self._sensor, 100)
        self.node.create_subscription(String, "gnss/gga", lambda m: None, 10)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Log, "/rosout", self._log, rosout_qos)
        self.running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg
            self.statuses.append(msg)

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _publish(self, msg):
        if msg.topic == "events/json":
            with self.lock:
                self.events.append(json.loads(msg.payload))

    def _sensor(self, msg):
        with self.lock:
            self.sensors[msg.sensor_id] = msg

    def _log(self, msg):
        with self.lock:
            self.logs.append(msg.msg)

    def wait(self, pred, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = pred(self)
            if value:
                return value
            time.sleep(0.05)
        return None

    def corrections_events(self, since=0):
        with self.lock:
            return [e for e in self.events[since:] if e["type"] == "CORRECTIONS"]

    def stop(self):
        self.running = False
        time.sleep(0.2)
        self.node.destroy_node()


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
        return self.probe.wait(lambda p: p.status and p.status.state == "connected" and p.status.frames > 0 and
                               0 <= p.status.last_frame_age < 2.0, timeout)

    def test_1_ntrip2_gives_rtk_fixed(self):
        p = self.probe
        self.assertTrue(self.connected_with_frames(), f"status: {p.status}")
        self.assertTrue(p.wait(lambda p: p.pose and p.pose.flags == FIXED, 20), f"pose flags {p.pose and p.pose.flags}")
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
        self.assertTrue(p.wait(lambda p: p.sensors.get("os_ntrip_state") and
                               p.sensors["os_ntrip_state"].text == "connected"))
        self.assertTrue(p.wait(lambda p: p.sensors.get("os_correction_age") and
                               p.sensors["os_correction_age"].value < 2.0))
        self.assertEqual(p.corrections_events(), [])

    def test_2_outage_float_lost_then_restored(self):
        p = self.probe
        since = len(p.events)
        board_since = len(board_log())
        with CASTER.lock:
            CASTER.down = True
        start = time.time()
        # Past the age limit the fix counts as float, although the receiver still says fixed.
        self.assertTrue(p.wait(lambda p: p.pose and p.pose.flags == FLOAT, MAX_AGE + 6), "still fixed")
        lost = p.wait(lambda p: [e for e in p.corrections_events(since) if e["reason"] == "LOST"], LOST_AFTER + 6)
        self.assertTrue(lost, p.corrections_events(since))
        self.assertEqual((lost[0]["severity"], lost[0]["resume"], lost[0]["limit"]), ("warning", "auto", LOST_AFTER))
        # Without RTCM the keep-alive stops after its window (10 s) and the board falls back to LoRa 6 s later.
        self.assertTrue(p.wait(lambda p: any(e.get("board") == "off" for e in board_log()[board_since:]), 25),
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
        self.assertTrue(p.wait(lambda p: p.pose and p.pose.flags == FIXED, 15), "no fix after the outage")
        events = [e["reason"] for e in p.corrections_events(since)]
        self.assertEqual(events, ["LOST", "RESTORED"])
        # The board came back on with the next RTCM, without a reset (it fell back while nothing was wanted).
        self.assertFalse(any(e.get("board") == "reset" for e in board_log()))

    def test_3_refused_login_waits_and_reports_once(self):
        p = self.probe
        since = len(p.events)
        write_access(password=WRONG)
        rejected = p.wait(lambda p: p.status and p.status.state == "rejected", 10)
        self.assertTrue(rejected, f"status: {p.status}")
        self.assertEqual(p.status.last_error, "login refused (401)")
        count = len(CASTER.attempts())
        time.sleep(5.0)
        # No new attempt within rejected_retry (8 s here, 300 s on the Tron).
        self.assertEqual(len(CASTER.attempts()), count)
        events = p.corrections_events(since)
        self.assertEqual([e["reason"] for e in events], ["REJECTED"], events)
        self.assertEqual((events[0]["severity"], events[0]["resume"], events[0]["code"]), ("error", "user", 401))
        self.assertTrue(p.wait(lambda p: p.sensors["os_ntrip_state"].text == "rejected"))
        # The right password: the changed file ends the wait at once.
        write_access()
        self.assertTrue(self.connected_with_frames(6), f"status: {p.status}")

    def test_4_ntrip1(self):
        p = self.probe
        write_access(mountpoint="V1", version=1)
        self.assertTrue(p.wait(lambda p: any(a[1] == "V1" and a[3] for a in CASTER.attempts()), 10))
        self.assertTrue(self.connected_with_frames(10), f"status: {p.status}")
        v1 = [r for r in CASTER.requests if r.startswith("GET /V1 ")][-1]
        self.assertIn("HTTP/1.0", v1.split("\r\n")[0])
        self.assertNotIn("Ntrip-Version", v1)
        self.assertTrue(p.wait(lambda p: p.pose and p.pose.flags == FIXED, 15))

    def test_5_without_access_file_off(self):
        p = self.probe
        os.remove(ACCESS)
        self.assertTrue(p.wait(lambda p: p.status and p.status.state == "off", 10), f"status: {p.status}")
        self.assertTrue(p.wait(lambda p: p.sensors["os_ntrip_state"].text == "off"))

    def test_6_access_data_appear_nowhere(self):
        p = self.probe
        with p.lock:
            texts = list(p.logs) + [json.dumps(e) for e in p.events] + [s.last_error for s in p.statuses]
        token = base64.b64encode(f"{USER}:{PASSWORD}".encode()).decode()
        for secret in (PASSWORD, WRONG, token):
            self.assertFalse([t for t in texts if secret in t], secret)
