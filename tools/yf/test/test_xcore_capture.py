"""Unit tests of xcore_capture on captures built here: file formats, the board's codecs, its clock and the analysis.

Run: python3 -m unittest discover -s tools/yf/test
"""
import json
import math
import os
import socket
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import xcore_capture as xc  # noqa: E402

BOARD = ("172.16.78.150", 49154)
HOST = ("172.16.78.1", 40000)
STOP, LIFT, LIFT_MULTIPLE, COLLISION, LATCH, HIGH_LEVEL = 4, 8, 16, 32, 1, 128
SERVICES = {"1": {"type": "EmergencyService", "version": 2, "inputs": [{"id": 0, "name": "e", "type": "uint16_t[2]"}],
                  "outputs": [{"id": 0, "name": "Emergency Reason", "type": "uint16_t"}],
                  "enums": [{"id": "EmergencyReason", "base_type": "uint16_t", "bitmask": True,
                             "values": {name: bit for name, bit in xc.REASON_BITS.items()}}]},
            "2": {"type": "DiffDriveService", "version": 1,
                  "inputs": [{"id": 0, "name": "Control Twist", "type": "double[6]"}],
                  "outputs": [{"id": 7, "name": "Wheel Ticks", "type": "uint32_t[2]"}]},
            "4": {"type": "ImuService", "version": 1, "outputs": [{"id": 0, "name": "Axes", "type": "double[9]"}]},
            "7": {"type": "InputService", "version": 1,
                  "outputs": [{"id": 0, "name": "Active Inputs", "type": "uint64_t"},
                              {"id": 1, "name": "Input Event", "type": "uint8_t[2]"}],
                  "registers": [{"id": 0, "name": "Input Configs", "type": "blob"}]}}


def cbor(value):
    """Small CBOR encoder for the tests: maps, arrays, text, unsigned integers, booleans."""
    def head(major, n):
        if n < 24:
            return bytes([major << 5 | n])
        if n < 256:
            return bytes([major << 5 | 24, n])
        return bytes([major << 5 | 25]) + n.to_bytes(2, "big")
    if value is True:
        return b"\xf5"
    if isinstance(value, int):
        return head(0, value)
    if isinstance(value, str):
        return head(3, len(value.encode())) + value.encode()
    if isinstance(value, list):
        return head(4, len(value)) + b"".join(cbor(v) for v in value)
    return head(5, len(value)) + b"".join(cbor(k) + cbor(v) for k, v in value.items())


def message(msg_type, sid, payload, stamp_us=0, arg1=0, arg2=0):
    return struct.pack("<BBBBHBBHHQI", 1, msg_type, 1, 0, sid, arg1, 0, arg2, 0, stamp_us, len(payload)) + payload


def transaction(sid, stamp_us, *values, arg1=0):
    body = b"".join(struct.pack("<HHI", cid, 0, len(raw)) + raw for cid, raw in values)
    return message(xc.TRANSACTION, sid, body, stamp_us, arg1=arg1)


def ipv4_udp(source, target, data):
    udp = struct.pack(">HHHH", source[1], target[1], 8 + len(data), 0) + data
    return struct.pack(">BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), 0, 0x4000, 64, 17, 0,
                       socket.inet_aton(source[0]), socket.inet_aton(target[0])) + udp


class Writer:
    """pcap in either byte order, µs or ns, with Ethernet, Linux cooked v1 or v2 frames."""

    def __init__(self, path, link=1, order="<", nanos=False, snap=65535):
        self.f = open(path, "wb")
        self.link, self.order, self.nanos, self.snap = link, order, nanos, snap
        magic = 0xA1B23C4D if nanos else 0xA1B2C3D4
        self.f.write(struct.pack(order + "IHHiIII", magic, 2, 4, 0, 0, snap, link))

    def add(self, t, source, target, data):
        ip = ipv4_udp(source, target, data)
        if self.link == 1:
            frame = bytes(6) + bytes(6) + b"\x81\x00\x00\x05\x08\x00" + ip
        elif self.link == 113:
            frame = struct.pack(">HHH8sH", 0, 1, 6, bytes(8), 0x0800) + ip
        else:
            frame = struct.pack(">HHIHBB8s", 0x0800, 0, 2, 1, 0, 6, bytes(8)) + ip
        captured = frame[:self.snap]
        fraction = round((t % 1) * (1e9 if self.nanos else 1e6))
        self.f.write(struct.pack(self.order + "IIII", int(t), fraction, len(captured), len(frame)) + captured)

    def close(self, cut=0):
        self.f.close()
        if cut:
            with open(self.f.name, "r+b") as f:
                f.truncate(os.path.getsize(self.f.name) - cut)


def announcement(stamp_us=0):
    payload = cbor({"sid": 1, "endpoint": {"ip": BOARD[0], "port": BOARD[1]}, "desc": SERVICES["1"]})
    return message(xc.ANNOUNCE, 0, payload, stamp_us)


class Session:
    """Events of one made-up session as (capture time, direction, datagram)."""

    def __init__(self, t0=1000.0, board0=100.0):
        self.t0, self.board0 = t0, board0
        self.items = [(t0, "board", announcement(int(board0 * 1e6)))]

    def board(self, board_t, data_of_stamp):
        self.items.append((self.t0 + board_t - self.board0 + 0.0003, "board",
                           data_of_stamp(int(board_t * 1e6) & 0xFFFFFFFF)))

    def host(self, capture_t, data):
        self.items.append((capture_t, "host", data))

    def reason(self, board_t, bits):
        self.board(board_t, lambda s: transaction(1, s, (0, struct.pack("<H", bits))))

    def mask(self, board_t, mask):
        self.board(board_t, lambda s: message(xc.DATA, 7, struct.pack("<Q", mask), s))

    def event(self, board_t, index, kind):
        self.board(board_t, lambda s: message(xc.DATA, 7, bytes([index, kind]), s, arg2=1))

    def imu(self, board_t, acc, rate):
        self.board(board_t, lambda s: message(xc.DATA, 4, struct.pack("<9d", *acc, *rate, 0, 0, 0), s))

    def write(self, path, **kwargs):
        writer = Writer(path, **kwargs)
        for t, direction, data in sorted(self.items, key=lambda item: item[0]):
            source, target = (BOARD, HOST) if direction == "board" else (HOST, BOARD)
            if data[1] == xc.ANNOUNCE:
                target = ("233.255.255.0", 4242)
            writer.add(t, source, target, data)
        return writer


def stop_session(with_events=False):
    s = Session()
    for k in range(0, 30):
        t = 100.0 + 0.2 * k
        mask = 1 if 100.2 <= t <= 101.0 else 0
        s.mask(t, mask)
    s.reason(100.05, 0)
    s.reason(100.15, STOP | LATCH)
    s.reason(101.1, LATCH)
    s.reason(102.5, 0)
    s.host(1002.4, struct.pack("<BBBBHBBHHQI", 1, xc.DATA, 0, 0, 1, 0, 0, 0, 0, 0, 4) + struct.pack("<HH", 0, LATCH))
    if with_events:
        s.event(100.14, 0, 0)
        s.event(101.1, 0, 1)
    return s


class Codecs(unittest.TestCase):
    def test_cbor(self):
        self.assertEqual(xc.cbor_decode(bytes.fromhex("1903e8")), 1000)
        self.assertEqual(xc.cbor_decode(bytes.fromhex("3863")), -100)
        self.assertEqual(xc.cbor_decode(bytes.fromhex("a201020304")), {1: 2, 3: 4})
        self.assertEqual(xc.cbor_decode(bytes.fromhex("9f0102ff")), [1, 2])
        self.assertEqual(xc.cbor_decode(bytes.fromhex("f93c00")), 1.0)
        self.assertEqual(xc.cbor_decode(bytes.fromhex("63616263")), "abc")
        self.assertEqual(xc.cbor_decode(cbor({"endpoint": {"ip": "1.2.3.4", "port": 49154}}))["endpoint"]["port"],
                         49154)
        with self.assertRaises(xc.CborError):
            xc.cbor_decode(bytes.fromhex("6361"))

    def test_heatshrink_literals_and_back_reference(self):
        bits = "1" + f"{ord('a'):08b}" + "1" + f"{ord('b'):08b}" + "0" + f"{1:09b}" + f"{3:05b}"
        bits += "0" * (-len(bits) % 8)
        stream = int(bits, 2).to_bytes(len(bits) // 8, "big")
        self.assertEqual(xc.heatshrink_expand(stream), b"ababab")

    def test_board_clock_unwraps(self):
        boot = xc.Boot(0, BOARD, 0.0)
        self.assertAlmostEqual(boot.board_time(0xFFFFFF00, 1.0), 0xFFFFFF00 / 1e6)
        self.assertAlmostEqual(boot.board_time(0x100, 1.1), (2 ** 32 + 0x100) / 1e6)
        self.assertAlmostEqual(boot.board_time(0x80, 1.1), (2 ** 32 + 0x80) / 1e6)
        self.assertAlmostEqual(boot.estimate(1.2), (2 ** 32 + 0x80) / 1e6 + 0.1)


class Files(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="xcore_capture_")
        self.services = os.path.join(self.dir, "services.json")
        with open(self.services, "w") as f:
            json.dump({"services": SERVICES}, f)

    def report(self, path, inputs=None):
        cap = xc.load_capture(path, self.services)
        inputs = (inputs or xc.NX60I_INPUTS, "test")
        return xc.analyse(cap, inputs, {"driver": (40.0, 0.5), "safety": (31.0, 1.0)}, 15.0, 0.1)

    def test_formats_give_the_same_press(self):
        for name, kwargs in [("eth_le_us", {}), ("eth_be_ns", {"order": ">", "nanos": True}),
                             ("sll", {"link": 113}), ("sll2", {"link": 276})]:
            with self.subTest(name):
                path = os.path.join(self.dir, name + ".pcap")
                stop_session().write(path, **kwargs).close()
                report = self.report(path)
                self.assertEqual(report["capture"]["board_found_by"], "announcements")
                press = report["boots"][0]["presses"]
                self.assertEqual(len(press), 1)
                self.assertEqual(press[0]["reason"], "STOP")
                self.assertAlmostEqual(press[0]["delay_min_ms"], -50, delta=0.01)
                self.assertAlmostEqual(press[0]["delay_max_ms"], 150, delta=0.01)
                self.assertTrue(press[0]["latched"])
                self.assertFalse(press[0]["reason_after_release"])
                self.assertAlmostEqual(press[0]["latch_gone_at"], 102.5)
                self.assertAlmostEqual(press[0]["host_clear_at"], 102.4, delta=0.01)

    def test_input_events_give_exact_edges(self):
        path = os.path.join(self.dir, "events.pcap")
        stop_session(with_events=True).write(path).close()
        boot = self.report(path)["boots"][0]
        self.assertEqual(boot["input_event_layout"], 0)
        press = boot["presses"][0]
        self.assertTrue(press["edge_exact"])
        self.assertAlmostEqual(press["delay_min_ms"], 10, delta=0.01)
        self.assertAlmostEqual(press["held_s"], 0.96, delta=1e-6)

    def test_cut_record_and_snap_length(self):
        path = os.path.join(self.dir, "cut.pcap")
        stop_session().write(path).close(cut=5)
        report = self.report(path)
        self.assertTrue(report["capture"]["cut_at_end"])
        path = os.path.join(self.dir, "snap.pcap")
        stop_session().write(path, snap=60).close()
        with self.assertRaises(xc.CaptureError):
            self.report(path)

    def test_pcapng_is_refused(self):
        path = os.path.join(self.dir, "x.pcapng")
        with open(path, "wb") as f:
            f.write(b"\x0a\x0d\x0d\x0a" + bytes(28))
        with self.assertRaises(xc.CaptureError):
            self.report(path)

    def test_short_press_and_lift_both(self):
        s = Session()
        masks = {100.0: 0, 100.2: 4, 101.0: 4, 101.2: 0, 102.0: 12, 102.4: 12, 102.6: 0}
        for t, mask in masks.items():
            s.mask(t, mask)
        s.reason(100.0, 0)
        s.reason(102.01, LIFT_MULTIPLE | LATCH)
        path = os.path.join(self.dir, "lift.pcap")
        s.write(path).close()
        presses = self.report(path)["boots"][0]["presses"]
        self.assertEqual([p["input"] for p in presses],
                         ["lift front left (GPIO10)", "lift front left (GPIO10)", "lift front right (GPIO11)"])
        self.assertIsNone(presses[0]["reason"])
        self.assertAlmostEqual(presses[0]["held_s"], 1.0)
        self.assertEqual(presses[1]["reason"], "LIFT_MULTIPLE")
        self.assertEqual(presses[2]["reason"], "LIFT_MULTIPLE")
        self.assertAlmostEqual(presses[1]["delay_min_ms"], 10, delta=0.01)
        self.assertAlmostEqual(presses[1]["delay_max_ms"], 810, delta=0.01)

    def test_heartbeats_find_the_board_without_announcements(self):
        s = Session()
        s.items = []
        s.board(100.0, lambda st: message(xc.HEARTBEAT, 1, b"", st))
        s.reason(100.1, STOP)
        path = os.path.join(self.dir, "hb.pcap")
        s.write(path).close()
        report = self.report(path)
        self.assertEqual(report["capture"]["board_found_by"], "heartbeats")
        self.assertEqual(report["boots"][0]["reasons"][0]["names"], "STOP")

    def test_imu_rest_tilt_and_yaw(self):
        s = Session()
        t = 100.0
        while t < 106.0:
            if 101.0 <= t < 102.2:
                angle = math.radians(45)
                acc = (0.0, xc.GRAVITY * math.sin(angle), xc.GRAVITY * math.cos(angle))
            elif 103.0 <= t < 103.7:
                angle = math.radians(35)
                acc = (-xc.GRAVITY * math.sin(angle), 0.0, xc.GRAVITY * math.cos(angle))
            else:
                acc = (0.0, 0.0, xc.GRAVITY)
            rate = (0.0, 0.0, 0.5 if 104.5 <= t < 105.5 else 0.0)
            s.imu(t, acc, rate)
            t = round(t + 0.01, 6)
        path = os.path.join(self.dir, "imu.pcap")
        s.write(path).close()
        imu = self.report(path)["boots"][0]["imu"]
        self.assertAlmostEqual(imu["rate_hz"], 100.0, delta=0.5)
        self.assertAlmostEqual(imu["rest"]["a_z"], xc.GRAVITY, delta=1e-6)
        first, second = imu["tilt_episodes"]
        self.assertAlmostEqual(first["max_tilt"], 45, delta=0.01)
        self.assertAlmostEqual(first["roll_at_max"], 45, delta=0.01)
        self.assertAlmostEqual(first["checks"]["driver"]["after_crossing_s"], 0.5, delta=0.011)
        self.assertAlmostEqual(first["checks"]["safety"]["after_crossing_s"], 1.0, delta=0.011)
        self.assertAlmostEqual(second["pitch_at_max"], 35, delta=0.01)
        self.assertIsNone(second["checks"]["driver"]["latched_at"])
        self.assertIsNone(second["checks"]["safety"]["latched_at"])
        yaw, = imu["yaw_episodes"]
        self.assertEqual(yaw["sign"], "+")
        self.assertAlmostEqual(yaw["angle_deg"], math.degrees(0.5 * 0.99), delta=0.6)

    def test_wheel_runs_and_full_duty_speed(self):
        s = Session()
        tpm = 2025.0

        def twist(board_t, v):
            s.host(s.t0 + board_t - s.board0, message(xc.DATA, 2, struct.pack("<6d", v, 0, 0, 0, 0, 0), arg2=0))
        left = right = 2 ** 32 - 500.0
        t = 100.0
        while t < 116.0:
            speed = 0.0
            if 100.2 <= t < 104.0:
                speed = 0.09
            elif 106.0 <= t < 106.5:
                speed = 0.05
            elif 110.0 <= t < 112.04:
                speed = 0.045
            left += speed * tpm * 0.04
            right -= speed * tpm * 0.04
            s.board(t, lambda st, l=int(left) % 2 ** 32, r=int(right) % 2 ** 32:
                    transaction(2, st, (7, struct.pack("<2I", l, r))))
            t = round(t + 0.04, 6)
        for k in range(30):
            twist(100.1 + 0.1 * k, 0.1)
        for k in range(20):
            twist(110.0 + 0.1 * k, 0.05)
        for k in range(3):
            twist(112.0 + 0.1 * k, 0.0)
        path = os.path.join(self.dir, "drive.pcap")
        s.write(path).close()
        drive = self.report(path)["boots"][0]["drive"]
        first, second, third = drive["runs"]
        self.assertTrue(first["fresh_command"])
        self.assertAlmostEqual(first["host_silent_s"], 1.0, delta=0.05)
        self.assertAlmostEqual(first["since_last_command_s"], 1.0, delta=0.05)
        self.assertIsNone(first["since_zero_command_s"])
        self.assertFalse(second["fresh_command"])
        self.assertTrue(third["fresh_command"])
        self.assertAlmostEqual(third["since_zero_command_s"], 0.04, delta=0.045)
        self.assertEqual(third["zeros_after_last_command"], 3)
        steady = drive["steady"][0]
        self.assertAlmostEqual(steady["per_duty_left"], 0.9, delta=0.02)
        self.assertAlmostEqual(steady["per_duty_right"], 0.9, delta=0.02)
        self.assertIn("WARNING", xc.text_report(self.report(path)))

    def test_text_report_renders(self):
        path = os.path.join(self.dir, "text.pcap")
        stop_session(with_events=True).write(path).close()
        text = xc.text_report(self.report(path))
        self.assertIn("stop top (GPIO1)", text)
        self.assertIn("LATCH, host cleared at", text)


if __name__ == "__main__":
    unittest.main()
