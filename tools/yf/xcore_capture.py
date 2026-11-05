#!/usr/bin/env python3
"""Report on a capture of the inner network of an OpenMower v2 mower: the xCore board on UDP and its host.

Reads a pcap (tcpdump -w, any byte order, µs or ns, Ethernet, Linux cooked v1/v2 or raw IP), finds the board by its
announcements (or its heartbeats), decodes what the board sends with its own clock (µs since its start, 32 bits) and
what the host sends, and reports:
  - every run of the wheels (from the ticks): how it started (with a fresh command or not) and how long after its
    cause it stood (the host falling silent, a command of 0, an emergency reason), the zeros the host sent after its
    last command, per steady command the wheel speed per unit of duty, and from all steady commands the speed model
    of each wheel (duty_slope and duty_deadband: speed = slope x (duty - deadband));
  - every run of the blade (from its rpm) with the same causes;
  - every press of an input: the delay from the input mask (or the input event) to the emergency report that carries
    the matching reason, in board time; whether the reason stays latched after the release, and who cleared it;
  - the emergency reasons over time;
  - the IMU: rate, values at rest (a_z), tilt from gravity like the driver's check (angle between the body's z axis
    and gravity), episodes of tilt with what the driver (tilt_limit for tilt_time) and the safety layer would have
    latched, episodes of yaw rate with their sign.
Passive: the program reads a file and never opens a socket.
"""
import argparse
import bisect
import json
import math
import os
import struct
import sys
from collections import Counter

HEADER = struct.Struct("<BBBBHBBHHQI")
DATA, CONFIG_REQUEST, CLAIM, HEARTBEAT, TRANSACTION, RPC_REQUEST, RPC_RESPONSE, LOG, ANNOUNCE = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x7F, 0x80)
TYPE_NAMES = {DATA: "data", CONFIG_REQUEST: "config request", CLAIM: "claim", HEARTBEAT: "heartbeat",
              TRANSACTION: "transaction", RPC_REQUEST: "rpc", RPC_RESPONSE: "rpc response", LOG: "log",
              ANNOUNCE: "announcement"}
EMERGENCY, DIFF_DRIVE, MOWER, IMU, POWER, GPS, INPUT = 1, 2, 3, 4, 5, 6, 7
ANNOUNCE_PORT = 4242
# Ports of the board that must never get traffic from a host of MowBite OS (boot loader, raw passthroughs).
DANGEROUS_PORTS = {8007, 10000, 65102, 65103, 65104}

REASON_BITS = {"LATCH": 0, "TIMEOUT_INPUTS": 1, "STOP": 2, "LIFT": 3, "LIFT_MULTIPLE": 4, "COLLISION": 5,
               "TIMEOUT_HIGH_LEVEL": 6, "HIGH_LEVEL": 7, "SERVICE_NOT_READY": 8, "COLLISION_MULTIPLE": 9,
               "MOWER_RPM_TIMEOUT": 10, "MOWER_RPM_LIMIT": 11, "FIRMWARE_INCOMPATIBLE": 12}
LATCH = 1 << REASON_BITS["LATCH"]
# Reasons an input causes, by its emergency reason in the input configuration.
REASONS_OF_INPUT = {
    "stop": (1 << REASON_BITS["STOP"],),
    "lift": (1 << REASON_BITS["LIFT"], 1 << REASON_BITS["LIFT_MULTIPLE"]),
    "collision": (1 << REASON_BITS["COLLISION"], 1 << REASON_BITS["COLLISION_MULTIPLE"]),
}
INPUT_REASON_MASK = sum(b for bits in REASONS_OF_INPUT.values() for b in bits)

# Inputs of the NX60i in the order of xcore_driver's nx60i.yaml; a capture with the input configuration replaces it.
NX60I_INPUTS = [
    {"name": "stop top (GPIO1)", "reason": "stop", "delay_ms": 10},
    {"name": "stop rear (GPIO5)", "reason": "stop", "delay_ms": 10},
    {"name": "lift front left (GPIO10)", "reason": "lift", "delay_ms": 2500},
    {"name": "lift front right (GPIO11)", "reason": "lift", "delay_ms": 2500},
    {"name": "bumper rear right (GPIO12)", "reason": "collision", "delay_ms": 500},
    {"name": "bumper rear left (GPIO13)", "reason": "collision", "delay_ms": 500},
    {"name": "cover (hall_mux=om)", "reason": None, "delay_ms": None},
]
DEFAULT_SERVICES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "mowbite_sim",
                                "config", "xcore_nx60i_services.json")
GRAVITY = 9.80665
# The NX60i: ticks per m and wheel distance as configured, the right counter falls going forward.
DRIVE_DEFAULTS = {"ticks_per_meter": 2025.0, "wheel_distance": 0.3556, "right_sign": -1}
RELEASE_SETTLE = 0.25
HOST_GAP = 0.3
# Single ticks after the wheels stood, at most this many intervals with them, count as settling back, not as a run.
SETTLE_INTERVALS = 2
# Wheel reports come every 40 ms; a longer silence is a gap in the capture.
TICK_GAP = 0.3


# Capture file ---------------------------------------------------------------------------------------------------------

class CaptureError(Exception):
    pass


PCAP_MAGICS = {b"\xd4\xc3\xb2\xa1": ("<", 1e-6), b"\xa1\xb2\xc3\xd4": (">", 1e-6),
               b"\x4d\x3c\xb2\xa1": ("<", 1e-9), b"\xa1\xb2\x3c\x4d": (">", 1e-9)}


class Packet:
    __slots__ = ("t", "proto", "src", "sport", "dst", "dport", "payload", "truncated")

    def __init__(self, t, proto, src, sport, dst, dport, payload, truncated):
        self.t, self.proto, self.src, self.sport, self.dst, self.dport = t, proto, src, sport, dst, dport
        self.payload, self.truncated = payload, truncated


def ip_text(raw):
    return ".".join(str(b) for b in raw)


def ipv4_offset(linktype, frame, order):
    """Offset of the IPv4 header in a frame of this link type, or None."""
    if linktype == 1:
        at, ethertype = 14, struct.unpack_from(">H", frame, 12)[0] if len(frame) >= 14 else 0
        while ethertype in (0x8100, 0x88A8) and len(frame) >= at + 4:
            ethertype = struct.unpack_from(">H", frame, at + 2)[0]
            at += 4
        return at if ethertype == 0x0800 else None
    if linktype == 113:
        return 16 if len(frame) >= 16 and struct.unpack_from(">H", frame, 14)[0] == 0x0800 else None
    if linktype == 276:
        return 20 if len(frame) >= 20 and struct.unpack_from(">H", frame, 0)[0] == 0x0800 else None
    if linktype in (101, 12, 228):
        return 0
    if linktype == 0:
        return 4 if len(frame) >= 4 and struct.unpack_from(order + "I", frame, 0)[0] == 2 else None
    raise CaptureError(f"link type {linktype} is not supported (Ethernet, Linux cooked or raw IP)")


def parse_frame(t, linktype, frame, wire_length, order):
    at = ipv4_offset(linktype, frame, order)
    if at is None or len(frame) < at + 20 or frame[at] >> 4 != 4:
        return None
    ihl = (frame[at] & 0x0F) * 4
    total = struct.unpack_from(">H", frame, at + 2)[0]
    fragment = struct.unpack_from(">H", frame, at + 6)[0]
    proto = frame[at + 9]
    if fragment & 0x3FFF or proto not in (6, 17):
        return None
    src, dst = ip_text(frame[at + 12:at + 16]), ip_text(frame[at + 16:at + 20])
    l4 = at + ihl
    if len(frame) < l4 + 4:
        return None
    sport, dport = struct.unpack_from(">HH", frame, l4)
    if proto == 6:
        return Packet(t, "tcp", src, sport, dst, dport, b"", False)
    length = struct.unpack_from(">H", frame, l4 + 4)[0] if len(frame) >= l4 + 6 else 8
    end = min(l4 + max(length, 8), at + total)
    payload = frame[l4 + 8:end]
    truncated = wire_length > len(frame) and len(payload) < max(length, 8) - 8
    return Packet(t, "udp", src, sport, dst, dport, payload, truncated)


def read_packets(path, stats):
    """IPv4 UDP and TCP packets of a pcap file in file order; a record cut off at the end is counted, not an error."""
    with open(path, "rb") as f:
        head = f.read(24)
        if len(head) < 24:
            raise CaptureError("not a pcap file (shorter than its header)")
        if head[:4] == b"\x0a\x0d\x0d\x0a":
            raise CaptureError("pcapng is not supported: write the capture with tcpdump -w (pcap)")
        if head[:4] not in PCAP_MAGICS:
            raise CaptureError("not a pcap file (unknown magic)")
        order, unit = PCAP_MAGICS[head[:4]]
        linktype = struct.unpack_from(order + "I", head, 20)[0] & 0x0FFFFFFF
        stats["linktype"] = linktype
        record = struct.Struct(order + "IIII")
        while True:
            raw = f.read(16)
            if not raw:
                break
            if len(raw) < 16:
                stats["cut_at_end"] = True
                break
            seconds, fraction, caplen, wire_length = record.unpack(raw)
            frame = f.read(caplen)
            if len(frame) < caplen:
                stats["cut_at_end"] = True
                break
            stats["frames"] += 1
            packet = parse_frame(seconds + fraction * unit, linktype, frame, wire_length, order)
            if packet is not None:
                yield packet


# Codecs of the board --------------------------------------------------------------------------------------------------

class CborError(Exception):
    pass


def cbor_decode(data):
    """Decodes one CBOR item (RFC 8949): integers, byte and text strings, arrays, maps, simple values, floats."""
    value, at = _cbor_item(data, 0, 0)
    return value


def _cbor_length(data, at, info):
    if info < 24:
        return info, at
    sizes = {24: 1, 25: 2, 26: 4, 27: 8}
    if info not in sizes:
        raise CborError("bad length")
    size = sizes[info]
    if at + size > len(data):
        raise CborError("cut off")
    return int.from_bytes(data[at:at + size], "big"), at + size


def _cbor_item(data, at, depth):
    if depth > 32:
        raise CborError("nested too deep")
    if at >= len(data):
        raise CborError("cut off")
    major, info = data[at] >> 5, data[at] & 0x1F
    at += 1
    if major == 7:
        if info == 20:
            return False, at
        if info == 21:
            return True, at
        if info in (22, 23):
            return None, at
        fmt = {25: ">e", 26: ">f", 27: ">d"}.get(info)
        if fmt is None:
            raise CborError("unsupported simple value")
        size = struct.calcsize(fmt)
        if at + size > len(data):
            raise CborError("cut off")
        return struct.unpack_from(fmt, data, at)[0], at + size
    if info == 31 and major in (2, 3, 4, 5):
        items = []
        while True:
            if at >= len(data):
                raise CborError("cut off")
            if data[at] == 0xFF:
                at += 1
                break
            item, at = _cbor_item(data, at, depth + 1)
            items.append(item)
        if major == 2:
            return b"".join(items), at
        if major == 3:
            return "".join(items), at
        if major == 4:
            return items, at
        if len(items) % 2:
            raise CborError("map with an odd count")
        return dict(zip(items[0::2], items[1::2])), at
    n, at = _cbor_length(data, at, info)
    if major == 0:
        return n, at
    if major == 1:
        return -1 - n, at
    if major in (2, 3):
        if at + n > len(data):
            raise CborError("cut off")
        raw = bytes(data[at:at + n])
        return (raw if major == 2 else raw.decode("utf-8", "replace")), at + n
    if major == 4:
        items = []
        for _ in range(n):
            item, at = _cbor_item(data, at, depth + 1)
            items.append(item)
        return items, at
    if major == 5:
        out = {}
        for _ in range(n):
            key, at = _cbor_item(data, at, depth + 1)
            out[key if not isinstance(key, list) else str(key)], at = _cbor_item(data, at, depth + 1)
        return out, at
    if major == 6:
        return _cbor_item(data, at, depth + 1)
    raise CborError("unknown major type")


def heatshrink_expand(stream, window_bits=9, lookahead_bits=5):
    """Expands a heatshrink stream: flag 1 and a byte, or flag 0, back index and count, MSB first."""
    out = bytearray()
    bit = 0
    total = len(stream) * 8

    def take(n):
        nonlocal bit
        value = 0
        for _ in range(n):
            value = value << 1 | (stream[bit >> 3] >> (7 - (bit & 7))) & 1
            bit += 1
        return value

    while bit < total:
        if take(1):
            if bit + 8 > total:
                break
            out.append(take(8))
            continue
        if bit + window_bits + lookahead_bits > total:
            break
        back = take(window_bits) + 1
        count = take(lookahead_bits) + 1
        if back > len(out):
            raise ValueError("back reference before the start")
        for _ in range(count):
            out.append(out[-back])
    return bytes(out)


SCALARS = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h", "uint32_t": "I", "int32_t": "i",
           "uint64_t": "Q", "int64_t": "q", "float": "f", "double": "d", "char": "c"}


class Channel:
    def __init__(self, cid, name, type_text, enums):
        self.id, self.name = cid, name
        base, self.array = type_text, False
        if base.endswith("]"):
            base = base[:base.index("[")]
            self.array = True
        base = enums.get(base, base)
        self.blob = base == "blob"
        self.text = base == "char"
        self.fmt = SCALARS.get(base)

    def decode(self, raw):
        if self.blob:
            return bytes(raw)
        if self.text:
            return bytes(raw).decode("ascii", "replace")
        if self.fmt is None:
            return None
        size = struct.calcsize(self.fmt)
        if not self.array:
            return struct.unpack("<" + self.fmt, raw)[0] if len(raw) == size else None
        if len(raw) % size:
            return None
        return list(struct.unpack(f"<{len(raw) // size}{self.fmt}", raw))


class Description:
    def __init__(self, desc):
        self.type = desc.get("type")
        enums = {e["id"]: e["base_type"] for e in desc.get("enums") or [] if "id" in e and "base_type" in e}
        self.reason_bits = {}
        for e in desc.get("enums") or []:
            if e.get("bitmask") and isinstance(e.get("values"), dict):
                self.reason_bits = {name: int(bit) for name, bit in e["values"].items()}

        def channels(kind):
            return {c["id"]: Channel(c["id"], c.get("name", ""), c["type"], enums) for c in desc.get(kind) or []}
        self.inputs, self.outputs, self.registers = channels("inputs"), channels("outputs"), channels("registers")


def sections(payload):
    out, at = [], 0
    while at + 8 <= len(payload):
        cid, _, length = struct.unpack_from("<HHI", payload, at)
        at += 8
        if at + length > len(payload):
            return None
        out.append((cid, payload[at:at + length]))
        at += length
    return out if at == len(payload) else None


# Reading a capture ----------------------------------------------------------------------------------------------------

class Boot:
    """One run of the board between restarts: its endpoint and an unwrapped clock."""

    def __init__(self, index, endpoint, first_seen):
        self.index, self.endpoint, self.first_seen = index, endpoint, first_seen
        self.last_raw = None
        self.wraps = 0
        self.last_pair = None
        self.pairs = []

    def board_time(self, raw, capture_t):
        raw &= 0xFFFFFFFF
        if self.last_raw is not None and raw < self.last_raw and self.last_raw - raw > 1 << 31:
            self.wraps += 1
        self.last_raw = raw
        t = (self.wraps * (1 << 32) + raw) / 1e6
        self.last_pair = (capture_t, t)
        if not self.pairs or capture_t - self.pairs[-1][0] >= 0.2:
            self.pairs.append((capture_t, t))
        return t

    def at_capture_time(self, capture_t):
        """Board time of any moment of the capture clock (the host's wall clock) while this run lasts."""
        i = bisect.bisect_right([p[0] for p in self.pairs], capture_t) - 1
        if i < 0:
            return None
        return self.pairs[i][1] + capture_t - self.pairs[i][0]

    def estimate(self, capture_t):
        """Board time of a host datagram from the board datagram seen last before it (the IMU comes every 10 ms)."""
        if self.last_pair is None:
            return None
        return self.last_pair[1] + capture_t - self.last_pair[0]


class Capture:
    def __init__(self):
        self.stats = Counter(frames=0)
        self.stats_info = {}
        self.boots = []
        self.descriptions = {}
        self.emergency = []      # (boot, board_t, capture_t, reason)
        self.masks = []          # (boot, board_t, capture_t, mask)
        self.input_events = []   # (boot, board_t, capture_t, byte0, byte1)
        self.imu = []            # (boot, board_t, capture_t, values)
        self.host_emergency = []  # (boot, board_t estimate, capture_t, set bits, clear bits)
        self.ticks = []          # (boot, board_t, capture_t, left, right)
        self.blade_rpm = []      # (boot, board_t, capture_t, rpm)
        self.twists = []         # (boot, board_t estimate, capture_t, v, w)
        self.blade_set = []      # (boot, board_t estimate, capture_t, speed)
        self.host_times = []     # (boot, board_t estimate) of every datagram from the host
        self.drive_config = {}
        self.input_config = None
        self.multiple_delays = {}
        self.dangerous = Counter()
        self.counts = Counter()
        self.malformed = Counter()
        self.span = None


def scan_board(path, info):
    """First pass: announcements and heartbeats tell the board's endpoints and its service descriptions."""
    announced, heartbeats, descriptions = {}, Counter(), {}
    for p in read_packets(path, Counter(frames=0)):
        if p.proto != "udp" or len(p.payload) < HEADER.size:
            continue
        version, msg_type = p.payload[0], p.payload[1]
        if version != 1:
            continue
        if msg_type == ANNOUNCE and (p.dport == ANNOUNCE_PORT or 224 <= int(p.dst.split(".")[0]) <= 239):
            try:
                doc = cbor_decode(p.payload[HEADER.size:])
                endpoint = (doc["endpoint"]["ip"], int(doc["endpoint"]["port"]))
                sid = int(doc["sid"])
            except (CborError, KeyError, TypeError, ValueError):
                info["bad_announcements"] = info.get("bad_announcements", 0) + 1
                continue
            announced.setdefault(endpoint, p.t)
            if isinstance(doc.get("desc"), dict):
                descriptions[sid] = doc["desc"]
        elif msg_type == HEARTBEAT:
            heartbeats[(p.src, p.sport)] += 1
    if announced:
        return announced, descriptions, "announcements"
    if heartbeats:
        return {ep: None for ep in heartbeats}, descriptions, "heartbeats"
    return {}, descriptions, None


def load_capture(path, services_path):
    cap = Capture()
    endpoints, announced_desc, source = scan_board(path, cap.stats_info)
    if not endpoints:
        raise CaptureError("no board found: neither announcements (UDP 4242) nor heartbeats in the capture")
    cap.stats_info["board_found_by"] = source
    descriptions = {}
    if services_path and os.path.exists(services_path):
        with open(services_path) as f:
            descriptions = {int(k): v for k, v in json.load(f)["services"].items()}
        cap.stats_info["descriptions"] = "file " + services_path
    if announced_desc:
        descriptions.update(announced_desc)
        cap.stats_info["descriptions"] = "announcements in the capture" + (
            " and the file" if len(announced_desc) < len(descriptions) else "")
    cap.descriptions = {sid: Description(d) for sid, d in descriptions.items()}
    boots_by_endpoint = {}
    first_t = last_t = None
    for p in read_packets(path, cap.stats):
        first_t = p.t if first_t is None else first_t
        last_t = p.t
        if p.dport in DANGEROUS_PORTS or (p.sport in DANGEROUS_PORTS and p.proto == "tcp"):
            cap.dangerous[(p.proto, p.dport if p.dport in DANGEROUS_PORTS else p.sport)] += 1
        if p.proto != "udp":
            continue
        source, target = (p.src, p.sport), (p.dst, p.dport)
        if source in endpoints:
            direction, endpoint = "board", source
        elif target in endpoints:
            direction, endpoint = "host", target
        else:
            continue
        boot = boots_by_endpoint.get(endpoint)
        if boot is None:
            boot = Boot(len(cap.boots), endpoint, p.t)
            boots_by_endpoint[endpoint] = boot
            cap.boots.append(boot)
        if len(p.payload) < HEADER.size:
            cap.malformed["short"] += 1
            continue
        version, msg_type, _, _, sid, arg1, _, arg2, _, stamp, length = HEADER.unpack_from(p.payload)
        body = p.payload[HEADER.size:]
        if version != 1 or length != len(body):
            cap.malformed["cut by the snap length" if p.truncated else "bad header"] += 1
            continue
        cap.counts[(direction, sid, msg_type)] += 1
        if direction == "board":
            board_t = boot.board_time(stamp, p.t)
            if msg_type in (DATA, TRANSACTION) and not (msg_type == TRANSACTION and arg1 != 0):
                values = [(arg2, body)] if msg_type == DATA else sections(body)
                if values is None:
                    cap.malformed["bad sections"] += 1
                    continue
                record_outputs(cap, boot, sid, values, board_t, p.t)
        else:
            board_t = boot.estimate(p.t)
            if board_t is not None:
                cap.host_times.append((boot.index, board_t))
            if msg_type in (DATA, TRANSACTION):
                values = [(arg2, body)] if msg_type == DATA else sections(body)
                if values is None:
                    cap.malformed["bad sections"] += 1
                    continue
                if msg_type == TRANSACTION and arg1 == 1:
                    record_configuration(cap, sid, values)
                else:
                    record_inputs(cap, boot, sid, values, board_t, p.t)
    cap.span = (first_t, last_t)
    return cap


def record_outputs(cap, boot, sid, values, board_t, capture_t):
    desc = cap.descriptions.get(sid)
    for cid, raw in values:
        channel = desc.outputs.get(cid) if desc else None
        if channel is None:
            continue
        value = channel.decode(raw)
        if value is None:
            cap.malformed[f"service {sid} output {cid} size"] += 1
            continue
        if sid == EMERGENCY and cid == 0:
            cap.emergency.append((boot.index, board_t, capture_t, value))
        elif sid == INPUT and cid == 0:
            cap.masks.append((boot.index, board_t, capture_t, value))
        elif sid == INPUT and cid == 1 and len(value) == 2:
            cap.input_events.append((boot.index, board_t, capture_t, value[0], value[1]))
        elif sid == IMU and cid == 0 and len(value) >= 6:
            cap.imu.append((boot.index, board_t, capture_t, value))
        elif sid == DIFF_DRIVE and cid == 7 and len(value) == 2:
            cap.ticks.append((boot.index, board_t, capture_t, value[0], value[1]))
        elif sid == MOWER and cid == 6:
            cap.blade_rpm.append((boot.index, board_t, capture_t, value))


def record_inputs(cap, boot, sid, values, board_t, capture_t):
    for cid, raw in values:
        if board_t is None:
            continue
        if sid == EMERGENCY and cid == 0 and len(raw) == 4:
            set_bits, clear_bits = struct.unpack("<HH", raw)
            cap.host_emergency.append((boot.index, board_t, capture_t, set_bits, clear_bits))
        elif sid == DIFF_DRIVE and cid == 0 and len(raw) == 48:
            twist = struct.unpack("<6d", raw)
            cap.twists.append((boot.index, board_t, capture_t, twist[0], twist[5]))
        elif sid == MOWER and cid == 0 and len(raw) == 4:
            cap.blade_set.append((boot.index, board_t, capture_t, struct.unpack("<f", raw)[0]))


def record_configuration(cap, sid, values):
    if sid == DIFF_DRIVE:
        for cid, raw in values:
            if cid in (0, 1) and len(raw) == 8:
                cap.drive_config["ticks_per_meter" if cid == 0 else "wheel_distance"] = struct.unpack("<d", raw)[0]
        return
    if sid != INPUT:
        return
    for cid, raw in values:
        if cid == 0:
            try:
                doc = json.loads(heatshrink_expand(raw).decode())
            except (ValueError, UnicodeDecodeError):
                continue
            cap.input_config = inputs_from_config(doc)
        elif cid in (3, 4) and len(raw) == 4:
            cap.multiple_delays["lift" if cid == 3 else "collision"] = struct.unpack("<I", raw)[0]


def inputs_from_config(doc):
    """Inputs in the board's order (document order over all drivers) from the input configuration JSON."""
    out = []
    if not isinstance(doc, dict):
        return out
    for driver, items in doc.items():
        for item in items if isinstance(items, list) else []:
            emergency = item.get("emergency") or {}
            label = item.get("line") or item.get("channel") or item.get("id") or item.get("button_id")
            out.append({"name": f"{driver} {label}" if label is not None else driver,
                        "reason": emergency.get("reason"), "delay_ms": emergency.get("delay")})
    return out


# Analysis -------------------------------------------------------------------------------------------------------------

def reason_names(bits, names=None):
    """Names of the set bits in their order; set bits without a name add UNKNOWN once at the end."""
    names = names or REASON_BITS
    out = [name for name, bit in sorted(names.items(), key=lambda kv: kv[1]) if bits >> bit & 1]
    if bits & ~sum(1 << bit for bit in set(names.values())):
        out.append("UNKNOWN")
    return "|".join(out) if out else "0"


class Timeline:
    """The last reported value at any board time."""

    def __init__(self, samples):
        self.times = [s[0] for s in samples]
        self.values = [s[1] for s in samples]

    def at(self, t):
        i = bisect.bisect_right(self.times, t) - 1
        return self.values[i] if i >= 0 else None

    def between(self, start, end):
        """Indices of samples with start < time <= end."""
        return range(bisect.bisect_right(self.times, start), bisect.bisect_right(self.times, end))


def event_layout(masks, events):
    """Which byte of an input event is the input index: the layout whose ACTIVE events match rising mask bits."""
    if not events or not masks:
        return None
    times = [m[0] for m in masks]
    scores = {}
    for layout in (0, 1):
        hits = misses = 0
        for t, a, b in events:
            index, kind = (a, b) if layout == 0 else (b, a)
            if kind not in (0, 1) or index > 63:
                misses += 1
                continue
            i = bisect.bisect_left(times, t - 1e-6)
            if i >= len(masks):
                continue
            active = masks[i][1] >> index & 1
            hits += active == (kind == 0)
            misses += active != (kind == 0)
        scores[layout] = (hits, misses)
    best = max(scores, key=lambda k: scores[k][0] - scores[k][1])
    hits, misses = scores[best]
    return best if hits >= 2 and misses <= hits // 10 else None


def press_intervals(masks):
    """(bit, t_before, t_on, t_last_on, t_off) of each press; t_before/t_off None at the edges of the capture."""
    out, open_presses = [], {}
    previous, previous_t = None, None
    for t, mask in masks:
        if previous is None:
            for bit in range(64):
                if mask >> bit & 1:
                    open_presses[bit] = [None, t, t]
        else:
            for bit in range(64):
                was, now = previous >> bit & 1, mask >> bit & 1
                if now and not was:
                    open_presses[bit] = [previous_t, t, t]
                elif now and was and bit in open_presses:
                    open_presses[bit][2] = t
                elif was and not now and bit in open_presses:
                    before, on, last_on = open_presses.pop(bit)
                    out.append((bit, before, on, last_on, t))
        previous, previous_t = mask, t
    for bit, (before, on, last_on) in open_presses.items():
        out.append((bit, before, on, last_on, None))
    return sorted(out, key=lambda p: p[2])


def analyse_inputs(cap, boot, inputs, reason_bit_names):
    masks = [(m[1], m[3]) for m in cap.masks if m[0] == boot]
    events = [(e[1], e[3], e[4]) for e in cap.input_events if e[0] == boot]
    reports = [(e[1], e[3]) for e in cap.emergency if e[0] == boot]
    clears = [(h[1], h[4]) for h in cap.host_emergency if h[0] == boot and h[1] is not None and h[4]]
    layout = event_layout(masks, events)
    reasons = Timeline(reports)
    rows = []
    for bit, before, on, last_on, off in press_intervals(masks):
        meta = inputs[bit] if bit < len(inputs) else {"name": f"input {bit}", "reason": None, "delay_ms": None}
        row = {"bit": bit, "input": meta["name"], "expected_reason": meta["reason"],
               "configured_delay_ms": meta["delay_ms"], "pressed_at": on, "edge_from": before, "edge_to": on,
               "edge_exact": False, "released_at": off, "released_from": last_on}
        if layout is not None:
            for t, a, b in events:
                index, kind = (a, b) if layout == 0 else (b, a)
                if index == bit and kind == 0 and (before is None or t > before) and t <= on + 1e-6:
                    row.update(edge_from=t, edge_to=t, edge_exact=True, pressed_at=t)
                if index == bit and kind == 1 and off is not None and last_on < t <= off + 1e-6:
                    row["released_from"] = row["released_at"] = t
        row["held_s"] = None if off is None else row["released_at"] - row["pressed_at"]
        expected = sum(REASONS_OF_INPUT.get(meta["reason"], ())) if meta["reason"] else INPUT_REASON_MASK
        start = row["edge_from"] if row["edge_from"] is not None else on
        end = row["released_at"] if off is not None else float("inf")
        before_reason = reasons.at(start - 1e-6) or 0
        row["reason_before"] = reason_names(before_reason & ~LATCH, reason_bit_names)
        row["already_set"] = bool(before_reason & expected)
        found = None
        for i in reasons.between(start - 1e-6, end + 1e-6):
            previous = reasons.values[i - 1] if i > 0 else 0
            new = reasons.values[i] & ~previous & expected
            if new:
                found = (reasons.times[i], reasons.values[i], new)
                break
        if found is None:
            row.update(reason=None, delay_min_ms=None, delay_max_ms=None)
            rows.append(row)
            continue
        t_reason, value, new = found
        row["reason"] = reason_names(new, reason_bit_names)
        row["reason_at"] = t_reason
        row["latch_with_reason"] = bool(value & LATCH)
        row["delay_min_ms"] = (t_reason - row["edge_to"]) * 1000
        row["delay_max_ms"] = None if row["edge_from"] is None else (t_reason - row["edge_from"]) * 1000
        if off is not None:
            released = row["released_at"]
            # The state a moment after the release: the board reports a change at once, the mask only every 200 ms.
            after = reasons.at(released + RELEASE_SETTLE) or 0
            row["reason_after_release"] = bool(after & new)
            row["latched"] = bool(after & LATCH and value & LATCH)
            later = reasons.between(released, float("inf"))
            row["latch_gone_at"] = next((reasons.times[i] for i in later if not reasons.values[i] & LATCH), None)
            row["host_clear_at"] = next(
                (t for t, clear in clears if t >= released and clear & LATCH
                 and (row["latch_gone_at"] is None or t <= row["latch_gone_at"] + 0.05)), None)
        rows.append(row)
    return rows, layout


def analyse_imu(cap, boot, tilt_limits, report_tilt, yaw_threshold):
    samples = [(s[1], s[3]) for s in cap.imu if s[0] == boot]
    out = {"samples": len(samples)}
    if len(samples) < 2:
        return out
    times = [t for t, _ in samples]
    gaps = [b - a for a, b in zip(times, times[1:])]
    span = times[-1] - times[0]
    out.update(rate_hz=(len(samples) - 1) / span if span > 0 else None,
               median_gap_ms=sorted(gaps)[len(gaps) // 2] * 1e3,
               max_gap_ms=max(gaps) * 1e3, max_gap_at=times[gaps.index(max(gaps)) + 1], board_span_s=span)
    tilt, roll, pitch, rest = [], [], [], []
    for _, v in samples:
        ax, ay, az, gx, gy, gz = v[:6]
        tilt.append(math.degrees(math.atan2(math.hypot(ax, ay), az)))
        roll.append(math.degrees(math.atan2(ay, az)))
        pitch.append(math.degrees(math.atan2(-ax, math.hypot(ay, az))))
        rest.append(math.sqrt(gx * gx + gy * gy + gz * gz) < 0.05 and abs(math.sqrt(ax * ax + ay * ay + az * az)
                                                                          - GRAVITY) < 0.5 and tilt[-1] <= report_tilt)
    # Rest: runs of at least 0.5 s in which every sample is still and level.
    still = []
    run = []
    for i, ok in enumerate(rest + [False]):
        if ok:
            run.append(i)
            continue
        if run and times[run[-1]] - times[run[0]] >= 0.5:
            still.extend(run)
        run = []
    if still:
        def stat(values):
            mean = sum(values) / len(values)
            return mean, math.sqrt(sum((x - mean) ** 2 for x in values) / len(values))
        az_mean, az_std = stat([samples[i][1][2] for i in still])
        out["rest"] = {"samples": len(still), "seconds": len(still) / out["rate_hz"] if out["rate_hz"] else None,
                       "a_z": az_mean, "a_z_std": az_std,
                       "a_norm": stat([math.sqrt(sum(x * x for x in samples[i][1][:3])) for i in still])[0],
                       "a_x": stat([samples[i][1][0] for i in still])[0],
                       "a_y": stat([samples[i][1][1] for i in still])[0],
                       "tilt": stat([tilt[i] for i in still])[0],
                       "gyro_bias": [stat([samples[i][1][3 + k] for i in still])[0] for k in range(3)]}
    out["tilt_episodes"] = tilt_episodes(times, tilt, roll, pitch, tilt_limits, report_tilt)
    out["yaw_episodes"] = yaw_episodes(times, [v[5] for _, v in samples], yaw_threshold)
    return out


def latch_after(times, tilt, start, end, limit, hold):
    """Board time at which a check "above limit for hold s" latches within samples start..end, or None."""
    since = None
    for i in range(start, end + 1):
        if tilt[i] > limit:
            since = times[i] if since is None else since
            if times[i] - since >= hold - 1e-6:
                return times[i]
        else:
            since = None
    return None


def tilt_episodes(times, tilt, roll, pitch, limits, report_tilt):
    out, i, n = [], 0, len(tilt)
    while i < n:
        if tilt[i] <= report_tilt:
            i += 1
            continue
        j = i
        while j + 1 < n and tilt[j + 1] > report_tilt:
            j += 1
        top = max(range(i, j + 1), key=lambda k: tilt[k])
        episode = {"start": times[i], "seconds": times[j] - times[i], "max_tilt": tilt[top], "roll_at_max": roll[top],
                   "pitch_at_max": pitch[top], "checks": {}}
        for name, (limit, hold) in limits.items():
            above = [k for k in range(i, j + 1) if tilt[k] > limit]
            latched = latch_after(times, tilt, i, j, limit, hold)
            first = times[above[0]] if above else None
            episode["checks"][name] = {"limit": limit, "hold": hold, "latched_at": latched,
                                       "after_crossing_s": None if latched is None else latched - first}
        out.append(episode)
        i = j + 1
    return out


def yaw_episodes(times, rates, threshold):
    out, i, n = [], 0, len(rates)
    while i < n:
        if abs(rates[i]) <= threshold:
            i += 1
            continue
        sign = 1 if rates[i] > 0 else -1
        j = i
        while j + 1 < n and rates[j + 1] * sign > threshold:
            j += 1
        if times[j] - times[i] >= 0.2:
            angle = sum(rates[k] * (times[k + 1] - times[k]) for k in range(i, j))
            peak = max((rates[k] for k in range(i, j + 1)), key=abs)
            out.append({"start": times[i], "seconds": times[j] - times[i], "sign": "+" if sign > 0 else "-",
                        "peak": peak, "angle_deg": math.degrees(angle)})
        i = j + 1
    return out


def last_before(samples, t):
    """Index of the last sample (sorted by time in [0]) at or before t, or None."""
    i = bisect.bisect_right([s[0] for s in samples], t) - 1
    return i if i >= 0 else None


def first_reason_in(reports, start, end, names):
    """First report in (start, end] that sets a bit other than the ones already set at start."""
    before = 0
    for t, value in reports:
        if t <= start:
            before = value
            continue
        if t > end:
            break
        new = value & ~before & ~LATCH
        if new:
            return t, reason_names(new, names)
        before = value
    return None, None


def cause_times(stop, run_start, twists, host_times, reports, names, active, emergency_inputs):
    """How long before `stop` the host fell silent, sent its last emergency input (the board's 1.0 s count from
    it), its last command and its first 0, and a reason came."""
    out = {"host_silent_s": None}
    i = last_before(emergency_inputs, stop)
    out["since_emergency_input_s"] = None if i is None else stop - emergency_inputs[i][0]
    # The host fell silent: its last datagram before a gap of HOST_GAP s or more that began in the run.
    i = last_before(host_times, stop)
    while i is not None and i >= 0 and host_times[i][0] >= run_start - HOST_GAP:
        following = host_times[i + 1][0] if i + 1 < len(host_times) else float("inf")
        if following - host_times[i][0] >= HOST_GAP:
            out["host_silent_s"] = stop - host_times[i][0]
            break
        i -= 1
    last_cmd = next((t for t, value in reversed(twists) if t <= stop and active(value)), None)
    out["since_last_command_s"] = None if last_cmd is None else stop - last_cmd
    zero = None if last_cmd is None else next((t for t, value in twists if last_cmd < t <= stop and not active(value)),
                                              None)
    out["since_zero_command_s"] = None if zero is None else stop - zero
    t_reason, reason = first_reason_in(reports, run_start, stop, names)
    out["reason"] = reason
    out["since_reason_s"] = None if t_reason is None else stop - t_reason
    return out


def runs(series, moving, still_s):
    """(start, stop) of each stretch in which `moving` holds, split by still periods of still_s or more."""
    out, start, last_move = [], None, None
    for t, value in series:
        if not moving(value):
            continue
        if start is not None and t - last_move >= still_s:
            out.append((start, last_move))
            start = None
        if start is None:
            start = t
        last_move = t
    if start is not None:
        out.append((start, last_move))
    return out


def tick_gaps(ticks, speeds, ticks_per_meter, right_sign):
    """Gaps without wheel reports (the inner network down, as the capture on the host sees it): the ticks the counters
    moved across the gap and, at the speed before it, how long the wheels went on turning (an estimate)."""
    out = []
    for (t0, l0, r0), (t1, l1, r1) in zip(ticks, ticks[1:]):
        if t1 - t0 < TICK_GAP:
            continue
        dl = ((l1 - l0 + (1 << 31)) % (1 << 32)) - (1 << 31)
        dr = right_sign * (((r1 - r0 + (1 << 31)) % (1 << 32)) - (1 << 31))
        before = [v for t, v in speeds if t0 - 0.5 <= t <= t0]
        speed = [sum(v[2 + k] for v in before) / len(before) if before else 0.0 for k in (0, 1)]
        turning = [abs(d) / ticks_per_meter / abs(v) for d, v in zip((dl, dr), speed) if abs(v) > 0.01]
        out.append({"from": t0, "to": t1, "ticks_left": dl, "ticks_right": dr, "speed_before": speed,
                    "turning_s": max(turning) if turning else None})
    return out


def first_still(speeds, start, stop):
    """The first interval without a tick after which the run has only single ticks left (a wheel settling back),
    at most SETTLE_INTERVALS of them; `stop` itself when there is none. Slow runs with single ticks keep `stop`."""
    window = [(t, v) for t, v in speeds if start < t <= stop]
    for i, (t, v) in enumerate(window):
        if v[0] or v[1]:
            continue
        rest = [w for _, w in window[i + 1:] if w[0] or w[1]]
        if len(rest) <= SETTLE_INTERVALS and all(abs(w[0]) <= 1 and abs(w[1]) <= 1 for w in rest) and \
                any(abs(w[0]) > 1 or abs(w[1]) > 1 for _, w in window[:i]):
            return {"first_still": t, "still_before_stop_s": stop - t,
                    "settle_ticks": sum(abs(w[0]) + abs(w[1]) for w in rest)}
    return {"first_still": stop, "still_before_stop_s": 0.0, "settle_ticks": 0}


def analyse_drive(cap, boot, names, ticks_per_meter, wheel_distance, right_sign):
    ticks = [(t[1], t[3], t[4]) for t in cap.ticks if t[0] == boot]
    twists = [(t[1], (t[3], t[4])) for t in cap.twists if t[0] == boot]
    host_times = [(t[1],) for t in cap.host_times if t[0] == boot]
    reports = [(e[1], e[3]) for e in cap.emergency if e[0] == boot]
    emergency_inputs = [(h[1],) for h in cap.host_emergency if h[0] == boot]
    out = {"ticks_per_meter": ticks_per_meter, "wheel_distance": wheel_distance, "runs": [], "steady": []}
    speeds = []
    for (t0, l0, r0), (t1, l1, r1) in zip(ticks, ticks[1:]):
        # Across a gap the wheels are unseen: tick_gaps tells that distance, it starts no run.
        if t1 <= t0 or t1 - t0 >= TICK_GAP:
            continue
        dl = ((l1 - l0 + (1 << 31)) % (1 << 32)) - (1 << 31)
        dr = ((r1 - r0 + (1 << 31)) % (1 << 32)) - (1 << 31)
        speeds.append((t1, (dl, dr, dl / ticks_per_meter / (t1 - t0), right_sign * dr / ticks_per_meter / (t1 - t0))))
    out["samples"] = len(speeds)
    out["gaps"] = tick_gaps(ticks, speeds, ticks_per_meter, right_sign)

    def active(value):
        return abs(value[0]) > 1e-6 or abs(value[1]) > 1e-6

    for start, last_move in runs(speeds, lambda v: v[0] != 0 or v[1] != 0, 0.3):
        if speeds[-1][0] - last_move < 0.3:
            out["runs"].append({"start": start, "stop": None, "ongoing_at_end": True})
            continue
        # The wheels stood at the latest by the first interval without a tick: the safe side of the stop.
        last_tick = next(t for t, _ in speeds if t > last_move)
        i = last_before(twists, start)
        command = None if i is None else {"age_s": start - twists[i][0], "v": twists[i][1][0], "w": twists[i][1][1]}
        fresh = command is not None and active((command["v"], command["w"])) and command["age_s"] <= 0.25
        run = {"start": start, "stop": last_tick, "last_tick": last_move, "seconds": last_tick - start,
               "command_at_start": command,
               "fresh_command": fresh,
               **cause_times(last_tick, start, twists, host_times, reports, names, active, emergency_inputs)}
        zeros = None
        if run["since_last_command_s"] is not None:
            last_command = last_tick - run["since_last_command_s"]
            zeros = 0
            for t, value in twists:
                if t <= last_command:
                    continue
                if active(value):
                    break
                zeros += 1
        run["zeros_after_last_command"] = zeros
        run.update(first_still(speeds, start, last_tick))
        out["runs"].append(run)
    # Steady commands: the same twist for 1.5 s or more; the speed is taken after 0.7 s.
    groups, current = [], None
    for t, value in twists:
        key = (round(value[0], 4), round(value[1], 4))
        if current and current["key"] == key and t - current["last"] < 0.5:
            current["last"] = t
            continue
        current = {"key": key, "first": t, "last": t}
        groups.append(current)
    for g in groups:
        v, w = g["key"]
        if not active((v, w)) or g["last"] - g["first"] < 1.5:
            continue
        window = [s for t, s in speeds if g["first"] + 0.7 <= t <= g["last"]]
        if not window:
            continue
        left = sum(s[2] for s in window) / len(window)
        right = sum(s[3] for s in window) / len(window)
        duty = (v - w * wheel_distance / 2, v + w * wheel_distance / 2)
        out["steady"].append({"from": g["first"], "seconds": g["last"] - g["first"], "v": v, "w": w,
                              "duty_left": duty[0], "duty_right": duty[1], "speed_left": left, "speed_right": right,
                              "per_duty_left": left / duty[0] if abs(duty[0]) > 0.005 else None,
                              "per_duty_right": right / duty[1] if abs(duty[1]) > 0.005 else None})
    out["duty_model"] = {side: duty_model([(g[f"duty_{side}"], g[f"speed_{side}"]) for g in out["steady"]])
                         for side in ("left", "right")}
    return out


def duty_model(points):
    """Least squares of |speed| over |duty| for the steady commands in which the wheel turned with its duty: slope (m/s
    per duty) and deadband (duty at speed 0), or None with fewer than two different duties."""
    used = [(abs(d), abs(v)) for d, v in points if abs(d) > 0.005 and abs(v) > 0.002 and d * v > 0]
    duties = {round(d, 4) for d, _ in used}
    if len(duties) < 2:
        return None
    n = len(used)
    mean_d = sum(d for d, _ in used) / n
    mean_v = sum(v for _, v in used) / n
    slope = sum((d - mean_d) * (v - mean_v) for d, v in used) / sum((d - mean_d) ** 2 for d, _ in used)
    if slope <= 0:
        return None
    deadband = mean_d - mean_v / slope
    worst = max(abs(v - slope * (d - deadband)) for d, v in used)
    return {"slope": slope, "deadband": deadband, "points": n, "duties": len(duties), "worst_m_s": worst}


def analyse_blade(cap, boot, names, spinning_rpm):
    rpm = [(b[1], b[3]) for b in cap.blade_rpm if b[0] == boot]
    sets = [(b[1], b[3]) for b in cap.blade_set if b[0] == boot]
    host_times = [(t[1],) for t in cap.host_times if t[0] == boot]
    reports = [(e[1], e[3]) for e in cap.emergency if e[0] == boot]
    emergency_inputs = [(h[1],) for h in cap.host_emergency if h[0] == boot]
    out = {"samples": len(rpm), "runs": []}

    def active(value):
        return abs(value) > 1e-6

    for start, last in runs(rpm, lambda r: abs(r) >= spinning_rpm, 0.6):
        i = last_before(sets, start)
        setpoint = None if i is None else {"age_s": start - sets[i][0], "speed": sets[i][1]}
        below = next(((t, r) for t, r in rpm if t > last), None)
        zero = next((t for t, r in rpm if t > last and r == 0), None)
        stop = below[0] if below else None
        run = {"start": start, "last_spinning": last, "below_at": stop, "zero_at": zero, "setpoint_at_start": setpoint,
               "fresh_setpoint": setpoint is not None and active(setpoint["speed"]) and setpoint["age_s"] <= 1.0}
        if stop is not None:
            run.update(cause_times(stop, start, sets, host_times, reports, names, active, emergency_inputs))
        out["runs"].append(run)
    return out


def reason_changes(cap, boot, names):
    out, previous = [], None
    for b, t, ct, value in cap.emergency:
        if b != boot or value == previous:
            continue
        out.append({"board_t": t, "capture_t": ct, "reason": value, "names": reason_names(value, names)})
        previous = value
    return out


def host_clears(cap, boot, names):
    out, previous = [], None
    for b, t, ct, set_bits, clear_bits in cap.host_emergency:
        if b != boot or (set_bits, clear_bits) == previous:
            continue
        previous = (set_bits, clear_bits)
        out.append({"board_t": t, "capture_t": ct, "set": reason_names(set_bits, names),
                    "clear": reason_names(clear_bits, names)})
    return out


def load_marks(path):
    """Marks, signals, aborts and findings of a recording of measure.py: (wall clock, kind, text)."""
    out = []
    with open(path) as f:
        for line in f:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            kind = entry.get("kind")
            if kind == "mark":
                out.append((entry["wall"], kind, f"{entry['step']} {entry['text']}"))
            elif kind == "signal":
                out.append((entry["wall"], kind, entry["signal"]))
            elif kind in ("abort", "finding"):
                out.append((entry["wall"], kind, entry.get("why") or entry.get("text", "")))
            elif kind == "cmd":
                out.append((entry["wall"], kind, (entry["v"], entry["w"])))
    return out


def place_marks(cap, boot, marks):
    """Marks within this run of the board, in board time."""
    starts = [b.first_seen for b in cap.boots]
    end = starts[boot.index + 1] if boot.index + 1 < len(starts) else float("inf")
    out = []
    for wall, kind, text in marks:
        if boot.first_seen <= wall < end:
            t = boot.at_capture_time(wall)
            if t is not None:
                out.append({"board_t": t, "kind": kind, "text": text})
    return out


def step_at(marks, t):
    step = None
    for m in marks:
        if m["kind"] == "mark" and m["board_t"] <= t:
            step = m["text"]
    return step


def analyse(cap, inputs, tilt_limits, report_tilt, yaw_threshold, drive=None, blade_rpm=100.0, marks=None):
    """drive: ticks_per_meter, wheel_distance (the configuration in the capture wins) and right_sign; marks from
    load_marks."""
    drive = {**DRIVE_DEFAULTS, **(drive or {}), **cap.drive_config}
    names = (cap.descriptions[EMERGENCY].reason_bits if EMERGENCY in cap.descriptions else {}) or REASON_BITS
    if cap.input_config:
        inputs, inputs_from = cap.input_config, "the input configuration in the capture"
    else:
        inputs_from = inputs[1]
        inputs = inputs[0]
    report = {"capture": {"frames": cap.stats["frames"], "link_type": cap.stats.get("linktype"),
                          "cut_at_end": bool(cap.stats.get("cut_at_end")), "span_s": cap.span[1] - cap.span[0]
                          if cap.span[0] is not None else 0, **cap.stats_info,
                          "malformed": dict(cap.malformed),
                          "dangerous_ports": {f"{p} {port}": n for (p, port), n in cap.dangerous.items()}},
              "inputs_from": inputs_from, "multiple_delays_ms": cap.multiple_delays,
              "drive_from": "the configuration in the capture" if cap.drive_config else "defaults or arguments",
              "boots": []}
    report["datagrams"] = {f"{d} {sid} {TYPE_NAMES.get(mt, hex(mt))}": n
                           for (d, sid, mt), n in sorted(cap.counts.items())}
    for boot in cap.boots:
        rows, layout = analyse_inputs(cap, boot.index, inputs, names)
        report["boots"].append({
            "boot": boot.index, "endpoint": f"{boot.endpoint[0]}:{boot.endpoint[1]}",
            "first_seen_capture_t": boot.first_seen, "input_event_layout": layout,
            "presses": rows, "reasons": reason_changes(cap, boot.index, names),
            "host_emergency": host_clears(cap, boot.index, names),
            "imu": analyse_imu(cap, boot.index, tilt_limits, report_tilt, yaw_threshold),
            "drive": analyse_drive(cap, boot.index, names, drive["ticks_per_meter"], drive["wheel_distance"],
                                   drive["right_sign"]),
            "blade": analyse_blade(cap, boot.index, names, blade_rpm)})
        entry = report["boots"][-1]
        entry["marks"] = place_marks(cap, boot, marks or [])
        for press in entry["presses"]:
            press["step"] = step_at(entry["marks"], press["pressed_at"])
        for run in entry["drive"]["runs"] + entry["blade"]["runs"]:
            run["step"] = step_at(entry["marks"], run["start"])
        commands = [m["board_t"] for m in entry["marks"] if m["kind"] == "cmd" and any(m["text"])]
        for run in entry["drive"]["runs"]:
            i = bisect.bisect_right(commands, run["stop"]) - 1 if run["stop"] is not None else -1
            run["since_script_command_s"] = run["stop"] - commands[i] if i >= 0 else None
        entry["marks"] = [m for m in entry["marks"] if m["kind"] != "cmd"]
    return report


# Output ---------------------------------------------------------------------------------------------------------------

def table(headers, rows):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) if rows else len(str(h))
              for i, h in enumerate(headers)]
    lines = ["  ".join(str(h).ljust(w) for h, w in zip(headers, widths)),
             "  ".join("-" * w for w in widths)]
    lines += ["  ".join(str(c).ljust(w) for c, w in zip(r, widths)) for r in rows]
    return "\n".join(lines)


def num(value, digits=3):
    return "-" if value is None else f"{value:.{digits}f}"


def seconds(value):
    return "-" if value is None else f"{value:.2f}"


def drive_text(drive):
    out = ["", f"Wheels: {drive['samples']} tick intervals, {drive['ticks_per_meter']:g} ticks/m, wheel distance "
               f"{drive['wheel_distance']:g} m"]
    rows = []
    for r in drive["runs"]:
        if r.get("ongoing_at_end"):
            rows.append([num(r["start"]), "-", r.get("step") or "-", "turning at the end"] + ["-"] * 9)
            continue
        c = r["command_at_start"]
        start = "fresh" if r["fresh_command"] else (
            "WARNING no command" if c is None
            else f"WARNING last command {c['v']:.3f}/{c['w']:.3f} {c['age_s']:.2f} s old")
        rows.append([num(r["start"]), num(r["stop"]), r.get("step") or "-", start, seconds(r["host_silent_s"]),
                     seconds(r["since_emergency_input_s"]), seconds(r.get("since_script_command_s")),
                     seconds(r["since_last_command_s"]),
                     seconds(r["since_zero_command_s"]),
                     r["reason"] or "-", seconds(r["since_reason_s"]),
                     "-" if r["zeros_after_last_command"] is None else r["zeros_after_last_command"],
                     f"-{r['still_before_stop_s']:.2f} s, {r['settle_ticks']} ticks" if r["settle_ticks"] else "-"])
    if rows:
        out.append("Runs (stop = first tick interval without a tick, ~40 ms; times are stop minus the cause; script "
                   "= last command of measure.py, cmd = last command and first 0 of the host to the board; settle = "
                   "the wheels stood this much before the stop, single ticks after it count as settling back):")
        out.append(table(["start", "stop", "step", "start with", "host silent", "emerg in", "script", "last cmd",
                          "cmd 0", "reason", "reason s", "zeros after", "settle"], rows))
    if drive.get("gaps"):
        out.append("Gaps without wheel reports (ticks across the gap; turning = their distance at the speed before it, "
                   "an estimate):")
        out.append(table(["from", "to", "ticks L", "ticks R", "m/s L before", "m/s R before", "turning s"],
                         [[num(g["from"]), num(g["to"]), g["ticks_left"], g["ticks_right"], num(g["speed_before"][0]),
                           num(g["speed_before"][1]), seconds(g["turning_s"])] for g in drive["gaps"]]))
    if drive["steady"]:
        out.append("Steady commands (m/s per unit of duty at that duty):")
        out.append(table(["from", "s", "v", "w", "duty L", "duty R", "m/s L", "m/s R", "per duty L", "per duty R"],
                         [[num(g["from"]), num(g["seconds"], 1), num(g["v"]), num(g["w"]), num(g["duty_left"]),
                           num(g["duty_right"]), num(g["speed_left"]), num(g["speed_right"]),
                           num(g["per_duty_left"]), num(g["per_duty_right"])] for g in drive["steady"]]))
        for side in ("left", "right"):
            model = drive["duty_model"][side]
            if model is None:
                out.append(f"Speed model {side}: needs steady runs at two duties or more.")
            else:
                out.append(f"Speed model {side}: duty_slope {model['slope']:.3f}, duty_deadband {model['deadband']:.4f} "
                           f"({model['points']} runs at {model['duties']} duties, worst {model['worst_m_s']:.3f} m/s "
                           "off the line)")
    return out


def blade_text(blade):
    if not blade["runs"]:
        return ["", f"Blade: {blade['samples']} rpm reports, no run."]
    rows = []
    for r in blade["runs"]:
        s = r["setpoint_at_start"]
        start = "fresh" if r["fresh_setpoint"] else (
            "WARNING no setpoint" if s is None else f"WARNING last setpoint {s['speed']:.2f} {s['age_s']:.2f} s old")
        rows.append([num(r["start"]), num(r["below_at"]), num(r["zero_at"]), r.get("step") or "-", start,
                     seconds(r.get("host_silent_s")), seconds(r.get("since_emergency_input_s")),
                     seconds(r.get("since_last_command_s")), seconds(r.get("since_zero_command_s")),
                     r.get("reason") or "-", seconds(r.get("since_reason_s"))])
    return ["", f"Blade runs (rpm every 0.5 s; times are 'below' minus the cause):",
            table(["start", "below", "rpm 0", "step", "start with", "host silent", "emerg in", "last on", "set 0",
                   "reason", "reason s"],
                  rows)]


def text_report(report):
    out = []
    c = report["capture"]
    out.append(f"Capture: {c['frames']} frames over {c['span_s']:.1f} s, link type {c['link_type']}, board found by "
               f"{c.get('board_found_by')}, descriptions from {c.get('descriptions', 'nowhere')}"
               + (", last record cut off" if c["cut_at_end"] else ""))
    if c["malformed"]:
        out.append(f"Malformed datagrams: {c['malformed']}")
    if c["dangerous_ports"]:
        out.append(f"WARNING traffic to boot loader or passthrough ports: {c['dangerous_ports']}")
    out.append(f"Inputs from {report['inputs_from']}" + (
        f", multiple delays {report['multiple_delays_ms']} ms" if report["multiple_delays_ms"] else ""))
    for boot in report["boots"]:
        out.append("")
        out.append(f"== Board run {boot['boot']} at {boot['endpoint']} (board time in s since its start) ==")
        if boot["input_event_layout"] is not None:
            out.append(f"Input events used for exact edges (index in byte {boot['input_event_layout']})")
        rows = []
        for p in boot["presses"]:
            delay = "-" if p["reason"] is None else (
                f"{p['delay_min_ms']:.0f}" if p["edge_exact"] or p["delay_max_ms"] is None
                or abs(p["delay_max_ms"] - p["delay_min_ms"]) < 1
                else f"{p['delay_min_ms']:.0f}..{p['delay_max_ms']:.0f}")
            if p["reason"] is None:
                latch = "-"
            elif p["released_at"] is None:
                latch = "held at end"
            elif p["latched"]:
                latch = ("LATCH, reason stays" if p["reason_after_release"] else "LATCH") + (
                    f", host cleared at {p['host_clear_at']:.3f}" if p["host_clear_at"] is not None else
                    (", until the end" if p["latch_gone_at"] is None else f", gone at {p['latch_gone_at']:.3f}"))
            else:
                latch = "no" + (", reason stays" if p["reason_after_release"] else "")
            rows.append([num(p["pressed_at"]), p["input"], num(p["held_s"], 2), p["reason"] or
                         ("none" + (" (already set)" if p["already_set"] else "")), delay,
                         "-" if p["configured_delay_ms"] is None else p["configured_delay_ms"], latch])
        out.append(table(["pressed", "input", "held s", "new reason", "delay ms", "config ms", "latched after release"],
                         rows) if rows else "No input pressed.")
        out.append("")
        out.append("Emergency reasons reported (changes):")
        out.append(table(["board s", "reason"], [[num(r["board_t"]), r["names"]] for r in boot["reasons"]]))
        if boot["host_emergency"]:
            out.append("")
            out.append("Emergency input of the host (changes; board time estimated from the capture clock):")
            out.append(table(["board s", "set", "clear"], [[num(h["board_t"]), h["set"], h["clear"]]
                                                           for h in boot["host_emergency"]]))
        if boot["marks"]:
            out.append("")
            out.append("Marks of the measurement script:")
            out.append(table(["board s", "kind", "text"], [[num(m["board_t"]), m["kind"], m["text"]]
                                                          for m in boot["marks"]]))
        out.extend(drive_text(boot["drive"]))
        out.extend(blade_text(boot["blade"]))
        imu = boot["imu"]
        out.append("")
        if imu["samples"] < 2:
            out.append("IMU: no samples.")
            continue
        out.append(f"IMU: {imu['samples']} samples, {num(imu['rate_hz'], 1)} Hz over {imu['board_span_s']:.1f} s, "
                   f"median gap {imu['median_gap_ms']:.1f} ms, longest gap {imu['max_gap_ms']:.1f} ms at "
                   f"{imu['max_gap_at']:.3f}")
        rest = imu.get("rest")
        if rest:
            out.append(f"At rest and level ({rest['samples']} samples): a_z {rest['a_z']:.3f} ± "
                       f"{rest['a_z_std']:.3f} m/s², "
                       f"|a| {rest['a_norm']:.3f}, a_x {rest['a_x']:.3f}, a_y {rest['a_y']:.3f}, tilt "
                       f"{rest['tilt']:.1f} deg, gyro bias " + ", ".join(f"{g:.4f}" for g in rest["gyro_bias"])
                       + " rad/s")
        else:
            out.append("At rest: no still and level stretch of 0.5 s.")
        if imu["tilt_episodes"]:
            names = list(imu["tilt_episodes"][0]["checks"])
            out.append(table(["from", "s", "max tilt", "roll", "pitch"] +
                             [f"{n} {e['limit']:.0f}°/{e['hold']}s" for n, e in
                              imu["tilt_episodes"][0]["checks"].items()],
                             [[num(e["start"]), num(e["seconds"], 2), num(e["max_tilt"], 1), num(e["roll_at_max"], 1),
                               num(e["pitch_at_max"], 1)] +
                              ["no" if e["checks"][n]["latched_at"] is None else
                               f"latch {e['checks'][n]['after_crossing_s']:.2f} s after crossing" for n in names]
                              for e in imu["tilt_episodes"]]))
        if imu["yaw_episodes"]:
            out.append(table(["from", "s", "yaw rate", "peak rad/s", "angle deg"],
                             [[num(e["start"]), num(e["seconds"], 2), e["sign"], num(e["peak"]),
                               num(e["angle_deg"], 0)] for e in imu["yaw_episodes"]]))
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pcap")
    parser.add_argument("--services", default=DEFAULT_SERVICES,
                        help="service descriptions (JSON) used when the capture holds no announcements")
    parser.add_argument("--inputs", help="JSON list of {name, reason, delay_ms} in the board's order "
                                         "(default: the NX60i as in nx60i.yaml)")
    parser.add_argument("--driver-tilt", type=float, nargs=2, default=[40.0, 0.5], metavar=("DEG", "S"),
                        help="tilt check of xcore_driver (tilt_limit, tilt_time)")
    parser.add_argument("--safety-tilt", type=float, nargs=2, default=[31.0, 1.0], metavar=("DEG", "S"),
                        help="tilt check of mowbite_safety (tilt_limit, tilt_time)")
    parser.add_argument("--report-tilt", type=float, default=15.0, help="tilt in deg above which an episode starts")
    parser.add_argument("--yaw-threshold", type=float, default=0.1, help="yaw rate in rad/s for a turn episode")
    parser.add_argument("--ticks-per-meter", type=float, default=DRIVE_DEFAULTS["ticks_per_meter"],
                        help="used when the capture holds no configuration of the diff drive")
    parser.add_argument("--wheel-distance", type=float, default=DRIVE_DEFAULTS["wheel_distance"])
    parser.add_argument("--right-sign", type=int, choices=(1, -1), default=DRIVE_DEFAULTS["right_sign"])
    parser.add_argument("--blade-rpm", type=float, default=100.0, help="rpm above which the blade counts as turning")
    parser.add_argument("--marks", help="recording of measure.py (JSON lines) whose marks go into the report")
    parser.add_argument("--json", help="also write the whole report as JSON to this file")
    args = parser.parse_args(argv)
    if args.inputs:
        with open(args.inputs) as f:
            inputs = (json.load(f), "file " + args.inputs)
    else:
        inputs = (NX60I_INPUTS, "the NX60i default (order of nx60i.yaml; an assumption for OpenMower's order)")
    try:
        cap = load_capture(args.pcap, args.services)
    except (CaptureError, OSError) as e:
        sys.exit(f"xcore_capture: {e}")
    report = analyse(cap, inputs, {"driver": tuple(args.driver_tilt), "safety": tuple(args.safety_tilt)},
                     args.report_tilt, args.yaw_threshold,
                     {"ticks_per_meter": args.ticks_per_meter, "wheel_distance": args.wheel_distance,
                      "right_sign": args.right_sign}, args.blade_rpm,
                     load_marks(args.marks) if args.marks else None)
    print(text_report(report))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)


if __name__ == "__main__":
    main()
