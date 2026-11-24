"""Checks a mower against the OpenMower app interface the way the app uses it (masterplan 3.8).

Read-only by default: what the mower publishes and answers without being asked to act. --write adds the steps that act
(map.replace, actions, teleop, flags); it needs the simulated MowBite OS, which alone offers the RPC sim.flags, freshly
started: it checks the state since the start and reads the frames from --sim-log, which fake_tron stops writing at its
--log-max-mb (20 MB, about an hour).
stdout: one JSON document with a status (ok, missing, different, skipped) per point; PASS/FAIL/SKIP lines go to stderr.
Exit 0 without missing or different, 1 with, 2 for a wrong call or a mower that cannot be reached.
"""
import argparse
import base64
import datetime
import hashlib
import json
import math
import os
import shutil
import struct
import subprocess
import sys
import threading
import time
import zoneinfo

import paho.mqtt.client as mqtt

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--host", required=True)
parser.add_argument("--port", type=int, required=True, help="MQTT over TCP")
parser.add_argument("--target", required=True, choices=["openmower", "mowbite-os"])
parser.add_argument("--user", default=os.environ.get("MQTT_USER"))
parser.add_argument("--pass", dest="password", default=os.environ.get("MQTT_PASS"))
parser.add_argument("--app-url", help="accepted for the common call of the test series, not used here")
parser.add_argument("--seconds", type=float, default=2.0, help="how long to collect what the mower publishes")
parser.add_argument("--write", action="store_true", help="also act on the mower; simulated MowBite OS only")
parser.add_argument("--sim-log", help="JSON lines log of the simulated MCU, for the checks of the frames it received")
parser.add_argument("--mqtt", type=int, choices=[3, 5], default=3,
                    help="5: connect with MQTT 5 and ask every RPC with a response topic and correlation data (AS4)")
parser.add_argument("--bridge-mqtt", type=int, choices=[3, 5], default=5,
                    help="with --mqtt 5: what system.info.mqtt is to say; 3 for a bridge behind a broker of 3.1.1")
args = parser.parse_args()
if args.write and args.target != "mowbite-os":
    parser.error("--write is implemented for --target mowbite-os only")
if args.mqtt == 5 and args.target != "mowbite-os":
    parser.error("--mqtt 5 is for --target mowbite-os only")
if args.mqtt == 5:
    from paho.mqtt.packettypes import PacketTypes
    from paho.mqtt.properties import Properties

LOG = args.sim_log
msgs = []
results = []
# With --mqtt 5: the correlation data that came with the message at that index of msgs.
correlations = {}
RESPONSE = "rpc/response/stack_test"
# With --mqtt 5: (id, topic, correlation sent, correlation back) of every answer rpc() took.
routes = []
# (method, answer) of every command, for the check against the schema at the end.
COMMAND_ANSWERS = []
# Every answer of rpc() with its method, for AS3 against the results of x-methods.
RPC_ANSWERS = []


def record(point, status, example=""):
    results.append({"point": point, "status": status, "example": example})
    label = {"ok": "PASS ", "skipped": "SKIP "}.get(status, "FAIL ")
    print(label + point + (f"  [{example}]" if example else ""), file=sys.stderr)


def check(name, cond, detail="", point=None, missing=False):
    """missing: what an absent part means for a mandatory part of the OpenMower interface (3.1)."""
    record(point or "3.1 " + name, "ok" if cond else ("missing" if missing else "different"), detail)


def skip(name, reason, point=None):
    record(point or "3.1 " + name, "skipped", reason)


def check_log(name, cond, detail="", point=None):
    if LOG:
        check(name, cond, detail, point)
    else:
        skip(name, "needs the MCU log of the simulation (--sim-log)", point)


def finish(code=None):
    if args.mqtt == 5:
        check_routes()
    summary = {k: sum(r["status"] == k for r in results) for k in ("ok", "missing", "different", "skipped")}
    print(json.dumps({"target": args.target, "host": args.host, "write": args.write, "points": results,
                      "summary": summary}, indent=1))
    sys.stdout.flush()
    c.loop_stop()
    code = code if code is not None else (1 if summary["missing"] or summary["different"] else 0)
    print("RESULT", "OK" if code == 0 else "FAILED", file=sys.stderr, flush=True)
    sys.exit(code)


def on_message(_c, _u, m):
    props = getattr(m, "properties", None)
    correlation = getattr(props, "CorrelationData", None) if props else None
    if correlation is not None:
        correlations[len(msgs)] = correlation
    msgs.append((time.time(), m.topic, m.payload, m.retain))


def latest(topic):
    for t, top, p, r in reversed(msgs):
        if top == topic:
            return p
    return None


def wait_for(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        r = pred()
        if r:
            return r
        time.sleep(0.05)
    return None


def set_flags(**flags):
    r = rpc("sim.flags", [flags], "flags")
    assert r and "result" in r, f"sim.flags failed: {r}"


def rpc(method, params=None, rid="1", timeout=5.0, response_topic=RESPONSE, correlation=None):
    """With --mqtt 5 asked with a response topic and correlation data; the answer is taken from there or from
    rpc/response, check_routes() tells whether it came where it should."""
    req = {"jsonrpc": "2.0", "method": method, "id": rid}
    if params is not None:
        req["params"] = params
    start = len(msgs)
    props = None
    if args.mqtt == 5:
        correlation = correlation if correlation is not None else ("c-" + json.dumps(rid)).encode()
        props = Properties(PacketTypes.PUBLISH)
        props.ResponseTopic = response_topic
        props.CorrelationData = correlation
    c.publish("rpc/request", json.dumps(req), properties=props)

    def answer():
        for i in range(start, len(msgs)):
            _, top, p, _ = msgs[i]
            if top in ("rpc/response", response_topic):
                j = json.loads(p)
                if j.get("id") == rid:
                    if args.mqtt == 5 and response_topic == RESPONSE:
                        routes.append((rid, top, correlation, correlations.get(i)))
                    return j
        return None
    r = wait_for(answer, timeout)
    RPC_ANSWERS.append((method, r))
    return r


REC = "mower_logic:area_recording/"
GEOMETRY_KEYS = ["robot_width", "robot_front", "robot_rear", "robot_outline", "mower_width", "blade_ahead",
                 "blade_offset", "antenna_x", "antenna_y", "track", "wheel_radius", "wheel_width", "dock_contact_x",
                 "dock_footprint", "ticks_per_m", "imu_roll", "imu_pitch", "robot_shape"]


def enabled(action_id):
    return {a["action_id"]: a["enabled"] for a in json.loads(latest("actions/json") or "[]")}.get(action_id)


def overlay_lines(color):
    return [p for p in json.loads(latest("map_overlay/json") or "{}").get("polygons") or [] if p.get("color") == color]


def teleop(vx, vz):
    doc = b"\x01vx\x00" + struct.pack("<d", vx) + b"\x01vz\x00" + struct.pack("<d", vz) + b"\x00"
    c.publish("teleop", struct.pack("<i", len(doc) + 4) + doc)


class Hold:
    """A held joystick: teleop every 100 ms like the app, until released with one zero."""

    def __init__(self, vx, vz):
        self.running = True
        self.thread = threading.Thread(target=self._run, args=(vx, vz), daemon=True)
        self.thread.start()

    def _run(self, vx, vz):
        while self.running:
            teleop(vx, vz)
            time.sleep(0.1)

    def release(self):
        self.running = False
        self.thread.join()
        teleop(0.0, 0.0)


def sent_speeds(since):
    out = []
    if not LOG:
        return out
    with open(LOG) as f:
        for line in f:
            e = json.loads(line)
            if e.get("type") == 0 and e.get("mod") == 4 and e["t"] >= since:
                out.append((e["t"], e["speed"]))
    return out


def sim_time():
    if not LOG:
        return 0.0
    with open(LOG) as f:
        lines = f.readlines()
    return json.loads(lines[-1])["t"] if lines else 0.0


c = mqtt.Client("stack_test", protocol=mqtt.MQTTv5 if args.mqtt == 5 else mqtt.MQTTv311)
if args.user:
    c.username_pw_set(args.user, args.password)
connected = []
c.on_connect = lambda _c, _u, _f, rc, *_: connected.append(getattr(rc, "value", rc))
c.on_message = on_message
try:
    if args.mqtt == 5:
        c.connect(args.host, args.port, clean_start=True)
    else:
        c.connect(args.host, args.port)
except OSError as e:
    print(f"cannot reach {args.host}:{args.port}: {e}", file=sys.stderr)
    sys.exit(2)
c.subscribe("#")
c.loop_start()
time.sleep(args.seconds)
if connected and connected[0] != 0:
    print(f"the broker refused the connection (code {connected[0]})", file=sys.stderr)
    sys.exit(2)

retained = {top for _, top, _, r in msgs if r}
for top in ["capabilities/json", "version/json", "params/json", "sensor_infos/json", "actions/json"]:
    check("retained " + top, top in retained, missing=True)

params = json.loads(latest("params/json") or "{}")
if args.target == "mowbite-os":
    check("params /ll/board is MowBite", params.get("/ll/board") == "MowBite", str(params.get("/ll/board")))
else:
    skip("params /ll/board is MowBite", "MowBite OS only")

infos = json.loads(latest("sensor_infos/json") or "[]")
ids = sorted(i["sensor_id"] for i in infos)
# MowBite OS: 11 of the driver and the logic, os_ntrip_state and os_correction_age of the corrections.
check("sensor ids", "om_v_battery" in ids and "om_gps_accuracy" in ids and (len(ids) == 13 or args.target != "mowbite-os"),
      ",".join(ids), missing=True)
battery_info = next((i for i in infos if i["sensor_id"] == "om_v_battery"), {})
if args.target == "mowbite-os":
    check("battery max 25.2 and 0/1 flags", battery_info.get("max_value") == 25.2 and battery_info.get("has_min_max") == 1,
          json.dumps(battery_info))
else:
    skip("battery max 25.2 and 0/1 flags", "MowBite OS only")

state = wait_for(lambda: latest("robot_state/json"))
s = json.loads(state) if state else {}
check("robot_state fields", all(k in s for k in ("battery_percentage", "gps_percentage", "current_state", "emergency",
                                                  "is_charging", "pose")), (state or b"")[:160].decode(), missing=True)
v = wait_for(lambda: latest("sensors/om_v_battery/data"))
try:
    float(v)
    battery_ok = True
except (TypeError, ValueError):
    battery_ok = False
check("sensor value text", battery_ok, str(v), missing=True)

actions = json.loads(latest("actions/json") or "[]")
by_id = {a["action_id"]: a["enabled"] for a in actions}
check("actions", "mower_logic/reset_emergency" in by_id, f"{len(actions)} actions", missing=True)
if args.target == "mowbite-os":
    # The ids of OpenMower; OpenMower itself registers them only once it undocks or docks.
    check("actions abort_undocking and abort_docking", "mower_logic:undocking/abort_undocking" in by_id
          and "mower_logic:docking/abort_docking" in by_id, ",".join(a for a in by_id if "dock" in a))
else:
    skip("actions abort_undocking and abort_docking", "MowBite OS only")

r = rpc("rpc.methods")
methods = r["result"] if r and isinstance(r.get("result"), list) else None
if methods is not None:
    check("rpc.methods", "rpc.ping" in methods, json.dumps(methods), missing=True)
elif args.target == "mowbite-os":
    check("rpc.methods", False, "no answer", missing=True)
else:
    skip("rpc.methods", "no answer, older mowers have none")
# A planner that cannot load the message types registers no method; like offers(), a point only when they are missing.
if args.target == "mowbite-os" and methods is not None and not any(m.startswith("planner.") for m in methods):
    check("planner.* in rpc.methods", False, "no planner.* method: the planner did not register", missing=True)
if args.write and (methods is None or "sim.flags" not in methods):
    print("refusing --write: the mower does not offer sim.flags, so it is not a simulation", file=sys.stderr)
    sys.exit(2)


def offers(method):
    """True when rpc.methods lists the method; a MowBite OS promises it, an OpenMower may lack it (K2)."""
    if methods is not None and method in methods:
        return True
    if args.target == "mowbite-os":
        check(method + " is offered", False, "not in rpc.methods", missing=True)
    else:
        skip(method, "not in rpc.methods")
    return False


if offers("system.info"):
    r = rpc("system.info", None, "info")
    info = (r or {}).get("result") or {}
    check("system.info", info.get("name") == "mowbite-os" and info.get("version") and info.get("platform")
          and isinstance(info.get("schema"), int), json.dumps(info))
API_SCHEMA = None
if args.target == "mowbite-os":
    # AS3: the version of the interface in system.info, the schema itself over system.api.
    api_version = info.get("api") if offers("system.info") else None
    check("system.info.api is the version of the interface", isinstance(api_version, int) and api_version >= 1,
          json.dumps(api_version), point="AS3 system.info.api")
    api_break = info.get("api_break") if offers("system.info") else None
    check("system.info.api_break is the break of the interface", isinstance(api_break, int) and api_break >= 1,
          json.dumps(api_break), point="AS3 system.info.api_break")
    if offers("system.api"):
        r = rpc("system.api", None, "api")
        api = (r or {}).get("result") or {}
        API_SCHEMA = api.get("json_schema") if isinstance(api.get("json_schema"), dict) else None
        check("system.api gives api, api_break and json_schema of the version of system.info",
              api.get("api") == api_version and bool(API_SCHEMA) and API_SCHEMA.get("x-api") == api_version
              and API_SCHEMA.get("$id") == f"urn:mowbite-os:api:{api_version}"
              and api.get("api_break") == api_break and API_SCHEMA.get("x-api-break") == api_break,
              json.dumps({k: v for k, v in api.items() if k != "json_schema"}), point="AS3 system.api")
    # AM4: an id comes back as it was sent, also a number, from the bridge and from a node.
    r = rpc("rpc.ping", None, 42)
    check("a number id comes back as a number (bridge)", bool(r) and r.get("id") == 42 and r.get("result") == "pong",
          json.dumps(r), point="AM4 ids")
    if offers("robot.geometry.get"):
        r = rpc("robot.geometry.get", None, 4711)
        check("a number id comes back as a number (node)", bool(r) and r.get("id") == 4711 and "result" in r,
              json.dumps(r)[:200], point="AM4 ids")
        r = rpc("robot.geometry.get", None, {"client": "check", "n": 1})
        check("an object id comes back as it was sent (node)", bool(r) and "result" in r, json.dumps(r)[:200],
              point="AM4 ids")
    if API_SCHEMA is not None:
        # I1: the names of the interface file against what the OS offers and sends.
        listed = API_SCHEMA.get("x-methods", {})
        offered = [m for m in methods or [] if not m.startswith(("planner.", "sim."))]
        unknown = [m for m in offered if m not in listed]
        absent = [m for m, e in listed.items() if not e.get("offered") and m not in (methods or [])]
        check("rpc.methods and x-methods name the same methods", bool(listed) and not unknown and not absent,
              f"not in x-methods {unknown}, not offered {absent}", point="I1 methods")
        known = API_SCHEMA.get("x-sensors", {})
        bad = [i["sensor_id"] for i in infos if i["sensor_id"] not in known
               or known[i["sensor_id"]].get("name") != i.get("sensor_name")
               or known[i["sensor_id"]].get("unit") != i.get("unit")
               or (known[i["sensor_id"]].get("kind") == "text") != (i.get("value_type") == "STRING")]
        check("every sensor of sensor_infos/json as in x-sensors", bool(infos) and not bad, ",".join(bad),
              point="I1 sensors")
        mow_refused = API_SCHEMA.get("x-reasons", {}).get("MOWING_REFUSED", {})
        start = next((a for a in actions if a["action_id"] == "mower_logic:idle/start_mowing"), {})
        others = [a["action_id"] for a in actions if "reason" in a and a["action_id"] != start.get("action_id")]
        check("actions/json: reason only at start_mowing while it is off, a name of MOWING_REFUSED",
              bool(start) and ("reason" in start) == (start.get("enabled") == 0) and not others
              and (start.get("enabled") == 1 or start.get("reason") in mow_refused),
              json.dumps(start) + (f" others {others}" if others else ""), point="I2 start_mowing")
if offers("sensors.list"):
    # Every sensor of sensor_infos/json with its fields unchanged, the last value and its time (null before one came).
    r = rpc("sensors.list", None, "sensors")
    listed = (r or {}).get("result")
    info_of = {i["sensor_id"]: i for i in infos}
    check("sensors.list", isinstance(listed, list) and sorted(e.get("sensor_id") for e in listed) == ids
          and all({k: v for k, v in e.items() if k not in ("value", "stamp")} == info_of.get(e.get("sensor_id"))
                  and (e.get("value") is None) == (e.get("stamp") is None) for e in listed)
          and any(e["sensor_id"] == "om_v_battery" and isinstance(e.get("value"), (int, float))
                  and isinstance(e.get("stamp"), (int, float)) for e in listed), json.dumps(r)[:300])
    r = rpc("sensors.list", {"ids": ["om_v_battery", "no_such_sensor"]}, "sensors_ids")
    check("sensors.list with ids", [e.get("sensor_id") for e in (r or {}).get("result") or []] == ["om_v_battery"],
          json.dumps(r)[:200])
    r = rpc("sensors.list", {"ids": "om_v_battery"}, "sensors_bad")
    check("sensors.list refuses ids that are no list -32602", bool(r) and r.get("error", {}).get("code") == -32602,
          json.dumps(r and r.get("error")))
for method in ("map.replace", "mowing.plan"):
    offers(method)
if args.target == "mowbite-os":
    # "Home" from IDLE where the OS docks: the Tron and its simulation back onto the station.
    offers("docking.start")
if args.target == "mowbite-os" and offers("system.hold"):
    # Only the updater holds; a call out of range changes nothing, so it is checked here as well.
    for bad in ({"seconds": 1801}, {"seconds": -1}, {"seconds": 1.5}, {"seconds": "60"}, {}, [60]):
        r = rpc("system.hold", bad, "hold_bad")
        check("system.hold refuses " + json.dumps(bad) + " -32602",
              bool(r) and r.get("error", {}).get("code") == -32602, json.dumps(r and r.get("error")))
if offers("events.history"):
    r = rpc("events.history", None, "2")
    types = [e.get("type") for e in (r or {}).get("result", [])]
    check("events.history", isinstance((r or {}).get("result"), list) and (args.target != "mowbite-os" or "BOOTED" in types),
          ",".join(map(str, types)))
if offers("logs.recent"):
    r = rpc("logs.recent", {"level": "WARN"}, "9")
    check("logs.recent answers", r is not None and isinstance(r.get("result"), list),
          f'{len((r or {}).get("result", []))} entries')
if offers("map.area_properties"):
    r = rpc("map.area_properties", None, "4")
    expected = ["name", "type", "active", "mowable", "angle", "outline_count", "outline_overlap_count", "outline_offset"]
    if isinstance(api_version, int) and api_version >= 59:
        expected.append("mow_height_mm")
    # mowbite_map adds the planner's keys while a node offers planner.settings
    if methods is not None and "planner.settings" in methods:
        expected += ["mow_around", "angle_min", "angle_max"]
    check("map.area_properties", r and isinstance(r.get("result"), list) and (r["result"] == expected
                                                                              or args.target != "mowbite-os"),
          json.dumps(r and r.get("result")))
if offers("params.set") and args.target == "mowbite-os":
    p = json.loads(latest("params/json") or "{}")
    logic = sorted(k[len("/mower_logic/"):] for k in p if k.startswith("/mower_logic/"))
    # The settings of params.set as the schema of system.api lists them.
    settings = sorted((API_SCHEMA or {}).get("x-settings") or {})
    check("params/json has the settings of params.set", bool(settings) and logic == settings, ",".join(logic))
if args.target == "mowbite-os" and offers("robot.geometry.set") and offers("robot.geometry.get"):
    # The dimensions of the mower: every key with value, default, stored and whether the user can set it, the version
    # and the platform; the same values with the version in params/json for the planner.
    r = rpc("robot.geometry.get", None, "geo_get")
    answer = (r or {}).get("result") or {}
    settings = answer.get("settings") or {}
    check("robot.geometry.get has every key in order", list(settings) == GEOMETRY_KEYS, ",".join(settings))
    fields = ("value", "default", "stored", "type", "unit", "settable")
    check("robot.geometry.get has the fields of planner.settings", all(
        all(f in entry for f in fields) for entry in settings.values()), json.dumps(settings)[:200])
    check("robot.geometry.get has version and platform", isinstance(answer.get("version"), int) and
          isinstance(answer.get("platform"), str) and bool(answer.get("platform")),
          json.dumps({k: answer.get(k) for k in ("version", "platform")}))
    r = rpc("system.info", None, "geo_info")
    check("robot.geometry.get names the platform of system.info",
          answer.get("platform") == ((r or {}).get("result") or {}).get("platform"),
          f'{answer.get("platform")} / {((r or {}).get("result") or {}).get("platform")}')
    fixed = ("wheel_radius", "wheel_width", "dock_contact_x", "dock_footprint", "robot_shape")
    check("robot.geometry.get: only the platform sets " + ", ".join(fixed), all(
        settings.get(k, {}).get("settable") is False for k in fixed) and all(
        settings.get(k, {}).get("settable") is True for k in GEOMETRY_KEYS if k not in fixed), "")
    p = json.loads(latest("params/json") or "{}")
    # The shape is for drawing only, the planner does not read it: it stays out of params/json.
    shown = {k: p.get("/mowbite_geometry/" + k) for k in GEOMETRY_KEYS if k != "robot_shape"}
    check("params/json has the dimensions of the mower with the version", None not in shown.values() and
          p.get("/mowbite_geometry/version") == answer.get("version") and all(
              shown[k] == settings[k]["value"] for k in shown if k in settings), json.dumps(shown)[:200])
    check("params/json has the antenna offset of the localization from the geometry",
          p.get("/xbot_positioning/antenna_offset_x") == settings.get("antenna_x", {}).get("value") and
          p.get("/xbot_positioning/antenna_offset_y") == settings.get("antenna_y", {}).get("value"),
          f'{p.get("/xbot_positioning/antenna_offset_x")}, {p.get("/xbot_positioning/antenna_offset_y")}')
    # The keys of the calibration drive: ranges as the effective min and max, units of the new values.
    new_numbers = {"ticks_per_m": "1/m", "imu_roll": "rad", "imu_pitch": "rad"}
    check("robot.geometry.get: the calibrated keys carry unit, min and max around their value", all(
        settings.get(k, {}).get("unit") == unit and settings[k].get("settable") is True and
        all(isinstance(settings[k].get(f), (int, float)) for f in ("value", "min", "max")) and
        settings[k]["min"] <= settings[k]["value"] <= settings[k]["max"] for k, unit in new_numbers.items()),
        json.dumps({k: settings.get(k) for k in new_numbers})[:300])
    check("robot.geometry.get: track, antenna_x and antenna_y have a range", all(
        isinstance(settings.get(k, {}).get("min"), (int, float)) and isinstance(settings[k].get("max"), (int, float))
        and settings[k]["min"] < settings[k]["max"] for k in ("track", "antenna_x", "antenna_y")), "")
ACCESS_FIELDS = ["configured", "host", "port", "mountpoint", "version", "has_user", "has_password"]
if args.target == "mowbite-os" and offers("ntrip.get"):
    # The NTRIP access without user and password; only the names of the fields go into the report.
    r = rpc("ntrip.get", None, "ntrip_get")
    access = (r or {}).get("result")
    fields = list(access) if isinstance(access, dict) else []
    expected = ACCESS_FIELDS if access and access.get("configured") is True else (
        ["configured", "error"] if "error" in fields else ["configured"])
    check("ntrip.get without user and password", fields == expected, json.dumps(fields or r))
    writable = [m for m in ("ntrip.set", "ntrip.delete") if m in methods]
    check("ntrip.set and ntrip.delete only together", len(writable) in (0, 2), ",".join(writable))
SETUP_METHODS = ["setup.get", "setup.start", "setup.answer", "setup.skip", "setup.abort", "setup.reset", "setup.image",
                 "setup.platforms", "setup.platform.set"]
setup_offered = [m for m in SETUP_METHODS if methods is not None and m in methods]
if args.target == "mowbite-os":
    # The setup program, offered where the platform has a step list. Nothing here starts, skips or resets a step.
    check("setup.* only together", len(setup_offered) in (0, len(SETUP_METHODS)), ",".join(setup_offered),
          point="B1 setup methods")
if "setup.get" in setup_offered:
    r = rpc("setup.get", None, "setup_get")
    setup_state = (r or {}).get("result") or {}
    setup_steps = setup_state.get("steps") or []
    setup_unnamed = [s.get("id") for s in setup_steps
                     if not all(bool(((s.get("texts") or {}).get(lang) or {}).get("name")) for lang in ("de", "en"))]
    check("setup.get answers version, catalog and steps with names in de and en",
          isinstance(setup_state.get("version"), int) and isinstance(setup_state.get("catalog"), int)
          and bool(setup_steps) and not setup_unnamed, json.dumps(setup_unnamed or r)[:300], point="B1 setup.get")
    if methods is not None and "robot.geometry.get" in methods:
        r = rpc("robot.geometry.get", None, "setup_geometry")
        geometry_platform = ((r or {}).get("result") or {}).get("platform")
        check("setup.get names the platform of robot.geometry.get",
              bool(geometry_platform) and setup_state.get("platform") == geometry_platform,
              f'{setup_state.get("platform")} / {geometry_platform}', point="B1 setup.get")
    else:
        skip("setup.get names the platform of robot.geometry.get", "robot.geometry.get not in rpc.methods",
             point="B1 setup.get")
    # The one that came on subscribe; a live one reaches a subscribed client without the retain flag.
    setup_kept = [p for _, top, p, ret in msgs if top == "setup/json" and ret]
    setup_sent = json.loads(setup_kept[-1]) if setup_kept else {}
    check("setup/json retained with the platform, catalog and steps of setup.get",
          bool(setup_kept) and bool(setup_steps)
          and all(setup_sent.get(k) == setup_state.get(k) for k in ("platform", "catalog"))
          and [s.get("id") for s in setup_sent.get("steps") or []] == [s.get("id") for s in setup_steps],
          json.dumps({k: setup_sent.get(k) for k in ("platform", "catalog", "version")}), point="B1 setup/json")
    # An old version is refused before anything else; the state may move by itself in between, then once more.
    setup_first = setup_steps[0].get("id") if setup_steps else "none"
    for _ in range(2):
        setup_version = setup_state.get("version") if isinstance(setup_state.get("version"), int) else 0
        r = rpc("setup.start", {"step": setup_first, "version": setup_version - 1}, "setup_start_old")
        setup_error = (r or {}).get("error") or {}
        setup_data = setup_error.get("data") or {}
        if setup_data.get("version") == setup_version:
            break
        setup_state = (rpc("setup.get", None, "setup_get_again") or {}).get("result") or {}
    check("setup.start with an old version refused -32600 SETUP_REFUSED VERSION with the version of now",
          setup_error.get("code") == -32600 and setup_data.get("event") == "SETUP_REFUSED"
          and setup_data.get("reason") == "VERSION" and setup_data.get("version") == setup_version,
          json.dumps(r)[:300], point="B1 setup refused")
    setup_image = next((s["image"] for s in setup_steps if isinstance(s.get("image"), dict)), None)
    if setup_image:
        r = rpc("setup.image", {"name": setup_image.get("name")}, "setup_image")
        setup_piece = (r or {}).get("result") or {}
        setup_bytes = base64.b64decode(setup_piece.get("data") or "")
        setup_hash = hashlib.sha256(setup_bytes).hexdigest()
        check("setup.image: the sha256 of the data is that of the answer and of the step",
              setup_piece.get("offset") == 0 and setup_piece.get("size") == len(setup_bytes) == setup_image.get("bytes")
              and setup_hash == setup_piece.get("sha256") == setup_image.get("sha256")
              and setup_piece.get("type") == "image/svg+xml",
              f'{setup_image.get("name")}: {setup_hash} / {setup_piece.get("sha256")} / {setup_image.get("sha256")}',
              point="B1 setup.image")
    else:
        skip("setup.image", "no step names an image", point="B1 setup.image")
else:
    skip("setup.get", "not in rpc.methods: the platform has no step list", point="B1 setup.get")
if args.target == "mowbite-os" and offers("schedule.get"):
    # FP22: the schedule; where the platform does not allow it, it says so and cannot be switched on.
    r = rpc("schedule.get", None, "schedule_get")
    got = (r or {}).get("result") or {}
    check("schedule.get answers schema, version, available, reason, clock, next_start, schedule and sun",
          got.get("schema") == 1 and isinstance(got.get("available"), bool) and
          got.get("reason") == (None if got.get("available") else "platform_off") and
          isinstance(got.get("schedule"), dict) and isinstance(got.get("sun"), dict) and "next_start" in got,
          json.dumps(r)[:300])
    r = rpc("schedule.log", {"limit": 5}, "schedule_log")
    check("schedule.log answers entries", isinstance(((r or {}).get("result") or {}).get("entries"), list),
          json.dumps(r)[:200])
    if args.write and got.get("available") is False:
        r = rpc("schedule.set", {"schedule": {"enabled": 1}}, "schedule_on")
        error = (r or {}).get("error") or {}
        check("schedule.set refuses enabled 1 with platform_off", error.get("code") == -32602 and
              error.get("data") == {"field": "enabled", "reason": "platform_off"}, json.dumps(r)[:200])
CALIBRATION_METHODS = ["calibration.check", "calibration.start", "calibration.abort"]
calibration_offered = [m for m in CALIBRATION_METHODS if methods is not None and m in methods]
if args.target == "mowbite-os":
    # The self-measurement drive: offered only while its program is ready; check, start and abort come together.
    check("calibration.check, .start and .abort only together", len(calibration_offered) in (0, 3),
          ",".join(calibration_offered))
    if len(calibration_offered) == 3:
        r = rpc("calibration.check", None, "cal_check")
        answer = (r or {}).get("result") or {}
        problems = answer.get("problems")
        check("calibration.check answers ok, problems and space", sorted(answer) == ["ok", "problems", "space"] and
              isinstance(answer.get("ok"), bool) and isinstance(problems, list) and answer["ok"] == (not problems),
              json.dumps(r)[:300])
        check("calibration.check names every problem with reason and message", isinstance(problems, list) and all(
            isinstance(p, dict) and isinstance(p.get("reason"), str) and p["reason"] == p["reason"].upper() and
            bool(p["reason"]) and isinstance(p.get("message"), str) and bool(p["message"]) for p in problems),
              json.dumps(problems)[:300])
        space = answer.get("space")
        check("calibration.check: space is null or has length, width, lane_length and heading", space is None or (
            isinstance(space, dict) and sorted(space) == ["heading", "lane_length", "length", "width"] and
            all(isinstance(v, (int, float)) for v in space.values())), json.dumps(space))
    else:
        skip("calibration.check", "not in rpc.methods: the program is not ready")
    if methods is not None and "calibration.result" in methods:
        # null before the first drive, else the evaluation with a status, a summary and the values to take over.
        r = rpc("calibration.result", None, "cal_result")
        stored = (r or {}).get("result", "absent")
        check("calibration.result is null or a result with status, summary and values", stored is None or (
            isinstance(stored, dict) and stored.get("status") in ("done", "not_evaluated", "failed") and
            isinstance(stored.get("summary"), str) and isinstance(stored.get("values"), list) and
            isinstance(stored.get("station_again"), bool) and "recording" in stored), json.dumps(stored)[:300])
    else:
        skip("calibration.result", "not in rpc.methods")
r = rpc("no.such.method", None, "3")
check("unknown method -32601", r and r.get("error", {}).get("code") == -32601, json.dumps(r and r.get("error")))
c.publish("rpc/request", "{broken")
r = wait_for(lambda: (latest("rpc/response") and json.loads(latest("rpc/response")).get("error", {}).get("code") == -32700))
check("invalid json -32700", bool(r))


def check_mqtt5():
    """AS4 with --mqtt 5: system.info.mqtt, refused response topics, correlation data up to 1 KB."""
    r = rpc("system.info", None, "mqtt5-info")
    mqtt_version = ((r or {}).get("result") or {}).get("mqtt")
    check(f"system.info.mqtt is {args.bridge_mqtt}", mqtt_version == args.bridge_mqtt,
          json.dumps(r and r.get("result")), point="AS4 system.info.mqtt")
    # A command where it may act (the simulation); a bridge on 3.1.1 runs it, in IDLE it is refused with WRONG_STATE.
    method, params = ("mowing.pause", {"client": "check"}) if args.write else ("rpc.ping", None)
    for i, topic in enumerate(["map/json", "robot_state/json", "rpc/response", "rpc/response/a/b", "rpc/response/+",
                               "rpc/response/" + "t" * 65]):
        rid = f"bad-topic-{i}"
        start = len(msgs)
        r = rpc(method, params, rid, response_topic=topic)
        time.sleep(0.5 if i else 1.5)
        seen = []
        for _, top, p, _ in msgs[start:]:
            if top == "rpc/request":
                continue
            try:
                if json.loads(p).get("id") == rid:
                    seen.append(top)
            except (ValueError, AttributeError):
                pass
        if args.bridge_mqtt == 5:
            error = (r or {}).get("error") or {}
            data = error.get("data") or {}
            COMMAND_ANSWERS.append((method, r))
            check(f"response topic {topic[:24]} refused on rpc/response, nothing done",
                  error.get("code") == -32600 and data.get("event") == "REQUEST_REFUSED"
                  and data.get("reason") == "RESPONSE_TOPIC" and seen == ["rpc/response"],
                  f"{json.dumps(r)[:200]} on {seen}", point="AS4 response topic")
        else:
            # A bridge on 3.1.1 does not see the response topic: it answers as before.
            check(f"response topic {topic[:24]} unseen by a 3.1.1 bridge", r is not None and seen == ["rpc/response"],
                  f"{json.dumps(r)[:200]} on {seen}", point="AS4 fallback")
    if args.bridge_mqtt == 5:
        most = b"\x00k" * 512
        start = len(msgs)
        r = rpc("rpc.ping", None, "corr-most", correlation=most)
        check("1024 bytes of correlation data come back", (r or {}).get("result") == "pong" and routes
              and routes[-1][3] == most, json.dumps(r), point="AS4 correlation")
        r = rpc("rpc.ping", None, "corr-long", correlation=most + b"k")
        data = ((r or {}).get("error") or {}).get("data") or {}
        back = [correlations.get(i) for i in range(start, len(msgs)) if msgs[i][1] == RESPONSE
                and json.loads(msgs[i][2]).get("id") == "corr-long"]
        check("1025 bytes of correlation data refused to the response topic",
              data.get("reason") == "CORRELATION_DATA" and data.get("limit") == 1024 and back == [None],
              json.dumps(r)[:200], point="AS4 correlation")
        COMMAND_ANSWERS.append(("rpc.ping", r))


def check_routes():
    """AS4 with --mqtt 5: every answer rpc() took came only to the response topic with its correlation data, or with a
    bridge on 3.1.1 to rpc/response without; nothing of them on rpc/response."""
    if args.bridge_mqtt == 5:
        # Correlation data over 1 KB is refused and not sent back.
        wrong = [f"{rid}: {top}" for rid, top, sent, back in routes
                 if top != RESPONSE or back != (None if rid == "corr-long" else sent)]
        own = {json.dumps(rid) for rid, _, _, _ in routes}
        leaked = []
        for _, top, p, _ in msgs:
            if top != "rpc/response":
                continue
            rid = json.loads(p).get("id")
            if rid is not None and not str(rid).startswith("bad-topic-") and json.dumps(rid) in own:
                leaked.append(json.dumps(rid))
        check(f"every answer ({len(routes)}) only on {RESPONSE} with its correlation data", routes and not wrong
              and not leaked, "; ".join(wrong[:3] + leaked[:3]), point="AS4 routing")
    else:
        wrong = [f"{rid}: {top}" for rid, top, _, back in routes if top != "rpc/response" or back is not None]
        check(f"every answer ({len(routes)}) of the 3.1.1 bridge on rpc/response", routes and not wrong,
              "; ".join(wrong[:3]), point="AS4 fallback")


if args.mqtt == 5:
    check_mqtt5()


def late_phone_gets_map(name, expected):
    late = []
    phone = mqtt.Client("late_phone")
    if args.user:
        phone.username_pw_set(args.user, args.password)
    phone.on_message = lambda _c, _u, m: late.append((m.topic, m.payload, m.retain))
    phone.connect(args.host, args.port)
    phone.subscribe("map/json")
    phone.loop_start()
    time.sleep(1)
    phone.loop_stop()
    phone.disconnect()
    check(name, any(t == "map/json" and ret and json.loads(p) == expected for t, p, ret in late), f"{len(late)} messages")


if latest("map/json") is not None:
    late_phone_gets_map("a phone connecting later gets the retained map", json.loads(latest("map/json")))
else:
    skip("a phone connecting later gets the retained map", "the mower has no map")

if not args.write:
    finish()

# From here on the check acts on the simulation: values of its start state, then map, actions, teleop and flags.
check("robot_state compact", b'"current_state":"IDLE"' in (state or b""), (state or b"")[:160].decode())
check("robot_state values", s["emergency"] == 0 and s["is_charging"] == 0 and abs(s["battery_percentage"] - 0.8) < 1e-6,
      f'emergency={s["emergency"]} charging={s["is_charging"]} battery={s["battery_percentage"]}')
check("standing since the start: heading not aligned, GPS 0", s["pose"]["heading_valid"] == 0 and s["gps_percentage"] == 0,
      f'heading_valid={s["pose"]["heading_valid"]} gps={s["gps_percentage"]} acc={s["pose"]["pos_accuracy"]}')
check("sensor value 24.5", v == b"24.500000", str(v))
cs = wait_for(lambda: latest("sensors/om_charge_state/data"))
check("charge state", cs == b"Not Charging", str(cs))
check("actions enabled at the start", by_id.get("mower_logic/reset_emergency") == 1
      and by_id.get("mower_logic:idle/start_area_recording") == 1 and by_id.get("mower_logic:idle/start_mowing") == 0
      and by_id.get("mower_logic:undocking/abort_undocking") == 0
      and by_id.get("mower_logic:docking/abort_docking") == 0, f"{len(actions)} actions")


def events_of(kind, since=0):
    return [json.loads(p) for _, top, p, _ in msgs[since:] if top == "events/json" and json.loads(p)["type"] == kind]


def fixed_reason(event, resume=None):
    """masterplan 3.1a: a name in capitals, an English message, severity and resume."""
    return (bool(event) and isinstance(event.get("reason"), str) and event["reason"].replace("_", "").isalpha()
            and event["reason"].isupper() and bool(event.get("message"))
            and event.get("severity") in ("info", "warning", "error")
            and event.get("resume") == (resume or event.get("resume")) and event.get("resume") in ("auto", "user"))


def start_refused():
    """The reason actions/json shows for start_mowing, then the MOWING_REFUSED events of a start."""
    shown = next((a.get("reason") for a in json.loads(latest("actions/json") or "[]")
                  if a["action_id"] == "mower_logic:idle/start_mowing"), None)
    mark = len(msgs)
    c.publish("action", "mower_logic:idle/start_mowing")
    return shown, wait_for(lambda: events_of("MOWING_REFUSED", mark), 3)


shown, refused = start_refused()
if refused and shown != refused[-1].get("reason"):
    # The conditions changed between the two (the first fix of the simulation): once more after they settle.
    time.sleep(1.0)
    shown, refused = start_refused()
check("MOWING_REFUSED with a fixed reason", fixed_reason(refused[-1] if refused else None, "user"),
      json.dumps(refused[-1] if refused else None), point="3.1a MOWING_REFUSED")
check("start_mowing showed the reason of its MOWING_REFUSED", bool(refused) and shown == refused[-1].get("reason"),
      f"shown {shown}, refused {refused[-1].get('reason') if refused else None}", point="I2 start_mowing")
action_refusal = refused[-1] if refused else {}

# AS2: every command as an RPC with an answer. The commands of the logic are those of system.api that take client.
COMMANDS = [] if API_SCHEMA is None else [
    name for name in API_SCHEMA.get("x-commands", {})
    if "client" in API_SCHEMA["$defs"].get(name + ".params", {}).get("properties", {})]


def command(method, params=None, rid=None, timeout=5.0):
    r = rpc(method, params, rid or "cmd-" + method, timeout)
    COMMAND_ANSWERS.append((method, r))
    return r


def refused_with(r, event, reason, sent=None, since=0):
    """-32600 with data {event, reason, message, severity, resume, state, sub_state}; sent: the event went out too,
    with the id in event_id and the client check."""
    error = (r or {}).get("error") or {}
    data = error.get("data") or {}
    ok = (error.get("code") == -32600 and data.get("event") == event and data.get("reason") == reason
          and data.get("message") == error.get("message") and data.get("severity") in ("info", "warning", "error")
          and data.get("resume") in ("auto", "user") and isinstance(data.get("state"), str)
          and isinstance(data.get("sub_state"), str))
    if sent is None:
        return ok
    if not sent:
        return ok and "event_id" not in data
    found = wait_for(lambda: [e for e in events_of(event, since) if e["id"] == data.get("event_id")], 3)
    return ok and bool(found) and found[0].get("reason") == reason and found[0].get("client") == "check"


if args.target == "mowbite-os" and COMMANDS:
    missing = [m for m in COMMANDS if m != "docking.start" and (methods is None or m not in methods)]
    check("rpc.methods names every command of system.api", not missing and len(COMMANDS) >= 19,
          f"{len(COMMANDS)} commands, missing {missing}", point="AS2 commands")
    # The same refusal as the action, with the id of its event, and a number id back as a number.
    mark = len(msgs)
    r = command("mowing.start", {"client": "check"}, 4711)
    check("mowing.start refused like start_mowing, with the id of its MOWING_REFUSED",
          bool(r) and r.get("id") == 4711 and refused_with(r, "MOWING_REFUSED", action_refusal.get("reason"), True,
                                                           mark), json.dumps(r)[:400], point="AS2 refusals")
    # Commands that do not fit IDLE: WRONG_STATE only in the answer, and the actions as silent as before.
    wrong = ["mowing.pause", "mowing.continue", "mowing.abort", "mowing.skip_area", "mowing.skip_path",
             "undocking.abort", "docking.abort", "recording.exit", "recording.ring_start", "recording.discard"]
    mark = len(msgs)
    bad = [m for m in wrong if not refused_with(command(m, {"client": "check"}), "COMMAND_REFUSED", "WRONG_STATE",
                                                False)]
    for action in ("mower_logic:mowing/pause", "mower_logic:mowing/skip_area", "mower_logic:docking/abort_docking",
                   REC + "start_recording"):
        c.publish("action", action)
    time.sleep(1.0)
    seen = [json.loads(p)["type"] for _, top, p, _ in msgs[mark:] if top == "events/json"]
    check("WRONG_STATE in IDLE without an event, by RPC and by action", not bad and not seen
          and json.loads(latest("robot_state/json") or "{}").get("current_state") == "IDLE",
          f"refused otherwise: {bad}, events: {seen}", point="AS2 WRONG_STATE")
    r = command("mowing.reset_job", {"client": "check"})
    check("mowing.reset_job without a job: NO_JOB", refused_with(r, "COMMAND_REFUSED", "NO_JOB", False),
          json.dumps(r)[:300], point="AS2 NO_JOB")
    # params that do not fit: -32602 with the field, nothing happens.
    for method, params, field in (("mowing.start", {"areas": ["lawn"]}, "areas"),
                                  ("mowing.start", {"client": "Check"}, "client"),
                                  ("recording.save", {"type": "pond"}, "type"),
                                  ("recording.auto_points", {}, "enabled"),
                                  ("mowing.pause", {"speed": 1}, "speed")):
        r = rpc(method, params, "bad-" + field)
        error = (r or {}).get("error") or {}
        check(f"{method} {json.dumps(params)}: -32602 with field {field}", error.get("code") == -32602
              and (error.get("data") or {}).get("field") == field, json.dumps(r), point="AS2 params")
    check("still IDLE after the refused commands",
          json.loads(latest("robot_state/json") or "{}").get("current_state") == "IDLE", "", point="AS2 params")
elif args.target == "mowbite-os":
    check("commands of system.api", False, "system.api names none", point="AS2 commands")


if methods is not None and "params.set" in methods:
    # In IDLE: a value out of range goes to the bound and shows in params/json, an unknown name changes nothing.
    wait = json.loads(latest("params/json"))["/mower_logic/gps_wait_time"]
    r = rpc("params.set", {"params": {"/mower_logic/gps_wait_time": 99}}, "pset")
    check("params.set answers the value taken", r and r.get("result") == {"/mower_logic/gps_wait_time": 60.0},
          json.dumps(r))
    shown = wait_for(lambda: json.loads(latest("params/json")).get("/mower_logic/gps_wait_time") == 60.0)
    check("params.set shows in params/json", bool(shown), (latest("params/json") or b"")[:200].decode())
    r = rpc("params.set", {"params": {"/mower_logic/gps_wait_time": 5, "/mower_logic/tool_width": 0.2}}, "pset_bad")
    check("params.set refuses a planning value -32602", bool(r) and r.get("error", {}).get("code") == -32602,
          json.dumps(r and r.get("error")))
    r = rpc("params.set", {"params": {"/mower_logic/gps_wait_time": wait}}, "pset_back")
    check("params.set back to the start value", r and r.get("result") == {"/mower_logic/gps_wait_time": wait},
          json.dumps(r))

if methods is not None and "robot.geometry.set" in methods:
    # In IDLE: a value changes the answer, params/json and an event, one out of the range or with an old version is
    # refused, and what the user had comes back.
    r = rpc("robot.geometry.get", None, "geo_start")
    start = (r or {}).get("result") or {}
    blade = start.get("settings", {}).get("blade_ahead", {})
    version = start.get("version")
    new_blade = round(blade.get("value", 0.0) + 0.02, 3)
    mark = len(msgs)
    r = rpc("robot.geometry.set", {"version": version, "settings": {"blade_ahead": new_blade}}, "geo_set")
    changed = (r or {}).get("result") or {}
    check("robot.geometry.set answers like get with the new value and version",
          changed.get("version") == version + 1 and
          changed.get("settings", {}).get("blade_ahead", {}).get("value") == new_blade and
          changed["settings"]["blade_ahead"].get("stored") is True and
          changed["settings"]["blade_ahead"].get("default") == blade.get("default"), json.dumps(r)[:200])
    shown = wait_for(
        lambda: json.loads(latest("params/json") or "{}").get("/mowbite_geometry/blade_ahead") == new_blade)
    check("robot.geometry.set shows in params/json", bool(shown), "")
    ev = wait_for(lambda: events_of("GEOMETRY_CHANGED", mark))
    check("GEOMETRY_CHANGED after robot.geometry.set", bool(ev) and ev[-1].get("changed") == ["blade_ahead"] and
          ev[-1].get("version") == version + 1 and ev[-1].get("severity") == "info" and "reason" not in ev[-1] and
          "resume" not in ev[-1] and ev[-1].get("below_default") == [] and bool(ev[-1].get("message")),
          json.dumps(ev and ev[-1]), point="3.1a GEOMETRY_CHANGED")
    r = rpc("robot.geometry.set", {"version": version, "settings": {"blade_ahead": blade.get("value")}}, "geo_old")
    check("robot.geometry.set with an old version -32602 VERSION",
          bool(r) and r.get("error", {}).get("code") == -32602 and
          r["error"].get("data", {}).get("reason") == "VERSION" and r["error"]["data"].get("version") == version + 1,
          json.dumps(r and r.get("error")))
    r = rpc("robot.geometry.set", {"version": version + 1, "settings": {"robot_width": 47}}, "geo_range")
    check("robot.geometry.set refuses a value out of the range -32602 OUT_OF_RANGE", bool(r) and
          r.get("error", {}).get("code") == -32602 and r["error"].get("data", {}).get("reason") == "OUT_OF_RANGE" and
          r["error"]["data"].get("key") == "robot_width", json.dumps(r and r.get("error")))
    r = rpc("robot.geometry.set", {"version": version + 1, "settings": {"wheel_radius": 0.1}}, "geo_track")
    check("robot.geometry.set refuses the wheel radius of the platform -32602 NOT_SETTABLE", bool(r) and
          r.get("error", {}).get("code") == -32602 and r["error"].get("data", {}).get("reason") == "NOT_SETTABLE",
          json.dumps(r and r.get("error")))
    # Back to what it was: the user's own value, or the platform's.
    back = {"settings": {"blade_ahead": blade.get("value")}} if blade.get("stored") else {"reset": ["blade_ahead"]}
    r = rpc("robot.geometry.set", {"version": version + 1, **back}, "geo_back")
    check("robot.geometry.set back to the start value", ((r or {}).get("result") or {}).get("settings", {}).get(
        "blade_ahead", {}).get("value") == blade.get("value"), json.dumps(r)[:200])

# Test access only, the broker of the simulation is open. Port 9 of the simulation's own host has no caster: the client
# only waits.
TEST_ACCESS = {"host": "127.0.0.1", "port": 9, "mountpoint": "/CHECK", "user": "check", "password": "check-only"}
ntrip_write = methods is not None and "ntrip.set" in methods
if ntrip_write and ((rpc("ntrip.get", None, "ntrip_before") or {}).get("result") or {}).get("configured") is not False:
    skip("ntrip.set", "the simulation has an NTRIP access, it stays as it is")
    ntrip_write = False
if ntrip_write:
    setup_seen = sum(top == "setup/json" for _, top, _, _ in msgs)
    r = rpc("ntrip.set", TEST_ACCESS, "ntrip_set")
    check("ntrip.set answers like ntrip.get", (r or {}).get("result") == {
        "configured": True, "host": "127.0.0.1", "port": 9, "mountpoint": "CHECK", "version": 2, "has_user": True,
        "has_password": True}, json.dumps(r))
    if "setup.get" in setup_offered:
        # The step of the corrections sees the access: the state goes out again, within its 0.5 s.
        wait_for(lambda: sum(top == "setup/json" for _, top, _, _ in msgs) > setup_seen, 2.0)
        r = rpc("setup.get", None, "setup_access")
        setup_texts = [json.dumps(r).encode()] + [p for _, top, p, _ in msgs if top == "setup/json"]
        setup_secret = TEST_ACCESS["password"].encode()
        check("the password of the NTRIP access is not in setup.get or setup/json",
              bool(r) and not any(setup_secret in text for text in setup_texts), point="B1 setup access")
    r = rpc("ntrip.set", {"host": "localhost", "port": 9, "mountpoint": "CHECK"}, "ntrip_move")
    check("ntrip.set keeps no login for another caster -32602", bool(r) and r.get("error", {}).get("code") == -32602,
          json.dumps(r and r.get("error")))
    r = rpc("ntrip.set", dict(TEST_ACCESS, pin="1234"), "ntrip_unknown")
    check("ntrip.set refuses an unknown field -32602 without naming it", bool(r)
          and r.get("error", {}).get("code") == -32602 and "pin" not in r["error"].get("message", ""),
          json.dumps(r and r.get("error")))
    # Gone again at once: with an access file RTK fixed counts only after corrections from NTRIP (G24), which the
    # test caster never sends, and the checks below need a good position.
    r = rpc("ntrip.delete", None, "ntrip_delete")
    check("ntrip.delete in IDLE", (r or {}).get("result") == {"configured": False}, json.dumps(r))
    off = wait_for(lambda: latest("sensors/os_ntrip_state/data") == b"off", 8)
    check("os_ntrip_state off after ntrip.delete", bool(off), str(latest("sensors/os_ntrip_state/data")))


HOLD_ANSWER = {"state", "is_charging", "hold_until", "sub_state", "idle", "emergency_reason", "next_start",
               "scheduled_start"}


def hold_refused(r):
    """-32004 with data {"reason": "HOLD", "until"}: no change of data while an update holds the mower."""
    error = (r or {}).get("error") or {}
    data = error.get("data") or {}
    return error.get("code") == -32004 and data.get("reason") == "HOLD" and data.get("until") == hold_until


if methods is not None and "system.hold" in methods:
    # Updater contract 3.5: in IDLE the hold is granted, nothing starts and no data changes while it lasts, 0 lets go.
    mark = len(msgs)
    r = rpc("system.hold", {"seconds": 120}, "hold")
    answer = (r or {}).get("result") or {}
    hold_until = answer.get("hold_until")
    check("system.hold in IDLE: hold_until about now + 120 and the state for the gate",
          set(answer) == HOLD_ANSWER and answer.get("state") == "IDLE" and answer.get("idle") is True
          and isinstance(hold_until, int) and abs(hold_until - (time.time() + 120)) < 5
          and isinstance(answer.get("is_charging"), bool) and isinstance(answer.get("emergency_reason"), int),
          json.dumps(r), point="3.5 system.hold")
    mirrored = wait_for(lambda: latest("mowbite/hold"), 5)
    check("mowbite/hold retained with until", bool(mirrored) and json.loads(mirrored) == {"until": hold_until}
          and any(top == "mowbite/hold" and p for _, top, p, _ in msgs[mark:]), str(mirrored), point="3.5 mowbite/hold")
    off = wait_for(lambda: enabled("mower_logic:idle/start_area_recording") == 0, 3)
    check("start_area_recording and start_mowing off while held",
          bool(off) and enabled("mower_logic:idle/start_mowing") == 0, str(latest("actions/json"))[:120])
    held_reason = next((a.get("reason") for a in json.loads(latest("actions/json") or "[]")
                        if a["action_id"] == "mower_logic:idle/start_mowing"), None)
    check("start_mowing shows HOLD while held", held_reason == "HOLD", str(held_reason), point="I2 start_mowing")
    for action, kind in (("mower_logic:idle/start_mowing", "MOWING_REFUSED"),
                         ("mower_logic:idle/start_area_recording", "RECORDING_REFUSED")):
        mark = len(msgs)
        c.publish("action", action)
        refused = wait_for(lambda: events_of(kind, mark), 3)
        event = refused[-1] if refused else {}
        check(kind + " HOLD with until", fixed_reason(event, "user") and event.get("reason") == "HOLD"
              and event.get("until") == hold_until and event.get("severity") == "info", json.dumps(event),
              point="3.5 " + kind + " HOLD")
    for method, kind in (("mowing.start", "MOWING_REFUSED"), ("recording.start", "RECORDING_REFUSED")):
        if method not in methods:
            continue
        mark = len(msgs)
        r = command(method, {"client": "check"}, "hold_" + method)
        check(method + " refused -32600 HOLD with until and the id of its " + kind,
              refused_with(r, kind, "HOLD", True, mark)
              and ((r.get("error") or {}).get("data") or {}).get("until") == hold_until, json.dumps(r)[:300],
              point="AS2 HOLD")
    check("still IDLE after the refused starts",
          json.loads(latest("robot_state/json") or "{}").get("current_state") == "IDLE", "")
    if "docking.start" in methods:
        mark = len(msgs)
        r = command("docking.start", {"client": "check"}, "hold_dock")
        data = ((r or {}).get("error") or {}).get("data") or {}
        check("docking.start refused HOLD", data.get("reason") == "HOLD" and data.get("until") == hold_until
              and refused_with(r, "DRIVE_REFUSED", "HOLD", True, mark), json.dumps(r and r.get("error")))
    if "calibration.start" in methods:
        r = rpc("calibration.start", None, "hold_calibration")
        data = ((r or {}).get("error") or {}).get("data") or {}
        check("calibration.start refused HOLD", data.get("reason") == "HOLD" and data.get("until") == hold_until,
              json.dumps(r and r.get("error")))
    if "params.set" in methods:
        r = rpc("params.set", {"params": {"/mower_logic/gps_wait_time": 20}}, "hold_params")
        check("params.set refused -32004 HOLD", hold_refused(r), json.dumps(r and r.get("error")))
    if "robot.geometry.set" in methods:
        version = (((rpc("robot.geometry.get", None, "hold_geo_get") or {}).get("result")) or {}).get("version")
        r = rpc("robot.geometry.set", {"version": version, "settings": {"blade_ahead": 0.2}}, "hold_geo")
        check("robot.geometry.set refused -32004 HOLD", hold_refused(r), json.dumps(r and r.get("error")))
    map_before = latest("map/json")
    r = rpc("map.replace", [json.loads(map_before or '{"areas": [], "docking_stations": []}')], "hold_map")
    check("map.replace refused -32004 HOLD", hold_refused(r), json.dumps(r and r.get("error")))
    if "ntrip.set" in methods:
        r = rpc("ntrip.set", TEST_ACCESS, "hold_ntrip")
        check("ntrip.set refused -32004 HOLD", hold_refused(r), json.dumps(r and r.get("error")))
    r = rpc("system.hold", {"seconds": 300}, "hold_again")
    again = ((r or {}).get("result") or {}).get("hold_until")
    check("system.hold again replaces the hold", isinstance(again, int) and again >= hold_until + 170, json.dumps(r))
    mark = len(msgs)
    r = rpc("system.hold", {"seconds": 0}, "hold_off")
    check("system.hold 0 lets go", ((r or {}).get("result") or {}).get("hold_until") == 0, json.dumps(r))
    cleared = wait_for(lambda: any(top == "mowbite/hold" and p == b"" for _, top, p, _ in msgs[mark:]), 5)
    check("mowbite/hold empty after the hold", bool(cleared), str(latest("mowbite/hold")), point="3.5 mowbite/hold")
    back = wait_for(lambda: enabled("mower_logic:idle/start_area_recording") == 1, 3)
    check("start_area_recording on again after the hold", bool(back), "")
    r = rpc("params.set", {"params": {"/mower_logic/gps_wait_time": json.loads(latest("params/json"))[
        "/mower_logic/gps_wait_time"]}}, "hold_params_after")
    check("params.set after the hold", bool(r) and "result" in r, json.dumps(r))


def planner_services():
    listed = subprocess.run(["ros2", "service", "list"], capture_output=True, text=True, timeout=60).stdout.split()
    return listed if "/mowbite_planner/plan_coverage" in listed and "/mowbite_planner/plan_transit" in listed else None


if shutil.which("ros2"):
    # discovery takes a moment after the start
    services = wait_for(planner_services, 40) or []
    check("planner process offers plan_coverage and plan_transit", bool(services), "")
else:
    skip("planner process offers plan_coverage and plan_transit", "needs the ros2 command on the same machine")


# The map the way the app saves it: open rings, settings of its own, a property the mower does not know.
APP_MAP = {
    "areas": [
        {"id": "lawn", "properties": {"name": "Front", "type": "mow", "angle": 0.35, "planner": {"fill_pattern": "lanes"}},
         "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}]},
        {"id": "tree", "properties": {"type": "obstacle", "pin": True},
         "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}]},
    ],
    "docking_stations": [{"id": "dock", "position": {"x": -1.5, "y": 3.0}, "heading": 3.14}],
}
r = rpc("map.replace", [APP_MAP], "5")
check("map.replace stores", r and r.get("result") == "Successfully stored map (2 areas)", json.dumps(r))
published = wait_for(lambda: (latest("map/json") if b'"lawn"' in (latest("map/json") or b"") else None))
saved = json.loads(published) if published else {"areas": [], "docking_stations": []}
check("map/json: rings closed, unknown properties kept",
      len(saved["areas"]) == 2 and len(saved["areas"][0]["outline"]) == 5
      and saved["areas"][0]["outline"][0] == saved["areas"][0]["outline"][-1]
      and saved["areas"][0]["properties"].get("planner") == {"fill_pattern": "lanes"}
      and saved["areas"][1]["properties"].get("pin") is True, json.dumps(saved)[:200])
bad = json.loads(json.dumps(APP_MAP))
bad["areas"][0]["outline"] = bad["areas"][0]["outline"][:2]
r = rpc("map.replace", [bad], "6")
check("invalid map -32602 with the reason", r and r["error"]["code"] == -32602
      and r["error"]["message"] == "Invalid map: areas[0] 'lawn': outline needs at least 3 points", json.dumps(r and r.get("error")))
r = rpc("map.replace", None, "7")
check("map.replace without params -32602", r and r["error"]["code"] == -32602, json.dumps(r and r.get("error")))
# mowing.plan: the planner process plans the area of the map, the way the app asks for it.
r = rpc("mowing.plan", {"area_id": "lawn"}, "8", 30)
plan = (r or {}).get("result") or {}
paths = plan.get("paths") or []
points = [p for path in paths for p in path["points"]]
TREE = [(4, 2), (5, 2), (5, 3)]


def in_polygon(point, ring):
    x, y = point
    hit = False
    for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            hit = not hit
    return hit


odd = [p for p in points if not (-0.01 <= p[0] <= 10.01 and -0.01 <= p[1] <= 6.01) or in_polygon(p, TREE)]
check("mowing.plan: loops first, then lanes", bool(paths) and paths[0]["is_outline"] is True and paths[-1]["is_outline"] is False
      and plan.get("area_id") == "lawn", json.dumps(r)[:200])
check("mowing.plan: the angle of the area", abs(plan.get("angle", 9) - 0.35) < 1e-6, str(plan.get("angle")))
check("mowing.plan: points inside the lawn, none in the tree", bool(points) and not odd,
      f"{len(points)} points, {len(odd)} odd, e.g. {odd[:3]}")
check("mowing.plan: pose_index counts the poses of each path", bool(paths) and all(
    len(p["pose_index"]) == len(p["points"]) >= 2 and p["pose_index"][0] == 0
    and all(a < b for a, b in zip(p["pose_index"], p["pose_index"][1:])) for p in paths))
check("mowing.plan: fingerprint", len(plan.get("fingerprint", "")) == 64, plan.get("fingerprint", ""))
again = (rpc("mowing.plan", {"area_id": "lawn"}, "9", 30) or {}).get("result") or {}
check("mowing.plan: the same plan again", again.get("fingerprint") == plan.get("fingerprint") and again.get("paths") == paths)
r = rpc("mowing.plan", {"outline": [{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 3}, {"x": 0, "y": 3}], "obstacles": [],
                        "outline_count": 1, "angle": 0.0}, "10", 30)
free = (r or {}).get("result") or {}
check("mowing.plan: an outline in the request", bool(free.get("paths")) and free.get("area_id") == "", json.dumps(r)[:200])
if args.target == "mowbite-os" and offers("planner.settings"):
    # Only MowBite OS's schedule starts the mower: the planner's own leftovers_auto is off and cannot be set.
    r = rpc("planner.settings", None, "12", 10)
    entry = (((r or {}).get("result") or {}).get("settings") or {}).get("leftovers_auto") or {}
    check("planner.settings: leftovers_auto is off and not settable", entry.get("value") is False
          and entry.get("settable") is False, json.dumps(entry))
    r = rpc("planner.settings.set", {"leftovers_auto": True}, "13", 10)
    check("planner.settings.set refuses leftovers_auto", bool(r and r.get("error"))
          and "leftovers_auto" in r["error"].get("message", ""), json.dumps(r and (r.get("error") or r.get("result")))[:200])

    # One place for the planning values: what the planner is set to counts for a plan, mowbite_logic adds nothing.
    def outline_poses(rid):
        res = (rpc("mowing.plan", {"outline": [{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 3}, {"x": 0, "y": 3}],
                                   "obstacles": []}, rid, 30) or {}).get("result") or {}
        return sum(p["pose_index"][-1] + 1 for p in res.get("paths", []) if p["is_outline"])

    before = outline_poses("14")
    rpc("planner.settings.set", {"perimeter_passes": 1}, "15", 10)
    fewer = outline_poses("16")
    rpc("planner.settings.set", {"perimeter_passes": None}, "17", 10)
    check("planner.settings perimeter_passes counts for a plan of mowing.plan", 0 < fewer < before
          and outline_poses("18") == before, f"3 passes {before} poses, 1 pass {fewer}")

for params, message in ((None, "parameters must be an object"), ({"area_id": "nobody"}, "no mowing area with id: nobody"),
                        ({"area_id": "tree"}, "no mowing area with id: tree"),
                        ({"area_id": "lawn", "outline_count": 300}, "parameter is not a whole number from 0 to 255: outline_count")):
    r = rpc("mowing.plan", params, "11")
    check("mowing.plan refuses: " + message, r and r.get("error", {}).get("code") == -32602
          and r["error"]["message"] == message, json.dumps(r and r.get("error")))
late_phone_gets_map("a phone connecting later gets the retained map after map.replace", saved)

# Teleop does nothing in IDLE
t0 = sim_time()
for _ in range(5):
    teleop(0.2, 0.0)
    time.sleep(0.1)
time.sleep(0.3)
check_log("teleop ignored in IDLE", not sent_speeds(t0), str(sent_speeds(t0)))

c.publish("action", "mower_logic:idle/start_area_recording")
state = wait_for(lambda: b'"current_state":"AREA_RECORDING"' in (latest("robot_state/json") or b""), 3)
check("state AREA_RECORDING", bool(state))
if ntrip_write:
    r = rpc("ntrip.delete", None, "ntrip_recording")
    check("ntrip.delete only in IDLE -32600", bool(r) and r.get("error", {}).get("code") == -32600,
          json.dumps(r and r.get("error")))
check("first recording step: start_recording and record_dock", bool(wait_for(
    lambda: enabled(REC + "start_recording") == 1 and enabled(REC + "record_dock") == 1
    and enabled(REC + "stop_recording") == 0, 3)))
c.publish("action", REC + "start_recording")
check("start_recording: the step the app calls recording", bool(wait_for(
    lambda: enabled(REC + "stop_recording") == 1 and enabled(REC + "start_recording") == 0, 3)))
# Standing still there are too few points for a ring; it stays open and recording goes on.
mark = len(msgs)
c.publish("action", REC + "stop_recording")
refused = wait_for(lambda: events_of("RECORDING_REFUSED", mark), 3)
last = refused[-1] if refused else {}
check("RECORDING_REFUSED RING_FEW_POINTS with points and limit", fixed_reason(last, "user")
      and last.get("reason") == "RING_FEW_POINTS" and last.get("action") == "stop_recording" and last.get("limit") == 3
      and isinstance(last.get("points"), int), json.dumps(last), point="3.1a RECORDING_REFUSED")
# Like the app on a mode change: one zero, then the joystick.
teleop(0.0, 0.0)
t0 = sim_time()
for _ in range(15):
    teleop(0.7, 0.1)
    time.sleep(0.1)
teleop(0.0, 0.0)
time.sleep(1.0)
speeds = sent_speeds(t0)
moving = [s for t, s in speeds if s != [0.0, 0.0]]
zeros = [t for t, s in speeds if s == [0.0, 0.0]]
# The Tron's safety filter lets a command grow at 0.5 m/s2: from a stop up to the driver's 0.55 in about 1.1 s.
check_log("teleop drives, grows smoothly and is clamped to the driver's 0.55", len(moving) >= 12
          and moving[0][0] <= 0.06 and all(b[0] >= a[0] - 1e-9 for a, b in zip(moving, moving[1:]))
          and all(s[0] <= 0.55 + 1e-3 for s in moving) and abs(moving[-1][0] - 0.55) < 1e-3,
          f"{len(moving)} frames, {moving[:2]} .. {moving[-1:]}")
check_log("stop after teleop: 3 zeros", len(zeros) == 3, str(zeros))
line = wait_for(lambda: [p for p in overlay_lines("blue") if len(p["poly"]) >= 2], 3)
check("map_overlay: the open line as poly, in mm", bool(line) and line[0]["is_closed"] == 0
      and "polygon" not in line[0] and all(round(p["x"], 3) == p["x"] for p in line[0]["poly"]),
      json.dumps(line[0]) if line else str(latest("map_overlay/json"))[:200])

set_flags(lift=1)
state = wait_for(lambda: b'"emergency":1' in (latest("robot_state/json") or b""), 3)
check("lift -> emergency", bool(state))
ev = wait_for(lambda: [json.loads(p) for _, top, p, _ in msgs if top == "events/json" and json.loads(p)["type"] == "EMERGENCY"])
check("EMERGENCY event with reason bits", ev and ev[-1]["emergency"] is True and ev[-1]["reason"] & 8,
      json.dumps(ev[-1] if ev else None))
check("EMERGENCY with severity error and resume user", bool(ev) and ev[-1].get("severity") == "error"
      and ev[-1].get("resume") == "user", json.dumps(ev[-1] if ev else None), point="3.1a EMERGENCY")
s = json.loads(latest("robot_state/json"))
check("robot_state emergency_reason LATCH|LIFT", s.get("emergency_reason") == 9, str(s.get("emergency_reason")))


def reason_line():
    r = rpc("logs.recent", {"level": "WARN"}, "reason")
    lines = [e["msg"] for e in (r or {}).get("result", [])]
    return [l for l in lines if l == "Emergency reason changed from: '' to: 'LATCH, LIFT'"] or None


check("log line with the reason, as the app reads it", bool(wait_for(reason_line, 3)))
t0 = sim_time()
for _ in range(5):
    teleop(0.2, 0.0)
    time.sleep(0.1)
check_log("teleop blocked during emergency", not [s for t, s in sent_speeds(t0) if s != [0.0, 0.0]])
# The joystick is still held while the emergency is reset.
hold = Hold(0.2, 0.0)
c.publish("action", "mower_logic/reset_emergency")
time.sleep(0.5)
set_flags()
state = wait_for(lambda: b'"emergency":0' in (latest("robot_state/json") or b""), 4)
check("reset_emergency clears", bool(state))
check("emergency_reason 0 after reset", json.loads(latest("robot_state/json")).get("emergency_reason") == 0)
t0 = sim_time()
time.sleep(1.5)
check_log("held joystick does not drive after reset", not [s for t, s in sent_speeds(t0) if s != [0.0, 0.0]],
      str(sent_speeds(t0)[:3]))
hold.release()
t0 = sim_time()
for _ in range(5):
    teleop(0.2, 0.0)
    time.sleep(0.1)
teleop(0.0, 0.0)
time.sleep(0.5)
check_log("teleop drives again after a zero", len([s for t, s in sent_speeds(t0) if s != [0.0, 0.0]]) >= 4,
      str(sent_speeds(t0)[:3]))

# The safety filter latches TILT (bit 13) for a tilted mower and keeps it until the mower is level again.
set_flags(tilt_roll=35)
tilt = wait_for(lambda: [e for e in (json.loads(p) for _, top, p, _ in msgs if top == "events/json")
                         if e["type"] == "EMERGENCY" and e["emergency"] and e["reason"] == 8193], 5)
c.publish("action", "mower_logic/reset_emergency")
time.sleep(1.0)
kept = b'"emergency":1' in (latest("robot_state/json") or b"")
if "emergency.reset" in COMMANDS:
    # AS2: the RPC answers with the refusal of the safety node and the id of its event.
    mark = len(msgs)
    r = command("emergency.reset", {"client": "check"}, "reset_tilted")
    check("emergency.reset while tilted: -32600 EMERGENCY_RESET_REFUSED STILL_TILTED with its event",
          refused_with(r, "EMERGENCY_RESET_REFUSED", "STILL_TILTED", True, mark), json.dumps(r)[:400],
          point="AS2 emergency.reset")
set_flags()
time.sleep(0.5)
c.publish("action", "mower_logic/reset_emergency")
cleared = wait_for(lambda: b'"emergency":0' in (latest("robot_state/json") or b""), 4)
check("tilt -> EMERGENCY with TILT, kept while tilted, cleared once level", bool(tilt) and kept and bool(cleared),
      f"event {bool(tilt)}, kept {kept}, cleared {bool(cleared)}", point="3.1a EMERGENCY TILT")
refused = [e for e in (json.loads(p) for _, top, p, _ in msgs if top == "events/json")
           if e["type"] == "EMERGENCY_RESET_REFUSED"]
r = refused[0] if refused else {}
check("reset while tilted -> EMERGENCY_RESET_REFUSED STILL_TILTED with angle and limit in deg, warning/user",
      len(refused) == (2 if "emergency.reset" in COMMANDS else 1) and "client" not in r
      and r.get("reason") == "STILL_TILTED" and abs(r.get("angle", 0) - 35) < 1.5
      and r.get("limit") == 26 and r.get("severity") == "warning" and r.get("resume") == "user"
      and bool(r.get("message")), json.dumps(refused), point="3.1a EMERGENCY_RESET_REFUSED STILL_TILTED")

# Leaving the recording mode while the joystick is held stops at once, not after the 0.5 s watchdog. 3 s of driving
# give the course from the positions (0.1 m of straight way) and the run of courses that aligns the heading.
t0 = sim_time()
hold = Hold(0.2, 0.0)
time.sleep(3.0)
c.publish("action", "mower_logic:area_recording/exit_recording_mode")
time.sleep(1.0)
hold.release()
time.sleep(0.3)
speeds = sent_speeds(t0)
last_moving = max((t for t, s in speeds if s != [0.0, 0.0]), default=None)
zeros = [t for t, s in speeds if s == [0.0, 0.0] and last_moving is not None and t > last_moving]
check_log("exit_recording_mode stops at once: 3 zeros within 0.3 s", last_moving is not None and len(zeros) == 3
      and zeros[0] - last_moving < 0.3, f"last moving {last_moving}, zeros {zeros}")
state = wait_for(lambda: b'"current_state":"IDLE"' in (latest("robot_state/json") or b""), 3)
check("state IDLE", bool(state))
check("map_overlay empty after leaving", bool(wait_for(
    lambda: latest("map_overlay/json") is not None and json.loads(latest("map_overlay/json"))["polygons"] == [], 3)))
s = json.loads(wait_for(lambda: latest("robot_state/json") if b'"heading_valid":1' in (latest("robot_state/json") or b"")
                        else None, 3) or latest("robot_state/json"))
check("heading aligned after driving, GPS above 0", s["pose"]["heading_valid"] == 1 and s["gps_percentage"] > 0.5,
      f'heading_valid={s["pose"]["heading_valid"]} gps={s["gps_percentage"]} acc={s["pose"]["pos_accuracy"]}')
clears = []
if LOG:
    with open(LOG) as f:
        clears = [json.loads(l) for l in f if '"mod": 10' in l and '"type": 0,' in l]
check_log("clear frame sent", len(clears) == 1 and clears[0]["hex"] == "00" * 8, str(clears))

# The driver keeps the cause of a latch until the reset, also after the bumper is let go.
set_flags(bumper=1)
state = wait_for(lambda: b'"emergency":1' in (latest("robot_state/json") or b""), 3)
check("bumper -> emergency", bool(state))
set_flags()
time.sleep(1.5)
s = json.loads(latest("robot_state/json"))
check("bumper let go: emergency and reason LATCH|COLLISION stay", s["emergency"] == 1 and s.get("emergency_reason") == 33,
      f'emergency={s["emergency"]} reason={s.get("emergency_reason")}')
mark = len(msgs)
c.publish("action", "mower_logic/reset_emergency")
state = wait_for(lambda: b'"emergency":0' in (latest("robot_state/json") or b""), 4)
check("reset after the bumper clears", bool(state) and json.loads(latest("robot_state/json")).get("emergency_reason") == 0)
if args.target == "mowbite-os":
    ended = wait_for(lambda: [e for e in events_of("EMERGENCY", mark) if e["emergency"] is False], 3)
    check("the end after reset_emergency: source rpc, no client", bool(ended) and ended[-1].get("source") == "rpc"
          and "client" not in ended[-1], json.dumps(ended), point="3.1a EMERGENCY source")
if "emergency.reset" in COMMANDS:
    set_flags(bumper=1)
    wait_for(lambda: b'"emergency":1' in (latest("robot_state/json") or b""), 3)
    set_flags()
    mark = len(msgs)
    r = command("emergency.reset", {"client": "check"}, "reset_bumper")
    check("emergency.reset answers once released: released, state", ((r or {}).get("result") or {}).get("released")
          is True and r["result"].get("state") == "IDLE", json.dumps(r), point="AS2 emergency.reset")
    ended = wait_for(lambda: [e for e in events_of("EMERGENCY", mark) if e["emergency"] is False], 3)
    check("the end after emergency.reset: source rpc and the client", bool(ended) and ended[-1].get("source") == "rpc"
          and ended[-1].get("client") == "check", json.dumps(ended), point="3.1a EMERGENCY source")

# Charging follows the debounced dock contact of the driver (3 battery reports at 1 Hz).
set_flags(dock=1)
state = wait_for(lambda: b'"is_charging":1' in (latest("robot_state/json") or b""), 6)
check("dock -> is_charging", bool(state))
cs = wait_for(lambda: latest("sensors/om_charge_state/data") == b"Fast Charge" or None, 3)
check("charge state Fast Charge", bool(cs), str(latest("sensors/om_charge_state/data")))
set_flags()
# The contact ends with the next battery report; the steps below start with the mower off the station.
state = wait_for(lambda: b'"is_charging":0' in (latest("robot_state/json") or b""), 6)
check("no contact -> not charging", bool(state))

if "recording.start" in COMMANDS:
    # AS2: the recording over its RPCs, as the actions did above.
    mark = len(msgs)
    r = command("recording.start", {"client": "check"})
    entered = wait_for(lambda: [e for e in events_of("STATE", mark) if e.get("state") == "AREA_RECORDING"], 3)
    check("recording.start answers AREA_RECORDING, STATE with the client", ((r or {}).get("result") or {}).get("state")
          == "AREA_RECORDING" and bool(entered) and entered[-1].get("client") == "check", json.dumps(r),
          point="AS2 recording")
    steps = [("recording.ring_start", {}), ("recording.auto_points", {"enabled": False}), ("recording.point", {}),
             ("recording.auto_points", {"enabled": True})]
    answers = [command(m, dict(p, client="check")) for m, p in steps]
    check("ring_start, auto_points off, point, auto_points on: accepted while recording",
          all((a or {}).get("result", {}).get("state") == "AREA_RECORDING" for a in answers),
          json.dumps(answers)[:400], point="AS2 recording")
    mark = len(msgs)
    r = command("recording.ring_stop", {"client": "check"})
    data = ((r or {}).get("error") or {}).get("data") or {}
    check("recording.ring_stop standing still: RECORDING_REFUSED RING_FEW_POINTS with its event and action",
          refused_with(r, "RECORDING_REFUSED", "RING_FEW_POINTS", True, mark)
          and data.get("action") == "stop_recording",
          json.dumps(r)[:400], point="AS2 recording")
    if "recording.undo" in COMMANDS and (data.get("points") or 0) >= 1:
        r = command("recording.undo", {"meters": 1, "client": "check"})
        check("recording.undo: points left, automatic points off",
              ((r or {}).get("result") or {}).get("points") == 0
              and bool(wait_for(lambda: enabled(REC + "auto_point_collecting_enable") == 1, 3)),
              json.dumps(r)[:300], point="AS2 recording")
    r = command("recording.discard", {"client": "check"})
    check("recording.discard", ((r or {}).get("result") or {}).get("state") == "AREA_RECORDING", json.dumps(r),
          point="AS2 recording")
    r = command("recording.save", {"type": "mow", "client": "check"})
    check("recording.save without an outline: WRONG_STATE", refused_with(r, "COMMAND_REFUSED", "WRONG_STATE", False)
          and r["error"]["data"].get("state") == "AREA_RECORDING", json.dumps(r)[:300], point="AS2 recording")
    mark = len(msgs)
    r = command("recording.exit", {"client": "check"})
    check("recording.exit answers IDLE", ((r or {}).get("result") or {}).get("state") == "IDLE"
          and bool(wait_for(lambda: b'"current_state":"IDLE"' in (latest("robot_state/json") or b""), 3)),
          json.dumps(r), point="AS2 recording")

# The heartbeat carries the local time of TZ (default of the start: Europe/Berlin), not UTC.
beats = []
if LOG:
    with open(LOG) as f:
        beats = [json.loads(l) for l in f if '"type": 255' in l]
b = bytes.fromhex(beats[-1]["hex"]) if beats else bytes(8)
zone = zoneinfo.ZoneInfo(os.environ.get("TZ") or "Europe/Berlin")
try:
    sent = datetime.datetime(2000 + b[0], b[1], b[2], b[3], b[4], b[5], tzinfo=zone)
    skew = abs((datetime.datetime.now(zone) - sent).total_seconds())
except ValueError:
    skew = None
check_log("heartbeat in local time", skew is not None and skew < 5, f"{b.hex()} in {zone}, skew {skew}")

# Docking onto the station of the simulation (sim.yaml: the docked mower at (-1, 0), backing in going west), from where
# the mower stands after the steps above.
if methods is not None and "docking.start" in methods:
    r = rpc("map.replace", [{"areas": APP_MAP["areas"], "docking_stations": []}], "dock_none_map")
    wait_for(lambda: b'"docking_stations": []' in (latest("map/json") or b"") or b'"docking_stations":[]' in
             (latest("map/json") or b""), 5)
    time.sleep(0.5)
    mark = len(msgs)
    r = rpc("docking.start", None, "dock_none")
    error = (r or {}).get("error") or {}
    data = error.get("data") or {}
    check("docking.start refused -32600 with reason and message in data", error.get("code") == -32600
          and data.get("reason") == "NO_DOCK" and data.get("message") == error.get("message"), json.dumps(r),
          point="3.1a docking.start")
    refused = wait_for(lambda: events_of("DRIVE_REFUSED", mark), 3)
    last = refused[-1] if refused else None
    check("docking.start refused: DRIVE_REFUSED with the same reason", bool(last) and last["reason"] == "NO_DOCK"
          and fixed_reason(last, "user"), json.dumps(last), point="3.1a DRIVE_REFUSED")
    YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
    DOCK_MAP = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                           "outline": [{"x": x, "y": y} for x, y in YARD]}],
                "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}
    r = rpc("map.replace", [DOCK_MAP], "dock_map")
    wait_for(lambda: b'"yard"' in (latest("map/json") or b""), 5)
    time.sleep(0.5)
    mark = len(msgs)
    r = command("docking.start", None, "dock_start")
    check("docking.start answers the state", (r or {}).get("result") == {"state": "DOCKING", "sub_state": ""},
          json.dumps(r))
    state = wait_for(lambda: b'"current_state":"DOCKING"' in (latest("robot_state/json") or b""), 3)
    check("robot_state DOCKING", bool(state))
    check("abort_docking enabled while docking",
          bool(wait_for(lambda: enabled("mower_logic:docking/abort_docking") == 1, 3)))
    docked = wait_for(lambda: events_of("DOCKED", mark) or events_of("DOCKING_FAILED", mark), 150)
    since = [json.loads(p)["type"] for _, top, p, _ in msgs[mark:] if top == "events/json"]
    check("DOCKED like OpenMower, without reason", bool(docked) and docked[-1]["type"] == "DOCKED"
          and "reason" not in docked[-1], json.dumps(docked[-1] if docked else since))
    states = [json.loads(p).get("state") for _, top, p, _ in msgs[mark:]
              if top == "events/json" and json.loads(p)["type"] == "STATE"]
    check("STATE DOCKING, then IDLE", states[:1] == ["DOCKING"] and bool(wait_for(
        lambda: b'"current_state":"IDLE"' in (latest("robot_state/json") or b""), 5)), ",".join(map(str, states)))
    # api 5: every way home carries its reason, the one of docking.start is MANUAL, before the state changes.
    going = [e for e in (json.loads(p) for _, top, p, _ in msgs[mark:] if top == "events/json")
             if e["type"] in ("DOCKING", "STATE")]
    first = going[0] if going else {}
    check("docking.start: DOCKING MANUAL before STATE DOCKING", first.get("type") == "DOCKING"
          and first.get("reason") == "MANUAL" and first.get("severity") == "info" and fixed_reason(first, "user")
          and len(going) > 1 and going[1].get("state") == "DOCKING",
          json.dumps(going[:2]), point="3.1a DOCKING")
    charging = wait_for(lambda: b'"is_charging":1' in (latest("robot_state/json") or b""), 6)
    check("docked -> is_charging", bool(charging))
    r = command("docking.start", None, "dock_there")
    data = ((r or {}).get("error") or {}).get("data") or {}
    check("docking.start on the contacts: ALREADY_THERE", data.get("reason") == "ALREADY_THERE"
          and refused_with(r, "DRIVE_REFUSED", "ALREADY_THERE"), json.dumps(r))
elif args.target == "mowbite-os":
    skip("docking.start", "not in rpc.methods")

if args.write and args.target == "mowbite-os" and len(calibration_offered) == 3:
    # Only while calibration.check names a problem: a start that would be accepted would drive the simulated mower.
    r = rpc("calibration.check", None, "cal_check_w")
    problems = ((r or {}).get("result") or {}).get("problems") or []
    if problems:
        mark = len(msgs)
        r = rpc("calibration.start", None, "cal_start")
        data = ((r or {}).get("error") or {}).get("data") or {}
        check("calibration.start refused -32600 with reason and message of the first problem",
              bool(r) and r.get("error", {}).get("code") == -32600 and data.get("reason") == problems[0].get("reason")
              and bool(data.get("message")), json.dumps(r)[:300])
        refused = wait_for(lambda: events_of("CALIBRATION_REFUSED", mark), 3)
        check("CALIBRATION_REFUSED with the reason", bool(refused) and refused[-1].get("reason") == data.get("reason"),
              json.dumps(refused[-1] if refused else None))
        check("a refused calibration.start leaves the state alone",
              b'"current_state":"CALIBRATING"' not in (latest("robot_state/json") or b""), "")
    else:
        skip("calibration.start", "calibration.check names no problem: a start would drive")
    r = rpc("calibration.abort", None, "cal_abort")
    check("calibration.abort without a drive answers ok false", (r or {}).get("result") == {"ok": False},
          json.dumps(r))

r = rpc("map.replace", [{"areas": [], "docking_stations": []}], "10")
check("an empty map empties it", r and "(0 areas)" in str(r.get("result")), json.dumps(r))


def check_against_schema():
    """AS3: everything seen on robot_state/json and events/json against the schema of system.api."""
    if API_SCHEMA is None:
        return
    try:
        import jsonschema
    except ImportError:
        skip("everything seen against system.api", "python3-jsonschema is missing", point="AS3 schema")
        return
    try:
        jsonschema.Draft202012Validator.check_schema(API_SCHEMA)
        check("system.api is a valid JSON Schema 2020-12", True, point="AS3 schema")
    except jsonschema.SchemaError as e:
        check("system.api is a valid JSON Schema 2020-12", False, str(e)[:200], point="AS3 schema")
        return
    defs = API_SCHEMA.get("$defs", {})

    def validator(name):
        return jsonschema.Draft202012Validator(dict(API_SCHEMA, **{"$ref": "#/$defs/" + name}))
    for topic, kind in (("robot_state/json", "robot_state"), ("events/json", "event")):
        payloads = [p for _, top, p, _ in msgs if top == topic and p]
        bad, whole, own = [], validator(kind), {}
        for p in payloads:
            item = json.loads(p)
            name = f"event.{item.get('type')}" if kind == "event" else kind
            name = name if name in defs else "event.other"
            if name not in own:
                own[name] = validator(name)
            errors = list(own[name].iter_errors(item)) or list(whole.iter_errors(item))
            if errors:
                bad.append(f"{item.get('type', kind)}: {errors[0].message[:160]}")
        check(f"every {kind} seen ({len(payloads)}) as in system.api", bool(payloads) and not bad,
              "; ".join(sorted(set(bad))[:4]), point="AS3 schema " + kind)
    r = rpc("system.info", None, "info_schema")
    errors = list(validator("system.info.result").iter_errors((r or {}).get("result")))
    check("system.info as in system.api", bool(r) and not errors, errors[0].message[:200] if errors else "",
          point="AS3 schema system.info")
    # B1: the state of the setup program as setup.get answers it and as setup/json sends it.
    if "setup" in defs and methods is not None and "setup.get" in methods:
        r = rpc("setup.get", None, "setup_schema")
        errors = list(validator("setup").iter_errors((r or {}).get("result")))
        check("setup.get as $defs.setup", bool(r) and not errors, errors[0].message[:200] if errors else "",
              point="B1 schema setup")
        payloads = [p for _, top, p, _ in msgs if top == "setup/json" and p]
        bad = []
        for p in payloads:
            bad += [e.message[:160] for e in validator("setup").iter_errors(json.loads(p))]
        check(f"every setup/json seen ({len(payloads)}) as $defs.setup", bool(payloads) and not bad,
              "; ".join(sorted(set(bad))[:4]), point="B1 schema setup")
    # AS2: the answers of the commands and the data of their refusals.
    bad = []
    for method, r in COMMAND_ANSWERS:
        if r and "result" in r:
            errors = list(validator(method + ".result").iter_errors(r["result"]))
        elif r and (r.get("error") or {}).get("code") == -32600:
            errors = list(validator("refusal").iter_errors(r["error"].get("data")))
        else:
            errors = []
        if errors:
            bad.append(f"{method}: {errors[0].message[:160]}")
    if COMMAND_ANSWERS:
        check(f"every answer of a command ({len(COMMAND_ANSWERS)}) as in system.api", not bad, "; ".join(bad[:4]),
              point="AS3 schema commands")
    # I1: the results of the other methods of x-methods.
    commands = API_SCHEMA.get("x-commands", {})
    bad, seen = [], 0
    for method, r in RPC_ANSWERS:
        if method in commands or method not in API_SCHEMA.get("x-methods", {}) or not r or "result" not in r:
            continue
        seen += 1
        errors = list(validator(method + ".result").iter_errors(r["result"]))
        if errors:
            bad.append(f"{method}: {errors[0].message[:160]}")
    check(f"every result of another method ({seen}) as in x-methods", seen > 0 and not bad, "; ".join(bad[:4]),
          point="I1 methods")
    payloads = [p for _, top, p, _ in msgs if top == "actions/json" and p]
    bad = []
    for p in payloads:
        bad += [e.message[:160] for e in validator("actions").iter_errors(json.loads(p))]
    check(f"every actions/json seen ({len(payloads)}) as in system.api", bool(payloads) and not bad,
          "; ".join(sorted(set(bad))[:4]), point="I2 actions")
    # I2: the next start of the schedule, only where a start is to come (empty texts clear it).
    payloads = [p for _, top, p, _ in msgs if top == "mowbite/schedule/ready" and p]
    if payloads and "schedule_ready" in defs:
        bad = []
        for p in payloads:
            bad += [e.message[:160] for e in validator("schedule_ready").iter_errors(json.loads(p))]
        check(f"every mowbite/schedule/ready seen ({len(payloads)}) as in system.api", not bad,
              "; ".join(sorted(set(bad))[:4]), point="I2 schedule ready")


check_against_schema()
finish()
