"""Checks a mower against the OpenMower app interface the way the app uses it (masterplan 3.8).

Read-only by default: what the mower publishes and answers without being asked to act. --write adds the steps that act
(map.replace, actions, teleop, flags); it needs the simulated MowBite OS, which alone offers the RPC sim.flags, freshly
started: it checks the state since the start and reads the frames from --sim-log, which fake_tron stops writing at its
--log-max-mb (20 MB, about an hour).
stdout: one JSON document with a status (ok, missing, different, skipped) per point; PASS/FAIL/SKIP lines go to stderr.
Exit 0 without missing or different, 1 with, 2 for a wrong call or a mower that cannot be reached.
"""
import argparse
import datetime
import json
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
args = parser.parse_args()
if args.write and args.target != "mowbite-os":
    parser.error("--write is implemented for --target mowbite-os only")

LOG = args.sim_log
msgs = []
results = []


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
    summary = {k: sum(r["status"] == k for r in results) for k in ("ok", "missing", "different", "skipped")}
    print(json.dumps({"target": args.target, "host": args.host, "write": args.write, "points": results,
                      "summary": summary}, indent=1))
    sys.stdout.flush()
    c.loop_stop()
    code = code if code is not None else (1 if summary["missing"] or summary["different"] else 0)
    print("RESULT", "OK" if code == 0 else "FAILED", file=sys.stderr, flush=True)
    sys.exit(code)


def on_message(_c, _u, m):
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


def rpc(method, params=None, rid="1", timeout=5.0):
    req = {"jsonrpc": "2.0", "method": method, "id": rid}
    if params is not None:
        req["params"] = params
    start = len(msgs)
    c.publish("rpc/request", json.dumps(req))

    def answer():
        for _, top, p, _ in msgs[start:]:
            if top == "rpc/response":
                j = json.loads(p)
                if j.get("id") == rid:
                    return j
        return None
    return wait_for(answer, timeout)


REC = "mower_logic:area_recording/"
LOGIC_SETTINGS = ["rain_mode", "rain_delay_minutes", "rain_check_seconds", "automatic_mode", "gps_timeout",
                  "max_position_accuracy", "gps_wait_time", "docking_approach_distance", "docking_distance",
                  "docking_retry_count", "undock_distance", "motor_hot_temperature", "motor_cold_temperature"]


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


c = mqtt.Client("stack_test")
if args.user:
    c.username_pw_set(args.user, args.password)
connected = []
c.on_connect = lambda _c, _u, _f, rc: connected.append(rc)
c.on_message = on_message
try:
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
    check("params /ll/board is Tron", params.get("/ll/board") == "Tron", str(params.get("/ll/board")))
else:
    skip("params /ll/board is Tron", "MowBite OS only")

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

r = rpc("rpc.methods")
methods = r["result"] if r and isinstance(r.get("result"), list) else None
if methods is not None:
    check("rpc.methods", "rpc.ping" in methods, json.dumps(methods), missing=True)
elif args.target == "mowbite-os":
    check("rpc.methods", False, "no answer", missing=True)
else:
    skip("rpc.methods", "no answer, older mowers have none")
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
    # mowbite_map adds the planner's keys while a node offers planner.settings
    if methods is not None and "planner.settings" in methods:
        expected += ["mow_around", "angle_min", "angle_max"]
    check("map.area_properties", r and isinstance(r.get("result"), list) and (r["result"] == expected
                                                                              or args.target != "mowbite-os"),
          json.dumps(r and r.get("result")))
if offers("params.set") and args.target == "mowbite-os":
    p = json.loads(latest("params/json") or "{}")
    logic = sorted(k[len("/mower_logic/"):] for k in p if k.startswith("/mower_logic/"))
    check("params/json has the settings of params.set", logic == sorted(LOGIC_SETTINGS), ",".join(logic))
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
r = rpc("no.such.method", None, "3")
check("unknown method -32601", r and r.get("error", {}).get("code") == -32601, json.dumps(r and r.get("error")))
c.publish("rpc/request", "{broken")
r = wait_for(lambda: (latest("rpc/response") and json.loads(latest("rpc/response")).get("error", {}).get("code") == -32700))
check("invalid json -32700", bool(r))


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
      and by_id.get("mower_logic:idle/start_area_recording") == 1 and by_id.get("mower_logic:idle/start_mowing") == 0,
      f"{len(actions)} actions")


def events_of(kind, since=0):
    return [json.loads(p) for _, top, p, _ in msgs[since:] if top == "events/json" and json.loads(p)["type"] == kind]


def fixed_reason(event, resume=None):
    """masterplan 3.1a: a name in capitals, an English message, severity and resume."""
    return (bool(event) and isinstance(event.get("reason"), str) and event["reason"].replace("_", "").isalpha()
            and event["reason"].isupper() and bool(event.get("message"))
            and event.get("severity") in ("info", "warning", "error")
            and event.get("resume") == (resume or event.get("resume")) and event.get("resume") in ("auto", "user"))


mark = len(msgs)
c.publish("action", "mower_logic:idle/start_mowing")
refused = wait_for(lambda: events_of("MOWING_REFUSED", mark), 3)
check("MOWING_REFUSED with a fixed reason", fixed_reason(refused[-1] if refused else None, "user"),
      json.dumps(refused[-1] if refused else None), point="3.1a MOWING_REFUSED")


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

# Test access only, the broker of the simulation is open. Port 9 of the simulation's own host has no caster: the client
# only waits.
TEST_ACCESS = {"host": "127.0.0.1", "port": 9, "mountpoint": "/CHECK", "user": "check", "password": "check-only"}
ntrip_write = methods is not None and "ntrip.set" in methods
if ntrip_write and ((rpc("ntrip.get", None, "ntrip_before") or {}).get("result") or {}).get("configured") is not False:
    skip("ntrip.set", "the simulation has an NTRIP access, it stays as it is")
    ntrip_write = False
if ntrip_write:
    r = rpc("ntrip.set", TEST_ACCESS, "ntrip_set")
    check("ntrip.set answers like ntrip.get", (r or {}).get("result") == {
        "configured": True, "host": "127.0.0.1", "port": 9, "mountpoint": "CHECK", "version": 2, "has_user": True,
        "has_password": True}, json.dumps(r))
    r = rpc("ntrip.set", {"host": "localhost", "port": 9, "mountpoint": "CHECK"}, "ntrip_move")
    check("ntrip.set keeps no login for another caster -32602", bool(r) and r.get("error", {}).get("code") == -32602,
          json.dumps(r and r.get("error")))
    r = rpc("ntrip.set", dict(TEST_ACCESS, pin="1234"), "ntrip_unknown")
    check("ntrip.set refuses an unknown field -32602 without naming it", bool(r)
          and r.get("error", {}).get("code") == -32602 and "pin" not in r["error"].get("message", ""),
          json.dumps(r and r.get("error")))


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
for _ in range(10):
    teleop(0.5, 0.1)
    time.sleep(0.1)
teleop(0.0, 0.0)
time.sleep(1.0)
speeds = sent_speeds(t0)
moving = [s for t, s in speeds if s != [0.0, 0.0]]
zeros = [t for t, s in speeds if s == [0.0, 0.0]]
check_log("teleop drives, clamped to 0.3", len(moving) >= 8 and all(s[0] == 0.3 for s in moving), f"{len(moving)} frames, {moving[:2]}")
check_log("stop after teleop: 3 zeros", len(zeros) == 3, str(zeros))
line = wait_for(lambda: [p for p in overlay_lines("blue") if len(p["poly"]) >= 2], 3)
check("map_overlay: the open line, in both formats", bool(line) and line[0]["is_closed"] == 0 and line[0]["closed"] == 0
      and line[0]["polygon"]["points"] == line[0]["poly"], json.dumps(line[0] if line else latest("map_overlay/json"))[:200])

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

# Leaving the recording mode while the joystick is held stops at once, not after the 0.5 s watchdog.
t0 = sim_time()
hold = Hold(0.2, 0.0)
time.sleep(0.6)
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
if ntrip_write:
    r = rpc("ntrip.delete", None, "ntrip_delete")
    check("ntrip.delete in IDLE", (r or {}).get("result") == {"configured": False}, json.dumps(r))
    off = wait_for(lambda: latest("sensors/os_ntrip_state/data") == b"off", 8)
    check("os_ntrip_state off after ntrip.delete", bool(off), str(latest("sensors/os_ntrip_state/data")))
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
c.publish("action", "mower_logic/reset_emergency")
state = wait_for(lambda: b'"emergency":0' in (latest("robot_state/json") or b""), 4)
check("reset after the bumper clears", bool(state) and json.loads(latest("robot_state/json")).get("emergency_reason") == 0)

# Charging follows the debounced dock contact of the driver (3 battery reports at 1 Hz).
set_flags(dock=1)
state = wait_for(lambda: b'"is_charging":1' in (latest("robot_state/json") or b""), 6)
check("dock -> is_charging", bool(state))
cs = wait_for(lambda: latest("sensors/om_charge_state/data") == b"Fast Charge" or None, 3)
check("charge state Fast Charge", bool(cs), str(latest("sensors/om_charge_state/data")))
set_flags()

# The heartbeat carries the local time of TZ (default of tron.launch.py: Europe/Berlin), not UTC.
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

r = rpc("map.replace", [{"areas": [], "docking_stations": []}], "10")
check("an empty map empties it", r and "(0 areas)" in str(r.get("result")), json.dumps(r))

finish()
