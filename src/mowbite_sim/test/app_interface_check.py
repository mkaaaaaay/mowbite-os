"""Drives MowBite OS like the app does and checks the answers.

Needs mosquitto on 127.0.0.1:1883 and `ros2 launch mowbite_sim sim.launch.py` running with its default paths.
"""
import datetime
import json
import os
import struct
import sys
import threading
import time
import zoneinfo

import paho.mqtt.client as mqtt

FLAGS = "/tmp/mowbite_sim_flags.json"
LOG = "/tmp/mowbite_sim_rx.jsonl"
msgs = []
ok = True


def check(name, cond, detail=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{detail}]" if detail else ""))
    ok = ok and cond


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
    with open(FLAGS, "w") as f:
        json.dump(flags, f)


def rpc(method, params=None, rid="1"):
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
    return wait_for(answer)


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
    with open(LOG) as f:
        for line in f:
            e = json.loads(line)
            if e.get("type") == 0 and e.get("mod") == 4 and e["t"] >= since:
                out.append((e["t"], e["speed"]))
    return out


def sim_time():
    with open(LOG) as f:
        lines = f.readlines()
    return json.loads(lines[-1])["t"] if lines else 0.0


c = mqtt.Client("stack_test")
c.on_message = on_message
c.connect("127.0.0.1", 1883)
c.subscribe("#")
c.loop_start()
time.sleep(2)

retained = {top for _, top, _, r in msgs if r}
for top in ["capabilities/json", "version/json", "params/json", "sensor_infos/json", "actions/json"]:
    check("retained " + top, top in retained)

params = json.loads(latest("params/json") or "{}")
check("params /ll/board is Tron", params.get("/ll/board") == "Tron", str(params.get("/ll/board")))

infos = json.loads(latest("sensor_infos/json") or "[]")
ids = sorted(i["sensor_id"] for i in infos)
check("sensor ids", len(ids) == 11 and "om_v_battery" in ids and "om_gps_accuracy" in ids, ",".join(ids))
battery_info = next((i for i in infos if i["sensor_id"] == "om_v_battery"), {})
check("battery max 25.2 and 0/1 flags", battery_info.get("max_value") == 25.2 and battery_info.get("has_min_max") == 1,
      json.dumps(battery_info))

state = wait_for(lambda: latest("robot_state/json"))
check("robot_state compact", state is not None and b'"current_state":"IDLE"' in state, (state or b"")[:160].decode())
s = json.loads(state)
check("robot_state values", s["emergency"] == 0 and s["is_charging"] == 0 and abs(s["battery_percentage"] - 0.8) < 1e-6,
      f'emergency={s["emergency"]} charging={s["is_charging"]} battery={s["battery_percentage"]}')
check("standing since the start: heading not aligned, GPS 0", s["pose"]["heading_valid"] == 0 and s["gps_percentage"] == 0,
      f'heading_valid={s["pose"]["heading_valid"]} gps={s["gps_percentage"]} acc={s["pose"]["pos_accuracy"]}')
v = wait_for(lambda: latest("sensors/om_v_battery/data"))
check("sensor value text", v == b"24.500000", str(v))
cs = wait_for(lambda: latest("sensors/om_charge_state/data"))
check("charge state", cs == b"Not Charging", str(cs))

actions = json.loads(latest("actions/json"))
by_id = {a["action_id"]: a["enabled"] for a in actions}
check("actions", by_id.get("mower_logic/reset_emergency") == 1 and by_id.get("mower_logic:idle/start_area_recording") == 1
      and by_id.get("mower_logic:idle/start_mowing") == 0, f"{len(actions)} actions")

r = rpc("rpc.methods")
check("rpc.methods", r and "rpc.ping" in r["result"] and "logs.recent" in r["result"], json.dumps(r and r["result"]))
r = rpc("events.history", None, "2")
types = [e.get("type") for e in (r or {}).get("result", [])]
check("events.history has BOOTED", "BOOTED" in types, ",".join(map(str, types)))
r = rpc("map.replace", [{}], "3")
check("unknown method -32601", r and r["error"]["code"] == -32601, json.dumps(r and r.get("error")))
c.publish("rpc/request", "{broken")
r = wait_for(lambda: (latest("rpc/response") and json.loads(latest("rpc/response")).get("error", {}).get("code") == -32700))
check("invalid json -32700", bool(r))

# Teleop does nothing in IDLE
t0 = sim_time()
for _ in range(5):
    teleop(0.2, 0.0)
    time.sleep(0.1)
time.sleep(0.3)
check("teleop ignored in IDLE", not sent_speeds(t0), str(sent_speeds(t0)))

c.publish("action", "mower_logic:idle/start_area_recording")
state = wait_for(lambda: b'"current_state":"AREA_RECORDING"' in (latest("robot_state/json") or b""), 3)
check("state AREA_RECORDING", bool(state))
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
check("teleop drives, clamped to 0.3", len(moving) >= 8 and all(s[0] == 0.3 for s in moving), f"{len(moving)} frames, {moving[:2]}")
check("stop after teleop: 3 zeros", len(zeros) == 3, str(zeros))

set_flags(lift=1)
state = wait_for(lambda: b'"emergency":1' in (latest("robot_state/json") or b""), 3)
check("lift -> emergency", bool(state))
ev = wait_for(lambda: [json.loads(p) for _, top, p, _ in msgs if top == "events/json" and json.loads(p)["type"] == "EMERGENCY"])
check("EMERGENCY event with reason bits", ev and ev[-1]["emergency"] is True and ev[-1]["reason"] & 8,
      json.dumps(ev[-1] if ev else None))
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
check("teleop blocked during emergency", not [s for t, s in sent_speeds(t0) if s != [0.0, 0.0]])
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
check("held joystick does not drive after reset", not [s for t, s in sent_speeds(t0) if s != [0.0, 0.0]],
      str(sent_speeds(t0)[:3]))
hold.release()
t0 = sim_time()
for _ in range(5):
    teleop(0.2, 0.0)
    time.sleep(0.1)
teleop(0.0, 0.0)
time.sleep(0.5)
check("teleop drives again after a zero", len([s for t, s in sent_speeds(t0) if s != [0.0, 0.0]]) >= 4,
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
check("exit_recording_mode stops at once: 3 zeros within 0.3 s", last_moving is not None and len(zeros) == 3
      and zeros[0] - last_moving < 0.3, f"last moving {last_moving}, zeros {zeros}")
state = wait_for(lambda: b'"current_state":"IDLE"' in (latest("robot_state/json") or b""), 3)
check("state IDLE", bool(state))
s = json.loads(wait_for(lambda: latest("robot_state/json") if b'"heading_valid":1' in (latest("robot_state/json") or b"")
                        else None, 3) or latest("robot_state/json"))
check("heading aligned after driving, GPS above 0", s["pose"]["heading_valid"] == 1 and s["gps_percentage"] > 0.5,
      f'heading_valid={s["pose"]["heading_valid"]} gps={s["gps_percentage"]} acc={s["pose"]["pos_accuracy"]}')
with open(LOG) as f:
    clears = [json.loads(l) for l in f if '"mod": 10' in l and '"type": 0,' in l]
check("clear frame sent", len(clears) == 1 and clears[0]["hex"] == "00" * 8, str(clears))

set_flags(dock=1)
state = wait_for(lambda: b'"is_charging":1' in (latest("robot_state/json") or b""), 4)
check("dock -> is_charging", bool(state))
cs = wait_for(lambda: latest("sensors/om_charge_state/data") == b"Fast Charge" or None, 3)
check("charge state Fast Charge", bool(cs), str(latest("sensors/om_charge_state/data")))
set_flags()

# The heartbeat carries the local time of TZ (default of tron.launch.py: Europe/Berlin), not UTC.
with open(LOG) as f:
    beats = [json.loads(l) for l in f if '"type": 255' in l]
b = bytes.fromhex(beats[-1]["hex"]) if beats else bytes(8)
zone = zoneinfo.ZoneInfo(os.environ.get("TZ") or "Europe/Berlin")
try:
    sent = datetime.datetime(2000 + b[0], b[1], b[2], b[3], b[4], b[5], tzinfo=zone)
    skew = abs((datetime.datetime.now(zone) - sent).total_seconds())
except ValueError:
    skew = None
check("heartbeat in local time", skew is not None and skew < 5, f"{b.hex()} in {zone}, skew {skew}")

r = rpc("logs.recent", {"level": "WARN"}, "9")
check("logs.recent answers", r is not None and isinstance(r.get("result"), list), f'{len((r or {}).get("result", []))} entries')

c.loop_stop()
print("RESULT", "OK" if ok else "FAILED")
sys.exit(0 if ok else 1)
