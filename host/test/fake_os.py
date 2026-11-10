"""A stand-in of MowBite OS inside the bundles of test_e2e.py, started by run-os from /opt/mowbite/current.

It writes os-session.json as the logic does, answers rpc.methods, system.info and system.hold on the broker and sends
robot_state. Files in $MOWBITE_UPDATE_PREFIX/state steer it: os-state (the state it reports, IDLE without), slow-start
(waits 2 s before it comes up), slow-stop (takes 1.5 s to stop); a file broken beside it makes it end at once with 3.
Like os-setup it takes the planner /opt/mowbite-planner/active names, else its own, and answers planner.settings for
it unless that planner holds a file silent.
"""
import json
import os
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

HERE = os.path.dirname(os.path.realpath(__file__))
P = os.environ["MOWBITE_UPDATE_PREFIX"]
DATA = P + "/userdata/mowbite"
STATE = P + "/state"


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def write_json(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        json.dump(value, f)
    os.replace(path + ".tmp", path)


REVISION = read(os.path.join(HERE, "revision"))
try:
    PLANNER = os.readlink(P + "/opt/mowbite-planner/active")
except OSError:
    PLANNER = None
PLANNER_SILENT = PLANNER is not None and os.path.exists(f"{P}/opt/mowbite-planner/{PLANNER}/planner/silent")
API = json.loads(read(os.path.join(HERE, "bundle.json"), "{}")).get("api")


def session(clean):
    write_json(DATA + "/os-session.json", {
        "state": read(STATE + "/os-state", "IDLE"), "job_id": "", "boot_id": read(P + "/proc/sys/kernel/random/boot_id"),
        "since": int(time.time()), "clean": clean, "emergency": False, "charging": True})


def answer(client, userdata, message):
    try:
        request = json.loads(message.payload)
    except ValueError:
        return
    method, params = request.get("method"), request.get("params") or {}
    if method == "rpc.methods":
        result = ["rpc.methods", "system.info", "system.hold"]
    elif method == "system.info":
        result = {"name": "mowbite-os", "api": API, "revision": REVISION}
    elif method == "system.hold":
        seconds = int(params.get("seconds", 0))
        now = int(time.time())
        if seconds > 0 and read(STATE + "/os-state", "IDLE") == "IDLE":
            write_json(DATA + "/hold.json", {"until": now + seconds, "set_at": now})
        elif seconds == 0 and os.path.exists(DATA + "/hold.json"):
            os.remove(DATA + "/hold.json")
        result = {"until": now + seconds if seconds > 0 else 0}
    elif method == "planner.settings" and not PLANNER_SILENT:
        result = {"settings": {}}
    else:
        return
    to = getattr(message.properties, "ResponseTopic", None) or "rpc/response"
    props = Properties(PacketTypes.PUBLISH)
    correlation = getattr(message.properties, "CorrelationData", None)
    if correlation is not None:
        props.CorrelationData = correlation
    client.publish(to, json.dumps({"jsonrpc": "2.0", "id": request.get("id"), "result": result}), qos=1,
                   properties=props)


def main():
    if os.path.exists(os.path.join(HERE, "broken")):
        return 3
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    if os.path.exists(STATE + "/slow-start") and stop.wait(2):
        return 0
    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=f"fake-os-{os.getpid()}",
                             protocol=mqtt.MQTTv5)
    else:
        client = mqtt.Client(client_id=f"fake-os-{os.getpid()}", protocol=mqtt.MQTTv5)
    client.on_connect = lambda c, *_: c.subscribe("rpc/request", 1)
    client.on_message = answer
    client.connect_async("127.0.0.1", int(os.environ["MOWBITE_MQTT_PORT"]))
    client.loop_start()
    state = read(STATE + "/os-state", "IDLE")
    session(False)
    write_json(STATE + "/running.json", {"revision": REVISION, "pid": os.getpid(), "planner": PLANNER})
    while not stop.wait(0.5):
        if read(STATE + "/os-state", "IDLE") != state:
            state = read(STATE + "/os-state", "IDLE")
            session(False)
        client.publish("robot_state/json", json.dumps({"current_state": state, "emergency_reason": 0}))
    if os.path.exists(STATE + "/slow-stop"):
        time.sleep(1.5)
    session(True)
    os.remove(STATE + "/running.json")
    client.disconnect()
    client.loop_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
