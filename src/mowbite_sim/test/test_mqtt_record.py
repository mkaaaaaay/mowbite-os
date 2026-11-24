"""What the app sees on MQTT in a fixed run, recorded and held against a recording of an earlier OS
(mqtt_record.json next to this file): everything the earlier one had is still there and the same, new parts only add.
Values that differ from run to run (times, positions, readings) count by their type.

The recording of each run goes to mqtt_record.json beside the journal of the run (MOWBITE_JOURNAL_DIR), else into the
temporary folder; without a recording to compare with the test only records. Run in a sourced workspace with mosquitto
installed: launch_test /ws/src/mowbite_sim/test/test_mqtt_record.py
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (TestShutdown, journal_dir, mosquitto, mqtt_connect, mqtt_rpc, sim_stack,  # noqa: E402,F401
                        store_answer, wait_until)

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
from launch.actions import TimerAction  # noqa: E402

MQTT_PORT = 18871
WORK = tempfile.mkdtemp(prefix="mowbite_mqtt_record_")
REFERENCE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "mqtt_record.json")
# Texts whose words depend on the build, not on the interface.
BUILD_FIELDS = {"version", "api", "schema"}


def generate_test_description():
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT)]),
                                     launch_testing.actions.ReadyToTest()]), {}


def kind(value):
    """The type of a value that changes from run to run, with the shape of an object or list."""
    if isinstance(value, dict):
        return {k: kind(v) for k, v in value.items()}
    if isinstance(value, list):
        return [kind(value[0])] if value else []
    if isinstance(value, bool) or value is None:
        return type(value).__name__
    return "number" if isinstance(value, (int, float)) else "text"


def by(entries, key):
    """A list of objects as an object by one of their fields."""
    return {e[key]: e for e in entries}


def additions(old, new, path=""):
    """(missing or changed in new, added in new) of new against old, as paths."""
    if isinstance(old, dict) and isinstance(new, dict):
        lost, added = [], [f"{path}/{k}" for k in new if k not in old]
        for k, v in old.items():
            if k not in new:
                lost.append(f"{path}/{k}: gone")
            else:
                sub_lost, sub_added = additions(v, new[k], f"{path}/{k}")
                lost += sub_lost
                added += sub_added
        return lost, added
    if isinstance(old, list) and isinstance(new, list) and all(isinstance(v, str) for v in old + new):
        return [f"{path}: {v} gone" for v in old if v not in new], [f"{path}: {v}" for v in new if v not in old]
    if old != new:
        return [f"{path}: {json.dumps(old)} -> {json.dumps(new)}"], []
    return [], []


class TestMqttRecord(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.messages = []
        cls.answers = {}

        def on_message(_client, _userdata, message):
            if message.topic.startswith("rpc/response"):
                store_answer(cls.answers, cls.lock, message.payload)
            with cls.lock:
                cls.messages.append((message.topic, message.payload.decode("utf-8", "replace"), message.retain))

        cls.client = mqtt_connect("mqtt_record", MQTT_PORT, on_message, "#")

    @classmethod
    def tearDownClass(cls):
        cls.client.loop_stop()
        cls.client.disconnect()
        shutil.rmtree(WORK, ignore_errors=True)

    def latest(self, topic):
        with self.lock:
            for top, payload, _ in reversed(self.messages):
                if top == topic:
                    return payload
        return None

    def rpc(self, method, params=None):
        answer = mqtt_rpc(self.client, self.answers, self.lock, method, params)
        self.assertIsNotNone(answer, method)
        return answer.get("result", answer.get("error"))

    def logic_sensors_live(self):
        infos = json.loads(self.latest("sensor_infos/json") or "[]")
        if len(infos) < 11:
            return False
        state = json.loads(self.latest("robot_state/json") or "{}")
        return state.get("current_state") == "IDLE" and all(
            self.latest(f"sensors/{i['sensor_id']}/data") is not None for i in infos
            if i["sensor_id"] not in ("os_correction_age", "os_ntrip_state"))

    def test_1_the_app_sees_what_it_saw_before(self):
        self.assertTrue(wait_until(self.logic_sensors_live, 120), self.latest("sensor_infos/json"))
        time.sleep(3.0)
        record = {}
        for topic in ("capabilities/json", "params/json", "sensor_infos/json", "actions/json"):
            value = json.loads(self.latest(topic))
            if topic == "sensor_infos/json":
                value = by(value, "sensor_id")
            if topic == "actions/json":
                value = {a["action_id"]: a["action_name"] for a in value}
            record[topic] = value
        record["robot_state/json"] = kind(json.loads(self.latest("robot_state/json")))
        with self.lock:
            topics = {top for top, _, _ in self.messages if not top.startswith("rpc/response")}
        record["topics"] = sorted(topics)
        sensors = {}
        for top in topics:
            if top.startswith("sensors/") and top.endswith("/data"):
                payload = self.latest(top)
                try:
                    float(payload)
                    sensors[top] = "number"
                except ValueError:
                    sensors[top] = payload
        record["sensors"] = sensors

        info = self.rpc("system.info")
        record["system.info"] = {k: kind(v) if k in BUILD_FIELDS else v for k, v in info.items()}
        record["rpc.methods"] = sorted(self.rpc("rpc.methods"))
        listed = self.rpc("sensors.list")
        record["sensors.list"] = {e["sensor_id"]: {k: v for k, v in e.items() if k not in ("value", "stamp")}
                                  for e in listed}
        record["sensors.list ids"] = [e["sensor_id"] for e in self.rpc("sensors.list", {"ids": ["om_v_battery"]})]
        history = self.rpc("sensors.history", {"sensor_ids": ["om_v_battery", "battery_temp"]})
        record["sensors.history"] = {"step": history.get("step"),
                                     "sensors": {s["sensor_id"]: {k: v for k, v in s.items() if k != "points"}
                                                 for s in history.get("sensors", [])}}

        # An emergency: the line of its reasons in the log, as the app reads it.
        self.rpc("sim.flags", [{"lift": 1}])
        self.assertTrue(wait_until(lambda: b'"emergency":1' in (self.latest("robot_state/json") or "").encode(), 10))
        lines = []

        def reason_lines():
            lines[:] = [e["msg"] for e in self.rpc("logs.recent", {"level": "WARN"})
                        if e["msg"].startswith("Emergency") and "LIFT" in e["msg"]]
            return bool(lines)

        self.assertTrue(wait_until(reason_lines, 10))
        record["emergency lines"] = lines
        self.rpc("sim.flags", [{}])

        written = os.path.join(os.path.dirname(os.path.abspath(journal_dir(WORK))), "mqtt_record.json")
        with open(written, "w") as f:
            json.dump(record, f, indent=1, sort_keys=True)
            f.write("\n")
        if not os.path.exists(REFERENCE):
            self.skipTest(f"no recording to compare with, this run's is in {written}")
        with open(REFERENCE) as f:
            reference = json.load(f)
        lost, added = additions(reference, record)
        print("added since the recording:\n  " + "\n  ".join(added or ["nothing"]), file=sys.stderr)
        self.assertEqual(lost, [])
