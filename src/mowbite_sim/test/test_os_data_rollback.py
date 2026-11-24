"""A way back of the data to an older backup whose map changed since the update (sim.launch.py on a private
mosquitto): the OS starts with <data_dir>/data_rollback.json with confirm, system.info shows it, nothing mows by itself
(MOWING_REFUSED DATA_ROLLBACK, schedule.get not available with data_rollback) while schedule.set still works;
data.rollback.confirm removes the note and frees both.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_os_data_rollback.py
"""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
from launch.actions import TimerAction

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import TRON_SPEED as SPEED  # noqa: E402
from sim_common import EXIT_CODES, mosquitto, mqtt_connect, sim_files, sim_stack, wait_until  # noqa: E402

MQTT_PORT = 18855
WORK = tempfile.mkdtemp(prefix="mowbite_os_data_rollback_")
_, _, LOG, DATA = sim_files(WORK)
NOTE_FILE = os.path.join(DATA, "data_rollback.json")
BACKUP_OF = int(time.time()) - 30 * 86400
NOTE = {"from": 3, "to": 2, "backup": "backup-3", "t": time.time() - 60, "backup_of": BACKUP_OF, "confirm": True}


def generate_test_description():
    os.makedirs(DATA)
    # Left by the start of the stack after it took the data back to the backup of a month ago.
    with open(NOTE_FILE, "w") as f:
        json.dump(NOTE, f)
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT)]),
                                     launch_testing.actions.ReadyToTest()]), {}


class Broker:
    def __init__(self):
        self.msgs = []
        self.client = mqtt_connect("os_data_rollback_test", MQTT_PORT, lambda _c, _u, m: self.msgs.append((m.topic, m.payload)),
                                   "#", timeout=30.0)

    def latest(self, topic):
        for top, payload in reversed(self.msgs):
            if top == topic:
                return payload
        return None

    def wait(self, pred, timeout=10.0):
        return wait_until(pred, timeout)

    def events(self, kind, since):
        return [json.loads(p) for top, p in self.msgs[since:] if top == "events/json" and json.loads(p)["type"] == kind]

    def rpc(self, method, params=None, rid="r"):
        start = len(self.msgs)
        self.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "method": method, "params": params or {},
                                                       "id": rid}))

        def answer():
            for top, p in self.msgs[start:]:
                if top == "rpc/response" and json.loads(p).get("id") == rid:
                    return json.loads(p)
            return None
        return self.wait(answer, 5)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


def speeds_moving():
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if entry.get("type") == 0 and entry.get("mod") == SPEED and entry["speed"] != [0.0, 0.0]:
                out.append(entry)
    return out


class TestOsDataRollback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.broker = Broker()
        assert cls.broker.wait(lambda: cls.broker.latest("robot_state/json"), 90), "no robot_state"

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()

    def schedule(self):
        answer = self.broker.rpc("schedule.get", rid="schedule")
        return answer["result"]["available"], answer["result"]["reason"]

    def test_1_the_note_from_the_start(self):
        info = self.broker.rpc("system.info", rid="info")["result"]["data_rollback"]
        self.assertEqual(info, {"from": 3, "to": 2, "backup_of": BACKUP_OF, "t": int(NOTE["t"]), "backup": "backup-3",
                                "confirm": True})

    def test_2_nothing_mows_by_itself(self):
        mark = len(self.broker.msgs)
        self.broker.client.publish("action", "mower_logic:idle/start_mowing")
        refused = self.broker.wait(lambda: self.broker.events("MOWING_REFUSED", mark), 5)
        self.assertTrue(refused, "no MOWING_REFUSED")
        self.assertEqual((refused[-1]["reason"], refused[-1]["backup_of"]), ("DATA_ROLLBACK", BACKUP_OF))
        self.assertTrue(self.broker.wait(lambda: self.schedule() == (False, "data_rollback"), 10), self.schedule())
        r = self.broker.rpc("schedule.set", {"skip_rain": 1}, "set")
        self.assertIn("result", r, r)
        self.assertEqual(speeds_moving(), [])

    def test_3_the_confirmation_frees_it(self):
        mark = len(self.broker.msgs)
        r = self.broker.rpc("data.rollback.confirm", rid="confirm")
        self.assertEqual(r["result"], {"released": True}, r)
        self.assertFalse(os.path.exists(NOTE_FILE))
        confirmed = self.broker.wait(lambda: self.broker.events("DATA_ROLLBACK", mark), 5)
        self.assertTrue(confirmed, "no DATA_ROLLBACK")
        self.assertEqual(confirmed[-1]["what"], "confirmed")
        self.assertTrue(self.broker.wait(lambda: self.schedule() == (True, None), 10), self.schedule())
        self.assertIsNone(self.broker.rpc("system.info", rid="info_after")["result"]["data_rollback"])
        mark = len(self.broker.msgs)
        self.broker.client.publish("action", "mower_logic:idle/start_mowing")
        time.sleep(2)
        self.assertFalse([e for e in self.broker.events("MOWING_REFUSED", mark) if e["reason"] == "DATA_ROLLBACK"])
        self.assertEqual(self.broker.rpc("data.rollback.confirm", rid="again")["result"], {"released": False})


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        shutil.rmtree(WORK, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES)
