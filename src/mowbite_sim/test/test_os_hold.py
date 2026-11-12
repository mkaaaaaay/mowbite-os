"""A hold of an update across a restart of the whole stack (sim.launch.py on a private mosquitto): the OS starts with
<data_dir>/hold.json from before, the logic holds for what is left of it and mirrors it on mowbite/hold, nothing
starts; system.hold 0 lets go and removes the file. The stop at the end leaves a clean os-session.json.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_os_hold.py
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
from sim_common import EXIT_CODES, mosquitto, mqtt_connect, sim_files, sim_stack, wait_until  # noqa: E402

MQTT_PORT = 18843
WORK = tempfile.mkdtemp(prefix="mowbite_os_hold_")
_, _, LOG, DATA = sim_files(WORK)
HOLD = os.path.join(DATA, "hold.json")
UNTIL = int(time.time()) + 600
SPEED = 4


def generate_test_description():
    os.makedirs(DATA)
    # Left by an update that stopped half way: set five minutes ago for fifteen.
    with open(HOLD, "w") as f:
        json.dump({"until": UNTIL, "set_at": UNTIL - 900}, f)
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT)]),
                                     launch_testing.actions.ReadyToTest()]), {}


class Broker:
    def __init__(self):
        self.msgs = []
        self.client = mqtt_connect("os_hold_test", MQTT_PORT, lambda _c, _u, m: self.msgs.append((m.topic, m.payload)),
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

    def rpc(self, method, params, rid):
        start = len(self.msgs)
        self.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "method": method, "params": params,
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


class TestOsHold(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.broker = Broker()
        assert cls.broker.wait(lambda: cls.broker.latest("robot_state/json"), 90), "no robot_state"

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()

    def test_1_the_hold_from_before_stands(self):
        mirrored = self.broker.wait(lambda: self.broker.latest("mowbite/hold"), 40)
        self.assertTrue(mirrored, "no mowbite/hold")
        self.assertEqual(json.loads(mirrored), {"until": UNTIL})
        self.assertTrue(os.path.exists(HOLD))

    def test_2_nothing_starts(self):
        mark = len(self.broker.msgs)
        self.broker.client.publish("action", "mower_logic:idle/start_mowing")
        refused = self.broker.wait(lambda: self.broker.events("MOWING_REFUSED", mark), 5)
        self.assertTrue(refused, "no MOWING_REFUSED")
        self.assertEqual(refused[-1]["reason"], "HOLD")
        self.assertEqual(refused[-1]["until"], UNTIL)
        r = self.broker.rpc("params.set", {"params": {"/mower_logic/gps_wait_time": 20}}, "params")
        self.assertEqual(r["error"]["code"], -32004)
        self.assertEqual(r["error"]["data"], {"reason": "HOLD", "until": UNTIL})
        self.assertEqual(speeds_moving(), [])

    def test_3_zero_lets_go_and_removes_the_file(self):
        mark = len(self.broker.msgs)
        r = self.broker.rpc("system.hold", {"seconds": 0}, "off")
        self.assertEqual(r["result"]["hold_until"], 0, r)
        self.assertFalse(os.path.exists(HOLD))
        cleared = self.broker.wait(
            lambda: any(top == "mowbite/hold" and p == b"" for top, p in self.broker.msgs[mark:]), 5)
        self.assertTrue(cleared, "mowbite/hold not emptied")
        r = self.broker.rpc("params.set", {"params": {"/mower_logic/gps_wait_time": 20}}, "params_after")
        self.assertIn("result", r, r)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        # The logic stopped on its own: the next start finds a clean session and holds nothing. fake_tron writes its
        # state into WORK until the end, so WORK goes only now.
        try:
            with open(os.path.join(DATA, "os-session.json")) as f:
                session = json.load(f)
            self.assertEqual((session["state"], session["clean"]), ("IDLE", True), session)
            self.assertFalse(os.path.exists(os.path.join(DATA, "restart-hold.json")))
        finally:
            shutil.rmtree(WORK, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES)
