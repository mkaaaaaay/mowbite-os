"""The schedule in the simulation (FP22 (g) B3): the whole stack of sim.launch.py on a private mosquitto, the schedule
allowed and its plans set over MQTT for the next whole minute, no hand on the clock. In this order: before a start
mowbite/schedule/ready is ok and system.hold grants nothing; an announcement canceled by a command and by STOP; a start
from the station that goes home at the end of its plan; a pause during a run; a plan while the mower is off the
contacts; a plan that waits for the battery when the container restarts (S5: not taken up again). Plans carry dark 1,
so the time of day does not matter.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_schedule.py
"""
import json
import math
import os
import shutil
import signal
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from mowbite_msgs.msg import ClockStatus, HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState
from mowbite_msgs.msg import RpcError, RpcRequest, RpcResponse
from mowbite_msgs.srv import ReplaceMap
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, container_pid, mosquitto, mqtt_connect, read_json,  # noqa: E402
                        restarted, sim_files, sim_stack, wait_until)

MQTT_PORT = 18877
WORK = tempfile.mkdtemp(prefix="mowbite_schedule_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "schedule.yaml")
READY = "mowbite/schedule/ready"
DAYS = [1, 2, 3, 4, 5, 6, 7]
# tron.launch.py: the wait before the container starts again.
RESTART_DELAY = 2.0

# The yard, the patch and the station of test_sunset.py.
YARD = [(-0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (-0.5, 2.0)]
PATCH = [(0.8, -1.5), (3.6, -1.5), (3.6, 1.5), (0.8, 1.5)]
DOCKED = [-1.0, 0.0, 0.0]
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]},
                    {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                     "outline": [{"x": x, "y": y} for x, y in PATCH]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}


def generate_test_description():
    # sim.yaml with the schedule allowed for this platform.
    with open(os.path.join(get_package_share_directory("mowbite_sim"), "config", "sim.yaml")) as f:
        text = f.read()
    with open(CONFIG, "w") as f:
        f.write(text + "\nmowbite_schedule:\n  ros__parameters:\n    allowed: true\n")
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT,
                                                                                extra_config=CONFIG)]),
                                     launch_testing.actions.ReadyToTest()]), {}


def next_minute(lead=20.0):
    """The next whole minute (unix s) at least lead s from now."""
    at = math.ceil(time.time() / 60.0) * 60
    return at + 60 if at - time.time() < lead else at


def clock_time(unix_s):
    """HH:MM in UTC, the zone of the plans."""
    return time.strftime("%H:%M", time.gmtime(unix_s))


def plan(at, **extra):
    return {"days": DAYS, "time": clock_time(at), "dark": 1, **extra}


class Broker:
    """The app's side: rpc over MQTT, the retained texts as they come."""

    def __init__(self):
        self.msgs = []
        self.count = 0
        self.client = mqtt_connect("schedule_test", MQTT_PORT,
                                   lambda _c, _u, m: self.msgs.append((m.topic, m.payload, m.retain)), "#",
                                   timeout=30.0)

    def latest(self, topic):
        for top, payload, _ in reversed(self.msgs):
            if top == topic:
                return payload
        return None

    def ready(self):
        payload = self.latest(READY)
        return None if payload is None else (json.loads(payload) if payload else {})

    def rpc(self, method, params=None):
        self.count += 1
        rid = f"schedule-{self.count}"
        start = len(self.msgs)
        self.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "method": method, "params": params or {},
                                                       "id": rid}))

        def answer():
            for top, p, _ in self.msgs[start:]:
                if top == "rpc/response" and json.loads(p).get("id") == rid:
                    return json.loads(p)
            return None
        return wait_until(answer, 10)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


class Probe(SimProbe):
    truth_tries = 5

    def __init__(self):
        super().__init__("schedule_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.clock = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(ClockStatus, "time/clock", self._set("clock"), LATCHED)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.start()

    def state(self):
        with self.lock:
            return self.robot.current_state if self.robot else None


class TestSchedule(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()
        cls.broker = Broker()

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def set_schedule(self, **values):
        answer = self.broker.rpc("schedule.set", {"schedule": values})
        self.assertIn("result", answer, answer)
        return answer["result"]

    def log(self):
        answer = self.broker.rpc("schedule.log", {"limit": 100})
        self.assertIn("result", answer, answer)
        return answer["result"]["entries"]

    def logged(self, what, at, timeout=10.0):
        """The entry of schedule.log with that word for the plan at that time."""
        return wait_until(lambda: next((e for e in self.log() if e["what"] == what and e["at"] == clock_time(at)),
                                       None), timeout, poll=0.5)

    def wait_announced(self, since, at):
        """SCHEDULE_START of the plan at that time; the wheels have not turned."""
        event = self.probe.event(since, "SCHEDULE_START", at - time.time() + 10)
        self.assertIsNotNone(event, self.probe.events[since:])
        self.assertGreaterEqual(time.time(), at)
        self.assertEqual(event["seconds"], 10)
        self.assertEqual(event["source"], "schedule")
        self.assertEqual(self.probe.state(), "IDLE")
        return event

    def assert_docked_idle(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.state() == "IDLE" and p.hw and p.hw.dock_contact, 10),
                        (p.state(), p.hw and p.hw.dock_contact))

    def test_01_on_the_station_with_a_safe_clock(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.clock is not None, 60), "no time/clock")
        self.assertTrue(p.clock.ok, p.clock.reason)
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.map and p.map["docking_stations"], 10), "the map did not come")
        time.sleep(0.5)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        since = len(p.events)
        p.flags(place=DOCKED)
        self.assertTrue(p.wait(lambda: p.hw and p.hw.dock_contact, 10), "no contact on the station")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy <= 0.03 and abs(p.pose.x + 1.0) < 0.05,
                               30), "no RTK position there")
        p.flags()
        self.assertIsNotNone(p.event(since, "DOCKED", 30), p.events[since:])
        # Allowed, switched off: no start is to come, nothing retained (the broker drops an empty text).
        answer = self.broker.rpc("schedule.get")
        self.assertEqual((answer["result"]["available"], answer["result"]["reason"]), (True, None), answer)
        self.assertIn(self.broker.ready(), (None, {}))

    def test_02_ready_and_no_hold_before_a_start(self):
        at = next_minute(lead=40.0)
        result = self.set_schedule(enabled=1, tz="UTC0", min_battery=50, plans=[plan(at)])
        self.assertEqual(result["next_start"], at, result)
        self.assertTrue(wait_until(lambda: self.broker.ready() == {"ok": True, "reason": None}, 5),
                        self.broker.ready())
        # Retained: a client that comes later gets it.
        late = []
        client = mqtt_connect("schedule_late", MQTT_PORT, lambda _c, _u, m: late.append((m.payload, m.retain)), READY)
        try:
            self.assertTrue(wait_until(lambda: late, 5), "nothing retained on " + READY)
            self.assertEqual((json.loads(late[0][0]), late[0][1]), ({"ok": True, "reason": None}, True))
        finally:
            client.loop_stop()
            client.disconnect()
        # Within 30 min of a start no hold: idle false and when the start is.
        answer = self.broker.rpc("system.hold", {"seconds": 600})
        self.assertIn("result", answer, answer)
        self.assertEqual((answer["result"]["idle"], answer["result"]["hold_until"], answer["result"]["next_start"]),
                         (False, 0, at), answer)
        self.__class__.at = at

    def test_03_a_command_cancels_the_announcement(self):
        p = self.probe
        at = self.__class__.at
        since = len(p.events)
        self.wait_announced(since, at)
        answer = p.rpc("mowing.abort", {"client": "test"})
        self.assertIsNotNone(answer)
        canceled = p.event(since, "SCHEDULE_START_CANCELED", 3, source="rpc")
        self.assertIsNotNone(canceled, p.events[since:])
        self.assertEqual((canceled["severity"], canceled["resume"]), ("info", "auto"))
        self.assertIsNotNone(self.logged("start_canceled", at))
        self.assertEqual(self.logged("start_canceled", at)["detail"], "rpc")
        self.assertIsNone(p.event(since, "STATE", 12, state="UNDOCKING"), p.events[since:])
        self.assert_docked_idle()

    def test_04_stop_cancels_the_announcement(self):
        p = self.probe
        at = next_minute()
        self.set_schedule(plans=[plan(at)])
        since = len(p.events)
        self.wait_announced(since, at)
        p.flags(stop=1)
        canceled = p.event(since, "SCHEDULE_START_CANCELED", 3, source="emergency")
        self.assertIsNotNone(canceled, p.events[since:])
        self.assertEqual(self.logged("start_canceled", at)["detail"], "emergency")
        p.flags()
        time.sleep(1.0)
        answer = p.rpc("emergency.reset", {"client": "test"})
        self.assertEqual((answer or {}).get("result", {}).get("released"), True, answer)
        self.assertTrue(p.wait(lambda: p.robot and not p.robot.emergency, 5), "the emergency stayed")
        self.assertIsNone(p.event(since, "STATE", 2, state="UNDOCKING"), p.events[since:])
        self.assert_docked_idle()

    def test_05_start_from_the_station_and_home_at_the_end(self):
        p = self.probe
        at = next_minute()
        end = at + 120
        self.set_schedule(plans=[plan(at, end=clock_time(end))])
        since = len(p.events)
        self.wait_announced(since, at)
        self.assertIsNotNone(p.event(since, "STATE", 60, state="UNDOCKING"), p.events[since:])
        mowing = p.event(since, "STATE", 90, state="MOWING")
        self.assertIsNotNone(mowing, p.events[since:])
        self.assertEqual(mowing.get("source"), "schedule", mowing)
        started = self.logged("started", at)
        self.assertIsNotNone(started, self.log())
        # The next start is tomorrow's and would find the mower busy.
        self.assertTrue(wait_until(lambda: self.broker.ready() == {"ok": False, "reason": "skip_busy"}, 5),
                        self.broker.ready())
        docking = p.event(since, "DOCKING", end - time.time() + 10, reason="SCHEDULE")
        self.assertIsNotNone(docking, p.events[since:])
        self.assertGreaterEqual(time.time(), end - 1)
        self.assertEqual((docking["detail"], docking["source"]), ("end", "schedule"), docking)
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        self.assertIsNotNone(self.logged("stopped_end", at))
        self.assert_docked_idle()

    def test_06_a_pause_sends_the_run_home(self):
        p = self.probe
        at = next_minute()
        self.set_schedule(plans=[plan(at)])
        since = len(p.events)
        self.wait_announced(since, at)
        self.assertIsNotNone(p.event(since, "STATE", 90, state="MOWING"), p.events[since:])
        time.sleep(5.0)
        today = time.strftime("%Y-%m-%d", time.gmtime())
        self.set_schedule(pause={"until": today})
        docking = p.event(since, "DOCKING", 10, reason="SCHEDULE")
        self.assertIsNotNone(docking, p.events[since:])
        self.assertEqual(docking["detail"], "pause", docking)
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        self.assertIsNotNone(self.logged("stopped_paused", at))
        self.set_schedule(pause=None)
        self.assert_docked_idle()

    def test_07_off_the_contacts_no_start(self):
        p = self.probe
        at = next_minute()
        p.flags(place=[0.0, 0.0, 0.0])
        self.assertTrue(p.wait(lambda: p.hw and not p.hw.dock_contact, 10), "still on the contacts")
        since = len(p.events)
        self.set_schedule(plans=[plan(at)])
        self.assertTrue(wait_until(lambda: self.broker.ready() == {"ok": False, "reason": "skip_not_docked"}, 5),
                        self.broker.ready())
        self.assertIsNotNone(self.logged("skip_not_docked", at, at - time.time() + 10), self.log())
        self.assertIsNone(p.event(since, "SCHEDULE_START", 2), p.events[since:])
        p.flags(place=DOCKED)
        self.assertTrue(p.wait(lambda: p.hw and p.hw.dock_contact, 10), "no contact on the station")
        p.flags()
        self.assertIsNotNone(p.event(since, "DOCKED", 30), p.events[since:])

    def test_08_a_restart_while_waiting_does_not_take_the_plan_up(self, proc_info):
        p = self.probe
        at = next_minute()
        # Low and not charging: the plan waits for min_battery 50.
        p.flags(battery_percentage=30, charge_current=0)
        self.assertTrue(p.wait(lambda: p.robot and p.robot.battery_percentage < 0.4, 10), "the battery stayed")
        self.set_schedule(plans=[plan(at)])
        self.assertTrue(wait_until(lambda: self.broker.ready() == {"ok": False, "reason": "waiting_battery"}, 5),
                        self.broker.ready())
        self.assertIsNotNone(self.logged("waiting_battery", at, at - time.time() + 10), self.log())
        path = os.path.join(DATA, "schedule-state.json")
        # The file is written beside the node; the mark of the wait comes shortly after the log.
        waiting = wait_until(lambda: ((read_json(path, tries=5) or {}).get("waiting") or {}).get("time"), 10)
        self.assertEqual(waiting, clock_time(at), read_json(path))
        since = len(p.events)
        pid = container_pid(proc_info)
        self.assertIsNotNone(pid)
        os.kill(pid, signal.SIGKILL)
        self.assertTrue(restarted(proc_info, pid, RESTART_DELAY + 10), "the container did not start again")
        self.assertIsNotNone(self.logged("skip_missed", at, 40), self.log())
        p.flags(battery_percentage=90, charge_current=0)
        self.assertIsNone(p.event(since, "SCHEDULE_START", 10), p.events[since:])
        state = wait_until(lambda: (read_json(path, tries=5) or {"waiting": True})["waiting"] is None
                           and read_json(path, tries=5), 10)
        self.assertTrue(state, read_json(path))
        self.assertIn(clock_time(at), state["handled"][time.strftime("%Y-%m-%d", time.gmtime(at))], state)
        self.set_schedule(enabled=0)
        self.assertTrue(wait_until(lambda: self.broker.ready() == {}, 5), self.broker.ready())
