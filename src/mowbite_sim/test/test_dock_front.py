"""Docking with the charging contacts in front (dock_contact_x above 0, like the NX60i) in the stack of sim.launch.py:
the mower faces the way in on the station. A job from the station undocks straight back at no more than docking_speed,
mows a small patch and drives forwards onto the contacts after JOB_COMPLETE, the last stretch at contact_speed;
docking.start from the field docks the same way. The driver never reverses faster than 0.12 m/s (safety filter).

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_front.py
"""
import functools
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import rclpy
from mowbite_msgs.msg import (HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcMethods,
                              RpcRequest, RpcResponse)
from mowbite_msgs.srv import DriveTo, ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HISTORY, LATCHED, SimProbe, host_packets, last_time, moving,  # noqa: E402
                        sim_description, sim_files, wrap)

WORK = tempfile.mkdtemp(prefix="mowbite_dock_front_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "dock_front.yaml")
START = "mower_logic:idle/start_mowing"
CONTACT_X = 0.3
CONTACT_SPEED = 0.04

# The station of test_dock.py: the docked mower at (-1, 0), the way in going west, so with the contacts in front it
# faces west there. The line onto it begins at (-0.2, 0), the gate is at (-0.75, 0). The yard reaches under the
# station: off it the 0.6 m nose of the Tron body points that way.
YARD = [(-1.2, -2.0), (4.0, -2.0), (4.0, 2.0), (-1.2, 2.0)]
PATCH = [(1.0, -0.5), (2.6, -0.5), (2.6, 0.5), (1.0, 0.5)]
DOCKED = [-1.0, 0.0, 180.0]
GARDEN = {"areas": [{"id": "yard", "properties": {"type": "nav", "name": "Yard"},
                     "outline": [{"x": x, "y": y} for x, y in YARD]},
                    {"id": "patch", "properties": {"type": "mow", "name": "Patch", "outline_count": 1},
                     "outline": [{"x": x, "y": y} for x, y in PATCH]}],
          "docking_stations": [{"id": "dock", "position": {"x": -1.0, "y": 0.0}, "heading": math.pi}]}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n"
            "mowbite_logic:\n  ros__parameters:\n    dock_guide: none\n"
            f"    contact_speed: {CONTACT_SPEED}\n    geometry:\n"
            f"      antenna_x: 0.098\n      antenna_y: 0.0\n      dock_contact_x: {CONTACT_X}\n"
            f"fake_tron:\n  ros__parameters:\n    dock: [-1.0, 0.0, 180.0]\n    dock_contact_x: {CONTACT_X}\n")


def generate_test_description():
    return sim_description(WORK, 18896, extra_config=CONFIG)


received = functools.partial(host_packets, RX)
sim_time = functools.partial(last_time, RX)


class Probe(SimProbe):
    truth_tries = 5

    def __init__(self):
        super().__init__("dock_front_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(String, "map/json", self._map, LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.drive_to = self.node.create_client(DriveTo, "logic/drive_to")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()


class TestDockFront(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        # The hollows of the Tron's station hold wheels slower than 0.10 m/s; the NX60i's station has none of that size.
        cls.probe.flags(dock_hollow_speed=0.0)

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def on_the_station(self):
        truth = self.probe.truth()
        self.assertAlmostEqual(truth["x"], -1.0, delta=0.01)
        self.assertLess(abs(truth["y"]), 0.02, truth)
        self.assertLess(abs(wrap(truth["yaw"] - math.pi)), math.radians(5), truth)
        self.assertTrue(truth["dock_ok"], truth)

    def test_01_on_the_station(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        self.assertTrue(p.call(p.replace_map, request).success)
        self.assertTrue(p.wait(lambda: p.map and p.map["docking_stations"], 10), "the map did not come")
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy < 0.03, 40), "no RTK position")
        since = len(p.events)
        p.flags(dock_hollow_speed=0.0, place=DOCKED)
        self.assertTrue(p.wait(lambda: p.hw and p.hw.dock_contact, 10), "no contact on the station")
        # Carried there facing west: the gyro of the simulation does not see the turn.
        request = SetPose.Request()
        request.pose.position.x, request.pose.position.y = DOCKED[0], DOCKED[1]
        request.pose.orientation.z, request.pose.orientation.w = 1.0, 0.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose and p.pose.position_accuracy <= 0.03 and abs(p.pose.x + 1.0) < 0.05, 30))
        p.flags(dock_hollow_speed=0.0)
        self.assertIsNotNone(p.event(since, "DOCKED", 20), p.events[since:])
        # Docking is offered with the contacts in front too.
        self.assertTrue(p.wait(lambda: "docking.start" in p.methods.get("mowbite_logic", []), 10), p.methods)

    def test_02_a_job_from_the_station_backs_out_and_docks_forwards(self):
        p = self.probe
        sim_since, since = sim_time(), len(p.events)
        p.act(START)
        undocked = p.event(since, "UNDOCKED", 40)
        self.assertIsNotNone(undocked, p.events[since:])
        undocked_at = sim_time()
        self.assertIsNotNone(p.event(since, "JOB_COMPLETE", 600), p.events[since:])
        docking_from = len(p.events)
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[docking_from:])
        docked_at = sim_time()
        after = p.events[since:]
        self.assertFalse([e for e in after if e["type"] in ("DOCKING_FAILED", "UNDOCKING_FAILED")], after)
        self.on_the_station()
        rx = received(sim_since)
        # Off the station straight back, never faster than docking_speed (speed is [linear, angular]).
        out = [e for e in rx if e["t"] <= undocked_at and moving(e)]
        self.assertTrue(out, rx[:5])
        self.assertEqual([e for e in out if not -0.1 - 1e-3 <= e["speed"][0] < 0], [])
        # Onto it forwards, the last stretch at contact_speed; nothing on the whole run reverses faster than 0.12 m/s.
        moves = [e for e in rx if e["t"] <= docked_at and moving(e)]
        last = [e for e in moves if e["t"] >= moves[-1]["t"] - 2.0]
        self.assertEqual([e for e in last if not 0 <= e["speed"][0] <= CONTACT_SPEED + 1e-3], [])
        slowest = min(moves, key=lambda e: e["speed"][0])
        self.assertGreaterEqual(slowest["speed"][0], -0.12 - 1e-3, slowest)

    def test_03_docking_start_from_the_field(self):
        p = self.probe
        request = DriveTo.Request()
        request.x, request.y = 1.5, 0.8
        since = len(p.events)
        for _ in range(10):
            answer = p.call(p.drive_to, request)
            if answer.success or "still stopping" not in answer.message:
                break
            time.sleep(0.3)
        self.assertTrue(answer.success, answer.message)
        end = p.event(since, "DRIVE", 120, phase="end")
        self.assertTrue(end and end.get("arrived"), p.events[since:])
        since = len(p.events)
        self.assertEqual(p.rpc("docking.start"), {"result": {"state": "DOCKING", "sub_state": ""}})
        self.assertIsNotNone(p.event(since, "DOCKED", 150), p.events[since:])
        self.on_the_station()
