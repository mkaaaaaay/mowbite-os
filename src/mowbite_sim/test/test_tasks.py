"""Two of three areas in the simulation: mowing.start with tasks names them in their order. The first one is mowed,
the second skipped by the app, the one not in the list is never reached, and the job completes after the list.
robot_state carries the place in the list (task_index, task_count) next to the index in the map (current_area). A list
that does not fit is refused with -32602 and its field before anything moves.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_tasks.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

import rclpy
from mowbite_msgs.msg import MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest, RpcResponse
from mowbite_msgs.srv import ReplaceMap, SetPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, SimProbe, sim_description, sim_files  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_tasks_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
CONFIG = os.path.join(WORK, "tasks.yaml")


def strip(x0, x1):
    return [{"x": x, "y": y} for x, y in [(x0, -0.11), (x1, -0.11), (x1, 1.49), (x0, 1.49)]]


# Three strips side by side; the first is the lawn of test_mowing, its loop starts at the mower (0, 0).
GARDEN = {
    "areas": [{"id": area_id, "properties": {"type": "mow", "name": area_id.title(), "outline_count": 1},
               "outline": strip(x0, x1)}
              for area_id, x0, x1 in [("near", -0.41, 2.59), ("middle", 2.59, 3.59), ("far", 3.59, 4.59)]],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    f.write("tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18840, extra_config=CONFIG)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("tasks_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.start()
        self.spawn(self._record)


def tasks(*ids):
    return {"tasks": [{"area_id": i} for i in ids], "client": "test"}


class TestTasks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_three_areas_and_an_aligned_heading(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.datum_state != MapStatus.DATUM_UNKNOWN, 30),
                        "the map never learnt the datum")
        request = ReplaceMap.Request()
        request.map = json.dumps(GARDEN)
        answer = p.call(p.replace_map, request)
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(p.wait(lambda: p.status.mowing_allowed, 10), "the map does not fit the datum")
        request = SetPose.Request()
        request.pose.orientation.w = 1.0
        p.call(p.set_pose, request)
        self.assertTrue(p.wait(lambda: p.pose is not None and p.pose.heading_valid
                               and p.pose.position_accuracy < 0.1, 30), "no good pose with a heading")

    def test_02_a_list_that_does_not_fit_moves_nothing(self):
        p = self.probe
        since = len(p.events)
        for params, field in [({"areas": ["near"]}, "areas"), (tasks("near", "gone"), "tasks"),
                              (tasks("near", "near"), "tasks"), ({"tasks": []}, "tasks")]:
            error = (p.rpc("mowing.start", params) or {}).get("error") or {}
            self.assertEqual(error.get("code"), -32602, params)
            self.assertEqual((error.get("data") or {}).get("field"), field, params)
        self.assertIsNone(p.event(since, "STATE", 2, state="MOWING"))
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 5))

    def test_03_mows_the_first_of_the_list(self):
        p = self.probe
        since, answer = p.started(lambda: p.rpc("mowing.start", tasks("near", "far")))
        self.assertIn("result", answer or {}, answer)
        state = p.event(since, "STATE", 10, state="MOWING")
        self.assertIsNotNone(state, [e["type"] for e in p.events[since:]])
        type(self).job = state["job_id"]
        self.assertIsNotNone(p.event(since, "AREA", 10, area_id="near"))
        self.assertTrue(p.wait(lambda: p.robot.current_state == "MOWING" and p.robot.current_area == 0
                               and p.robot.task_index == 0 and p.robot.task_count == 2, 30),
                        f"{p.robot.current_area} {p.robot.task_index} {p.robot.task_count}")

    def test_04_then_the_second_which_the_app_skips(self):
        p = self.probe
        far = p.event(0, "AREA", 600, area_id="far")
        self.assertIsNotNone(far, "the first area did not end")
        # current_area stays the index in the map.
        self.assertTrue(p.wait(lambda: p.robot.current_area == 2 and p.robot.task_index == 1
                               and p.robot.task_count == 2, 10),
                        f"{p.robot.current_area} {p.robot.task_index} {p.robot.task_count}")
        answer = p.rpc("mowing.skip_area", {"area": 2, "client": "test"})
        self.assertIn("result", answer or {}, answer)
        skipped = p.event(0, "AREA_SKIPPED", 30, area_id="far")
        self.assertIsNotNone(skipped)

    def test_05_the_job_completes_after_the_list(self):
        p = self.probe
        done = p.event(0, "JOB_COMPLETE", 60)
        self.assertIsNotNone(done, "the job did not complete")
        self.assertEqual(done["job_id"], self.job)
        self.assertEqual([e["area_id"] for e in p.events if e["type"] == "AREA"], ["near", "far"])
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.robot.task_index == -1
                               and p.robot.task_count == -1, 10))

