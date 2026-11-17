"""robot.geometry of mowbite_logic: the test plays the bridge (bridge/rpc/request) and the driver, and reads what the
logic answers, saves, reports on bridge/params and as events, and what reaches the localization. The logic starts with
the file of a user saved before.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_geometry.py
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
import rclpy
import yaml
from launch_ros.actions import Node
from mowbite_msgs.msg import ActionList, HardwareStatus, MqttPublish, RpcError, RpcMethods, RpcRequest, RpcResponse
from mowbite_msgs.msg import RobotPose
from rcl_interfaces.msg import Log
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, LATCHED, ROSOUT, SimProbe, TestShutdown, layers  # noqa: E402,F401

DATUM = (52.0, 9.0, 50.0)
WORK = tempfile.mkdtemp(prefix="mowbite_geometry_")
FILE = os.path.join(WORK, "geometry.json")
GEOMETRY = layers.geometry("tron")
G = "/mowbite_geometry/"
with open(GEOMETRY) as f:
    PLATFORM = yaml.safe_load(f)["mowbite_logic"]["ros__parameters"]["geometry"]


def pairs(flat):
    return [list(p) for p in zip(flat[::2], flat[1::2])]


OUTLINE = pairs(PLATFORM["robot_outline"])
SHAPE = PLATFORM["robot_shape"]


def generate_test_description():
    with open(FILE, "w") as f:
        json.dump({"version": 7, "entries": {"antenna_x": 0.12, "blade_ahead": 0.25}}, f)
    nodes = [
        Node(package="mowbite_map", executable="mowbite_map_node", output="screen", parameters=[{"data_dir": WORK}]),
        Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen",
             parameters=[GEOMETRY, {"data_dir": WORK}]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", output="screen"),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {}


class Probe(SimProbe):
    def __init__(self):
        super().__init__("geometry_probe")
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.pose_pub = self.node.create_publisher(RobotPose, "localization/pose", 10)
        self.datum_pub = self.node.create_publisher(NavSatFix, "gps/datum", LATCHED)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._result, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._failure, 10)
        self.node.create_subscription(String, "bridge/params", self._params, HISTORY)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(ActionList, "bridge/actions", self._actions, HISTORY)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.start()
        self.spawn(self._keepalive)

    def call(self, rid, method, params, raw=None):
        """The method as the bridge hands it on; the answer, or the error as (code, message, data); None without one."""
        self._send(method, params, raw, rid)
        return self.wait(lambda: self.responses.get(rid) or self.errors.get(rid), 5.0)

    def geometry_events(self):
        with self.lock:
            return [e for e in self.events if e["type"] == "GEOMETRY_CHANGED"]


def saved():
    with open(FILE) as f:
        return json.load(f)


class TestGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def set(self, rid, version, settings=None, reset=None):
        request = {"version": version}
        if settings is not None:
            request["settings"] = settings
        if reset is not None:
            request["reset"] = reset
        return self.probe.call(rid, "robot.geometry.set", request)

    def test_1_starts_with_the_file_of_the_user(self):
        methods = self.probe.wait(lambda: self.probe.methods.get("mowbite_logic"))
        self.assertIn("robot.geometry.get", methods)
        self.assertIn("robot.geometry.set", methods)
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(G + "version") == 7), str(self.probe.params))
        params = self.probe.params
        self.assertEqual(params[G + "antenna_x"], 0.12)
        self.assertEqual(params[G + "blade_ahead"], 0.25)
        self.assertEqual(params[G + "robot_width"], 0.47)
        self.assertEqual(params[G + "robot_front"], 0.60)
        self.assertEqual(params[G + "dock_contact_x"], -0.10)
        self.assertEqual(params[G + "track"], 0.395)
        # The outline of the platform file, as pairs.
        self.assertEqual(len(OUTLINE), 6)
        self.assertEqual(params[G + "robot_outline"], OUTLINE)
        # No footprint of the station on the Tron yet: the planner's own.
        self.assertEqual(params[G + "dock_footprint"], [])
        self.assertTrue(self.probe.logged("Dimensions of the mower from " + FILE + " (version 7)"))
        # The localization takes the antenna from it and tells the app.
        self.assertTrue(self.probe.logged("Antenna offset: 0.120, 0.000"))
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get("/xbot_positioning/antenna_offset_x") == 0.12),
                        str(self.probe.params))

    def test_2_get_gives_value_default_and_stored(self):
        answer = self.probe.call("get", "robot.geometry.get", None, raw="")
        self.assertEqual(answer["platform"], "airseekers_tron")
        self.assertEqual(answer["version"], 7)
        settings = answer["settings"]
        self.assertEqual(settings["blade_ahead"]["value"], 0.25)
        self.assertEqual(settings["blade_ahead"]["default"], 0.23)
        self.assertTrue(settings["blade_ahead"]["stored"])
        self.assertFalse(settings["robot_width"]["stored"])
        self.assertEqual(settings["robot_width"]["min"], 0.1)
        self.assertEqual(settings["robot_width"]["max"], 1.5)
        self.assertEqual(settings["robot_outline"]["type"], "points")
        for name in ("wheel_radius", "wheel_width", "dock_contact_x", "dock_footprint"):
            self.assertFalse(settings[name]["settable"], name)
        for name in ("robot_width", "mower_width", "antenna_x", "blade_offset", "track", "ticks_per_m", "imu_pitch"):
            self.assertTrue(settings[name]["settable"], name)
        self.assertEqual(list(settings)[:4], ["robot_width", "robot_front", "robot_rear", "robot_outline"])

    def test_3_set_answers_saves_publishes_and_tells(self):
        answer = self.set("set", 7, {"blade_ahead": 0.3, "antenna_x": 0.15, "antenna_y": -0.02})
        self.assertEqual(answer["version"], 8)
        self.assertEqual(answer["settings"]["blade_ahead"]["value"], 0.3)
        self.assertEqual(answer["settings"]["blade_ahead"]["default"], 0.23)
        self.assertTrue(answer["settings"]["antenna_y"]["stored"])
        file = saved()
        self.assertEqual((file["version"], file["entries"]),
                         (8, {"blade_ahead": 0.3, "antenna_x": 0.15, "antenna_y": -0.02}))
        # What the user set counts as confirmed by them, with the time.
        self.assertEqual(sorted(file["confirmed"]), ["antenna_x", "antenna_y", "blade_ahead"])
        self.assertAlmostEqual(file["confirmed"]["blade_ahead"], time.time(), delta=10)
        self.assertEqual(answer["settings"]["blade_ahead"]["confirmed"], file["confirmed"]["blade_ahead"])
        self.assertIsNone(answer["settings"]["track"]["confirmed"])
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(G + "version") == 8))
        self.assertEqual(self.probe.params[G + "blade_ahead"], 0.3)
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get("/xbot_positioning/antenna_offset_x") == 0.15))
        self.assertEqual(self.probe.params["/xbot_positioning/antenna_offset_y"], -0.02)
        self.assertTrue(self.probe.logged("Antenna offset: 0.150, -0.020 (the geometry changed)"))
        events = self.probe.wait(lambda: self.probe.geometry_events())
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event["changed"], ["blade_ahead", "antenna_x", "antenna_y"])
        self.assertEqual(event["below_default"], [])
        self.assertEqual(event["version"], 8)
        self.assertEqual(event["severity"], "info")
        self.assertNotIn("reason", event)
        self.assertNotIn("resume", event)
        self.assertIn("blade_ahead", event["message"])

    def test_4_a_smaller_body_than_the_platforms_warns(self):
        self.assertIsNotNone(self.set("smaller", 8, {"robot_width": 0.40}))
        events = self.probe.wait(lambda: len(self.probe.geometry_events()) == 2 and self.probe.geometry_events())
        event = events[1]
        self.assertEqual(event["changed"], ["robot_width"])
        self.assertEqual(event["below_default"], ["robot_width"])
        self.assertEqual(event["severity"], "warning")
        self.assertEqual(event["reason"], "BELOW_DEFAULT")
        self.assertNotIn("resume", event)
        self.assertEqual(event["version"], 9)
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(G + "robot_width") == 0.40))

    def test_5_reset_goes_back_to_the_platforms_values(self):
        answer = self.set("reset", 9, reset=["robot_width", "blade_ahead"])
        self.assertEqual(answer["version"], 10)
        self.assertEqual(answer["settings"]["robot_width"]["value"], 0.47)
        self.assertFalse(answer["settings"]["robot_width"]["stored"])
        self.assertEqual(answer["settings"]["blade_ahead"]["value"], 0.23)
        file = saved()
        self.assertEqual((file["version"], file["entries"]), (10, {"antenna_x": 0.15, "antenna_y": -0.02}))
        # A key back at the platform's value is not confirmed any more.
        self.assertEqual(sorted(file["confirmed"]), ["antenna_x", "antenna_y"])
        self.assertIsNone(answer["settings"]["robot_width"]["confirmed"])
        events = self.probe.wait(lambda: len(self.probe.geometry_events()) == 3 and self.probe.geometry_events())
        self.assertEqual(events[2]["changed"], ["robot_width", "blade_ahead"])
        self.assertEqual(events[2]["severity"], "info")
        self.assertTrue(self.probe.wait(lambda: self.probe.params.get(G + "blade_ahead") == 0.23))

    def test_6_a_bad_call_changes_nothing(self):
        before = saved()
        events = len(self.probe.geometry_events())
        cases = {
            "stale": ({"version": 3, "settings": {"blade_ahead": 0.2}}, "VERSION", "version"),
            "range": ({"version": 10, "settings": {"robot_width": 47}}, "OUT_OF_RANGE", "robot_width"),
            "fixed": ({"version": 10, "settings": {"wheel_radius": 0.1}}, "NOT_SETTABLE", "wheel_radius"),
            "unknown": ({"version": 10, "settings": {"nope": 1}}, "UNKNOWN_KEY", "nope"),
            "text": ({"version": 10, "settings": {"blade_ahead": "0.2"}}, "INVALID_VALUE", "blade_ahead"),
            "sign": ({"version": 10, "settings": {"blade_ahead": -0.23}}, "OUTSIDE_BODY", "blade_ahead"),
            "empty": ({"version": 10}, "INVALID_REQUEST", None),
        }
        for rid, (request, reason, key) in cases.items():
            answer = self.probe.call(rid, "robot.geometry.set", request)
            self.assertEqual(answer[0], RpcError.ERROR_INVALID_PARAMS, rid)
            self.assertEqual(answer[2]["reason"], reason, rid)
            self.assertEqual(answer[2].get("key"), key, rid)
            self.assertEqual(answer[2]["message"], answer[1], rid)
        stale = self.probe.errors["stale"]
        self.assertEqual(stale[2]["version"], 10)
        self.assertEqual(self.probe.errors["range"][2]["min"], 0.1)
        self.assertEqual(self.probe.errors["range"][2]["max"], 1.5)
        broken = self.probe.call("broken", "robot.geometry.set", None, raw="{broken")
        self.assertEqual(broken[:2], (RpcError.ERROR_INVALID_PARAMS, "The parameters are not valid JSON"))
        self.assertEqual(saved(), before)
        self.assertEqual(len(self.probe.geometry_events()), events)
        self.assertEqual(self.probe.params[G + "version"], 10)

    def test_7_only_in_idle(self):
        self.probe.datum_pub.publish(NavSatFix(latitude=DATUM[0], longitude=DATUM[1], altitude=DATUM[2]))
        start = "mower_logic:idle/start_area_recording"
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get(start)), "start_area_recording stays off")
        self.probe.act(start)
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get("mower_logic:area_recording/start_recording")))
        before = saved()
        answer = self.set("recording", 10, {"blade_offset": 0.01})
        self.assertEqual(answer[:2], (RpcError.ERROR_INVALID_REQUEST,
                                      "The dimensions change only in IDLE, not while recording, driving or mowing"))
        self.assertEqual(answer[2], {"reason": "BUSY"})
        self.assertEqual(saved(), before)
        self.probe.act("mower_logic:area_recording/exit_recording_mode")
        self.assertTrue(self.probe.wait(lambda: self.probe.actions.get(start)))
        answer = self.set("idle", 10, {"blade_offset": 0.01})
        self.assertEqual(answer["version"], 11)
        self.assertEqual(saved()["entries"]["blade_offset"], 0.01)

    def test_8_a_set_that_changes_nothing_keeps_the_version(self):
        events = len(self.probe.geometry_events())
        answer = self.set("same", 11, {"blade_offset": 0.01})
        self.assertEqual(answer["version"], 11)
        self.assertEqual(saved()["version"], 11)
        time.sleep(0.5)
        self.assertEqual(len(self.probe.geometry_events()), events)

    def test_9_a_save_that_fails_changes_nothing(self):
        before = saved()
        events = len(self.probe.geometry_events())
        # A folder where the new file is written first: the save fails, as it would with a full or broken card.
        os.mkdir(FILE + ".tmp")
        try:
            answer = self.set("nosave", 11, {"blade_offset": 0.02})
        finally:
            os.rmdir(FILE + ".tmp")
        self.assertEqual(answer[0], RpcError.ERROR_INTERNAL)
        self.assertTrue(answer[1].startswith("Cannot save the dimensions: "), answer[1])
        self.assertEqual(saved(), before)
        self.assertEqual(self.probe.call("get_after", "robot.geometry.get", None, raw="")["version"], 11)
        self.assertEqual(self.probe.params[G + "blade_offset"], 0.01)
        time.sleep(0.5)
        self.assertEqual(len(self.probe.geometry_events()), events)
        self.assertEqual(self.set("again", 11, {"blade_offset": 0.02})["version"], 12)

    def test_a_the_shape_of_the_platform_is_shown_not_set(self):
        answer = self.probe.call("shape", "robot.geometry.get", None, raw="")
        shape = answer["settings"]["robot_shape"]
        self.assertEqual((shape["type"], shape["unit"], shape["settable"], shape["stored"]),
                         ("shape", "m", False, False))
        self.assertNotIn("confirmed", shape)
        self.assertEqual([part["id"] for part in shape["value"]], SHAPE["parts"])
        for part in shape["value"]:
            file = SHAPE[part["id"]]
            self.assertEqual((part["kind"], part["points"], part["closed"]),
                             (file["kind"], pairs(file["points"]), file.get("closed", True)), part["id"])
        self.assertEqual(shape["default"], shape["value"])
        self.assertNotIn(G + "robot_shape", self.probe.params)
        refused = self.set("shape_set", answer["version"], {"robot_shape": []})
        self.assertEqual((refused[2]["reason"], refused[2]["key"]), ("NOT_SETTABLE", "robot_shape"))

    def test_b_confirming_changes_no_value(self):
        events = len(self.probe.geometry_events())
        request = {"version": 12, "confirm": ["track", "robot_outline"]}
        answer = self.probe.call("confirm", "robot.geometry.set", request)
        self.assertEqual(answer["version"], 12)
        for key in request["confirm"]:
            self.assertAlmostEqual(answer["settings"][key]["confirmed"], time.time(), delta=10, msg=key)
            self.assertFalse(answer["settings"][key]["stored"], key)
        self.assertIsNone(answer["settings"]["mower_width"]["confirmed"])
        confirmed = ["antenna_x", "antenna_y", "blade_offset", "robot_outline", "track"]
        self.assertEqual(sorted(saved()["confirmed"]), confirmed)
        self.assertEqual(saved()["version"], 12)
        self.assertTrue(self.probe.wait(lambda: sorted(self.probe.params.get(G + "confirmed", [])) == confirmed),
                        str(self.probe.params.get(G + "confirmed")))
        cases = {
            "confirm_fixed": ({"version": 12, "confirm": ["wheel_radius"]}, "NOT_SETTABLE"),
            "confirm_unknown": ({"version": 12, "confirm": ["nope"]}, "UNKNOWN_KEY"),
            "confirm_text": ({"version": 12, "confirm": "track"}, "INVALID_REQUEST"),
            "confirm_reset": ({"version": 12, "reset": ["track"], "confirm": ["track"]}, "INVALID_REQUEST"),
        }
        for rid, (wrong, reason) in cases.items():
            refused = self.probe.call(rid, "robot.geometry.set", wrong)
            self.assertEqual((refused[0], refused[2]["reason"]), (RpcError.ERROR_INVALID_PARAMS, reason), rid)
        time.sleep(0.5)
        self.assertEqual(len(self.probe.geometry_events()), events)

    def test_c_an_outline_that_leaves_out_a_part_is_told(self):
        events = len(self.probe.geometry_events())
        # The bar reaches as far as the outline of the platform: 2 cm less in front leaves it out.
        front = max(x for x, _ in OUTLINE)
        short = [[x - 0.02 if x == front else x, y] for x, y in OUTLINE]
        answer = self.set("short", 12, {"robot_outline": short})
        self.assertEqual(answer["version"], 13)
        event = self.probe.wait(
            lambda: len(self.probe.geometry_events()) > events and self.probe.geometry_events()[-1])
        self.assertEqual(event["changed"], ["robot_outline"])
        outside = {part["part"]: part["by"] for part in event["outside_shape"]}
        self.assertLessEqual({"bumper_left", "bumper_right"}, set(outside))
        self.assertLessEqual(set(outside), set(SHAPE["parts"]))
        self.assertTrue(0.005 < outside["bumper_left"] <= 0.02, outside)
        self.assertEqual(outside["bumper_left"], outside["bumper_right"])
        self.assertEqual(event["severity"], "warning")
        self.assertEqual(event["reason"], "BELOW_DEFAULT" if event["below_default"] else "SHAPE_OUTSIDE")
        self.assertIn("of the shape of the mower", event["message"])
        # Back at the outline of the platform nothing is left out.
        self.assertEqual(self.set("whole", 13, reset=["robot_outline"])["version"], 14)
        event = self.probe.wait(
            lambda: len(self.probe.geometry_events()) > events + 1 and self.probe.geometry_events()[-1])
        self.assertNotIn("outside_shape", event)
        self.assertEqual(event["severity"], "info")
