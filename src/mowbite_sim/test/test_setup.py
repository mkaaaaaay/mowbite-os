"""setup.* of mowbite_setup with the step list of a platform: the test plays the bridge (bridge/rpc/request), the driver
(hw/status, hw/capabilities, imu/data, gps/absolute_pose) and the other nodes (robot state, map, corrections,
geometry) and does every step of the list by hand, as a user would, until the program is complete. Nothing moves.
The emergency a press of an input brings is released by the program, over safety/release_emergency, which the test
plays as well. A step inputs_assign is left out of the list: it restarts the OS and needs a driver that learns
(test_nx60i_inputs.py).

Run in a sourced workspace: MOWBITE_PLATFORM=<platform> launch_test /ws/src/mowbite_sim/test/test_setup.py
"""
import base64
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from diagnostic_msgs.msg import KeyValue
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from mowbite_msgs.msg import GnssFix, HardwareCapabilities, HardwareStatus, MapStatus, MqttPublish, NtripStatus
from mowbite_msgs.msg import RobotState, RpcError, RpcMethods, RpcRequest, RpcResponse
from mowbite_msgs.srv import SetEmergency
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import HISTORY, LATCHED, SimProbe, TestShutdown, layers, wait_until  # noqa: E402,F401

PLATFORM = os.environ["MOWBITE_PLATFORM"]
WORK = tempfile.mkdtemp(prefix="mowbite_setup_")
FILE = os.path.join(WORK, "setup.json")
SETUP = layers.setup(PLATFORM)
LIST = json.loads(SETUP["steps"]) if SETUP["steps"] else {"catalog": 0, "steps": []}
LIST["steps"] = [step for step in LIST["steps"] if step["check"] != "inputs_assign"]
SETUP["steps"] = json.dumps(LIST) if LIST["steps"] else ""
STEPS = {step["id"]: step for step in LIST["steps"]}
BOARD = next((step["id"] for step in LIST["steps"] if step["check"] == "board"), None)
METHODS = ["setup.get", "setup.start", "setup.answer", "setup.skip", "setup.abort", "setup.reset", "setup.image"]
G = 9.81
FIRMWARE = "main 1.2.3"
# The emergency a press of an input brings where the step names none.
EMERGENCY = {"stop": "STOP", "lift": "LIFT", "bumper": "COLLISION", "bumper_left": "COLLISION",
             "bumper_right": "COLLISION"}


def generate_test_description():
    node = Node(package="mowbite_setup", executable="mowbite_setup_node", output="screen",
                parameters=[{"data_dir": WORK},
                            {name: ParameterValue(value, value_type=str) for name, value in SETUP.items()}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


def param(step, name, default):
    return STEPS[step].get("params", {}).get(name, default)


def bit(name):
    return 0 if name == "NONE" else getattr(HardwareStatus, "REASON_" + name)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("setup_probe")
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.hw_pub = self.node.create_publisher(HardwareStatus, "hw/status", 10)
        self.caps_pub = self.node.create_publisher(HardwareCapabilities, "hw/capabilities", LATCHED)
        self.imu_pub = self.node.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.fix_pub = self.node.create_publisher(GnssFix, "gps/absolute_pose", 10)
        self.state_pub = self.node.create_publisher(RobotState, "bridge/robot_state", 10)
        self.map_pub = self.node.create_publisher(MapStatus, "map/status", LATCHED)
        self.ntrip_pub = self.node.create_publisher(NtripStatus, "gnss/ntrip_status", 10)
        self.params_pub = self.node.create_publisher(String, "bridge/params", HISTORY)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._result, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._failure, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_service(SetEmergency, "safety/release_emergency", self._release)
        # What the driver and the other nodes report, changed by the test.
        self.driver = True
        self.inputs = {}
        self.reason = 0
        # latch_count of hw/status: one more with every reason that latches.
        self.latches = 0
        # Every release asked for, as (latch_count, reason).
        self.releases = []
        self.accel = (0.0, 0.0, G)
        self.turn = 0.0
        self.fix = None
        self.ntrip = False
        self.state = "IDLE"
        self.sub = ""
        # Every setup/json with whether it came retained.
        self.states = []
        firmware = [KeyValue(key="main", value="1.2.3")]
        self.caps_pub.publish(HardwareCapabilities(platform=SETUP["platform"], firmware=firmware))
        self.start()
        self.spawn(self._drive)

    def _publish(self, msg):
        if msg.topic == "setup/json":
            with self.lock:
                self.states.append((json.loads(msg.payload), msg.retain))
        super()._publish(msg)

    def _drive(self):
        """The IMU at 50 Hz, hw/status at 10 Hz, positions and the state of the mower at 5 Hz, corrections at 2 Hz."""
        count = 0
        while self.running:
            with self.lock:
                driver, inputs, reason, latches = self.driver, dict(self.inputs), self.reason, self.latches
                accel, turn, fix, ntrip, state = self.accel, self.turn, self.fix, self.ntrip, self.state
                sub = self.sub
            imu = Imu()
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel
            imu.angular_velocity.z = turn
            self.imu_pub.publish(imu)
            if driver and count % 5 == 0:
                hw = HardwareStatus(telemetry_ok=True, battery_source=HardwareStatus.BATTERY_SOURCE_BMS,
                                    battery_voltage=24.5, battery_percentage=80, emergency=reason != 0,
                                    emergency_reason=reason, emergency_latched=reason != 0, latch_count=latches)
                for motor in (hw.blade_motor, hw.left_motor, hw.right_motor):
                    motor.ok = True
                for name, on in inputs.items():
                    setattr(hw, name, on)
                self.hw_pub.publish(hw)
            if count % 10 == 0:
                if fix:
                    self.fix_pub.publish(GnssFix(solution=fix[0], accuracy=fix[1]))
                self.state_pub.publish(RobotState(current_state=state, current_sub_state=sub))
            if ntrip and count % 25 == 0:
                self.ntrip_pub.publish(NtripStatus(state="connected", frames=5, last_frame_age=0.5))
            count += 1
            time.sleep(0.02)

    def _release(self, request, response):
        """The safety filter and the driver in one: a release with the latch count of now ends the emergency."""
        with self.lock:
            self.releases.append((request.latch_count, request.reason))
            response.success = not request.latch and request.latch_count == self.latches
            if response.success:
                self.reason = 0
            else:
                response.code = "LATCH_CHANGED"
        return response

    def put(self, **values):
        """Sets what the driver and the other nodes report from now on."""
        with self.lock:
            if values.get("reason", 0) & ~self.reason:
                self.latches += 1
            for name, value in values.items():
                setattr(self, name, value)

    def switch(self, **inputs):
        """Inputs of hw/status by name."""
        with self.lock:
            self.inputs.update(inputs)

    def tilt(self, pitch=0.0, roll=0.0):
        """The mower held with its front (pitch) or its left side (roll) up by that many degrees."""
        most = math.radians(max(abs(pitch), abs(roll)))
        self.put(accel=(G * math.sin(math.radians(pitch)), G * math.sin(math.radians(roll)), G * math.cos(most)))

    def mower(self, *states):
        """The states of the logic one after the other, each long enough to be seen."""
        for state in states:
            self.put(state=state)
            time.sleep(0.5)

    def call(self, method, params=None):
        """("ok", result) or ("error", (code, message, data)) as the node answers."""
        outcome = self.rpc_outcome(method, params)
        assert outcome is not None, f"no answer to {method}"
        return outcome

    def get(self):
        kind, state = self.call("setup.get")
        assert kind == "ok", state
        return state

    def step(self, step):
        return next(entry for entry in self.get()["steps"] if entry["id"] == step)

    def do(self, method, step, **params):
        """A method that changes a step, with the version of now; its own entry of the state it answers."""
        kind, answer = self.call(method, {"step": step, "version": self.get()["version"], **params})
        assert kind == "ok", f"{method} {step}: {answer}"
        return next(entry for entry in answer["steps"] if entry["id"] == step)

    def refused(self, method, params):
        """The data of the error a method ends with."""
        kind, answer = self.call(method, params)
        assert kind == "error", f"{method} {params}: {answer}"
        return answer

    def step_when(self, step, pred, timeout):
        """The entry of the step once pred holds for it, else None."""
        def look():
            entry = self.step(step)
            return entry if pred(entry) else None
        return wait_until(look, timeout)

    def setup_events(self, since=0):
        with self.lock:
            return [e for e in self.events[since:] if e["type"] in ("SETUP_STEP", "SETUP")]


def saved():
    with open(FILE) as f:
        return json.load(f)


@unittest.skipUnless(STEPS, f"{PLATFORM} has no step list")
class TestSetup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def part_ok(self, step, index, timeout=10.0):
        """Waits until the part is done; the entry of the step."""
        entry = self.probe.step_when(
            step, lambda e: e["state"] != "running" or e["parts"][index]["state"] == "ok", timeout)
        self.assertIsNotNone(entry, f"{step} part {index} stays open")
        self.assertIn(entry["state"], ("running", "ok"), f"{step}: {entry['result']}")
        self.assertEqual(entry["parts"][index]["state"], "ok", f"{step}: {entry['result']}")
        return entry

    def part_ready(self, step, index):
        """Waits until the part waits for the user with nothing in its way."""
        entry = self.probe.step_when(
            step, lambda e: e["state"] == "running" and e["parts"][index]["state"] == "waiting" and not e["blocked"],
            10.0)
        self.assertIsNotNone(entry, f"{step} part {index} does not begin: {self.probe.step(step)}")
        self.assertEqual(entry["progress"], {"n": index + 1, "of": len(entry["parts"])})
        # The program arms the part with its next look.
        time.sleep(0.3)

    # The checks, each done the way a user does it.

    def do_choice(self, step):
        name, value = param(step, "name", None), param(step, "values", None)[0]
        entry = self.probe.do("setup.answer", step, values={name: value})
        self.assertEqual(entry["inputs"][0]["value"], value)
        self.assertEqual([v["id"] for v in entry["inputs"][0]["values"]], param(step, "values", None))

    def do_number(self, step):
        name, value = param(step, "name", None), param(step, "min", 0)
        self.assertEqual(self.probe.do("setup.answer", step, values={name: value})["inputs"][0]["value"], value)

    def do_confirm(self, step):
        points = [point["id"] for point in self.probe.step(step)["confirm"]]
        self.probe.do("setup.start", step, confirm=points)

    def do_board(self, step):
        entry = self.probe.do("setup.start", step)
        self.assertEqual(entry["state"], "running")
        self.assertIsNotNone(entry["ends_at"])
        entry = self.probe.step_when(step, lambda e: e["state"] != "running", param(step, "time", 5) + 5)
        self.assertEqual(entry["state"], "ok", entry["result"])
        values = {value["id"]: value for value in entry["values"]}
        self.assertEqual([name for name, value in values.items() if value["ok"] is not True], [])
        self.assertAlmostEqual(values["status_rate"]["value"], 10.0, delta=1.5)
        self.assertEqual(values["firmware"]["value"], FIRMWARE)
        self.assertEqual(values["battery"]["value"], 80)
        self.assertAlmostEqual(values["gravity"]["value"], G, delta=0.05)

    def do_input(self, step):
        role = param(step, "role", None)
        reason = bit(param(step, "emergency", EMERGENCY[role]))
        within = param(step, "within", 1)
        self.assertEqual(self.probe.do("setup.start", step)["state"], "running")
        delays = []
        for index, part in enumerate(STEPS[step]["parts"]):
            signal = part.get("role", role)
            # A side of a split bumper comes with the bumper as a whole.
            names = {signal: True, "bumper": True} if signal.startswith("bumper") else {signal: True}
            self.part_ready(step, index)
            asked = len(self.probe.releases)
            self.probe.switch(**names)
            if part.get("mode", "press") == "short":
                time.sleep(0.3)
                self.probe.switch(**{name: False for name in names})
                self.assertEqual(self.probe.step(step)["parts"][index]["state"], "waiting")
                self.part_ok(step, index, within + 3)
                continue
            time.sleep(0.4)
            self.probe.put(reason=reason)
            time.sleep(0.4)
            # Held: the part waits for the release.
            self.assertEqual(self.probe.step(step)["parts"][index]["state"], "waiting")
            self.probe.switch(**{name: False for name in names})
            self.part_ok(step, index)
            if reason:
                delays.append(part["id"])
                # The program releases the emergency of its press itself, with the latch count the driver reports.
                self.assertTrue(self.probe.wait(lambda: self.probe.reason == 0, 5.0),
                                f"{step} {part['id']}: the emergency is not released")
                self.assertEqual(self.probe.releases[asked:], [(self.probe.latches, self.probe.releases[-1][1])])
                self.assertIn(signal, self.probe.releases[-1][1])
        entry = self.probe.step(step)
        self.assertEqual(entry["state"], "ok", entry["result"])
        self.assertEqual([value["id"] for value in entry["values"]], delays)
        for value in entry["values"]:
            self.assertAlmostEqual(value["value"], 0.4, delta=0.25, msg=f"{step} {value['id']}")
            self.assertEqual((value["unit"], value["max"], value["ok"]), ("s", within, True))

    def do_imu(self, step):
        tilt, turn = param(step, "tilt", 10), param(step, "turn", 30)
        self.assertEqual(self.probe.do("setup.start", step)["state"], "running")
        moves = {"front_up": {"pitch": tilt + 5}, "left_up": {"roll": tilt + 5}, "right_up": {"roll": -tilt - 5}}
        for index, move in enumerate(STEPS[step]["parts"]):
            self.part_ready(step, index)
            if move in moves:
                self.probe.tilt(**moves[move])
                self.part_ok(step, index)
                self.probe.tilt()
            elif move.startswith("turn"):
                self.probe.put(turn=1.0 if move == "turn_left" else -1.0)
                self.part_ok(step, index)
                self.probe.put(turn=0.0)
            else:
                self.assertEqual(move, "tilt_stop")
                self.probe.tilt(pitch=35)
                time.sleep(0.5)
                self.assertEqual(self.probe.step(step)["parts"][index]["state"], "waiting")
                self.probe.put(reason=HardwareStatus.REASON_TILT)
                self.part_ok(step, index)
                self.probe.tilt()
                self.probe.put(reason=0)
        entry = self.probe.step(step)
        self.assertEqual(entry["state"], "ok", entry["result"])
        for value, move in zip(entry["values"], STEPS[step]["parts"]):
            self.assertEqual((value["id"], value["unit"], value["ok"]), (move, "deg", True))
            least = turn if move.startswith("turn") else 30 if move == "tilt_stop" else tilt
            self.assertGreaterEqual(value["value"], least, move)

    def do_position(self, step):
        self.assertEqual([b["reason"] for b in self.probe.step(step)["blocked"]], ["NO_DATA"])
        self.probe.put(fix=(GnssFix.RTK_FIXED, 0.02))
        self.assertIsNotNone(self.probe.step_when(step, lambda e: not e["blocked"], 5.0))
        self.assertEqual(self.probe.do("setup.start", step)["state"], "running")
        hold = param(step, "time", 10)
        time.sleep(hold / 2)
        entry = self.probe.step(step)
        self.assertEqual(entry["state"], "running")
        values = {value["id"]: value for value in entry["values"]}
        self.assertEqual((values["solution"]["value"], values["solution"]["ok"]), ("fixed", True))
        self.assertEqual(values["accuracy"]["value"], 0.02)
        self.assertAlmostEqual(values["rate"]["value"], 5.0, delta=1.0)
        entry = self.probe.step_when(step, lambda e: e["state"] != "running", hold)
        self.assertEqual(entry["state"], "ok", entry["result"])

    def do_geometry(self, step):
        keys = param(step, "keys", [])
        text = json.dumps({"node": "/mowbite_geometry", "params": {"version": 3, "confirmed": keys[:-1]}})
        self.probe.params_pub.publish(String(data=text))
        time.sleep(0.5)
        self.assertEqual(self.probe.step(step)["state"], "open")
        text = json.dumps({"node": "/mowbite_geometry", "params": {"version": 3, "confirmed": keys}})
        self.probe.params_pub.publish(String(data=text))

    def do_done_when(self, step):
        condition = param(step, "condition", None)
        if condition == "ntrip":
            self.probe.put(ntrip=True)
        elif condition == "station":
            self.probe.map_pub.publish(MapStatus(docking_stations=1))
            time.sleep(0.5)
            self.assertEqual(self.probe.step(step)["state"], "open")
            self.probe.switch(dock_contact=True)
        elif condition == "area":
            self.probe.map_pub.publish(MapStatus(docking_stations=1, areas=1, mowing_allowed=True))
        else:
            # Off the contacts and back onto them by itself, with mowing in between for the trial run.
            self.probe.switch(dock_contact=False)
            self.probe.mower("UNDOCKING", *(["MOWING"] if condition == "mow" else []), "DOCKING")
            self.assertEqual(self.probe.step(step)["state"], "open")
            self.probe.put(state="IDLE")
            self.probe.switch(dock_contact=True)

    # The tests.

    def test_1_offers_the_methods_and_shows_the_list(self):
        self.assertEqual(self.probe.wait(lambda: self.probe.methods.get("mowbite_setup")), METHODS)
        state = self.probe.get()
        self.assertEqual((state["platform"], state["catalog"], state["complete"]),
                         (SETUP["platform"], LIST["catalog"], False))
        self.assertEqual([step["id"] for step in state["steps"]], list(STEPS))
        self.assertEqual(state["current"], LIST["steps"][0]["id"])
        self.assertEqual(state["stage"], LIST["steps"][0]["stage"])
        for step in state["steps"]:
            self.assertIn(step["state"], ("open", "locked"), step["id"])
            self.assertEqual((step["attempt"], step["result"], step["motion"]), (0, None, None), step["id"])
            for language in ("de", "en"):
                self.assertTrue(step["texts"][language]["name"], step["id"])
        self.assertEqual(state["safety"]["open"], False)
        self.assertFalse(os.path.exists(FILE))

    def test_2_the_images_come_in_pieces_with_their_hash(self):
        named = {}
        for step in self.probe.get()["steps"]:
            wanted = [STEPS[step["id"]].get(key) for key in ("image", "accent")]
            if not wanted[0]:
                self.assertIsNone(step["image"], step["id"])
                continue
            self.assertEqual(step["image"]["type"], "image/svg+xml")
            for name, entry in zip(wanted, (step["image"], step["image"]["accent"])):
                if name:
                    self.assertEqual(entry["name"], name)
                    named[name] = entry
                else:
                    self.assertIsNone(entry)
        self.assertEqual(sorted(named), sorted(os.listdir(SETUP["image_dir"])))
        for name, entry in named.items():
            with open(os.path.join(SETUP["image_dir"], name), "rb") as f:
                file = f.read()
            self.assertEqual((entry["bytes"], entry["sha256"]), (len(file), hashlib.sha256(file).hexdigest()), name)
            whole = self.probe.call("setup.image", {"name": name})[1]
            self.assertEqual((whole["size"], whole["offset"], whole["sha256"], whole["type"]),
                             (len(file), 0, entry["sha256"], "image/svg+xml"))
            self.assertEqual(base64.b64decode(whole["data"]), file)
            pieces = b""
            while len(pieces) < len(file):
                piece = self.probe.call("setup.image", {"name": name, "offset": len(pieces), "size": 700})[1]
                self.assertEqual(piece["offset"], len(pieces))
                pieces += base64.b64decode(piece["data"])
            self.assertEqual(pieces, file)
        for params, field in (({"name": "nothing.svg"}, "name"), ({}, "name"),
                              ({"name": next(iter(named)), "offset": -1}, "offset"),
                              ({"name": next(iter(named)), "size": 0}, "size")):
            code, _, data = self.probe.refused("setup.image", params)
            self.assertEqual((code, data), (RpcError.ERROR_INVALID_PARAMS, {"field": field, "reason": "INVALID"}))

    def test_3_what_is_refused_changes_nothing(self):
        state = self.probe.get()
        version = state["version"]
        board = BOARD
        waiting = next(step["id"] for step in LIST["steps"] if step["check"] == "done_when")
        cases = (("setup.start", {"step": board, "version": version - 1}, "VERSION"),
                 ("setup.start", {"step": "nothing", "version": version}, "UNKNOWN_STEP"),
                 ("setup.start", {"step": waiting, "version": version}, "WRONG_METHOD"),
                 ("setup.answer", {"step": board, "version": version, "values": {}}, "WRONG_METHOD"),
                 ("setup.skip", {"step": board, "version": version, "reason": "tired"}, "INVALID_VALUE"),
                 ("setup.skip", {"step": board, "version": version, "reason": "other"}, "INVALID_VALUE"),
                 ("setup.reset", {"version": version + 1}, "VERSION"))
        for method, params, reason in cases:
            code, message, data = self.probe.refused(method, params)
            self.assertEqual((code, data["event"], data["reason"]),
                             (RpcError.ERROR_INVALID_REQUEST, "SETUP_REFUSED", reason), f"{method} {params}")
            self.assertEqual((data["message"], data["severity"]), (message, "info"), reason)
        self.assertEqual(self.probe.refused("setup.start", {"step": board, "version": version - 1})[2]["version"],
                         version)
        code, _, data = self.probe.refused("setup.start", {"step": board})
        self.assertEqual((code, data), (RpcError.ERROR_INVALID_PARAMS, {"field": "version", "reason": "INVALID"}))
        self.assertEqual(self.probe.get()["version"], version)
        self.assertEqual(self.probe.setup_events(), [])
        self.assertFalse(os.path.exists(FILE))

    def test_4_the_state_goes_out_retained_and_tells_what_blocks(self):
        board = BOARD

        def sent(reasons):
            def look():
                with self.probe.lock:
                    state, retained = self.probe.states[-1] if self.probe.states else (None, False)
                found = state and [b["reason"] for s in state["steps"] if s["id"] == board for b in s["blocked"]]
                return state if retained and found == reasons else None
            return look
        self.assertIsNotNone(wait_until(sent([]), 5.0), "no setup/json with the board free")
        # A driver that falls silent blocks the checks of the board, without a change of a step.
        version = self.probe.get()["version"]
        self.probe.put(driver=False)
        state = wait_until(sent(["DRIVER_MISSING"]), 5.0)
        self.assertIsNotNone(state, "setup/json does not tell that the driver is gone")
        self.assertEqual(state["version"], version)
        refusal = self.probe.refused("setup.start", {"step": board, "version": version})[2]
        self.assertEqual(refusal["reason"], "DRIVER_MISSING")
        self.probe.put(driver=True)
        state = wait_until(sent([]), 5.0)
        self.assertIsNotNone(state)
        # Driving by hand is IDLE with a sub state: no check starts on a mower that moves.
        self.probe.put(sub="DRIVING")
        self.assertIsNotNone(wait_until(sent(["BUSY"]), 5.0), "setup/json does not tell that the mower drives")
        refusal = self.probe.refused("setup.start", {"step": board, "version": version})[2]
        self.assertEqual((refusal["reason"], refusal["state"]), ("BUSY", "DRIVING"))
        self.probe.put(sub="")
        self.assertIsNotNone(wait_until(sent([]), 5.0))
        # What went out last is what setup.get answers.
        time.sleep(1.0)
        with self.probe.lock:
            last = self.probe.states[-1][0]
        self.assertEqual(last, self.probe.get())

    def test_5_stopped_and_skipped_steps_stay_to_be_done(self):
        board = BOARD
        events = len(self.probe.setup_events())
        self.assertEqual(self.probe.do("setup.start", board)["state"], "running")
        self.assertEqual(self.probe.get()["current"], board)
        kind, state = self.probe.call("setup.abort")
        self.assertEqual(kind, "ok")
        entry = next(step for step in state["steps"] if step["id"] == board)
        self.assertEqual((entry["state"], entry["attempt"]), ("open", 1))
        self.assertEqual((entry["result"]["ok"], entry["result"]["reason"]), (False, "STOPPED"))
        told = self.probe.wait(lambda: self.probe.setup_events(events))
        self.assertEqual({k: told[0][k] for k in ("type", "step", "state", "attempt", "reason", "severity")},
                         {"type": "SETUP_STEP", "step": board, "state": "failed", "attempt": 1, "reason": "STOPPED",
                          "severity": "info"})
        self.assertEqual(saved()["steps"][board]["state"], "open")
        # Skipped with a reason; the step is done all the same when the user does it later.
        entry = self.probe.do("setup.skip", board, reason="other", note="the board is new")
        self.assertEqual(entry["state"], "skipped")
        self.assertEqual((entry["result"]["skip"], entry["result"]["note"], entry["result"]["reason"]),
                         ("other", "the board is new", None))
        saved_now = saved()
        self.assertEqual((saved_now["schema"], saved_now["platform"], saved_now["catalog"]),
                         (1, SETUP["platform"], LIST["catalog"]))
        self.assertEqual(saved_now["steps"][board]["state"], "skipped")
        told = self.probe.wait(lambda: len(self.probe.setup_events(events)) >= 2 and self.probe.setup_events(events))
        self.assertEqual((told[1]["state"], told[1]["skip"], told[1]["note"]), ("skipped", "other", "the board is new"))
        self.assertNotIn("reason", told[1])

    def test_6_every_step_of_the_list_by_hand(self):
        events = len(self.probe.setup_events())
        for step in LIST["steps"]:
            before = self.probe.step(step["id"])
            self.assertIn(before["state"], ("open", "skipped"), step["id"])
            getattr(self, "do_" + step["check"])(step["id"])
            entry = self.probe.step_when(step["id"], lambda e: e["state"] == "ok", 5.0)
            self.assertIsNotNone(entry, f"{step['id']} is not done: {self.probe.step(step['id'])}")
            self.assertEqual((entry["result"]["ok"], entry["result"]["reason"], entry["result"]["firmware"]),
                             (True, None, FIRMWARE), step["id"])
            self.assertEqual(entry["attempt"], before["attempt"] + 1, step["id"])
            self.assertEqual(entry["blocked"] if entry["kind"] != "measure" else [], [], step["id"])
        state = self.probe.get()
        self.assertTrue(state["complete"])
        self.assertEqual((state["current"], state["stage"]), (None, None))
        self.assertEqual({stage["state"] for stage in state["stages"]}, {"ok"})

        def complete():
            told = self.probe.setup_events(events)
            return told if told and told[-1].get("what") == "complete" else None
        told = self.probe.wait(complete, 5.0)
        self.assertIsNotNone(told, "no event that the program is complete")
        self.assertEqual([e["step"] for e in told if e["type"] == "SETUP_STEP"], list(STEPS))
        self.assertEqual({e["state"] for e in told if e["type"] == "SETUP_STEP"}, {"ok"})
        # A stage is told again when a step of it was done once more, here the board skipped before.
        stages = [e["stage"] for e in told if e.get("what") == "stage_done"]
        self.assertEqual(list(dict.fromkeys(stages)), [stage["id"] for stage in state["stages"]])
        self.assertEqual(sum(e.get("what") == "complete" for e in told), 1)
        record = saved()
        self.assertEqual(record["version"], state["version"])
        self.assertEqual({entry["state"] for entry in record["steps"].values()}, {"ok"})
        self.assertEqual(list(record["steps"]), list(STEPS))
        # Retained for an app that comes later.
        time.sleep(1.0)
        with self.probe.lock:
            last, retained = self.probe.states[-1]
        self.assertTrue(retained)
        self.assertEqual(last, self.probe.get())

    def test_7_reset_opens_every_step_again(self):
        events = len(self.probe.setup_events())
        kind, state = self.probe.call("setup.reset", {"version": self.probe.get()["version"]})
        self.assertEqual(kind, "ok")
        self.assertFalse(state["complete"])
        for step in state["steps"]:
            self.assertEqual((step["attempt"], step["result"]), (0, None), step["id"])
        # What the OS still sees is done again at once; what the user did by hand is open.
        state = self.probe.get()
        byhand = [step["id"] for step in LIST["steps"] if step["check"] in ("board", "input", "imu", "position")]
        for step in state["steps"]:
            if step["id"] in byhand:
                self.assertIn(step["state"], ("open", "locked"), step["id"])
        self.assertEqual(self.probe.wait(lambda: self.probe.setup_events(events))[0],
                         {**self.probe.setup_events(events)[0], "type": "SETUP", "what": "reset"})
        self.assertEqual(saved()["version"], state["version"])


@launch_testing.post_shutdown_test()
class TestExit(TestShutdown):
    pass
