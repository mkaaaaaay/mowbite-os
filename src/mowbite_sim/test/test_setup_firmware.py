"""The measurement of firmware tron_mcu does not list (stage firmware) on the simulated Tron: fake_tron reports boards
0.6.38/0.6.34 and behaves as the Tron (latching stop and lift, its rates, the setpoint it keeps 3.6 s without host), the
driver keeps motion and blade latched with FIRMWARE_INCOMPATIBLE until setup/firmware_measure opens its window. Every
step of the stage runs as the app and the owner do it, with the inputs as flags of fake_tron. Without a variant every
step passes, the step firmware writes the entry into the file of the setup program and asks mowbite-host for the
restart. With MOWBITE_FW_VARIANT a board that fails: the run fails at the step that measures it, the steps before
passed.

Run in a sourced workspace: MOWBITE_FW_VARIANT=<variant> launch_test /ws/src/mowbite_sim/test/test_setup_firmware.py
"""
import json
import os
import random
import shutil
import sys
import tempfile
import time
import unittest

import launch_testing.asserts
import rclpy
from geometry_msgs.msg import TwistStamped
from mowbite_msgs.msg import HardwareStatus, MqttPublish, RobotState, RpcError, RpcMethods, RpcRequest, RpcResponse

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, HISTORY, Keeper, SimProbe, journal_dir, layers, sim_description,  # noqa: E402
                        sim_files, wait_until, write_json)

VARIANT = os.environ.get("MOWBITE_FW_VARIANT", "")
WORK = tempfile.mkdtemp(prefix=f"mowbite_setup_firmware_{VARIANT or 'tron'}_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
SETUP = layers.setup("tron")
STEPS = {step["id"]: step for step in json.loads(SETUP["steps"])["steps"]}
ORDER = [step["id"] for step in json.loads(SETUP["steps"])["steps"] if step.get("firmware")]
# The Tron as measured (T9), on firmware the driver does not list.
BOARD = {"gps_fix": "fix", "real_rates": 1, "stop_latches": 1, "lift_latches": 1, "setpoint_hold": 3.6,
         "firmware": ["0.6.38", "0.6.34"]}
# A board that fails: what it changes, and the step and reason it fails with.
VARIANTS = {
    "stop_delay": ({"stop_delay": 0.5}, "fw_stop_moving", "LIMIT"),
    "no_stop_latches": ({"stop_latches": 0}, "fw_latch", "LIMIT"),
    "clear_releases_held": ({"clear_releases_held": 1}, "fw_latch", "LIMIT"),
    "setpoint_hold": ({"setpoint_hold": 6.0}, "fw_setpoint", "LIMIT"),
    "no_real_rates": ({"real_rates": 0}, "fw_link", "LIMIT"),
    "swap_bumpers": ({"swap_bumpers": 1}, "fw_bumper", "WRONG_INPUT"),
    "cutter_ignores_stop": ({"cutter_ignores_stop": 1}, "fw_blade", "LIMIT"),
}
CHANGE, FAILS, REASON = VARIANTS[VARIANT] if VARIANT else ({}, None, None)
FLAGGED = {**BOARD, **CHANGE}
NODE = STEPS["fw_release"]["params"]["node"]
CID = os.path.join(WORK, "cid")
ENTRY_PREFIX = "0.6.38/0.6.34/"

os.makedirs(DATA, exist_ok=True)
with open(CID, "w") as f:
    f.write("150100474a52354b42071a1d3e8c4e00\n")
# The id of the device from a file of the simulation (on the Tron the CID of the eMMC); the blade allowed before
# the run.
with open(layers.device_file(DATA, "tron"), "w") as f:
    f.write(f"{NODE}:\n  ros__parameters:\n    device_id_file: {CID}\n")
with open(layers.setup_file(DATA, "tron"), "w") as f:
    f.write(f"{STEPS['fw_blade']['params']['node']}:\n  ros__parameters:\n    allow_blade: true\n")
write_json(FLAGS, FLAGGED)


def generate_test_description():
    description, _ = sim_description(WORK, random.randint(18000, 18999), journal_dir=journal_dir(WORK))
    return description, {}


def set_flags(**flags):
    write_json(FLAGS, {**FLAGGED, **flags})


class Probe(SimProbe):
    offer_timeout = 30.0
    call_timeout = 20.0
    emergency_reason = "setup firmware test"

    def __init__(self):
        super().__init__(f"setup_firmware_{VARIANT or 'tron'}_probe")
        self.status = None
        self.robot = None
        self.wheels = None
        self.host_requests = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._set("wheels"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.keeper = Keeper()
        self.start()

    def _publish(self, msg):
        if msg.topic == "rpc/request":
            with self.lock:
                self.host_requests.append(json.loads(msg.payload).get("method"))
        super()._publish(msg)

    def turning(self):
        return self.wheels is not None and abs(self.wheels.twist.linear.x) > 0.02

    def get(self):
        for _ in range(4):
            answer = self.rpc("setup.get", timeout=4.0)
            if answer:
                break
        assert answer and "result" in answer, f"setup.get: {answer}"
        return answer["result"]

    def step(self, step):
        return next(entry for entry in self.get()["steps"] if entry["id"] == step)

    def change(self, method, step, **params):
        for _ in range(3):
            answer = self.rpc(method, {"step": step, "version": self.get()["version"], **params})
            data = ((answer or {}).get("error") or {}).get("data") or {}
            if data.get("reason") != "VERSION":
                break
        return answer


class TestSetupFirmware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.keeper.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def start(self, step):
        p = self.probe
        entry = wait_until(lambda: (lambda e: e if not e["blocked"] else None)(p.step(step)), 20)
        self.assertIsNotNone(entry, f"{step} stays blocked: {p.step(step)['blocked']}")
        p.keeper.step = step if STEPS[step].get("confirm") else None
        answer = p.change("setup.start", step, confirm=STEPS[step].get("confirm", []))
        self.assertIn("result", answer, answer)

    def ready(self, step, n, turning=False, timeout=30):
        """The step waits at n (from 1) with nothing in its way: the entry, or the entry it ended with."""
        p = self.probe
        return wait_until(lambda: (lambda e: e if e["state"] != "running" or (
            (e.get("progress") or {}).get("n") == n and not e["blocked"] and (not turning or p.turning())) else None)(
                p.step(step)), timeout)

    def press(self, flag, hold=0.3):
        set_flags(**{flag: 1})
        time.sleep(hold)
        set_flags()

    def ended(self, step, timeout):
        p = self.probe
        entry = wait_until(lambda: (lambda e: e if e["state"] != "running" else None)(p.step(step)), timeout)
        p.keeper.step = None
        self.assertIsNotNone(entry, f"{step} still runs: {p.step(step)}")
        return entry

    def inputs(self, step, flags):
        for n, flag in enumerate(flags, 1):
            entry = self.ready(step, n)
            self.assertIsNotNone(entry, f"{step} does not wait at {n}: {self.probe.step(step)}")
            if entry["state"] != "running":
                return
            time.sleep(0.3)
            self.press(flag)

    def latch(self, step):
        parts = STEPS[step]["parts"]
        rounds = STEPS[step]["params"]["rounds"]
        for r in range(rounds):
            for i, part in enumerate(parts):
                n = r * len(parts) + i + 1
                entry = self.ready(step, n)
                self.assertIsNotNone(entry, f"{step} does not wait at {n}: {self.probe.step(step)}")
                if entry["state"] != "running":
                    return
                time.sleep(0.3)
                flag = part["role"]
                if part.get("mode") == "press":
                    set_flags(**{flag: 1})
                elif part.get("mode") == "release":
                    set_flags()
                else:
                    self.press(flag)

    def wheels(self, step):
        p = self.probe
        for _ in range(STEPS[step]["params"]["rounds"]):
            asked = wait_until(lambda: (lambda e: e if e["state"] != "running" or any(
                i["name"] == "wheel" and i["ask"] for i in e["inputs"]) else None)(p.step(step)), 40)
            if asked is None or asked["state"] != "running":
                return
            time.sleep(0.5)
            self.assertIn("result", p.change("setup.answer", step, values={"wheel": "right"}))

    def stop_moving(self, step):
        parts = STEPS[step]["parts"]
        for r in range(STEPS[step]["params"]["rounds"]):
            for i, part in enumerate(parts):
                entry = self.ready(step, r * len(parts) + i + 1, turning=True, timeout=20)
                if entry["state"] != "running":
                    return
                time.sleep(0.5)
                self.press(part["role"], 0.5)

    def blade(self, step):
        p = self.probe
        for r in range(STEPS[step]["params"]["rounds"]):
            for part, flag in ((4, "stop"), (5, "lift"), (6, "bumper")):
                entry = wait_until(lambda: (lambda e: e if e["state"] != "running" or (
                    e["progress"]["n"] == r * 6 + part and e["ends_at"] is not None) else None)(p.step(step)), 40)
                if entry is None or entry["state"] != "running":
                    return
                time.sleep(0.5)
                self.press(flag, 0.5)

    def run_step(self, step):
        """Starts the step, does by hand what it asks for and returns the entry it ended with; once more after the
        driver ended the measurement (NOT_OPENED, a stall of its link under load), as the owner would."""
        entry = self.run_once(step)
        if entry["state"] != "ok" and (entry.get("result") or {}).get("reason") == "NOT_OPENED":
            entry = self.run_once(step)
        return entry

    def run_once(self, step):
        self.start(step)
        check = STEPS[step]["check"]
        if check == "input":
            parts = STEPS[step].get("parts") or [{"role": STEPS[step]["params"]["role"]}]
            roles = [part.get("role", STEPS[step]["params"]["role"]) for part in parts]
            self.inputs(step, roles * STEPS[step]["params"].get("rounds", 1))
        elif check == "latch":
            self.latch(step)
        elif check == "wheels":
            self.wheels(step)
        elif check == "stop_moving":
            self.stop_moving(step)
        elif check == "blade":
            self.blade(step)
        set_flags()
        return self.ended(step, 240)

    def test_01_the_stage_is_offered(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.status is not None and p.status.telemetry_ok and p.robot is not None
                               and p.methods.get("mowbite_setup"), 90), "the OS does not come up")
        firmware = wait_until(lambda: (lambda f: f if f["offered"] and f["device"] else None)(p.get()["firmware"]), 30)
        self.assertIsNotNone(firmware, p.get().get("firmware"))
        self.assertEqual(p.get()["safety"]["firmware_source"], "none")
        self.assertTrue(p.status.emergency_reason & HardwareStatus.REASON_FIRMWARE_INCOMPATIBLE,
                        p.status.emergency_reason)
        self.assertEqual(p.step("fw_link")["blocked"], [])

    def test_02_the_steps(self):
        p = self.probe
        for step in ORDER:
            if step == "fw_release":
                break
            entry = self.run_step(step)
            if step == FAILS:
                self.assertEqual((entry["state"], entry["result"]["reason"]), ("failed", REASON), entry["result"])
                runs = p.rpc("setup.firmware.export")["result"]["runs"]
                self.assertEqual(runs[-1]["result"], "failed", runs[-1])
                # The driver latched the window again; nothing drives.
                self.assertTrue(p.wait(lambda: not p.turning(), 10))
                return
            self.assertEqual(entry["state"], "ok", f"{step}: {entry['result']}")
        self.assertIsNone(FAILS, f"{FAILS} passed")

    def test_03_the_entry_is_written(self):
        if FAILS:
            self.skipTest("the run failed")
        p = self.probe
        self.start("fw_release")
        layer = layers.setup_file(DATA, "tron")
        written = wait_until(lambda: (lambda v: v if v.get("measured_firmware") else None)(
            layers.parameters([layer], NODE)), 15)
        self.assertIsNotNone(written, "measured_firmware is not written")
        self.assertEqual(len(written["measured_firmware"]), 1)
        self.assertTrue(written["measured_firmware"][0].startswith(ENTRY_PREFIX), written)
        self.assertTrue(written.get("allow_blade"), written)
        self.assertTrue(p.wait(lambda: "host.restart_os" in p.host_requests, 15), p.host_requests)
        self.assertEqual(p.rpc("setup.firmware.export")["result"]["runs"][-1]["result"], "passed")


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        # The driver that hw/end ended leaves with exit code 3.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES + [3])
