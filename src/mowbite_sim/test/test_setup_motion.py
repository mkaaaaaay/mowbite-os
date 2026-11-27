"""The steps of the setup program that move the wheels, on a platform as its start runs it against its fake board
(the NX60i with mowbite.launch.py and fake_xcore, the Tron with sim.launch.py and fake_tron): the program drives on
setup/cmd_vel while setup/motion holds the logic in SETUP, mowbite_safety caps the commands, and the driver, the board
and the start do what they do on the mower. The steps of the inputs passed before (setup.json). Each step passes,
then the counter checks: a board that does not stop as it should fails its step (the xCore keeping its last duty
fails no_command, the Tron holding its setpoint without the host fails silence), the app that stops calling
setup.keep ends a step, an emergency of the app ends it, and driver_end is not offered without two restarts of the
driver left. The failed step closes the lock of autonomous driving (safety_lock.json): HIGH_LEVEL latches, its release
leaves the lock closed and a calibration is refused, until the step passes again; a file that cannot be read closes
it as well. The Tron has a safety measurement date in its simulation, so there it is the date the lock closes against.
At the end the step ground drives on the simulated ground: straight, turn, back until STOP, against the bumper; on the
NX60i, whose lock stays closed without a date, only once every step jacked up passed again. Then the step blade, with
the blade allowed as the restart of its first start leaves the file of the setup program: on and off, without renewal,
without host, then STOP, lifting and the bumper. Last the step summary up to its restart: it writes the date and the
stand time of silence into the file of the setup program and keeps the lock closed with a reset until it checked them
after the restart; the Tron asks mowbite-host for it, the launch of the NX60i ends (test_setup_summary.py goes on).

Run in a sourced workspace: MOWBITE_PLATFORM=<platform> launch_test /ws/src/mowbite_sim/test/test_setup_motion.py
"""
import json
import os
import random
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from geometry_msgs.msg import TwistStamped
from launch.actions import ExecuteProcess, TimerAction
from mowbite_msgs.msg import (HardwareStatus, MqttPublish, ReasonChange, RobotState, RpcError, RpcMethods, RpcRequest,
                              RpcResponse, Rtcm, SafetyStatus)
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, HISTORY, Keeper, SimProbe, fake_xcore, journal_dir, layers,  # noqa: E402,F401
                        loopback_group, sim_description, sim_files, stall_thread, station_frame, thread_owner,
                        wait_until, write_json)

PLATFORM = os.environ["MOWBITE_PLATFORM"]
WORK = tempfile.mkdtemp(prefix=f"mowbite_setup_motion_{PLATFORM}_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
SETUP = layers.setup(PLATFORM)
STEPS = {step["id"]: step for step in json.loads(SETUP["steps"])["steps"]}
CONFIRM = ["raised", "hand_at_stop"]
# What the driver shows after a silence: the xCore reports the host missing as well. The board that does not stop as it
# should, the step that fails with it and the part, the name of the driver in driver_exit.json.
SILENCE = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_HIGH_LEVEL
if PLATFORM == "nx60i":
    # TIMEOUT_HIGH_LEVEL, bit 6 of x-emergency-bits.
    SILENCE |= 1 << 6
COUNTER = {"nx60i": ("no_command", {"zero_duty_ignored": 1}, None),
           "tron": ("silence", {"host_timeout": 30}, "round_1")}[PLATFORM]
DRIVER = {"nx60i": "xcore_driver", "tron": "tron_mcu_node"}[PLATFORM]
LOCK = os.path.join(DATA, "safety_lock.json")

BLADE_CONFIRM = ["blade_removed", "hand_at_stop"]
BLADE_NODE = STEPS["blade"]["params"]["node"]

os.makedirs(DATA, exist_ok=True)
# allow_blade as the first start of the step blade writes it before the restart.
with open(layers.setup_file(DATA, PLATFORM), "w") as f:
    f.write("# Written by the setup program of MowBite OS, which writes it anew: values of your own go into the "
            "file of this\n# mower beside it, which is read after this one.\n"
            f"{BLADE_NODE}:\n  ros__parameters:\n    allow_blade: true\n")
# The steps of the inputs passed: the rule of the program lets no wheel turn before; imu and the dimensions as well,
# for summary.
DIMENSIONS = tuple(step for step, entry in STEPS.items() if entry["stage"] == "dimensions")
write_json(os.path.join(DATA, "setup.json"),
           {"schema": 1, "platform": SETUP["platform"],
            "steps": {step: {"state": "ok"} for step in ("stop", "lift", "bumper", "imu") + DIMENSIONS}})
write_json(FLAGS, {"gps_fix": "fix"})


def generate_test_description():
    if PLATFORM == "tron":
        description, _ = sim_description(WORK, random.randint(18000, 18999), journal_dir=JOURNAL)
        return description, {"system": None}
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--log", LOG, "--log-max-mb", "60")
    system = ExecuteProcess(
        cmd=["ros2", "launch", "mowbite_bringup", "mowbite.launch.py", "platform:=nx60i", "interface:=lo",
             f"bind_ip:={BIND_IP}", f"announcement_port:={GROUP_PORT}", f"data_dir:={DATA}",
             f"journal_dir:={JOURNAL}", f"datum_lat:={DATUM[0]}", f"datum_long:={DATUM[1]}",
             f"datum_height:={DATUM[2]}", f"mqtt_port:={random.randint(18000, 18999)}"],
        output="screen", sigterm_timeout="30", sigkill_timeout="30")
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[system, launch_testing.actions.ReadyToTest()]),
    ]), {"system": system}


def lock_causes():
    try:
        with open(LOCK) as f:
            return json.load(f)["causes"]
    except (OSError, ValueError, KeyError):
        return []


def set_flags(**flags):
    write_json(FLAGS, {"gps_fix": "fix", **flags})


class Probe(SimProbe):
    offer_timeout = 30.0
    call_timeout = 20.0
    emergency_reason = "setup motion test"

    def __init__(self):
        super().__init__(f"setup_motion_{PLATFORM}_probe")
        self.status = None
        self.robot = None
        self.wheels = None
        self.sub_states = set()
        # The requests of the OS to mowbite-host over the broker.
        self.host_requests = []
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._robot, 10)
        self.reason_changes = []
        self.node.create_subscription(ReasonChange, "bridge/reason_change",
                                      lambda msg: self.reason_changes.append((msg.before, msg.after)), 50)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._set("wheels"), 10)
        self.safety = None
        self.node.create_subscription(SafetyStatus, "safety/status", self._set("safety"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        # setup.keep as the app sends it while a step that moves runs.
        self.keeper = Keeper()
        if PLATFORM == "nx60i":
            # Corrections once a second, as a caster sends them: the xCore counts RTK fixed only with them (ground).
            self.rtcm_pub = self.node.create_publisher(Rtcm, "gnss/rtcm", 10)
            self.node.create_timer(1.0, lambda: self.rtcm_pub.publish(Rtcm(data=list(station_frame()))))
        self.start()

    def _robot(self, msg):
        with self.lock:
            self.robot = msg
            self.sub_states.add(msg.current_sub_state)

    def _publish(self, msg):
        if msg.topic == "rpc/request":
            with self.lock:
                self.host_requests.append(json.loads(msg.payload).get("method"))
        super()._publish(msg)

    def turning(self):
        return self.wheels is not None and abs(self.wheels.twist.linear.x) > 0.02

    def driving(self, sign):
        return self.wheels is not None and self.wheels.twist.linear.x * sign > 0.02

    def up(self, timeout=90.0):
        return self.wait(lambda: self.status is not None and self.status.telemetry_ok and self.robot is not None
                         and self.wheels is not None and self.methods.get("mowbite_setup"), timeout)

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


class TestSetupMotion(unittest.TestCase):
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

    def start(self, step, keep=True, confirm=CONFIRM):
        p = self.probe
        entry = wait_until(lambda: (lambda e: e if not e["blocked"] else None)(p.step(step)), 15)
        self.assertIsNotNone(entry, f"{step} stays blocked: {p.step(step)['blocked']}")
        p.keeper.step = step if keep else None
        answer = p.change("setup.start", step, confirm=confirm)
        self.assertIn("result", answer, answer)

    def ended(self, step, timeout):
        """The entry of the step once it no longer runs; the mower stands in IDLE again."""
        p = self.probe
        entry = wait_until(lambda: (lambda e: e if e["state"] != "running" else None)(p.step(step)), timeout)
        p.keeper.step = None
        self.assertIsNotNone(entry, f"{step} still runs: {p.step(step)}")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.robot.current_sub_state == "", 5),
                        p.robot)
        self.assertTrue(p.wait(lambda: not p.turning(), 5), "the wheels turn after the step")
        return entry

    def passed(self, step, timeout):
        entry = self.ended(step, timeout)
        self.assertEqual(entry["state"], "ok", entry["result"])
        self.assertEqual(self.probe.status.emergency_reason, 0)
        return entry

    def test_01_the_os_comes_up_and_offers_the_steps(self):
        p = self.probe
        self.assertTrue(p.up(), "the OS does not come up")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 60), p.robot)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)
        for step in ("wheels", "stop_moving", "no_command", "silence", "driver_end"):
            entry = p.step(step)
            self.assertEqual((entry["kind"], entry["state"]), ("drive", "open"), step)
            self.assertEqual(entry["motion"]["raised"], True, step)
        self.assertEqual(p.step("ground")["motion"]["raised"], False)
        refused = p.change("setup.start", "wheels", confirm=["raised"])
        self.assertEqual(refused["error"]["data"]["reason"], "CONFIRM_MISSING", refused)

    def wheels(self):
        p = self.probe
        self.start("wheels")
        asked = wait_until(lambda: (lambda e: e if e["state"] != "running" or any(
            i["name"] == "wheel" and i["ask"] for i in e["inputs"]) else None)(p.step("wheels")), 40)
        self.assertIsNotNone(asked, p.step("wheels"))
        self.assertEqual(asked["state"], "running", asked["result"])
        time.sleep(0.5)
        self.assertIn("result", p.change("setup.answer", "wheels", values={"wheel": "right"}))
        return self.passed("wheels", 10)

    def test_02_wheels(self):
        p = self.probe
        entry = self.wheels()
        self.assertIn("SETUP", p.sub_states)
        for value in entry["values"]:
            self.assertTrue(value["ok"], value)

    def test_03_stop_moving(self):
        p = self.probe
        self.start("stop_moving")
        for index, part in enumerate(STEPS["stop_moving"]["parts"]):
            ready = wait_until(lambda: (lambda e: e if e["state"] != "running" or (
                e["progress"]["n"] == index + 1 and p.turning() and p.status.emergency_reason == 0) else None)(
                    p.step("stop_moving")), 15)
            self.assertEqual((ready or {}).get("state"), "running", p.step("stop_moving"))
            time.sleep(0.5)
            set_flags(**{part["role"]: 1})
            self.assertTrue(p.wait(lambda: p.status.emergency_reason != 0, 2), part)
            time.sleep(0.5)
            set_flags()
        entry = self.passed("stop_moving", 20)
        for value in entry["values"]:
            self.assertLessEqual(value["value"], 1.0, value)

    def test_04_no_command(self):
        self.start("no_command")
        entry = self.passed("no_command", 20)
        self.assertLessEqual(entry["values"][0]["value"], 1.0, entry["values"])

    def test_05_silence(self):
        p = self.probe
        self.start("silence")
        # Meanwhile the board reports the host missing as well, which the list of the step names.
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == SILENCE, 15), p.status.emergency_reason)
        entry = self.passed("silence", 90)
        rounds = [value for value in entry["values"] if value["id"].startswith("round_")]
        self.assertEqual(len(rounds), STEPS["silence"]["params"]["rounds"])
        for value in rounds:
            self.assertGreater(value["value"], 0.0, value)
            self.assertLessEqual(value["value"], STEPS["silence"]["params"]["stop_max"], value)

    def test_06_driver_end(self):
        p = self.probe
        self.start("driver_end")
        entry = self.passed("driver_end", 60)
        self.assertGreater(entry["values"][0]["value"], 1.0)
        with open(os.path.join(DATA, "driver_exit.json")) as f:
            note = json.load(f)
        self.assertEqual((note["exit_code"], note["restart"]), (3, 1), note)
        # Two restarts left: still offered.
        self.assertEqual(p.step("driver_end")["blocked"], [])

    def test_07_a_board_that_does_not_stop_fails(self):
        p = self.probe
        step, flags, part = COUNTER
        set_flags(**flags)
        try:
            self.start(step)
            entry = wait_until(lambda: (lambda e: e if e["state"] != "running" else None)(p.step(step)), 30)
        finally:
            p.keeper.step = None
            set_flags()
        self.assertIsNotNone(entry, p.step(step))
        self.assertEqual((entry["result"]["reason"], entry["result"]["part"]), ("LIMIT", part), entry["result"])
        # The failure closed the lock: the file, the state of the program, S2, and HIGH_LEVEL latched.
        self.assertTrue(os.path.exists(LOCK))
        safety = p.get()["safety"]
        self.assertEqual((safety["open"], safety["why"]), (False, "CHECK_FAILED"), safety)
        self.assertEqual((safety["closed"][0]["step"], safety["closed"][0]["reason"]), (step, "LIMIT"), safety)
        self.assertTrue(p.wait(lambda: p.safety is not None and p.safety.lock_closed, 2), p.safety)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason & HardwareStatus.REASON_HIGH_LEVEL, 3),
                        p.status.emergency_reason)
        # Such a board turns on until it takes a zero again; the step releases nothing it did not ask for, also not the
        # emergency at the end of a silence that is still on.
        self.assertTrue(p.wait(lambda: not p.turning(), 10), "the wheels turn on")
        # A driver that is still silent refuses the release (SILENT) until the silence is over.
        released = wait_until(lambda: "result" in (p.rpc("emergency.reset") or {}) or None, 15, poll=1.0)
        self.assertTrue(released, "emergency.reset is refused")
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)
        # The release leaves the lock closed: nothing drives by itself.
        time.sleep(1.0)
        self.assertTrue(p.safety.lock_closed)
        self.assertFalse(p.get()["safety"]["open"])
        refused = p.rpc("calibration.start")
        data = ((refused or {}).get("error") or {}).get("data") or {}
        self.assertEqual(data.get("reason"), "NO_SAFETY_MEASUREMENT", refused)

    def test_08_without_keep_the_step_stops(self):
        p = self.probe
        self.start("wheels", keep=False)
        entry = self.ended("wheels", 10)
        self.assertEqual(entry["result"]["reason"], "RELEASED", entry["result"])
        self.assertEqual(p.status.emergency_reason, 0)

    def test_08a_a_node_that_stood_still_reads_its_queue_first(self):
        """G34: a node held longer than the timeout of an input finds what came meanwhile still in its queue. The logic
        held 3 s tells no timeout of hw/status; a step goes on when setup.keep came while the setup program stood
        still and left the last one it read older than its timeout (shorter than the 1 s the logic waits for a sign
        of the setup program)."""
        p = self.probe
        since = len(p.reason_changes)
        stall_thread(thread_owner("logic"), "logic", 3.0)
        time.sleep(1.0)
        self.assertEqual(p.reason_changes[since:], [])
        self.assertFalse(p.robot.emergency, p.robot)
        self.start("stop_moving")
        ready = wait_until(lambda: (lambda e: e if e["state"] != "running" or (
            e["progress"]["n"] == 1 and p.turning()) else None)(p.step("stop_moving")), 15)
        self.assertEqual((ready or {}).get("state"), "running", p.step("stop_moving"))
        p.keeper.step = None
        time.sleep(0.5)
        p._send("setup.keep", {"step": "stop_moving"}, None, None)
        time.sleep(0.5)
        p.keeper.step = "stop_moving"
        stall_thread(thread_owner("setup"), "setup", 0.6)
        time.sleep(0.5)
        self.assertEqual(p.step("stop_moving")["state"], "running", p.step("stop_moving"))
        p.keeper.step = None
        self.assertEqual(self.ended("stop_moving", 10)["result"]["reason"], "RELEASED")

    def test_09_an_emergency_of_the_app_ends_the_step(self):
        p = self.probe
        self.start("stop_moving")
        self.assertTrue(p.wait(lambda: p.turning(), 10), "the wheels do not turn")
        # As the emergency button of the app reaches the driver.
        self.assertTrue(p.set_emergency(True).success)
        entry = self.ended("stop_moving", 10)
        self.assertEqual(entry["result"]["reason"], "EMERGENCY", entry["result"])
        time.sleep(3.0)
        self.assertNotEqual(p.status.emergency_reason, 0)
        self.assertIn("result", p.rpc("emergency.reset"))
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)

    def test_10_driver_end_needs_two_restarts_left(self):
        p = self.probe
        write_json(os.path.join(DATA, "driver_exit.json"),
                   {"name": DRIVER, "exit_code": 3, "restart": 2, "max_restarts": 3, "t": time.time(),
                    "mono": time.monotonic(), "stalled": None, "down": False, "stable_s": 600.0})
        entry = wait_until(lambda: (lambda e: e if e["blocked"] else None)(p.step("driver_end")), 5)
        self.assertIsNotNone(entry)
        self.assertEqual([b["reason"] for b in entry["blocked"]], ["RESTARTS_LOW"], entry["blocked"])
        refused = p.change("setup.start", "driver_end", confirm=CONFIRM)
        self.assertEqual(refused["error"]["data"]["reason"], "RESTARTS_LOW", refused)

    def test_11_the_step_passed_again_opens_the_lock(self):
        p = self.probe
        step = COUNTER[0]
        self.start(step)
        self.passed(step, 90)
        self.assertTrue(p.wait(lambda: not os.path.exists(LOCK), 5), "the lock file stays")
        safety = p.get()["safety"]
        self.assertEqual(safety["closed"], [], safety)
        # Open only where the platform has a date: the simulated Tron.
        self.assertEqual(safety["open"], PLATFORM == "tron", safety)
        self.assertTrue(p.wait(lambda: not p.safety.lock_closed, 2), p.safety)
        self.assertEqual(p.status.emergency_reason, 0)

    def test_12_a_lock_file_that_cannot_be_read_closes(self):
        p = self.probe
        with open(LOCK, "w") as f:
            f.write("{")
        try:
            self.assertTrue(p.wait(lambda: p.safety.lock_closed, 2), p.safety)
            self.assertFalse(p.safety.autonomous_ok)
        finally:
            os.remove(LOCK)
        self.assertTrue(p.wait(lambda: not p.safety.lock_closed, 2), p.safety)

    def press_while(self, part, sign, flag):
        """Waits until part n of ground drives in the way of sign, then triggers flag for 0.5 s."""
        p = self.probe
        ready = wait_until(lambda: (lambda e: e if e["state"] != "running" or (
            e["progress"]["n"] == part and p.driving(sign) and p.status.emergency_reason == 0) else None)(
                p.step("ground")), 40)
        self.assertEqual((ready or {}).get("state"), "running", p.step("ground"))
        time.sleep(0.5)
        set_flags(**{flag: 1})
        self.assertTrue(p.wait(lambda: p.status.emergency_reason != 0, 2), flag)
        time.sleep(0.5)
        set_flags()

    def test_13_ground(self):
        p = self.probe
        if PLATFORM == "nx60i":
            # The lock is closed: the steps jacked up that ended failed above come first.
            blocked = p.step("ground")["blocked"]
            self.assertEqual([b["reason"] for b in blocked], ["SAFETY_AIR"], blocked)
            self.assertEqual(blocked[0]["steps"], ["wheels", "stop_moving"], blocked)
            self.wheels()
            self.test_03_stop_moving()
        refused = p.change("setup.start", "ground", confirm=CONFIRM)
        self.assertEqual(refused["error"]["data"]["reason"], "CONFIRM_MISSING", refused)
        self.start("ground", confirm=["area_clear"])
        self.press_while(3, -1, "stop")
        self.press_while(4, -1 if STEPS["ground"]["params"]["bumper_direction"] == "backward" else 1, "bumper")
        entry = self.passed("ground", 30)
        for value in entry["values"]:
            self.assertTrue(value["ok"], value)
        self.assertFalse(os.path.exists(LOCK))

    def test_14_blade(self):
        p = self.probe
        entry = p.step("blade")
        self.assertEqual((entry["kind"], entry["restart"], entry["motion"]["blade"]), ("blade", False, True), entry)
        self.start("blade", confirm=BLADE_CONFIRM)
        # on_off, lease and silent by themselves, then the inputs by hand while the blade turns.
        for part, flag in ((4, "stop"), (5, "lift"), (6, "bumper")):
            ready = wait_until(lambda: (lambda e: e if e["state"] != "running" or (
                e["progress"]["n"] == part and e["ends_at"] is not None) else None)(p.step("blade")), 40)
            self.assertEqual((ready or {}).get("state"), "running", p.step("blade"))
            self.assertGreaterEqual(abs(p.status.blade_motor.rpm), STEPS["blade"]["params"]["rpm"], flag)
            time.sleep(0.5)
            set_flags(**{flag: 1})
            # The NX60i latches a lift only after its delay.
            self.assertTrue(p.wait(lambda: p.status.emergency_reason != 0, 4), flag)
            time.sleep(0.5)
            set_flags()
        entry = self.passed("blade", 40)
        for value in entry["values"]:
            self.assertTrue(value["ok"], value)
        self.assertEqual(p.status.blade_motor.rpm, 0)

    def test_15_summary(self, proc_info, system):
        p = self.probe
        node = STEPS["summary"]["params"]["node"]
        if PLATFORM == "tron":
            # test_08 and test_09 left wheels and stop_moving failed: summary names them.
            blocked = p.step("summary")["blocked"]
            self.assertEqual([(b["reason"], b.get("steps")) for b in blocked],
                             [("SAFETY_STEPS", ["wheels", "stop_moving"])], blocked)
            self.wheels()
            self.test_03_stop_moving()
        entry = p.step("summary")
        self.assertEqual((entry["kind"], entry["restart"], entry["state"]), ("confirm", True, "open"), entry)
        refused = p.change("setup.start", "summary", confirm=[])
        self.assertEqual(refused["error"]["data"]["reason"], "CONFIRM_MISSING", refused)
        longest = max(value["value"] for value in p.step("silence")["values"] if value["id"].startswith("round_"))
        self.start("summary", keep=False, confirm=["seen_myself"])
        layer = layers.setup_file(DATA, PLATFORM)
        written = wait_until(lambda: (lambda v: v if "safety_measurement_date" in v else None)(
            (layers.parameters([layer], node) if os.path.exists(layer) else {})), 10)
        self.assertIsNotNone(written, "the date is not written")
        self.assertEqual(written["safety_measurement_date"], time.strftime("%Y-%m-%d", time.gmtime()))
        self.assertAlmostEqual(written["failsafe_without_host"], longest, places=3)
        self.assertTrue(written.get("allow_blade"), written)
        # Until the check after the restart a reset keeps the lock closed.
        self.assertTrue(wait_until(lambda: any(cause.get("reset") for cause in lock_causes()), 5), lock_causes())
        if PLATFORM == "tron":
            self.assertTrue(p.wait(lambda: "host.restart_os" in p.host_requests, 10), p.host_requests)
            state = p.get()
            self.assertTrue(state["restart_pending"], state)
            self.assertEqual(p.step("summary")["state"], "running")
            self.assertFalse(state["safety"]["open"], state["safety"])
            self.assertTrue(p.wait(lambda: p.safety.lock_closed, 2), p.safety)
            return
        # The NX60i: the setup program leaves its note, the launch ends in order and nothing starts it here again.
        proc_info.assertWaitForShutdown(process=system, timeout=60)
        self.assertTrue(os.path.exists(os.path.join(DATA, "setup_restart.json")))
        self.assertIn(proc_info[system].returncode, EXIT_CODES)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        # The driver that hw/end ended leaves with exit code 3.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES + [3])
