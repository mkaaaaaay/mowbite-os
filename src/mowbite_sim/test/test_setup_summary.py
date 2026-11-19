"""The step summary of the NX60i over its restarts, with MowBite OS as mowbite.launch.py starts it against fake_xcore:
the launch runs as a process of its own that is started again whenever it ends, as compose does on the mower. Every
safety step passed before (setup.json). First the file of this mower kept by hand sets an empty date: the program
writes its date, restarts the OS, the driver shows the empty one and the step ends NOT_APPLIED, the lock stays closed.
Without that file the next summary restarts the OS once more: xcore_driver shows the date and the stand time of
silence, the step passes, the lock opens by the program and a calibration is no longer refused for want of a safety
measurement. setup.reset then takes the date out of the file of the program and closes the lock with a reset.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_setup_summary.py
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
import rclpy
from launch.actions import ExecuteProcess, TimerAction
from mowbite_msgs.msg import (HardwareCapabilities, HardwareStatus, MqttPublish, RobotState, RpcError, RpcMethods,
                              RpcRequest, RpcResponse, SafetyStatus)
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HISTORY, LATCHED, SimProbe, TestShutdown, fake_xcore, journal_dir, layers,  # noqa: E402,F401
                        loopback_group, sim_files, wait_until, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_setup_summary_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
STARTS = os.path.join(WORK, "starts")
RESPAWN = 2.0
STEP = "summary"
SETUP = layers.setup("nx60i")
LIST = json.loads(SETUP["steps"])
STEPS = {step["id"]: step for step in LIST["steps"]}
DRIVER = STEPS[STEP]["params"]["node"]
LAYER = layers.setup_file(DATA, "nx60i")
DEVICE = layers.device_file(DATA, "nx60i")
LOCK = os.path.join(DATA, "safety_lock.json")
SAFETY = ("stop", "lift", "bumper", "imu", "wheels", "stop_moving", "no_command", "silence", "driver_end", "ground",
          "blade")
ROUNDS = [0.42, 0.37]
CRASH_NOTES = ("container_exit.json", "driver_exit.json", "watch_exit.json", "restart-hold.json")

SCRIPT = os.path.join(WORK, "os.sh")
with open(SCRIPT, "w") as f:
    f.write(f"""echo start >> {STARTS}
exec ros2 launch mowbite_bringup mowbite.launch.py platform:=nx60i interface:=lo bind_ip:={BIND_IP} \\
  announcement_port:={GROUP_PORT} data_dir:={DATA} journal_dir:={JOURNAL} datum_lat:={DATUM[0]} \\
  datum_long:={DATUM[1]} datum_height:={DATUM[2]} mqtt_port:={random.randint(18000, 18999)}
""")
os.makedirs(DATA, exist_ok=True)
# The dimensions come before the summary.
DIMENSIONS = tuple(step["id"] for step in LIST["steps"] if step["stage"] == "dimensions")
records = {step: {"state": "ok"} for step in SAFETY + DIMENSIONS}
records["silence"]["values"] = [{"id": f"round_{n + 1}", "value": value} for n, value in enumerate(ROUNDS)]
write_json(os.path.join(DATA, "setup.json"), {"schema": 1, "platform": SETUP["platform"], "steps": records})
# The file kept by hand, read after the one of the program: an empty date.
with open(DEVICE, "w") as f:
    f.write(f"{DRIVER}:\n  ros__parameters:\n    safety_measurement_date: \"\"\n")
write_json(FLAGS, {"gps_fix": "fix"})


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--log", LOG, "--log-max-mb", "60")
    system = ExecuteProcess(cmd=["sh", SCRIPT], output="screen", respawn=True, respawn_delay=RESPAWN,
                            sigterm_timeout="30", sigkill_timeout="30")
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[system, launch_testing.actions.ReadyToTest()]),
    ]), {}


def starts():
    try:
        with open(STARTS) as f:
            return len(f.readlines())
    except OSError:
        return 0


def program_values():
    try:
        return layers.parameters([LAYER], DRIVER) if os.path.exists(LAYER) else {}
    except (OSError, ValueError):
        return {}


def lock_causes():
    try:
        with open(LOCK) as f:
            return json.load(f)["causes"]
    except (OSError, ValueError, KeyError):
        return []


class Probe(SimProbe):
    offer_timeout = 30.0
    call_timeout = 20.0
    emergency_reason = "summary test"

    def __init__(self):
        super().__init__("setup_summary_probe")
        self.status = None
        self.robot = None
        self.capabilities = None
        self.safety = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("capabilities"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(SafetyStatus, "safety/status", self._set("safety"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def forget(self):
        """Drops what the OS before a restart told."""
        with self.lock:
            self.status = self.robot = self.capabilities = self.safety = None
            self.methods.clear()

    def up(self, timeout=90.0):
        return self.wait(lambda: self.status is not None and self.status.telemetry_ok and self.robot is not None
                         and self.capabilities is not None and self.methods.get("mowbite_setup"), timeout)

    def get(self):
        for _ in range(4):
            answer = self.rpc("setup.get", timeout=4.0)
            if answer:
                break
        assert answer and "result" in answer, f"setup.get: {answer}"
        return answer["result"]

    def step(self, step=STEP):
        return next(entry for entry in self.get()["steps"] if entry["id"] == step)

    def step_when(self, pred, timeout, step=STEP):
        def look():
            entry = self.step(step)
            return entry if pred(entry) else None
        return wait_until(look, timeout, poll=1.0)

    def change(self, method, step=STEP, **params):
        for _ in range(3):
            answer = self.rpc(method, {"step": step, "version": self.get()["version"], **params})
            data = ((answer or {}).get("error") or {}).get("data") or {}
            if data.get("reason") != "VERSION":
                break
        return answer


class TestSetupSummary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def restarted(self, number, timeout=60.0):
        """Waits for start `number` of the launch and for its OS; the end before it was in order."""
        p = self.probe
        self.assertTrue(p.wait(lambda: starts() >= number, timeout), "the launch did not start again")
        p.forget()
        self.assertTrue(p.up(), f"start {number} does not come up")
        self.assertEqual(starts(), number)
        self.assertEqual([name for name in CRASH_NOTES if os.path.exists(os.path.join(DATA, name))], [])
        self.assertFalse(os.path.exists(os.path.join(DATA, "setup_restart.json")))

    def start_summary(self):
        p = self.probe
        entry = p.step_when(lambda e: not e["blocked"], 30)
        self.assertIsNotNone(entry, p.step())
        answer = p.change("setup.start", confirm=["seen_myself"])
        self.assertIn("result", answer, answer)

    def test_1_the_os_comes_up_closed_and_summary_may_start(self):
        p = self.probe
        self.assertTrue(p.up(), "the OS does not come up")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 60), p.robot)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)
        self.assertEqual(p.capabilities.safety_measurement_date, "")
        safety = p.get()["safety"]
        self.assertEqual((safety["open"], safety["why"], safety["source"]), (False, "NO_SAFETY_MEASUREMENT", None))
        entry = p.step_when(lambda e: not e["blocked"], 30)
        self.assertIsNotNone(entry, f"summary stays blocked: {p.step()['blocked']}")
        self.assertEqual((entry["kind"], entry["restart"], entry["state"]), ("confirm", True, "open"), entry)

    def test_2_an_empty_date_by_hand_is_not_applied(self):
        p = self.probe
        self.start_summary()
        self.restarted(2)
        entry = p.step_when(lambda e: e["state"] != "running", 30)
        self.assertIsNotNone(entry, p.step())
        self.assertEqual(entry["state"], "failed", entry)
        self.assertEqual(entry["result"]["reason"], "NOT_APPLIED", entry["result"])
        # The event names the key of the file kept by hand.
        told = wait_until(lambda: [e for e in list(p.events) if e["type"] == "SETUP_STEP" and e.get("step") == STEP
                                   and e.get("state") == "failed"], 5)
        self.assertTrue(told, p.events)
        self.assertEqual((told[-1]["reason"], told[-1].get("key")),
                         ("NOT_APPLIED", f"{DRIVER}.safety_measurement_date"), told)
        # The file of the program has its values back, the lock stays closed and nothing latched.
        self.assertTrue(wait_until(lambda: not ({"safety_measurement_date", "failsafe_without_host"}
                                                & set(program_values())), 5), program_values())
        self.assertEqual(p.capabilities.safety_measurement_date, "")
        self.assertFalse(p.get()["safety"]["open"])
        self.assertTrue(p.wait(lambda: p.safety is not None and p.safety.lock_closed, 5), p.safety)
        # The board reports SERVICE_NOT_READY until the services of the new driver are up; nothing latches.
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 15), p.status.emergency_reason)

    def test_3_without_the_file_by_hand_summary_opens_the_lock(self):
        p = self.probe
        os.remove(DEVICE)
        events = len(p.events)
        self.start_summary()
        self.restarted(3)
        entry = p.step_when(lambda e: e["state"] != "running", 30)
        self.assertIsNotNone(entry, p.step())
        self.assertEqual(entry["state"], "ok", entry["result"])
        today = time.strftime("%Y-%m-%d", time.gmtime())
        values = {value["id"]: value["value"] for value in entry["values"]}
        self.assertEqual(values["date"], today)
        self.assertAlmostEqual(values["failsafe"], max(ROUNDS), places=6)
        # The driver of this start took both out of the file of the program.
        self.assertEqual(p.capabilities.safety_measurement_date, today)
        self.assertAlmostEqual(p.capabilities.failsafe_without_host_s, max(ROUNDS), places=4)
        written = program_values()
        self.assertEqual(written["safety_measurement_date"], today, written)
        self.assertAlmostEqual(written["failsafe_without_host"], max(ROUNDS), places=6)
        safety = p.get()["safety"]
        self.assertEqual((safety["open"], safety["why"], safety["source"], safety["date"]),
                         (True, None, "program", today), safety)
        self.assertEqual(lock_causes(), [])
        self.assertTrue(p.wait(lambda: p.safety is not None and not p.safety.lock_closed, 5), p.safety)
        self.assertTrue(p.wait(lambda: any(e["type"] == "SETUP" and e.get("what") == "safety_opened"
                                           for e in p.events), 5), p.events[events:])
        # A calibration is no longer refused for want of a safety measurement; should it start, the app stops it.
        answer = p.rpc("calibration.start") or {}
        data = (answer.get("error") or {}).get("data") or {}
        self.assertNotEqual(data.get("reason"), "NO_SAFETY_MEASUREMENT", answer)
        if "result" in answer:
            self.assertTrue(p.set_emergency(True).success)
            self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 30), p.robot)
            self.assertIn("result", p.rpc("emergency.reset"))
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.status.emergency_reason == 0, 30),
                        (p.robot, p.status.emergency_reason))

    def test_4_reset_takes_the_date_back_and_closes(self):
        p = self.probe
        answer = p.rpc("setup.reset", {"version": p.get()["version"]})
        self.assertIn("result", answer, answer)
        state = p.get()
        self.assertEqual((state["safety"]["open"], state["safety"]["why"], state["safety"]["source"]),
                         (False, "RESET", "program"), state["safety"])
        self.assertEqual(p.step()["state"], "open")
        self.assertTrue(wait_until(lambda: not ({"safety_measurement_date", "failsafe_without_host", "allow_blade"}
                                                & set(program_values())), 5), program_values())
        self.assertTrue(wait_until(lambda: any(cause.get("reset") for cause in lock_causes()), 5), lock_causes())
        self.assertTrue(p.wait(lambda: p.safety.lock_closed, 5), p.safety)
        # Closing by a reset latches nothing and restarts nothing.
        time.sleep(RESPAWN + 1.0)
        self.assertEqual((starts(), p.status.emergency_reason), (3, 0))


@launch_testing.post_shutdown_test()
class TestExit(TestShutdown):
    pass
