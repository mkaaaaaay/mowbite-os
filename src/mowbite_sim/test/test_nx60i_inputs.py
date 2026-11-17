"""The setup program assigns the inputs of the NX60i, with MowBite OS as mowbite.launch.py starts it against fake_xcore:
the launch runs as a process of its own that is started again whenever it ends, as compose does on the mower. The step
inputs_assign restarts the OS with the driver learning (INPUTS_LEARNING, nothing stops), takes one line per sensor,
writes the table into the parameter file of the program and restarts once more: the driver comes up with that table.
Each end of the launch is a stop in order (clean session, no note of a crash, no restart hold). A latch from above
during learning waits for a release before the restart. The board stays held from learning until the user releases it;
after that the steps of the inputs run against the new table, and the program itself releases the emergency each
press brought.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_nx60i_inputs.py
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
import yaml
from launch.actions import ExecuteProcess, TimerAction
from mowbite_msgs.msg import (HardwareInputs, HardwareStatus, MqttPublish, RobotState, RpcError, RpcMethods,
                              RpcRequest, RpcResponse)
from mowbite_msgs.srv import SetEmergency

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HISTORY, LATCHED, SimProbe, TestShutdown, fake_xcore, journal_dir, layers,  # noqa: E402,F401
                        loopback_group, sim_files, wait_until, write_json, xcore_config)

WORK = tempfile.mkdtemp(prefix="mowbite_nx60i_inputs_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
# One line for each start of the launch.
STARTS = os.path.join(WORK, "starts")
# Each start but the first begins this long after the launch before it ended (s).
RESPAWN = 2.0
STEP = "inputs_assign"
LIST = json.loads(layers.setup("nx60i")["steps"])
STEPS = {step["id"]: step for step in LIST["steps"]}
DRIVER = STEPS[STEP]["params"]["node"]
QUIET = STEPS[STEP]["params"].get("quiet", 10)
CANDIDATES = layers.parameters([xcore_config()], DRIVER)["inputs.candidates"]
LEARNING = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_INPUTS_LEARNING
# The sensors as this mower is wired, by part of the step: the flag of fake_xcore and its line. Both STOP buttons and
# the sides of the bumper are the other way round than in the table of nx60i.yaml.
WIRED = {"stop_top": ("rear_stop", "GPIO5"), "stop_rear": ("stop", "GPIO1"), "lift_left": ("lift_left", "GPIO10"),
         "lift_right": ("lift_right", "GPIO11"), "bumper_left": ("bumper_right", "GPIO12"),
         "bumper_right": ("bumper_left", "GPIO13")}
# What the launch leaves in data_dir after a crash or for a hold.
CRASH_NOTES = ("container_exit.json", "driver_exit.json", "watch_exit.json", "restart-hold.json")

# Before each start: the session the OS left and the names in data_dir, as the next start finds them.
SCRIPT = os.path.join(WORK, "os.sh")
with open(SCRIPT, "w") as f:
    f.write(f"""n=$(wc -l < {STARTS} 2>/dev/null || echo 0)
cp {DATA}/os-session.json {WORK}/session.$n.json 2>/dev/null
ls {DATA} > {WORK}/files.$n.txt 2>/dev/null
echo start >> {STARTS}
exec ros2 launch mowbite_bringup mowbite.launch.py platform:=nx60i interface:=lo bind_ip:={BIND_IP} \\
  announcement_port:={GROUP_PORT} data_dir:={DATA} journal_dir:={JOURNAL} datum_lat:={DATUM[0]} \\
  datum_long:={DATUM[1]} datum_height:={DATUM[2]} mqtt_port:={random.randint(18000, 18999)}
""")
write_json(FLAGS, {"gps_fix": "fix"})


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--log", LOG, "--log-max-mb", "60")
    # The stop at the end of the test takes the whole stack down: longer than the default before SIGTERM and SIGKILL.
    system = ExecuteProcess(cmd=["sh", SCRIPT], output="screen", respawn=True, respawn_delay=RESPAWN,
                            sigterm_timeout="30", sigkill_timeout="30")
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[system, launch_testing.actions.ReadyToTest()]),
    ]), {}


def set_flags(*names):
    write_json(FLAGS, {"gps_fix": "fix", **{name: 1 for name in names}})


def starts():
    try:
        with open(STARTS) as f:
            return len(f.readlines())
    except OSError:
        return 0


def left_by(start):
    """What the launch of that number (from 1) left when it ended: (the session, the names in data_dir)."""
    with open(os.path.join(WORK, f"session.{start}.json")) as f:
        session = json.load(f)
    with open(os.path.join(WORK, f"files.{start}.txt")) as f:
        return session, f.read().split()


class Probe(SimProbe):
    offer_timeout = 30.0
    call_timeout = 20.0
    emergency_reason = "inputs test"

    def __init__(self):
        super().__init__("nx60i_inputs_probe")
        self.status = None
        self.inputs = None
        self.robot = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareInputs, "hw/inputs", self._set("inputs"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
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
            self.status = self.inputs = self.robot = None
            self.methods.clear()

    def up(self, timeout=90.0):
        """Waits for the OS of this start: driver, logic and the setup program."""
        return self.wait(lambda: self.status is not None and self.status.telemetry_ok and self.inputs is not None
                         and self.robot is not None and self.methods.get("mowbite_setup"), timeout)

    def step(self, step=STEP):
        answer = self.rpc("setup.get")
        assert answer and "result" in answer, f"setup.get: {answer}"
        return next(entry for entry in answer["result"]["steps"] if entry["id"] == step)

    def step_when(self, pred, timeout, step=STEP):
        """The entry of the step once pred holds for it, else None."""
        def look():
            entry = self.step(step)
            return entry if pred(entry) else None
        # Without the lock of the probe: the answer of setup.get comes in under it.
        return wait_until(look, timeout)

    def change(self, method, step=STEP, **params):
        """A setup.* call for the step with the version of now, once more if the state moved in between."""
        for _ in range(3):
            version = self.rpc("setup.get")["result"]["version"]
            answer = self.rpc(method, {"step": step, "version": version, **params})
            data = ((answer or {}).get("error") or {}).get("data") or {}
            if data.get("reason") != "VERSION":
                break
        return answer

    def lines(self):
        return {line.name: line for line in self.inputs.inputs}

    def active(self):
        return sorted(line.name for line in self.inputs.inputs if line.active)


class TestNx60iInputs(unittest.TestCase):
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
        """Waits for start `number` of the launch and for its OS; what the launch before it left."""
        p = self.probe
        self.assertTrue(p.wait(lambda: starts() >= number, timeout), "the launch did not start again")
        p.forget()
        self.assertTrue(p.up(), f"start {number} does not come up")
        self.assertEqual(starts(), number)
        session, files = left_by(number - 1)
        # An end in order: the logic closed its session, the launch wrote no note of a crash, nothing holds the mower.
        self.assertTrue(session["clean"], session)
        self.assertEqual(session["state"], "IDLE")
        self.assertEqual([name for name in CRASH_NOTES if name in files], [])
        self.assertIn("setup_restart.json", files)
        self.assertFalse(os.path.exists(os.path.join(DATA, "setup_restart.json")))
        return session

    def part_waits(self, index, timeout=10.0):
        entry = self.probe.step_when(lambda e: e["state"] != "running" or (
            e["parts"][index]["state"] == "waiting" and not e["blocked"] and e["ask"] is None), timeout)
        self.assertIsNotNone(entry, f"part {index} does not wait: {self.probe.step()}")
        self.assertEqual(entry["state"], "running", entry["result"])
        # The program takes the levels at rest with its next look.
        time.sleep(0.3)

    def asked(self, timeout=5.0):
        entry = self.probe.step_when(lambda e: e["state"] != "running" or e["ask"] is not None, timeout)
        self.assertIsNotNone(entry, f"no proposal: {self.probe.step()}")
        self.assertEqual(entry["state"], "running", entry["result"])
        return entry["ask"]

    def answer(self, accept):
        answer = self.probe.change("setup.answer", values={"accept": accept})
        self.assertIn("result", answer, answer)

    def press(self, step, index, flag, bit, delay):
        """One part of a step input: the sensor of that flag until its emergency `bit`, then let go. The program
        releases the emergency itself; the entry of the step after it."""
        p = self.probe
        events = len(p.events)
        entry = p.step_when(lambda e: e["state"] != "running" or (
            e["parts"][index]["state"] == "waiting" and not e["blocked"]), 10.0, step)
        self.assertEqual((entry or {}).get("state"), "running", f"{step} part {index} does not wait: {p.step(step)}")
        time.sleep(0.3)
        set_flags(flag)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason & bit, delay + 2.0),
                        f"{step} {flag}: no emergency, {p.status.emergency_reason}")
        self.assertEqual(p.status.emergency_reason, bit | HardwareStatus.REASON_LATCH)
        self.assertNotEqual(p.status.latch_count, 0)
        # The logic stays in IDLE during the emergency: the program releases and restarts in that state only.
        self.assertTrue(p.wait(lambda: p.robot.emergency, 2.0))
        self.assertEqual(p.robot.current_state, "IDLE")
        time.sleep(0.5)
        self.assertEqual(p.step(step)["parts"][index]["state"], "waiting")
        set_flags()
        # Nobody releases it in the app.
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 6.0),
                        f"{step} {flag}: the emergency stays, {p.status.emergency_reason}: {p.step(step)}")
        entry = p.step_when(lambda e: e["state"] != "running" or e["parts"][index]["state"] == "ok", 5.0, step)
        self.assertIsNotNone(entry, f"{step} part {index} stays open: {p.step(step)}")
        self.assertEqual(entry["parts"][index]["state"], "ok", entry["result"])
        # The logic saw it begin and end and refused nothing: the release went past it.
        self.assertTrue(p.wait(lambda: not p.robot.emergency, 3.0), p.robot)
        told = [e["type"] for e in p.events[events:] if e["type"].startswith("EMERGENCY")]
        self.assertEqual(told, ["EMERGENCY", "EMERGENCY"], p.events[events:])
        return entry

    def test_1_the_os_comes_up_with_the_table_of_the_platform(self):
        p = self.probe
        self.assertTrue(p.up(), "the OS does not come up")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 60), p.robot)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)
        self.assertEqual(starts(), 1)
        self.assertFalse(p.inputs.learning)
        lines = p.lines()
        self.assertEqual((lines["GPIO1"].role, lines["GPIO5"].role, lines["GPIO12"].role, lines["GPIO13"].role),
                         ("stop", "stop", "bumper_right", "bumper_left"))
        self.assertTrue(all(line.assigned and not line.held for line in lines.values()), lines)
        entry = p.step_when(lambda e: not e["blocked"], 10)
        self.assertIsNotNone(entry, p.step())
        self.assertEqual((entry["state"], entry["skippable"]), ("open", True))
        self.assertEqual([part["id"] for part in entry["parts"]], list(WIRED))
        self.assertFalse(os.path.exists(layers.setup_file(DATA, "nx60i")))

    def test_2_the_step_restarts_the_os_with_the_driver_learning(self):
        p = self.probe
        answer = p.change("setup.start")
        self.assertIn("result", answer, answer)
        self.restarted(2)
        self.assertTrue(p.wait(lambda: p.inputs.learning and p.inputs.inputs, 10), "the driver does not learn")
        self.assertEqual([line.name for line in p.inputs.inputs], CANDIDATES)
        for line in p.inputs.inputs:
            self.assertEqual((line.role, line.assigned, line.active), ("", False, False), line.name)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == LEARNING, 10), p.status.emergency_reason)
        # IDLE with that emergency on: the restart out of learning needs it.
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.robot.emergency, 30), p.robot)
        with open(layers.setup_file(DATA, "nx60i")) as f:
            self.assertEqual(yaml.safe_load(f), {DRIVER: {"ros__parameters": {"inputs.learn": True}}})
        entry = p.step()
        self.assertEqual((entry["state"], entry["attempt"]), ("running", 1))
        # Another step does not start while the driver learns.
        refused = p.change("setup.start", step="stop")
        self.assertEqual(refused["error"]["data"]["reason"], "BUSY", refused)

    def test_3_one_line_per_sensor(self):
        p = self.probe
        parts = list(WIRED)
        self.part_waits(0, QUIET + 15)
        # Two sensors at once are no proposal; the part waits again once both are back.
        set_flags("stop", "lift_left")
        time.sleep(1.0)
        self.assertIsNone(p.step()["ask"])
        set_flags()
        self.part_waits(0)
        # A proposal the user turns down: the part is asked again.
        set_flags("lift_right")
        self.assertEqual(self.asked(), {"part": parts[0], "line": "GPIO11", "active_high": True})
        set_flags()
        self.answer("no")
        for index, part in enumerate(parts):
            flag, line = WIRED[part]
            self.part_waits(index)
            set_flags(flag)
            self.assertEqual(self.asked(), {"part": part, "line": line, "active_high": True})
            # No sensor stops anything meanwhile.
            self.assertEqual(p.status.emergency_reason, LEARNING)
            if index == len(parts) - 1:
                break
            set_flags()
            self.answer("yes")
            self.assertEqual(p.step_when(lambda e: e["parts"][index]["state"] == "ok", 3)["state"], "running")
        self.assertEqual(starts(), 2)

    def test_4_a_latch_from_above_waits_for_its_release_before_the_restart(self):
        p = self.probe
        self.assertTrue(p.set_emergency(True).success)
        both = LEARNING | HardwareStatus.REASON_HIGH_LEVEL
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == both, 3), p.status.emergency_reason)
        set_flags()
        self.answer("yes")
        entry = p.step_when(lambda e: [b["reason"] for b in e["blocked"]] == ["EMERGENCY"], 5)
        self.assertIsNotNone(entry, p.step())
        time.sleep(RESPAWN + 2.0)
        self.assertEqual(starts(), 2)
        self.assertEqual(p.step()["state"], "running")
        # The release of the app ends the latch from above; the one of learning stays and is told.
        answer = p.rpc("emergency.reset")
        self.assertEqual(((answer.get("error") or {}).get("data") or {}).get("reason"), "LEARNING", answer)
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == LEARNING or starts() > 2, 5),
                        p.status.emergency_reason)

    def test_5_the_os_restarts_with_the_table_found(self):
        p = self.probe
        self.restarted(3)
        entry = p.step_when(lambda e: e["state"] != "running", 30)
        self.assertIsNotNone(entry, p.step())
        self.assertEqual((entry["state"], entry["result"]["ok"], entry["attempt"]), ("ok", True, 1), entry["result"])
        self.assertEqual({value["id"]: value["value"] for value in entry["values"]},
                         {part: line for part, (_, line) in WIRED.items()})
        # The driver read the file of the program: the lines in the order of the parts, each with role and delay.
        self.assertFalse(p.inputs.learning)
        lines = p.lines()
        self.assertEqual(list(lines), [line for _, line in WIRED.values()])
        for part in STEPS[STEP]["parts"]:
            line = lines[WIRED[part["id"]][1]]
            self.assertEqual((line.role, line.assigned, line.active_high, line.delay_ms, line.active),
                             (part["role"], True, True, part["delay_ms"], False), part["id"])
        with open(layers.setup_file(DATA, "nx60i")) as f:
            table = yaml.safe_load(f)[DRIVER]["ros__parameters"]
        self.assertEqual(table, {"inputs.gpio_lines": [line for _, line in WIRED.values()],
                                 "inputs.gpio_active_high": [True] * 6,
                                 "inputs.gpio_reasons": ["stop", "stop", "lift", "lift", "collision", "collision"],
                                 "inputs.gpio_delays_ms": [10, 10, 2500, 2500, 500, 500],
                                 "inputs.gpio_roles": [part["role"] for part in STEPS[STEP]["parts"]]})
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 60), p.robot)
        # The driver that learned held the board, and no restart takes that away: it waits for the release of the user.
        held = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_HIGH_LEVEL
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == held, 10), p.status.emergency_reason)
        time.sleep(2.0)
        self.assertEqual(p.status.emergency_reason, held)
        self.assertIn("result", p.rpc("emergency.reset"))
        self.assertTrue(p.wait(lambda: p.status.emergency_reason == 0, 10), p.status.emergency_reason)
        self.assertEqual([name for name in CRASH_NOTES if os.path.exists(os.path.join(DATA, name))], [])

    def test_6_the_inputs_are_checked_with_the_new_table_and_released_by_the_program(self):
        p = self.probe
        # The side of the bumper follows the table: the sensor the board has on GPIO12 is the left one now.
        cases = (("stop", ("stop", "rear_stop"), HardwareStatus.REASON_STOP, 0.01),
                 ("bumper", ("bumper_right", "bumper_left"), HardwareStatus.REASON_COLLISION, 0.5))
        for step, flags, bit, delay in cases:
            self.assertEqual(p.step(step)["state"], "open")
            self.assertIn("result", p.change("setup.start", step=step))
            for index, flag in enumerate(flags):
                entry = self.press(step, index, flag, bit, delay)
            self.assertEqual(entry["state"], "ok", entry["result"])
            for value in entry["values"]:
                self.assertAlmostEqual(value["value"], delay, delta=0.3, msg=f"{step} {value['id']}")
        lines = {line["name"]: line for line in p.step("bumper")["lines"]}
        self.assertEqual((lines["GPIO12"]["role"], lines["GPIO13"]["role"]), ("bumper_left", "bumper_right"))
        # A mower released that way stands: nothing began by itself.
        self.assertEqual((p.robot.current_state, p.status.emergency_reason), ("IDLE", 0))
        self.assertEqual(starts(), 3)


@launch_testing.post_shutdown_test()
class TestExit(TestShutdown):
    pass
