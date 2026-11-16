"""The rest of MowBite OS dies while the mower mows (G25): the whole stack of sim.launch.py with the blade allowed, as
in test_follower_stall.py. Once the blade turns and the mower drives on a path, the container with every node but
tron_mcu is killed (SIGKILL: no node shuts down on its own). fake_tron goes on getting the heartbeat, so like the MCU it
holds the last setpoint and the blade. tron_mcu runs in a process of its own and has to stop both by itself: speed 0
after cmd_vel_timeout, the blade off once its lease ran out or the missing IMU latched IMU_TIMEOUT. tron.launch.py
then starts the container again: the logic comes back idle with the restart hold (G35, OS_CRASH after MOWING), the
bridge logs why, S2 latches UNEXPECTED_RESTART at the driver, a release at the driver alone does not lift it, only
emergency.reset of the logic does. A crash in IDLE only tells, and after the fourth crash the container stays down.
With a disk that cannot be written (a folder where the file is written first): a start whose os-session.json does not
get there is refused and nothing moves; a job goes on when the disk fails meanwhile; a restart hold that cannot be
written holds in memory and is written once it can. The runs the kill left without their end come back as an incident
CRASH (FP23 B2a) that replays bit-identically.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_stack_crash.py
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import launch_testing.asserts
import rclpy
from ament_index_python.packages import get_package_prefix
from launch.events.process import ProcessStarted
from mowbite_msgs.msg import (HardwareStatus, MapStatus, MqttPublish, RobotPose, RobotState, RpcError, RpcRequest,
                              RpcResponse)
from mowbite_msgs.srv import ReplaceMap, SetEmergency, SetPose
from rcl_interfaces.msg import Log
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (EXIT_CODES, LATCHED, ROSOUT, SimProbe, container_pid, last_time, restarted,  # noqa: E402
                        sim_description, sim_files, wait_until)

WORK = tempfile.mkdtemp(prefix="mowbite_crash_")
STATE, FLAGS, RX, DATA = sim_files(WORK)
RESTART_HOLD = os.path.join(DATA, "restart-hold.json")
SESSION = os.path.join(DATA, "os-session.json")
HISTORY = os.path.join(DATA, "event_history")
UNEXPECTED_RESTART = HardwareStatus.REASON_UNEXPECTED_RESTART
CONFIG = os.path.join(WORK, "crash.yaml")
# The journal of ctest (MOWBITE_JOURNAL_DIR), else one of its own.
JOURNAL = os.environ.get("MOWBITE_JOURNAL_DIR") or os.path.join(WORK, "journal")
REPLAY = os.path.join(get_package_prefix("mowbite_replay"), "bin", "mowbite_replay")
# The Tron: cmd_vel_timeout of tron_mcu and blade_lease of the logic. fake_tron stops on its own only after 2 s
# without any frame of the host.
CMD_VEL_TIMEOUT = 0.5
BLADE_LEASE = 1.0
HOST_TIMEOUT = 2.0
HEARTBEAT = 0xFF
# stack.py: restarts of the container and the wait before each.
RESTARTS = 3
RESTART_DELAY = 2.0
# SpeedData 0, 0 as sent.
ZERO = "00" * 8

# The lawn of test_mowing.py, one loop around it.
LAWN = [(-0.41, -0.11), (2.59, -0.11), (2.59, 1.49), (-0.41, 1.49)]
GARDEN = {
    "areas": [{"id": "lawn", "properties": {"type": "mow", "name": "Lawn", "outline_count": 1},
               "outline": [{"x": x, "y": y} for x, y in LAWN]}],
    "docking_stations": [],
}

with open(CONFIG, "w") as f:
    # Short segments, so that an incident is sealed soon after its window.
    f.write("/**:\n  ros__parameters:\n    journal_segment_s: 20.0\n"
            "tron_mcu:\n  ros__parameters:\n    safety_measurement_date: \"simulation\"\n    allow_blade: true\n"
            "mowbite_logic:\n  ros__parameters:\n    mow_with_blade: true\n    gps_wait_time: 2.0\n"
            # The antenna of fake_tron, as in sim.yaml, which this file replaces.
            "    geometry:\n      antenna_x: 0.098\n      antenna_y: 0.0\n")


def generate_test_description():
    return sim_description(WORK, 18833, extra_config=CONFIG, journal_dir=JOURNAL)


class Probe(SimProbe):
    def __init__(self):
        super().__init__("crash_probe", state=STATE, flags=FLAGS, log=RX)
        self.status = None
        self.pose = None
        self.robot = None
        self.node.create_subscription(MapStatus, "map/status", self._set("status"), LATCHED)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("pose"), 10)
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("hw"), 10)
        self.node.create_subscription(MqttPublish, "bridge/publish", self._publish, 100)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        # Like the rosout publishers, so lines logged before the match with a node still come in.
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.action_pub = self.node.create_publisher(String, "bridge/action", 10)
        self.replace_map = self.node.create_client(ReplaceMap, "map/replace")
        self.set_pose = self.node.create_client(SetPose, "localization/set_pose")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.start()

    def sim_time(self):
        return last_time(self.log_file)

    def driving_now(self):
        """fake_tron time of its newest entry if the newest setpoint it got moves the mower."""
        entries = self.recent()
        speeds = [e for e in entries if "speed" in e and e["t"] > entries[-1]["t"] - 0.2]
        return entries[-1]["t"] if speeds and speeds[-1]["hex"] != ZERO else None


def history(type_):
    """The events of that type the bridge kept in its history, which hears them also before the probe does."""
    out = []
    for name in sorted(os.listdir(HISTORY)) if os.path.isdir(HISTORY) else []:
        with open(os.path.join(HISTORY, name)) as f:
            out += [e for e in map(json.loads, f) if e.get("type") == type_]
    return out


class TestStackCrash(unittest.TestCase):
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

    def test_01_lawn_and_an_aligned_heading(self):
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

    def test_01b_a_start_without_its_record_on_the_disk_is_refused(self):
        p = self.probe
        os.mkdir(SESSION + ".tmp")
        try:
            since = len(p.events)
            p.act("mower_logic:idle/start_mowing")
            refused = p.event(since, "MOWING_REFUSED", 10)
            self.assertTrue(refused, [e["type"] for e in p.events[since:]])
            self.assertEqual((refused["reason"], refused["file"]), ("STORAGE_NOT_WRITABLE", "os-session.json"), refused)
            storage = p.event(since, "STORAGE", 5)
            self.assertEqual((storage or {}).get("reason"), "STORAGE_NOT_WRITABLE", storage)
            self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE", 5), p.robot)
            still, poses = p.standing()
            self.assertTrue(still and poses[1]["linear"] == 0.0, poses)
            with open(SESSION) as f:
                self.assertEqual(json.load(f)["state"], "IDLE")
            # Known now: the next start is refused before it begins, and the event does not come again.
            since = len(p.events)
            p.act("mower_logic:idle/start_mowing")
            refused = p.event(since, "MOWING_REFUSED", 10)
            self.assertEqual((refused or {}).get("reason"), "STORAGE_NOT_WRITABLE", refused)
            time.sleep(1.0)
            self.assertEqual([e["type"] for e in p.events[since:]], ["MOWING_REFUSED"])
        finally:
            os.rmdir(SESSION + ".tmp")
        self.assertTrue(p.logged("data_dir can be written again", 10))

    def test_02_the_blade_turns_and_the_mower_drives(self):
        p = self.probe
        since = len(p.events)
        p.act("mower_logic:idle/start_mowing")
        self.assertIsNotNone(p.event(since, "STATE", 10, state="MOWING"), [e["type"] for e in p.events[since:]])
        self.assertIsNotNone(p.event(since, "BLADES", 90, enabled=True), "the blade never ran")
        self.assertTrue(p.wait(lambda: p.hw is not None and p.hw.blade_motor.rpm > 1000, 5), "no rpm from fake_tron")
        self.assertTrue(p.wait(lambda: (p.truth() or {}).get("linear", 0.0) > 0.05, 30), "the mower does not drive")
        with open(SESSION) as f:
            session = json.load(f)
        self.assertEqual((session["state"], session["clean"]), ("MOWING", False), session)
        self.assertFalse(os.path.exists(RESTART_HOLD))

    def test_02b_a_job_goes_on_when_the_disk_fails_meanwhile(self):
        p = self.probe
        since = len(p.events)
        # Both stay until after the crash: the restart finds neither file writable.
        os.mkdir(SESSION + ".tmp")
        os.mkdir(RESTART_HOLD + ".tmp")
        p.act("mower_logic:mowing/pause")
        storage = p.event(since, "STORAGE", 10)
        self.assertEqual((storage or {}).get("file"), "os-session.json", storage)
        p.act("mower_logic:mowing/continue")
        self.assertTrue(p.wait(lambda: p.robot.current_state == "MOWING", 10), p.robot)
        self.assertTrue(p.wait(lambda: (p.truth() or {}).get("linear", 0.0) > 0.05, 30), "the mower does not drive")
        self.assertFalse([e for e in p.events[since:] if e["type"] == "MOWING_REFUSED"])
        # The record of the start is on the disk: a crash now holds.
        with open(SESSION) as f:
            session = json.load(f)
        self.assertEqual((session["state"], session["clean"]), ("MOWING", False), session)
        heard = len([e for e in p.events if e["type"] == "STORAGE"])
        self.assertTrue(p.wait(lambda: len(history("STORAGE")) == heard, 5), "STORAGE not in the history")
        TestStackCrash.storage_events = len(history("STORAGE"))

    def test_03_without_the_container_the_driver_stops_mower_and_blade(self, proc_info):
        p = self.probe
        self.assertIsInstance(proc_info["tron_mcu_node"], ProcessStarted)
        container = container_pid(proc_info)
        offset = p.clock_offset()
        # Not in a stop at a corner: fake_tron holds a setpoint that moves the mower.
        t0 = p.wait(p.driving_now, 30)
        self.assertTrue(t0, "the mower does not drive")
        # Stop and blade off count from the kill in the time of fake_tron; t0 is only its newest entry read before.
        kill = time.monotonic() - offset
        os.kill(container, signal.SIGKILL)
        TestStackCrash.crash = (container, t0)
        # Long enough for fake_tron to drop setpoint and blade by itself, were the heartbeat gone.
        time.sleep(HOST_TIMEOUT + 1.0)
        rx = p.received(t0)
        speeds = [e for e in rx if "speed" in e]
        zeros = [e for e in speeds if e["hex"] == ZERO]
        self.assertTrue(zeros, f"no stop after the crash: {speeds[-5:]}")
        stop = zeros[0]["t"]
        TestStackCrash.stop = stop
        self.assertLess(stop - kill, CMD_VEL_TIMEOUT + 0.5, f"stop {stop - kill:.2f} s after the crash")
        after = [e for e in speeds if e["t"] >= stop]
        self.assertTrue(all(e["hex"] == ZERO for e in after), after)
        # The stop burst and no stream of zero setpoints.
        self.assertLessEqual(len(after), 3, after)
        cutter = [e for e in rx if "cutter" in e]
        self.assertTrue(cutter, "no blade frame after the crash")
        off = next((e["t"] for e in cutter if e["cutter"] == 0), None)
        self.assertIsNotNone(off, cutter)
        self.assertLess(off - kill, BLADE_LEASE + 0.5, f"blade off {off - kill:.2f} s after the crash")
        self.assertFalse([e for e in cutter if e["t"] >= off and e["cutter"] != 0], cutter)
        # The driver went on talking, so it was its own frames that stopped the setpoint and the blade.
        beats = [e for e in rx if e["type"] == HEARTBEAT and e["t"] > t0 + HOST_TIMEOUT]
        self.assertGreaterEqual(len(beats), 5, "the heartbeat stopped")
        still, poses = p.standing()
        self.assertTrue(still, poses)
        self.assertEqual((poses[1]["linear"], poses[1]["angular"]), (0.0, 0.0), poses)
        hw = p.wait(lambda: p.hw is not None and p.hw.blade_motor.ok and p.hw.blade_motor.rpm == 0 and p.hw, 3)
        self.assertTrue(hw, "fake_tron still reports blade rpm")
        self.assertTrue(hw.emergency and hw.emergency_latched, hw)
        self.assertTrue(hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, hw.emergency_reason)
        self.assertFalse(hw.blade_on or hw.moving, hw)
        print(f"after the crash: stop {stop - kill:.2f} s, blade off {off - kill:.2f} s, "
              f"kill {kill - t0:.3f} s after t0, {len(after)} zero setpoints, "
              f"reason {hw.emergency_reason:#x}", flush=True)

    def test_04_the_driver_keeps_the_mower_blocked(self, proc_info):
        p = self.probe
        t1 = p.sim_time()
        time.sleep(3.0)
        rx = p.received(t1)
        self.assertFalse([e for e in rx if "speed" in e], "a setpoint after the stop")
        self.assertFalse([e for e in rx if e.get("cutter", 0) != 0], "the blade on again")
        self.assertGreaterEqual(len([e for e in rx if e["type"] == HEARTBEAT]), 20, "the heartbeat stopped")
        self.assertTrue(p.hw.emergency and p.hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, p.hw)
        self.assertIsInstance(proc_info["tron_mcu_node"], ProcessStarted)

    def test_05_the_container_comes_back_idle_and_says_why(self, proc_info):
        p = self.probe
        pid, t0 = TestStackCrash.crash
        self.assertTrue(restarted(proc_info, pid, RESTART_DELAY + 10), "the container did not start again")
        self.assertTrue(p.logged(f"its container ended with exit code -9 (restart 1 of {RESTARTS})", 15),
                        [line for line in p.logs if "restart" in line])
        self.assertTrue(p.wait(lambda: p.robot is not None and p.robot.current_state == "IDLE", 10), p.robot)
        # Back with its IMU, the driver keeps the latch until a release.
        self.assertTrue(p.hw.emergency_latched and p.hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, p.hw)
        # The hold cannot be written: the logic holds in memory and tells it, S2 has no file to latch on.
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & UNEXPECTED_RESTART, 10),
                        p.robot)
        self.assertFalse(os.path.exists(RESTART_HOLD))
        self.assertTrue(p.logged("Restart hold in memory until it can be written", 5))
        storage = p.wait(lambda: history("STORAGE")[TestStackCrash.storage_events:], 10)
        self.assertEqual(storage[0]["file"] if storage else None, "os-session.json", storage)
        again = sum("data_dir can be written again" in line for line in p.logs)
        os.rmdir(SESSION + ".tmp")
        os.rmdir(RESTART_HOLD + ".tmp")
        self.assertTrue(p.wait(lambda: os.path.exists(RESTART_HOLD), 5), "the hold was not written again")
        self.assertTrue(p.wait(lambda: sum("data_dir can be written again" in line for line in p.logs) > again, 10))
        # The restart hold: its file, the event, the bit in robot_state and, through S2, at the driver.
        self.assertTrue(os.path.exists(RESTART_HOLD))
        event = p.event(0, "UNEXPECTED_RESTART", 15)
        self.assertTrue(event, [e["type"] for e in p.events])
        self.assertEqual((event["cause"], event["reason"], event["state_before"], event["hold"]),
                         ("OS_CRASH", "OS_CRASH", "MOWING", True), event)
        self.assertEqual((event["severity"], event["resume"]), ("error", "user"), event)
        self.assertIsNone(event["crashed_at"])
        self.assertTrue(event["job_id_before"], event)
        self.assertTrue(p.wait(lambda: p.robot.emergency and p.robot.emergency_reason & UNEXPECTED_RESTART, 5),
                        p.robot)
        self.assertTrue(p.wait(lambda: p.hw.emergency_reason & UNEXPECTED_RESTART, 5), p.hw)
        speeds = [e for e in p.received(TestStackCrash.stop) if "speed" in e]
        self.assertTrue(all(e["hex"] == ZERO for e in speeds) and len(speeds) <= 3, speeds)
        self.assertFalse([e for e in p.received(t0) if e.get("cutter", 0) != 0], "the blade on again")

    def test_05b_the_runs_without_their_end_are_an_incident(self):
        p = self.probe
        pid, _ = TestStackCrash.crash

        def crash():
            answer = p.rpc("incidents.list", timeout=2) or {}
            return next((i for i in answer.get("result", []) if "CRASH" in i["triggers"] and i["state"] == "done"),
                        None)
        # Found at the first pass after the start, sealed once its window and segments are closed.
        self.assertTrue([n for n in os.listdir(JOURNAL) if n.endswith(f"_{pid}")], os.listdir(JOURNAL))
        row = wait_until(crash, 120)
        self.assertTrue(row, p.rpc("incidents.list"))
        self.assertTrue(row["safety"], row)
        manifest = p.rpc("incidents.get", {"id": row["id"]})["result"]["manifest"]
        crashed = [r for r in manifest["runs"] if r["pid"] == pid]
        self.assertTrue(crashed, [(r["name"], r["pid"]) for r in manifest["runs"]])
        self.assertFalse(any(r["ended"] for r in crashed), crashed)
        self.assertTrue(any(t["kind"] == "CRASH" and t["safety"] for t in manifest["triggers"]), manifest["triggers"])
        out = subprocess.run([REPLAY, "check", os.path.join(JOURNAL, "incidents", row["id"])], capture_output=True,
                             text=True, timeout=300)
        self.assertEqual(out.returncode, 0, out.stdout[-3000:] + out.stderr[-2000:])

    def test_06_a_release_leaves_the_mower_standing(self):
        p = self.probe
        # At the driver alone the hold stays: S2 latches it again.
        answer = p.call(p.emergency, SetEmergency.Request(latch=False, reason="stack crash test"))
        self.assertTrue(answer.success, answer.message)
        time.sleep(2.5)
        self.assertTrue(p.hw.emergency and p.hw.emergency_reason & UNEXPECTED_RESTART, p.hw)
        self.assertTrue(os.path.exists(RESTART_HOLD))
        # emergency.reset of the logic: the file goes, then S2 and the driver let go.
        since = len(p.events)
        p.act("mower_logic/reset_emergency")
        self.assertTrue(p.wait(lambda: not p.hw.emergency, 5), p.hw)
        self.assertFalse(os.path.exists(RESTART_HOLD))
        self.assertTrue(p.wait(lambda: not p.robot.emergency and p.robot.emergency_reason == 0, 5), p.robot)
        self.assertIsNotNone(p.event(since, "EMERGENCY", 5, emergency=False, source="rpc"),
                             p.events[since:])
        t2 = p.sim_time()
        time.sleep(3.0)
        self.assertEqual(p.robot.current_state, "IDLE")
        rx = p.received(t2)
        self.assertFalse([e for e in rx if "speed" in e and e["hex"] != ZERO], "it drove again")
        self.assertFalse([e for e in rx if e.get("cutter", 0) != 0], "the blade on again")
        still, poses = p.standing()
        self.assertTrue(still, poses)

    def test_07_after_three_restarts_the_container_stays_down(self, proc_info):
        p = self.probe
        pid = container_pid(proc_info)
        for restart in range(2, RESTARTS + 1):
            since = len(p.events)
            os.kill(pid, signal.SIGKILL)
            pid = restarted(proc_info, pid, RESTART_DELAY + 10)
            self.assertTrue(pid, f"no restart {restart}")
            # Loaded once the bridge says so; only then the next crash.
            self.assertTrue(p.logged(f"exit code -9 (restart {restart} of {RESTARTS})", 15))
            # In IDLE an unexpected end only tells.
            event = p.event(since, "UNEXPECTED_RESTART", 15)
            self.assertTrue(event, [e for e in p.events if e["type"] == "UNEXPECTED_RESTART"])
            self.assertEqual((event["cause"], event["state_before"], event["hold"], event["severity"], event["resume"]),
                             ("OS_CRASH", "IDLE", False, "warning", "auto"), event)
            self.assertFalse(os.path.exists(RESTART_HOLD))
        t3 = p.sim_time()
        os.kill(pid, signal.SIGKILL)
        self.assertIsNone(restarted(proc_info, pid, RESTART_DELAY + 4), "a restart more than allowed")
        self.assertIsNone(container_pid(proc_info))
        self.assertIsInstance(proc_info["tron_mcu_node"], ProcessStarted)
        rx = p.received(t3)
        self.assertFalse([e for e in rx if "speed" in e and e["hex"] != ZERO], "it drove again")
        self.assertGreaterEqual(len([e for e in rx if e["type"] == HEARTBEAT]), 30, "the heartbeat stopped")
        self.assertTrue(p.hw.emergency and p.hw.emergency_reason & HardwareStatus.REASON_IMU_TIMEOUT, p.hw)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_the_container_was_killed_last_and_the_driver_ended_cleanly(self, proc_info):
        # The first one and its three successors.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[-signal.SIGKILL],
                                               process="component_container_isolated", strict_proc_matching=False)
        self.assertEqual(len([a for a in proc_info.processes()
                              if "component_container_isolated" in a.process_details["name"]]), RESTARTS + 1)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES,
                                               process="tron_mcu_node")
