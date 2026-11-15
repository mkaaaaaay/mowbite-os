"""Rehearsal of T1-T5 against fake_tron: measure.py --platform tron starts tron_imu and tron_mcu on the fake's ptys, the
person's actions go to the fake's flags. The fake reports the noise of standing wheels and the motors like the Tron.
With the fake holding the setpoint 1.5 s after the last SpeedData, every host-gone run and heartbeat alone have to find
that time and the proposal has to follow from it; with 4 s the limit has to end T2; a key while tron_mcu is frozen
has to continue it and stop the wheels. T6: the blade motor of the fake stops without host like its wheels, and the
runs have to find that; with the flag cutter_holds it keeps turning, which has to end T6 with the finding and the blade
off. With --blade-percent 100 and --blade-window 5 every blade run has to take those values; a window longer than the
lease of a DDS participant (20 s) has to end with the same finding and nothing else. T7: by default only the front
goes up by hand and has to show lift at the tilt of the fake, latched until the release, the mower lifted level too,
and each lift of the front on the turning blade motor has to stop it; with the flag lift_tilt the fake shows no lift
for the level lift, which has to be the one finding (G1) without an abort. With the flag lift_front_only the fake
shows lift as the Tron, only with the front up or level: --lift-sides left,right sees no lift at both sides, which is
no finding, and the front on the blade motor goes up until lift shows. The fake keeps STOP until a clear frame, as the
Tron does, so a run may start with STOP kept from the last one, as on the Tron after each start of tron_mcu, and has to
release it first. The STOP check has to pass before the first wheel command and before the first blade on; STOP held
through its release has to end the run before any. T8: STOP kept until the clear frame and while pressed, bumper and
lift not kept, no wheel command and no blade at all. No process may be left.

Reading the port while tron_mcu holds it (TIOCEXCL) needs root with CAP_SYS_ADMIN, as on the Tron:
  docker run --user 0 --cap-add SYS_ADMIN ... python3 -m unittest tools/yf/test/test_rehearsal_tron.py
"""
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import measure as m  # noqa: E402

MEASURE = os.path.join(HERE, "..", "measure.py")
FAKE = os.path.join(get_package_prefix("mowbite_sim"), "lib", "mowbite_sim", "fake_tron")
HOLD_S = 1.5
# The recording, not the decoded captures next to it.
RECORDING = "measure-????????-??????.jsonl"


def records(path):
    with open(path) as f:
        return [json.loads(line) for line in f]


def nodes_left():
    left = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                name = os.path.basename(f.read().split(b"\0")[0].decode())
        except OSError:
            continue
        if name in ("tron_mcu_node", "tron_imu_node") and m.process_state(int(pid)) not in (None, "Z"):
            left.append(int(pid))
    return left


@unittest.skipUnless(os.geteuid() == 0, "reads a port held with TIOCEXCL: root with CAP_SYS_ADMIN, as on the Tron")
class TestRehearsalTron(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.mkdtemp(prefix="mowbite_rehearsal_tron_")
        cls.flags = os.path.join(cls.work, "flags.json")
        cls.links = {name: os.path.join(cls.work, name) for name in ("mcu", "gnss", "imu")}
        cls.state = os.path.join(cls.work, "state.json")
        cls.set_flags({"host_timeout": HOLD_S})
        cls.fake = subprocess.Popen(
            [FAKE, "--link", cls.links["mcu"], "--gnss-link", cls.links["gnss"], "--imu-link", cls.links["imu"],
             "--flags", cls.flags, "--state", cls.state, "--host-timeout", "2.0", "--standstill-noise"],
            stdout=open(os.path.join(cls.work, "fake.log"), "w"), stderr=subprocess.STDOUT)
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.fake.send_signal(signal.SIGTERM)
        cls.fake.wait(10)
        shutil.rmtree(cls.work, ignore_errors=True)

    @classmethod
    def set_flags(cls, flags=None):
        """`flags` over STOP kept until a clear frame and the hold time HOLD_S."""
        with open(cls.flags + ".tmp", "w") as f:
            json.dump({"host_timeout": HOLD_S, "stop_latches": 1, **(flags or {})}, f)
        os.replace(cls.flags + ".tmp", cls.flags)

    def measure(self, name, *args, abort_when=None, hold_stop_when=None, timeout=900):
        """measure.py in its own record dir; abort_when(records) makes the abort file once it holds, from when
        hold_stop_when(records) holds STOP stays pressed."""
        record = os.path.join(self.work, name)
        abort = os.path.join(self.work, f"{name}.abort")
        argv = [sys.executable, MEASURE, "--platform", "tron", "--yes", "--fake-flags", self.flags, "--mcu-port",
                self.links["mcu"], "--imu-port", self.links["imu"], "--data-dir", os.path.join(self.work, "data"),
                "--record-dir", record, "--restart-wait", "30", "--abort-file", abort, *args]
        with open(os.path.join(self.work, f"{name}.log"), "w") as log:
            process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
            end = time.monotonic() + timeout
            while process.poll() is None and time.monotonic() < end:
                found = glob.glob(os.path.join(record, RECORDING))
                if abort_when and found and not os.path.exists(abort) and abort_when(records(found[0])):
                    open(abort, "w").close()
                if hold_stop_when and found and hold_stop_when(records(found[0])):
                    self.set_flags({"stop": 1})
                time.sleep(0.05)
            if process.poll() is None:
                process.kill()
            process.wait()
        with open(log.name) as f:
            output = f.read()
        print(output)
        recording, = glob.glob(os.path.join(record, RECORDING))
        return process.returncode, output, records(recording)

    def tearDown(self):
        self.assertEqual(nodes_left(), [])
        self.assertEqual(m.port_holders(self.links["mcu"]), [])

    def test_0_stop_held_through_the_release(self):
        def asked(lines):
            return any(r["kind"] == "mark" and r["step"] == "STOP" for r in lines)
        try:
            code, output, rec = self.measure("stop-held", "--steps", "T3", hold_stop_when=asked)
        finally:
            self.set_flags()
        self.assertEqual(code, 1)
        self.assertIn("STOP: stop or the emergency stays after the release: STOP stuck or held", rec[-1]["findings"])
        check, = [r for r in rec if r["kind"] == "stop_check"]
        self.assertIsNone(check["let_go"])
        self.assertFalse(check["released"] or check["free"])
        self.assertIn("abort", [r["kind"] for r in rec])
        self.assertNotIn("T3", [r["step"] for r in rec if r["kind"] == "mark"])
        self.assertFalse([r for r in rec if r["kind"] == "cmd" and (r["v"] or r["w"])])
        self.assertIn("let go of STOP now", output)

    def test_1_programme(self):
        self.set_flags({"host_timeout": HOLD_S})
        code, output, rec = self.measure("all", "--repeats", "2")
        end = rec[-1]
        self.assertEqual(end["kind"], "end")
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        self.assertEqual([r for r in rec if r["kind"] == "firmware"][0]["cutter"], "0.6.36")
        check, = [r for r in rec if r["kind"] == "stop_check"]
        self.assertEqual(check["button"], "STOP")
        self.assertIsNone(check["let_go"])
        self.assertTrue(check["released"] and check["free"] and check["latched"], check)
        self.assertEqual(m.reason_text(check["reason"]), "LATCH|STOP")
        before = rec[:rec.index(check)]
        self.assertIn("T1", [r["step"] for r in before if r["kind"] == "mark"])
        self.assertFalse([r for r in before if r["kind"] == "cmd" and (r["v"] or r["w"])])
        telemetry, = [r for r in rec if r["kind"] == "telemetry"]
        self.assertGreater(telemetry["frames_per_s"], 40)
        self.assertEqual(telemetry["link_errors"], 0)
        self.assertAlmostEqual(telemetry["a_z"], 9.81, delta=0.3)
        self.assertEqual([r["speed"] for r in rec if r["kind"] == "turn_speed"], [m.TRON_FIRST_SPEED])
        runs = [r for r in rec if r["kind"] == "host_gone"]
        self.assertEqual([r["way"] for r in runs], ["freeze", "freeze", "kill", "kill"])
        for run in runs:
            self.assertTrue(run["turning_at_start"], run)
            self.assertGreaterEqual(run["stand_s"], HOLD_S - 0.2, run)
            self.assertLessEqual(run["stand_s"], HOLD_S + 0.3, run)
            self.assertEqual(run["summary"]["checksum_errors"], 0)
        # Like the Tron, the fake drops the setpoint after its time also while the heartbeat comes.
        heartbeat = [r["stand_s"] for r in rec if r["kind"] == "heartbeat_only"]
        self.assertEqual(len(heartbeat), 2)
        for stand in heartbeat:
            self.assertGreaterEqual(stand, HOLD_S - 0.2)
            self.assertLessEqual(stand, HOLD_S + 0.4)
        starts = [r["params"] for r in rec if r["kind"] == "process" and r["node"] == "tron_mcu"]
        self.assertEqual([p.get("cmd_vel_timeout") for p in starts], [None, None, None, 10.0, None])
        self.assertEqual([p.get("speed_repeat_window") for p in starts], [None, None, None, 0.0, None])
        self.assertTrue(all(p["allow_blade"] is False for p in starts))
        watchdog = [r["stand_s"] for r in rec if r["kind"] == "watchdog"]
        self.assertEqual(len(watchdog), 2)
        # The driver stops cmd_vel_timeout s after the last command; the fake holds no longer than its timeout.
        self.assertTrue(all(0.3 <= s <= 0.8 for s in watchdog), watchdog)
        self.assertTrue(all(r["moving"] is False for r in rec if r["kind"] == "watchdog"))
        directions = [r for r in rec if r["kind"] == "direction"]
        self.assertEqual(len(directions), 4)
        left_turn = directions[2]
        self.assertLess(left_turn["left_rpm"], 0)
        self.assertGreater(left_turn["right_rpm"], 0)
        self.assertEqual(len([r for r in rec if r["kind"] == "emergency_stop"]), 1 + 4 * 2)
        proposal = end["proposal"]
        self.assertIn(f"failsafe_without_host: {m.failsafe_proposal([r['stand_s'] for r in runs]):.1f}", proposal)
        self.assertIn('expected_chassis_version: "0.6.34"', proposal)
        self.assertIn("nothing written", output)
        self.assertFalse(os.path.exists(os.path.join(self.work, "data", "tron.yaml")))

    def test_2_too_long_without_host_ends_t2(self):
        self.set_flags({"host_timeout": 4.0})
        try:
            code, output, rec = self.measure("late", "--steps", "T2", "--repeats", "2")
        finally:
            self.set_flags({"host_timeout": HOLD_S})
        self.assertEqual(code, 1)
        end = rec[-1]
        self.assertIsNone(end["proposal"])
        self.assertTrue(any("limit 3.0 s" in f for f in end["findings"]), end["findings"])
        self.assertEqual([r["way"] for r in rec if r["kind"] == "host_gone"], ["freeze", "freeze"])
        self.assertIn("abort", [r["kind"] for r in rec])
        self.assertNotIn("heartbeat_only", [r["kind"] for r in rec])

    def test_3_key_while_frozen(self):
        def frozen(lines):
            return any(r["kind"] == "signal" and r.get("signal") == "SIGSTOP" for r in lines)
        code, output, rec = self.measure("key", "--steps", "T2", "--repeats", "1", abort_when=frozen)
        self.assertEqual(code, 1)
        kinds = [(r["kind"], r.get("signal")) for r in rec]
        stop = kinds.index(("signal", "SIGSTOP"))
        self.assertIn(("signal", "SIGCONT"), kinds[stop:])
        self.assertLess(kinds.index(("signal", "SIGCONT")), kinds.index(("abort", None)))
        self.assertNotIn("host_gone", [r["kind"] for r in rec])
        time.sleep(0.5)
        with open(self.state) as f:
            self.assertEqual(json.load(f)["linear"], 0.0)

    def test_4_blade_without_host(self):
        code, output, rec = self.measure("blade", "--steps", "T6", "--blade", "--repeats", "1")
        end = rec[-1]
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        check, = [r for r in rec if r["kind"] == "stop_check"]
        self.assertTrue(check["released"], check)
        self.assertFalse([r for r in rec[:rec.index(check)] if r["kind"] == "blade" and r["on"]])
        offs = [r for r in rec if r["kind"] == "blade_off"]
        self.assertEqual([(r["percent"], r["rpm"]) for r in offs], [(30, 3000), (100, 3000)])
        lease, = [r for r in rec if r["kind"] == "blade_lease"]
        for r in offs + [lease]:
            self.assertLessEqual(r["still_s"], 3.0, r)
        requests = [r for r in rec if r["kind"] == "blade" and r["on"]]
        # The lease run counts from its last request, not from the end of the wait after it.
        renewed = max(r["t"] for r in requests if r["t"] < lease["t"])
        self.assertAlmostEqual(lease["t"] - lease["still_s"], renewed, delta=0.2)
        self.assertGreaterEqual(lease["still_s"], 1.0)
        self.assertEqual(len([r for r in rec if r["kind"] == "blade_emergency"]), 3)
        runs = [r for r in rec if r["kind"] == "blade_gone"]
        self.assertEqual([(r["way"], r["percent"]) for r in runs],
                         [("freeze", 30), ("freeze", 100), ("kill", 30), ("kill", 100)])
        for run in runs:
            self.assertTrue(run["turning_at_start"], run)
            # The fake reports the motors once a second.
            self.assertGreaterEqual(run["still_s"], HOLD_S - 0.2, run)
            self.assertLessEqual(run["still_s"], HOLD_S + 1.3, run)
            self.assertEqual(run["last_rpm"], 0)
            self.assertEqual(run["summary"]["checksum_errors"], 0)
        self.assertTrue(all(r["height_mm"] == m.TRON_BLADE_HEIGHT_MM and r["lease_s"] == 1.0 for r in requests))
        starts = [r["params"]["allow_blade"] for r in rec if r["kind"] == "process" and r["node"] == "tron_mcu"]
        self.assertEqual(starts, [False, True, True, True, False])
        self.assertFalse(any(r["kind"] == "cmd" and (r["v"] or r["w"]) for r in rec))

    def test_5_blade_kept_turning_ends_t6(self):
        self.set_flags({"host_timeout": HOLD_S, "cutter_holds": 1})
        try:
            code, output, rec = self.measure("held", "--steps", "T6", "--blade", "--repeats", "1")
        finally:
            self.set_flags({"host_timeout": HOLD_S})
        self.assertEqual(code, 1)
        end = rec[-1]
        self.assertTrue(any("kept the motor turning for 10 s" in f for f in end["findings"]), end["findings"])
        run, = [r for r in rec if r["kind"] == "blade_gone"]
        self.assertIsNone(run["still_s"])
        self.assertEqual(run["last_rpm"], 3000)
        self.assertIn("abort", [r["kind"] for r in rec])
        starts = [r["params"]["allow_blade"] for r in rec if r["kind"] == "process" and r["node"] == "tron_mcu"]
        self.assertEqual(starts, [False, True, False])
        status = [r for r in rec if r["kind"] == "status"]
        self.assertEqual(status[-1]["blade_rpm"], 0)

    def test_6_blade_percent_and_window(self):
        code, output, rec = self.measure("options", "--steps", "T6", "--blade", "--repeats", "1",
                                         "--blade-percent", "100", "--blade-window", "5")
        end = rec[-1]
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        self.assertEqual([(r["percent"], r["rpm"]) for r in rec if r["kind"] == "blade_off"], [(100, 3000)])
        self.assertEqual([r["percent"] for r in rec if r["kind"] == "blade_lease"], [100])
        self.assertEqual([r["rpm"] for r in rec if r["kind"] == "blade_emergency"], [3000] * 3)
        self.assertTrue(all(r["percent"] == 100 for r in rec if r["kind"] == "blade" and r["on"]))
        runs = [r for r in rec if r["kind"] == "blade_gone"]
        self.assertEqual([(r["way"], r["percent"]) for r in runs], [("freeze", 100), ("kill", 100)])
        signals = [(r["signal"], r["t"]) for r in rec if r["kind"] == "signal"]
        frozen = [t for name, t in signals if name == "SIGSTOP"]
        continued = [t for name, t in signals if name == "SIGCONT"]
        self.assertEqual(len(frozen), 1)
        self.assertEqual(len(continued), 1)
        self.assertGreaterEqual(continued[0] - frozen[0], 5.0)
        self.assertLess(continued[0] - frozen[0], 6.0)
        self.assertIn("up to 14 s a run", output)
        self.assertIn("reads the port, 5 s", output.replace("\n", " "))

    def test_7_blade_kept_turning_for_a_long_window(self):
        self.set_flags({"host_timeout": HOLD_S, "cutter_holds": 1})
        try:
            code, output, rec = self.measure("held-window", "--steps", "T6", "--blade", "--repeats", "1",
                                             "--blade-window", "25")
        finally:
            self.set_flags({"host_timeout": HOLD_S})
        self.assertEqual(code, 1)
        # hw/status was back in time after the window: no finding of a motor still turning with its host back.
        finding, = rec[-1]["findings"]
        self.assertIn("kept the motor turning for 25 s", finding)
        run, = [r for r in rec if r["kind"] == "blade_gone"]
        self.assertIsNone(run["still_s"])
        self.assertEqual(run["last_rpm"], 3000)
        # The fake reports the motors once a second: the capture holds the whole window.
        self.assertGreaterEqual(run["samples"], 24)
        status = [r for r in rec if r["kind"] == "status"]
        self.assertEqual(status[-1]["blade_rpm"], 0)

    def lift_records(self, rec, sides=("front",)):
        """The lift_level record and the lift_angle records, which have to be `sides` in that order."""
        angles = [r for r in rec if r["kind"] == "lift_angle"]
        self.assertEqual([(r["side"], r["expected_lift"]) for r in angles], [(s, s == "front") for s in sides])
        level, = [r for r in rec if r["kind"] == "lift_level"]
        starts = [r for r in rec if r["kind"] == "process" and r["node"] == "tron_mcu"]
        self.assertEqual([r["params"]["allow_blade"] for r in starts], [False, True, False])
        # Without a motor before the blade is allowed; the blade only at 100 % and no wheel command.
        self.assertLess(level["t"], starts[1]["t"])
        check, = [r for r in rec if r["kind"] == "stop_check"]
        self.assertLess(check["t"], angles[0]["t"])
        requests = [r for r in rec if r["kind"] == "blade" and r["on"]]
        self.assertTrue(requests)
        self.assertGreater(requests[0]["t"], starts[1]["t"])
        self.assertTrue(all(r["percent"] == m.LIFT_BLADE_PERCENT for r in requests))
        self.assertFalse(any(r["kind"] == "cmd" and (r["v"] or r["w"]) for r in rec))
        return level, angles

    def assert_lifted(self, angle):
        self.assertTrue(angle["lift"] and angle["latched"] and angle["released"], angle)
        # The fake tilts 38 deg ahead of the lift.
        self.assertAlmostEqual(angle["tilt_at_lift"], 38.0, delta=2.0)
        self.assertGreaterEqual(angle["max_tilt"], angle["tilt_at_lift"])

    def test_8_lift_on_the_blade(self):
        # STOP kept from before the start: the run releases it before the STOP check, as on the Tron.
        self.set_flags({"stop": 1})
        time.sleep(0.2)
        self.set_flags()
        code, output, rec = self.measure("lift", "--steps", "T7", "--blade", "--repeats", "2")
        end = rec[-1]
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        before = rec[:rec.index(next(r for r in rec if r["kind"] == "stop_check"))]
        self.assertTrue(any(r["kind"] == "status" and r["stop"] and r["emergency"] for r in before))
        self.assertTrue(any(r["kind"] == "release" and r["success"] for r in before))
        level, (front,) = self.lift_records(rec)
        self.assert_lifted(front)
        self.assertTrue(level["lift"] and level["latched"] and level["released"], level)
        self.assertLess(level["tilt_at_lift"], 5.0)
        stops = [r for r in rec if r["kind"] == "blade_emergency"]
        self.assertEqual([(r["text"], r["run"]) for r in stops],
                         [("lift the front past 38 deg while the blade motor turns", 1),
                          ("lift the front past 38 deg while the blade motor turns", 2)])
        self.assertTrue(all(r["rpm"] == 3000 for r in stops))
        self.assertIn("T7.3 now, within 10 s: lift the front past 38 deg", output)

    def test_9_level_lift_without_lift(self):
        self.set_flags({"host_timeout": HOLD_S, "lift_tilt": 20})
        try:
            code, output, rec = self.measure("level", "--steps", "T7", "--blade", "--repeats", "1",
                                             "--input-wait", "5")
        finally:
            self.set_flags({"host_timeout": HOLD_S})
        self.assertEqual(code, 1)
        end = rec[-1]
        self.assertTrue(end["complete"])
        finding, = end["findings"]
        self.assertTrue(finding.startswith("T7.2: no lift"), finding)
        self.assertIn("G1", finding)
        self.assertNotIn("abort", [r["kind"] for r in rec])
        level, (front,) = self.lift_records(rec)
        self.assert_lifted(front)
        self.assertEqual((level["lift"], level["tilt_at_lift"], level["latched"], level["released"]),
                         (False, None, None, True))
        stops = [r for r in rec if r["kind"] == "blade_emergency"]
        self.assertEqual(len(stops), 1)
        self.assertIn("T7.3 now, within 5 s: lift the front past 38 deg", output)

    def test_10_sides_without_lift_are_no_finding(self):
        self.set_flags({"host_timeout": HOLD_S, "lift_front_only": 1})
        try:
            code, output, rec = self.measure("sides", "--steps", "T7", "--blade", "--repeats", "1",
                                             "--lift-sides", "left,right")
        finally:
            self.set_flags({"host_timeout": HOLD_S})
        end = rec[-1]
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        level, angles = self.lift_records(rec, ("left", "right"))
        for angle in angles:
            self.assertEqual((angle["lift"], angle["tilt_at_lift"], angle["latched"], angle["released"]),
                             (False, None, None, True), angle)
            self.assertAlmostEqual(angle["max_tilt"], 38.0, delta=2.0)
        self.assertEqual(output.count("as expected: the MCU shows lift only with both front wheels off the ground"), 2)
        self.assertTrue(level["lift"] and level["latched"] and level["released"], level)
        stops = [r for r in rec if r["kind"] == "blade_emergency"]
        self.assertEqual([(r["text"], r["run"]) for r in stops],
                         [("lift the front until lift shows while the blade motor turns", 1)])

    def test_11_stop_latch_g37(self):
        self.set_flags({"stop": 1})
        time.sleep(0.2)
        self.set_flags()
        code, output, rec = self.measure("g37", "--steps", "T8")
        end = rec[-1]
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        parts = {r["step"]: r for r in rec if r["kind"] == "g37"}
        self.assertEqual(list(parts), ["T8.1", "T8.2", "T8.3", "T8.4", "T8.5"])
        # The fake keeps STOP until a clear frame, keeps it while pressed, and keeps neither bumper nor lift.
        short = parts["T8.1"]
        self.assertTrue(short["kept"] and short["released"], short)
        self.assertLess(short["free_s"], m.G37_CLEAR_WAIT_S)
        held = parts["T8.2"]
        self.assertIsNone(held["dropped_s"], held)
        self.assertGreaterEqual(held["releases"], m.G37_HOLD_S / m.G37_RELEASE_EVERY_S - 1)
        self.assertIsNotNone(held["free_s"])
        self.assertTrue(held["released"])
        self.assertEqual((parts["T8.3"]["kept"], parts["T8.3"]["released"]), (False, True))
        lift = parts["T8.4"]
        self.assertIsNone(lift["free_up_s"], lift)
        self.assertTrue(lift["released"])
        gone = parts["T8.5"]
        self.assertEqual(gone["before"], {"stop": False, "lift": False})
        self.assertEqual(gone["checksum_errors"], 0)
        self.assertTrue(gone["released"])
        self.assertNotIn("stop_check", [r["kind"] for r in rec])
        self.assertFalse([r for r in rec if r["kind"] == "cmd" and (r["v"] or r["w"])])
        self.assertFalse([r for r in rec if r["kind"] == "blade" and r["on"]])
        self.assertIn("keep it pressed until this programme says let go", output)


if __name__ == "__main__":
    unittest.main()
