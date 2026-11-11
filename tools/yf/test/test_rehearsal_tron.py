"""Rehearsal of T1-T5 against fake_tron: measure.py --platform tron starts tron_imu and tron_mcu on the fake's ptys, the
person's actions go to the fake's flags. The fake reports the noise of standing wheels and the motors like the Tron.
With the fake holding the setpoint 1.5 s after the last SpeedData, every host-gone run and heartbeat alone have to find
that time and the proposal has to follow from it; with 4 s the limit has to end T2; a key while tron_mcu is frozen
has to continue it and stop the wheels. T6: the blade motor of the fake stops without host like its wheels, and the
runs have to find that; with the flag cutter_holds it keeps turning, which has to end T6 with the finding and the blade
off. With --blade-percent 100 and --blade-window 5 every blade run has to take those values; a window longer than the
lease of a DDS participant (20 s) has to end with the same finding and nothing else. No process may be left.

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
    def set_flags(cls, flags):
        with open(cls.flags + ".tmp", "w") as f:
            json.dump(flags, f)
        os.replace(cls.flags + ".tmp", cls.flags)

    def measure(self, name, *args, abort_when=None, timeout=900):
        """measure.py in its own record dir; abort_when(records) makes the abort file once it holds."""
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

    def test_1_programme(self):
        self.set_flags({"host_timeout": HOLD_S})
        code, output, rec = self.measure("all", "--repeats", "2")
        end = rec[-1]
        self.assertEqual(end["kind"], "end")
        self.assertEqual(end["findings"], [])
        self.assertTrue(end["complete"])
        self.assertEqual(code, 0)
        self.assertEqual([r for r in rec if r["kind"] == "firmware"][0]["cutter"], "0.6.36")
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
        offs = [r for r in rec if r["kind"] == "blade_off"]
        self.assertEqual([(r["percent"], r["rpm"]) for r in offs], [(30, 840), (100, 2800)])
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
        self.assertEqual(run["last_rpm"], 840)
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
        self.assertEqual([(r["percent"], r["rpm"]) for r in rec if r["kind"] == "blade_off"], [(100, 2800)])
        self.assertEqual([r["percent"] for r in rec if r["kind"] == "blade_lease"], [100])
        self.assertEqual([r["rpm"] for r in rec if r["kind"] == "blade_emergency"], [2800] * 3)
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
        self.assertEqual(run["last_rpm"], 840)
        # The fake reports the motors once a second: the capture holds the whole window.
        self.assertGreaterEqual(run["samples"], 24)
        status = [r for r in rec if r["kind"] == "status"]
        self.assertEqual(status[-1]["blade_rpm"], 0)


if __name__ == "__main__":
    unittest.main()
