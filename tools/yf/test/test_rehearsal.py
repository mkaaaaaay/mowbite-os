"""V1 and V5 of the safety measurement against fake_xcore: measure.py runs M1-M8 through the minimal launch, the person's
actions go to the fake's flags, and xcore_capture must find the fake's known values in its capture (STOP 10 ms, lift
2.5 s, collision 0.5 s, both sensors at once after 10 ms, wheels stand 1.0 s after the host fell silent plus their
run-down, 0.47 m/s per duty, three stops of the driver's watchdog, every start with a fresh command, the blade below
100 rpm about 2 s after off). Then the limits of the recording,
of the fake's capture and of the run time.

Run in a sourced workspace: launch_test tools/yf/test/test_rehearsal.py
"""
import glob
import json
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import xcore_capture as xc  # noqa: E402

MEASURE = os.path.join(HERE, "..", "measure.py")
WORK = tempfile.mkdtemp(prefix="mowbite_rehearsal_")
FLAGS = os.path.join(WORK, "flags.json")
PCAP = os.path.join(WORK, "inner.pcap")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
SERVICES = os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "config", "xcore_nx60i_services.json")
FAKE = os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore")
# As nx60i.yaml, so that the wheels turn as fast as the driver commands.
FULL_DUTY_SPEED = 0.47


def generate_test_description():
    with open(FLAGS, "w") as f:
        json.dump({}, f)
    fake_xcore = ExecuteProcess(
        cmd=[FAKE, "--services", SERVICES, "--group-port", str(GROUP_PORT), "--flags", FLAGS, "--pcap", PCAP,
             "--pcap-max-mb", "80", "--full-duty-speed", str(FULL_DUTY_SPEED)],
        output="screen")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "launch",
                                                   "measure.launch.py")),
        launch_arguments={"interface": "lo", "bind_ip": BIND_IP, "announcement_port": str(GROUP_PORT),
                          "allow_blade": "true", "respawn": "true", "respawn_delay": "1.0"}.items())
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ])


def measure(*args, timeout):
    log = open(os.path.join(WORK, f"measure-{time.monotonic():.0f}.log"), "w")
    done = subprocess.run([sys.executable, MEASURE, "--yes", "--fake-flags", FLAGS, "--restart-wait", "30", *args],
                          stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    log.close()
    with open(log.name) as f:
        return done.returncode, f.read()


class TestRehearsal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        time.sleep(5.0)  # claim and configuration
        record = os.path.join(WORK, "record")
        cls.code, cls.output = measure("--steps", "M1,M2,M3,M4,M5,M6,M7,M8", "--repeats", "1", "--blade", "--ground",
                                       "--fast", "--record-dir", record, timeout=1500)
        print(cls.output)
        cls.recording, = glob.glob(os.path.join(record, "measure-*.jsonl"))
        cap = xc.load_capture(PCAP, SERVICES)
        cls.report = xc.analyse(cap, (xc.NX60I_INPUTS, "test"), {"driver": (40.0, 0.5), "safety": (31.0, 1.0)},
                                15.0, 0.1, marks=xc.load_marks(cls.recording))
        print(xc.text_report(cls.report))

    @classmethod
    def tearDownClass(cls):
        keep = os.environ.get("REHEARSAL_KEEP")
        if keep:
            shutil.copytree(WORK, keep, dirs_exist_ok=True)
        shutil.rmtree(WORK, ignore_errors=True)

    def boot(self):
        boot, = self.report["boots"]
        return boot

    def of_step(self, items, step):
        return [x for x in items if (x.get("step") or "").startswith(step)]

    def test_a_script_ran_through(self):
        self.assertEqual(self.code, 0, self.output)
        self.assertNotIn("ABORT", self.output)
        steps = {m["text"].split()[0] for m in self.boot()["marks"] if m["kind"] == "mark"}
        self.assertEqual(steps, {"M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8"})
        c = self.report["capture"]
        self.assertEqual((c["malformed"], c["dangerous_ports"], c["cut_at_end"]), ({}, {}, False))

    def test_b_m1_inputs(self):
        presses = self.of_step(self.boot()["presses"], "M1")
        expected = [("STOP", 10), ("STOP", 10), ("LIFT", 2500), (None, None), ("LIFT", 2500), ("LIFT_MULTIPLE", 10),
                    ("LIFT_MULTIPLE", 10), ("COLLISION", 500), ("COLLISION", 500), ("COLLISION_MULTIPLE", 10),
                    ("COLLISION_MULTIPLE", 10), ("STOP", 10)]
        self.assertEqual([p["reason"] for p in presses], [e[0] for e in expected])
        for press, (reason, delay) in zip(presses, expected):
            if reason is None:
                continue
            with self.subTest(press["step"], input=press["input"]):
                self.assertGreaterEqual(press["delay_min_ms"], delay - 1)
                self.assertLessEqual(press["delay_min_ms"], delay + 25)
                self.assertTrue(press["latched"])

    def test_c_m2_imu(self):
        imu = self.boot()["imu"]
        self.assertAlmostEqual(imu["rate_hz"], 100.0, delta=3.0)
        self.assertAlmostEqual(imu["rest"]["a_z"], xc.GRAVITY, delta=0.05)
        tilts = [e for e in imu["tilt_episodes"] if e["max_tilt"] > 30]
        pitch, roll, tilt45 = tilts[:3]
        self.assertAlmostEqual(abs(pitch["pitch_at_max"]), 38, delta=1.0)
        self.assertAlmostEqual(roll["roll_at_max"], 38, delta=1.0)
        for e in (pitch, roll):
            self.assertIsNone(e["checks"]["driver"]["latched_at"])
            self.assertAlmostEqual(e["checks"]["safety"]["after_crossing_s"], 1.0, delta=0.03)
        self.assertAlmostEqual(tilt45["checks"]["driver"]["after_crossing_s"], 0.5, delta=0.03)
        turns = [e for e in imu["yaw_episodes"] if e["seconds"] > 2.5]
        self.assertEqual(turns[0]["sign"], "+")
        self.assertAlmostEqual(turns[0]["angle_deg"], 86, delta=10)

    def test_d_m3_speed_per_duty(self):
        drive = self.boot()["drive"]
        straight = [g for g in drive["steady"] if g["w"] == 0 and g["v"] > 0]
        self.assertGreaterEqual(len(straight), 6)
        for g in straight:
            with self.subTest(v=g["v"]):
                self.assertAlmostEqual(g["per_duty_left"], FULL_DUTY_SPEED, delta=0.05)
                self.assertAlmostEqual(g["per_duty_right"], FULL_DUTY_SPEED, delta=0.05)
        self.assertTrue(all(abs(g["v"]) <= 0.3 + 1e-6 for g in drive["steady"]))

    def test_e_every_start_has_a_fresh_command(self):
        for run in self.boot()["drive"]["runs"]:
            if "wheel by hand" in (run.get("step") or ""):
                continue
            with self.subTest(run.get("step"), start=run["start"]):
                self.assertTrue(run["fresh_command"])

    def test_f_m4_wheels_stand_one_second_after_the_host(self):
        runs = self.of_step(self.boot()["drive"]["runs"], "M4")
        lost = [r for r in runs if "frozen" in r["step"] or "killed" in r["step"]]
        self.assertEqual(len(lost), 2)
        for r in lost:
            with self.subTest(r["step"]):
                # The board counts its 1.0 s from the host's last emergency input, then the wheels run down (NX60i
                # 0.07-0.12 s); a tick interval is 40 ms.
                self.assertGreater(r["since_emergency_input_s"], 0.98)
                self.assertLess(r["since_emergency_input_s"], 1.2)
                self.assertGreater(r["host_silent_s"], 0.7)
                self.assertLess(r["host_silent_s"], 1.2)
        # The inner network down: the script commands until the driver reports the loss; the capture sees nothing
        # meanwhile, the counters tell how long the wheels turned after it went down (board 1.0 s, run-down).
        gap, = [g for g in self.boot()["drive"]["gaps"] if g["turning_s"] is not None]
        self.assertGreater(gap["turning_s"], 0.6)
        self.assertLess(gap["turning_s"], 1.3)
        self.assertIn("[M4] inner network down", self.output)
        ended, = [r for r in runs if "SIGINT" in r["step"]]
        self.assertLess(ended["since_zero_command_s"], 0.2)

    def test_g_m5_reasons_stop_the_wheels(self):
        runs = [r for r in self.of_step(self.boot()["drive"]["runs"], "M5") if r["reason"]]
        self.assertEqual([r["reason"].split("|")[0] for r in runs],
                         ["STOP", "LIFT", "LIFT_MULTIPLE", "COLLISION_MULTIPLE", "COLLISION", "HIGH_LEVEL"])
        for r in runs:
            with self.subTest(r["step"]):
                # The wheels run down (NX60i 0.07-0.12 s); the first interval without a tick ends up to 40 ms later.
                self.assertLess(r["since_reason_s"], 0.18)
        lift = runs[1]
        self.assertGreater(lift["seconds"], 2.5)

    def test_h_m6_watchdog_and_guards(self):
        runs = self.of_step(self.boot()["drive"]["runs"], "M6")
        quiet = [r for r in runs if "command stops" in r["step"]][0]
        self.assertEqual(quiet["zeros_after_last_command"], 3)
        # The driver's watchdog (0.5 s), the run-down and the stop within one tick interval.
        self.assertGreater(quiet["since_script_command_s"], 0.45)
        self.assertLess(quiet["since_script_command_s"], 0.8)
        by_hand = [r for r in runs if "wheel by hand" in r["step"]]
        self.assertTrue(by_hand and not by_hand[-1]["fresh_command"])
        self.assertNotIn("Finding", self.output)

    def test_i_m7_blade(self):
        runs = self.of_step(self.boot()["blade"]["runs"], "M7")
        self.assertEqual(len(runs), 7)
        for r in runs:
            with self.subTest(r["step"]):
                self.assertTrue(r["fresh_setpoint"])
                self.assertIsNotNone(r["zero_at"])
        off, lease, stop, lift, bumper, frozen, killed = runs
        # As on the NX60i without the disc: 0.5 s until the controller follows, then 2000 rpm/s from 2800; the
        # rpm comes every 0.5 s.
        self.assertGreater(off["since_zero_command_s"], 1.4)
        self.assertLess(off["since_zero_command_s"], 2.5)
        for r, reason in ((stop, "STOP"), (lift, "LIFT"), (bumper, "COLLISION_MULTIPLE")):
            self.assertTrue(r["reason"].startswith(reason))
            self.assertLess(r["since_reason_s"], 2.5)
        for r in (frozen, killed):
            self.assertLess(r["host_silent_s"], 3.5)
            self.assertIn("TIMEOUT_HIGH_LEVEL", r["reason"])

    def test_j_m8_ground(self):
        runs = self.of_step(self.boot()["drive"]["runs"], "M8")
        stop = [r for r in runs if "STOP" in r["step"]][0]
        self.assertTrue(stop["reason"].startswith("STOP"))
        killed = [r for r in runs if "killed" in r["step"]][0]
        self.assertGreater(killed["since_emergency_input_s"], 0.98)
        self.assertLess(killed["since_emergency_input_s"], 1.25)
        backward = [r for r in runs if "backward" in r["step"]][0]
        self.assertTrue(backward["reason"].startswith("COLLISION"))
        # The driver keeps 0.3 m/s: the fast runs wait for raised limits.
        self.assertIn("fast runs: the driver allows 0.30 m/s", self.output)
        self.assertFalse([r for r in runs if "fast" in (r["step"] or "")])
        stopping = [json.loads(line) for line in open(self.recording) if '"kind": "stopping"' in line]
        self.assertEqual(len(stopping), 3)  # STOP, no command, backward; not the killed driver
        self.assertTrue(all(0.0 <= s["odometry_m"] < 0.3 for s in stopping), stopping)
        # Without a command the wheels go on for the driver's 0.5 s at 0.14 m/s (duty 0.3 at 0.47 m/s per duty).
        quiet, = [s for s in stopping if "no command" in s["text"]]
        self.assertGreater(quiet["odometry_m"], 0.03)

    def test_k_recording_limit(self):
        record = os.path.join(WORK, "small")
        code, output = measure("--steps", "M1,M2", "--max-mb", "0.08", "--record-dir", record, timeout=300)
        path, = glob.glob(os.path.join(record, "measure-*.jsonl"))
        self.assertLessEqual(os.path.getsize(path), 0.08 * 1024 * 1024 + 200)
        with open(path) as f:
            lines = f.read().splitlines()
        self.assertEqual(json.loads(lines[-1])["kind"], "limit")
        self.assertIn("(limit reached)", output)
        self.assertIn("reached its limit", output)
        # The step that hit the limit ends, no further one starts: M1 and M2 have 14 steps.
        self.assertLess(output.count("[y] run"), 4)

    def test_l_time_limit(self):
        record = os.path.join(WORK, "short")
        code, output = measure("--steps", "M1", "--max-minutes", "0.1", "--record-dir", record, timeout=300)
        self.assertIn("reached its limit", output)
        self.assertLess(output.count("[y] run"), 4)

    def test_m_capture_limit_of_the_fake(self):
        path = os.path.join(WORK, "limited.pcap")
        fake = subprocess.Popen([FAKE, "--services", SERVICES, "--group-port", str(GROUP_PORT + 1), "--pcap", path,
                                 "--pcap-max-mb", "0.005"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(4.0)
        finally:
            fake.terminate()
            fake.wait(5)
        self.assertLessEqual(os.path.getsize(path), 0.005 * 1024 * 1024 + 1600)
        self.assertFalse(xc.load_capture(path, SERVICES).stats.get("cut_at_end"))

    def test_n_abort_key(self):
        record = os.path.join(WORK, "abort")
        abort = os.path.join(WORK, "abort-now")
        log = open(os.path.join(WORK, "abort.log"), "w")
        run = subprocess.Popen([sys.executable, MEASURE, "--yes", "--fake-flags", FLAGS, "--steps", "M3",
                                "--record-dir", record, "--abort-file", abort],
                               stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        time.sleep(8.0)
        open(abort, "w").close()
        code = run.wait(30)
        log.close()
        os.remove(abort)
        with open(log.name) as f:
            output = f.read()
        self.assertIn("ABORT", output)
        self.assertNotEqual(code, 0)
        path, = glob.glob(os.path.join(record, "measure-*.jsonl"))
        with open(path) as f:
            entries = [json.loads(line) for line in f]
        abort_at = [e["t"] for e in entries if e["kind"] == "abort"][0]
        after = [e for e in entries if e["t"] > abort_at]
        self.assertEqual([(e["v"], e["w"]) for e in after if e["kind"] == "cmd"][:3], [(0.0, 0.0)] * 3)
        self.assertTrue([e for e in after if e["kind"] == "latch" and e["success"]])
        self.assertTrue([e for e in after if e["kind"] == "status" and e["emergency"]])
