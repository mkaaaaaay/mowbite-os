"""V1 of the fast runs of M8 against fake_xcore, the driver in the raised start of measure.launch.py (speed_limit:=0.5
max_duty:=1.0). Measurement build: the driver reports 0.5 m/s and keeps measurement_speed_limit read-only; measure.py
takes nothing but M8 --fast, runs 0.4 m/s with STOP, then the top speed (duty 1.0 = 0.47 m/s at the fake's 0.47 m/s
per duty) with STOP, without a command and with the driver killed, each within its limits; the capture shows no duty
above 1.0 and the board's 1.0 s for the killed driver; a wheel above 0.6 m/s aborts. Any other build: the driver does
not start in the raised start, measure.py finds no capabilities.

Run in a sourced workspace: launch_test tools/yf/test/test_rehearsal_fast.py
"""
import glob
import json
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareCapabilities
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import measure as m  # noqa: E402
import xcore_capture as xc  # noqa: E402

MEASURE = os.path.join(HERE, "..", "measure.py")
WORK = tempfile.mkdtemp(prefix="mowbite_rehearsal_fast_")
FLAGS = os.path.join(WORK, "flags.json")
PCAP = os.path.join(WORK, "inner.pcap")
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
SERVICES = os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "config", "xcore_nx60i_services.json")
FAKE = os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore")
FULL_DUTY_SPEED = 0.47
REFUSED = "is for the measurement image only"


def generate_test_description():
    with open(FLAGS, "w") as f:
        json.dump({}, f)
    fake_xcore = ExecuteProcess(
        cmd=[FAKE, "--services", SERVICES, "--group-port", str(GROUP_PORT), "--flags", FLAGS, "--pcap", PCAP,
             "--pcap-max-mb", "60", "--full-duty-speed", str(FULL_DUTY_SPEED)],
        output="screen")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "launch",
                                                   "measure.launch.py")),
        launch_arguments={"interface": "lo", "bind_ip": BIND_IP, "announcement_port": str(GROUP_PORT),
                          "respawn": "true", "respawn_delay": "1.0", "speed_limit": "0.5", "max_duty": "1.0"}.items())
    return launch.LaunchDescription([
        fake_xcore,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ])


def measure(*args, timeout, record):
    log = open(os.path.join(WORK, f"measure-{time.monotonic():.0f}.log"), "w")
    done = subprocess.run([sys.executable, MEASURE, "--yes", "--fake-flags", FLAGS, "--restart-wait", "30",
                           "--record-dir", os.path.join(WORK, record), *args],
                          stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    log.close()
    with open(log.name) as f:
        return done.returncode, f.read()


def entries(record, kind):
    path, = glob.glob(os.path.join(WORK, record, "measure-*.jsonl"))
    with open(path) as f:
        return [e for e in map(json.loads, f) if e["kind"] == kind]


def capabilities_limit(node, seconds):
    got = []
    latched = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
    node.create_subscription(HardwareCapabilities, "hw/capabilities", got.append, latched)
    end = time.monotonic() + seconds
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got[-1].max_linear_speed if got else None


def call(node, client, request):
    if not client.wait_for_service(timeout_sec=5.0):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


def overspeed_later(record, flags_path, after):
    """Once the first fast run is marked, both wheels of the fake report 0.65 m/s `after` s later."""
    def run():
        end = time.monotonic() + 120
        while time.monotonic() < end:
            paths = glob.glob(os.path.join(WORK, record, "measure-*.jsonl"))
            if paths:
                with open(paths[0]) as f:
                    if any('"kind": "mark"' in line and "fast" in line for line in f):
                        time.sleep(after)
                        with open(flags_path + ".tmp", "w") as out:
                            json.dump({"wheel_speed": 0.65}, out)
                        os.replace(flags_path + ".tmp", flags_path)
                        return
            time.sleep(0.05)
    watcher = threading.Thread(target=run, name="overspeed", daemon=True)
    watcher.start()
    return watcher


class TestRehearsalFast(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        time.sleep(5.0)  # claim and configuration
        rclpy.init()
        cls.node = rclpy.create_node("rehearsal_fast_probe")
        cls.limit = capabilities_limit(cls.node, 15.0)
        cls.measurement_build = cls.limit is not None
        if not cls.measurement_build:
            cls.code, cls.output = measure("--steps", "M8", "--ground", "--fast", timeout=120, record="refused")
            return
        cls.parameters = (
            call(cls.node, cls.node.create_client(GetParameters, "/xcore_driver/get_parameters"),
                 GetParameters.Request(names=["measurement_speed_limit", "max_linear_speed", "max_duty"])),
            call(cls.node, cls.node.create_client(SetParameters, "/xcore_driver/set_parameters"),
                 SetParameters.Request(parameters=[Parameter(
                     name="measurement_speed_limit",
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.6))])))
        cls.refused = [measure("--steps", "M3", timeout=120, record="m3"),
                       measure("--steps", "M8", "--ground", timeout=120, record="m8")]
        cls.code, cls.output = measure("--steps", "M8", "--ground", "--fast", "--repeats", "3", timeout=900,
                                       record="fast")
        print(cls.output)
        cap = xc.load_capture(PCAP, SERVICES)
        recording, = glob.glob(os.path.join(WORK, "fast", "measure-*.jsonl"))
        cls.report = xc.analyse(cap, (xc.NX60I_INPUTS, "test"), {"driver": (40.0, 0.5), "safety": (31.0, 1.0)},
                                15.0, 0.1, marks=xc.load_marks(recording))
        print(xc.text_report(cls.report))
        # Last, as it leaves the board latched: a wheel above 0.6 m/s in the first fast run.
        overspeed_later("guard", FLAGS, 1.0)
        cls.guard = measure("--steps", "M8", "--ground", "--fast", "--repeats", "1", timeout=300, record="guard")
        with open(FLAGS, "w") as f:
            json.dump({}, f)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        keep = os.environ.get("REHEARSAL_KEEP")
        if keep:
            shutil.copytree(WORK, keep, dirs_exist_ok=True)
        shutil.rmtree(WORK, ignore_errors=True)

    def measurement_only(self):
        if not self.measurement_build:
            self.skipTest("not the measurement build")

    def test_a_the_build_decides(self):
        if self.measurement_build:
            self.assertAlmostEqual(self.limit, 0.5, places=5)
            self.assertTrue(self.proc_output.waitFor("Measurement build: driving capped at 0.50 m/s", timeout=5))
        else:
            self.assertTrue(self.proc_output.waitFor(REFUSED, timeout=5))
            self.assertNotEqual(self.code, 0)
            self.assertIn("no hw/capabilities", self.output)

    def test_b_cap_is_read_only(self):
        self.measurement_only()
        got, set_ = self.parameters
        self.assertIsNotNone(got)
        self.assertEqual([round(v.double_value, 6) for v in got.values], [0.5, 0.5, 1.0])
        self.assertIsNotNone(set_)
        self.assertFalse(set_.results[0].successful)

    def test_c_raised_start_takes_nothing_else(self):
        self.measurement_only()
        for code, output in self.refused:
            self.assertNotEqual(code, 0)
            self.assertIn("the raised start is only for --steps M8 --ground --fast", output)
            self.assertNotIn("[y] run", output)

    def test_d_fast_runs_within_their_limits(self):
        self.measurement_only()
        self.assertEqual(self.code, 0, self.output)
        self.assertNotIn("ABORT", self.output)
        self.assertNotIn("Finding", self.output)
        # The runs at 0.3 m/s belong to the normal start.
        self.assertIn("M8 at 0.3 m/s: skipped, the driver runs in the raised start", self.output)
        stopping = entries("fast", "stopping")
        first = [s for s in stopping if s["text"].startswith("fast 0.40 m/s")]
        top_stop = [s for s in stopping if s["text"].startswith("fast 0.50 m/s straight, press STOP")]
        quiet = [s for s in stopping if "no command" in s["text"]]
        self.assertEqual((len(first), len(top_stop), len(quiet)), (3, 3, 3), stopping)
        for s in first:
            self.assertAlmostEqual(s["steady"], 0.4, delta=0.03)
        # 0.5 m/s is a duty of 1.06 at 0.47 m/s per duty: max_duty 1.0 holds it at 0.47 m/s.
        for s in top_stop + quiet:
            self.assertAlmostEqual(s["steady"], FULL_DUTY_SPEED, delta=0.03)
        for s in first + top_stop:
            self.assertLessEqual(s["stand_s"], m.FAST_STAND_S["stop"])
            self.assertLessEqual(s["odometry_m"], m.FAST_DISTANCE_M["stop"])
        for s in quiet:
            self.assertLessEqual(s["stand_s"], m.FAST_STAND_S["quiet"])
            self.assertLessEqual(s["odometry_m"], m.FAST_DISTANCE_M["quiet"])
            # The driver's 0.5 s at 0.47 m/s, then its stop.
            self.assertGreater(s["odometry_m"], 0.2)
        marks = [e["text"] for e in entries("fast", "mark")]
        self.assertEqual(len(marks), 12)
        self.assertTrue(all(t.startswith("fast ") for t in marks), marks)
        self.assertEqual(sum("driver killed" in t for t in marks), 3)

    def test_e_capture(self):
        self.measurement_only()
        boot, = self.report["boots"]
        drive = boot["drive"]
        self.assertTrue(drive["steady"])
        for g in drive["steady"]:
            with self.subTest(v=g["v"]):
                self.assertLessEqual(max(abs(g["duty_left"]), abs(g["duty_right"])), 1.0 + 1e-6)
                self.assertLessEqual(max(abs(g["speed_left"]), abs(g["speed_right"])), m.FAST_ABORT_SPEED)
        runs = [r for r in drive["runs"] if "fast" in (r.get("step") or "")]
        killed = [r for r in runs if "killed" in r["step"]]
        self.assertEqual(len(killed), 3)
        for r in killed:
            with self.subTest(r["step"]):
                self.assertGreater(r["since_emergency_input_s"], 0.98)
                self.assertLess(r["since_emergency_input_s"], 1.25)
        for r in runs:
            with self.subTest(r["step"], start=r["start"]):
                self.assertTrue(r["fresh_command"])

    def test_f_a_wheel_too_fast_aborts(self):
        self.measurement_only()
        code, output = self.guard
        self.assertNotEqual(code, 0)
        self.assertIn("ABORT", output)
        # The guard reads the driver's odometry, whole ticks: the fake's 0.65 m/s comes out at 0.64 to 0.67.
        found = re.search(r"a wheel at (\d+\.\d+) m/s", output)
        self.assertIsNotNone(found, output)
        self.assertGreater(float(found.group(1)), m.FAST_ABORT_SPEED)
        self.assertLessEqual(float(found.group(1)), 0.7)
        abort, = [e["t"] for e in entries("guard", "abort")]
        after = [e for e in entries("guard", "cmd") if e["t"] > abort]
        self.assertEqual([(e["v"], e["w"]) for e in after][:3], [(0.0, 0.0)] * 3)
        self.assertTrue([e for e in entries("guard", "latch") if e["t"] > abort and e["success"]])
        # The top speed never ran.
        self.assertFalse([e for e in entries("guard", "mark") if e["text"].startswith("fast 0.50")])
