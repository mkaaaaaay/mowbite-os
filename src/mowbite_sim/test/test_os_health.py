"""mowbite_health (HEALTHCHECK of the image) against the whole stack of sim.launch.py on a private mosquitto: healthy
once robot_state/json comes, unhealthy with the reason once the broker is gone. tron.launch.py has prepared data_dir
for the image and holds its lock. With MOWBITE_JOURNAL_DIR the journal of the stack carries nothing of a decoy NTRIP
access that stood in data_dir while it ran.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_os_health.py
"""
import fcntl
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

MQTT_PORT = 18841
WORK = tempfile.mkdtemp(prefix="mowbite_health_")
DATA = os.path.join(WORK, "data")
BROKER_CONF = os.path.join(WORK, "mosquitto.conf")
HALT = os.path.join(WORK, "halt")
HEALTH = os.path.join(get_package_prefix("mowbite_bridge"), "lib", "mowbite_bridge", "mowbite_health")
# The launch runs in this process and reads it from here.
os.environ["MOWBITE_HALT_FILE"] = HALT
JOURNAL = os.environ.get("MOWBITE_JOURNAL_DIR", "")
DECOY_USER = "decoy.user.of.the.journal"
DECOY_PASSWORD = "decoy.pw#of.the.journal"


def generate_test_description():
    with open(BROKER_CONF, "w") as f:
        f.write(f"listener {MQTT_PORT} 127.0.0.1\nallow_anonymous true\n")
    mosquitto = ExecuteProcess(cmd=["mosquitto", "-c", BROKER_CONF], output="screen")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": os.path.join(WORK, "state.json"),
                          "flags": os.path.join(WORK, "flags.json"), "log": os.path.join(WORK, "rx.jsonl"),
                          "data_dir": DATA, "mqtt_port": str(MQTT_PORT)}.items())
    return launch.LaunchDescription([mosquitto, TimerAction(period=1.0, actions=[sim]),
                                     launch_testing.actions.ReadyToTest()]), {}


def health(port=MQTT_PORT):
    env = dict(os.environ, MOWBITE_MQTT_PORT=str(port), MOWBITE_HALT_FILE=HALT)
    started = time.monotonic()
    done = subprocess.run([HEALTH], env=env, capture_output=True, text=True, timeout=15)
    return done.returncode, done.stdout.strip(), time.monotonic() - started


class TestHealth(unittest.TestCase):
    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_healthy_once_the_logic_reports(self):
        deadline = time.monotonic() + 90
        result = None
        while time.monotonic() < deadline:
            result = health()
            if result[0] == 0:
                break
            time.sleep(2)
        code, line, took = result
        self.assertEqual(code, 0, line)
        self.assertRegex(line, r"^OK [A-Z_]+$")
        # Within the 10 s the HEALTHCHECK gives it.
        self.assertLess(took, 6.0)
        self.assertFalse(os.path.exists(HALT))

    def test_02_data_dir_has_the_schema_of_the_image_and_is_locked(self):
        with open(os.path.join(get_package_share_directory("mowbite_bridge"), "data_schema.json")) as f:
            schema = json.load(f)["schema"]
        with open(os.path.join(DATA, "data_schema.json")) as f:
            self.assertEqual(json.load(f)["schema"], schema)
        fd = os.open(os.path.join(DATA, ".lock"), os.O_RDWR)
        try:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            os.close(fd)

    def test_03_a_wrong_port_is_unhealthy(self):
        code, line, _ = health(MQTT_PORT + 100)
        self.assertEqual(code, 1)
        self.assertTrue(line.startswith(f"MQTT_UNREACHABLE 127.0.0.1:{MQTT_PORT + 100}: "), line)
        env = dict(os.environ, MOWBITE_MQTT_PORT="nope", MOWBITE_HALT_FILE=HALT)
        done = subprocess.run([HEALTH], env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(done.returncode, 1)
        self.assertEqual(done.stdout.strip(), "BAD_CONFIG MOWBITE_MQTT_PORT is not a port: nope")

    def test_04_unhealthy_once_the_broker_is_gone(self):
        pids = subprocess.run(["pgrep", "-f", f"mosquitto -c {BROKER_CONF}"], capture_output=True,
                              text=True).stdout.split()
        self.assertTrue(pids)
        for pid in pids:
            os.kill(int(pid), signal.SIGKILL)
        time.sleep(1.0)
        code, line, took = health()
        self.assertEqual(code, 1)
        self.assertTrue(line.startswith(f"MQTT_UNREACHABLE 127.0.0.1:{MQTT_PORT}: "), line)
        self.assertLess(took, 6.0)

    def test_05_a_decoy_ntrip_access_is_read(self):
        path = os.path.join(DATA, "ntrip.yaml")
        with open(path, "w") as f:
            f.write(f"host: 127.0.0.1\nport: 9\nmountpoint: DECOY\nuser: '{DECOY_USER}'\n"
                    f"password: '{DECOY_PASSWORD}'\nversion: 2\n")
        os.chmod(path, 0o600)
        # mowbite_ntrip looks at the file every 5 s.
        time.sleep(8.0)


@launch_testing.post_shutdown_test()
class TestJournal(unittest.TestCase):
    def test_no_segment_carries_the_ntrip_access(self):
        if not JOURNAL:
            self.skipTest("no MOWBITE_JOURNAL_DIR")
        raw = subprocess.run(["mowbite_replay", "cat", JOURNAL], capture_output=True, check=True, timeout=120).stdout
        self.assertGreater(len(raw), 0)
        self.assertNotIn(DECOY_PASSWORD.encode(), raw)
        self.assertNotIn(DECOY_USER.encode(), raw)
