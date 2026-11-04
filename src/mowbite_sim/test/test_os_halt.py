"""tron.launch.py halts when data_dir does not fit the image: no node starts, one MOWBITE_OS_HALT line, the launch stays
without nodes until SIGINT (no restart loop under restart: unless-stopped) and mowbite_health reports the line, even
with the broker up. Here inside sim.launch.py with data newer than the image and no backup to go back to; then
ros2 launch of tron.launch.py on its own for the same case and for a data_dir another OS holds. At the end every process
of the launch has ended, the broker and the simulation beside the halt included.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_os_halt.py
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
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource

MQTT_PORT = 18842
WORK = tempfile.mkdtemp(prefix="mowbite_halt_")
HEALTH = os.path.join(get_package_prefix("mowbite_bridge"), "lib", "mowbite_bridge", "mowbite_health")
with open(os.path.join(get_package_share_directory("mowbite_bridge"), "data_schema.json")) as f:
    IMAGE_SCHEMA = json.load(f)["schema"]
NEWER = IMAGE_SCHEMA + 1
OS_NODES = {"tron_mcu", "tron_gnss", "tron_imu", "mowbite_localization", "mowbite_logic", "mowbite_bridge",
            "mowbite_map", "mowbite_follower", "mowbite_ntrip", "mowbite_safety", "mowbite_planner"}
MAP = '{"areas": []}'


def newer_data(name):
    data = os.path.join(WORK, name)
    os.makedirs(data)
    with open(os.path.join(data, "data_schema.json"), "w") as f:
        json.dump({"schema": NEWER, "by": "test"}, f)
    with open(os.path.join(data, "map.json"), "w") as f:
        f.write(MAP)
    return data


DATA = newer_data("data")
HALT = os.path.join(WORK, "halt")
# The launch runs in this process and reads it from here.
os.environ["MOWBITE_HALT_FILE"] = HALT


def generate_test_description():
    conf = os.path.join(WORK, "mosquitto.conf")
    with open(conf, "w") as f:
        f.write(f"listener {MQTT_PORT} 127.0.0.1\nallow_anonymous true\n")
    mosquitto = ExecuteProcess(cmd=["mosquitto", "-c", conf], output="screen")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": os.path.join(WORK, "state.json"),
                          "flags": os.path.join(WORK, "flags.json"), "log": os.path.join(WORK, "rx.jsonl"),
                          "data_dir": DATA, "mqtt_port": str(MQTT_PORT)}.items())
    return launch.LaunchDescription([mosquitto, TimerAction(period=1.0, actions=[sim]),
                                     launch_testing.actions.ReadyToTest()]), {}


def wait_for(path, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if os.path.exists(path):
            with open(path) as f:
                return f.read()
        time.sleep(0.2)
    return None


def health(halt):
    env = dict(os.environ, MOWBITE_MQTT_PORT=str(MQTT_PORT), MOWBITE_HALT_FILE=halt)
    done = subprocess.run([HEALTH], env=env, capture_output=True, text=True, timeout=15)
    return done.returncode, done.stdout.strip()


def tron_launch(data_dir, halt):
    """ros2 launch of tron.launch.py as on the mower, in a process group of its own."""
    env = dict(os.environ, MOWBITE_HALT_FILE=halt, MOWBITE_MQTT_PORT=str(MQTT_PORT))
    return subprocess.Popen(["ros2", "launch", "mowbite_bringup", "tron.launch.py", f"data_dir:={data_dir}"],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def stop(process):
    os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=15)
        return process.returncode, output
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.communicate()
        return None, ""


class TestHalt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("halt_probe")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_one_halt_line_and_health_reports_it(self):
        text = wait_for(HALT, 30)
        self.assertIsNotNone(text, "no halt line")
        line = text.strip()
        self.assertTrue(line.startswith(f"MOWBITE_OS_HALT reason=SCHEMA_NEWER data_schema={NEWER} "
                                        f"image_schema={IMAGE_SCHEMA} "), line)
        self.assertEqual(len(text.splitlines()), 1)
        # The broker is up, yet the OS is not healthy.
        code, out = health(HALT)
        self.assertEqual(code, 1)
        self.assertEqual(out, line)

    def test_02_no_node_of_the_os_starts_and_the_data_stays(self):
        seen = set()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            seen |= {name for name, _ in self.node.get_node_names_and_namespaces()}
        self.assertFalse(seen & OS_NODES, seen)
        self.assertEqual(sorted(os.listdir(DATA)), [".lock", "data_schema.json", "map.json"])
        with open(os.path.join(DATA, "map.json")) as f:
            self.assertEqual(f.read(), MAP)
        with open(os.path.join(DATA, "data_schema.json")) as f:
            self.assertEqual(json.load(f), {"schema": NEWER, "by": "test"})

    def test_03_tron_launch_stays_without_nodes_until_sigint(self):
        data = newer_data("alone")
        halt = os.path.join(WORK, "halt_alone")
        process = tron_launch(data, halt)
        line = (wait_for(halt, 30) or "").strip()
        time.sleep(5.0)
        running = process.poll() is None
        code, output = stop(process)
        self.assertTrue(running, output)
        self.assertTrue(line.startswith("MOWBITE_OS_HALT reason=SCHEMA_NEWER"), output)
        self.assertEqual(output.count("MOWBITE_OS_HALT"), 1, output)
        self.assertEqual(code, 0, output)
        self.assertNotIn("tron_mcu_node", output)

    def test_04_a_data_dir_held_by_another_os_halts_after_the_wait(self):
        data = os.path.join(WORK, "held")
        os.makedirs(data)
        halt = os.path.join(WORK, "halt_held")
        fd = os.open(os.path.join(data, ".lock"), os.O_RDWR | os.O_CREAT, 0o600)
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            started = time.monotonic()
            process = tron_launch(data, halt)
            line = (wait_for(halt, 60) or "").strip()
            took = time.monotonic() - started
            code, output = stop(process)
        finally:
            os.close(fd)
        self.assertTrue(line.startswith(f"MOWBITE_OS_HALT reason=DATA_DIR_LOCKED data_schema=- "
                                        f"image_schema={IMAGE_SCHEMA} "), output)
        self.assertGreaterEqual(took, 29.0)
        self.assertEqual(code, 0, output)
        self.assertFalse(os.path.exists(os.path.join(data, "data_schema.json")))


@launch_testing.post_shutdown_test()
class TestHaltShutdown(unittest.TestCase):
    def test_every_process_of_the_launch_ended(self, proc_info):
        names = sorted(info.process_name for info in proc_info)
        self.assertTrue(any(name.startswith("sim_control") for name in names), names)
        running = sorted(info.process_name for info in proc_info if not isinstance(info, ProcessExited))
        self.assertEqual(running, [], names)
