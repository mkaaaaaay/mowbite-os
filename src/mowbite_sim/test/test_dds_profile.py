"""DDS of MowBite OS stays on the loopback (fastdds_loopback.xml of mowbite_bringup, as in every image): the whole stack
of sim.launch.py and xcore_driver against fake_xcore run under SYSTEM_DEFAULT with the profile. tron_mcu, the container,
the planner and xcore_driver map no shared memory of Fast DDS, and every UDP socket of theirs lies on 127.0.0.0/8, but
for the board sockets of xcore_driver. Then tron.launch.py and measure.launch.py on their own with a profile that is not
there: one MOWBITE_OS_HALT line with reason DDS_PROFILE, no node, the launch ends on SIGINT.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dds_profile.py
"""
import os
import random
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.events.process import ProcessStarted
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import HardwareStatus

# The battery voltages of the platform file, which nx60i.launch.py hands to the driver (stack.py of mowbite_bringup).
sys.path.insert(0, os.path.join(get_package_share_directory("mowbite_bringup"), "launch"))
import stack  # noqa: E402

BATTERY = stack.battery_voltages([os.path.join(get_package_share_directory("mowbite_bringup"), "config",
                                               "nx60i.yaml")])

WORK = tempfile.mkdtemp(prefix="mowbite_dds_")
PROFILE = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "fastdds_loopback.xml")
# As in the images: the launch, its nodes and this probe all on the profile.
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = PROFILE
GROUP_PORT = random.randint(42000, 48000)
BIND_IP = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
GROUP = "233.255.255.0"
# The processes to look at, by a part of their name in the launch.
PROCESSES = ("tron_mcu_node", "component_container_isolated", "mowbite_planner_os_node", "xcore_driver_node")


def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments={"link": os.path.join(WORK, "mcu"), "gnss_link": os.path.join(WORK, "gnss"),
                          "imu_link": os.path.join(WORK, "imu"), "state": os.path.join(WORK, "state.json"),
                          "flags": os.path.join(WORK, "flags.json"), "log": os.path.join(WORK, "rx.jsonl"),
                          "data_dir": os.path.join(WORK, "data"), "mqtt_port": "18851"}.items())
    sim_share = get_package_share_directory("mowbite_sim")
    fake_xcore = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_xcore"),
             "--services", os.path.join(sim_share, "config", "xcore_nx60i_services.json"),
             "--group-port", str(GROUP_PORT), "--flags", os.path.join(WORK, "xflags.json"),
             "--log", os.path.join(WORK, "xrx.jsonl"), "--log-max-mb", "20",
             "--state", os.path.join(WORK, "xstate.json")],
        output="screen")
    config = os.path.join(get_package_share_directory("xcore_driver"), "config", "nx60i.yaml")
    with open(config) as f:
        params = yaml.safe_load(f)["xcore_driver"]["ros__parameters"]
    # In a namespace of its own, beside tron_mcu; the file is keyed by the bare node name, so its values go in as a dict.
    driver = Node(package="xcore_driver", executable="xcore_driver_node", namespace="xcore", output="screen",
                  parameters=[params, BATTERY,
                              {"interface": "lo", "bind_ip": BIND_IP, "announcement_port": GROUP_PORT}])
    return launch.LaunchDescription([
        sim, fake_xcore,
        TimerAction(period=0.5, actions=[driver]),
        launch_testing.actions.ReadyToTest(),
    ]), {}


def pids(proc_info):
    found = {}
    for action in proc_info.processes():
        event = proc_info[action]
        name = action.process_details["name"]
        for wanted in PROCESSES:
            if wanted in name and isinstance(event, ProcessStarted):
                found[wanted] = event.pid
    return found


def socket_inodes(pid):
    out = set()
    for fd in os.listdir(f"/proc/{pid}/fd"):
        try:
            target = os.readlink(f"/proc/{pid}/fd/{fd}")
        except OSError:
            continue
        if target.startswith("socket:["):
            out.add(target[8:-1])
    return out


def udp_sockets(pid):
    """(family, local address, port) of every UDP socket of the process."""
    own = socket_inodes(pid)
    found = []
    for family, path in ((4, "/proc/net/udp"), (6, "/proc/net/udp6")):
        with open(path) as f:
            for line in f.read().split("\n")[1:]:
                parts = line.split()
                if len(parts) < 10 or parts[9] not in own:
                    continue
                address, port = parts[1].split(":")
                if family == 4:
                    address = socket.inet_ntoa(struct.pack("<I", int(address, 16)))
                found.append((family, address, int(port, 16)))
    return found


def shared_memory(pid):
    """Segments of Fast DDS in /dev/shm the process maps (not its typesupport libraries)."""
    with open(f"/proc/{pid}/maps") as f:
        paths = {line.split()[-1] for line in f}
    return sorted(p for p in paths if p.startswith("/dev/shm/") and ("fastrtps" in p or "fast_datasharing" in p))


def run_until_halt(cmd, seconds=30):
    """Starts cmd in a process group of its own with the profile missing; its output up to the halt line, then SIGINT."""
    env = dict(os.environ, FASTRTPS_DEFAULT_PROFILES_FILE=os.path.join(WORK, "missing.xml"),
               MOWBITE_HALT_FILE=os.path.join(WORK, "halt"))
    process = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               start_new_session=True)
    lines = []
    deadline = time.monotonic() + seconds
    os.set_blocking(process.stdout.fileno(), False)
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if line:
            lines.append(line.rstrip())
            if "MOWBITE_OS_HALT" in line:
                break
        else:
            time.sleep(0.05)
    # No node may follow the halt.
    time.sleep(2.0)
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    rest = process.stdout.read() or ""
    lines.extend(rest.splitlines())
    return process.returncode, lines


class TestDdsProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dds_profile_probe")
        cls.status = {}
        cls.node.create_subscription(HardwareStatus, "hw/status", lambda m: cls.status.update(tron=m), 10)
        cls.node.create_subscription(HardwareStatus, "xcore/hw/status", lambda m: cls.status.update(xcore=m), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_loopback_only(self, proc_info):
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not ({"tron", "xcore"} <= set(self.status)
                                                   and set(PROCESSES) <= set(pids(proc_info))):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(set(self.status), {"tron", "xcore"}, "hw/status of both drivers over the profile")
        found = pids(proc_info)
        self.assertEqual(set(found), set(PROCESSES))
        # The planner and the probe have found each other by now; let every participant open what it opens.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for name, pid in found.items():
            with self.subTest(process=name):
                self.assertEqual(shared_memory(pid), [], f"{name} maps shared memory of Fast DDS")
                sockets = udp_sockets(pid)
                self.assertTrue(sockets, f"{name} has no UDP socket")
                for family, address, port in sockets:
                    board = name == "xcore_driver_node" and (address == BIND_IP or
                                                             (address == GROUP and port == GROUP_PORT))
                    self.assertTrue(board or (family == 4 and address.startswith("127.")),
                                    f"{name}: UDP socket at {address}:{port} (IPv{family})")

    def test_2_tron_launch_halts_without_profile(self, proc_info):
        code, lines = run_until_halt(["ros2", "launch", "mowbite_bringup", "tron.launch.py",
                                      f"data_dir:={os.path.join(WORK, 'halt_data')}"])
        output = "\n".join(lines)
        halts = [line for line in lines if "MOWBITE_OS_HALT" in line]
        self.assertEqual(len(halts), 1, output)
        self.assertIn("MOWBITE_OS_HALT reason=DDS_PROFILE data_schema=- ", halts[0])
        self.assertIn("missing.xml", halts[0])
        self.assertNotIn("process started", output)
        with open(os.path.join(WORK, "halt")) as f:
            self.assertEqual(f.read().strip(), halts[0][halts[0].index("MOWBITE_OS_HALT"):])
        self.assertEqual(code, 0, output)

    def test_3_measure_launch_halts_without_profile(self, proc_info):
        code, lines = run_until_halt(["ros2", "launch", "xcore_driver", "measure.launch.py"])
        output = "\n".join(lines)
        halts = [line for line in lines if "MOWBITE_OS_HALT" in line]
        self.assertEqual(len(halts), 1, output)
        self.assertIn("MOWBITE_OS_HALT reason=DDS_PROFILE", halts[0])
        self.assertNotIn("process started", output)
        self.assertEqual(code, 0, output)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_work(self):
        shutil.rmtree(WORK, ignore_errors=True)
