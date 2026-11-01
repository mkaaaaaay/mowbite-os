"""Safety inputs of the Tron drivers against fake_tron on ptys: a gap in the IMU latches IMU_TIMEOUT in tron_mcu, a blade
that still turns after off latches MOWER_NOT_STOPPED and is sent off again, and tron_gnss counts RTK fixed on old or
missing corrections as float. mowbite_logic logs the reason names in the line the app reads.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_safety_inputs.py
"""
import json
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from mowbite_msgs.msg import AbsolutePose, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

WORK = tempfile.mkdtemp(prefix="mowbite_safety_")
LINKS = {name: os.path.join(WORK, name) for name in ("mcu", "gnss", "imu")}
FLAGS = os.path.join(WORK, "flags.json")
LOG = os.path.join(WORK, "rx.jsonl")
CUTTER = 11
BLADE_STOP_TIME = 2.0
IMU_GAP = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_IMU_TIMEOUT
NOT_STOPPED = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_MOWER_NOT_STOPPED
FIXED = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FIXED
FLOAT = AbsolutePose.FLAG_GPS_RTK | AbsolutePose.FLAG_GPS_RTK_FLOAT


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = os.path.join(get_package_share_directory("mowbite_bringup"), "config", "geometry", "tron.yaml")


def generate_test_description():
    fake_tron = ExecuteProcess(
        cmd=[os.path.join(FindPackagePrefix("mowbite_sim").find("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"),
             "--link", LINKS["mcu"], "--gnss-link", LINKS["gnss"], "--imu-link", LINKS["imu"],
             "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"],
        output="screen")
    imu = Node(package="tron_imu", executable="tron_imu_node", output="screen", parameters=[{"port": LINKS["imu"]}])
    driver = Node(package="tron_mcu", executable="tron_mcu_node", output="screen",
                  parameters=[{"port": LINKS["mcu"], "send_imu": True, "imu_timeout": 0.5, "allow_blade": True,
                               "blade_lease_max": 2.0, "blade_stop_time": BLADE_STOP_TIME}])
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0}])
    logic = Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen", parameters=[GEOMETRY])
    # The ptys have to exist before the drivers look for them.
    return launch.LaunchDescription([
        fake_tron,
        TimerAction(period=1.0, actions=[imu, driver, gnss, logic, launch_testing.actions.ReadyToTest()]),
    ]), {}


def received(since, module=None):
    """Sub packets fake_tron got from the driver since sim time `since`."""
    out = []
    with open(LOG) as f:
        for line in f:
            entry = json.loads(line)
            if entry.get("type") == 0 and entry["t"] >= since and (module is None or entry["mod"] == module):
                out.append(entry)
    return out


def sim_time():
    with open(LOG) as f:
        lines = f.readlines()
    return json.loads(lines[-1])["t"] if lines else 0.0


def set_flags(**flags):
    with open(FLAGS + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(FLAGS + ".tmp", FLAGS)


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.pose = None
        self.logs = []
        self.node = rclpy.create_node("safety_inputs_probe")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.node.create_subscription(AbsolutePose, "gps/absolute_pose", self._pose, 10)
        # Like the rosout publishers, so lines logged before the match with a node still come in.
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Log, "/rosout", self._log, rosout_qos)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status = msg

    def _pose(self, msg):
        with self.lock:
            self.pose = msg

    def _log(self, msg):
        with self.lock:
            self.logs.append(msg.msg)

    def wait(self, pred, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                value = pred(self)
            if value:
                return value
            time.sleep(0.02)
        return None

    def logged(self, text, timeout=5.0):
        return self.wait(lambda p: any(text in line for line in p.logs), timeout)

    def call(self, client, request, timeout=5.0):
        assert client.wait_for_service(timeout_sec=timeout), "service missing"
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.01)
        return future.result()

    def set_blade(self, on, lease_s=0.0):
        return self.call(self.blade, SetBlade.Request(on=on, lease_s=lease_s))

    def release(self):
        self.call(self.emergency, SetEmergency.Request(latch=False, reason="safety inputs test"))

    def stop(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


class TestSafetyInputs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        # Free once telemetry and the first IMU samples arrived.
        ready = cls.probe.wait(lambda p: p.status is not None and p.status.telemetry_ok and not p.status.emergency, 20)
        assert ready, "driver never ready"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_imu_gap_latches_until_released(self):
        set_flags(imu_off=1)
        s = self.probe.wait(lambda p: p.status.emergency and p.status.emergency_reason == IMU_GAP and p.status, 3)
        self.assertTrue(s, f"no IMU_TIMEOUT: {self.probe.status.emergency_reason}")
        # Without the IMU a release does nothing.
        self.probe.release()
        time.sleep(1.5)
        self.assertTrue(self.probe.status.emergency)
        set_flags()
        time.sleep(1.0)
        self.assertTrue(self.probe.status.emergency, "the gap did not stay latched")
        self.assertEqual(self.probe.status.emergency_reason, IMU_GAP)
        self.probe.release()
        self.assertTrue(self.probe.wait(lambda p: not p.status.emergency, 3), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)
        self.assertTrue(self.probe.logged("Emergency reason changed from: '' to: 'LATCH, IMU_TIMEOUT'"),
                        [line for line in self.probe.logs if "Emergency" in line])

    def test_2_blade_still_turning_latches_and_is_sent_off_again(self):
        set_flags(cutter_ignores_off=1)
        self.assertTrue(self.probe.set_blade(True, lease_s=2.0).success)
        turning = self.probe.wait(lambda p: p.status.blade_motor.rpm > 1000, 3)
        self.assertTrue(turning, "blade never turned")
        t0 = sim_time()
        self.assertTrue(self.probe.set_blade(False).success)
        s = self.probe.wait(lambda p: p.status.emergency and p.status.emergency_reason == NOT_STOPPED and p.status,
                            BLADE_STOP_TIME + 3)
        self.assertTrue(s, f"no MOWER_NOT_STOPPED: {self.probe.status.emergency_reason}")
        time.sleep(0.5)
        offs = [e["t"] for e in received(t0, CUTTER) if e["hex"][:2] == "00"]
        self.assertGreaterEqual(len(offs), 6, str(offs))
        # The first burst at the request, the next after the run-down time.
        self.assertGreaterEqual(offs[3] - offs[0], BLADE_STOP_TIME - 0.1, str(offs))
        # While the blade turns a release does nothing.
        self.probe.release()
        time.sleep(1.5)
        self.assertTrue(self.probe.status.emergency)
        # The MCU obeys again: the next off frames stop the blade, then a release works.
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: p.status.blade_motor.rpm == 0, 4), "blade did not stop")
        time.sleep(1.2)
        self.probe.release()
        self.assertTrue(self.probe.wait(lambda p: not p.status.emergency, 3), "release failed")
        self.assertTrue(self.probe.logged("to: 'LATCH, MOWER_NOT_STOPPED'"),
                        [line for line in self.probe.logs if "Emergency" in line])

    def test_3_old_or_missing_corrections_count_as_float(self):
        self.assertTrue(self.probe.wait(lambda p: p.pose is not None and p.pose.flags == FIXED, 5), "never fixed")
        set_flags(gnss_age=45.0)
        p = self.probe.wait(lambda p: p.pose.flags == FLOAT and p.pose, 3)
        self.assertTrue(p, "old corrections stayed fixed")
        self.assertAlmostEqual(p.position_accuracy, 0.3, places=5)
        self.assertTrue(self.probe.logged("RTK fixed on corrections 45.0 s old (limit 30 s): counted as float"))
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: p.pose.flags == FIXED, 3), "fresh corrections did not fix again")
        set_flags(gnss_age=None)
        self.assertTrue(self.probe.wait(lambda p: p.pose.flags == FLOAT, 3), "missing age stayed fixed")
        self.assertTrue(self.probe.logged("RTK fixed without the age of its corrections"))
        set_flags()
        self.assertTrue(self.probe.wait(lambda p: p.pose.flags == FIXED, 3))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
