"""Safety inputs of the Tron drivers against fake_tron on ptys: a gap in the IMU latches IMU_TIMEOUT in tron_mcu, a
blade that still turns after off latches MOWER_NOT_STOPPED and is sent off again, and tron_gnss counts RTK fixed on old
or missing corrections as float. mowbite_logic logs the reason names in the line the app reads.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_safety_inputs.py
"""
import functools
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import GnssFix, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency
from rcl_interfaces.msg import Log
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (DriverProbe, ROSOUT, TestShutdown, fake_tron, last_time, layers, sim_files,  # noqa: E402,F401
                        sim_links, sub_packets, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_safety_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
CUTTER = 11
BLADE_STOP_TIME = 2.0
IMU_GAP = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_IMU_TIMEOUT
NOT_STOPPED = HardwareStatus.REASON_LATCH | HardwareStatus.REASON_MOWER_NOT_STOPPED
FIXED = GnssFix.RTK_FIXED
FLOAT = GnssFix.RTK_FLOAT


# The dimensions of the platform, which the logic needs to start.
GEOMETRY = layers.geometry("tron")


def generate_test_description():
    imu = Node(package="tron_imu", executable="tron_imu_node", output="screen", parameters=[{"port": LINKS["imu"]}])
    driver = Node(package="tron_mcu", executable="tron_mcu_node", output="screen",
                  parameters=[{"port": LINKS["mcu"], "send_imu": True, "imu_timeout": 0.5, "allow_blade": True,
                               "blade_lease_max": 2.0, "blade_stop_time": BLADE_STOP_TIME}])
    gnss = Node(package="tron_gnss", executable="tron_gnss_node", output="screen",
                parameters=[{"port": LINKS["gnss"], "datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0}])
    logic = Node(package="mowbite_logic", executable="mowbite_logic_node", output="screen", parameters=[GEOMETRY])
    # The ptys have to exist before the drivers look for them.
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
        TimerAction(period=1.0, actions=[imu, driver, gnss, logic, launch_testing.actions.ReadyToTest()]),
    ]), {}


received = functools.partial(sub_packets, LOG)


sim_time = functools.partial(last_time, LOG)


def set_flags(**flags):
    write_json(FLAGS, flags)


class Probe(DriverProbe):
    emergency_reason = "safety inputs test"

    def __init__(self):
        super().__init__("safety_inputs_probe")
        self.status = None
        self.pose = None
        self.imu_seen = False
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(GnssFix, "gps/absolute_pose", self._set("pose"), 10)
        self.node.create_subscription(Imu, "imu/data", self._imu, qos_profile_sensor_data)
        self.node.create_subscription(Log, "/rosout", self._log, ROSOUT)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.start()

    def _imu(self, msg):
        with self.lock:
            self.imu_seen = True

    def logged(self, text, timeout=5.0):
        return super().logged(text, timeout)

    def release(self):
        self.set_emergency(False)


class TestSafetyInputs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        set_flags()
        cls.probe = Probe()
        # Free once telemetry and the first IMU samples arrived.
        ready = cls.probe.wait_for(
            lambda p: p.imu_seen and p.status is not None and p.status.telemetry_ok and not p.status.emergency, 20)
        assert ready, "driver never ready"
        # The IMU start time of the driver (2 s) has passed, so an IMU that never reached it shows as a block.
        time.sleep(2.5)
        assert cls.probe.wait_for(lambda p: not p.status.emergency, 5), "driver never got an IMU sample"

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_imu_gap_latches_until_released(self):
        set_flags(imu_off=1)
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status.emergency_reason == IMU_GAP and p.status, 3)
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
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.emergency, 3), "release failed")
        self.assertEqual(self.probe.status.emergency_reason, 0)
        self.assertTrue(self.probe.logged("Emergency reason changed from: '' to: 'LATCH, IMU_TIMEOUT'"),
                        [line for line in self.probe.logs if "Emergency" in line])

    def test_2_blade_still_turning_latches_and_is_sent_off_again(self):
        set_flags(cutter_ignores_off=1)
        self.assertTrue(self.probe.set_blade(True, lease_s=2.0).success)
        turning = self.probe.wait_for(lambda p: p.status.blade_motor.rpm > 1000, 3)
        self.assertTrue(turning, "blade never turned")
        t0 = sim_time()
        self.assertTrue(self.probe.set_blade(False).success)
        s = self.probe.wait_for(lambda p: p.status.emergency and p.status.emergency_reason == NOT_STOPPED and p.status,
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
        self.assertTrue(self.probe.wait_for(lambda p: p.status.blade_motor.rpm == 0, 4), "blade did not stop")
        time.sleep(1.2)
        self.probe.release()
        self.assertTrue(self.probe.wait_for(lambda p: not p.status.emergency, 3), "release failed")
        self.assertTrue(self.probe.logged("to: 'LATCH, MOWER_NOT_STOPPED'"),
                        [line for line in self.probe.logs if "Emergency" in line])

    def test_3_old_or_missing_corrections_count_as_float(self):
        self.assertTrue(self.probe.wait_for(lambda p: p.pose is not None and p.pose.solution == FIXED, 5),
                        "never fixed")
        set_flags(gnss_age=45.0)
        p = self.probe.wait_for(lambda p: p.pose.solution == FLOAT and p.pose, 3)
        self.assertTrue(p, "old corrections stayed fixed")
        self.assertAlmostEqual(p.accuracy, 0.3, places=5)
        self.assertTrue(self.probe.logged("RTK fixed on corrections 45.0 s old (limit 30 s): counted as float"))
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: p.pose.solution == FIXED, 3),
                        "fresh corrections did not fix again")
        set_flags(gnss_age=None)
        self.assertTrue(self.probe.wait_for(lambda p: p.pose.solution == FLOAT, 3), "missing age stayed fixed")
        self.assertTrue(self.probe.logged("RTK fixed without the age of its corrections"))
        set_flags()
        self.assertTrue(self.probe.wait_for(lambda p: p.pose.solution == FIXED, 3))
