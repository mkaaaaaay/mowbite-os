"""hardware:=read against fake_tron: tron_mcu and tron_imu only read their ptys, tron_gnss does not start. Not one byte
reaches the MCU pty, also while hw/cmd_vel, hw/blade and hw/charge ask for more; the capabilities let nothing move; the
app sees the battery and the sensors.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_read_only.py
"""
import json
import os
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
from geometry_msgs.msg import Twist, Vector3
from launch.actions import TimerAction
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetCharge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import LATCHED, mosquitto, read_json, sim_files, sim_stack, wait_until  # noqa: E402

MQTT_PORT = 18851
WORK = tempfile.mkdtemp(prefix="mowbite_read_only_")
STATE = sim_files(WORK)[0]
APP_CHECK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "app_interface_check.py")


def generate_test_description():
    sim = sim_stack(WORK, MQTT_PORT, hardware="read")
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT), TimerAction(period=1.0, actions=[sim]),
                                     launch_testing.actions.ReadyToTest()]), {}


def state():
    return read_json(STATE)


class TestReadOnly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("read_only_probe")
        cls.lock = threading.Lock()
        cls.status = cls.caps = None
        cls.imu = 0
        cls.node.create_subscription(HardwareStatus, "hw/status", lambda m: cls._set("status", m), 10)
        cls.node.create_subscription(HardwareCapabilities, "hw/capabilities", lambda m: cls._set("caps", m), LATCHED)
        cls.node.create_subscription(Imu, "imu/data", lambda m: cls._count_imu(), qos_profile_sensor_data)
        cls.cmd_vel = cls.node.create_publisher(Twist, "hw/cmd_vel", 10)
        cls.blade = cls.node.create_client(SetBlade, "hw/blade")
        cls.charge = cls.node.create_client(SetCharge, "hw/charge")
        cls.running = True
        cls.spinner = threading.Thread(target=cls._spin, daemon=True)
        cls.spinner.start()

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        cls.spinner.join()
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    @classmethod
    def _spin(cls):
        while cls.running:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _set(cls, name, msg):
        with cls.lock:
            setattr(cls, name, msg)

    @classmethod
    def _count_imu(cls):
        with cls.lock:
            cls.imu += 1

    def wait(self, what, seconds=60):
        return bool(wait_until(what, seconds, self.lock, 0.1))

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(request)
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        self.assertTrue(future.done())
        return future.result()

    def test_1_reads_telemetry_and_imu(self):
        self.assertTrue(self.wait(lambda: self.status is not None and self.status.telemetry_ok and self.imu > 50))
        with self.lock:
            self.assertAlmostEqual(self.status.battery_voltage, 24.5, delta=0.05)
            self.assertFalse(self.status.moving)
        self.assertTrue(self.wait(lambda: self.caps is not None and self.caps.firmware[0].value == "0.6.36"))
        with self.lock:
            self.assertEqual((self.caps.max_linear_speed, self.caps.max_angular_speed), (0.0, 0.0))
            self.assertEqual(self.caps.safety_measurement_date, "")
            self.assertFalse(self.caps.charge_stop)
        self.assertEqual(state()["host_bytes"], 0)

    def test_2_commands_reach_nothing(self):
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.cmd_vel.publish(Twist(linear=Vector3(x=0.2)))
            time.sleep(0.05)
        blade = self.call(self.blade, SetBlade.Request(on=True, lease_s=1.0))
        self.assertFalse(blade.success)
        charge = self.call(self.charge, SetCharge.Request(on=True))
        self.assertFalse(charge.success)
        time.sleep(1.0)
        with self.lock:
            self.assertFalse(self.status.moving)
            self.assertFalse(self.status.blade_on)
        now = state()
        self.assertEqual(now["host_bytes"], 0)
        self.assertEqual(now["linear"], 0.0)

    def test_3_no_gnss_node(self):
        # One listing can miss a node under load, so it asks again until both drivers show.
        end = time.monotonic() + 15.0
        while True:
            nodes = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=30).stdout.split()
            if {"/tron_mcu", "/tron_imu"} <= set(nodes) or time.monotonic() > end:
                break
            time.sleep(1.0)
        self.assertIn("/tron_mcu", nodes)
        self.assertIn("/tron_imu", nodes)
        self.assertNotIn("/tron_gnss", nodes)

    def test_4_the_app_sees_battery_and_sensors(self):
        done = subprocess.run([sys.executable, APP_CHECK, "--host", "127.0.0.1", "--port", str(MQTT_PORT), "--target",
                               "mowbite-os", "--seconds", "3"], capture_output=True, text=True, timeout=120)
        print(done.stderr)
        points = {p["point"]: p for p in json.loads(done.stdout)["points"]}
        for point in ("3.1 sensor value text", "3.1 robot_state fields"):
            self.assertEqual(points[point]["status"], "ok", point)
        # Without tron_gnss there is no RTK correction age; every sensor of the MCU is there.
        ids = set(points["3.1 sensor ids"]["example"].split(","))
        self.assertTrue({"om_v_battery", "battery_temp", "om_charge_state", "om_left_esc_temp", "om_right_esc_temp",
                         "om_mow_motor_rpm"} <= ids, ids)
        self.assertNotIn("os_correction_age", ids)
        self.assertEqual(state()["host_bytes"], 0)
