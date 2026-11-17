"""The NX60i taken over from OpenMower, with MowBite OS as mowbite.launch.py starts it against fake_xcore and a folder
like OpenMower's (ros/map.json, params/mower_params.yaml) as import_dir; the launch is started again whenever it ends,
as compose does on the mower. The first start with an empty data_dir takes the map, its datum, the antenna, track and
ticks and the NTRIP access; the lock stays shut. The export gives the map back in OpenMower's fields. A second start
imports nothing again, even from a changed source.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_migrate_nx60i.py
"""
import json
import os
import random
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from mowbite_msgs.msg import (HardwareCapabilities, HardwareStatus, MapStatus, RobotState, RpcError, RpcMethods,
                              RpcRequest, RpcResponse)

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (HISTORY, LATCHED, SimProbe, TestShutdown, fake_xcore, journal_dir,  # noqa: E402,F401
                        loopback_group, read_json, sim_files, wait_until, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_migrate_")
STATE, FLAGS, LOG, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
DATUM = (52.0, 9.0, 80.0)
IMPORT = os.path.join(WORK, "openmower")
EXPORT = os.path.join(WORK, "export")
STARTS = os.path.join(WORK, "starts")
RESPAWN = 2.0
MIGRATE = os.path.join(get_package_share_directory("mowbite_bringup"), "launch", "migrate.py")

MAP = {
    "areas": [
        {"id": "lawn", "properties": {"name": "Lawn", "type": "mow", "angle": 0.3, "outline_count": 2},
         "outline": [{"x": -3.0, "y": -3.0}, {"x": 6.0, "y": -3.0}, {"x": 6.0, "y": 5.0}, {"x": -3.0, "y": 5.0}]},
        {"id": "bed", "properties": {"name": "Bed", "type": "obstacle"},
         "outline": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}, {"x": 2.0, "y": 2.0}, {"x": 1.0, "y": 2.0}]},
    ],
    "docking_stations": [{"id": "dock", "properties": {"name": "Dock"}, "position": {"x": 0.0, "y": 0.0},
                          "heading": 0.0}],
}
PARAMS = f"""ll:
  services:
    gps:
      datum_lat: {DATUM[0]}
      datum_long: {DATUM[1]}
      datum_height: {DATUM[2]}
    diff_drive:
      wheel_distance_m: 0.36
      ticks_per_m: 2010.0
    power:
      charge_voltage: 29.0
ntrip_client:
  host: 127.0.0.1
  port: 9
  username: tester
  password: pw
  mountpoint: NONE
xbot_positioning:
  antenna_offset_x: -0.11
  antenna_offset_y: 0.02
mower_logic:
  undock_distance: 1.5
"""
GEOMETRY = {"antenna_x": -0.11, "antenna_y": 0.02, "track": 0.36, "ticks_per_m": 2010.0}

SCRIPT = os.path.join(WORK, "os.sh")
with open(SCRIPT, "w") as f:
    f.write(f"""echo $$ >> {STARTS}
exec ros2 launch mowbite_bringup mowbite.launch.py platform:=nx60i interface:=lo bind_ip:={BIND_IP} \\
  announcement_port:={GROUP_PORT} data_dir:={DATA} journal_dir:={JOURNAL} import_dir:={IMPORT} \\
  datum_lat:={DATUM[0]} datum_long:={DATUM[1]} datum_height:={DATUM[2]} mqtt_port:={random.randint(18000, 18999)}
""")
os.makedirs(os.path.join(IMPORT, "ros"))
os.makedirs(os.path.join(IMPORT, "params"))
write_json(os.path.join(IMPORT, "ros", "map.json"), MAP)
with open(os.path.join(IMPORT, "params", "mower_params.yaml"), "w") as f:
    f.write(PARAMS)
write_json(FLAGS, {"gps_fix": "fix"})


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--origin", str(DATUM[0]), str(DATUM[1]), "--flags", FLAGS, "--state", STATE,
                       "--log", LOG, "--log-max-mb", "60")
    system = ExecuteProcess(cmd=["sh", SCRIPT], output="screen", respawn=True, respawn_delay=RESPAWN,
                            sigterm_timeout="30", sigkill_timeout="30")
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[system, launch_testing.actions.ReadyToTest()]),
    ]), {}


def starts():
    try:
        with open(STARTS) as f:
            return [int(line) for line in f.read().split()]
    except (OSError, ValueError):
        return []


class Probe(SimProbe):
    offer_timeout = 30.0
    call_timeout = 20.0
    emergency_reason = "migrate test"

    def __init__(self):
        super().__init__("migrate_probe")
        self.status = self.robot = self.capabilities = self.map_status = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("capabilities"), LATCHED)
        self.node.create_subscription(RobotState, "bridge/robot_state", self._set("robot"), 10)
        self.node.create_subscription(MapStatus, "map/status", self._set("map_status"), LATCHED)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.node.create_subscription(RpcMethods, "bridge/rpc/methods", self._methods, HISTORY)
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.start()

    def forget(self):
        with self.lock:
            self.status = self.robot = self.capabilities = self.map_status = None
            self.methods.clear()

    def up(self, timeout=90.0):
        return self.wait(lambda: self.status is not None and self.status.telemetry_ok and self.robot is not None
                         and self.capabilities is not None and self.methods.get("mowbite_setup"), timeout)


class TestMigrate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_1_the_first_start_takes_over_from_openmower(self):
        p = self.probe
        self.assertTrue(p.up(), "the OS does not come up")
        note = read_json(os.path.join(DATA, "migrate.json"))
        self.assertEqual(note["state"], "done", note)
        self.assertEqual(note["taken"], ["map", "datum", "geometry.antenna_x", "geometry.antenna_y", "geometry.track",
                                         "geometry.ticks_per_m", "ntrip"])
        self.assertEqual(note["not_taken"], [{"key": "ll/services/power/charge_voltage", "why": "never"},
                                             {"key": "mower_logic/undock_distance", "why": "os"}])
        self.assertNotIn("tester", json.dumps(note))
        # The map holds, tied to the datum in use.
        self.assertTrue(p.wait(lambda: p.map_status is not None and p.map_status.areas == 2, 30), p.map_status)
        self.assertEqual((p.map_status.datum_state, p.map_status.docking_stations), (MapStatus.DATUM_MATCH, 1),
                         p.map_status)
        # The logic took the dimensions, not confirmed.
        answer = p.rpc("robot.geometry.get")
        self.assertIn("result", answer, answer)
        settings = answer["result"]["settings"]
        for key, value in GEOMETRY.items():
            self.assertAlmostEqual(settings[key]["value"], value, places=6, msg=key)
            self.assertTrue(settings[key]["stored"], key)
            self.assertIsNone(settings[key]["confirmed"], key)
        ntrip = os.path.join(DATA, "ntrip.yaml")
        self.assertEqual(stat.S_IMODE(os.stat(ntrip).st_mode), 0o600)
        # The lock stays shut: no date of a safety measurement came over.
        self.assertEqual(p.capabilities.safety_measurement_date, "")
        answer = p.rpc("setup.get")
        self.assertIn("result", answer, answer)
        safety = answer["result"]["safety"]
        self.assertEqual((safety["open"], safety["why"]), (False, "NO_SAFETY_MEASUREMENT"), safety)

    def test_2_the_export_gives_the_map_back(self):
        p = self.probe
        self.assertTrue(p.wait(lambda: p.robot.current_state == "IDLE" and p.robot.current_sub_state == "", 60),
                        p.robot)
        result = subprocess.run([sys.executable, MIGRATE, "export", EXPORT, "--data-dir", DATA], capture_output=True,
                                text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(read_json(os.path.join(EXPORT, "map.json")), MAP)
        report = read_json(os.path.join(EXPORT, "report.json"))
        self.assertEqual(report["lost"], [], report)
        self.assertEqual(report["datum"], {"datum_lat": DATUM[0], "datum_long": DATUM[1], "datum_height": DATUM[2]})

    def test_3_a_second_start_imports_nothing_again(self):
        p = self.probe
        note = read_json(os.path.join(DATA, "migrate.json"))
        changed = {"areas": [], "docking_stations": []}
        write_json(os.path.join(IMPORT, "ros", "map.json"), changed)
        first = starts()
        self.assertEqual(len(first), 1, first)
        os.kill(first[0], signal.SIGINT)
        self.assertTrue(wait_until(lambda: len(starts()) >= 2, 60), "the launch did not start again")
        p.forget()
        self.assertTrue(p.up(), "the second start does not come up")
        self.assertEqual(read_json(os.path.join(DATA, "migrate.json")), note)
        self.assertTrue(p.wait(lambda: p.map_status is not None and p.map_status.areas == 2, 30), p.map_status)
        self.assertEqual(read_json(os.path.join(DATA, "map.json")), MAP)


@launch_testing.post_shutdown_test()
class TestExit(TestShutdown):
    pass
