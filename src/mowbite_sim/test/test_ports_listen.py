"""The ports of the mower (FP27 S2) in the stage of the choice: setup.ports lists a pty as a port of the machine,
free and with the mower at rest; setup.ports.listen hears NMEA on it at the first rate, hears nothing on a quiet one,
refuses an unknown port and a second listen while one runs, and never writes a byte to the port.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_ports_listen.py
"""
import json
import os
import pty
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

import launch
import launch_testing.actions
import rclpy
from launch.actions import ExecuteProcess
from mowbite_msgs.msg import RpcError, RpcRequest, RpcResponse

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import SimProbe, journal_dir, loopback_group, sim_files, stop_process  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_ports_listen_")
_, _, _, DATA = sim_files(WORK)
JOURNAL = journal_dir(WORK)
GROUP_PORT, BIND_IP = loopback_group()
# Two ports as the pty of a device: a receiver the test writes NMEA to and a quiet one.
RECEIVER_MASTER, RECEIVER = pty.openpty()
QUIET_MASTER, QUIET = pty.openpty()
RECEIVER_PATH = os.ttyname(RECEIVER)
QUIET_PATH = os.ttyname(QUIET)
# Nobody holds the device side, like a port with a device on it: raw, then closed here.
for _slave in (RECEIVER, QUIET):
    tty.setraw(_slave)
    os.close(_slave)
os.set_blocking(RECEIVER_MASTER, False)
EXTRA = os.path.join(WORK, "ports.yaml")
ENV = {key: value for key, value in os.environ.items() if key != "MOWBITE_PLATFORM"}
COMMAND = ["ros2", "launch", "mowbite_bringup", "mowbite.launch.py", f"data_dir:={DATA}", f"journal_dir:={JOURNAL}",
           f"mqtt_port:={random.randint(18000, 18999)}", "interface:=lo", f"bind_ip:={BIND_IP}",
           f"announcement_port:={GROUP_PORT}", f"extra_config:={EXTRA}"]

os.makedirs(DATA, exist_ok=True)
with open(EXTRA, "w") as f:
    f.write("mowbite_setup:\n  ros__parameters:\n    ports_extra: '%s'\n" % json.dumps([RECEIVER_PATH, QUIET_PATH]))


def generate_test_description():
    keep = ExecuteProcess(cmd=["sleep", "infinity"])
    return launch.LaunchDescription([keep, launch_testing.actions.ReadyToTest()]), {}


def gga():
    body = "GNGGA,120000.00,5200.0,N,00900.0,E,4,12,0.8,80.0,M,47.0,M,1.0,0000"
    sum_ = 0
    for c in body:
        sum_ ^= ord(c)
    return ("$%s*%02X\r\n" % (body, sum_)).encode()


def drained(fd):
    """The bytes waiting on the master side of a pty: what the OS wrote to the port."""
    os.set_blocking(fd, False)
    data = b""
    try:
        while True:
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            data += chunk
    except OSError:
        # Nothing waits, or nobody has the device side open.
        pass
    return data


class Probe(SimProbe):
    def __init__(self):
        super().__init__("setup_ports_probe")
        self.rpc_pub = self.node.create_publisher(RpcRequest, "bridge/rpc/request", 10)
        self.node.create_subscription(RpcResponse, "bridge/rpc/response", self._response, 10)
        self.node.create_subscription(RpcError, "bridge/rpc/error", self._error, 10)
        self.start()


class TestSetupPorts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.sending = True
        cls.sender = threading.Thread(target=cls.send, daemon=True)
        cls.sender.start()
        with open(os.path.join(WORK, "launch.log"), "w") as log:
            cls.process = subprocess.Popen(COMMAND, env=ENV, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)

    @classmethod
    def send(cls):
        while cls.sending:
            try:
                os.write(RECEIVER_MASTER, gga())
            except OSError:
                # Full while nobody listens.
                pass
            time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.sending = False
        if cls.process.poll() is None:
            stop_process(cls.process, signal.SIGINT, group=True)
        cls.probe.stop()
        rclpy.shutdown()
        path = os.path.join(WORK, "launch.log")
        if os.path.exists(path):
            with open(path) as f:
                print("--- launch, last lines ---\n" + "".join(f.readlines()[-60:]))
        shutil.rmtree(WORK, ignore_errors=True)

    def ports(self):
        answer = self.probe.rpc("setup.ports", {}, 60)
        self.assertIsNotNone(answer)
        return answer["result"]

    def listening(self, timeout):
        """Whether setup.ports tells of a running listen within timeout s."""
        end = time.time() + timeout
        while time.time() < end:
            if self.ports()["listening"]:
                return True
            time.sleep(0.1)
        return False

    def test_1_the_ports_and_what_they_send(self):
        result = self.ports()
        self.assertTrue(result["resting"])
        self.assertFalse(result["listening"])
        listed = {port["path"]: port for port in result["ports"]}
        for path in (RECEIVER_PATH, QUIET_PATH):
            self.assertIn(path, listed)
            self.assertTrue(listed[path]["free"], listed[path])
            self.assertEqual(listed[path]["node"], "")

        heard = self.probe.rpc("setup.ports.listen", {"port": RECEIVER_PATH, "seconds": 8}, 20)
        self.assertIn("result", heard, heard)
        heard = heard["result"]
        self.assertEqual(heard["kind"], "nmea")
        self.assertEqual(heard["baud"], 115200)
        self.assertEqual(heard["nmea"]["fix"], 4)
        self.assertEqual(heard["nmea"]["talkers"], ["GN"])
        self.assertFalse(heard["stopped"])

        quiet = self.probe.rpc("setup.ports.listen", {"port": QUIET_PATH, "seconds": 2}, 15)["result"]
        self.assertEqual(quiet["kind"], "silent")
        self.assertIsNone(quiet["baud"])
        self.assertEqual(len(quiet["tried"]), 2)
        # Listening never sends: nothing came out on the side of the devices.
        self.assertEqual(drained(QUIET_MASTER), b"")

    def test_2_what_listen_refuses(self):
        unknown = self.probe.rpc("setup.ports.listen", {"port": "/dev/ttyMOWBITE9"}, 15)
        self.assertEqual(unknown["error"]["data"]["reason"], "PORT_UNKNOWN", unknown)
        invalid = self.probe.rpc("setup.ports.listen", {"port": QUIET_PATH, "seconds": 31}, 15)
        self.assertEqual(invalid["error"]["data"]["field"], "seconds", invalid)
        long_id = self.probe._send("setup.ports.listen", {"port": QUIET_PATH, "seconds": 4}, None, None)
        self.assertTrue(self.listening(5))
        second = self.probe.rpc("setup.ports.listen", {"port": RECEIVER_PATH, "seconds": 1}, 15)
        self.assertEqual(second["error"]["data"]["reason"], "LISTENING", second)
        self.assertEqual(second["error"]["data"]["port"], QUIET_PATH)
        self.assertIsNotNone(self.probe.wait(lambda: self.probe.answers.get(long_id), 15))
        self.assertFalse(self.ports()["listening"])
        self.assertEqual(drained(QUIET_MASTER), b"")


if __name__ == "__main__":
    unittest.main()
