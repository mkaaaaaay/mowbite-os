"""The guard of tron_mcu (review 1, F1) against fake_tron on a pty: when the I/O thread of the driver stalls while the
mower drives, a thread of its own sends the stop and the blade off within about 0.35 s, three times; after a short
stall the driver goes on as after a reopen, after a long one the watch ends the process. The stall: hold.json becomes
a FIFO without a writer, and the I/O thread hangs in its open().

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_driver_stall.py
"""
import functools
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import TRON_CUTTER as CUTTER, TRON_SPEED as SPEED  # noqa: E402
from sim_common import (HwProbe, TestShutdown, fake_tron, last_time, package_program, sim_files,  # noqa: E402,F401
                        sim_links, stop_process, sub_packets, wheel_speeds, write_json)

WORK = tempfile.mkdtemp(prefix="mowbite_stall_")
LINKS = sim_links(WORK)
_, FLAGS, LOG, _ = sim_files(WORK)
HOLD = os.path.join(WORK, "hold.json")
# kGuardAfterS of the driver; the stop may come up to a motion frame (0.1 s) and the look of the guard later.
GUARD_AFTER = 0.25
WATCH_EXIT = 86


def generate_test_description():
    write_json(FLAGS, {})
    return launch.LaunchDescription([fake_tron(LINKS, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "5"),
                                     launch_testing.actions.ReadyToTest()]), {}


received = functools.partial(sub_packets, LOG)


def zero(entry):
    return entry.get("speed") == [0.0, 0.0]


def blade_off(entry):
    return entry["hex"][:2] == "00"


class Driver:
    """tron_mcu as a process of its own: the watch may end it."""

    def __init__(self):
        exe = package_program("tron_mcu", "tron_mcu_node")
        self.proc = subprocess.Popen([exe, "--ros-args", "-p", f"port:={LINKS['mcu']}", "-p", "send_imu:=false",
                                      "-p", f"hold_file:={HOLD}"], start_new_session=True)

    def stop(self):
        return stop_process(self.proc, signal.SIGINT, group=True)


class Probe(HwProbe):
    def __init__(self):
        super().__init__("driver_stall_probe", log=LOG)
        self.start()

    def driving(self, seconds):
        """cmd_vel 0.2 m/s for that long in a thread of its own."""
        thread = threading.Thread(target=self.cmd_vel_for, args=(0.2, seconds), daemon=True)
        thread.start()
        return thread


def stall():
    """hold.json as a FIFO: the next poll of the driver hangs in open() until a writer comes."""
    os.mkfifo(HOLD + ".new")
    os.replace(HOLD + ".new", HOLD)


def unstall():
    """A writer that closes at once: the open of the driver returns, the FIFO goes, the hold with it."""
    try:
        fd = os.open(HOLD, os.O_WRONLY | os.O_NONBLOCK)
    except OSError:
        fd = None
    os.remove(HOLD)
    if fd is not None:
        os.close(fd)


class TestDriverStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.driver = Driver()

    @classmethod
    def tearDownClass(cls):
        if os.path.exists(HOLD):
            unstall()
        if cls.driver:
            cls.driver.stop()
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def stalled_while_driving(self, seconds):
        """Drives, stalls the I/O thread, waits that long; (time of the last motion frame, the frames after it)."""
        self.assertTrue(self.probe.wait(lambda: self.probe.status and self.probe.status.telemetry_ok, 15),
                        "no telemetry")
        began = last_time(LOG)
        drive = self.probe.driving(seconds + 3.0)
        time.sleep(1.0)
        self.assertGreater(len(wheel_speeds(LOG, began, SPEED)), 3, "no motion before the stall")
        stall()
        time.sleep(seconds)
        last = max(e["t"] for e in wheel_speeds(LOG, began, SPEED))
        return last, drive, [e for e in received(last) if e["t"] > last]

    def guarded(self, last, after):
        speeds = [e for e in after if e["mod"] == SPEED]
        self.assertTrue(speeds, f"no stop after the last motion at {last}")
        self.assertTrue(all(zero(e) for e in speeds[:3]), speeds[:3])
        self.assertLessEqual(speeds[0]["t"] - last, GUARD_AFTER + 0.25,
                             f"the stop came {speeds[0]['t'] - last:.2f} s after the last motion")
        blades = [e for e in after if e["mod"] == CUTTER]
        self.assertTrue(blades and blade_off(blades[0]), blades[:3])
        print(f"stall: stop {speeds[0]['t'] - last:.2f} s after the last motion frame")

    def test_a_a_short_stall_stops_the_wheels_and_it_drives_on_after(self):
        last, drive, after = self.stalled_while_driving(1.0)
        self.guarded(last, after)
        self.assertIsNone(self.driver.proc.poll(), "the driver ended on a short stall")
        resumed = last_time(LOG)
        unstall()
        drive.join()
        self.assertIsNone(self.driver.proc.poll())
        self.assertGreater(len(wheel_speeds(LOG, resumed, SPEED)), 3, "no motion after the stall")

    def test_b_a_long_stall_stops_the_wheels_and_the_watch_ends_the_driver(self):
        time.sleep(1.0)
        last, drive, after = self.stalled_while_driving(3.5)
        self.guarded(last, after)
        self.assertEqual(self.driver.proc.wait(timeout=5), WATCH_EXIT)
        speeds = [e for e in received(last) if e["t"] > last and e["mod"] == SPEED]
        self.assertFalse([e for e in speeds if not zero(e)], "motion after the stop")
        drive.join()
        unstall()
        type(self).driver = None
