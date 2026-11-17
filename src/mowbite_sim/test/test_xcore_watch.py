"""xcore_driver against fake_xcore on UDP over the loopback: a thread of the driver that hangs ends the process by its
watch with code 86, so that the launch starts a latched driver. The hold file becomes a FIFO, whose read blocks the
output thread until a writer comes, which never does.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_xcore_watch.py
"""
import os
import shutil
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from launch.actions import TimerAction

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import EXIT_CODES, HwProbe, fake_xcore, loopback_group, sim_files, xcore_driver  # noqa: E402

WORK = tempfile.mkdtemp(prefix="mowbite_xcore_watch_")
STATE, FLAGS, LOG, _ = sim_files(WORK)
GROUP_PORT, BIND_IP = loopback_group()
HOLD = os.path.join(WORK, "hold.json")
# mowbite_watch::kExitCode, and the factor its limit of 2 s gets in sanitizer runs.
WATCH_EXIT = 86
WATCH_FACTOR = float(os.environ.get("MOWBITE_WATCH_FACTOR", "1"))


def generate_test_description():
    board = fake_xcore(GROUP_PORT, "--flags", FLAGS, "--log", LOG, "--log-max-mb", "20", "--state", STATE)
    driver = xcore_driver(BIND_IP, GROUP_PORT, hold_file=HOLD)
    return launch.LaunchDescription([
        board,
        TimerAction(period=0.5, actions=[driver, launch_testing.actions.ReadyToTest()]),
    ]), {"board": board, "driver": driver}


class TestXcoreWatch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = HwProbe("xcore_watch_probe", log=LOG)
        cls.probe.start()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()

    def test_a_hanging_output_thread_ends_the_driver(self, proc_info, proc_output, driver):
        p = self.probe
        self.assertTrue(p.wait_for(lambda p: p.status is not None and p.status.telemetry_ok, 20), "driver never ready")
        os.mkfifo(HOLD)
        # Seen at the next look at the hold file (0.5 s), then silent for the limit.
        proc_info.assertWaitForShutdown(process=driver, timeout=0.5 + 2.0 * WATCH_FACTOR + 5.0)
        self.assertEqual(proc_info[driver].returncode, WATCH_EXIT)
        proc_output.assertWaitFor("mowbite_watch: xcore_output silent", process=driver, stream="stderr", timeout=1)
        proc_output.assertWaitFor("last 'hold file'", process=driver, stream="stderr", timeout=1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_as_expected(self, proc_info, board, driver):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES, process=board)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[WATCH_EXIT], process=driver)
        shutil.rmtree(WORK, ignore_errors=True)
