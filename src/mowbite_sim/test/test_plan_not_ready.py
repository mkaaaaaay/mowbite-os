"""mowing.plan right after the start, without a map (sim.launch.py on a private mosquitto): the first request the logic
sends once the planner offers its service gets a plan of the outline, even where the planner still answers NOT_READY
(no map or no dimensions of the mower taken yet): the logic asks again every 2 s for up to 60 s. An outline is planned
as a preview of the planner, not as an area of the map.

Run in a sourced workspace with mosquitto installed: launch_test /ws/src/mowbite_sim/test/test_plan_not_ready.py
"""
import itertools
import os
import shutil
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
from launch.actions import TimerAction

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import EXIT_CODES, MqttBroker, mosquitto, sim_files, sim_stack, wait_until  # noqa: E402

MQTT_PORT = 18856
WORK = tempfile.mkdtemp(prefix="mowbite_plan_not_ready_")
_, _, _, DATA = sim_files(WORK)
OUTLINE = {"outline": [{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 3}, {"x": 0, "y": 3}], "obstacles": [],
           "outline_count": 1, "angle": 0.0}
# The answer of the logic before the planner offers its service.
MISSING = "The planner is not running"
# The planner's 30 s and the retries' 60 s.
PLAN_TIMEOUT = 100


def generate_test_description():
    return launch.LaunchDescription([mosquitto(WORK, MQTT_PORT),
                                     TimerAction(period=1.0, actions=[sim_stack(WORK, MQTT_PORT)]),
                                     launch_testing.actions.ReadyToTest()]), {}


class TestPlanNotReady(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.broker = MqttBroker("plan_not_ready_test", MQTT_PORT)

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()

    def test_1_the_first_plan_after_the_start(self):
        # Asked again only while the logic says the planner is not running yet.
        asked = itertools.count(1)

        def answered():
            answer = self.broker.rpc("mowing.plan", OUTLINE, f"early-{next(asked)}", PLAN_TIMEOUT)
            return answer if answer and (answer.get("error") or {}).get("message") != MISSING else None

        answer = wait_until(answered, 120, poll=0.2)
        self.assertTrue(answer, "no answer to mowing.plan")
        self.assertNotIn("error", answer, answer)
        self.assertTrue(answer["result"].get("paths"), answer["result"])
        self.assertEqual(answer["result"].get("area_id"), "")

    def test_2_again_later(self):
        answer = self.broker.rpc("mowing.plan", OUTLINE, "later", PLAN_TIMEOUT)
        self.assertTrue(answer and (answer.get("result") or {}).get("paths"), answer)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        shutil.rmtree(WORK, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES)
