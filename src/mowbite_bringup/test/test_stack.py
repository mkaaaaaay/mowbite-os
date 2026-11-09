"""stack.own_process: a process beside the stack with its limits as command prefix and an exit note of its own.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_stack.py
"""
import logging
import os
import sys

from launch.actions import ExecuteProcess, RegisterEventHandler

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "launch")))
import stack  # noqa: E402

LOGGER = logging.getLogger("test_stack")


def own_process(tmp_path, **limits):
    prefixes = []

    def make(prefix):
        prefixes.append(prefix)
        return ExecuteProcess(cmd=["true"], prefix=prefix)
    return stack.own_process(make, "vision", str(tmp_path), LOGGER, **limits), prefixes


def test_limits_go_into_the_prefix(tmp_path):
    actions, prefixes = own_process(tmp_path, oom_score_adj=1000, nice=10, cpus="0-3")
    assert prefixes == ["taskset -c 0-3 nice -n 10"]
    assert isinstance(actions[-1], ExecuteProcess)
    # Exit handler and oom_score_adj on start.
    assert sum(isinstance(a, RegisterEventHandler) for a in actions) == 2


def test_without_limits_nothing_is_added(tmp_path):
    actions, prefixes = own_process(tmp_path)
    assert prefixes == [None]
    assert sum(isinstance(a, RegisterEventHandler) for a in actions) == 1


def test_a_start_by_hand_removes_the_old_note(tmp_path):
    note = tmp_path / "vision_exit.json"
    note.write_text("{}")
    own_process(tmp_path)
    assert not note.exists()
