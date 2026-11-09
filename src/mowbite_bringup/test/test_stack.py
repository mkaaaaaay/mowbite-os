"""stack.own_process: a process beside the stack with its limits as command prefix, restarts and an exit note.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_stack.py
"""
import json
import logging
import os
import sys

from launch import LaunchDescription, LaunchService
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


# Reports nice and CPUs of a thread it starts, and its oom_score_adj, then ends with code 3.
PROBE = """
import os, sys, threading, time
time.sleep(0.5)
seen = {}
def probe():
    tid = threading.get_native_id()
    seen["nice"] = os.getpriority(os.PRIO_PROCESS, tid)
    seen["cpus"] = sorted(os.sched_getaffinity(tid))
t = threading.Thread(target=probe)
t.start()
t.join()
with open(f"/proc/{os.getpid()}/oom_score_adj") as f:
    seen["oom"] = int(f.read())
with open(sys.argv[1], "a") as f:
    f.write(repr(seen) + chr(10))
sys.exit(3)
"""


def test_the_process_restarts_three_times_with_its_limits(tmp_path, monkeypatch):
    monkeypatch.setattr(stack, "RESTART_DELAY", 0.1)
    runs = tmp_path / "runs"
    script = tmp_path / "probe.py"
    script.write_text(PROBE)
    actions = stack.own_process(
        lambda prefix: ExecuteProcess(cmd=[sys.executable, str(script), str(runs)], prefix=prefix),
        "vision", str(tmp_path), LOGGER, oom_score_adj=1000, nice=10, cpus="0")
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    assert service.run() == 0
    lines = runs.read_text().splitlines()
    assert len(lines) == 4
    for line in lines:
        assert line == "{'nice': 10, 'cpus': [0], 'oom': 1000}"
    note = json.loads((tmp_path / "vision_exit.json").read_text())
    assert (note["exit_code"], note["restart"], note["max_restarts"]) == (3, 3, 3)
