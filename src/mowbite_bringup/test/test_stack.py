"""stack.own_process: a process beside the stack with its limits as command prefix, restarts and an exit note. The
container: its watch knows the nodes, the note of the watch goes into the exit note, every component has its watch.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_stack.py
"""
import glob
import json
import logging
import os
import re
import sys
import time

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import ComposableNodeContainer

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
    before = time.monotonic()
    assert service.run() == 0
    lines = runs.read_text().splitlines()
    assert len(lines) == 4
    for line in lines:
        assert line == "{'nice': 10, 'cpus': [0], 'oom': 1000}"
    note = json.loads((tmp_path / "vision_exit.json").read_text())
    assert (note["exit_code"], note["restart"], note["max_restarts"], note["down"]) == (3, 3, 3, True)
    # The end on the monotonic clock, for readers that compare it with their own.
    assert before < note["mono"] <= time.monotonic()


# Writes the note of the watch as mowbite_watch does and ends with its code.
WATCH_PROBE = """
import sys
with open(sys.argv[1], "w") as f:
    f.write('{"node":"mowbite_bridge","seconds":10.2,"limit":10.0}' + chr(10))
sys.exit(86)
"""


def test_the_note_of_the_watch_goes_into_the_exit_note(tmp_path, monkeypatch):
    monkeypatch.setattr(stack, "RESTART_DELAY", 0.1)
    note = tmp_path / stack.EXIT_NOTE
    watch_note = tmp_path / stack.WATCH_NOTE
    script = tmp_path / "probe.py"
    script.write_text(WATCH_PROBE)
    starts = []

    def make(restart):
        starts.append(restart)
        if restart == 0:
            return ExecuteProcess(cmd=[sys.executable, str(script), str(watch_note)])
        return ExecuteProcess(cmd=[sys.executable, "-c", "import sys; sys.exit(9)"])
    actions = stack.restarting(make, str(note), "The container", LOGGER, 2, watch_note=str(watch_note))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    assert service.run() == 0
    assert starts == [0, 1, 2]
    # The last end that wrote the note was no stall: null, and the note of the watch is gone after it was taken.
    assert json.loads(note.read_text())["stalled"] is None
    assert not watch_note.exists()


class Lines:
    def __init__(self):
        self.lines = []

    def error(self, text):
        self.lines.append(text)

    warning = error


def test_the_last_end_says_down_with_the_stalled_node(tmp_path, monkeypatch):
    monkeypatch.setattr(stack, "RESTART_DELAY", 0.1)
    note = tmp_path / stack.EXIT_NOTE
    watch_note = tmp_path / stack.WATCH_NOTE
    script = tmp_path / "probe.py"
    script.write_text(WATCH_PROBE)
    logged = Lines()
    actions = stack.restarting(lambda start: ExecuteProcess(cmd=[sys.executable, str(script), str(watch_note)]),
                               str(note), "The container", logged, 1, watch_note=str(watch_note))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    assert service.run() == 0
    written = json.loads(note.read_text())
    assert (written["down"], written["restart"], written["exit_code"]) == (True, 1, 86)
    assert written["stalled"] == {"node": "mowbite_bridge", "seconds": 10.2, "limit": 10.0}
    assert written["stable_s"] == stack.STABLE_AFTER
    assert logged.lines[-1] == ("The container ended with exit code 86 (the watch: mowbite_bridge gave no sign) "
                                "after 1 restarts and stays down until the next start")


def test_a_stable_run_counts_the_crashes_from_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(stack, "RESTART_DELAY", 0.1)
    note = tmp_path / stack.EXIT_NOTE
    # Start and end of each run: the second one lasts STABLE_AFTER s.
    times = iter([0, 1, 1, 1 + stack.STABLE_AFTER, 1 + stack.STABLE_AFTER, 2 + stack.STABLE_AFTER])
    starts = []

    def make(start):
        starts.append(start)
        return ExecuteProcess(cmd=[sys.executable, "-c", "import sys; sys.exit(5)"])
    actions = stack.restarting(make, str(note), "The container", LOGGER, 1, clock=lambda: next(times))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    assert service.run() == 0
    assert starts == [0, 1, 2]
    written = json.loads(note.read_text())
    assert written["down"] is True
    # Each end reads the clock once, for the count and for the note.
    assert written["mono"] == 2 + stack.STABLE_AFTER


def test_the_exit_note_after_a_stall(tmp_path):
    watch_note = tmp_path / stack.WATCH_NOTE
    watch_note.write_text('{"node":"mowbite_safety","seconds":2.1,"limit":2.0}\n')
    assert stack.take_watch_note(str(watch_note), LOGGER) == {"node": "mowbite_safety", "seconds": 2.1, "limit": 2.0}
    assert not watch_note.exists()
    for text in ("", "[]", "{", '{"seconds": 2}'):
        watch_note.write_text(text)
        assert stack.take_watch_note(str(watch_note), LOGGER) is None
        assert not watch_note.exists()
    assert stack.take_watch_note(None, LOGGER) is None


def test_a_start_by_hand_removes_the_note_of_the_watch(tmp_path):
    watch_note = tmp_path / stack.WATCH_NOTE
    watch_note.write_text("{}")
    stack.restarting(lambda restart: ExecuteProcess(cmd=["true"]), None, "x", LOGGER, 1, watch_note=str(watch_note))
    assert not watch_note.exists()


def test_the_watch_of_the_container_expects_its_nodes(tmp_path):
    nodes = stack.components(["a.yaml"], "g.yaml", str(tmp_path), "127.0.0.1", "1883", "")
    actions = stack.container_and_planner(nodes, str(tmp_path), ["a.yaml"], LOGGER)
    container, = [a for a in actions if isinstance(a, ComposableNodeContainer)]
    env = {"".join(p.text for p in key): "".join(p.text for p in value)
           for key, value in container.process_description.additional_env}
    names = ["mowbite_localization", "mowbite_safety", "mowbite_logic", "mowbite_bridge", "mowbite_map",
             "mowbite_follower", "mowbite_ntrip", "mowbite_calibration", "mowbite_clock"]
    assert stack.node_names(nodes) == names
    assert env == {"MOWBITE_WATCH_NODES": ",".join(names),
                   "MOWBITE_WATCH_NOTE": os.path.join(str(tmp_path), stack.WATCH_NOTE)}


SRC = os.path.normpath(os.path.join(HERE, "..", ".."))
# Components that run in a process of their own and not in the container.
OWN_PROCESS = {"tron_mcu", "xcore_driver"}


def test_every_component_in_the_container_has_its_watch():
    seen = set()
    for path in glob.glob(os.path.join(SRC, "*", "src", "*.cpp")):
        text = open(path).read()
        if "RCLCPP_COMPONENTS_REGISTER_NODE" not in text:
            continue
        package = os.path.relpath(path, SRC).split(os.sep)[0]
        seen.add(package)
        calls = len(re.findall(r"mowbite_watch::Attach\(\*this", text))
        assert calls == (0 if package in OWN_PROCESS else 1), path
    assert len(seen) >= 14, seen


def test_no_parameter_file_names_a_limit_of_the_watch():
    for path in glob.glob(os.path.join(SRC, "*", "config", "**", "*.yaml"), recursive=True):
        assert "watch" not in open(path).read().lower(), path
