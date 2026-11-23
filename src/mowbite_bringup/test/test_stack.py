"""stack.own_process: a process beside the stack with its limits as command prefix, restarts and an exit note. The
container: its watch knows the nodes, the note of the watch goes into the exit note, every component has its watch.

Run in a sourced workspace (the drivers of the platforms are found by the ament index): python3 -m pytest
/ws/src/mowbite_bringup/test/test_stack.py
"""
import glob
import json
import logging
import os
import re
import sys
import time

import pytest
import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import ComposableNodeContainer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "launch")))
import layers  # noqa: E402
import stack  # noqa: E402
from stack_probe import start  # noqa: E402,F401

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


# The container ends itself for a restart of the OS the setup program asked for: the launch ends with it, without a
# restart of the container and without the note of a crash.
def test_a_process_that_ends_for_a_restart_of_the_os_ends_the_launch(tmp_path, monkeypatch):
    monkeypatch.setattr(stack, "RESTART_DELAY", 0.1)
    asked = tmp_path / stack.RESTART_NOTE
    starts = []

    def make(number):
        starts.append(number)
        return ExecuteProcess(cmd=[sys.executable, "-c", f"open({str(asked)!r}, 'w').close()"])
    actions = stack.restarting(make, str(tmp_path / stack.EXIT_NOTE), "The container", LOGGER, 3,
                               leave=asked.exists)
    # Beside it a process that would run on: the end of the launch stops it.
    actions.append(ExecuteProcess(cmd=[sys.executable, "-c", "import time; time.sleep(60)"]))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    before = time.monotonic()
    assert service.run() == 0
    assert time.monotonic() - before < 30
    assert starts == [0]
    assert not (tmp_path / stack.EXIT_NOTE).exists()


def test_a_start_removes_the_note_of_the_restart_before(tmp_path):
    asked = tmp_path / stack.RESTART_NOTE
    asked.write_text("{}")
    nodes = stack.components(["a.yaml"], "g.yaml", str(tmp_path), "127.0.0.1", "1883", "")
    stack.container_and_planner(nodes, str(tmp_path), ["a.yaml"], LOGGER)
    assert not asked.exists()


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
    actions = stack.restarting(lambda number: ExecuteProcess(cmd=[sys.executable, str(script), str(watch_note)]),
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

    def make(number):
        starts.append(number)
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
             "mowbite_follower", "mowbite_ntrip", "mowbite_calibration", "mowbite_clock", "mowbite_schedule",
             "mowbite_stats", "mowbite_obstacles", "mowbite_setup"]
    assert stack.node_names(nodes) == names
    assert env == {"MOWBITE_WATCH_NODES": ",".join(names),
                   "MOWBITE_WATCH_NOTE": os.path.join(str(tmp_path), stack.WATCH_NOTE)}


SRC = os.path.normpath(os.path.join(HERE, "..", ".."))


# Every component attaches itself to the watch once, the drivers of all platforms as well.
def test_every_component_has_its_watch(start):  # noqa: F811
    drivers = {start(platform).node(stack.platform_module(platform).DRIVER)["package"]
               for platform in layers.platforms()}
    seen = set()
    for path in glob.glob(os.path.join(SRC, "*", "src", "*.cpp")):
        text = open(path).read()
        if "RCLCPP_COMPONENTS_REGISTER_NODE" not in text:
            continue
        package = os.path.relpath(path, SRC).split(os.sep)[0]
        seen.add(package)
        calls = len(re.findall(r"mowbite_watch::Attach\(\*this", text))
        assert calls == 1, path
    assert drivers <= seen, drivers - seen
    assert len(seen) >= 14, seen


def test_no_parameter_file_names_a_limit_of_the_watch():
    paths = [*glob.glob(os.path.join(SRC, "*", "config", "**", "*.yaml"), recursive=True),
             *glob.glob(os.path.join(SRC, "platform_*", "*.yaml"))]
    assert len(paths) >= 6, paths
    for path in paths:
        assert "watch" not in open(path).read().lower(), path


# The three readers of the safety measurement find the lock of the setup program in data_dir.
def test_the_readers_of_the_safety_measurement_get_data_dir(start, tmp_path):  # noqa: F811
    for platform in layers.platforms():
        built = start(platform)
        for name in ("mowbite_safety", "mowbite_logic", "mowbite_follower", "mowbite_setup"):
            parameters = next(k for k in built.components if k["name"] == name)["parameters"]
            dirs = [str(p["data_dir"]) for p in parameters if isinstance(p, dict) and "data_dir" in p]
            assert dirs == [str(tmp_path)], (platform, name, dirs)


def geometry_file(folder, text="mowbite_logic: {ros__parameters: {geometry: {platform: some_mower}}}"):
    (folder / "geometry.yaml").write_text(text)


# What the setup program found for this mower comes before its file kept by hand, also for a driver with files of its
# own, and the program learns which of its values that file overrules.
def test_the_file_of_the_setup_program_comes_before_the_one_kept_by_hand(start, tmp_path):  # noqa: F811
    found, device = tmp_path / "nx60i.setup.yaml", tmp_path / "nx60i.yaml"
    program = next(k for k in start("nx60i").components if k["name"] == "mowbite_setup")["parameters"][-1]
    assert program["layer"].value == str(found) and program["shadowed"].value == ""
    assert program["driver_restarts"].value == "3"
    found.write_text("xcore_driver:\n  ros__parameters:\n    inputs.learn: false\n    max_correction_age: 20.0\n")
    device.write_text("xcore_driver:\n  ros__parameters:\n    max_correction_age: 25.0\n")
    built = start("nx60i")
    files = [value for value in built.node("xcore_driver")["parameters"] if isinstance(value, str)]
    assert files[-2:] == [str(found), str(device)]
    logic = next(k for k in built.components if k["name"] == "mowbite_logic")["parameters"]
    assert [value for value in logic if isinstance(value, str)][-2:] == [str(found), str(device)]
    program = next(k for k in built.components if k["name"] == "mowbite_setup")["parameters"][-1]
    assert program["layer"].value == str(found)
    assert program["shadowed"].value == "xcore_driver.max_correction_age"


def step_list(folder, text):
    (folder / "setup").mkdir()
    (folder / "setup" / "setup.yaml").write_text(text)


def test_the_step_list_goes_out_under_the_name_of_the_driver():
    with_list = 0
    for platform in layers.platforms():
        setup = layers.setup(platform)
        if not setup["steps"]:
            assert setup == {"platform": platform, "steps": "", "image_dir": ""}
            continue
        with_list += 1
        with open(layers.geometry(platform)) as f:
            name = yaml.safe_load(f)["mowbite_logic"]["ros__parameters"]["geometry"]["platform"]
        assert setup["platform"] == name
        assert json.loads(setup["steps"])["steps"]
        assert os.path.isdir(setup["image_dir"])
        assert stack.check_setup(setup, LOGGER) is None
    assert with_list >= 2


def test_a_platform_without_step_list_offers_none(tmp_path):
    geometry_file(tmp_path)
    setup = layers.setup("some", folder_of=str(tmp_path))
    assert setup == {"platform": "some", "steps": "", "image_dir": ""}
    assert stack.check_setup(setup, LOGGER) is None


def test_a_step_list_that_cannot_be_read_is_an_error(tmp_path):
    step_list(tmp_path, "catalog: 1\nsteps: [")
    geometry_file(tmp_path)
    with pytest.raises(ValueError, match="setup.yaml"):
        layers.setup("some", folder_of=str(tmp_path))
    (tmp_path / "setup" / "setup.yaml").write_text("catalog: 1\nsteps: []\n")
    geometry_file(tmp_path, "mowbite_logic: {ros__parameters: {geometry: {}}}")
    with pytest.raises(ValueError, match="geometry.platform is missing"):
        layers.setup("some", folder_of=str(tmp_path))
    os.remove(tmp_path / "geometry.yaml")
    with pytest.raises(OSError):
        layers.setup("some", folder_of=str(tmp_path))


def test_a_step_list_against_the_rules_halts_the_start(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(stack, "halt_file", lambda: str(tmp_path / "halt"))
    step_list(tmp_path, "catalog: 1\nsteps:\n  - {id: board, stage: device, check: nothing}\n")
    geometry_file(tmp_path)
    setup = layers.setup("some", folder_of=str(tmp_path))
    assert setup["platform"] == "some_mower"
    assert stack.check_setup(setup, LOGGER) is not None
    line = (tmp_path / "halt").read_text()
    assert line.startswith("MOWBITE_OS_HALT reason=PLATFORM_CONFIG data_schema=- image_schema=- The step list of the "
                           "platform: "), line
    assert "nothing" in line
    assert "PLATFORM_CONFIG" in capsys.readouterr().err
    # An image the list names has to be there and keep the rules of the node.
    step_list_text = "catalog: 1\nsteps:\n  - {id: board, stage: device, check: board, image: board.svg}\n"
    (tmp_path / "setup" / "setup.yaml").write_text(step_list_text)
    setup = layers.setup("some", folder_of=str(tmp_path))
    assert stack.check_setup(setup, LOGGER) is not None
    assert "board.svg: it is missing" in (tmp_path / "halt").read_text()


# system.info.update_path: a bundle tree says native, the image compose, a workspace nothing.
def test_update_path_from_the_bundle_or_the_image(tmp_path, monkeypatch):
    import ament_index_python.packages as packages
    tree = tmp_path / "rev"
    (tree / "install" / "mowbite_bringup").mkdir(parents=True)
    monkeypatch.setattr(packages, "get_package_prefix", lambda name: str(tree / "install" / name))
    monkeypatch.delenv("MOWBITE_UPDATE_PATH", raising=False)
    assert stack.update_path() == ""
    monkeypatch.setenv("MOWBITE_UPDATE_PATH", "compose")
    assert stack.update_path() == "compose"
    (tree / "bundle.json").write_text("{}")
    assert stack.update_path() == "native"
    monkeypatch.setenv("MOWBITE_UPDATE_PATH", "other")
    assert stack.update_path() == "native"
