"""platform.py of the Tron in the start of the stack: which processes start for hardware and the camera switches.

Run in a sourced workspace (stack.py of mowbite_bringup): python3 -m pytest /ws/src/platform_tron/test/test_platform.py
"""
import os
import sys

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, os.path.join(get_package_share_directory("mowbite_bringup"), "launch"))
import layers  # noqa: E402
import stack  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLATFORM_FILE = os.path.join(HERE, "..", "platform.yaml")
platform = stack.platform_module("tron", os.path.join(HERE, ".."))


def launched(tmp_path, monkeypatch, **switches):
    # The data schema needs the installed bridge; here only the choice of processes counts.
    monkeypatch.setattr(stack, "prepare_data_dir", lambda data_dir, logger: None)
    context = LaunchContext()
    values = {"data_dir": str(tmp_path / "data"), "config": layers.STACK,
              "platform_config": PLATFORM_FILE, "extra_config": PLATFORM_FILE,
              "geometry": "geometry.yaml", "mcu_port": "/dev/ttyS9", "gnss_port": "/dev/ttyS4",
              "imu_port": "/dev/ttyS1", "hardware": "true", "datum_lat": "nan", "datum_long": "nan",
              "datum_height": "0.0", "vision": "off", "stereo": "off", "right": "off", "right_node": "vision",
              "rear": "off", "camera_video": "",
              "mqtt_host": "127.0.0.1",
              "mqtt_port": "1883", "journal_dir": "", "oom_score_adj": ""}
    values.update(switches)
    context.launch_configurations.update(values)
    return stack.launch_setup(context, "tron", platform)


def executables(tmp_path, monkeypatch, **switches):
    return {a.node_executable for a in launched(tmp_path, monkeypatch, **switches) if isinstance(a, Node)}


def test_stereo_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_stereo_node" in executables(tmp_path, monkeypatch, stereo="on")
    assert "tron_stereo_node" not in executables(tmp_path, monkeypatch)
    assert "tron_vision_node" not in executables(tmp_path, monkeypatch, stereo="on")


def test_rear_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_rear_node" in executables(tmp_path, monkeypatch, rear="on")
    assert "tron_rear_node" not in executables(tmp_path, monkeypatch, stereo="on", vision="on", right="on")


def nodes(tmp_path, monkeypatch, **switches):
    return [a for a in launched(tmp_path, monkeypatch, **switches) if isinstance(a, Node)]


def test_right_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    started = executables(tmp_path, monkeypatch, right="on")
    assert "tron_vision_node" in started and "tron_side_node" not in started
    assert "tron_vision_node" not in executables(tmp_path, monkeypatch, stereo="on", rear="on")


# The camera AI on both sides: two instances of tron_vision, the right one in its namespace.
def test_left_and_right_are_two_instances(tmp_path, monkeypatch):
    made = []

    class Recorded(Node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            made.append(kwargs)

    monkeypatch.setattr(platform, "Node", Recorded)
    launched(tmp_path, monkeypatch, vision="on", right="on")
    vision = sorted((k["namespace"], k["name"], k["exec_name"]) for k in made if k["executable"] == "tron_vision_node")
    assert vision == [("/", "tron_vision", "tron_vision"), ("/right", "tron_vision", "tron_vision_right")]


# side: the live picture alone, never next to a second camera AI.
def test_right_node_side_starts_only_the_picture(tmp_path, monkeypatch):
    started = [n.node_executable for n in nodes(tmp_path, monkeypatch, vision="on", right="on", right_node="side")]
    assert "tron_side_node" in started and started.count("tron_vision_node") == 1


def test_another_right_node_starts_no_right_camera(tmp_path, monkeypatch):
    started = executables(tmp_path, monkeypatch, right="on", right_node="ai")
    assert not {"tron_vision_node", "tron_side_node", "mowbite_camera_node"} & started


# Each node names its process as stack.own_process does, so the OS ties an exit note to its cameras.
def test_each_camera_process_names_itself(tmp_path, monkeypatch):
    made = {}
    real = stack.own_process

    def own_process(make, name, *args, **kwargs):
        def keep(prefix):
            node = make(prefix)
            made[name] = node
            return node
        return real(keep, name, *args, **kwargs)

    params = {}

    class Recorded(Node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            params[kwargs.get("exec_name")] = kwargs.get("parameters")

    monkeypatch.setattr(stack, "own_process", own_process)
    monkeypatch.setattr(platform, "Node", Recorded)
    launched(tmp_path, monkeypatch, vision="on", stereo="on", right="on", rear="on")
    names = {"vision": ("tron_vision", "tron_vision_node"), "stereo": ("tron_stereo", "tron_stereo_node"),
             "right": ("tron_vision_right", "tron_vision_node"), "rear": ("tron_rear", "tron_rear_node")}
    for process, (exec_name, executable) in names.items():
        assert made[process].node_executable == executable
        assert params[exec_name][-1] == {"process": process}


def camera_params(tmp_path, monkeypatch, **switches):
    params = {}

    class Recorded(Node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            params[kwargs.get("exec_name")] = kwargs.get("parameters")[-1]

    monkeypatch.setattr(platform, "Node", Recorded)
    launched(tmp_path, monkeypatch, **switches)
    return params


def test_camera_video_off_turns_off_each_h264_stream(tmp_path, monkeypatch):
    params = camera_params(tmp_path, monkeypatch, vision="on", stereo="on", right="on", rear="on", camera_video="off")
    for exec_name in ("tron_vision", "tron_vision_right", "tron_rear"):
        assert params[exec_name]["video"] is False
    assert "video" not in params["tron_stereo"]
    side = camera_params(tmp_path, monkeypatch, right="on", right_node="side", camera_video="off")
    assert side["tron_side"]["video"] is False


# Unset, on or another value: video stays as config has it.
@pytest.mark.parametrize("value", ["", "on", "no"])
def test_camera_video_otherwise_keeps_the_config(tmp_path, monkeypatch, value):
    params = camera_params(tmp_path, monkeypatch, vision="on", right="on", rear="on", camera_video=value)
    assert params and all("video" not in p for p in params.values())


# With hardware:=read another host drives the blade; without hardware nothing reports it.
@pytest.mark.parametrize("hardware", ["read", "false"])
def test_no_camera_without_the_own_driver(tmp_path, monkeypatch, hardware):
    started = executables(tmp_path, monkeypatch, hardware=hardware, stereo="on", vision="on", right="on", rear="on")
    assert started and not {"tron_stereo_node", "tron_vision_node", "tron_side_node", "tron_rear_node"} & started


def test_another_value_starts_nothing(tmp_path, monkeypatch):
    assert not {"tron_stereo_node", "mowbite_camera_node"} & executables(tmp_path, monkeypatch, stereo="yes")
    assert not {"tron_side_node", "mowbite_camera_node"} & executables(tmp_path, monkeypatch, right="yes")
    assert not {"tron_rear_node", "mowbite_camera_node"} & executables(tmp_path, monkeypatch, rear="1")


# The camera service comes with any camera process, each has a picture on hw/cameras, and only then.
@pytest.mark.parametrize("switches", [{"stereo": "on"}, {"vision": "on"}, {"right": "on"}, {"rear": "on"},
                                      {"stereo": "on", "vision": "on", "right": "on", "rear": "on"}])
def test_the_camera_service_starts_with_a_camera(tmp_path, monkeypatch, switches):
    assert "mowbite_camera_node" in executables(tmp_path, monkeypatch, **switches)


def test_no_camera_service_without_a_camera(tmp_path, monkeypatch):
    assert "mowbite_camera_node" not in executables(tmp_path, monkeypatch)
    for hardware in ("read", "false"):
        started = executables(tmp_path, monkeypatch, hardware=hardware, stereo="on", vision="on", right="on",
                              rear="on")
        assert started and "mowbite_camera_node" not in started


# The check of the parameter files is asked for every node the launch can build, so none starts on a value left unset.
def test_the_parameter_check_knows_every_node(tmp_path, monkeypatch):
    built = set()
    checked = set()

    def recording(base):
        class Recorded(base):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                built.add(f"{(kwargs.get('namespace') or '/').strip('/')}/{kwargs['name']}".lstrip("/"))
        return Recorded

    for module in (platform, stack):
        monkeypatch.setattr(module, "Node", recording(Node))
        monkeypatch.setattr(module, "ComposableNode", recording(ComposableNode))
    monkeypatch.setattr(stack, "check_parameters", lambda files, nodes, logger: checked.update(nodes))
    for right_node in ("vision", "side"):
        launched(tmp_path, monkeypatch, vision="on", stereo="on", right="on", rear="on", right_node=right_node)
    assert built and built <= checked, built - checked
    assert checked == {*platform.NODES, *stack.started([])} | built



# tron_mcu is not started again after it ended: its exit note says down at once.
def test_the_driver_stays_down_after_it_ended(tmp_path, monkeypatch):
    calls = []
    real = stack.restarting

    def restarting(make, note, label, logger, restarts, *args, **kwargs):
        calls.append((note, label, restarts))
        return real(make, note, label, logger, restarts, *args, **kwargs)

    monkeypatch.setattr(stack, "restarting", restarting)
    launched(tmp_path, monkeypatch)
    assert (str(tmp_path / "data" / stack.DRIVER_EXIT_NOTE), "tron_mcu", 0) in calls


# The measured values of this Tron in data_dir come last, for the driver as for every node.
def test_the_file_of_the_mower_comes_last(tmp_path, monkeypatch):
    made = {}

    class Recorded(Node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            made[kwargs["name"]] = kwargs["parameters"]

    monkeypatch.setattr(platform, "Node", Recorded)
    (tmp_path / "data").mkdir()
    device = tmp_path / "data" / "tron.yaml"
    launched(tmp_path, monkeypatch)
    assert str(device) not in made["tron_mcu"]
    device.write_text("tron_mcu:\n  ros__parameters:\n    wheel_distance: 0.4\n")
    launched(tmp_path, monkeypatch)
    assert made["tron_mcu"][:4] == [layers.STACK, PLATFORM_FILE, PLATFORM_FILE, str(device)]
