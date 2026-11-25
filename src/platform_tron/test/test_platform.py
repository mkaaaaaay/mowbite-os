"""platform.py of the Tron in the start of the stack: which processes start for hardware and the camera switches,
and the latch of a restarted driver.

Run in a sourced workspace (stack.py of mowbite_bringup): python3 -m pytest /ws/src/platform_tron/test/test_platform.py
"""
import functools
import os
import sys

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

BRINGUP = get_package_share_directory("mowbite_bringup")
sys.path[:0] = [os.path.join(BRINGUP, "test"), os.path.join(BRINGUP, "launch")]
import layers  # noqa: E402
import stack  # noqa: E402
from stack_probe import start  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
PLATFORM_FILE = os.path.join(HERE, "..", "platform.yaml")
platform = stack.platform_module("tron", os.path.join(HERE, ".."))
CAMERAS = {"vision": "on", "stereo": "on", "right": "on", "rear": "on"}


@pytest.fixture
def launched(start):  # noqa: F811
    """launched(**arguments): the stack of the Tron built with the files of this tree."""
    return functools.partial(start, "tron", platform, platform_config=PLATFORM_FILE)


@pytest.fixture
def executables(launched):
    return lambda **arguments: launched(**arguments).executables()


@pytest.fixture
def camera_params(launched):
    """camera_params(**arguments): the last parameters of each camera process by its name."""
    return lambda **arguments: {k["exec_name"]: k["parameters"][-1] for k in launched(**arguments).nodes
                                if k.get("package") == "tron_vision"}


@pytest.mark.parametrize("hardware", ["true", "read"])
def test_the_driver_restarts_latched(start, monkeypatch, tmp_path, hardware):  # noqa: F811
    monkeypatch.setattr(stack, "restarting", lambda make, *args, **kwargs: [make(0), make(1)])
    built = start("tron", platform, platform_config=PLATFORM_FILE, hardware=hardware)
    first, second = [k["parameters"][-1] for k in built.nodes if k["name"] == platform.DRIVER]
    assert first["start_latched"] is False and second["start_latched"] is True
    assert (str(tmp_path / stack.DRIVER_EXIT_NOTE), "tron_mcu", platform.DRIVER_RESTARTS) in built.restarting
    assert platform.DRIVER_RESTARTS > 0


def test_stereo_starts_with_the_hardware_and_its_switch(executables):
    assert "tron_stereo_node" in executables(stereo="on")
    assert "tron_stereo_node" not in executables()
    assert "tron_vision_node" not in executables(stereo="on")


def test_rear_starts_with_the_hardware_and_its_switch(executables):
    assert "tron_rear_node" in executables(rear="on")
    assert "tron_rear_node" not in executables(stereo="on", vision="on", right="on")


def test_right_starts_with_the_hardware_and_its_switch(executables):
    started = executables(right="on")
    assert "tron_vision_node" in started and "tron_side_node" not in started
    assert "tron_vision_node" not in executables(stereo="on", rear="on")


# The camera AI on both sides: two instances of tron_vision, the right one in its namespace.
def test_left_and_right_are_two_instances(launched):
    made = launched(vision="on", right="on").nodes
    vision = sorted((k["namespace"], k["name"], k["exec_name"]) for k in made if k["executable"] == "tron_vision_node")
    assert vision == [("/", "tron_vision", "tron_vision"), ("/right", "tron_vision", "tron_vision_right")]


# side: the live picture alone, never next to a second camera AI.
def test_right_node_side_starts_only_the_picture(executables):
    started = executables(vision="on", right="on", right_node="side")
    assert "tron_side_node" in started and started.count("tron_vision_node") == 1


def test_another_right_node_starts_no_right_camera(executables):
    started = executables(right="on", right_node="ai")
    assert not {"tron_vision_node", "tron_side_node", "mowbite_camera_node"} & set(started)


# Each node names its process as stack.own_process does, so the OS ties an exit note to its cameras.
def test_each_camera_process_names_itself(camera_params, monkeypatch):
    made = {}
    real = stack.own_process

    def own_process(make, name, *args, **kwargs):
        def keep(prefix):
            node = make(prefix)
            made[name] = node
            return node
        return real(keep, name, *args, **kwargs)

    monkeypatch.setattr(stack, "own_process", own_process)
    params = camera_params(**CAMERAS)
    names = {"vision": ("tron_vision", "tron_vision_node"), "stereo": ("tron_stereo", "tron_stereo_node"),
             "right": ("tron_vision_right", "tron_vision_node"), "rear": ("tron_rear", "tron_rear_node")}
    extra = {"stereo": {"wheel_radius": 0.1125}}
    for process, (exec_name, executable) in names.items():
        assert made[process].node_executable == executable
        assert params[exec_name] == {"process": process, **extra.get(process, {})}


def test_camera_video_off_turns_off_each_h264_stream(camera_params):
    params = camera_params(**CAMERAS, camera_video="off")
    for exec_name in ("tron_vision", "tron_vision_right", "tron_rear"):
        assert params[exec_name]["video"] is False
    assert "video" not in params["tron_stereo"]
    side = camera_params(right="on", right_node="side", camera_video="off")
    assert side["tron_side"]["video"] is False


# Unset, on or another value: video stays as config has it.
@pytest.mark.parametrize("value", ["", "on", "no"])
def test_camera_video_otherwise_keeps_the_config(camera_params, value):
    params = camera_params(vision="on", right="on", rear="on", camera_video=value)
    assert params and all("video" not in p for p in params.values())


# With hardware:=read another host drives the blade; without hardware nothing reports it.
@pytest.mark.parametrize("hardware", ["read", "false"])
def test_no_camera_without_the_own_driver(executables, hardware):
    started = executables(hardware=hardware, **CAMERAS)
    assert started
    assert not {"tron_stereo_node", "tron_vision_node", "tron_side_node", "tron_rear_node", "mowbite_camera_node"} \
        & set(started)


# The buttons on top start only with the own driver: with hardware:=read another host owns the mower.
def test_the_buttons_start_with_the_own_driver_only(executables):
    assert "tron_buttons_node" in executables()
    assert "tron_buttons_node" not in executables(hardware="read")
    assert "tron_buttons_node" not in executables(hardware="false")


def test_another_value_starts_nothing(executables):
    assert not {"tron_stereo_node", "mowbite_camera_node"} & set(executables(stereo="yes"))
    assert not {"tron_side_node", "mowbite_camera_node"} & set(executables(right="yes"))
    assert not {"tron_rear_node", "mowbite_camera_node"} & set(executables(rear="1"))


# The camera service comes with any camera process, each has a picture on hw/cameras, and only then.
@pytest.mark.parametrize("switches", [{"stereo": "on"}, {"vision": "on"}, {"right": "on"}, {"rear": "on"}, CAMERAS])
def test_the_camera_service_starts_with_a_camera(executables, switches):
    assert "mowbite_camera_node" in executables(**switches)


def test_no_camera_service_without_a_camera(executables):
    assert "mowbite_camera_node" not in executables()


# The check of the parameter files is asked for every node the launch can build, so none starts on a value left unset.
def test_the_parameter_check_knows_every_node(launched):
    starts = [launched(**CAMERAS, right_node=right_node) for right_node in ("vision", "side")]
    built = set().union(*(s.names() for s in starts))
    checked = set().union(*(s.checked for s in starts))
    assert built and built <= checked, built - checked
    assert checked == set(stack.checked_nodes([], platform)) | built


# The measured values of this Tron in data_dir come last, for the driver as for every node.
def test_the_file_of_the_mower_comes_last(launched, tmp_path):
    def driver_files():
        return launched().node(platform.DRIVER)["parameters"]

    device = tmp_path / "tron.yaml"
    assert str(device) not in driver_files()
    device.write_text("tron_mcu:\n  ros__parameters:\n    wheel_distance: 0.4\n")
    assert driver_files()[:4] == [layers.STACK, PLATFORM_FILE, PLATFORM_FILE, str(device)]


# The driver names the stereo camera as the obstacle sensor by the id the camera reports itself under.
def test_the_obstacle_source_is_the_stereo_camera():
    with open(PLATFORM_FILE) as f:
        values = yaml.safe_load(f)
    stereo = values["tron_stereo"]["ros__parameters"]
    assert stereo["obstacles"] is True
    assert values["tron_mcu"]["ros__parameters"]["obstacle_sources"] == [stereo["camera_id"]]


# T9 (2026-11-19): the MCU ends a setpoint ~0.23 s after a heartbeat that comes alone after it, else it keeps it ~3.6 s.
def test_a_heartbeat_follows_each_setpoint_as_measured():
    with open(PLATFORM_FILE) as f:
        mcu = yaml.safe_load(f)["tron_mcu"]["ros__parameters"]
    assert mcu["speed_heartbeat"] == "later"
    assert "heartbeat_after_speed" not in mcu


# N7 (2026-11-24): a Tron whose motor boards run firmware outside the measured stands does not move.
def test_the_driver_checks_the_firmware_against_the_measured_stands():
    with open(PLATFORM_FILE) as f:
        mcu = yaml.safe_load(f)["tron_mcu"]["ros__parameters"]
    assert mcu["cutter_firmware"] == ["0.6.36"]
    assert mcu["chassis_firmware"] == ["0.6.34"]
    assert "expected_cutter_version" not in mcu and "expected_chassis_version" not in mcu
