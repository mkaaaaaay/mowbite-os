"""tron.launch.py: which processes start for hardware and the camera switches.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_tron_launch.py
"""
import importlib.util
import os

import pytest
from launch import LaunchContext
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("tron_launch", os.path.join(HERE, "..", "launch", "tron.launch.py"))
tron_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tron_launch)


def launched(tmp_path, monkeypatch, **switches):
    # The data schema needs the installed bridge; here only the choice of processes counts.
    monkeypatch.setattr(tron_launch.stack, "prepare_data_dir", lambda data_dir, logger: None)
    context = LaunchContext()
    values = {"data_dir": str(tmp_path / "data"), "config": "tron.yaml", "extra_config": "tron.yaml",
              "geometry": "geometry.yaml", "mcu_port": "/dev/ttyS9", "gnss_port": "/dev/ttyS4",
              "imu_port": "/dev/ttyS1", "hardware": "true", "datum_lat": "nan", "datum_long": "nan",
              "datum_height": "0.0", "vision": "off", "stereo": "off", "right": "off", "rear": "off",
              "mqtt_host": "127.0.0.1",
              "mqtt_port": "1883", "journal_dir": "", "oom_score_adj": ""}
    values.update(switches)
    context.launch_configurations.update(values)
    return tron_launch.launch_setup(context)


def executables(tmp_path, monkeypatch, **switches):
    return {a.node_executable for a in launched(tmp_path, monkeypatch, **switches) if isinstance(a, Node)}


def test_stereo_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_stereo_node" in executables(tmp_path, monkeypatch, stereo="on")
    assert "tron_stereo_node" not in executables(tmp_path, monkeypatch)
    assert "tron_vision_node" not in executables(tmp_path, monkeypatch, stereo="on")


def test_rear_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_rear_node" in executables(tmp_path, monkeypatch, rear="on")
    assert "tron_rear_node" not in executables(tmp_path, monkeypatch, stereo="on", vision="on", right="on")


def test_right_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_side_node" in executables(tmp_path, monkeypatch, right="on")
    assert "tron_side_node" not in executables(tmp_path, monkeypatch, stereo="on", vision="on", rear="on")


# Each node names its process as stack.own_process does, so the OS ties an exit note to its cameras.
def test_each_camera_process_names_itself(tmp_path, monkeypatch):
    made = {}
    real = tron_launch.stack.own_process

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

    monkeypatch.setattr(tron_launch.stack, "own_process", own_process)
    monkeypatch.setattr(tron_launch, "Node", Recorded)
    launched(tmp_path, monkeypatch, vision="on", stereo="on", right="on", rear="on")
    names = {"vision": "tron_vision", "stereo": "tron_stereo", "right": "tron_side", "rear": "tron_rear"}
    for process, exec_name in names.items():
        assert made[process].node_executable == exec_name + "_node"
        assert params[exec_name][-1] == {"process": process}


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
