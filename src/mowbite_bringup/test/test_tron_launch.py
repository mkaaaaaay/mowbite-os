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


def executables(tmp_path, monkeypatch, **switches):
    # The data schema needs the installed bridge; here only the choice of processes counts.
    monkeypatch.setattr(tron_launch.stack, "prepare_data_dir", lambda data_dir, logger: None)
    context = LaunchContext()
    values = {"data_dir": str(tmp_path / "data"), "config": "tron.yaml", "extra_config": "tron.yaml",
              "geometry": "geometry.yaml", "mcu_port": "/dev/ttyS9", "gnss_port": "/dev/ttyS4", "imu_port": "/dev/ttyS1",
              "hardware": "true", "datum_lat": "nan", "datum_long": "nan", "datum_height": "0.0", "vision": "off",
              "stereo": "off", "mqtt_host": "127.0.0.1", "mqtt_port": "1883", "journal_dir": "", "oom_score_adj": ""}
    values.update(switches)
    context.launch_configurations.update(values)
    return {a.node_executable for a in tron_launch.launch_setup(context) if isinstance(a, Node)}


def test_stereo_starts_with_the_hardware_and_its_switch(tmp_path, monkeypatch):
    assert "tron_stereo_node" in executables(tmp_path, monkeypatch, stereo="on")
    assert "tron_stereo_node" not in executables(tmp_path, monkeypatch)
    assert "tron_vision_node" not in executables(tmp_path, monkeypatch, stereo="on")


# With hardware:=read another host drives the blade; without hardware nothing reports it.
@pytest.mark.parametrize("hardware", ["read", "false"])
def test_no_stereo_without_the_own_driver(tmp_path, monkeypatch, hardware):
    started = executables(tmp_path, monkeypatch, hardware=hardware, stereo="on", vision="on")
    assert started and not {"tron_stereo_node", "tron_vision_node"} & started


def test_another_value_starts_nothing(tmp_path, monkeypatch):
    assert "tron_stereo_node" not in executables(tmp_path, monkeypatch, stereo="yes")
