"""platform.py of the NX60i in the start of the stack: the driver with its files, the battery voltages of the logic and
the latch of a restart.

Run in a sourced workspace (stack.py of mowbite_bringup, xcore_driver): python3 -m pytest
/ws/src/platform_nx60i/test/test_platform.py
"""
import os
import sys

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory("mowbite_bringup"), "launch"))
import layers  # noqa: E402
import stack  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLATFORM_FILE = os.path.join(HERE, "..", "platform.yaml")
platform = stack.platform_module("nx60i", os.path.join(HERE, ".."))


@pytest.fixture
def drivers(tmp_path, monkeypatch):
    """launched(**arguments) starts the stack and gives the parameters of each driver built, by its start number."""
    made = []
    starts = []

    class Recorded(Node):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            made.append(kwargs["parameters"])

    def restarting(make, note, label, logger, restarts, *args, **kwargs):
        starts.append((note, label, restarts))
        return [make(0), make(1)]

    # The data schema needs the installed bridge; here only the driver counts.
    monkeypatch.setattr(stack, "prepare_data_dir", lambda data_dir, logger: None)
    monkeypatch.setattr(stack, "restarting", restarting)
    monkeypatch.setattr(platform, "Node", Recorded)

    def launched(**arguments):
        del made[:]
        context = LaunchContext()
        values = {"data_dir": str(tmp_path), "config": layers.STACK, "platform_config": PLATFORM_FILE,
                  "extra_config": PLATFORM_FILE, "geometry": "geometry.yaml", "driver_params": "driver.yaml",
                  "datum_lat": "", "datum_long": "", "datum_height": "", "interface": "", "bind_ip": "",
                  "announcement_port": "", "mqtt_host": "127.0.0.1", "mqtt_port": "1883", "journal_dir": "",
                  "oom_score_adj": ""}
        values.update(arguments)
        context.launch_configurations.update(values)
        stack.launch_setup(context, "nx60i", platform)
        return made
    launched.starts = starts
    return launched


def test_the_driver_restarts_latched(drivers, tmp_path):
    first, second = drivers()
    assert first[-1]["start_latched"] is False and second[-1]["start_latched"] is True
    # The driver comes first, before the container and the processes beside it.
    assert drivers.starts[0] == (str(tmp_path / stack.DRIVER_EXIT_NOTE), "xcore_driver", platform.DRIVER_RESTARTS)
    assert platform.DRIVER_RESTARTS > 0


def test_the_driver_gets_the_battery_of_the_logic_and_the_hold(drivers, tmp_path):
    values = drivers()[0][-1]
    logic = layers.parameters(layers.files("nx60i"), "mowbite_logic")
    for name in platform.logic_params.BATTERY_VOLTAGES:
        assert values[name] == logic[name]
    assert values["hold_file"] == str(tmp_path / stack.data_store.HOLD_FILE)
    assert not {"datum_lat", "interface", "bind_ip", "announcement_port"} & set(values)


def test_the_file_of_the_mower_comes_after_the_one_of_the_driver(drivers, tmp_path):
    assert drivers()[0][:-1] == ["driver.yaml"]
    device = tmp_path / "nx60i.yaml"
    device.write_text("xcore_driver:\n  ros__parameters:\n    wheel_distance: 0.36\nmowbite_logic:\n"
                      "  ros__parameters:\n    battery_empty_voltage: 24.9\n")
    first = drivers()[0]
    assert first[:-1] == ["driver.yaml", str(device)]
    # The logic and the board keep the same voltage.
    assert first[-1]["battery_empty_voltage"] == 24.9


def test_arguments_go_to_the_driver(drivers):
    values = drivers(datum_lat="52.0", datum_long="9.0", datum_height="80", interface="lo", bind_ip="127.0.0.1",
                     announcement_port="4242")[0][-1]
    assert (values["datum_lat"], values["datum_long"], values["datum_height"]) == (52.0, 9.0, 80.0)
    assert (values["interface"], values["bind_ip"], values["announcement_port"]) == ("lo", "127.0.0.1", 4242)


@pytest.mark.parametrize("value", ["north", "nan", "inf"])
def test_a_datum_that_is_no_number_ends_the_start(drivers, value):
    with pytest.raises(RuntimeError):
        drivers(datum_lat=value)
