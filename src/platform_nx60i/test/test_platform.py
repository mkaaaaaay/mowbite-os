"""platform.py of the NX60i in the start of the stack: the driver with its files, the battery voltages of the logic and
the latch of a restart.

Run in a sourced workspace (stack.py of mowbite_bringup, xcore_driver): python3 -m pytest
/ws/src/platform_nx60i/test/test_platform.py
"""
import json
import os
import sys

import pytest
from ament_index_python.packages import get_package_share_directory

BRINGUP = get_package_share_directory("mowbite_bringup")
sys.path[:0] = [os.path.join(BRINGUP, "test"), os.path.join(BRINGUP, "launch")]
import layers  # noqa: E402
import stack  # noqa: E402
from stack_probe import start  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
PLATFORM_FILE = os.path.join(HERE, "..", "platform.yaml")
platform = stack.platform_module("nx60i", os.path.join(HERE, ".."))


@pytest.fixture
def drivers(start, monkeypatch):  # noqa: F811
    """drivers(**arguments) builds the stack with the files of this tree and gives the parameters of the driver for
    its first start and for a restart; drivers.starts are the calls of stack.restarting."""
    monkeypatch.setattr(stack, "restarting", lambda make, *args, **kwargs: [make(0), make(1)])

    def launched(**arguments):
        built = start("nx60i", platform, platform_config=PLATFORM_FILE, driver_params="driver.yaml", **arguments)
        launched.starts = built.restarting
        return [k["parameters"] for k in built.nodes if k["name"] == platform.DRIVER]
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


def test_the_step_model_offers_the_nx60i_and_another_mower():
    steps = json.loads(layers.setup("nx60i", os.path.join(HERE, ".."))["steps"])["steps"]
    model = next(step for step in steps if step["id"] == "model")
    assert model["params"]["values"] == ["nx60i", "own"]
    # The NX60i keeps the values of its platform files.
    assert model["value_params"] == {}
