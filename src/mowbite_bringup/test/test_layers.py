"""layers.py and the parameter files: stack.yaml leaves values to the platform, every platform sets all of them for
its nodes, in sections of its nodes only and under names stack.yaml knows. A value left over halts the start
(stack.check_parameters).

Run in a sourced workspace (nx60i.launch.py looks for xcore_driver): python3 -m pytest
/ws/src/mowbite_bringup/test/test_layers.py
"""
import importlib.util
import logging
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.normpath(os.path.join(HERE, "..", "launch"))
sys.path.insert(0, LAUNCH)
import layers  # noqa: E402
import stack  # noqa: E402


def launch_file(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(LAUNCH, f"{name}.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STACK_NODES = stack.started(stack.components([], "", "", "127.0.0.1", "1883", ""))
# The nodes a platform can start.
NODES = {platform: [*STACK_NODES, *launch_file(platform).NODES] for platform in ("nx60i", "tron")}
PLATFORMS = sorted(NODES)
LOGGER = logging.getLogger("test_layers")


def test_the_stack_alone_leaves_values_to_the_platform():
    missing = layers.unset([layers.STACK], STACK_NODES)
    assert "mowbite_logic.battery_empty_voltage" in missing
    assert "mowbite_safety.tilt_limit" in missing
    # A value under /** is missing for every node.
    assert {f"{node}.journal_platform" for node in STACK_NODES} <= set(missing)


@pytest.mark.parametrize("platform", PLATFORMS)
def test_a_platform_sets_every_value_left_to_it(platform):
    assert layers.unset(layers.files(platform), NODES[platform]) == []


@pytest.mark.parametrize("platform", PLATFORMS)
def test_a_platform_file_has_sections_of_its_nodes_only(platform):
    for section, _ in layers.sections(layers.files(platform)[1]):
        assert any(layers.matches(section, node) for node in NODES[platform]), section


# Where stack.yaml has a section, it names every parameter of it: a platform file adds none there.
@pytest.mark.parametrize("platform", PLATFORMS)
def test_a_platform_file_sets_only_known_values(platform):
    known = {}
    for section, values in layers.sections(layers.STACK):
        known.setdefault(section, set()).update(values)
    for section, values in layers.sections(layers.files(platform)[1]):
        if section in known:
            assert set(values) <= known[section], (section, set(values) - known[section])


def test_a_placeholder_is_no_value_of_a_platform_file():
    for platform in PLATFORMS:
        for section, values in layers.sections(layers.files(platform)[1]):
            assert layers.PLACEHOLDER not in values.values(), section


def test_sections_match_by_name_and_wildcard():
    assert layers.matches("/**", "mowbite_logic") and layers.matches("/**", "right/tron_vision")
    assert layers.matches("mowbite_logic", "mowbite_logic") and layers.matches("/mowbite_logic", "mowbite_logic")
    assert not layers.matches("mowbite_logic", "mowbite_logic_2")
    assert layers.matches("/**/tron_vision", "tron_vision") and layers.matches("/**/tron_vision", "right/tron_vision")
    assert layers.matches("/right/tron_vision", "right/tron_vision")
    assert not layers.matches("/right/tron_vision", "tron_vision")
    assert layers.matches("/*/tron_vision", "right/tron_vision") and not layers.matches("/*/tron_vision", "tron_vision")


def write(path, text):
    path.write_text(text)
    return str(path)


def test_a_later_file_and_a_later_section_win(tmp_path):
    first = write(tmp_path / "a.yaml", "/**:\n  ros__parameters:\n    x: 1\n    y: 1\nnode:\n  ros__parameters:\n"
                                       "    y: 2\n    nested:\n      z: 3\n")
    second = write(tmp_path / "b.yaml", "/**:\n  ros__parameters:\n    y: 4\n")
    assert layers.parameters([first], "node") == {"x": 1, "y": 2, "nested.z": 3}
    assert layers.parameters([first, second], "node") == {"x": 1, "y": 4, "nested.z": 3}
    assert layers.parameters([first, second], "other") == {"x": 1, "y": 4}


def test_a_value_is_left_over_only_for_the_nodes_asked(tmp_path):
    base = write(tmp_path / "stack.yaml", "/**:\n  ros__parameters:\n    limit: <platform>\nlogic:\n"
                                          "  ros__parameters:\n    voltage: <platform>\n    speed: <platform>\n"
                                          "light:\n  ros__parameters:\n    device: <platform>\n")
    platform = write(tmp_path / "mower.yaml", "/**:\n  ros__parameters:\n    limit: 1.0\nlogic:\n  ros__parameters:\n"
                                              "    speed: 0.3\n")
    # The light is no node of this platform, its device may stay open.
    assert layers.unset([base, platform], ["logic", "safety"]) == ["logic.voltage"]
    assert layers.unset([base, platform], ["logic", "light"]) == ["light.device", "logic.voltage"]
    done = write(tmp_path / "done.yaml", "logic:\n  ros__parameters:\n    voltage: 24.0\n")
    assert layers.unset([base, platform, done], ["logic", "safety"]) == []


@pytest.fixture
def halt_file(tmp_path, monkeypatch):
    path = tmp_path / "halt"
    monkeypatch.setenv("MOWBITE_HALT_FILE", str(path))
    return path


def test_a_complete_platform_goes_on(halt_file):
    assert stack.check_parameters(layers.files("tron"), NODES["tron"], LOGGER) is None
    assert not halt_file.exists()


def test_a_value_left_over_halts_with_its_names(halt_file, capsys):
    actions = stack.check_parameters([layers.STACK], STACK_NODES, LOGGER)
    assert len(actions) == 1
    line = halt_file.read_text()
    assert line.startswith("MOWBITE_OS_HALT reason=PLATFORM_CONFIG data_schema=- image_schema=- The platform does not "
                           "set ")
    assert "mowbite_logic.battery_empty_voltage" in line
    assert capsys.readouterr().err == line


@pytest.mark.parametrize("content", ["/**: [", "- a\n- b\n", "node: 3\n", None])
def test_a_file_that_cannot_be_read_halts(tmp_path, halt_file, content):
    path = tmp_path / "bad.yaml"
    if content is not None:
        path.write_text(content)
    assert len(stack.check_parameters([str(path)], ["node"], LOGGER)) == 1
    assert halt_file.read_text().startswith("MOWBITE_OS_HALT reason=PLATFORM_CONFIG data_schema=- image_schema=- "
                                            "A parameter file cannot be read: ")
