"""For the tests of the start: the stack of a platform built without running anything, with what it was built from.

A test file imports the fixture, with the launch folder of mowbite_bringup on its path:
from stack_probe import start  # noqa: F401
"""
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

import stack


class Built:
    """One start of the stack."""

    def __init__(self):
        # The actions launch_setup returned.
        self.actions = []
        # The arguments of every Node and of every ComposableNode built, in their order.
        self.nodes = []
        self.components = []
        # The nodes the parameter check was asked for.
        self.checked = set()
        # The calls of stack.restarting as (note, label, restarts).
        self.restarting = []

    def node(self, name):
        """The arguments of the first Node built with that name."""
        return next(k for k in self.nodes if k["name"] == name)

    def executables(self):
        """The executables among the actions."""
        return [a.node_executable for a in self.actions if isinstance(a, Node)]

    def names(self):
        """Every node built, with its namespace where it has one."""
        return {f"{(k.get('namespace') or '/').strip('/')}/{k['name']}".lstrip("/")
                for k in [*self.nodes, *self.components]}


def recording(base, into):
    class Recorded(base):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            into.append(kwargs)
    return Recorded


@pytest.fixture
def start(tmp_path, monkeypatch):
    """start(platform, module=None, **arguments) builds the stack of the platform (module: its platform.py, the
    installed one by default) with data_dir tmp_path, the defaults of its launch arguments in an environment without
    MOWBITE_* and these arguments on top: a Built."""
    for name in [n for n in os.environ if n.startswith("MOWBITE_")]:
        monkeypatch.delenv(name)
    # The data schema needs the installed bridge.
    monkeypatch.setattr(stack, "prepare_data_dir", lambda data_dir, logger, besides=(): None)

    def started(platform, module=None, **arguments):
        module = module or stack.platform_module(platform)
        built = Built()
        check, restarting = stack.check_parameters, stack.restarting

        def checked(files, nodes, logger):
            built.checked.update(nodes)
            return check(files, nodes, logger)

        def restarted(make, note, label, logger, restarts, *args, **kwargs):
            built.restarting.append((note, label, restarts))
            return restarting(make, note, label, logger, restarts, *args, **kwargs)

        with monkeypatch.context() as patch:
            for owner in (module, stack):
                for name, base, into in (("Node", Node, built.nodes),
                                         ("ComposableNode", ComposableNode, built.components)):
                    if hasattr(owner, name):
                        patch.setattr(owner, name, recording(base, into))
            patch.setattr(stack, "check_parameters", checked)
            patch.setattr(stack, "restarting", restarted)
            context = LaunchContext()
            context.launch_configurations.update({"data_dir": str(tmp_path), **arguments})
            for action in stack.all_arguments(platform, module):
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            built.actions = stack.launch_setup(context, platform, module, True)
        return built
    return started
