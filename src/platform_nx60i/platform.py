"""The YardForce NX60i (OpenMower v2 carrier, xCore board on Ethernet) for mowbite.launch.py: xcore_driver in a process
of its own, so that a crash of any other node leaves its watchdogs running (G25); it passes IMU and GNSS of the board
on. A driver that ends outside the shutdown starts again at most DRIVER_RESTARTS times, each new one latched from its
first frame (start_latched): a new driver would otherwise take the board without a latch (M4).
"""
import math
import os
import sys

from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "launch"))
import logic_params  # noqa: E402

# The NX60i has no node beside its driver and those of the stack.
NODES = ()
DRIVER = "xcore_driver"
DRIVER_RESTARTS = 3


def arguments():
    return [
        DeclareLaunchArgument("driver_params", default_value=PathJoinSubstitution(
            [FindPackageShare("xcore_driver"), "config", "nx60i.yaml"])),
        # The datum of the garden; empty: the one of the file of this mower in data_dir, without any no pose is
        # published.
        DeclareLaunchArgument("datum_lat", default_value=""),
        DeclareLaunchArgument("datum_long", default_value=""),
        DeclareLaunchArgument("datum_height", default_value=""),
        # The network of the board; empty: the files of the driver.
        DeclareLaunchArgument("interface", default_value=""),
        DeclareLaunchArgument("bind_ip", default_value=""),
        DeclareLaunchArgument("announcement_port", default_value=""),
    ]


# A number argument, None when empty; anything else that is no finite number ends the launch before a node starts.
def number(value, name):
    if value == "":
        return None
    try:
        x = float(value)
    except ValueError:
        x = math.nan
    if not math.isfinite(x):
        raise RuntimeError(f"{name}:={value} is no number")
    return x


def parts(start):
    arg = start.arg
    # The driver's file, then what the setup program found for this mower (the table of its inputs) and its file kept
    # by hand in data_dir (ticks_per_meter, wheel_distance, the values of the safety measurement, the datum), if there
    # are any, then what is given here: the battery voltages of the logic and the arguments.
    driver_files = [arg("driver_params")] + [path for path in (start.setup_file, start.device) if path]
    overrides = {"hold_file": start.hold_file, **logic_params.battery_voltages(start.parameters("mowbite_logic"))}
    for name in ("datum_lat", "datum_long", "datum_height"):
        value = number(arg(name), name)
        if value is not None:
            overrides[name] = value
    for name in ("interface", "bind_ip"):
        if arg(name):
            overrides[name] = arg(name)
    if arg("announcement_port"):
        overrides["announcement_port"] = int(arg("announcement_port"))

    def driver(restart):
        return Node(package="xcore_driver", executable="xcore_driver_node", name=DRIVER, namespace="/",
                    parameters=[*driver_files, {**overrides, "start_latched": restart > 0}], output="screen")

    return start.Parts(driver=driver)
