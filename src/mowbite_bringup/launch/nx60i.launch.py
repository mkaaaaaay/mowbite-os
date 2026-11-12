# MowBite OS on the YardForce NX60i (OpenMower v2 carrier, xCore board on Ethernet): xcore_driver in a process of its
# own, so that a crash of any other node leaves its watchdogs running (G25), the container of stack.py without
# tron_gnss and tron_imu (the driver passes IMU and GNSS of the board on) and the MowBite Planner. A driver that ends
# outside the shutdown starts again like the container (stack.restarting), at most DRIVER_RESTARTS times, each new one
# latched from its first frame (start_latched): a new driver would otherwise take the board without a latch (M4).
import math
import os
import sys

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# stack.py and data_store.py lie next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import data_store  # noqa: E402
import stack  # noqa: E402

sys.path.insert(0, os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "launch"))
import logic_params  # noqa: E402

DRIVER_RESTARTS = 3
# In data_dir: how the driver ended, written before it starts again.
DRIVER_EXIT_NOTE = "driver_exit.json"


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


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    logger = launch.logging.get_logger("nx60i.launch")
    data_dir = arg("data_dir")
    halt = stack.prepare_data_dir(data_dir, logger)
    if halt:
        return halt
    files = [arg("config"), arg("platform_config"), arg("extra_config")]

    # The driver's file, then the file of this mower (measured values, datum), then what is given here: the battery
    # voltages of the logic and the arguments.
    driver_files = [arg("driver_params")] + ([arg("mower_params")] if arg("mower_params") else [])
    overrides = {"hold_file": os.path.join(data_dir, data_store.HOLD_FILE) if data_dir else "",
                 **logic_params.battery_voltages(files)}
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
        return Node(package="xcore_driver", executable="xcore_driver_node", name="xcore_driver", namespace="/",
                    parameters=[*driver_files, {**overrides, "start_latched": restart > 0}], output="screen")

    note = os.path.join(data_dir, DRIVER_EXIT_NOTE) if data_dir else None
    journal = stack.journal_dir(arg("journal_dir"), data_dir, logger)
    nodes = stack.components(files, arg("geometry"), data_dir, arg("mqtt_host"), arg("mqtt_port"), journal)
    return [*stack.restarting(driver, note, "xcore_driver", logger, DRIVER_RESTARTS),
            *stack.container_and_planner(nodes, data_dir, files, logger, arg("oom_score_adj"))]


def generate_launch_description():
    bringup = FindPackageShare("mowbite_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([bringup, "config", "tron.yaml"])),
        # What differs on this platform, on top of config.
        DeclareLaunchArgument("platform_config", default_value=PathJoinSubstitution([bringup, "config", "nx60i.yaml"])),
        # Values on top of both, e.g. those of a test; by default the platform file once more.
        DeclareLaunchArgument("extra_config", default_value=LaunchConfiguration("platform_config")),
        DeclareLaunchArgument("geometry", default_value=PathJoinSubstitution(
            [bringup, "config", "geometry", "nx60i.yaml"])),
        DeclareLaunchArgument("driver_params", default_value=PathJoinSubstitution(
            [FindPackageShare("xcore_driver"), "config", "nx60i.yaml"])),
        # The parameter file of this mower in data_dir (ticks_per_meter, wheel_distance, the values of the safety
        # measurement, the datum); empty: the platform file alone.
        DeclareLaunchArgument("mower_params", default_value=""),
        # The datum of the garden; empty: the one of mower_params, without any no pose is published.
        DeclareLaunchArgument("datum_lat", default_value=""),
        DeclareLaunchArgument("datum_long", default_value=""),
        DeclareLaunchArgument("datum_height", default_value=""),
        # The network of the board; empty: the platform file.
        DeclareLaunchArgument("interface", default_value=""),
        DeclareLaunchArgument("bind_ip", default_value=""),
        DeclareLaunchArgument("announcement_port", default_value=""),
        # Without it nothing is kept and no update can hold the driver; the compose file of the mower sets it.
        DeclareLaunchArgument("data_dir", default_value=""),
        *stack.arguments(),
        OpaqueFunction(function=launch_setup),
    ])
