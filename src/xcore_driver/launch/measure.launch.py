"""Only xcore_driver_node, for the safety measurement of a mower with an xCore board (no logic, bridge or MQTT).

Parameters: the platform file (default nx60i.yaml of this package), then the parameter file of the mower if given
(datum, measured values), then the overrides given here. respawn:=true starts a killed or ended driver again after
respawn_delay s (M4, M7, M8 with the driver killed).

  ros2 launch xcore_driver measure.launch.py mower_params:=/data/nx60i.yaml respawn:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = {
    "params": ("", "platform parameter file, empty = nx60i.yaml of xcore_driver"),
    "mower_params": ("", "parameter file of this mower, read after the platform file"),
    "interface": ("", "network interface of the board, empty = from the files"),
    "bind_ip": ("", "address of the host at the board, empty = from the files"),
    "announcement_port": ("", "port of the board's announcements, empty = from the files"),
    "allow_blade": ("false", "true only for M7, on the user's own word"),
    "respawn": ("false", "start the driver again after it ended"),
    "respawn_delay": ("3.0", "s before a new start"),
}


def driver(context):
    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    files = [value["params"] or os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config",
                                             "nx60i.yaml")]
    if value["mower_params"]:
        files.append(value["mower_params"])
    overrides = {"allow_blade": value["allow_blade"].lower() == "true"}
    for name in ("interface", "bind_ip"):
        if value[name]:
            overrides[name] = value[name]
    if value["announcement_port"]:
        overrides["announcement_port"] = int(value["announcement_port"])
    return [Node(package="xcore_driver", executable="xcore_driver_node", name="xcore_driver", output="screen",
                 namespace="/", parameters=files + [overrides], respawn=value["respawn"].lower() == "true",
                 respawn_delay=float(value["respawn_delay"]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, (default, text) in ARGUMENTS.items()] + [OpaqueFunction(function=driver)])
