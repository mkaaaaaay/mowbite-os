"""Only xcore_driver_node, for the safety measurement of a mower with an xCore board (no logic, bridge or MQTT).

Parameters: the file of the driver (default nx60i.yaml of this package), then the parameter file of the mower if given
(datum, measured values), then the battery voltages of the logic from battery_params (platform.yaml of the platform,
which the measurement image keeps in /etc/mowbite) and the overrides given here. respawn:=true starts a
killed or ended driver again after respawn_delay s (M4, M7, M8 with the driver killed). speed_limit and max_duty hold
for this start only: without them the files hold (driving at most 0.3 m/s). speed_limit above 0.3, with max_duty above
0.3 if wanted, is the raised start for the fast runs of M8 (own word of the user): only the driver of the measurement
image starts with it. max_duty:=0 locks the wheels (M7 with the disc).

  ros2 launch xcore_driver measure.launch.py mower_params:=/data/nx60i.yaml respawn:=true
  ros2 launch xcore_driver measure.launch.py mower_params:=/data/nx60i.yaml respawn:=true speed_limit:=0.5 max_duty:=1.0
"""
import asyncio
import math
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueCoroutine, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# logic_params.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import logic_params  # noqa: E402

ARGUMENTS = {
    "params": ("", "parameter file of the driver, empty = nx60i.yaml of xcore_driver"),
    "mower_params": ("", "parameter file of this mower, read after the file of the driver"),
    "battery_params": ("/etc/mowbite/nx60i.yaml", "parameter file of the logic with the battery voltages, as "
                                                  "the start of the OS hands them to the driver"),
    "interface": ("", "network interface of the board, empty = from the files"),
    "bind_ip": ("", "address of the host at the board, empty = from the files"),
    "announcement_port": ("", "port of the board's announcements, empty = from the files"),
    "allow_blade": ("false", "true only for M7, on the user's own word"),
    "speed_limit": ("", "m/s the driver drives at most in this start, 0 to 0.6, empty = the files; above 0.3 only the "
                        "measurement image, for the fast runs of M8 on the user's own word"),
    "max_duty": ("", "largest duty of either wheel in this start, 0 to 1, empty = the files; 0 locks the wheels"),
    "respawn": ("false", "start the driver again after it ended"),
    "respawn_delay": ("3.0", "s before a new start"),
}


# Stands in for the driver after a halt, until SIGINT; ends quietly when the shutdown cancels it.
async def halted(context):
    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        pass


# The loopback profile of the image (fastdds_loopback.xml) under SYSTEM_DEFAULT, else no driver; LOCALHOST or SUBNET
# from the environment start with a warning. Without the file Fast DDS would use every interface and shared memory.
def dds_problem():
    scope = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if scope == "SYSTEM_DEFAULT":
        if profile and os.path.isfile(profile) and os.access(profile, os.R_OK):
            return None
        return (f"ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT needs the profile of the image in "
                f"FASTRTPS_DEFAULT_PROFILES_FILE, '{profile}' is not readable.")
    if not scope:
        scope = os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    print(f"[WARN] ROS_AUTOMATIC_DISCOVERY_RANGE={scope} instead of the loopback profile: DDS uses shared memory and "
          "listens on every interface", file=sys.stderr, flush=True)
    return None


# A number argument from low to high; anything else ends the launch before a driver starts.
def number(value, name, low, high):
    try:
        x = float(value[name])
    except ValueError:
        x = math.nan
    if not low <= x <= high:
        raise RuntimeError(f"{name}:={value[name]} is no number from {low:g} to {high:g}")
    return x


def driver(context):
    problem = dds_problem()
    if problem:
        print(f"MOWBITE_OS_HALT reason=DDS_PROFILE data_schema=- image_schema=- {problem}", file=sys.stderr, flush=True)
        return [OpaqueCoroutine(coroutine=halted)]
    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    files = [value["params"] or os.path.join(FindPackageShare("xcore_driver").find("xcore_driver"), "config",
                                             "nx60i.yaml")]
    if value["mower_params"]:
        files.append(value["mower_params"])
    overrides = {"allow_blade": value["allow_blade"].lower() == "true",
                 **logic_params.battery_voltages(logic_params.logic_section(value["battery_params"]))}
    for name in ("interface", "bind_ip"):
        if value[name]:
            overrides[name] = value[name]
    if value["announcement_port"]:
        overrides["announcement_port"] = int(value["announcement_port"])
    if value["speed_limit"]:
        overrides["max_linear_speed"] = number(value, "speed_limit", 0.0, 0.6)
        if overrides["max_linear_speed"] > 0.3:
            overrides["measurement_speed_limit"] = overrides["max_linear_speed"]
    if value["max_duty"]:
        overrides["max_duty"] = number(value, "max_duty", 0.0, 1.0)
        # Else the steps at 0.3 m/s would run at a higher duty than their word.
        if overrides["max_duty"] > 0.3 and overrides.get("max_linear_speed", 0.0) <= 0.3:
            raise RuntimeError("max_duty above 0.3 belongs to the raised start: give speed_limit above 0.3 as well")
    return [Node(package="xcore_driver", executable="xcore_driver_node", name="xcore_driver", output="screen",
                 namespace="/", parameters=files + [overrides], respawn=value["respawn"].lower() == "true",
                 respawn_delay=float(value["respawn_delay"]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, (default, text) in ARGUMENTS.items()] + [OpaqueFunction(function=driver)])
