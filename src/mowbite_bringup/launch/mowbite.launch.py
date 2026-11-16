# MowBite OS on the platform of this mower: platform:=<name>, by default MOWBITE_PLATFORM, a value of the device (the
# compose file or os.conf sets it). stack.py starts the stack, platform.py of the platform's package adds its part.
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# stack.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import stack  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform", default_value=EnvironmentVariable("MOWBITE_PLATFORM", default_value="")),
        OpaqueFunction(function=lambda context: stack.start(context, LaunchConfiguration("platform").perform(context))),
    ])
