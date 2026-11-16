# MowBite OS on the Tron: the start command of its rootfs (run-os) names this file. The same as mowbite.launch.py
# platform:=tron.
import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

# stack.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import stack  # noqa: E402


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=lambda context: stack.start(context, "tron"))])
