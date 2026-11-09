# All nodes of MowBite OS in one process (one ROS context, one DDS participant) except three: tron_mcu, so that a crash
# of any other node leaves its watchdogs running (G25), the MowBite Planner (Python, shapely) and the camera AI. The
# part every platform shares is in stack.py.
import os
import sys

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# stack.py and data_store.py lie next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import data_store  # noqa: E402
import stack  # noqa: E402


# The nodes with the values of the launch arguments taken now: the components are loaded again after a crash of the
# container, when the arguments of an including launch may be out of scope (sim.launch.py has them in a TimerAction).
def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    logger = launch.logging.get_logger("tron.launch")
    data_dir = arg("data_dir")
    halt = stack.prepare_data_dir(data_dir, logger)
    if halt:
        return halt
    config = arg("config")
    # The dimensions of the platform; config and extra come after it and can change single values (the simulation does).
    geometry = arg("geometry")
    # Values on top of config, e.g. those of the simulation; by default config once more. Then the measured values of
    # this Tron (data_dir/tron.yaml), if there are any.
    extra = arg("extra_config")
    mower = os.path.join(data_dir, data_store.TRON_PARAMS_FILE) if data_dir else ""
    files = [config, extra] + ([mower] if os.path.isfile(mower) else [])
    # Should the container die, the driver stops the wheels after cmd_vel_timeout, the blade lease runs out and the
    # missing IMU latches IMU_TIMEOUT. With the namespace the dict goes under the full name; without it, under /**,
    # the port in the node's own section of config would win.
    # The driver reads the hold of an update itself, also without the logic and right after its own restart.
    hold_file = os.path.join(data_dir, data_store.HOLD_FILE) if data_dir else ""
    journal = stack.journal_dir(arg("journal_dir"), data_dir, logger)
    hardware = arg("hardware")
    mcu = Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", namespace="/",
               parameters=[*files, {"port": arg("mcu_port"), "hold_file": hold_file, "journal_dir": journal,
                                    "read_only": hardware == "read"}],
               output="screen")
    gnss = ComposableNode(
        package="tron_gnss", plugin="tron_gnss::GnssNode", name="tron_gnss",
        parameters=[*files, {"port": arg("gnss_port"), "datum_lat": float(arg("datum_lat")),
                                    "datum_long": float(arg("datum_long")),
                                    "datum_height": float(arg("datum_height")), "data_dir": data_dir}])
    imu = ComposableNode(
        package="tron_imu", plugin="tron_imu::ImuNode", name="tron_imu",
        parameters=[*files, {"port": arg("imu_port")}])
    # The LEDs on /dev/spidev3.0, only with hardware:=true; output in config.
    light = ComposableNode(
        package="mowbite_light", plugin="mowbite_light::LightNode", name="mowbite_light",
        parameters=[*files, {"data_dir": data_dir}])
    # The speaker (ALSA), only with hardware:=true; output in config.
    sound = ComposableNode(
        package="mowbite_sound", plugin="mowbite_sound::SoundNode", name="mowbite_sound",
        parameters=[*files, {"data_dir": data_dir}])
    nodes = stack.components(files, geometry, data_dir, arg("mqtt_host"), arg("mqtt_port"), journal)
    oom = arg("oom_score_adj")
    # Without hardware no process opens the MCU, GNSS or IMU port: the stack runs, the logic finds no driver.
    if hardware == "false":
        logger.warning("hardware:=false: tron_mcu, tron_gnss and tron_imu do not start")
        return stack.container_and_planner(nodes, data_dir, files, logger, oom)
    # read: tron_mcu and tron_imu only read their ports, tron_gnss (it writes to its port) does not start.
    if hardware == "read":
        logger.warning("hardware:=read: tron_mcu and tron_imu only listen, nothing moves; tron_gnss does not start")
        return [mcu, *stack.container_and_planner([imu, *nodes], data_dir, files, logger, oom)]
    return [mcu, *vision(files, data_dir, arg("vision"), logger),
            *stack.container_and_planner([gnss, imu, *nodes, light, sound], data_dir, files, logger, oom)]


# The camera AI (tron_vision) with vision:=on: a process of its own, so that a crash of the NPU runtime takes nothing
# along. It only reports; the kernel ends it first when memory runs out, and it keeps CPU 4-7 free.
def vision(files, data_dir, switch, logger):
    if switch != "on":
        # Another value in os.conf must not stop the stack.
        if switch != "off":
            logger.warning(f"vision:={switch} is neither on nor off: no camera AI")
        return []
    # The node names this process in hw/cameras, so the OS ties its exit note to the camera.
    process = "vision"
    return stack.own_process(
        lambda prefix: Node(package="tron_vision", executable="tron_vision_node", name="tron_vision", namespace="/",
                            parameters=[*files, {"process": process}], output="screen", prefix=prefix,
                            exec_name="tron_vision"),
        process, data_dir, logger, oom_score_adj=1000, nice=10, cpus="0-3")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [FindPackageShare("mowbite_bringup"), "config", "tron.yaml"])),
        DeclareLaunchArgument("extra_config", default_value=LaunchConfiguration("config")),
        DeclareLaunchArgument("geometry", default_value=PathJoinSubstitution(
            [FindPackageShare("mowbite_bringup"), "config", "geometry", "tron.yaml"])),
        DeclareLaunchArgument("mcu_port", default_value="/dev/ttyS9"),
        DeclareLaunchArgument("gnss_port", default_value="/dev/ttyS4"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyS1"),
        # false: neither tron_mcu nor the GNSS and IMU nodes, nothing is sent to a port of the mower. read: tron_mcu and
        # tron_imu only read, nothing is sent either.
        DeclareLaunchArgument("hardware", default_value="true", choices=["true", "false", "read"]),
        # The datum of the garden stays out of the repo; without it tron_gnss publishes nothing.
        DeclareLaunchArgument("datum_lat", default_value="nan"),
        DeclareLaunchArgument("datum_long", default_value="nan"),
        DeclareLaunchArgument("datum_height", default_value="0.0"),
        # Without it nothing is kept; with it, it is prepared for the data schema of the image before any node starts.
        DeclareLaunchArgument("data_dir", default_value="/userdata/mowbite"),
        # on: the camera AI starts as well, only with hardware:=true; MOWBITE_VISION in /etc/mowbite/os.conf.
        DeclareLaunchArgument("vision", default_value=EnvironmentVariable("MOWBITE_VISION", default_value="off")),
        *stack.arguments(),
        OpaqueFunction(function=launch_setup),
    ])
