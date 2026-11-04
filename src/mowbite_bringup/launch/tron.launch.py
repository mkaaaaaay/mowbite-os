# All nodes of MowBite OS in one process (one ROS context, one DDS participant) except two: tron_mcu, so that a crash of
# any other node leaves its watchdogs running (G25), and the MowBite Planner (Python, shapely).
import asyncio
import json
import os
import sys
import time

import launch.logging
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueCoroutine, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# data_store.py lies next to this file, in the source tree and in the install space alike.
STORE_FOLDER = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, STORE_FOLDER)
import data_store  # noqa: E402

# After a crash the container starts again at most this often, each time after RESTART_DELAY s, then it stays down
# until the next start by hand.
CONTAINER_RESTARTS = 3
RESTART_DELAY = 2.0
# In data_dir: how the container ended, written before it starts again, for its bridge to log.
EXIT_NOTE = "container_exit.json"


# Stands in for the nodes after a halt, so that the launch stays without a restart loop until SIGINT.
async def halted(context):
    await asyncio.Event().wait()


# data_dir for this image before any node starts (data_store.py): None to go on, else the halt.
def prepare_data_dir(data_dir, logger):
    halt_file = os.environ.get("MOWBITE_HALT_FILE") or data_store.HALT_FILE
    try:
        os.remove(halt_file)
    except FileNotFoundError:
        pass
    if not data_dir:
        return None
    schema = "-"
    try:
        schema, _ = data_store.image_schema()
        # The lock stays open until this process ends, after its nodes have stopped writing.
        _, note = data_store.prepare(data_dir, schema, os.environ.get("MOWBITE_OS_REVISION") or "unknown")
    except data_store.Halt as halt:
        line = halt.line(schema)
    except (OSError, ValueError, KeyError, LookupError) as error:
        line = data_store.Halt("DATA_ERROR", f"The data schema of the image cannot be read: {error}.").line(schema)
    else:
        if note:
            logger.warning(f"data_dir went back from schema {note['from']} to {note['to']}, the changes since the "
                           f"update are in {os.path.join(data_dir, note['backup'])}")
        return None
    print(line, file=sys.stderr, flush=True)
    try:
        data_store.write_halt(halt_file, line)
    except OSError as error:
        logger.error(f"Cannot write {halt_file}: {error}")
    return [OpaqueCoroutine(coroutine=halted)]


# The nodes with the values of the launch arguments taken now: the components are loaded again after a crash of the
# container, when the arguments of an including launch may be out of scope (sim.launch.py has them in a TimerAction).
def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    logger = launch.logging.get_logger("tron.launch")
    data_dir = arg("data_dir")
    halt = prepare_data_dir(data_dir, logger)
    if halt:
        return halt
    config = arg("config")
    # The dimensions of the platform; config and extra come after it and can change single values (the simulation does).
    geometry = arg("geometry")
    # Values on top of config, e.g. those of the simulation; by default config once more.
    extra = arg("extra_config")
    # Should the container die, the driver stops the wheels after cmd_vel_timeout, the blade lease runs out and the
    # missing IMU latches IMU_TIMEOUT. With the namespace the dict goes under the full name; without it, under /**,
    # the port in the node's own section of config would win.
    mcu = Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", namespace="/",
               parameters=[config, extra, {"port": arg("mcu_port")}], output="screen")
    gnss = ComposableNode(
        package="tron_gnss", plugin="tron_gnss::GnssNode", name="tron_gnss",
        parameters=[config, extra, {"port": arg("gnss_port"), "datum_lat": float(arg("datum_lat")),
                                    "datum_long": float(arg("datum_long")),
                                    "datum_height": float(arg("datum_height")), "data_dir": data_dir}])
    imu = ComposableNode(
        package="tron_imu", plugin="tron_imu::ImuNode", name="tron_imu",
        parameters=[config, extra, {"port": arg("imu_port")}])
    localization = ComposableNode(
        package="mowbite_localization", plugin="mowbite_localization::LocalizationNode", name="mowbite_localization",
        parameters=[config, extra])
    logic = ComposableNode(
        package="mowbite_logic", plugin="mowbite_logic::LogicNode", name="mowbite_logic",
        parameters=[geometry, config, extra, {"data_dir": data_dir}])
    bridge = ComposableNode(
        package="mowbite_bridge", plugin="mowbite_bridge::BridgeNode", name="mowbite_bridge",
        parameters=[config, extra, {"mqtt_host": arg("mqtt_host"), "mqtt_port": int(arg("mqtt_port")),
                                    "data_dir": data_dir}])
    map_node = ComposableNode(
        package="mowbite_map", plugin="mowbite_map::MapNode", name="mowbite_map",
        parameters=[config, extra, {"data_dir": data_dir}])
    follower = ComposableNode(
        package="mowbite_follower", plugin="mowbite_follower::FollowerNode", name="mowbite_follower",
        parameters=[config, extra])
    # Corrections from <data_dir>/ntrip.yaml for tron_gnss; without the file it only reports "off".
    ntrip = ComposableNode(
        package="mowbite_ntrip", plugin="mowbite_ntrip::NtripNode", name="mowbite_ntrip",
        parameters=[config, extra, {"data_dir": data_dir}])
    # S2 between the logic and the driver, the only writer of hw/cmd_vel.
    safety = ComposableNode(
        package="mowbite_safety", plugin="mowbite_safety::SafetyNode", name="mowbite_safety",
        parameters=[config, extra])
    components = [gnss, imu, localization, safety, logic, bridge, map_node, follower, ntrip]
    note = os.path.join(data_dir, EXIT_NOTE)
    # A start by hand begins without restarts.
    try:
        os.remove(note)
    except FileNotFoundError:
        pass
    except OSError as error:
        logger.error(f"Cannot remove {note}: {error}")

    # Started again after a crash. tron_mcu goes on meanwhile and keeps motion and blade blocked until a release; the
    # logic comes back idle. Every restart is a new container with a name of its own: DDS knows the load service of a
    # crashed one for a while, and launch_ros would wait for its answer for ever.
    def container(restart):
        action = ComposableNodeContainer(
            name="mowbite" if restart == 0 else f"mowbite_{restart}", namespace="", package="rclcpp_components",
            executable="component_container", composable_node_descriptions=components, output="screen")
        return [RegisterEventHandler(OnProcessExit(target_action=action,
                                                   on_exit=lambda event, context: crashed(restart, event, context))),
                action]

    def crashed(restart, event, context):
        # At the end of the launch the container is stopped, it did not crash.
        if context.is_shutdown:
            return None
        if restart >= CONTAINER_RESTARTS:
            logger.error(f"The container ended with exit code {event.returncode} after {CONTAINER_RESTARTS} restarts "
                         "and stays down until the next start")
            return None
        try:
            with open(note + ".tmp", "w") as f:
                json.dump({"exit_code": event.returncode, "restart": restart + 1,
                           "max_restarts": CONTAINER_RESTARTS, "t": time.time()}, f)
            os.replace(note + ".tmp", note)
        except OSError as error:
            logger.error(f"Cannot write {note}: {error}")
        return [TimerAction(period=RESTART_DELAY, actions=container(restart + 1))]

    # Plans the mowing paths for mowing.plan; a crash only costs the plans until it is back. Namespace as for tron_mcu.
    planner = Node(package="mowbite_planner", executable="mowbite_planner_node", name="mowbite_planner", namespace="/",
                   parameters=[config, extra, {"data_dir": data_dir}], output="screen", respawn=True,
                   respawn_delay=5.0)
    return [mcu, *container(0), planner]


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
        # The datum of the garden stays out of the repo; without it tron_gnss publishes nothing.
        DeclareLaunchArgument("datum_lat", default_value="nan"),
        DeclareLaunchArgument("datum_long", default_value="nan"),
        DeclareLaunchArgument("datum_height", default_value="0.0"),
        # The broker of the OS, the same one mowbite_health asks.
        DeclareLaunchArgument("mqtt_host", default_value=EnvironmentVariable("MOWBITE_MQTT_HOST",
                                                                             default_value="127.0.0.1")),
        DeclareLaunchArgument("mqtt_port", default_value=EnvironmentVariable("MOWBITE_MQTT_PORT",
                                                                             default_value="1883")),
        # Without it nothing is kept; with it, it is prepared for the data schema of the image before any node starts.
        DeclareLaunchArgument("data_dir", default_value="/userdata/mowbite"),
        # Local time for the heartbeat of the MCU (and later the schedules); a TZ from the environment wins.
        DeclareLaunchArgument("tz", default_value=EnvironmentVariable("TZ", default_value="Europe/Berlin")),
        SetEnvironmentVariable("TZ", LaunchConfiguration("tz")),
        # DDS stays on the mower unless the environment says otherwise. Set before the first ROS node of this launch,
        # which launch_ros creates only to load the components.
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE",
                               EnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", default_value="LOCALHOST")),
        OpaqueFunction(function=launch_setup),
    ])
