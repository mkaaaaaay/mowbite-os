"""The part of MowBite OS every platform starts (tron.launch.py, nx60i.launch.py): DDS and data_dir before any node,
the container with the nodes above the hardware contract, restarted after a crash, and the MowBite Planner.
"""
import asyncio
import json
import os
import sys
import time

import yaml
from launch.actions import (DeclareLaunchArgument, OpaqueCoroutine, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# data_store.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import data_store  # noqa: E402

# After a crash the container starts again at most this often, each time after RESTART_DELAY s, then it stays down
# until the next start by hand.
CONTAINER_RESTARTS = 3
RESTART_DELAY = 2.0
# In data_dir: how the container ended, written before it starts again, for its bridge to log.
EXIT_NOTE = "container_exit.json"
# Battery voltages that stand only in the parameter files of the logic; a driver that sets them on its board gets them
# from there under the same names.
BATTERY_VOLTAGES = ("battery_empty_voltage", "battery_full_voltage", "battery_critical_voltage")


# Stands in for the nodes after a halt, so that the launch stays without a restart loop until SIGINT. It ends quietly
# when the shutdown cancels it: a cancelled task would end the launch before it stops the processes beside it.
async def halted(context):
    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        pass


# DDS of this launch and every node it starts, before the first of them: the loopback profile of the image
# (fastdds_loopback.xml) under SYSTEM_DEFAULT, else the halt; LOCALHOST or SUBNET from the environment start with a
# warning. Without the file Fast DDS would fall back to multicast on every interface and to shared memory.
def check_dds(logger):
    scope = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if scope == "SYSTEM_DEFAULT":
        if profile and os.path.isfile(profile) and os.access(profile, os.R_OK):
            return
        raise data_store.Halt("DDS_PROFILE", f"ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT needs the profile of the "
                                             f"image in FASTRTPS_DEFAULT_PROFILES_FILE, '{profile}' is not readable.")
    if not scope:
        # Outside the image: DDS stays on the machine, as before the profile.
        scope = os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    logger.warning(f"ROS_AUTOMATIC_DISCOVERY_RANGE={scope} instead of the loopback profile: DDS uses shared memory "
                   "and listens on every interface")


# DDS and data_dir for this image before any node starts (data_store.py): None to go on, else the halt.
def prepare_data_dir(data_dir, logger):
    halt_file = os.environ.get("MOWBITE_HALT_FILE") or data_store.HALT_FILE
    try:
        os.remove(halt_file)
    except FileNotFoundError:
        pass
    schema = "-"
    try:
        check_dds(logger)
        if not data_dir:
            return None
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


# journal_dir as given, or empty with an error when it lies in data_dir: the backup of data_dir would take it along
# (contract 3.5 (7)), and the journal has limits of its own.
def journal_dir(path, data_dir, logger):
    if path and data_dir:
        root = os.path.realpath(data_dir)
        if os.path.commonpath([os.path.realpath(path), root]) == root:
            logger.error(f"journal_dir {path} lies in data_dir {data_dir}: no journal")
            return ""
    return path


# The battery voltages of the logic in the parameter files, the last file winning as for the node.
def battery_voltages(files):
    values = {}
    for path in files:
        with open(path) as f:
            params = ((yaml.safe_load(f) or {}).get("mowbite_logic") or {}).get("ros__parameters") or {}
        values.update({key: params[key] for key in BATTERY_VOLTAGES if key in params})
    return values


# The arguments every platform shares, after its own.
def arguments():
    return [
        # The broker of the OS, the same one mowbite_health asks.
        DeclareLaunchArgument("mqtt_host", default_value=EnvironmentVariable("MOWBITE_MQTT_HOST",
                                                                             default_value="127.0.0.1")),
        DeclareLaunchArgument("mqtt_port", default_value=EnvironmentVariable("MOWBITE_MQTT_PORT",
                                                                             default_value="1883")),
        # Local time for the heartbeat of the MCU (and later the schedules); a TZ from the environment wins.
        DeclareLaunchArgument("tz", default_value=EnvironmentVariable("TZ", default_value="Europe/Berlin")),
        SetEnvironmentVariable("TZ", LaunchConfiguration("tz")),
        # The journal of the cores (mowbite_journal), outside of data_dir; empty: none. Its limits are in the config.
        DeclareLaunchArgument("journal_dir", default_value=EnvironmentVariable("MOWBITE_JOURNAL_DIR",
                                                                               default_value="")),
        # oom_score_adj of the container and the planner, so that the kernel ends them before the driver when memory
        # runs out; empty: inherited.
        DeclareLaunchArgument("oom_score_adj", default_value=EnvironmentVariable("MOWBITE_OOM_SCORE_ADJ",
                                                                                 default_value="")),
    ]


# The nodes above the hardware contract, the same on every platform; files are the parameter files of the platform in
# their order (later ones change single values), geometry its dimensions.
def components(files, geometry, data_dir, mqtt_host, mqtt_port, journal):
    journal_dir = {"journal_dir": journal}
    localization = ComposableNode(
        package="mowbite_localization", plugin="mowbite_localization::LocalizationNode", name="mowbite_localization",
        parameters=[*files, journal_dir])
    logic = ComposableNode(
        package="mowbite_logic", plugin="mowbite_logic::LogicNode", name="mowbite_logic",
        parameters=[geometry, *files, {"data_dir": data_dir}, journal_dir])
    bridge = ComposableNode(
        package="mowbite_bridge", plugin="mowbite_bridge::BridgeNode", name="mowbite_bridge",
        parameters=[*files, {"mqtt_host": mqtt_host, "mqtt_port": int(mqtt_port), "data_dir": data_dir}])
    map_node = ComposableNode(
        package="mowbite_map", plugin="mowbite_map::MapNode", name="mowbite_map",
        parameters=[*files, {"data_dir": data_dir}])
    follower = ComposableNode(
        package="mowbite_follower", plugin="mowbite_follower::FollowerNode", name="mowbite_follower",
        parameters=[*files])
    # Corrections from <data_dir>/ntrip.yaml for the GNSS of the platform; without the file it only reports "off".
    ntrip = ComposableNode(
        package="mowbite_ntrip", plugin="mowbite_ntrip::NtripNode", name="mowbite_ntrip",
        parameters=[*files, {"data_dir": data_dir}])
    # S2 between the logic and the driver, the only writer of hw/cmd_vel; it latches the restart hold of data_dir.
    safety = ComposableNode(
        package="mowbite_safety", plugin="mowbite_safety::SafetyNode", name="mowbite_safety",
        parameters=[*files, {"data_dir": data_dir}, journal_dir])
    # The calibration drive: its program, recording and result; it moves only through the logic and S2.
    calibration = ComposableNode(
        package="mowbite_calibration", plugin="mowbite_calibration::CalibrationNode", name="mowbite_calibration",
        parameters=[*files, {"data_dir": data_dir}])
    return [localization, safety, logic, bridge, map_node, follower, ntrip, calibration]


# On every start of action its oom_score_adj; raising it needs no privilege. Empty: inherited.
def oom_score(action, oom_score_adj, logger):
    if not oom_score_adj:
        return []
    value = str(int(oom_score_adj))

    def started(event, context):
        try:
            with open(f"/proc/{event.pid}/oom_score_adj", "w") as f:
                f.write(value)
        except OSError as error:
            logger.error(f"Cannot set oom_score_adj of {event.process_name}: {error}")
    return [RegisterEventHandler(OnProcessStart(target_action=action, on_start=started))]


# make(restart) as a process that starts again after it ended outside the shutdown, at most `restarts` times, each time
# after RESTART_DELAY s; before each restart `note` says how it ended. on_start(action) adds handlers to every start.
def restarting(make, note, label, logger, restarts, on_start):
    # A start by hand begins without restarts.
    try:
        os.remove(note)
    except FileNotFoundError:
        pass
    except OSError as error:
        logger.error(f"Cannot remove {note}: {error}")

    def start(restart):
        action = make(restart)
        return [RegisterEventHandler(OnProcessExit(target_action=action,
                                                   on_exit=lambda event, context: ended(restart, event, context))),
                *on_start(action), action]

    def ended(restart, event, context):
        # At the end of the launch the process is stopped, it did not crash.
        if context.is_shutdown:
            return None
        if restart >= restarts:
            logger.error(f"{label} ended with exit code {event.returncode} after {restarts} restarts and stays down "
                         "until the next start")
            return None
        try:
            with open(note + ".tmp", "w") as f:
                json.dump({"exit_code": event.returncode, "restart": restart + 1, "max_restarts": restarts,
                           "t": time.time()}, f)
            os.replace(note + ".tmp", note)
        except OSError as error:
            logger.error(f"Cannot write {note}: {error}")
        return [TimerAction(period=RESTART_DELAY, actions=start(restart + 1))]
    return start(0)


# A process of its own beside the stack, e.g. one that loads a library which may crash: make(prefix) builds its action
# with that command prefix (None without limits), which sets nice and the CPUs before the first thread starts. Restarts
# as the container, with <name>_exit.json in data_dir.
def own_process(make, name, data_dir, logger, restarts=CONTAINER_RESTARTS, oom_score_adj="", nice=None, cpus=None):
    prefix = []
    if cpus:
        prefix += ["taskset", "-c", str(cpus)]
    if nice is not None:
        prefix += ["nice", "-n", str(int(nice))]
    return restarting(lambda restart: make(" ".join(prefix) or None), os.path.join(data_dir, f"{name}_exit.json"),
                      name, logger, restarts, lambda action: oom_score(action, oom_score_adj, logger))


# The container with these components, started again after a crash, and the planner. The driver of the platform runs
# beside them in a process of its own and keeps motion and blade blocked until a release; the logic comes back idle.
def container_and_planner(nodes, data_dir, files, logger, oom_score_adj=""):
    # Every restart is a new container with a name of its own: DDS knows the load service of a crashed one for a while,
    # and launch_ros would wait for its answer for ever.
    def container(restart):
        return ComposableNodeContainer(
            name="mowbite" if restart == 0 else f"mowbite_{restart}", namespace="", package="rclcpp_components",
            executable="component_container", composable_node_descriptions=nodes, output="screen")

    # Plans the mowing paths for mowing.plan; a crash only costs the plans until it is back. Namespace "/" so that a
    # dict of parameters goes under its full name.
    planner = Node(package="mowbite_planner", executable="mowbite_planner_node", name="mowbite_planner", namespace="/",
                   parameters=[*files, {"data_dir": data_dir}], output="screen", respawn=True,
                   respawn_delay=5.0)
    return [*restarting(container, os.path.join(data_dir, EXIT_NOTE), "The container", logger, CONTAINER_RESTARTS,
                        lambda action: oom_score(action, oom_score_adj, logger)),
            *oom_score(planner, oom_score_adj, logger), planner]
