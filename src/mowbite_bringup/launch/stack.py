"""The part of MowBite OS every platform starts (tron.launch.py, nx60i.launch.py): DDS and data_dir before any node,
the container with the nodes above the hardware contract, restarted after a crash, and the MowBite Planner.
"""
import asyncio
import json
import os
import sys
import time

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
# until the next start by hand. A run of STABLE_AFTER s counts the crashes from 0 again.
CONTAINER_RESTARTS = 3
RESTART_DELAY = 2.0
STABLE_AFTER = 600.0
# In data_dir: how the container ended, for its bridge to log and mowbite-host to show; down true after the last end.
EXIT_NOTE = "container_exit.json"
# In data_dir: the node the watch of the container (mowbite_watch) ended it for, taken into EXIT_NOTE as stalled.
WATCH_NOTE = "watch_exit.json"


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
    # Whether the wall clock can be trusted for date and sun (time/clock); data_dir holds its last synchronization.
    clock = ComposableNode(
        package="mowbite_time", plugin="mowbite_time::ClockNode", name="mowbite_clock",
        parameters=[*files, {"data_dir": data_dir}])
    # The plans that start the mower from the station, only through the logic; the platform allows it (allowed).
    schedule = ComposableNode(
        package="mowbite_schedule", plugin="mowbite_schedule::ScheduleNode", name="mowbite_schedule",
        parameters=[*files, {"data_dir": data_dir}, journal_dir])
    # Mowing statistics and the blade hours of the parts (stats.*), for the app and the pushes of maintenance.
    stats = ComposableNode(
        package="mowbite_stats", plugin="mowbite_stats::StatsNode", name="mowbite_stats",
        parameters=[*files, {"data_dir": data_dir}])
    return [localization, safety, logic, bridge, map_node, follower, ntrip, calibration, clock, schedule, stats]


# The names of these composable nodes, as their watch knows them.
def node_names(nodes):
    return ["".join(part.text for part in node.node_name) for node in nodes]


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


# Removes path if it is there; None: nothing.
def remove(path, logger):
    try:
        if path:
            os.remove(path)
    except FileNotFoundError:
        pass
    except OSError as error:
        logger.error(f"Cannot remove {path}: {error}")


# {node, seconds, limit} of the note of the watch, which it then removes; None without one or when it cannot be read.
def take_watch_note(path, logger):
    if not path:
        return None
    try:
        with open(path) as f:
            note = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        logger.error(f"Cannot read {path}: {error}")
        note = None
    remove(path, logger)
    if not isinstance(note, dict) or not isinstance(note.get("node"), str):
        return None
    return {key: note.get(key) for key in ("node", "seconds", "limit")}


# make(start) as a process that starts again after it ended outside the shutdown, at most `restarts` times in a row,
# each time after RESTART_DELAY s; start counts every start for a name of its own. A run of STABLE_AFTER s begins the
# count again. Each end goes into `note` (None: none) with what the watch of the process left in `watch_note` (None:
# none) as stalled, the last one with down true. Every start gets oom_score_adj.
def restarting(make, note, label, logger, restarts, oom_score_adj="", watch_note=None, clock=time.monotonic):
    # A start by hand begins without restarts.
    remove(note, logger)
    remove(watch_note, logger)

    def start(number, crashes):
        action = make(number)
        began = clock()
        return [RegisterEventHandler(OnProcessExit(
                    target_action=action,
                    on_exit=lambda event, context: ended(number, crashes, began, event, context))),
                *oom_score(action, oom_score_adj, logger), action]

    def ended(number, crashes, began, event, context):
        # At the end of the launch the process is stopped, it did not crash.
        if context.is_shutdown:
            return None
        stalled = take_watch_note(watch_note, logger)
        now = clock()
        if now - began >= STABLE_AFTER:
            crashes = 0
        down = crashes >= restarts
        why = f" (the watch: {stalled['node']} gave no sign)" if stalled else ""
        if down:
            logger.error(f"{label} ended with exit code {event.returncode}{why} after {restarts} restarts and stays "
                         "down until the next start")
        else:
            logger.warning(f"{label} ended with exit code {event.returncode}{why}: started again in "
                           f"{RESTART_DELAY:.0f} s (restart {crashes + 1} of {restarts})")
        try:
            if note:
                # mono is the end on `clock`: a reader compares it with its own CLOCK_MONOTONIC, t may jump.
                data_store.write_json(os.path.dirname(note), os.path.basename(note),
                                      {"exit_code": event.returncode, "restart": crashes if down else crashes + 1,
                                       "max_restarts": restarts, "t": time.time(), "mono": now, "stalled": stalled,
                                       "down": down, "stable_s": STABLE_AFTER})
        except OSError as error:
            logger.error(f"Cannot write {note}: {error}")
        if down:
            return None
        return [TimerAction(period=RESTART_DELAY, actions=start(number + 1, crashes + 1))]
    return start(0, 0)


# A process of its own beside the stack, e.g. one that loads a library which may crash: make(prefix) builds its action
# with that command prefix (None without limits), which sets nice and the CPUs before the first thread starts. Restarts
# as the container, with <name>_exit.json in data_dir.
def own_process(make, name, data_dir, logger, restarts=CONTAINER_RESTARTS, oom_score_adj="", nice=None, cpus=None):
    prefix = []
    if cpus:
        prefix += ["taskset", "-c", str(cpus)]
    if nice is not None:
        prefix += ["nice", "-n", str(int(nice))]
    return restarting(lambda start: make(" ".join(prefix) or None), os.path.join(data_dir, f"{name}_exit.json"),
                      name, logger, restarts, oom_score_adj)


# mowbite_camera in a process of its own: camera.list and camera.stream for the cameras the platform reports on
# hw/cameras. A platform with cameras starts it beside their processes; limits as own_process takes them.
def camera_service(files, data_dir, logger, **limits):
    return own_process(
        lambda prefix: Node(package="mowbite_camera", executable="mowbite_camera_node", name="mowbite_camera",
                            namespace="/", parameters=[*files, {"data_dir": data_dir}], output="screen",
                            prefix=prefix, exec_name="mowbite_camera"),
        "camera", data_dir, logger, **limits)


# mowbite_net in a process of its own, the only one that talks to the internet: weather and pushes (notify.*).
def net_service(files, data_dir, logger):
    return own_process(
        lambda prefix: Node(package="mowbite_net", executable="mowbite_net_node", name="mowbite_net", namespace="/",
                            parameters=[*files, {"data_dir": data_dir}], output="screen", prefix=prefix,
                            exec_name="mowbite_net"),
        "net", data_dir, logger)


# The container with these components, started again after a crash, and the planner. The driver of the platform runs
# beside them in a process of its own and keeps motion and blade blocked until a release; the logic comes back idle.
def container_and_planner(nodes, data_dir, files, logger, oom_score_adj=""):
    # Every restart is a new container with a name of its own: DDS knows the load service of a crashed one for a while,
    # and launch_ros would wait for its answer for ever. Each node spins in its own thread, so a node that blocks stalls
    # only itself, not S2 or the follower beside it; the watch of the container ends it when one stays silent or one
    # of these nodes does not come (mowbite_watch).
    watch_note = os.path.join(data_dir, WATCH_NOTE) if data_dir else None
    watch_env = {"MOWBITE_WATCH_NODES": ",".join(node_names(nodes))}
    if watch_note:
        watch_env["MOWBITE_WATCH_NOTE"] = watch_note

    def container(start):
        return ComposableNodeContainer(
            name="mowbite" if start == 0 else f"mowbite_{start}", namespace="", package="rclcpp_components",
            executable="component_container_isolated", composable_node_descriptions=nodes, output="screen",
            additional_env=watch_env)

    # Plans the mowing paths for mowing.plan; a crash only costs the plans until it is back. Namespace "/" so that a
    # dict of parameters goes under its full name. calib_root is the folder of mowbite_calibration: only under it
    # planner.calib.evaluate reads a recording.
    calib_root = os.path.join(data_dir, "calibration") if data_dir else ""
    planner = Node(package="mowbite_planner_os", executable="mowbite_planner_os_node", name="mowbite_planner",
                   namespace="/", parameters=[*files, {"data_dir": data_dir, "calib_root": calib_root}],
                   output="screen", respawn=True, respawn_delay=5.0)
    return [*restarting(container, os.path.join(data_dir, EXIT_NOTE), "The container", logger, CONTAINER_RESTARTS,
                        oom_score_adj, watch_note),
            *oom_score(planner, oom_score_adj, logger), planner, *net_service(files, data_dir, logger)]
