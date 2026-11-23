"""The start of MowBite OS on a platform (mowbite.launch.py): DDS, data_dir and the parameter files before any node,
the driver of the platform in a process of its own, the container with the nodes above the hardware contract,
restarted after a crash, and the MowBite Planner. What the platform adds comes from platform.py of its package.
"""
import asyncio
import importlib.util
import json
import os
import subprocess
import sys
import time

import launch.logging
from launch.actions import (DeclareLaunchArgument, OpaqueCoroutine, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

# data_store.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import data_store  # noqa: E402
import layers  # noqa: E402
import migrate  # noqa: E402

# After a crash the container starts again at most this often, each time after RESTART_DELAY s, then it stays down
# until the next start by hand. A run of STABLE_AFTER s counts the crashes from 0 again.
CONTAINER_RESTARTS = 3
RESTART_DELAY = 2.0
STABLE_AFTER = 600.0
# In data_dir: how the container ended, for its bridge to log and mowbite-host to show; down true after the last end.
EXIT_NOTE = "container_exit.json"
# In data_dir: the node the watch of the container (mowbite_watch) ended it for, taken into EXIT_NOTE as stalled.
WATCH_NOTE = "watch_exit.json"
# In data_dir: the setup program asked for a restart of the OS and ended the container for it (mowbite_setup).
RESTART_NOTE = "setup_restart.json"
# In data_dir: how the driver of the platform ended outside the shutdown, written before it starts again.
DRIVER_EXIT_NOTE = "driver_exit.json"
# The nodes beside the container, in their own processes.
PLANNER = "mowbite_planner"
NET = "mowbite_net"


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


def halt_file():
    return os.environ.get("MOWBITE_HALT_FILE") or data_store.HALT_FILE


# The halt line on stderr and in the halt file for mowbite_health, then the launch only waits for SIGINT.
def halt_actions(line, logger):
    print(line, file=sys.stderr, flush=True)
    path = halt_file()
    try:
        data_store.write_halt(path, line)
    except OSError as error:
        logger.error(f"Cannot write {path}: {error}")
    return [OpaqueCoroutine(coroutine=halted)]


# DDS and data_dir for this image before any node starts (data_store.py): None to go on, else the halt.
def prepare_data_dir(data_dir, logger, besides=()):
    try:
        os.remove(halt_file())
    except FileNotFoundError:
        pass
    schema = "-"
    try:
        check_dds(logger)
        if not data_dir:
            return None
        schema, _ = data_store.image_schema()
        # The lock stays open until this process ends, after its nodes have stopped writing.
        _, note = data_store.prepare(data_dir, schema, migrate.image(), besides=besides)
    except data_store.Halt as halt:
        line = halt.line(schema)
    except (OSError, ValueError, KeyError, LookupError) as error:
        line = data_store.Halt("DATA_ERROR", f"The data schema of the image cannot be read: {error}.").line(schema)
    else:
        if note:
            made = time.strftime("%Y-%m-%d", time.gmtime(note["backup_of"]))
            logger.warning(f"data_dir went back from schema {note['from']} to {note['to']} with the backup of {made} "
                           f"(UTC), map included; the changes since the update are in "
                           f"{os.path.join(data_dir, note['backup'])}")
        return None
    return halt_actions(line, logger)


# The parameter files before any node starts (layers.py): None when they can be read and the platform set every value
# stack.yaml leaves to it for these nodes, else the halt PLATFORM_CONFIG.
def check_parameters(files, nodes, logger):
    try:
        missing = layers.unset(files, nodes)
    except (OSError, ValueError) as error:
        text = f"A parameter file cannot be read: {error}."
    else:
        if not missing:
            return None
        text = f"The platform does not set {', '.join(missing)}."
    return halt_actions(data_store.Halt("PLATFORM_CONFIG", text).line("-"), logger)


# The step list of the setup program and its images before any node starts, by the rules of mowbite_setup: None when
# the platform has none or all is well, else the halt PLATFORM_CONFIG.
def check_setup(setup, logger):
    if not setup["steps"]:
        return None
    from ament_index_python.packages import get_package_prefix
    tool = os.path.join(get_package_prefix("mowbite_setup"), "lib", "mowbite_setup", "mowbite_setup_check")
    try:
        result = subprocess.run([tool, setup["image_dir"]], input=setup["steps"], capture_output=True, text=True,
                                timeout=20)
    except (OSError, subprocess.SubprocessError) as error:
        text = f"The step list of the platform cannot be checked: {error}."
    else:
        if result.returncode == 0:
            return None
        lines = (result.stdout or result.stderr).strip().splitlines()
        text = f"The step list of the platform: {lines[0] if lines else 'not valid'}."
    return halt_actions(data_store.Halt("PLATFORM_CONFIG", text).line("-"), logger)


# <node>.<parameter> of what the setup program found and the file kept by hand sets as well: that file has the last
# word, so the value of the program does not hold. None for a file that cannot be read, which check_parameters halts on.
def shadowed_values(found, device, logger):
    if not found or not device:
        return []
    try:
        names = layers.shadowed(found, device)
    except (OSError, ValueError):
        return []
    for name in names:
        logger.warning(f"{name} of the setup program does not hold: {device} sets it as well and is read last")
    return names


# journal_dir as given, or empty with an error when it lies in data_dir: the backup of data_dir would take it along
# (contract 3.5 (7)), and the journal has limits of its own.
def journal_dir(path, data_dir, logger):
    if path and data_dir:
        root = os.path.realpath(data_dir)
        if os.path.commonpath([os.path.realpath(path), root]) == root:
            logger.error(f"journal_dir {path} lies in data_dir {data_dir}: no journal")
            return ""
    return path


# The arguments every platform has, beside its own.
def arguments(platform, module):
    return [
        DeclareLaunchArgument("config", default_value=layers.STACK),
        # The values of this platform, on top of config.
        DeclareLaunchArgument("platform_config", default_value=layers.platform_file(platform)),
        # Values on top of both, e.g. those of the simulation or a test; by default the platform file once more.
        DeclareLaunchArgument("extra_config", default_value=LaunchConfiguration("platform_config")),
        DeclareLaunchArgument("geometry", default_value=layers.geometry(platform)),
        # Without it nothing is kept and no update can hold the driver; with it, it is prepared for the data schema of
        # the image before any node starts.
        DeclareLaunchArgument("data_dir", default_value=getattr(module, "DATA_DIR", "")),
        # OpenMower's folders ros and params, read only; on a platform with OPENMOWER_IMPORT its map and values come
        # over on the first start with data_dir (migrate.py). Empty: no import.
        DeclareLaunchArgument("import_dir", default_value=EnvironmentVariable("MOWBITE_IMPORT_DIR",
                                                                              default_value="")),
        # The broker of the OS, the same one mowbite_health asks.
        DeclareLaunchArgument("mqtt_host", default_value=EnvironmentVariable("MOWBITE_MQTT_HOST",
                                                                             default_value="127.0.0.1")),
        DeclareLaunchArgument("mqtt_port", default_value=EnvironmentVariable("MOWBITE_MQTT_PORT",
                                                                             default_value="1883")),
        # Local time for the schedules and a driver that needs it; a TZ from the environment wins.
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


# system.info.update_path: native for a bundle of mowbite-host (its tree holds bundle.json), compose for the image,
# empty where neither says it.
def update_path():
    from ament_index_python.packages import get_package_prefix
    tree = os.path.dirname(os.path.dirname(get_package_prefix("mowbite_bringup")))
    if os.path.isfile(os.path.join(tree, "bundle.json")):
        return "native"
    return "compose" if os.environ.get("MOWBITE_UPDATE_PATH") == "compose" else ""


# The nodes above the hardware contract, the same on every platform; files are the parameter files of the platform in
# their order (later ones change single values), geometry its dimensions.
def components(files, geometry, data_dir, mqtt_host, mqtt_port, journal, setup=None):
    journal_dir = {"journal_dir": journal}
    version = {"software_version": migrate.revision()}
    localization = ComposableNode(
        package="mowbite_localization", plugin="mowbite_localization::LocalizationNode", name="mowbite_localization",
        parameters=[geometry, *files, journal_dir])
    logic = ComposableNode(
        package="mowbite_logic", plugin="mowbite_logic::LogicNode", name="mowbite_logic",
        parameters=[geometry, *files, {"data_dir": data_dir}, journal_dir])
    bridge = ComposableNode(
        package="mowbite_bridge", plugin="mowbite_bridge::BridgeNode", name="mowbite_bridge",
        parameters=[*files, {"mqtt_host": mqtt_host, "mqtt_port": int(mqtt_port), "data_dir": data_dir,
                             "update_path": update_path()}, version])
    map_node = ComposableNode(
        package="mowbite_map", plugin="mowbite_map::MapNode", name="mowbite_map",
        parameters=[*files, {"data_dir": data_dir}])
    follower = ComposableNode(
        package="mowbite_follower", plugin="mowbite_follower::FollowerNode", name="mowbite_follower",
        parameters=[*files, {"data_dir": data_dir}])
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
        parameters=[*files, {"data_dir": data_dir}, version])
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
        parameters=[*files, {"data_dir": data_dir}, journal_dir])
    # The obstacles of the sensors the platform reports (hw/obstacles): tracks, sight and zones on obstacles/status,
    # and the list with photos for the app (vision.obstacles.*) in data_dir/obstacles.
    obstacles = ComposableNode(
        package="mowbite_obstacles", plugin="mowbite_obstacles::ObstaclesNode", name="mowbite_obstacles",
        parameters=[*files, {"data_dir": data_dir}])
    # The setup program of the platform (setup.*): its steps come from layers.setup, layer is the parameter file it
    # writes for this mower, shadowed what the file kept by hand overrules of it, driver_restarts how often the start
    # starts an ended driver again.
    setup = {"platform": "", "steps": "", "image_dir": "", "layer": "", "shadowed": "", "driver_restarts": "0",
             **(setup or {})}
    program = ComposableNode(
        package="mowbite_setup", plugin="mowbite_setup::SetupNode", name="mowbite_setup",
        parameters=[*files, {"data_dir": data_dir},
                    {name: ParameterValue(value, value_type=str) for name, value in setup.items()}])
    return [localization, safety, logic, bridge, map_node, follower, ntrip, calibration, clock, schedule, stats,
            obstacles, program]


# The names of these composable nodes, as their watch knows them.
def node_names(nodes):
    return ["".join(part.text for part in node.node_name) for node in nodes]


# The nodes container_and_planner starts with these components, as the parameter files name them.
def started(nodes):
    return [*node_names(nodes), PLANNER, NET]


# The nodes whose values the start checks: those of the stack, the driver and the others of the platform.
def checked_nodes(nodes, module):
    return [*started(nodes), module.DRIVER, *module.NODES]


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
# count again. Each end goes into `note` (None: none) under the name `label`, with what the watch of the process left
# in `watch_note` (None: none) as stalled, the last one with down true. Every start gets oom_score_adj. A process that
# ended while leave() holds wanted the whole launch to end: no restart and no note, the launch shuts down.
def restarting(make, note, label, logger, restarts, oom_score_adj="", watch_note=None, clock=time.monotonic,
               leave=None):
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
        if leave and leave():
            logger.info(f"{label} ended for a restart of the OS: the launch ends")
            return [Shutdown(reason="restart of the OS asked for")]
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
                                      {"name": label, "exit_code": event.returncode,
                                       "restart": crashes if down else crashes + 1,
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
        lambda prefix: Node(package="mowbite_net", executable="mowbite_net_node", name=NET, namespace="/",
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
    # The note of a restart the setup program asked for belongs to the start before this one.
    restart_note = os.path.join(data_dir, RESTART_NOTE) if data_dir else None
    remove(restart_note, logger)
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
    planner = Node(package="mowbite_planner_os", executable="mowbite_planner_os_node", name=PLANNER,
                   namespace="/", parameters=[*files, {"data_dir": data_dir, "calib_root": calib_root}],
                   output="screen", respawn=True, respawn_delay=5.0)
    return [*restarting(container, os.path.join(data_dir, EXIT_NOTE), "The container", logger, CONTAINER_RESTARTS,
                        oom_score_adj, watch_note,
                        leave=lambda: bool(restart_note) and os.path.lexists(restart_note)),
            *oom_score(planner, oom_score_adj, logger), planner, *net_service(files, data_dir, logger)]


# platform.py of this platform as a module (new-mower.md, "Platform package"), from folder instead of the index.
def platform_module(platform, folder=None):
    spec = importlib.util.spec_from_file_location(f"mowbite_platform_{platform}",
                                                  os.path.join(folder or layers.folder(platform), "platform.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Parts:
    """What a platform adds to the stack: driver(restart) builds the process that holds the motors for its start
    number restart (None: no driver starts), before and after are its components in the container around those of
    the stack, processes its other actions."""

    def __init__(self, driver=None, before=(), after=(), processes=()):
        self.driver = driver
        self.before = list(before)
        self.after = list(after)
        self.processes = list(processes)


class Start:
    """What parts(start) of a platform builds on: arg(name) gives a launch argument, files are the parameter files in
    their order, setup_file and device the last of them: what the setup program found for this mower and its file kept
    by hand in data_dir (empty: it has none); a driver with files of its own reads these two after them, in this
    order. journal is the journal directory (empty: none), hold_file the hold of an update, which a driver reads
    itself (empty without data_dir), data_store has the names of the other files in data_dir."""

    Parts = Parts
    data_store = data_store

    def __init__(self, arg, files, setup_file, device, data_dir, journal, logger):
        self.arg = arg
        self.files = files
        self.setup_file = setup_file
        self.device = device
        self.data_dir = data_dir
        self.journal = journal
        self.logger = logger
        self.hold_file = self.data_file(data_store.HOLD_FILE)

    def data_file(self, name):
        """<data_dir>/<name>, empty without data_dir."""
        return os.path.join(self.data_dir, name) if self.data_dir else ""

    def parameters(self, node):
        """What the files give the node of this name."""
        return layers.parameters(self.files, node)

    def own_process(self, make, name, **limits):
        return own_process(make, name, self.data_dir, self.logger, **limits)

    def camera_service(self, **limits):
        return camera_service(self.files, self.data_dir, self.logger, **limits)


# The nodes with the values of the launch arguments taken now: the components are loaded again after a crash of the
# container, when the arguments of an including launch may be out of scope (sim.launch.py has them in a TimerAction).
def launch_setup(context, platform, module):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    logger = launch.logging.get_logger(f"{platform}.launch")
    data_dir = arg("data_dir")
    # The file of the device alone is no data yet: the first start counts as new and imports.
    device_name = (os.path.basename(layers.device_file(data_dir, platform)),) if data_dir else ()
    halt = prepare_data_dir(data_dir, logger, device_name)
    if halt:
        return halt
    # What every mower shares, the values of the platform, values on top of both, then what the setup program found
    # for this mower and its values kept by hand, if there are any.
    device = layers.device_file(data_dir, platform) if data_dir else ""
    device = device if os.path.isfile(device) else ""
    setup_file = layers.setup_file(data_dir, platform) if data_dir else ""
    found = setup_file if os.path.isfile(setup_file) else ""
    files = [arg("config"), arg("platform_config"), arg("extra_config")] + [path for path in (found, device) if path]
    journal = journal_dir(arg("journal_dir"), data_dir, logger)
    # The dimensions of the platform come before the files, which can change single values (the simulation does).
    try:
        setup = layers.setup(platform)
    except (OSError, ValueError) as error:
        return halt_actions(data_store.Halt("PLATFORM_CONFIG", f"The step list cannot be read: {error}.").line("-"),
                            logger)
    setup["layer"] = setup_file
    setup["shadowed"] = ",".join(shadowed_values(found, device, logger))
    setup["driver_restarts"] = str(module.DRIVER_RESTARTS)
    nodes = components(files, arg("geometry"), data_dir, arg("mqtt_host"), arg("mqtt_port"), journal, setup)
    halt = check_parameters(files, checked_nodes(nodes, module), logger) or check_setup(setup, logger)
    if halt:
        return halt
    import_dir = arg("import_dir")
    if (import_dir and data_dir and getattr(module, "OPENMOWER_IMPORT", False)
            and migrate.wanted(data_dir, device_name)):
        migrate.take(data_dir, import_dir, int(layers.parameters(files, "mowbite_map")["max_map_bytes"]), logger)
    parts = module.parts(Start(arg, files, found, device, data_dir, journal, logger))
    driver = []
    if parts.driver:
        # Should the container die, the driver stops the wheels after its own timeout and the blade lease runs out; a
        # driver that ends starts again at most DRIVER_RESTARTS times.
        note = os.path.join(data_dir, DRIVER_EXIT_NOTE) if data_dir else None
        driver = restarting(parts.driver, note, module.DRIVER, logger, module.DRIVER_RESTARTS)
    return [*driver, *parts.processes,
            *container_and_planner([*parts.before, *nodes, *parts.after], data_dir, files, logger,
                                   arg("oom_score_adj"))]


# The launch arguments of a start on this platform: those of its package, then those every platform has.
def all_arguments(platform, module):
    return [*module.arguments(), *arguments(platform, module)]


# The start on this platform: its arguments, then the nodes. A name no package of the image entered halts with
# PLATFORM_CONFIG.
def start(context, platform):
    try:
        module = platform_module(platform)
        declared = all_arguments(platform, module)
    except (LookupError, ValueError, OSError) as error:
        text = (f"The image has no platform '{platform}' (platform:=<name> or MOWBITE_PLATFORM), it has "
                f"{', '.join(layers.platforms()) or 'none'}: {error}.")
        return halt_actions(data_store.Halt("PLATFORM_CONFIG", text).line("-"),
                            launch.logging.get_logger("mowbite.launch"))
    return [*declared, OpaqueFunction(function=lambda context: launch_setup(context, platform, module))]
