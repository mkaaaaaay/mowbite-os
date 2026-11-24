"""The Airseekers Tron for mowbite.launch.py: tron_mcu in a process of its own, so that a crash of any other node
leaves its watchdogs running (G25), tron_gnss, tron_imu, the LEDs and the speaker in the container, and in processes
of their own the camera AI on the left and on the right side camera, the live picture of the stereo camera, the dock
marker of the rear camera with its live picture, and with any of these the camera service. A driver that ends outside
the shutdown starts again at most DRIVER_RESTARTS times, each new one latched from its first frame (start_latched):
the MCU does not latch by itself when its host is gone.
"""
import os

import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

# The nodes of the Tron beside its driver and those of the stack, as the parameter files name them.
NODES = ("tron_gnss", "tron_imu", "mowbite_light", "mowbite_sound", "tron_vision", "right/tron_vision", "tron_stereo",
         "tron_side", "tron_rear", "mowbite_camera")
DRIVER = "tron_mcu"
DRIVER_RESTARTS = 3
# What setup.platforms tells of this mower, by language.
ABOUT = {
    "de": {"name": "Airseekers Tron",
           "text": "Mähroboter von Airseekers mit Kameras und RTK.",
           "board": "MCU an /dev/ttyS9, GNSS an /dev/ttyS4, IMU an /dev/ttyS1: MowBite OS sendet erst, wenn die MCU "
                    "sich meldet."},
    "en": {"name": "Airseekers Tron",
           "text": "Robot mower by Airseekers with cameras and RTK.",
           "board": "MCU on /dev/ttyS9, GNSS on /dev/ttyS4, IMU on /dev/ttyS1: MowBite OS sends only once the MCU "
                    "answers."},
}
# Where the rootfs of the Tron keeps the data of the OS.
DATA_DIR = "/userdata/mowbite"



def wheel_radius():
    """The radius of the drive wheels from geometry.yaml beside this file, the only place that sets it."""
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "geometry.yaml")) as f:
        return float(yaml.safe_load(f)["mowbite_logic"]["ros__parameters"]["geometry"]["wheel_radius"])


# What every process around the cameras gets: the kernel ends it first when memory runs out, and it keeps CPU 4-7 free.
CAMERA_LIMITS = {"oom_score_adj": 1000, "nice": 10, "cpus": "0-3"}


def arguments():
    return [
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
        # on: the camera AI starts as well, only with hardware:=true; MOWBITE_VISION in /etc/mowbite/os.conf.
        DeclareLaunchArgument("vision", default_value=EnvironmentVariable("MOWBITE_VISION", default_value="off")),
        # on: the live picture of the stereo camera starts as well, only with hardware:=true; MOWBITE_STEREO in os.conf.
        DeclareLaunchArgument("stereo", default_value=EnvironmentVariable("MOWBITE_STEREO", default_value="off")),
        # on: the right side camera starts as well, only with hardware:=true; MOWBITE_RIGHT in os.conf.
        DeclareLaunchArgument("right", default_value=EnvironmentVariable("MOWBITE_RIGHT", default_value="off")),
        # vision: the camera AI on the right side camera; side: only its live picture. MOWBITE_RIGHT_NODE in os.conf.
        DeclareLaunchArgument("right_node",
                              default_value=EnvironmentVariable("MOWBITE_RIGHT_NODE", default_value="vision")),
        # on: the dock marker of the rear camera with its live picture starts as well, only with hardware:=true;
        # MOWBITE_REAR in os.conf.
        DeclareLaunchArgument("rear", default_value=EnvironmentVariable("MOWBITE_REAR", default_value="off")),
        # off: the side and rear cameras send no H.264, only JPEG pictures; MOWBITE_CAMERA_VIDEO in os.conf.
        DeclareLaunchArgument("camera_video",
                              default_value=EnvironmentVariable("MOWBITE_CAMERA_VIDEO", default_value="")),
    ]


def parts(start):
    arg, files, data_dir, logger = start.arg, start.files, start.data_dir, start.logger
    hardware = arg("hardware")
    # Should the container die, the driver stops the wheels after cmd_vel_timeout, the blade lease runs out and the
    # missing IMU latches IMU_TIMEOUT. With the namespace the dict goes under the full name; without it, under /**,
    # the port in the node's own section of the files would win.
    # The driver reads the hold of an update itself, also without the logic and right after its own restart.
    mcu_values = {"port": arg("mcu_port"), "hold_file": start.hold_file, "journal_dir": start.journal,
                  "stop_clear_mark": start.data_file(start.data_store.STOP_CLEAR_MARK_FILE),
                  "read_only": hardware == "read"}

    # Also with hardware:=read; the driver takes no latch there.
    def mcu(restart):
        return Node(package="tron_mcu", executable="tron_mcu_node", name=DRIVER, namespace="/",
                    parameters=[*files, {**mcu_values, "start_latched": restart > 0}], output="screen")

    gnss = ComposableNode(
        package="tron_gnss", plugin="tron_gnss::GnssNode", name="tron_gnss",
        parameters=[*files, {"port": arg("gnss_port"), "datum_lat": float(arg("datum_lat")),
                             "datum_long": float(arg("datum_long")),
                             "datum_height": float(arg("datum_height")), "data_dir": data_dir}])
    imu = ComposableNode(
        package="tron_imu", plugin="tron_imu::ImuNode", name="tron_imu",
        parameters=[*files, {"port": arg("imu_port")}])
    # The LEDs on /dev/spidev3.0, only with hardware:=true; output in the files.
    light = ComposableNode(
        package="mowbite_light", plugin="mowbite_light::LightNode", name="mowbite_light",
        parameters=[*files, {"data_dir": data_dir}])
    # The speaker (ALSA), only with hardware:=true; output in the files.
    sound = ComposableNode(
        package="mowbite_sound", plugin="mowbite_sound::SoundNode", name="mowbite_sound",
        parameters=[*files, {"data_dir": data_dir}])
    # Without hardware no process opens the MCU, GNSS or IMU port: the stack runs, the logic finds no driver.
    if hardware == "false":
        logger.warning("hardware:=false: tron_mcu, tron_gnss and tron_imu do not start")
        return start.Parts()
    # read: tron_mcu and tron_imu only read their ports, tron_gnss (it writes to its port) does not start.
    if hardware == "read":
        logger.warning("hardware:=read: tron_mcu and tron_imu only listen, nothing moves; tron_gnss does not start")
        return start.Parts(driver=mcu, before=[imu])
    # Only here: with hardware:=read another host drives the blade, and blade_on of hw/status would always be false.
    video = camera_video(arg("camera_video"), logger)
    cameras = [*camera(start, arg("vision"), "vision", "tron_vision_node", "tron_vision", video),
               # The obstacles of the stereo camera put the ground at -wheel_radius.
               *camera(start, arg("stereo"), "stereo", "tron_stereo_node", "tron_stereo",
                       {"wheel_radius": wheel_radius()}),
               *right_camera(start, arg("right"), arg("right_node"), video),
               *camera(start, arg("rear"), "rear", "tron_rear_node", "tron_rear", video)]
    # The camera service of the OS shows their pictures in the app; without a camera process it has none to show.
    if cameras:
        cameras += start.camera_service(**CAMERA_LIMITS)
    return start.Parts(driver=mcu, before=[gnss, imu], after=[light, sound], processes=cameras)


# A camera process of tron_vision with <process>:=on, each of its own, so that a crash of the NPU runtime, of the
# media library or in a USB camera takes nothing along. They only report.
def camera(start, switch, process, executable, name, video=None, namespace="/"):
    if switch != "on":
        # Another value in os.conf must not stop the stack.
        if switch != "off":
            start.logger.warning(f"{process}:={switch} is neither on nor off: {name} does not start")
        return []
    # A second instance of a node takes the name of its namespace into its output lines.
    label = name if namespace == "/" else f"{name}_{namespace.strip('/')}"
    # The node names this process in hw/cameras, so the OS ties its exit note to its cameras.
    return start.own_process(
        lambda prefix: Node(package="tron_vision", executable=executable, name=name, namespace=namespace,
                            parameters=[*start.files, {"process": process, **(video or {})}], output="screen",
                            prefix=prefix, exec_name=label),
        process, **CAMERA_LIMITS)


# The right side camera under the switch right: the camera AI as a second tron_vision in the namespace /right (its
# values in the files under /right/tron_vision on top of those of both), or with right_node:=side only its live picture.
def right_camera(start, switch, node, video):
    if node == "side":
        return camera(start, switch, "right", "tron_side_node", "tron_side", video)
    if node != "vision" and switch == "on":
        start.logger.warning(f"right_node:={node} is neither vision nor side: the right camera does not start")
        return []
    return camera(start, switch, "right", "tron_vision_node", "tron_vision", video, "/right")


# The H.264 stream of the side and rear cameras: off turns it off without a new bundle, unset keeps the files.
def camera_video(switch, logger):
    if switch == "off":
        return {"video": False}
    if switch not in ("", "on"):
        logger.warning(f"camera_video:={switch} is neither on nor off: the cameras keep video from the files")
    return {}
