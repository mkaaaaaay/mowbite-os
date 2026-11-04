# MowBite OS against the simulated Tron (MCU, GNSS, IMU), e.g. on the PC with a local mosquitto.
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


# fake_tron with the charging station of the section fake_tron in extra_config, if it has one (sim.yaml does).
def fake_tron_setup(context):
    try:
        with open(LaunchConfiguration("extra_config").perform(context)) as f:
            values = (yaml.safe_load(f) or {}).get("fake_tron", {}).get("ros__parameters", {})
    except (OSError, yaml.YAMLError, AttributeError):
        values = {}
    station = []
    if "dock" in values:
        station = ["--dock", *(str(float(v)) for v in values["dock"]),
                   "--dock-contact-x", str(float(values.get("dock_contact_x", -0.10)))]
    return [ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackagePrefix("mowbite_sim"), "lib", "mowbite_sim", "fake_tron"]),
             "--link", LaunchConfiguration("link"), "--gnss-link", LaunchConfiguration("gnss_link"),
             "--imu-link", LaunchConfiguration("imu_link"),
             "--flags", LaunchConfiguration("flags"), "--log", LaunchConfiguration("log"),
             "--state", LaunchConfiguration("state"),
             "--datum", LaunchConfiguration("datum_lat"), LaunchConfiguration("datum_long"),
             LaunchConfiguration("datum_height"), *station],
        output="screen")]


def generate_launch_description():
    link = LaunchConfiguration("link")
    gnss_link = LaunchConfiguration("gnss_link")
    imu_link = LaunchConfiguration("imu_link")
    datum_lat = LaunchConfiguration("datum_lat")
    datum_long = LaunchConfiguration("datum_long")
    datum_height = LaunchConfiguration("datum_height")
    sim_control = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackagePrefix("mowbite_sim"), "lib", "mowbite_sim", "sim_control"]),
             "--flags", LaunchConfiguration("flags")],
        output="screen")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("mowbite_bringup"), "launch",
                                                            "tron.launch.py"])),
        launch_arguments={"mcu_port": link, "gnss_port": gnss_link, "imu_port": imu_link,
                          "datum_lat": datum_lat, "datum_long": datum_long, "datum_height": datum_height,
                          "mqtt_host": LaunchConfiguration("mqtt_host"),
                          "mqtt_port": LaunchConfiguration("mqtt_port"),
                          "data_dir": LaunchConfiguration("data_dir"),
                          "extra_config": LaunchConfiguration("extra_config")}.items())
    return LaunchDescription([
        DeclareLaunchArgument("link", default_value="/tmp/mowbite_sim_mcu"),
        DeclareLaunchArgument("gnss_link", default_value="/tmp/mowbite_sim_gnss"),
        DeclareLaunchArgument("imu_link", default_value="/tmp/mowbite_sim_imu"),
        # Test datum, not a real garden.
        DeclareLaunchArgument("datum_lat", default_value="52.0"),
        DeclareLaunchArgument("datum_long", default_value="9.0"),
        DeclareLaunchArgument("datum_height", default_value="50.0"),
        DeclareLaunchArgument("state", default_value="/tmp/mowbite_sim_state.json"),
        DeclareLaunchArgument("flags", default_value="/tmp/mowbite_sim_flags.json"),
        DeclareLaunchArgument("log", default_value="/tmp/mowbite_sim_rx.jsonl"),
        DeclareLaunchArgument("mqtt_host", default_value=EnvironmentVariable("MOWBITE_MQTT_HOST",
                                                                             default_value="127.0.0.1")),
        DeclareLaunchArgument("mqtt_port", default_value=EnvironmentVariable("MOWBITE_MQTT_PORT",
                                                                             default_value="1883")),
        DeclareLaunchArgument("data_dir", default_value="/tmp/mowbite_sim_data"),
        DeclareLaunchArgument("extra_config", default_value=PathJoinSubstitution(
            [FindPackageShare("mowbite_sim"), "config", "sim.yaml"])),
        OpaqueFunction(function=fake_tron_setup),
        sim_control,
        # The pty has to exist before the driver looks for it.
        TimerAction(period=1.0, actions=[stack]),
    ])
