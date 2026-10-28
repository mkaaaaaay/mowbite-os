# MowBite OS against the simulated MCU, e.g. on the PC with a local mosquitto.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    link = LaunchConfiguration("link")
    fake_mcu = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackagePrefix("mowbite_sim"), "lib", "mowbite_sim", "fake_mcu"]),
             "--link", link, "--flags", LaunchConfiguration("flags"), "--log", LaunchConfiguration("log")],
        output="screen")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("mowbite_bringup"), "launch",
                                                            "tron.launch.py"])),
        launch_arguments={"mcu_port": link, "mqtt_host": LaunchConfiguration("mqtt_host"),
                          "mqtt_port": LaunchConfiguration("mqtt_port"),
                          "data_dir": LaunchConfiguration("data_dir")}.items())
    return LaunchDescription([
        DeclareLaunchArgument("link", default_value="/tmp/mowbite_sim_mcu"),
        DeclareLaunchArgument("flags", default_value="/tmp/mowbite_sim_flags.json"),
        DeclareLaunchArgument("log", default_value="/tmp/mowbite_sim_rx.jsonl"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("data_dir", default_value="/tmp/mowbite_sim_data"),
        fake_mcu,
        # The pty has to exist before the driver looks for it.
        TimerAction(period=1.0, actions=[stack]),
    ])
