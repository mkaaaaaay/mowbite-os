# All of MowBite OS in one process: one ROS context and one DDS participant for every node.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    mcu = ComposableNode(
        package="tron_mcu", plugin="tron_mcu::McuNode", name="tron_mcu",
        parameters=[config, {"port": LaunchConfiguration("mcu_port")}])
    logic = ComposableNode(
        package="mowbite_logic", plugin="mowbite_logic::LogicNode", name="mowbite_logic", parameters=[config])
    bridge = ComposableNode(
        package="mowbite_bridge", plugin="mowbite_bridge::BridgeNode", name="mowbite_bridge",
        parameters=[config, {"mqtt_host": LaunchConfiguration("mqtt_host"),
                             "mqtt_port": ParameterValue(LaunchConfiguration("mqtt_port"), value_type=int),
                             "data_dir": LaunchConfiguration("data_dir")}])
    container = ComposableNodeContainer(
        name="mowbite", namespace="", package="rclcpp_components", executable="component_container",
        composable_node_descriptions=[mcu, logic, bridge], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [FindPackageShare("mowbite_bringup"), "config", "tron.yaml"])),
        DeclareLaunchArgument("mcu_port", default_value="/dev/ttyS9"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("data_dir", default_value="/userdata/mowbite"),
        container,
    ])
