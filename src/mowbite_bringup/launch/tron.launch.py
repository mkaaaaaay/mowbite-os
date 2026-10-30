# All of MowBite OS in one process: one ROS context and one DDS participant for every node. Only the MowBite Planner
# (Python, shapely) runs as a process of its own.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    # Values on top of config, e.g. those of the simulation; by default config once more.
    extra = LaunchConfiguration("extra_config")
    mcu = ComposableNode(
        package="tron_mcu", plugin="tron_mcu::McuNode", name="tron_mcu",
        parameters=[config, extra, {"port": LaunchConfiguration("mcu_port")}])
    gnss = ComposableNode(
        package="tron_gnss", plugin="tron_gnss::GnssNode", name="tron_gnss",
        parameters=[config, extra, {"port": LaunchConfiguration("gnss_port"),
                             "datum_lat": ParameterValue(LaunchConfiguration("datum_lat"), value_type=float),
                             "datum_long": ParameterValue(LaunchConfiguration("datum_long"), value_type=float),
                             "datum_height": ParameterValue(LaunchConfiguration("datum_height"), value_type=float)}])
    imu = ComposableNode(
        package="tron_imu", plugin="tron_imu::ImuNode", name="tron_imu",
        parameters=[config, extra, {"port": LaunchConfiguration("imu_port")}])
    localization = ComposableNode(
        package="mowbite_localization", plugin="mowbite_localization::LocalizationNode", name="mowbite_localization",
        parameters=[config, extra])
    logic = ComposableNode(
        package="mowbite_logic", plugin="mowbite_logic::LogicNode", name="mowbite_logic",
        parameters=[config, extra, {"data_dir": LaunchConfiguration("data_dir")}])
    bridge = ComposableNode(
        package="mowbite_bridge", plugin="mowbite_bridge::BridgeNode", name="mowbite_bridge",
        parameters=[config, extra, {"mqtt_host": LaunchConfiguration("mqtt_host"),
                             "mqtt_port": ParameterValue(LaunchConfiguration("mqtt_port"), value_type=int),
                             "data_dir": LaunchConfiguration("data_dir")}])
    map_node = ComposableNode(
        package="mowbite_map", plugin="mowbite_map::MapNode", name="mowbite_map",
        parameters=[config, extra, {"data_dir": LaunchConfiguration("data_dir")}])
    follower = ComposableNode(
        package="mowbite_follower", plugin="mowbite_follower::FollowerNode", name="mowbite_follower",
        parameters=[config, extra])
    # Corrections from <data_dir>/ntrip.yaml for tron_gnss; without the file it only reports "off".
    ntrip = ComposableNode(
        package="mowbite_ntrip", plugin="mowbite_ntrip::NtripNode", name="mowbite_ntrip",
        parameters=[config, extra, {"data_dir": LaunchConfiguration("data_dir")}])
    container = ComposableNodeContainer(
        name="mowbite", namespace="", package="rclcpp_components", executable="component_container",
        composable_node_descriptions=[mcu, gnss, imu, localization, logic, bridge, map_node, follower,
                                      ntrip], output="screen")
    # Plans the mowing paths for mowing.plan; a crash only costs the plans until it is back.
    planner = Node(package="mowbite_planner", executable="mowbite_planner_node", name="mowbite_planner",
                   parameters=[config, extra, {"data_dir": LaunchConfiguration("data_dir")}], output="screen",
                   respawn=True, respawn_delay=5.0)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [FindPackageShare("mowbite_bringup"), "config", "tron.yaml"])),
        DeclareLaunchArgument("extra_config", default_value=config),
        DeclareLaunchArgument("mcu_port", default_value="/dev/ttyS9"),
        DeclareLaunchArgument("gnss_port", default_value="/dev/ttyS4"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyS1"),
        # The datum of the garden stays out of the repo; without it tron_gnss publishes nothing.
        DeclareLaunchArgument("datum_lat", default_value="nan"),
        DeclareLaunchArgument("datum_long", default_value="nan"),
        DeclareLaunchArgument("datum_height", default_value="0.0"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("data_dir", default_value="/userdata/mowbite"),
        # Local time for the heartbeat of the MCU (and later the schedules); a TZ from the environment wins.
        DeclareLaunchArgument("tz", default_value=EnvironmentVariable("TZ", default_value="Europe/Berlin")),
        SetEnvironmentVariable("TZ", LaunchConfiguration("tz")),
        # DDS stays on the mower unless the environment says otherwise. Set before the first ROS node of this launch,
        # which launch_ros creates only to load the components.
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE",
                               EnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", default_value="LOCALHOST")),
        container,
        planner,
    ])
