from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "params",
        "go2_nav2_straight_params.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "rviz",
        "go2_pointcloud.rviz",
    ])

    start_go2_bridge = LaunchConfiguration("start_go2_bridge")
    start_rviz = LaunchConfiguration("start_rviz")
    params_file = LaunchConfiguration("params_file")
    network_interface = LaunchConfiguration("network_interface")
    bridge_cmd_vel_topic = LaunchConfiguration("bridge_cmd_vel_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    rviz_config = LaunchConfiguration("rviz_config")
    stand_on_start = LaunchConfiguration("stand_on_start")
    classic_walk_on_start = LaunchConfiguration("classic_walk_on_start")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_go2_bridge",
            default_value="false",
            description="Start the Go2 hardware bridge. Disable if it is already running.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the Go2 pointcloud display.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Go2 bridge parameter file used when start_go2_bridge is true.",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="enp5s0",
            description="Network interface used by Unitree SDK2 DDS when start_go2_bridge is true.",
        ),
        DeclareLaunchArgument(
            "bridge_cmd_vel_topic",
            default_value="/go2_rviz_view/cmd_vel_unused",
            description="Command topic used by the optional bridge; default avoids consuming /cmd_vel.",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/go2/pointcloud",
            description="ROS 2 PointCloud2 topic published by go2_nav2_bridge.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config used for live Go2 pointcloud visualization.",
        ),
        DeclareLaunchArgument(
            "stand_on_start",
            default_value="false",
            description="Whether the optional Go2 bridge should call BalanceStand on startup.",
        ),
        DeclareLaunchArgument(
            "classic_walk_on_start",
            default_value="false",
            description="Whether the optional Go2 bridge should call ClassicWalk on startup.",
        ),
        Node(
            condition=IfCondition(start_go2_bridge),
            package="go2_nav2_bridge",
            executable="go2_nav2_bridge",
            name="go2_nav2_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "network_interface": network_interface,
                    "cmd_vel_topic": bridge_cmd_vel_topic,
                    "publish_point_cloud": True,
                    "ros_point_cloud_topic": cloud_topic,
                    "stand_on_start": ParameterValue(stand_on_start, value_type=bool),
                    "classic_walk_on_start": ParameterValue(classic_walk_on_start, value_type=bool),
                    "speed_level_on_start": -1,
                },
            ],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package="rviz2",
            executable="rviz2",
            name="go2_pointcloud_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
