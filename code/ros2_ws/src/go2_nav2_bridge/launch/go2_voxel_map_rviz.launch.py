from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "rviz",
        "go2_voxel_map.rviz",
    ])

    network_interface = LaunchConfiguration("network_interface")
    voxel_topic = LaunchConfiguration("voxel_topic")
    bit_order = LaunchConfiguration("bit_order")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_marker = LaunchConfiguration("publish_marker")
    publish_points = LaunchConfiguration("publish_points")

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_interface",
            default_value="enp5s0",
            description="Network interface used by Unitree SDK2 DDS.",
        ),
        DeclareLaunchArgument(
            "voxel_topic",
            default_value="rt/utlidar/voxel_map_compressed",
            description="Unitree DDS VoxelMapCompressed topic.",
        ),
        DeclareLaunchArgument(
            "bit_order",
            default_value="lsb",
            description="Bit order used to expand the decoded 1-bit occupancy bitmap: lsb or msb.",
        ),
        DeclareLaunchArgument(
            "publish_marker",
            default_value="true",
            description="Publish /go2/voxel_map_marker as CUBE_LIST.",
        ),
        DeclareLaunchArgument(
            "publish_points",
            default_value="true",
            description="Publish /go2/voxel_map_points as PointCloud2.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the voxel map display.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config used for live Go2 voxel map visualization.",
        ),
        Node(
            package="go2_nav2_bridge",
            executable="go2_voxel_map_visualizer",
            name="go2_voxel_map_visualizer",
            output="screen",
            parameters=[{
                "network_interface": network_interface,
                "voxel_topic": voxel_topic,
                "bit_order": bit_order,
                "publish_marker": ParameterValue(publish_marker, value_type=bool),
                "publish_points": ParameterValue(publish_points, value_type=bool),
            }],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package="rviz2",
            executable="rviz2",
            name="go2_voxel_map_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
