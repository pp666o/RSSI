from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "rviz",
        "go2_voxel_map.rviz",
    ])

    network_interface = LaunchConfiguration("network_interface")
    voxel_topic = LaunchConfiguration("voxel_topic")
    points_topic = LaunchConfiguration("points_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    output_frame = LaunchConfiguration("output_frame")
    bit_order = LaunchConfiguration("bit_order")
    start_visualizer = LaunchConfiguration("start_visualizer")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_interface",
            default_value="enp5s0",
            description="Network interface connected to the Go2 DDS network.",
        ),
        DeclareLaunchArgument(
            "voxel_topic",
            default_value="rt/utlidar/voxel_map_compressed",
            description="Unitree DDS VoxelMapCompressed topic.",
        ),
        DeclareLaunchArgument(
            "points_topic",
            default_value="/go2/voxel_map_points",
            description="Decoded PointCloud2 topic for occupied voxels.",
        ),
        DeclareLaunchArgument(
            "marker_topic",
            default_value="/go2/voxel_map_marker",
            description="Decoded Marker CUBE_LIST topic for occupied voxels.",
        ),
        DeclareLaunchArgument(
            "output_frame",
            default_value="odom",
            description="Frame used for decoded voxel map visualization.",
        ),
        DeclareLaunchArgument(
            "bit_order",
            default_value="lsb",
            description="Bit order used to unpack the 1-bit voxel bitmap: lsb or msb.",
        ),
        DeclareLaunchArgument(
            "start_visualizer",
            default_value="true",
            description="Start the Unitree voxel decoder and ROS publishers.",
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
            condition=IfCondition(start_visualizer),
            package="go2_nav2_bridge",
            executable="go2_voxel_map_visualizer",
            name="go2_voxel_map_visualizer",
            output="screen",
            parameters=[{
                "network_interface": network_interface,
                "voxel_topic": voxel_topic,
                "points_topic": points_topic,
                "marker_topic": marker_topic,
                "output_frame": output_frame,
                "bit_order": bit_order,
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
