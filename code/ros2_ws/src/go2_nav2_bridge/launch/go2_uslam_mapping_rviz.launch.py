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
        "go2_uslam_mapping.rviz",
    ])

    start_path_visualizer = LaunchConfiguration("start_path_visualizer")
    pose_topic = LaunchConfiguration("pose_topic")
    path_topic = LaunchConfiguration("path_topic")
    path_marker_topic = LaunchConfiguration("path_marker_topic")
    output_frame = LaunchConfiguration("output_frame")
    min_path_distance_m = LaunchConfiguration("min_path_distance_m")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_path_visualizer",
            default_value="true",
            description="Convert robot pose into a visible nav_msgs/Path and current-pose marker.",
        ),
        DeclareLaunchArgument(
            "pose_topic",
            default_value="/utlidar/robot_pose",
            description="Pose topic used to draw the mapping trajectory.",
        ),
        DeclareLaunchArgument(
            "path_topic",
            default_value="/go2/mapping_path",
            description="Output nav_msgs/Path topic for the mapping trajectory.",
        ),
        DeclareLaunchArgument(
            "path_marker_topic",
            default_value="/go2/mapping_pose_marker",
            description="Output marker topic for the current robot pose.",
        ),
        DeclareLaunchArgument(
            "output_frame",
            default_value="odom",
            description="Frame used by the mapping trajectory visualizer.",
        ),
        DeclareLaunchArgument(
            "min_path_distance_m",
            default_value="0.05",
            description="Minimum robot motion before adding a new trajectory point.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz for Unitree official USLAM mapping visualization.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config for official USLAM map, USLAM odometry, and Unitree voxel cloud.",
        ),
        Node(
            condition=IfCondition(start_path_visualizer),
            package="go2_nav2_bridge",
            executable="go2_pose_path_visualizer",
            name="go2_pose_path_visualizer",
            output="screen",
            parameters=[{
                "pose_topic": pose_topic,
                "path_topic": path_topic,
                "marker_topic": path_marker_topic,
                "output_frame": output_frame,
                "min_distance_m": min_path_distance_m,
            }],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package="rviz2",
            executable="rviz2",
            name="go2_uslam_mapping_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
