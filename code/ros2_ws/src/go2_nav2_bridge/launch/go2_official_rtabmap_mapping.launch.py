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
        "go2_mapping_rtabmap.rviz",
    ])

    start_rtabmap = LaunchConfiguration("start_rtabmap")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    subscribe_imu = LaunchConfiguration("subscribe_imu")
    frame_id = LaunchConfiguration("frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    map_frame_id = LaunchConfiguration("map_frame_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    rtabmap_args = LaunchConfiguration("rtabmap_args")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rtabmap",
            default_value="true",
            description="Start RTAB-Map using official Go2 ROS topics.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with RTAB-Map and official Go2 voxel displays.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config for RTAB-Map mapping and live official voxel map.",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/utlidar/cloud_deskewed",
            description="Official Go2 deskewed PointCloud2 topic used as RTAB-Map scan_cloud input.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/utlidar/robot_odom",
            description="Official Go2 odometry topic used by RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/utlidar/imu",
            description="Official Go2 IMU topic used by RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "subscribe_imu",
            default_value="true",
            description="Whether RTAB-Map should subscribe to IMU.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="base_link",
            description="Robot base frame.",
        ),
        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="odom",
            description="Odometry frame.",
        ),
        DeclareLaunchArgument(
            "map_frame_id",
            default_value="map",
            description="RTAB-Map global frame.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock. Set true for offline rosbag playback.",
        ),
        DeclareLaunchArgument(
            "rtabmap_database_path",
            default_value="/home/luping/go2_rssi_runs/rtabmap/go2_official_mapping.db",
            description="RTAB-Map database output path.",
        ),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value="--delete_db_on_start",
            description="Extra command-line arguments passed to rtabmap.",
        ),
        Node(
            condition=IfCondition(start_rtabmap),
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[{
                "frame_id": frame_id,
                "odom_frame_id": odom_frame_id,
                "map_frame_id": map_frame_id,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,
                "subscribe_odom": True,
                "subscribe_imu": subscribe_imu,
                "approx_sync": True,
                "queue_size": 30,
                "use_sim_time": use_sim_time,
                "database_path": rtabmap_database_path,
                "Grid/FromDepth": "false",
                "Grid/FromScan": "true",
                "Grid/3D": "true",
                "Grid/CellSize": "0.08",
                "Grid/RangeMax": "12.0",
                "RGBD/ProximityBySpace": "true",
                "RGBD/NeighborLinkRefining": "true",
                "Reg/Strategy": "1",
                "Mem/IncrementalMemory": "true",
            }],
            remappings=[
                ("scan_cloud", cloud_topic),
                ("odom", odom_topic),
                ("imu", imu_topic),
            ],
            arguments=[rtabmap_args],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package="rviz2",
            executable="rviz2",
            name="go2_mapping_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
