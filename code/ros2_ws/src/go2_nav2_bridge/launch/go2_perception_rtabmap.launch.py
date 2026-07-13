from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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
        "go2_rtabmap_live.rviz",
    ])

    network_interface = LaunchConfiguration("network_interface")
    params_file = LaunchConfiguration("params_file")
    start_bridge = LaunchConfiguration("start_bridge")
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
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    rtabmap_args = LaunchConfiguration("rtabmap_args")

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_interface",
            default_value="eth0",
            description="Network interface connected to the Go2 Unitree DDS network.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Base parameter file for go2_nav2_bridge.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the read-only Unitree DDS to ROS 2 perception bridge.",
        ),
        DeclareLaunchArgument(
            "start_rtabmap",
            default_value="true",
            description="Start RTAB-Map if rtabmap_ros is installed.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with live Go2 perception and RTAB-Map displays.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config for live Go2 perception and RTAB-Map.",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/go2/pointcloud",
            description="ROS 2 PointCloud2 topic used by RTAB-Map and RViz.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom",
            description="Go2 odometry topic published by the bridge.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/go2/imu",
            description="Go2 IMU topic published by the bridge.",
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
            "rtabmap_database_path",
            default_value="/home/luping/go2_rssi_runs/rtabmap/go2_live.db",
            description="RTAB-Map database output path.",
        ),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value="--delete_db_on_start",
            description="Extra command-line arguments passed to rtabmap.",
        ),
        Node(
            condition=IfCondition(start_bridge),
            package="go2_nav2_bridge",
            executable="go2_nav2_bridge",
            name="go2_perception_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "network_interface": network_interface,
                    "motion_client": "none",
                    "cmd_vel_topic": "/go2_perception_readonly/cmd_vel_unused",
                    "publish_point_cloud": True,
                    "ros_point_cloud_topic": cloud_topic,
                    "odom_topic": odom_topic,
                    "imu_topic": imu_topic,
                    "odom_frame": odom_frame_id,
                    "base_frame": frame_id,
                    "point_cloud_frame": frame_id,
                    "publish_tf": True,
                    "stand_on_start": False,
                    "classic_walk_on_start": False,
                    "speed_level_on_start": -1,
                    "obstacle_avoid_switch_on_start": False,
                    "obstacle_avoid_remote_api_on_start": False,
                    "stop_on_shutdown": False,
                    "require_fresh_point_cloud_for_motion": False,
                    "log_cmd_vel": False,
                    "log_odom_state": True,
                    "log_point_cloud_roi": True,
                },
            ],
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
                "use_sim_time": False,
                "database_path": rtabmap_database_path,
                "Grid/FromDepth": "false",
                "Grid/FromScan": "true",
                "Grid/3D": "true",
                "Grid/RangeMax": "8.0",
                "Grid/CellSize": "0.08",
                "RGBD/ProximityBySpace": "true",
                "RGBD/NeighborLinkRefining": "true",
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
            name="go2_rtabmap_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
