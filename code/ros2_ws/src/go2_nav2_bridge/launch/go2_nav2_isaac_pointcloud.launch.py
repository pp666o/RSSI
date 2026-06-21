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
        "go2_nav2_isaac_pointcloud_params.yaml",
    ])

    params_file = LaunchConfiguration("params_file")
    use_bt_navigator = LaunchConfiguration("use_bt_navigator")
    start_voxel_cloud = LaunchConfiguration("start_voxel_cloud")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Nav2 parameters for Isaac Go2 pointcloud obstacle testing.",
        ),
        DeclareLaunchArgument(
            "use_bt_navigator",
            default_value="false",
            description="Start behavior_server and bt_navigator for NavigateToPose tests.",
        ),
        DeclareLaunchArgument(
            "start_voxel_cloud",
            default_value="true",
            description="Convert local voxel_grid to a PointCloud2 topic for RViz debugging.",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params_file],
            remappings=[
                ("cmd_vel", "/cmd_vel_nav"),
                ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
            ],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            condition=IfCondition(use_bt_navigator),
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            condition=IfCondition(use_bt_navigator),
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            condition=IfCondition(use_bt_navigator),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_bt",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["behavior_server", "bt_navigator"],
            }],
        ),
        Node(
            condition=IfCondition(start_voxel_cloud),
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d_cloud",
            name="local_voxel_cloud",
            output="screen",
            remappings=[
                ("voxel_grid", "/local_costmap/voxel_grid"),
                ("voxel_marked_cloud", "/local_costmap/voxel_marked_cloud"),
                ("voxel_unknown_cloud", "/local_costmap/voxel_unknown_cloud"),
            ],
        ),
    ])
