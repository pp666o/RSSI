from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "params",
        "go2_nav2_straight_params.yaml",
    ])
    params_file = LaunchConfiguration("params_file")
    network_interface = LaunchConfiguration("network_interface")
    move_backend = LaunchConfiguration("move_backend")
    use_bt_navigator = LaunchConfiguration("use_bt_navigator")
    use_planner = LaunchConfiguration("use_planner")
    start_voxel_cloud = LaunchConfiguration("start_voxel_cloud")
    planner_needed = PythonExpression([
        "'", use_planner, "' == 'true' or '", use_bt_navigator, "' == 'true'",
    ])
    planner_not_needed = PythonExpression([
        "not ('", use_planner, "' == 'true' or '", use_bt_navigator, "' == 'true')",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the unified Go2 Nav2 3D pointcloud collection parameter file.",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="enp5s0",
            description="Network interface used by Unitree SDK2 DDS.",
        ),
        DeclareLaunchArgument(
            "move_backend",
            default_value="nav2",
            description="nav2 starts the Nav2 controller; cmd_vel starts only the Go2 bridge.",
        ),
        DeclareLaunchArgument(
            "use_bt_navigator",
            default_value="false",
            description="Start Nav2 bt_navigator/behavior_server for NavigateToPose replanning.",
        ),
        DeclareLaunchArgument(
            "use_planner",
            default_value="true",
            description="Start planner_server for ComputePathToPose/NavigateToPose.",
        ),
        DeclareLaunchArgument(
            "start_voxel_cloud",
            default_value="true",
            description="Convert local voxel_grid to voxel_marked_cloud for RViz visualization.",
        ),
        Node(
            package="go2_nav2_bridge",
            executable="go2_nav2_bridge",
            name="go2_nav2_bridge",
            output="screen",
            parameters=[
                params_file,
                {"network_interface": network_interface},
            ],
        ),
        GroupAction(
            condition=IfCondition(EqualsSubstitution(move_backend, "nav2")),
            actions=[
                Node(
                    condition=IfCondition(planner_needed),
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
                GroupAction(
                    condition=IfCondition(use_bt_navigator),
                    actions=[
                        Node(
                            package="nav2_behaviors",
                            executable="behavior_server",
                            name="behavior_server",
                            output="screen",
                            parameters=[params_file],
                        ),
                        Node(
                            package="nav2_bt_navigator",
                            executable="bt_navigator",
                            name="bt_navigator",
                            output="screen",
                            parameters=[params_file],
                        ),
                        Node(
                            package="nav2_lifecycle_manager",
                            executable="lifecycle_manager",
                            name="lifecycle_manager_bt",
                            output="screen",
                            parameters=[{
                                "use_sim_time": False,
                                "autostart": True,
                                "node_names": ["behavior_server", "bt_navigator"],
                            }],
                        ),
                    ],
                ),
                Node(
                    condition=IfCondition(planner_needed),
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[params_file],
                ),
                Node(
                    condition=IfCondition(planner_not_needed),
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[
                        params_file,
                        {
                            "use_sim_time": False,
                            "autostart": True,
                            "node_names": [
                                "controller_server",
                                "velocity_smoother",
                                "collision_monitor",
                            ],
                        },
                    ],
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
            ],
        ),
    ])
