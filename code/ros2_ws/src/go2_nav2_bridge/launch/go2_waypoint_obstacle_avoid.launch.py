import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_relative_waypoints(context):
    raw_value = LaunchConfiguration("relative_waypoints").perform(context)
    try:
        parsed = ast.literal_eval(raw_value)
    except (SyntaxError, ValueError) as exc:
        raise ValueError(
            "relative_waypoints must be a Python/YAML-style list, for example "
            "'[3.0, 0.0, 0.0]'."
        ) from exc

    if not isinstance(parsed, (list, tuple)):
        raise ValueError("relative_waypoints must be a list of x,y,yaw triples.")
    if len(parsed) == 0 or len(parsed) % 3 != 0:
        raise ValueError(
            "relative_waypoints must contain one or more x,y,yaw triples: "
            "[x1, y1, yaw1, x2, y2, yaw2, ...]."
        )

    try:
        return [float(value) for value in parsed]
    except (TypeError, ValueError) as exc:
        raise ValueError("relative_waypoints entries must all be numeric.") from exc


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    network_interface = LaunchConfiguration("network_interface")
    start_bridge = LaunchConfiguration("start_bridge")
    start_follower = LaunchConfiguration("start_follower")
    relative_waypoints = _parse_relative_waypoints(context)
    ros2_env = {
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_DOMAIN_ID": "10",
        "ROS2_DISABLE_DAEMON": "1",
    }

    return [
        Node(
            condition=IfCondition(start_bridge),
            package="go2_nav2_bridge",
            executable="go2_nav2_bridge",
            name="go2_nav2_bridge",
            output="screen",
            additional_env={
                **ros2_env,
                "GO2_NAV2_BRIDGE_NETWORK_INTERFACE": network_interface,
            },
            parameters=[
                params_file,
                {
                    "network_interface": network_interface,
                    "motion_client": "obstacles_avoid",
                    "obstacle_avoid_switch_on_start": True,
                    "obstacle_avoid_remote_api_on_start": True,
                    "publish_point_cloud": False,
                    "require_fresh_point_cloud_for_motion": False,
                    "log_point_cloud_roi": False,
                },
            ],
        ),
        Node(
            condition=IfCondition(start_follower),
            package="go2_nav2_bridge",
            executable="go2_waypoint_follower",
            name="go2_waypoint_follower",
            output="screen",
            additional_env=ros2_env,
            parameters=[
                params_file,
                {
                    "relative_waypoints": relative_waypoints,
                },
            ],
        ),
    ]


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare("go2_nav2_bridge"),
        "params",
        "go2_nav2_straight_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Unified Go2 waypoint collection parameter file.",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="enp5s0",
            description="Network interface used by Unitree SDK2 DDS.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the Go2 hardware bridge using Unitree obstacles_avoid as motion backend.",
        ),
        DeclareLaunchArgument(
            "start_follower",
            default_value="true",
            description="Start the waypoint follower.",
        ),
        DeclareLaunchArgument(
            "relative_waypoints",
            default_value="[3.0, 0.0, 0.0]",
            description="Relative waypoint triples in the start frame: [x_m, y_m, yaw_rad, ...].",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
