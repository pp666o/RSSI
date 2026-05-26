from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="rssi_localization",
                executable="rssi_localizer_node",
                name="rssi_localizer",
                output="screen",
                parameters=[
                    {
                        "radio_map_path": "",
                        "train_csv_glob": "",
                        "beacon_order": [
                            "beacon_1",
                            "beacon_2",
                            "beacon_3",
                            "beacon_4",
                            "beacon_6",
                            "beacon_9",
                        ],
                        "input_topic": "/rssi_scan",
                        "output_topic": "/path_s_estimate",
                        "debug_topic": "/path_localization_debug",
                        "start_s": 0.0,
                        "monotonic_increase": True,
                    }
                ],
            )
        ]
    )
