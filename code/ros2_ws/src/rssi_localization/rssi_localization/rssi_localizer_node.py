from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

from .core import LocalizerConfig, RssiPathLocalizer


class RssiLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("rssi_localizer")

        self.declare_parameter("radio_map_path", "")
        self.declare_parameter("train_csv_glob", "")
        self.declare_parameter("beacon_order", ["beacon_1", "beacon_2", "beacon_3", "beacon_4", "beacon_6", "beacon_9"])
        self.declare_parameter("input_topic", "/rssi_scan")
        self.declare_parameter("output_topic", "/path_s_estimate")
        self.declare_parameter("debug_topic", "/path_localization_debug")
        self.declare_parameter("start_s", 0.0)
        self.declare_parameter("monotonic_increase", True)

        radio_map_path = self.get_parameter("radio_map_path").get_parameter_value().string_value
        train_csv_glob = self.get_parameter("train_csv_glob").get_parameter_value().string_value
        beacon_order = list(self.get_parameter("beacon_order").value)

        config = LocalizerConfig(
            start_s=float(self.get_parameter("start_s").value),
            monotonic_increase=bool(self.get_parameter("monotonic_increase").value),
        )

        if radio_map_path:
            self.localizer = RssiPathLocalizer.from_radio_map_csv(radio_map_path, config=config)
        elif train_csv_glob:
            self.localizer = RssiPathLocalizer.from_train_inputs([train_csv_glob], config=config)
        else:
            raise ValueError("Either radio_map_path or train_csv_glob must be provided.")

        if beacon_order and beacon_order != self.localizer.beacon_ids:
            self.get_logger().warning(
                f"Configured beacon_order {beacon_order} does not match loaded model order {self.localizer.beacon_ids}. "
                "The loaded model order will be used."
            )

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value

        self.s_pub = self.create_publisher(Float32, output_topic, 10)
        self.debug_pub = self.create_publisher(String, debug_topic, 10)
        self.scan_sub = self.create_subscription(Float32MultiArray, input_topic, self.on_scan, 10)

        self.get_logger().info(f"RSSI localizer ready. listening on {input_topic}")

    def on_scan(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != len(self.localizer.beacon_ids):
            self.get_logger().warning(
                f"Expected {len(self.localizer.beacon_ids)} RSSI values, got {len(msg.data)}. dropping frame."
            )
            return

        result = self.localizer.update(list(msg.data))

        out = Float32()
        out.data = float(result["s_pred"])
        self.s_pub.publish(out)

        debug = String()
        debug.data = json.dumps(result, ensure_ascii=True)
        self.debug_pub.publish(debug)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RssiLocalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
