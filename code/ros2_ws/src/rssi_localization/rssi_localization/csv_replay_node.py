from __future__ import annotations

from pathlib import Path

import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CsvReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("csv_replay_rssi")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("output_topic", "/rssi_scan")
        self.declare_parameter("beacon_order", ["beacon_1", "beacon_2", "beacon_3", "beacon_4", "beacon_6", "beacon_9"])
        self.declare_parameter("loop_playback", True)

        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        if not csv_path:
            raise ValueError("csv_path parameter is required.")

        self.df = pd.read_csv(Path(csv_path))
        self.beacon_order = list(self.get_parameter("beacon_order").value)
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.loop_playback = bool(self.get_parameter("loop_playback").value)
        self.row_idx = 0

        self.publisher = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / max(hz, 1e-6), self.publish_next)

        self.get_logger().info(f"Replaying {csv_path} to {self.output_topic}")

    def publish_next(self) -> None:
        if self.row_idx >= len(self.df):
            if not self.loop_playback:
                self.get_logger().info("CSV replay finished.")
                self.timer.cancel()
                return
            self.row_idx = 0

        row = self.df.iloc[self.row_idx]
        msg = Float32MultiArray()
        msg.data = [float(row[f"{beacon}_mean"]) for beacon in self.beacon_order]
        self.publisher.publish(msg)
        self.row_idx += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CsvReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
