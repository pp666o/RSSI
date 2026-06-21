#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


@dataclass
class BoxStats:
    count: int = 0
    min_x: float = math.inf

    def add(self, x: float) -> None:
        self.count += 1
        self.min_x = min(self.min_x, x)

    def text(self) -> str:
        if self.count == 0:
            return "0/inf"
        return f"{self.count}/{self.min_x:.2f}m"


class Nav2PerceptionDiag(Node):
    def __init__(self, duration_sec: float) -> None:
        super().__init__("go2_nav2_perception_diag")
        self.duration_sec = duration_sec
        self.cloud_frames = 0
        self.last_cloud_boxes: Dict[str, BoxStats] = {}
        self.max_cloud_boxes: Dict[str, BoxStats] = {}
        self.last_valid_points = 0
        self.costmap_frames = 0
        self.last_costmap_summary: Optional[str] = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PointCloud2, "/go2/pointcloud", self.on_cloud, sensor_qos)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_costmap, map_qos)

    def on_cloud(self, msg: PointCloud2) -> None:
        boxes = {
            "hard_front_stop": BoxStats(),
            "right_side_stop": BoxStats(),
            "right_side_inner": BoxStats(),
            "right_side_outer": BoxStats(),
            "right_side_forward": BoxStats(),
            "left_side_stop": BoxStats(),
            "left_side_inner": BoxStats(),
            "left_side_outer": BoxStats(),
            "left_side_forward": BoxStats(),
            "front_center": BoxStats(),
            "front_left": BoxStats(),
            "front_right": BoxStats(),
            "body_slow": BoxStats(),
            "side_slow": BoxStats(),
            "near_body": BoxStats(),
        }
        valid = 0
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            valid += 1
            if 0.24 <= x <= 0.58 and -0.18 <= y <= 0.18 and -0.20 <= z <= 0.90:
                boxes["hard_front_stop"].add(x)
            if -0.30 <= x <= 0.95 and -0.85 <= y <= -0.52 and -0.20 <= z <= 0.90:
                boxes["right_side_stop"].add(x)
            if -0.30 <= x <= 0.42 and -0.52 <= y <= -0.30 and -0.20 <= z <= 0.90:
                boxes["right_side_inner"].add(x)
            if -0.30 <= x <= 0.95 and -0.95 <= y < -0.85 and -0.20 <= z <= 0.90:
                boxes["right_side_outer"].add(x)
            if 0.42 < x <= 0.95 and -0.52 <= y <= -0.30 and -0.20 <= z <= 0.90:
                boxes["right_side_forward"].add(x)
            if -0.30 <= x <= 0.95 and 0.52 <= y <= 0.85 and -0.20 <= z <= 0.90:
                boxes["left_side_stop"].add(x)
            if -0.30 <= x <= 0.42 and 0.30 <= y <= 0.52 and -0.20 <= z <= 0.90:
                boxes["left_side_inner"].add(x)
            if -0.30 <= x <= 0.95 and 0.85 < y <= 0.95 and -0.20 <= z <= 0.90:
                boxes["left_side_outer"].add(x)
            if 0.42 < x <= 0.95 and 0.30 <= y <= 0.52 and -0.20 <= z <= 0.90:
                boxes["left_side_forward"].add(x)
            if 0.20 <= x <= 1.20 and -0.20 <= y <= 0.20 and -0.20 <= z <= 0.90:
                boxes["front_center"].add(x)
            if 0.20 <= x <= 1.20 and 0.20 < y <= 0.65 and -0.20 <= z <= 0.90:
                boxes["front_left"].add(x)
            if 0.20 <= x <= 1.20 and -0.65 <= y < -0.20 and -0.20 <= z <= 0.90:
                boxes["front_right"].add(x)
            if -0.25 <= x <= 1.10 and -0.55 <= y <= 0.55 and -0.20 <= z <= 0.90:
                boxes["body_slow"].add(x)
            if -0.35 <= x <= 0.60 and -0.62 <= y <= 0.62 and -0.20 <= z <= 0.90:
                boxes["side_slow"].add(x)
            if -0.45 <= x <= 0.45 and -0.65 <= y <= 0.65 and -0.30 <= z <= 0.90:
                boxes["near_body"].add(x)

        self.cloud_frames += 1
        self.last_valid_points = valid
        self.last_cloud_boxes = boxes
        for name, stats in boxes.items():
            current = self.max_cloud_boxes.setdefault(name, BoxStats())
            current.count = max(current.count, stats.count)
            current.min_x = min(current.min_x, stats.min_x)

    def on_costmap(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        lethal = sum(1 for v in data if v >= 90)
        occupied = sum(1 for v in data if v >= 50)
        unknown = sum(1 for v in data if v < 0)

        cx = width // 2
        cy = height // 2
        radius_cells = max(1, int(0.60 / msg.info.resolution))
        center_occupied = 0
        center_lethal = 0
        for y in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
            row = y * width
            for x in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
                v = data[row + x]
                if v >= 50:
                    center_occupied += 1
                if v >= 90:
                    center_lethal += 1

        self.costmap_frames += 1
        self.last_costmap_summary = (
            f"frames={self.costmap_frames} size={width}x{height} res={msg.info.resolution:.3f} "
            f"occupied>=50={occupied} lethal>=90={lethal} unknown={unknown} "
            f"center_1.2m occupied={center_occupied} lethal={center_lethal}"
        )

    def summary(self) -> str:
        lines = [
            f"POINTCLOUD frames={self.cloud_frames} last_valid={self.last_valid_points}",
        ]
        names = [
            "hard_front_stop",
            "right_side_stop",
            "right_side_inner",
            "right_side_outer",
            "right_side_forward",
            "left_side_stop",
            "left_side_inner",
            "left_side_outer",
            "left_side_forward",
            "front_center",
            "front_left",
            "front_right",
            "body_slow",
            "side_slow",
            "near_body",
        ]
        for name in names:
            last = self.last_cloud_boxes.get(name, BoxStats()).text()
            max_seen = self.max_cloud_boxes.get(name, BoxStats()).text()
            lines.append(f"POINTCLOUD {name}: last={last} max_seen={max_seen}")
        lines.append(f"COSTMAP {self.last_costmap_summary or 'no frames'}")
        return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = Nav2PerceptionDiag(args.duration)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.duration
    while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(node.summary())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
