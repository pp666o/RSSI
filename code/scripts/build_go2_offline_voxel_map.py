#!/usr/bin/env python3
"""Build an offline 2D/3D map from Go2 SDK voxel captures.

Input is the directory produced by go2_capture_voxel_cloud_raw.sh:
  metadata.jsonl
  voxel_raw/*.bin
  cloud_raw/*.bin

The map is built from VoxelMapCompressed frames. When pose/odom records are
present in metadata.jsonl they are used as the trajectory source. Otherwise
the script falls back to the center of each voxel frame. That fallback is only
useful if the voxel map origin moves in the map frame.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

from decode_go2_voxel_map import decompress_lz4_block, load_metadata


Point = tuple[float, float, float]
GridKey = tuple[int, int, int]


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def bit_is_set(payload: bytes, linear: int, bit_order: str) -> bool:
    byte = payload[linear // 8]
    bit_index = linear % 8
    if bit_order == "msb":
        return bool((byte >> (7 - bit_index)) & 1)
    return bool((byte >> bit_index) & 1)


def quantize(value: float, resolution: float) -> int:
    return int(round(value / resolution))


def voxel_center(record: dict) -> Point:
    origin = [float(x) for x in record["origin"]]
    width = [int(x) for x in record["width"]]
    resolution = float(record["resolution"])
    return (
        origin[0] + 0.5 * width[0] * resolution,
        origin[1] + 0.5 * width[1] * resolution,
        origin[2] + 0.5 * width[2] * resolution,
    )


def decode_voxel_payload(capture_dir: Path, record: dict) -> bytes:
    raw = (capture_dir / record["file"]).read_bytes()
    return decompress_lz4_block(raw, int(record["src_size"]))


def iter_occupied_world_points(record: dict, decoded: bytes, bit_order: str) -> Iterable[Point]:
    width = [int(x) for x in record["width"]]
    origin = [float(x) for x in record["origin"]]
    resolution = float(record["resolution"])
    wx, wy, wz = width
    total = wx * wy * wz
    for linear in range(total):
        if not bit_is_set(decoded, linear, bit_order):
            continue
        ix = linear % wx
        iy = (linear // wx) % wy
        iz = linear // (wx * wy)
        yield (
            origin[0] + (ix + 0.5) * resolution,
            origin[1] + (iy + 0.5) * resolution,
            origin[2] + (iz + 0.5) * resolution,
        )


def record_stamp(record: dict) -> float:
    if "stamp" in record:
        return float(record["stamp"])
    if "stamp_sec" in record:
        return float(record["stamp_sec"]) + float(record.get("stamp_nanosec", 0)) * 1e-9
    return float(record.get("recv_steady_sec", 0.0))


def pose_records(records: list[dict]) -> list[dict]:
    candidates = [r for r in records if r.get("type") in {"pose", "odom"}]
    candidates.sort(key=record_stamp)
    return candidates


def point_from_pose_record(record: dict) -> Point | None:
    if "position" in record:
        position = record["position"]
        return float(position[0]), float(position[1]), float(position[2])
    if "pose" in record and "position" in record["pose"]:
        position = record["pose"]["position"]
        return float(position[0]), float(position[1]), float(position[2])
    return None


def build_trajectory(records: list[dict], voxel_records: list[dict], min_distance: float) -> tuple[str, list[dict]]:
    poses = pose_records(records)
    raw_points: list[dict] = []
    source = "pose_or_odom"
    for rec in poses:
        point = point_from_pose_record(rec)
        if point is None:
            continue
        raw_points.append(
            {
                "stamp": record_stamp(rec),
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "source": rec.get("type", "pose"),
            }
        )

    if not raw_points:
        source = "voxel_frame_center"
        for rec in voxel_records:
            x, y, z = voxel_center(rec)
            raw_points.append(
                {
                    "stamp": record_stamp(rec),
                    "x": x,
                    "y": y,
                    "z": z,
                    "source": "voxel_center",
                }
            )

    downsampled: list[dict] = []
    for point in raw_points:
        if not downsampled:
            downsampled.append(point)
            continue
        prev = downsampled[-1]
        distance = math.hypot(point["x"] - prev["x"], point["y"] - prev["y"])
        if distance >= min_distance:
            downsampled.append(point)
    if len(downsampled) == 1 and raw_points:
        last = raw_points[-1]
        if last is not downsampled[0]:
            distance = math.hypot(last["x"] - downsampled[0]["x"], last["y"] - downsampled[0]["y"])
            if distance > 1e-6:
                downsampled.append(last)

    for i, point in enumerate(downsampled):
        if len(downsampled) == 1:
            yaw = 0.0
        elif i + 1 < len(downsampled):
            nxt = downsampled[i + 1]
            yaw = math.atan2(nxt["y"] - point["y"], nxt["x"] - point["x"])
        else:
            prev = downsampled[i - 1]
            yaw = math.atan2(point["y"] - prev["y"], point["x"] - prev["x"])
        point["yaw"] = yaw
        point["index"] = i

    return source, downsampled


def write_global_ply(path: Path, voxels: dict[GridKey, int], resolution: float) -> None:
    if not voxels:
        path.write_text("ply\nformat ascii 1.0\nelement vertex 0\nend_header\n", encoding="utf-8")
        return
    z_values = [key[2] * resolution for key in voxels]
    z_min = min(z_values)
    z_max = max(z_values)
    z_span = max(1e-9, z_max - z_min)
    max_hits = max(voxels.values())
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(voxels)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("property uchar hit_count\n")
        handle.write("end_header\n")
        for key, hits in sorted(voxels.items()):
            x = key[0] * resolution
            y = key[1] * resolution
            z = key[2] * resolution
            t = (z - z_min) / z_span
            red = int(255 * t)
            green = int(255 * (1.0 - abs(t - 0.5) * 2.0))
            blue = int(255 * (1.0 - t))
            hit_count = min(255, int(round(255 * hits / max_hits)))
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {red} {green} {blue} {hit_count}\n")


def write_path_csv(path: Path, trajectory: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "stamp", "x", "y", "z", "yaw", "source"])
        for point in trajectory:
            writer.writerow(
                [
                    point["index"],
                    f"{point['stamp']:.9f}",
                    f"{point['x']:.6f}",
                    f"{point['y']:.6f}",
                    f"{point['z']:.6f}",
                    f"{point['yaw']:.6f}",
                    point["source"],
                ]
            )


def write_path_ply(path: Path, trajectory: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(trajectory)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        edge_count = max(0, len(trajectory) - 1)
        handle.write(f"element edge {edge_count}\n")
        handle.write("property int vertex1\nproperty int vertex2\n")
        handle.write("end_header\n")
        for point in trajectory:
            handle.write(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f} 255 40 40\n")
        for i in range(edge_count):
            handle.write(f"{i} {i + 1}\n")


def write_relative_waypoints(output_dir: Path, trajectory: list[dict]) -> None:
    csv_path = output_dir / "waypoints_relative.csv"
    text_path = output_dir / "waypoints_relative_text.txt"
    values: list[float] = []
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "x_rel", "y_rel", "yaw_rel"])
        if len(trajectory) < 2:
            text_path.write_text("", encoding="utf-8")
            return
        origin = trajectory[0]
        x0 = origin["x"]
        y0 = origin["y"]
        yaw0 = origin["yaw"]
        c = math.cos(-yaw0)
        s = math.sin(-yaw0)
        for point in trajectory[1:]:
            dx = point["x"] - x0
            dy = point["y"] - y0
            x_rel = c * dx - s * dy
            y_rel = s * dx + c * dy
            yaw_rel = wrap_angle(point["yaw"] - yaw0)
            writer.writerow([point["index"], f"{x_rel:.6f}", f"{y_rel:.6f}", f"{yaw_rel:.6f}"])
            values.extend([x_rel, y_rel, yaw_rel])
    text_path.write_text(",".join(f"{value:.6f}" for value in values) + ("\n" if values else ""), encoding="utf-8")


def write_2d_map(
    output_dir: Path,
    voxels: dict[GridKey, int],
    resolution: float,
    trajectory: list[dict],
    max_svg_cells: int,
) -> dict:
    if not voxels:
        return {"error": "no occupied voxels"}

    xy_cells = {(key[0], key[1]) for key in voxels}
    min_x = min(x for x, _ in xy_cells)
    max_x = max(x for x, _ in xy_cells)
    min_y = min(y for _, y in xy_cells)
    max_y = max(y for _, y in xy_cells)
    width = max_x - min_x + 1
    height = max_y - min_y + 1

    pgm_path = output_dir / "map_2d.pgm"
    yaml_path = output_dir / "map_2d.yaml"
    with pgm_path.open("wb") as handle:
        handle.write(f"P5\n# Go2 offline voxel occupancy map\n{width} {height}\n255\n".encode("ascii"))
        occupied = xy_cells
        for row in range(height):
            y = max_y - row
            line = bytearray()
            for col in range(width):
                x = min_x + col
                line.append(0 if (x, y) in occupied else 205)
            handle.write(line)

    origin_x = min_x * resolution
    origin_y = min_y * resolution
    yaml_path.write_text(
        "\n".join(
            [
                "image: map_2d.pgm",
                "mode: trinary",
                f"resolution: {resolution:.9f}",
                f"origin: [{origin_x:.9f}, {origin_y:.9f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )

    svg_path = output_dir / "map_2d_path.svg"
    cells_for_svg = sorted(xy_cells)
    stride = max(1, math.ceil(len(cells_for_svg) / max(1, max_svg_cells)))
    view_w = max(1, width)
    view_h = max(1, height)
    with svg_path.open("w", encoding="utf-8") as handle:
        handle.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {view_w} {view_h}" '
            f'width="{max(600, min(1800, view_w * 4))}" height="{max(400, min(1400, view_h * 4))}">\n'
        )
        handle.write('<rect x="0" y="0" width="100%" height="100%" fill="#f3f4f6"/>\n')
        for x, y in cells_for_svg[::stride]:
            sx = x - min_x
            sy = max_y - y
            handle.write(f'<rect x="{sx}" y="{sy}" width="1" height="1" fill="#111827"/>\n')
        if trajectory:
            points = []
            for point in trajectory:
                sx = (point["x"] / resolution) - min_x
                sy = max_y - (point["y"] / resolution)
                points.append(f"{sx:.3f},{sy:.3f}")
            handle.write(
                '<polyline points="' + " ".join(points) +
                '" fill="none" stroke="#ef4444" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>\n'
            )
            for point in trajectory:
                sx = (point["x"] / resolution) - min_x
                sy = max_y - (point["y"] / resolution)
                handle.write(f'<circle cx="{sx:.3f}" cy="{sy:.3f}" r="2" fill="#dc2626"/>\n')
        handle.write("</svg>\n")

    return {
        "pgm": pgm_path.name,
        "yaml": yaml_path.name,
        "svg": svg_path.name,
        "width_cells": width,
        "height_cells": height,
        "origin": [origin_x, origin_y, 0.0],
        "resolution": resolution,
        "occupied_xy_cells": len(xy_cells),
        "svg_cell_stride": stride,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--bit-order", choices=["lsb", "msb"], default="lsb")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames")
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--min-path-distance", type=float, default=0.50)
    parser.add_argument("--max-svg-cells", type=int, default=100000)
    args = parser.parse_args()

    capture_dir = args.capture_dir.resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else capture_dir / "offline_map"
    output_dir.mkdir(parents=True, exist_ok=True)

    records = load_metadata(capture_dir)
    voxel_records = [rec for rec in records if rec.get("type") == "voxel"]
    voxel_records.sort(key=lambda rec: int(rec.get("index", 0)))
    if args.frame_step > 1:
        voxel_records = voxel_records[:: args.frame_step]
    if args.max_frames > 0:
        voxel_records = voxel_records[: args.max_frames]
    if not voxel_records:
        raise RuntimeError(f"No voxel records found in {capture_dir}")

    resolution = float(voxel_records[0]["resolution"])
    global_voxels: dict[GridKey, int] = {}
    decoded_frames = 0
    occupied_samples = 0
    for record in voxel_records:
        decoded = decode_voxel_payload(capture_dir, record)
        decoded_frames += 1
        for x, y, z in iter_occupied_world_points(record, decoded, args.bit_order):
            key = (quantize(x, resolution), quantize(y, resolution), quantize(z, resolution))
            global_voxels[key] = global_voxels.get(key, 0) + 1
            occupied_samples += 1

    trajectory_source, trajectory = build_trajectory(records, voxel_records, args.min_path_distance)

    write_global_ply(output_dir / "global_voxel_map.ply", global_voxels, resolution)
    write_path_csv(output_dir / "path_points.csv", trajectory)
    write_path_ply(output_dir / "path_3d.ply", trajectory)
    write_relative_waypoints(output_dir, trajectory)
    map2d = write_2d_map(output_dir, global_voxels, resolution, trajectory, args.max_svg_cells)

    summary = {
        "capture_dir": str(capture_dir),
        "output_dir": str(output_dir),
        "bit_order": args.bit_order,
        "decoded_frames": decoded_frames,
        "occupied_samples_before_merge": occupied_samples,
        "global_occupied_voxels": len(global_voxels),
        "resolution": resolution,
        "trajectory_source": trajectory_source,
        "trajectory_points": len(trajectory),
        "path_length_m": sum(
            math.hypot(trajectory[i]["x"] - trajectory[i - 1]["x"], trajectory[i]["y"] - trajectory[i - 1]["y"])
            for i in range(1, len(trajectory))
        ),
        "map2d": map2d,
        "outputs": {
            "global_3d_ply": "global_voxel_map.ply",
            "path_csv": "path_points.csv",
            "path_3d_ply": "path_3d.ply",
            "relative_waypoints_csv": "waypoints_relative.csv",
            "relative_waypoints_text": "waypoints_relative_text.txt",
        },
        "notes": [
            "If trajectory_source is voxel_frame_center and trajectory_points is 1, the capture does not contain enough motion information for automatic path reproduction.",
            "For automatic route replay, use waypoints_relative_text.txt with go2_waypoint_follower only after verifying the path points in the 2D/3D map.",
        ],
    }
    summary_path = output_dir / "offline_map_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"capture_dir: {capture_dir}")
    print(f"output_dir: {output_dir}")
    print(f"decoded_frames: {decoded_frames}")
    print(f"global_occupied_voxels: {len(global_voxels)}")
    print(f"trajectory_source: {trajectory_source}")
    print(f"trajectory_points: {len(trajectory)}")
    print(f"path_length_m: {summary['path_length_m']:.3f}")
    print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
