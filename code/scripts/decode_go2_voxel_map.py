#!/usr/bin/env python3
"""Decode and inspect Unitree Go2 VoxelMapCompressed captures.

The script intentionally tries several compression paths because Unitree's
public SDK exposes the message fields but not the payload codec.
"""

from __future__ import annotations

import argparse
import bz2
import gzip
import json
import lzma
import math
import shutil
import struct
import subprocess
import tempfile
import zlib
from pathlib import Path
from typing import Iterable


class DecodeError(Exception):
    pass


def load_metadata(capture_dir: Path) -> list[dict]:
    metadata_path = capture_dir / "metadata.jsonl"
    records: list[dict] = []
    with metadata_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def decompress_lz4_block(payload: bytes, expected_size: int | None = None) -> bytes:
    """Decode a raw LZ4 block without the .lz4 frame header.

    Unitree's voxel payload starts like a valid LZ4 block token stream rather
    than a framed file, so command-line lz4 cannot decode it directly.
    """
    ip = 0
    out = bytearray()
    payload_len = len(payload)

    while ip < payload_len:
        token = payload[ip]
        ip += 1

        literal_len = token >> 4
        if literal_len == 15:
            while True:
                if ip >= payload_len:
                    raise DecodeError("truncated literal length")
                value = payload[ip]
                ip += 1
                literal_len += value
                if value != 255:
                    break

        if ip + literal_len > payload_len:
            raise DecodeError("literal section exceeds payload")
        out.extend(payload[ip : ip + literal_len])
        ip += literal_len

        if ip >= payload_len:
            break

        if ip + 2 > payload_len:
            raise DecodeError("missing match offset")
        offset = payload[ip] | (payload[ip + 1] << 8)
        ip += 2
        if offset == 0 or offset > len(out):
            raise DecodeError(f"invalid match offset {offset}")

        match_len = token & 0x0F
        if match_len == 15:
            while True:
                if ip >= payload_len:
                    raise DecodeError("truncated match length")
                value = payload[ip]
                ip += 1
                match_len += value
                if value != 255:
                    break
        match_len += 4

        for _ in range(match_len):
            out.append(out[-offset])

        if expected_size is not None and len(out) > expected_size:
            raise DecodeError("decoded output exceeds expected size")

    if expected_size is not None and len(out) != expected_size:
        raise DecodeError(f"decoded output {len(out)} != expected {expected_size}")
    return bytes(out)


def try_internal_codecs(data: bytes) -> list[tuple[str, bytes]]:
    outputs: list[tuple[str, bytes]] = []
    candidates = [
        ("raw", lambda payload: payload),
        ("zlib", zlib.decompress),
        ("zlib_raw", lambda payload: zlib.decompress(payload, -zlib.MAX_WBITS)),
        ("gzip", gzip.decompress),
        ("bz2", bz2.decompress),
        ("lzma", lzma.decompress),
    ]
    for name, fn in candidates:
        try:
            outputs.append((name, fn(data)))
        except Exception:
            pass
    return outputs


def try_lz4_block_codecs(data: bytes, expected_size: int) -> list[tuple[str, bytes]]:
    outputs: list[tuple[str, bytes]] = []
    for skip in range(0, min(32, len(data)) + 1):
        suffix = data[skip:]
        try:
            decoded = decompress_lz4_block(suffix, expected_size)
        except Exception:
            continue
        name = "lz4_block" if skip == 0 else f"lz4_block_skip_{skip}"
        outputs.append((name, decoded))
    return outputs


def try_external_codec(data: bytes, command: str, args: list[str]) -> bytes | None:
    exe = shutil.which(command)
    if not exe:
        return None
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "payload.bin"
        dst = Path(tmp) / "payload.out"
        src.write_bytes(data)
        result = subprocess.run(
            [exe, *args, str(src), "-c"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode == 0 and result.stdout:
            return result.stdout

        result = subprocess.run(
            [exe, *args, str(src), str(dst)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode == 0 and dst.exists():
            return dst.read_bytes()
    return None


def try_external_codecs(data: bytes) -> list[tuple[str, bytes]]:
    outputs: list[tuple[str, bytes]] = []
    for name, command, args in [
        ("lz4", "lz4", ["-d", "-f"]),
        ("zstd", "zstd", ["-d", "-f"]),
    ]:
        decoded = try_external_codec(data, command, args)
        if decoded is not None:
            outputs.append((name, decoded))
    return outputs


def try_offset_codecs(data: bytes, max_skip: int = 32) -> list[tuple[str, bytes]]:
    outputs: list[tuple[str, bytes]] = []
    for skip in range(1, min(max_skip, len(data)) + 1):
        suffix = data[skip:]
        for name, decoded in try_internal_codecs(suffix) + try_external_codecs(suffix):
            if name == "raw":
                continue
            outputs.append((f"{name}_skip_{skip}", decoded))
    return outputs


def unpack_bits(payload: bytes, width: list[int], bit_order: str) -> list[tuple[int, int, int]]:
    wx, wy, wz = width
    total = wx * wy * wz
    occupied: list[tuple[int, int, int]] = []
    for linear in range(total):
        byte = payload[linear // 8]
        bit_index = linear % 8
        if bit_order == "msb":
            bit = (byte >> (7 - bit_index)) & 1
        else:
            bit = (byte >> bit_index) & 1
        if not bit:
            continue
        x = linear % wx
        y = (linear // wx) % wy
        z = linear // (wx * wy)
        occupied.append((x, y, z))
    return occupied


def write_xyz(path: Path, occupied: Iterable[tuple[int, int, int]], origin: list[float], resolution: float) -> int:
    count = 0
    with path.open("w", encoding="utf-8") as handle:
        handle.write("x,y,z,ix,iy,iz\n")
        for ix, iy, iz in occupied:
            x = origin[0] + (ix + 0.5) * resolution
            y = origin[1] + (iy + 0.5) * resolution
            z = origin[2] + (iz + 0.5) * resolution
            handle.write(f"{x:.6f},{y:.6f},{z:.6f},{ix},{iy},{iz}\n")
            count += 1
    return count


def write_ply(path: Path, occupied: list[tuple[int, int, int]], origin: list[float], resolution: float) -> None:
    max_z = max((p[2] for p in occupied), default=1)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(occupied)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("end_header\n")
        for ix, iy, iz in occupied:
            x = origin[0] + (ix + 0.5) * resolution
            y = origin[1] + (iy + 0.5) * resolution
            z = origin[2] + (iz + 0.5) * resolution
            hue = iz / max(1, max_z)
            red = int(255 * min(1.0, max(0.0, hue * 2.0)))
            green = int(255 * min(1.0, max(0.0, 1.0 - abs(hue - 0.5) * 2.0)))
            blue = int(255 * min(1.0, max(0.0, 1.0 - hue * 2.0)))
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {red} {green} {blue}\n")


def read_float32(payload: bytes, offset: int, bigendian: bool) -> float:
    fmt = ">f" if bigendian else "<f"
    return struct.unpack_from(fmt, payload, offset)[0]


def export_cloud_xyz_ply(capture_dir: Path, output_dir: Path, cloud_record: dict, tag: str) -> dict:
    fields = {field["name"]: field for field in cloud_record.get("fields", [])}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        return {"error": f"missing fields: {missing}"}

    cloud_path = capture_dir / cloud_record["file"]
    payload = cloud_path.read_bytes()
    bigendian = bool(cloud_record.get("is_bigendian", False))
    point_step = int(cloud_record["point_step"])
    row_step = int(cloud_record["row_step"])
    width = int(cloud_record["width"])
    height = int(cloud_record["height"])
    x_off = int(fields["x"]["offset"])
    y_off = int(fields["y"]["offset"])
    z_off = int(fields["z"]["offset"])

    points: list[tuple[float, float, float]] = []
    for row in range(height):
        row_base = row * row_step
        for col in range(width):
            base = row_base + col * point_step
            if base + max(x_off, y_off, z_off) + 4 > len(payload):
                continue
            x = read_float32(payload, base + x_off, bigendian)
            y = read_float32(payload, base + y_off, bigendian)
            z = read_float32(payload, base + z_off, bigendian)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))

    csv_path = output_dir / f"{tag}.cloud.xyz.csv"
    with csv_path.open("w", encoding="utf-8") as handle:
        handle.write("x,y,z\n")
        for x, y, z in points:
            handle.write(f"{x:.6f},{y:.6f},{z:.6f}\n")

    ply_path = output_dir / f"{tag}.cloud.ply"
    with ply_path.open("w", encoding="utf-8") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("end_header\n")
        for x, y, z in points:
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} 180 180 180\n")

    return {
        "cloud_point_count": len(points),
        "cloud_xyz": str(csv_path.relative_to(output_dir)),
        "cloud_ply": str(ply_path.relative_to(output_dir)),
    }


def write_projection_png(path: Path, occupied: list[tuple[int, int, int]], width: list[int]) -> bool:
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return False

    wx, wy, wz = width
    scale = max(1, min(5, 800 // max(wx, wy, wz)))
    panels = [
        ("xy", wx, wy),
        ("xz", wx, wz),
        ("yz", wy, wz),
    ]
    margin = 20
    label_h = 20
    panel_w = max(w for _, w, _ in panels) * scale
    panel_h = max(h for _, _, h in panels) * scale
    image = Image.new("RGB", (len(panels) * (panel_w + margin) + margin, panel_h + label_h + 2 * margin), "white")
    draw = ImageDraw.Draw(image)

    occupied_set = set(occupied)
    for panel_idx, (name, pw, ph) in enumerate(panels):
        ox = margin + panel_idx * (panel_w + margin)
        oy = margin + label_h
        draw.text((ox, margin // 2), name, fill=(0, 0, 0))
        draw.rectangle([ox, oy, ox + pw * scale, oy + ph * scale], outline=(120, 120, 120))

        if name == "xy":
            points = {(ix, iy, iz) for ix, iy, iz in occupied_set}
            for ix, iy, iz in points:
                color = (int(255 * iz / max(1, wz - 1)), 80, 255 - int(255 * iz / max(1, wz - 1)))
                draw.rectangle(
                    [ox + ix * scale, oy + (ph - 1 - iy) * scale,
                     ox + (ix + 1) * scale - 1, oy + (ph - iy) * scale - 1],
                    fill=color,
                )
        elif name == "xz":
            for ix, _iy, iz in occupied_set:
                color = (int(255 * iz / max(1, wz - 1)), 80, 255 - int(255 * iz / max(1, wz - 1)))
                draw.rectangle(
                    [ox + ix * scale, oy + (ph - 1 - iz) * scale,
                     ox + (ix + 1) * scale - 1, oy + (ph - iz) * scale - 1],
                    fill=color,
                )
        else:
            for _ix, iy, iz in occupied_set:
                color = (int(255 * iz / max(1, wz - 1)), 80, 255 - int(255 * iz / max(1, wz - 1)))
                draw.rectangle(
                    [ox + iy * scale, oy + (ph - 1 - iz) * scale,
                     ox + (iy + 1) * scale - 1, oy + (ph - iz) * scale - 1],
                    fill=color,
                )

    image.save(path)
    return True


def nearest_cloud(voxel_record: dict, cloud_records: list[dict]) -> dict | None:
    if not cloud_records:
        return None
    voxel_t = voxel_record.get("recv_steady_sec", voxel_record.get("stamp", 0.0))
    return min(
        cloud_records,
        key=lambda rec: abs(rec.get("recv_steady_sec", rec.get("stamp", 0.0)) - voxel_t),
    )


def decode_one(capture_dir: Path, output_dir: Path, voxel_record: dict, cloud_records: list[dict]) -> dict:
    raw_path = capture_dir / voxel_record["file"]
    raw = raw_path.read_bytes()
    src_size = int(voxel_record["src_size"])
    width = [int(x) for x in voxel_record["width"]]
    origin = [float(x) for x in voxel_record["origin"]]
    resolution = float(voxel_record["resolution"])

    attempts = (
        try_internal_codecs(raw)
        + try_external_codecs(raw)
        + try_offset_codecs(raw)
        + try_lz4_block_codecs(raw, src_size)
    )
    unique: dict[tuple[str, int], bytes] = {}
    for name, decoded in attempts:
        unique[(name, len(decoded))] = decoded

    success: list[dict] = []
    for (name, decoded_len), decoded in unique.items():
        ok = decoded_len == src_size
        item = {"codec": name, "decoded_size": decoded_len, "matches_src_size": ok}
        if ok:
            base = f"voxel_{voxel_record['index']:06d}_{name}"
            decoded_path = output_dir / f"{base}.decoded.bin"
            decoded_path.write_bytes(decoded)
            item["decoded_file"] = str(decoded_path.relative_to(output_dir))

            for bit_order in ("lsb", "msb"):
                occupied = unpack_bits(decoded, width, bit_order)
                xyz_path = output_dir / f"{base}.{bit_order}.xyz.csv"
                ply_path = output_dir / f"{base}.{bit_order}.ply"
                png_path = output_dir / f"{base}.{bit_order}.projection.png"
                count = write_xyz(xyz_path, occupied, origin, resolution)
                if occupied:
                    write_ply(ply_path, occupied, origin, resolution)
                png_written = write_projection_png(png_path, occupied, width)
                item[f"{bit_order}_occupied_count"] = count
                item[f"{bit_order}_xyz"] = str(xyz_path.relative_to(output_dir))
                item[f"{bit_order}_ply"] = str(ply_path.relative_to(output_dir))
                if png_written:
                    item[f"{bit_order}_projection_png"] = str(png_path.relative_to(output_dir))
        success.append(item)

    cloud = nearest_cloud(voxel_record, cloud_records)
    tag = f"voxel_{voxel_record['index']:06d}"
    cloud_export = export_cloud_xyz_ply(capture_dir, output_dir, cloud, tag) if cloud else None
    summary = {
        "voxel_index": voxel_record["index"],
        "voxel_file": voxel_record["file"],
        "frame_id": voxel_record.get("frame_id"),
        "resolution": resolution,
        "origin": origin,
        "width": width,
        "src_size": src_size,
        "compressed_size": len(raw),
        "expected_bitpacked_size": math.ceil(width[0] * width[1] * width[2] / 8),
        "nearest_cloud": cloud,
        "nearest_cloud_export": cloud_export,
        "decode_attempts": success,
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--max-frames", type=int, default=5)
    parser.add_argument("--voxel-index", type=int)
    args = parser.parse_args()

    capture_dir = args.capture_dir.resolve()
    output_dir = args.output_dir.resolve() if args.output_dir else capture_dir / "decoded"
    output_dir.mkdir(parents=True, exist_ok=True)

    records = load_metadata(capture_dir)
    voxel_records = [rec for rec in records if rec.get("type") == "voxel"]
    cloud_records = [rec for rec in records if rec.get("type") == "cloud"]
    if args.voxel_index is not None:
        voxel_records = [rec for rec in voxel_records if int(rec["index"]) == args.voxel_index]
    else:
        voxel_records = voxel_records[: args.max_frames]

    summaries = [decode_one(capture_dir, output_dir, rec, cloud_records) for rec in voxel_records]
    summary_path = output_dir / "decode_summary.json"
    summary_path.write_text(json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"capture_dir: {capture_dir}")
    print(f"output_dir: {output_dir}")
    print(f"voxel_frames: {len(voxel_records)} cloud_frames: {len(cloud_records)}")
    for summary in summaries:
        matches = [x for x in summary["decode_attempts"] if x["matches_src_size"]]
        print(
            f"voxel {summary['voxel_index']}: compressed={summary['compressed_size']} "
            f"src_size={summary['src_size']} matches={len(matches)}"
        )
        for match in matches:
            print(
                f"  {match['codec']}: "
                f"lsb_occ={match.get('lsb_occupied_count')} "
                f"msb_occ={match.get('msb_occupied_count')}"
            )
    print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
