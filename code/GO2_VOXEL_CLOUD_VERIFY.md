# Go2 VoxelMapCompressed Offline Verification

This workflow follows the official `go2_voxel_cloud_logger.cpp` path:

1. Capture raw `VoxelMapCompressed` frames.
2. Capture synchronized official `rt/utlidar/cloud` `PointCloud2` frames.
3. Offline decode `VoxelMapCompressed.data`.
4. Validate `decoded_size == src_size`.
5. Expand bit-packed data into `128 x 128 x 38` voxel indices.
6. Compare exported voxel `.ply` with the synchronized official point cloud `.ply`.

## Capture

```bash
cd /home/luping/桌面/RSSI/RSSI/code

IFACE=enp5s0 \
DURATION_SEC=10 \
scripts/go2_capture_voxel_cloud_raw.sh /home/luping/go2_voxel_cloud_captures/test_001
```

The capture directory contains:

- `metadata.jsonl`
- `voxel_raw/*.bin`
- `cloud_raw/*.bin`

The metadata stores:

- `src_size`
- `resolution`
- `origin`
- `width`
- `data_size`
- synchronized cloud frame metadata

## Decode And Validate

```bash
cd /home/luping/桌面/RSSI/RSSI/code

MAX_FRAMES=5 \
scripts/go2_decode_voxel_capture.sh /home/luping/go2_voxel_cloud_captures/test_001
```

Expected voxel dimensions from current Go2 captures:

```text
width = [128, 128, 38]
bits = 128 * 128 * 38 = 622592
src_size = ceil(bits / 8) = 77824 bytes
```

Successful decode means:

```text
decoded_size == src_size == 77824
```

The script exports both bit orders because this must be verified visually:

- `*.lsb.ply`
- `*.msb.ply`
- `*.lsb.xyz.csv`
- `*.msb.xyz.csv`
- `*.projection.png`
- synchronized `*.cloud.ply`

Open the `.ply` files with MeshLab, CloudCompare, or any point-cloud viewer.

## Why This Is Different From RViz

This path does not rely on RViz online display. It records exactly what the
official Unitree logger receives:

```text
rt/utlidar/voxel_map_compressed
rt/utlidar/cloud
```

The offline decoder then verifies whether the compressed voxel payload expands
to the expected bit-packed occupancy volume. This is the right workflow for
checking alignment with the official App view and the official point cloud.
