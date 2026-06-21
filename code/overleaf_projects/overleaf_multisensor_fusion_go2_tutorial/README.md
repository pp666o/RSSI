# Multi-Sensor Fusion Go2 Tutorial Overleaf Project

This folder is a complete Overleaf project for a Chinese tutorial on multi-sensor fusion algorithms, written specifically for the Go2 RSSI path-progress localization project.

Project state-space convention: the fusion state is `s` now, with only `e` as the planned extension. Velocity, yaw, IMU motion score, odom deltas, and command velocity are inputs/constraints/features, not state variables.

Recommended Overleaf settings:

- Main file: `main.tex`
- Compiler: XeLaTeX

The tutorial is aligned with:

- `../robot_s_baseline.py`
- `../rssi_fingerprint_baseline.py`
- `../REAL_S_BASELINE.md`
- `../PROJECT_PLAN_AB.md`
- `../RSSI_BASELINE.md`
- `../GO2_PHASE_A_STRAIGHT_LINE.md`
- `../GO2_NAV2_INTEGRATION.md`
- `../scripts/go2_collect_rssi_imu.sh`
- `../go2_sdk/go2_imu_logger.cpp`
