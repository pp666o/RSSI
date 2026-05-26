#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="$project_root/test_rssi.cpp"
out_dir="$project_root/build_rssi"
out="$out_dir/test_rssi"

if [[ ! -f "$src" ]]; then
  echo "Missing source file: $src" >&2
  exit 1
fi

mkdir -p "$out_dir"
g++ -std=c++17 -O2 -pthread "$src" -o "$out"
chmod +x "$out"

echo "Built RSSI logger: $out"
file "$out" || true
