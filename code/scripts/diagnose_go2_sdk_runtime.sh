#!/usr/bin/env bash
set -u

bin="${1:-unitree_sdk2/build_go2/bin/go2_sport_client}"
iface="${2:-eth0}"

echo "== basic =="
echo "pwd=$(pwd)"
echo "bin=$bin"
echo "iface=$iface"
date
uname -a

echo
echo "== binary =="
if [[ -x "$bin" ]]; then
  file "$bin"
else
  echo "missing or not executable: $bin"
fi

echo
echo "== linked libraries =="
if command -v ldd >/dev/null 2>&1 && [[ -e "$bin" ]]; then
  ldd "$bin" || true
else
  echo "ldd not available"
fi

echo
echo "== rpath/runpath =="
if command -v readelf >/dev/null 2>&1 && [[ -e "$bin" ]]; then
  readelf -d "$bin" | grep -E 'RPATH|RUNPATH|NEEDED' || true
else
  echo "readelf not available"
fi

echo
echo "== sdk local libraries =="
for lib in \
  unitree_sdk2/lib/aarch64/libunitree_sdk2.a \
  unitree_sdk2/thirdparty/lib/aarch64/libddsc.so \
  unitree_sdk2/thirdparty/lib/aarch64/libddscxx.so; do
  if [[ -e "$lib" ]]; then
    ls -l "$lib"
    sha256sum "$lib" 2>/dev/null || true
  else
    echo "missing: $lib"
  fi
done

echo
echo "== possible system dds libraries =="
find /usr /lib /opt -name 'libddsc*.so*' -o -name 'libunitree_sdk2*' 2>/dev/null | sort | sed -n '1,80p'

echo
echo "== network interface =="
ip link show "$iface" || true
ip addr show "$iface" || true
ip route || true

echo
echo "== environment =="
env | sort | grep -E 'CYCLONE|DDS|RMW|LD_LIBRARY_PATH|AMENT|ROS|UNITREE' || true

echo
echo "== dry dynamic loader trace =="
if [[ -x "$bin" ]]; then
  LD_DEBUG=libs "$bin" "$iface" >/tmp/go2_sdk_diag_stdout.txt 2>/tmp/go2_sdk_diag_lddebug.txt
  status=$?
  echo "program_status=$status"
  echo "-- stdout tail --"
  tail -n 40 /tmp/go2_sdk_diag_stdout.txt
  echo "-- lddebug tail --"
  tail -n 80 /tmp/go2_sdk_diag_lddebug.txt
else
  echo "skip run: binary missing"
fi
