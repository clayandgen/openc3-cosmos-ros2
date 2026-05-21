#!/usr/bin/env bash
# Discover ROS2 graph and emit COSMOS cmd.txt / tlm.txt definitions.
#
# Usage:
#   ./discover_ros2.sh [--target NAME] [--out-dir PATH] [--manifest PATH]
#
# Prereqs:
#   - ROS2 environment sourced (`source /opt/ros/<distro>/setup.bash`)
#   - python3 on PATH
#   - Your ROS graph (nodes / topics) running so `ros2 topic list` sees them.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${SCRIPT_DIR}/ros2_to_cosmos.py"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: ros2 CLI not on PATH. Source your ROS2 setup.bash first." >&2
  exit 2
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 not on PATH." >&2
  exit 2
fi

# Quick sanity dump so user sees what will be discovered.
echo "=== ros2 topic list -t ===" >&2
ros2 topic list -t >&2 || true
echo "=== ros2 service list -t ===" >&2
ros2 service list -t >&2 || true
echo "=== ros2 action list -t ===" >&2
ros2 action list -t >&2 || true
echo >&2

exec python3 "${PY}" "$@"
