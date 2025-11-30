#!/usr/bin/env bash
# Demo script: repeatedly randomize the grid and replan,
# causing RViz to update/flicker.

INTERVAL="${1:-1.5}"   # seconds between updates (default 1.5s)

echo "[flicker] Calling /randomize_grid every ${INTERVAL}s (Ctrl+C to stop)..."

while true; do
  ros2 service call /randomize_grid std_srvs/srv/Trigger "{}" >/dev/null 2>&1
  sleep "${INTERVAL}"
done

