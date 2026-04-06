#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO_PATH="${1:-$ROOT_DIR/config/scenario_default.yaml}"
BACKEND="${BACKEND:-mavlink}"
MAVLINK_CONNECTION="${MAVLINK_CONNECTION:-udp:127.0.0.1:14550}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
WORLD_TEMPLATE_PATH="${WORLD_PATH:-$ROOT_DIR/sim/world/fox_hunt_iris_runway.sdf}"
GENERATED_WORLD_PATH="${GENERATED_WORLD_PATH:-$ROOT_DIR/.sitl/generated/demo_fox_hunt_world.sdf}"

cd "$ROOT_DIR"

cleanup() {
  if [[ -n "${SITL_PID:-}" ]]; then
    kill "$SITL_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "${GAZEBO_PID:-}" ]]; then
    kill "$GAZEBO_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "$BACKEND" == "mavlink" ]]; then
  "$PYTHON_BIN" -m src.scenario \
    --scenario "$SCENARIO_PATH" \
    --world-input "$WORLD_TEMPLATE_PATH" \
    --world-output "$GENERATED_WORLD_PATH"

  "$ROOT_DIR/sim/launch/launch_gazebo.sh" "$GENERATED_WORLD_PATH" &
  GAZEBO_PID=$!
  sleep 5

  MAVPROXY_OUT="$MAVLINK_CONNECTION" "$ROOT_DIR/sim/launch/launch_sitl.sh" &
  SITL_PID=$!
  sleep 12
fi

"$PYTHON_BIN" -m src.main --scenario "$SCENARIO_PATH" --backend "$BACKEND" --connection "$MAVLINK_CONNECTION" --plot
