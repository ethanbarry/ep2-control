#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common_env.sh
source "$SCRIPT_DIR/common_env.sh"

SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"
if [[ ! -f "$SIM_VEHICLE" ]]; then
  echo "sim_vehicle.py not found at $SIM_VEHICLE. Run scripts/setup_linux.sh or scripts/setup_macos.sh first." >&2
  exit 1
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python runtime not found at $PYTHON_BIN. Create the project venv first." >&2
  exit 1
fi

export PATH="$VENV_DIR/bin:$ARDUPILOT_HOME/Tools/autotest:$PATH"
cd "$ARDUPILOT_HOME/ArduCopter"

FRAME="${SITL_FRAME:-gazebo-iris}"
MODEL="${SITL_MODEL:-JSON}"
CONNECTION_OUT="${MAVPROXY_OUT:-udp:127.0.0.1:14550}"
USE_DIR="${SITL_USE_DIR:-$ROOT_DIR/.sitl/live_fox_hunt}"
MAVPROXY_FLAGS="${MAVPROXY_FLAGS:-}"
SITL_SPEEDUP="${SITL_SPEEDUP:-1}"
NO_REBUILD_FLAG="${NO_REBUILD_FLAG:-0}"
USE_MAVPROXY_FLAG="${USE_MAVPROXY_FLAG:-1}"

mkdir -p "$USE_DIR"

RUN_ARGS=(
  -v ArduCopter
  -f "$FRAME"
  --model "$MODEL"
  --use-dir "$USE_DIR"
  --out="$CONNECTION_OUT"
  --speedup "$SITL_SPEEDUP"
)
if [[ "$NO_REBUILD_FLAG" == "1" ]]; then
  RUN_ARGS+=(--no-rebuild)
fi
if [[ "$USE_MAVPROXY_FLAG" != "1" ]]; then
  RUN_ARGS+=(--no-mavproxy)
fi
if [[ -n "$MAVPROXY_FLAGS" ]]; then
  RUN_ARGS+=(--mavproxy-args="$MAVPROXY_FLAGS")
fi

echo "Launching ArduPilot SITL with frame=$FRAME model=$MODEL out=$CONNECTION_OUT"
exec "$PYTHON_BIN" "$SIM_VEHICLE" "${RUN_ARGS[@]}"
