#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO_PATH="${1:-$ROOT_DIR/config/scenario_live_sitl.yaml}"
WORLD_TEMPLATE_PATH="${WORLD_PATH:-$ROOT_DIR/sim/world/fox_hunt_iris_runway.sdf}"
GENERATED_WORLD_PATH="${GENERATED_WORLD_PATH:-$ROOT_DIR/.sitl/generated/live_fox_hunt_world.sdf}"
HEADLESS="${HEADLESS:-0}"
MAVLINK_CONNECTION="${MAVLINK_CONNECTION:-udp:127.0.0.1:14550}"
MISSION_BACKEND="${MISSION_BACKEND:-mavlink}"
SITL_MASTER_ENDPOINT="${SITL_MASTER_ENDPOINT:-tcp:127.0.0.1:5760}"
SITL_CONTROL_PORT="${SITL_CONTROL_PORT:-5760}"
SITL_LINK_ENDPOINT="${SITL_LINK_ENDPOINT:-127.0.0.1:5501}"
SITL_LOG_DIR="${SITL_LOG_DIR:-$ROOT_DIR/.sitl/logs}"

cd "$ROOT_DIR"

# shellcheck source=../sim/launch/common_env.sh
source "$ROOT_DIR/sim/launch/common_env.sh"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Expected project Python at $PYTHON_BIN. Run setup first." >&2
  exit 1
fi
if [[ ! -d "$ARDUPILOT_HOME" || ! -d "$ARDUPILOT_GAZEBO_HOME" ]]; then
  echo "ArduPilot or ardupilot_gazebo is missing under $DEPS_DIR. Run make setup-macos first." >&2
  exit 1
fi
if [[ ! -f "$WORLD_TEMPLATE_PATH" ]]; then
  echo "World file not found: $WORLD_TEMPLATE_PATH" >&2
  exit 1
fi

export PATH="$VENV_DIR/bin:$PATH"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/sim/world/models:$ROOT_DIR/sim/world:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:${GZ_SIM_RESOURCE_PATH:-}"

mkdir -p "$SITL_LOG_DIR"

"$PYTHON_BIN" -m src.scenario \
  --scenario "$SCENARIO_PATH" \
  --world-input "$WORLD_TEMPLATE_PATH" \
  --world-output "$GENERATED_WORLD_PATH"
WORLD_PATH="$GENERATED_WORLD_PATH"

cleanup() {
  local code=$?
  for pid_var in MISSION_PID MAVPROXY_PID SITL_PID GZ_CLIENT_PID GZ_SERVER_PID; do
    local pid="${!pid_var:-}"
    if [[ -n "$pid" ]]; then
      pkill -P "$pid" >/dev/null 2>&1 || true
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
  exit "$code"
}
trap cleanup EXIT INT TERM

echo "Starting Gazebo server..."
"$ROOT_DIR/sim/launch/launch_gazebo.sh" "$WORLD_PATH" server >"$SITL_LOG_DIR/gazebo_server.log" 2>&1 &
GZ_SERVER_PID=$!
sleep 4

if [[ "$HEADLESS" != "1" ]]; then
  echo "Starting Gazebo client..."
  "$ROOT_DIR/sim/launch/launch_gazebo.sh" "$WORLD_PATH" client >"$SITL_LOG_DIR/gazebo_client.log" 2>&1 &
  GZ_CLIENT_PID=$!
  sleep 2
fi

echo "Starting ArduPilot SITL..."
NO_REBUILD_FLAG="${NO_REBUILD_FLAG:-0}" \
SITL_FRAME="${SITL_FRAME:-gazebo-iris}" \
SITL_MODEL="${SITL_MODEL:-JSON}" \
SITL_USE_DIR="${SITL_USE_DIR:-$ROOT_DIR/.sitl/live_fox_hunt}" \
USE_MAVPROXY_FLAG=0 \
MAVPROXY_OUT="$MAVLINK_CONNECTION" \
MAVPROXY_FLAGS="${MAVPROXY_FLAGS:---streamrate=10}" \
"$ROOT_DIR/sim/launch/launch_sitl.sh" >"$SITL_LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!

echo "Waiting for SITL control port on $SITL_MASTER_ENDPOINT ..."
export SITL_CONTROL_PORT
"$PYTHON_BIN" - <<'PY'
import os
import socket
import sys
import time

port = int(os.environ["SITL_CONTROL_PORT"])
deadline = time.time() + 240.0
while time.time() < deadline:
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(1.0)
    try:
        probe.connect(("127.0.0.1", port))
        print("SITL control port is ready")
        sys.exit(0)
    except OSError:
        time.sleep(1.0)
    finally:
        probe.close()
print("Timed out waiting for SITL control port", file=sys.stderr)
sys.exit(1)
PY

echo "Starting MAVProxy bridge..."
MASTER_ENDPOINT="$SITL_MASTER_ENDPOINT" \
SITL_ENDPOINT="$SITL_LINK_ENDPOINT" \
MAVPROXY_OUT="${MAVLINK_CONNECTION#udp:}" \
MAVPROXY_STATE_BASEDIR="${MAVPROXY_STATE_BASEDIR:-$ROOT_DIR/.sitl/mavproxy}" \
"$ROOT_DIR/sim/launch/launch_mavproxy.sh" >"$SITL_LOG_DIR/mavproxy.log" 2>&1 &
MAVPROXY_PID=$!
sleep 3

if ! kill -0 "$MAVPROXY_PID" >/dev/null 2>&1; then
  echo "MAVProxy exited unexpectedly. Check $SITL_LOG_DIR/mavproxy.log" >&2
  exit 1
fi

echo "Process logs are being written to $SITL_LOG_DIR"
echo "Waiting for MAVLink heartbeat on $MAVLINK_CONNECTION ..."
export MAVLINK_CONNECTION
"$PYTHON_BIN" - <<'PY'
import os
import sys
import time
from pymavlink import mavutil

connection = os.environ["MAVLINK_CONNECTION"]
deadline = time.time() + 120.0
master = mavutil.mavlink_connection(connection, autoreconnect=True)
try:
    while time.time() < deadline:
        try:
            master.wait_heartbeat(timeout=2)
            print("MAVLink heartbeat received")
            sys.exit(0)
        except Exception:
            time.sleep(1.0)
    print("Timed out waiting for MAVLink heartbeat", file=sys.stderr)
    sys.exit(1)
finally:
    try:
        master.close()
    except Exception:
        pass
PY

echo "Running fox-hunt mission controller..."
PYTHONUNBUFFERED=1 "$PYTHON_BIN" -m src.main --scenario "$SCENARIO_PATH" --backend "$MISSION_BACKEND" --connection "$MAVLINK_CONNECTION" --plot
