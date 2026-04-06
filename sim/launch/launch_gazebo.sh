#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common_env.sh
source "$SCRIPT_DIR/common_env.sh"

if ! command -v gz >/dev/null 2>&1; then
  echo "gz command not found. Install Gazebo Harmonic first." >&2
  exit 1
fi

if [[ -d "$ARDUPILOT_GAZEBO_HOME/build" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
fi
export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/sim/world/models:$ROOT_DIR/sim/world:${GZ_SIM_RESOURCE_PATH:-}"
if [[ -d "$ARDUPILOT_GAZEBO_HOME/models" ]]; then
  export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:$GZ_SIM_RESOURCE_PATH"
fi

WORLD_PATH="${1:-}"
if [[ -z "$WORLD_PATH" ]]; then
  WORLD_PATH="$ROOT_DIR/sim/world/fox_hunt_iris_runway.sdf"
fi

MODE="${2:-${GAZEBO_MODE:-combined}}"
echo "Launching Gazebo world: $WORLD_PATH (mode=$MODE)"
case "$MODE" in
  server)
    exec gz sim -s -v4 -r "$WORLD_PATH"
    ;;
  client)
    exec gz sim -g -v4
    ;;
  combined)
    exec gz sim -v4 -r "$WORLD_PATH"
    ;;
  *)
    echo "Unknown Gazebo mode: $MODE" >&2
    exit 1
    ;;
esac
