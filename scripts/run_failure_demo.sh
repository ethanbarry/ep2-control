#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO_PATH="${1:-$ROOT_DIR/config/scenario_low_battery.yaml}"
BACKEND="${BACKEND:-mock}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MAVLINK_CONNECTION="${MAVLINK_CONNECTION:-udp:127.0.0.1:14550}"

cd "$ROOT_DIR"
"$PYTHON_BIN" -m src.main --scenario "$SCENARIO_PATH" --backend "$BACKEND" --connection "$MAVLINK_CONNECTION" --plot
