#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common_env.sh
source "$SCRIPT_DIR/common_env.sh"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python runtime not found at $PYTHON_BIN. Create the project venv first." >&2
  exit 1
fi

MASTER_ENDPOINT="${MASTER_ENDPOINT:-tcp:127.0.0.1:5760}"
SITL_ENDPOINT="${SITL_ENDPOINT:-127.0.0.1:5501}"
CONNECTION_OUT="${1:-${MAVPROXY_OUT:-127.0.0.1:14550}}"
STREAMRATE="${MAVPROXY_STREAMRATE:-10}"
STATE_BASEDIR="${MAVPROXY_STATE_BASEDIR:-$ROOT_DIR/.sitl/mavproxy}"

mkdir -p "$STATE_BASEDIR"
export PATH="$VENV_DIR/bin:$PATH"

echo "Launching MAVProxy with master=$MASTER_ENDPOINT out=$CONNECTION_OUT"
exec mavproxy.py \
  --master "$MASTER_ENDPOINT" \
  --sitl "$SITL_ENDPOINT" \
  --out "$CONNECTION_OUT" \
  --streamrate "$STREAMRATE" \
  --state-basedir "$STATE_BASEDIR" \
  --non-interactive \
  --nowait \
  --retries 10
