#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="${DEPS_DIR:-$ROOT_DIR/.deps}"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$DEPS_DIR/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$DEPS_DIR/ardupilot_gazebo}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

mkdir -p "$DEPS_DIR"

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is required for macOS setup." >&2
  exit 1
fi

brew update
brew install python cmake rapidjson opencv gstreamer gz-harmonic qt@5 pyenv ccache gawk coreutils wget

if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_HOME"
fi
git -C "$ARDUPILOT_HOME" submodule update --init --recursive

if [[ ! -d "$ARDUPILOT_GAZEBO_HOME" ]]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_HOME"
fi

if [[ -x "$ARDUPILOT_HOME/Tools/environment_install/install-prereqs-mac.sh" ]]; then
  DO_AP_STM_ENV=0 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_EXT_ENV=0 SKIP_AP_COMPLETION_ENV=1 \
    "$ARDUPILOT_HOME/Tools/environment_install/install-prereqs-mac.sh" -y
fi

mkdir -p "$ARDUPILOT_GAZEBO_HOME/build"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:${CMAKE_PREFIX_PATH:-}"
export Qt5_DIR="/opt/homebrew/opt/qt@5/lib/cmake/Qt5"
cmake -S "$ARDUPILOT_GAZEBO_HOME" -B "$ARDUPILOT_GAZEBO_HOME/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$ARDUPILOT_GAZEBO_HOME/build" -j4

PYTHON_BOOTSTRAP="python3"
if [[ -x "$HOME/.pyenv/versions/3.10.18/bin/python3" ]]; then
  PYTHON_BOOTSTRAP="$HOME/.pyenv/versions/3.10.18/bin/python3"
fi
"$PYTHON_BOOTSTRAP" -m venv "$VENV_DIR"
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$ROOT_DIR/requirements.txt"

cat <<EOF
macOS setup complete.

Next:
  source "$VENV_DIR/bin/activate"
  export GZ_VERSION="$GZ_VERSION"
  export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:\${CMAKE_PREFIX_PATH:-}"
  export Qt5_DIR="/opt/homebrew/opt/qt@5/lib/cmake/Qt5"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/sim/world/models:$ROOT_DIR/sim/world:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:\${GZ_SIM_RESOURCE_PATH:-}"
  ./scripts/run_live_sitl_gazebo_fox_hunt.sh
EOF
