#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="${DEPS_DIR:-$ROOT_DIR/.deps}"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$DEPS_DIR/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$DEPS_DIR/ardupilot_gazebo}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

mkdir -p "$DEPS_DIR"

sudo apt-get update
sudo apt-get install -y git curl lsb-release gnupg python3 python3-pip python3-venv python3-dev build-essential cmake ninja-build

if ! command -v gz >/dev/null 2>&1; then
  sudo sh -c 'curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg'
  sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
  sudo apt-get update
  sudo apt-get install -y gz-harmonic
fi

sudo apt-get install -y libgz-sim8-dev rapidjson-dev
sudo apt-get install -y libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_HOME"
fi
git -C "$ARDUPILOT_HOME" submodule update --init --recursive

if [[ ! -d "$ARDUPILOT_GAZEBO_HOME" ]]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_HOME"
fi

"$ARDUPILOT_HOME/Tools/environment_install/install-prereqs-ubuntu.sh" -y

mkdir -p "$ARDUPILOT_GAZEBO_HOME/build"
cmake -S "$ARDUPILOT_GAZEBO_HOME" -B "$ARDUPILOT_GAZEBO_HOME/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$ARDUPILOT_GAZEBO_HOME/build" -j4

python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$ROOT_DIR/requirements.txt"

cat <<EOF
Linux setup complete.

Next:
  source "$VENV_DIR/bin/activate"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/sim/world/models:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:\${GZ_SIM_RESOURCE_PATH:-}"
  make run-demo
EOF
