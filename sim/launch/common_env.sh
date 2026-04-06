#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEPS_DIR="${DEPS_DIR:-$ROOT_DIR/.deps}"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$DEPS_DIR/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$DEPS_DIR/ardupilot_gazebo}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv}"
GZ_VERSION="${GZ_VERSION:-harmonic}"
PYTHON_BIN="${PYTHON_BIN:-$VENV_DIR/bin/python}"
QT5_PREFIX="${QT5_PREFIX:-/opt/homebrew/opt/qt@5}"

export ROOT_DIR
export DEPS_DIR
export ARDUPILOT_HOME
export ARDUPILOT_GAZEBO_HOME
export VENV_DIR
export GZ_VERSION
export PYTHON_BIN
export QT5_PREFIX
