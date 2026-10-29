#!/usr/bin/env bash
# One-time setup of WSL Ubuntu 24.04 for this repo. It never runs sudo itself: it prints the
# apt line for you to run, then creates the Python virtualenv.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=paths.sh
source "$repo/tools/wsl/paths.sh" "$repo"
venv="$(cobot_venv)"

pkgs=(cmake ninja-build ccache g++ libeigen3-dev libgtest-dev libgmock-dev libtinyxml2-dev
  libyaml-cpp-dev clang-format-18 clang-tidy-18 gcovr python3-venv)
missing=()
for p in "${pkgs[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
if [ "${#missing[@]}" -gt 0 ]; then
  echo "Missing apt packages. Run:"
  echo "  sudo apt install -y ${missing[*]}"
fi
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found (needed only for sim/ and ros2_node/): see docs.ros.org/en/jazzy"
fi

if [ ! -x "$venv/bin/python" ]; then
  # Without python3-venv (no ensurepip) fall back to a venv that borrows the system pip.
  python3 -m venv "$venv" 2>/dev/null || {
    rm -rf "$venv"
    python3 -m venv --without-pip --system-site-packages "$venv"
  }
fi
"$venv/bin/python" -m pip install --quiet -e "$repo[dev,eval]"
echo "SETUP OK (venv: $venv)"
