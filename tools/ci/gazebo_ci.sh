#!/usr/bin/env bash
# Runs inside docker/ros.Dockerfile with the repo mounted at the working directory:
# colcon build of the ROS packages, static asset check, smoke test, then E2E-07 when the
# Gazebo scenario exists (feature `ros2-controller`).
set -euo pipefail
repo="$(pwd)"
export COBOT_COLCON_WS=/tmp/colcon
bash "$repo/tools/wsl/sim.sh" build
bash "$repo/tools/wsl/sim.sh" check
bash "$repo/tools/wsl/sim.sh" smoke 90
if ls "$repo"/tests/e2e/scenarios/*.yaml >/dev/null 2>&1 && grep -lq "runner: gazebo" "$repo"/tests/e2e/scenarios/*.yaml; then
  # shellcheck disable=SC1091
  set +u; source /tmp/colcon/install/setup.bash; set -u
  python3 "$repo/tools/run_e2e.py" --all --tag gazebo --out /tmp/e2e-gazebo --no-report
fi
echo "GAZEBO CI OK"
