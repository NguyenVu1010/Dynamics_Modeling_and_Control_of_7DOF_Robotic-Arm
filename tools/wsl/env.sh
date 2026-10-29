#!/usr/bin/env bash
# ROS 2 / Gazebo environment for this repo. Source it, do not run it:
#   source tools/wsl/env.sh
# Sources ROS 2 Jazzy and the colcon workspace (when built) and isolates this project from other
# ROS 2 projects in the same WSL, which use the default domain 0 and Gazebo partition: two
# simulations running at once would otherwise share /clock, /joint_states and gz topics.
#   COBOT_ROS_DOMAIN_ID (default 42) -> ROS_DOMAIN_ID, overriding the value from ~/.bashrc
#   COBOT_GZ_PARTITION  (default cobot) -> GZ_PARTITION
#   COBOT_COLCON_WS     (default $COBOT_ROOT/colcon/<branch>, tools/wsl/paths.sh) -> the
#                       workspace sourced here

export ROS_DOMAIN_ID="${COBOT_ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${COBOT_GZ_PARTITION:-cobot}"
# shellcheck source=paths.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/paths.sh"   "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cobot_ws="$(cobot_colcon_ws)"

# cobot_ws_guard WS REPO: wipe WS/build and WS/install when a package there was configured from
# a source directory outside REPO (a deleted worktree, another clone). CMake would otherwise stop
# with "The source directory ... does not exist", or build stale sources.
cobot_ws_guard() {
  local ws="$1" repo="${2%/}" cache src
  for cache in "$ws"/build/*/CMakeCache.txt; do
    [ -f "$cache" ] || continue
    src="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")"
    case "$src" in
      "$repo"/*) ;;
      *)
        echo "colcon workspace $ws was built from $src, not $repo: removing build/ and install/"
        rm -rf "$ws/build" "$ws/install"
        return 0
        ;;
    esac
  done
}

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$cobot_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$cobot_ws/install/setup.bash"
fi
