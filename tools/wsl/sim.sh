#!/usr/bin/env bash
# ROS 2 / Gazebo side of the repo (colcon packages under sim/ and ros2_node/).
# Usage: sim.sh build | check | smoke [seconds]
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ws="${COBOT_COLCON_WS:-$HOME/cobot-build/colcon}"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

paths=("$repo/sim")
if [ -f "$repo/ros2_node/package.xml" ]; then paths+=("$repo/ros2_node"); fi

cmd="${1:-build}"
case "$cmd" in
  build)
    colcon build --paths "${paths[@]}" --build-base "$ws/build" --install-base "$ws/install" \
      --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "COLCON OK: $ws/install"
    ;;
  check)
    bash "$repo/tools/ci/check_sim_assets.sh" "$ws/install/cobot_sim"
    ;;
  smoke)
    secs="${2:-25}"
    set +u
    # shellcheck disable=SC1091
    source "$ws/install/setup.bash"
    set -u
    log="$(mktemp)"
    ros2 launch cobot_sim sim.launch.py gui:=false > "$log" 2>&1 &
    pid=$!
    cleanup() { kill -INT "$pid" 2>/dev/null || true; sleep 3; pkill -f "gz sim" 2>/dev/null || true; }
    trap cleanup EXIT
    ok=0
    for _ in $(seq 1 "$secs"); do
      if ros2 control list_controllers 2>/dev/null | grep -q "effort_passthrough.*active"; then ok=1; break; fi
      sleep 1
    done
    if [ "$ok" != 1 ]; then echo "controllers not active"; tail -40 "$log"; exit 1; fi
    ros2 control list_controllers
    # Zero effort from the start: the arm must have fallen away from the home pose.
    names="$(timeout 10 ros2 topic echo --once --field name /joint_states | grep -o "joint_[0-9]" | sort -u | wc -l)"
    q="$(timeout 10 ros2 topic echo --once --field position /joint_states | grep -m1 array)"
    echo "joints: $names"
    echo "q: $q"
    python3 - "$q" "$names" <<'EOF'
import re, sys
q = [float(x) for x in re.findall(r"-?\d+\.\d+(?:e-?\d+)?", sys.argv[1])]
home = [0.0, 0.5235987755982988, 0.0, -1.5707963267948966, 0.0, 1.0471975511965976, 0.0]
assert int(sys.argv[2]) == 7, f"expected 7 joints, got {sys.argv[2]}"
assert len(q) == 7, f"expected 7 positions, got {len(q)}"
moved = max(abs(a - b) for a, b in zip(q, home))
assert moved > 0.05, f"arm did not fall under gravity with zero effort (max {moved:.4f} rad)"
print(f"max deviation from home under gravity: {moved:.3f} rad")
EOF
    echo "SMOKE OK"
    ;;
  *)
    echo "usage: $0 build | check | smoke [seconds]" >&2
    exit 2
    ;;
esac
