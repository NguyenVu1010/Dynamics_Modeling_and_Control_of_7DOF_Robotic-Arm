#!/usr/bin/env bash
# ROS 2 / Gazebo side of the repo (colcon packages under sim/, ros2_interfaces/, ros2_node/ and
# ros2_gui/).
# Usage: sim.sh build | check | tidy | test | smoke [seconds]
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# ROS 2, the workspace when built, and this project's ROS domain / Gazebo partition.
# shellcheck source=env.sh
source "$repo/tools/wsl/env.sh"
ws="$cobot_ws"
set -u

paths=("$repo/sim" "$repo/ros2_interfaces")
if [ -f "$repo/ros2_node/package.xml" ]; then paths+=("$repo/ros2_node"); fi
if [ -f "$repo/ros2_gui/package.xml" ]; then paths+=("$repo/ros2_gui"); fi

cmd="${1:-build}"
case "$cmd" in
  build)
    cobot_ws_guard "$ws" "$repo"
    colcon --log-base "$ws/log" build --paths "${paths[@]}" --build-base "$ws/build" \
      --install-base "$ws/install" --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli
    echo "COLCON OK: $ws/install"
    ;;
  check)
    bash "$repo/tools/ci/check_sim_assets.sh" "$ws/install/cobot_sim"
    ;;
  tidy)
    # clang-tidy on the ROS controller sources (the lint job cannot: no ROS headers there).
    db="$ws/build/cobot_ros2"
    if [ ! -f "$db/compile_commands.json" ]; then echo "run sim.sh build first" >&2; exit 2; fi
    clang-tidy-18 -p "$db" --quiet --warnings-as-errors='*' "$repo"/ros2_node/src/*.cpp
    gui="$ws/build/cobot_gui"
    if [ -f "$gui/compile_commands.json" ]; then
      # OGRE (via rviz_common) forward-declares its own Pose next to cobot::common::Pose.
      clang-tidy-18 -p "$gui" --quiet --warnings-as-errors='*' \
        --checks=-bugprone-forward-declaration-namespace "$repo"/ros2_gui/src/*.cpp
    fi
    echo "TIDY OK"
    ;;
  test)
    # Unit tests of the ROS packages (cobot_gui data model); fails when a test fails.
    set +u
    # shellcheck disable=SC1091
    source "$ws/install/setup.bash"
    set -u
    colcon --log-base "$ws/log" test --paths "${paths[@]}" --packages-select cobot_gui \
      --build-base "$ws/build" --install-base "$ws/install"
    colcon test-result --test-result-base "$ws/build/cobot_gui" --verbose
    echo "TEST OK"
    ;;
  smoke)
    secs="${2:-25}"
    set +u
    # shellcheck disable=SC1091
    source "$ws/install/setup.bash"
    set -u
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION"
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
    echo "usage: $0 build | check | tidy | test | smoke [seconds]" >&2
    exit 2
    ;;
esac
