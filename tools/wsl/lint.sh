#!/usr/bin/env bash
# Same checks as the CI `lint` job: clang-format, ruff, and clang-tidy on changed C++ files.
# Usage: lint.sh [--fix] [--tidy-base REF]   (clang-tidy needs a build dir from build.sh)
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=git_env.sh
source "$repo/tools/wsl/git_env.sh" "$repo"
venv="${COBOT_VENV:-$HOME/.venvs/cobot}"
fix=0
base="origin/main"
while [ $# -gt 0 ]; do
  case "$1" in
    --fix) fix=1 ;;
    --tidy-base) base="$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
cd "$repo"

mapfile -t cpp < <(git ls-files '*.cpp' '*.hpp')
if [ "${#cpp[@]}" -gt 0 ]; then
  if [ "$fix" = 1 ]; then clang-format-18 -i "${cpp[@]}"; else clang-format-18 --dry-run --Werror "${cpp[@]}"; fi
fi

if [ "$fix" = 1 ]; then
  "$venv/bin/ruff" check --fix .
  "$venv/bin/ruff" format .
else
  "$venv/bin/ruff" check .
  "$venv/bin/ruff" format --check .
fi

branch="$(git rev-parse --abbrev-ref HEAD)"
build_dir="$HOME/cobot-build/${branch//\//-}"
# ros2_node needs the ROS headers: it is built (with warnings) by sim.sh and the gazebo job.
mapfile -t changed < <(git diff --name-only --diff-filter=AM "$base"...HEAD -- '*.cpp' ':!ros2_node/**' 2>/dev/null || true)
if [ "${#changed[@]}" -gt 0 ] && [ -f "$build_dir/compile_commands.json" ]; then
  clang-tidy-18 -p "$build_dir" --quiet --warnings-as-errors='*' "${changed[@]}"
elif [ "${#changed[@]}" -gt 0 ]; then
  echo "NOTE: run tools/wsl/build.sh first to enable clang-tidy" >&2
fi
echo "LINT OK"
