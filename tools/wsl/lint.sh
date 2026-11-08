#!/usr/bin/env bash
# Same checks as the CI `lint` job: clang-format, ruff, and clang-tidy on changed C++ files.
# Usage: lint.sh [--fix] [--tidy-base REF]   (clang-tidy needs a build dir from build.sh)
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=git_env.sh
source "$repo/tools/wsl/git_env.sh" "$repo"
# shellcheck source=paths.sh
source "$repo/tools/wsl/paths.sh" "$repo"
venv="$(cobot_venv)"
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

# New files count before `git add` too (--others --exclude-standard): CI sees them once committed.
mapfile -t cpp < <(git ls-files --cached --others --exclude-standard '*.cpp' '*.hpp')
if [ "${#cpp[@]}" -eq 0 ]; then
  echo "LINT: git lists no C++ file; is this a git checkout?" >&2
  exit 1
fi
echo "clang-format: ${#cpp[@]} files"
if [ "$fix" = 1 ]; then clang-format-18 -i "${cpp[@]}"; else clang-format-18 --dry-run --Werror "${cpp[@]}"; fi

if [ "$fix" = 1 ]; then
  "$venv/bin/ruff" check --fix .
  "$venv/bin/ruff" format .
else
  "$venv/bin/ruff" check .
  "$venv/bin/ruff" format --check .
fi

build_dir="$(cobot_build_dir)"
# ros2_node and ros2_gui need the ROS headers: `sim.sh tidy` checks them (gazebo CI job).
# Committed and uncommitted changes since the branch left $base, plus new untracked files.
fork="$(git merge-base "$base" HEAD 2>/dev/null || echo "$base")"
mapfile -t changed < <({
  git diff --name-only --diff-filter=AM "$fork" -- '*.cpp' ':!src/ros2_node/**' ':!src/ros2_gui/**'
  git ls-files --others --exclude-standard -- '*.cpp' ':!src/ros2_node/**' ':!src/ros2_gui/**'
} 2>/dev/null | sort -u || true)
if [ "${#changed[@]}" -gt 0 ] && [ -f "$build_dir/compile_commands.json" ]; then
  clang-tidy-18 -p "$build_dir" --quiet --warnings-as-errors='*' "${changed[@]}"
elif [ "${#changed[@]}" -gt 0 ]; then
  echo "NOTE: run tools/wsl/build.sh first to enable clang-tidy" >&2
fi
echo "LINT OK"
