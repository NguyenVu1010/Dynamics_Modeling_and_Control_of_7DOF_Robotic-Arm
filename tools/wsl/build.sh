#!/usr/bin/env bash
# Configure, build and test the C++ superbuild inside WSL (or any Ubuntu 24.04).
# Usage: build.sh [--asan] [--coverage] [--build-dir DIR] [--target T] [--no-test]
set -euo pipefail

usage() { echo "Usage: $0 [--asan] [--coverage] [--build-dir DIR] [--target T] [--no-test]" >&2; }

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=git_env.sh
source "$repo/tools/wsl/git_env.sh" "$repo"
asan=0
coverage=0
build_dir=""
target=""
run_tests=1
jobs="${COBOT_JOBS:-3}" # WSL has ~9 GB RAM shared by parallel builds

while [ $# -gt 0 ]; do
  case "$1" in
    --asan) asan=1 ;;
    --coverage) coverage=1 ;;
    --build-dir)
      [ $# -ge 2 ] || { usage; exit 2; }
      build_dir="$2"
      shift
      ;;
    --target)
      [ $# -ge 2 ] || { usage; exit 2; }
      target="$2"
      shift
      ;;
    --no-test) run_tests=0 ;;
    -h | --help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if [ -z "$build_dir" ]; then
  branch="$(git -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null || echo detached)"
  build_dir="$HOME/cobot-build/${branch//\//-}"
  if [ "$asan" = 1 ]; then build_dir="${build_dir}-asan"; fi
  if [ "$coverage" = 1 ]; then build_dir="${build_dir}-cov"; fi
fi

args=(-S "$repo" -B "$build_dir" -DCOBOT_WERROR=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
if [ "$asan" = 1 ]; then
  args+=(-DCMAKE_BUILD_TYPE=Debug "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer")
elif [ "$coverage" = 1 ]; then
  args+=(-DCMAKE_BUILD_TYPE=Debug -DCOBOT_COVERAGE=ON)
else
  args+=(-DCMAKE_BUILD_TYPE=Release)
fi
if command -v ninja >/dev/null 2>&1; then args+=(-G Ninja); fi
if command -v ccache >/dev/null 2>&1; then args+=(-DCMAKE_CXX_COMPILER_LAUNCHER=ccache); fi

cmake "${args[@]}"
if [ -n "$target" ]; then
  cmake --build "$build_dir" --parallel "$jobs" --target "$target"
  if [ "$run_tests" = 1 ]; then
    bin="$(find "$build_dir/tests" -type f -name "$target" -perm -u+x | head -1)"
    if [ -n "$bin" ]; then "$bin" --gtest_color=yes; else echo "NOTE: $target is not a test binary"; fi
  fi
else
  cmake --build "$build_dir" --parallel "$jobs"
  if [ "$run_tests" = 1 ]; then
    ctest --test-dir "$build_dir" --output-on-failure --parallel "$jobs"
  fi
fi
echo "BUILD OK: $build_dir"
