#!/usr/bin/env bash
# Where this repo keeps its generated files in WSL: one folder per project, so builds of
# different projects (and of different branches) never mix. Source it with the repo root:
#   source tools/wsl/paths.sh <repo-root>
#
#   $COBOT_ROOT/build/<branch>[-asan|-cov]   CMake builds (build.sh, lint.sh)
#   $COBOT_ROOT/colcon/<branch>              colcon workspace (sim.sh, env.sh)
#   $COBOT_ROOT/venv                         Python virtualenv (setup.sh, pytest.sh, lint.sh)
#   $COBOT_ROOT/out                          run outputs (suggested --out location)
#
# <branch> is the checked-out branch with "/" replaced by "-", or "detached".
# Overrides: COBOT_ROOT (default ~/projects/7dof_dynamic_control), COBOT_COLCON_WS, COBOT_VENV.
# tools/wsl/prune.sh removes build/ and colcon/ entries of branches that no longer exist.

COBOT_ROOT="${COBOT_ROOT:-$HOME/projects/7dof_dynamic_control}"
cobot_repo="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

cobot_slug() {
  local branch gitdir head
  branch="$(git -C "$cobot_repo" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  # A worktree created by Windows git has "gitdir: C:/..." in its .git file, which git in WSL
  # cannot open (git_env.sh): read the branch from that gitdir's HEAD instead, without exporting
  # GIT_DIR into the caller's shell.
  if [ -z "$branch" ] && [ -f "$cobot_repo/.git" ]; then
    gitdir="$(sed -n 's/^gitdir: //p' "$cobot_repo/.git" | tr -d '\r')"
    if [[ "$gitdir" =~ ^([A-Za-z]):/(.*)$ ]]; then
      gitdir="/mnt/${BASH_REMATCH[1],,}/${BASH_REMATCH[2]}"
    fi
    head="$(cat "$gitdir/HEAD" 2>/dev/null || true)"
    case "$head" in
      "ref: refs/heads/"*) branch="${head#ref: refs/heads/}" ;;
    esac
  fi
  if [ -z "$branch" ] || [ "$branch" = HEAD ]; then branch=detached; fi
  echo "${branch//\//-}"
}

# cobot_build_dir [asan|cov]
cobot_build_dir() {
  local suffix=""
  case "${1:-}" in
    asan) suffix="-asan" ;;
    cov) suffix="-cov" ;;
  esac
  echo "$COBOT_ROOT/build/$(cobot_slug)$suffix"
}

cobot_colcon_ws() { echo "${COBOT_COLCON_WS:-$COBOT_ROOT/colcon/$(cobot_slug)}"; }
cobot_venv() { echo "${COBOT_VENV:-$COBOT_ROOT/venv}"; }
cobot_out() { echo "$COBOT_ROOT/out"; }
