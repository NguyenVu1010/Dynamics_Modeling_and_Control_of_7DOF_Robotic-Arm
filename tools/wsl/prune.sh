#!/usr/bin/env bash
# Remove the builds and colcon workspaces of branches that no longer exist locally (merged and
# deleted), under $COBOT_ROOT/build and $COBOT_ROOT/colcon (tools/wsl/paths.sh). Outputs in
# $COBOT_ROOT/out and the venv are kept.
# Usage: prune.sh [--dry-run] [--repo DIR]
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dry=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) dry=1 ;;
    --repo) repo="$2"; shift ;;
    *) echo "usage: $0 [--dry-run] [--repo DIR]" >&2; exit 2 ;;
  esac
  shift
done
# shellcheck source=git_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/git_env.sh" "$repo"
# shellcheck source=paths.sh
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh" "$repo"

declare -A live=([detached]=1)
while read -r branch; do live["${branch//\//-}"]=1; done < <(git -C "$repo" branch --format='%(refname:short)')

removed=0
kept=0
for dir in "$COBOT_ROOT"/build/* "$COBOT_ROOT"/colcon/*; do
  [ -d "$dir" ] || continue
  name="$(basename "$dir")"
  slug="$name"
  case "$(basename "$(dirname "$dir")")" in
    build) slug="${slug%-asan}"; slug="${slug%-cov}" ;;
  esac
  if [ -n "${live[$slug]:-}" ]; then
    kept=$((kept + 1))
    continue
  fi
  removed=$((removed + 1))
  if [ "$dry" = 1 ]; then
    echo "would remove $dir"
  else
    echo "removing $dir"
    rm -rf "$dir"
  fi
done
echo "PRUNE: kept $kept, $([ "$dry" = 1 ] && echo "would remove" || echo removed) $removed"
