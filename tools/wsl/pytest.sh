#!/usr/bin/env bash
# Run the Python tests in the repo virtualenv (created by tools/wsl/setup.sh).
# Usage: pytest.sh [pytest args...]
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=paths.sh
source "$repo/tools/wsl/paths.sh" "$repo"
venv="$(cobot_venv)"
if [ ! -x "$venv/bin/python" ]; then
  echo "virtualenv $venv missing: run tools/wsl/setup.sh" >&2
  exit 2
fi
cd "$repo"
"$venv/bin/python" -m pytest "$@"
