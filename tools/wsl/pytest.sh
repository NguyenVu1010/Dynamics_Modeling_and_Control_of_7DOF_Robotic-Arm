#!/usr/bin/env bash
# Run the Python tests in the repo virtualenv (created by tools/wsl/setup.sh).
# Usage: pytest.sh [pytest args...]
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
venv="${COBOT_VENV:-$HOME/.venvs/cobot}"
if [ ! -x "$venv/bin/python" ]; then
  echo "virtualenv $venv missing: run tools/wsl/setup.sh" >&2
  exit 2
fi
cd "$repo"
"$venv/bin/python" -m pytest "$@"
