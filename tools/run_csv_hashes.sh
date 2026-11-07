#!/usr/bin/env bash
# sha256 of the five bench Run CSVs that pin the computed-torque path bit for bit (ADR-023
# "Hệ quả" 3, feature controller-registry). Run it with the cobot_bench of two builds on the same
# machine and compare the lists; arm64 (FMA contraction) and x86_64 differ in the last bits, so a
# hash is never committed. Only the .csv files are hashed (the meta file has the git describe).
# Usage: tools/run_csv_hashes.sh <cobot_bench> <out_dir>
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 <cobot_bench> <out_dir>" >&2
  exit 2
fi
bench="$(realpath "$1")"
out="$2"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$out"
out="$(realpath "$out")"
cd "$repo"

# name config profile seed [extra args]
run() {
  local name="$1" config="$2" profile="$3" seed="$4"
  shift 4
  "$bench" run --config "$config" --imperfections "config/imperfections/$profile.yaml" \
    --seed "$seed" --out "$out/$name.csv" "$@" >/dev/null
}

run e2e_01 tests/e2e/configs/ideal_world.yaml ideal 1
run e2e_02 config/default.yaml realistic 7
run e2e_03 config/default.yaml realistic 3
run e2e_04 tests/e2e/configs/e2e_04_safety_limit.yaml ideal 1
run e2e_10 config/default.yaml realistic 7 --waypoints tests/e2e/waypoints/e2e_10.yaml

for name in e2e_01 e2e_02 e2e_03 e2e_04 e2e_10; do
  echo "$(sha256sum "$out/$name.csv" | cut -d' ' -f1)  $name"
done
