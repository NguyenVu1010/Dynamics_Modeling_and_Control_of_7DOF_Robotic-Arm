#!/usr/bin/env bash
# Static check of the Gazebo assets (no simulator run): xacro expands, the URDF parses, and
# Gazebo's SDF parser accepts it with every mesh resolved.
# Needs a sourced ROS 2 Jazzy and the cobot_sim package installed (tools/wsl/sim.sh build).
# Usage: check_sim_assets.sh <install-prefix-of-cobot_sim>
set -euo pipefail
prefix="${1:?install prefix}"
share="$prefix/share/cobot_sim"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

xacro "$share/urdf/cobot30_5.urdf.xacro" controllers_yaml:="$share/config/controllers.yaml" \
  > "$tmp/cobot.urdf"
check_urdf "$tmp/cobot.urdf" | tee "$tmp/check.txt"
grep -q "root Link: world has 1 child" "$tmp/check.txt"
links="$(grep -c "^\s*\(child(1):\)\?\s*\*\?\s*link" "$tmp/check.txt" || true)"
echo "links reported: $links"

export GZ_SIM_RESOURCE_PATH="$prefix/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sdf -p "$tmp/cobot.urdf" > "$tmp/cobot.sdf"
count="$(grep -c "<link name=" "$tmp/cobot.sdf")"
if [ "$count" -lt 8 ]; then
  echo "expected at least 8 links in the SDF, got $count" >&2
  exit 1
fi
echo "SIM ASSETS OK ($count links in SDF)"
