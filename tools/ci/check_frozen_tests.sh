#!/usr/bin/env bash
# Frozen tests (design 11.4). Fails when a PR
#   (1) modifies, renames or deletes a file under tests/ that exists on the base branch,
#       unless the PR carries the label `test-change`, or
#   (2) touches tests/ in any commit after its last `test(` commit.
# Usage: check_frozen_tests.sh <base-ref> <head-ref> [labels-comma-separated]
set -euo pipefail
base="${1:?base ref}"
head="${2:?head ref}"
labels="${3:-}"

merge_base="$(git merge-base "$base" "$head")"
status=0

if [[ ",$labels," != *",test-change,"* ]]; then
  changed_existing="$(git diff --name-status "$merge_base" "$head" -- tests/ | awk '$1 !~ /^A/ {print}')"
  if [ -n "$changed_existing" ]; then
    echo "FROZEN-TESTS: existing tests changed without the test-change label:"
    echo "$changed_existing"
    status=1
  fi
fi

last_test_commit="$(git log --format='%H %s' "$merge_base..$head" "^$base" | awk '$2 ~ /^test(\(|:)/ {print $1; exit}')"
if [ -n "$last_test_commit" ]; then
  # ^base: commits that came from the base branch (merged or rebased in) are not the PR's own.
  late="$(git log --format='%h %s' "$last_test_commit..$head" "^$base" -- tests/)"
  if [ -n "$late" ]; then
    echo "FROZEN-TESTS: commits after the last test( commit touch tests/:"
    echo "$late"
    status=1
  fi
fi

if [ "$status" = 0 ]; then echo "FROZEN-TESTS OK"; fi
exit "$status"
