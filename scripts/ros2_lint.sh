#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# ros2_lint.sh — run the SAME ament linters that ros2_build.yml enforces in CI,
# locally, via the jazzy CI container. Catches the flake8 / pep257 / uncrustify /
# xmllint / lint_cmake failures that gate the ROS 2 build BEFORE you push, so the
# required jazzy leg stays green.
#
# Usage:
#   scripts/ros2_lint.sh          # check-only, non-zero exit on any violation
#   scripts/ros2_lint.sh --fix    # auto-reformat C++ (uncrustify) in place, then
#                                  # re-check everything
#
# Requires: docker. No local ROS install needed. jazzy is the required CI target,
# so this script lints against jazzy; humble/lyrical are informational in CI.
#
# What is linted (mirrors what each package's CMake actually enforces in CI):
#   - protective_stop_machine : flake8, pep257, uncrustify, xmllint, lint_cmake
#   - other ament_cmake pkgs   : xmllint, lint_cmake
#   (protective_stop_node / _remote declare no ament code-style lints in CI.)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="polymathrobotics/ros:jazzy-builder-ubuntu"
LINT_IMAGE="pstop-ros2-lint:jazzy"
FIX=0
[[ "${1:-}" == "--fix" ]] && FIX=1

# Package with full code-style enforcement (flake8/pep257/uncrustify).
STYLE_PKG="ros2/protective_stop_machine"
# All ament_cmake packages get xmllint + lint_cmake.
CMAKE_PKGS=(
  "ros2/protective_stop_machine"
  "ros2/protective_stop_msg"
  "archive/protective_stop_node"
)

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker is required but not found on PATH." >&2
  exit 2
fi

# Build a cached lint image (base CI image + ament linters) once.
if ! docker image inspect "$LINT_IMAGE" >/dev/null 2>&1; then
  echo ">> Preparing lint image ($LINT_IMAGE); this runs once..."
  docker pull "$BASE_IMAGE"
  cid="$(docker create "$BASE_IMAGE" bash -c '
    apt-get update -qq &&
    apt-get install -y -qq \
      ros-jazzy-ament-flake8 ros-jazzy-ament-uncrustify \
      ros-jazzy-ament-pep257 ros-jazzy-ament-xmllint \
      ros-jazzy-ament-lint-cmake >/dev/null')"
  docker start -a "$cid"
  docker commit "$cid" "$LINT_IMAGE" >/dev/null
  docker rm "$cid" >/dev/null
fi

# Mount read-only for check, read-write for --fix (uncrustify rewrites files).
MOUNT_OPT=":ro"
[[ "$FIX" -eq 1 ]] && MOUNT_OPT=""

docker run --rm -v "${REPO_ROOT}:/ws${MOUNT_OPT}" -w /ws -e FIX="$FIX" \
  -e STYLE_PKG="$STYLE_PKG" -e CMAKE_PKGS="${CMAKE_PKGS[*]}" \
  "$LINT_IMAGE" bash -c '
    # ROS setup.bash is not "set -u" safe; keep pipefail only.
    set -o pipefail
    source /opt/ros/jazzy/setup.bash
    rc=0

    if [ "$FIX" -eq 1 ]; then
      echo ">> uncrustify --reformat ($STYLE_PKG)"
      ament_uncrustify --reformat "$STYLE_PKG" >/dev/null 2>&1 || true
    fi

    echo "== flake8  ($STYLE_PKG) =="
    ament_flake8 "$STYLE_PKG" || rc=1
    echo "== pep257  ($STYLE_PKG) =="
    ament_pep257 "$STYLE_PKG" || rc=1
    echo "== uncrustify ($STYLE_PKG) =="
    ament_uncrustify "$STYLE_PKG" >/dev/null 2>&1 || {
      ament_uncrustify "$STYLE_PKG" 2>&1 | grep -E "divergence in file" || true
      echo "   (run: scripts/ros2_lint.sh --fix)"; rc=1; }

    for pkg in $CMAKE_PKGS; do
      echo "== xmllint / lint_cmake ($pkg) =="
      ament_xmllint "$pkg" >/dev/null 2>&1 || { ament_xmllint "$pkg"; rc=1; }
      ament_lint_cmake "$pkg" >/dev/null 2>&1 || { ament_lint_cmake "$pkg"; rc=1; }
    done

    if [ "$rc" -eq 0 ]; then
      echo "ALL ROS 2 LINTS CLEAN (jazzy)"
    else
      echo "ROS 2 LINT FAILURES — fix before pushing (jazzy is a required gate)."
    fi
    exit $rc
  '
