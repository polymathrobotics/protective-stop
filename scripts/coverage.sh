#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Aggregate STRUCTURAL coverage for the in-scope codebases and emit one summary:
#   1. remote firmware decision core  — host-compiled (gcc-14), line+branch+MC/DC
#   2. host machine (machine_app_runner.c) — gcc-14, line+branch+MC/DC
#   3. ROS2 machine (hand-written)    — colcon + gcovr (gcc-11, line+branch)
#
# pstop_c is a PRE-QUALIFIED component with its own Bullseye coverage + CI
# (.github/workflows/pstop_c_coverage.yml) and is excluded here by design.
#
# Usage:  scripts/coverage.sh            # summary to stdout + docs/safety/coverage/SUMMARY.md
#         GCOVR=/path/to/gcovr scripts/coverage.sh
#         SKIP_ROS2=1 scripts/coverage.sh # host + firmware only (no ROS 2 env)
#
# Env: CC (gcc-14), GCOV (gcov-14), GCOVR (gcovr), ROS_SETUP (auto-detected).
# shellcheck disable=SC2012,SC2015  # ls-glob for toolchain path; guard uses && || intentionally
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
CC=${CC:-gcc-14}
GCOV=${GCOV:-gcov-14}
GCOVR=${GCOVR:-gcovr}
OUT="$ROOT/docs/safety/coverage"
mkdir -p "$OUT"
ROWS=""    # markdown rows accumulator
rc=0

have() { command -v "$1" >/dev/null 2>&1; }

# summarize <json> -> "line% covered total branch% cond%"  (cond% blank if none)
summarize() {
  python3 - "$1" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
ct = d.get("condition_total") or 0
cond = "%.1f" % d["condition_percent"] if ct else "-"
print("%.1f %d %d %.1f %s" % (
    d["line_percent"], d["line_covered"], d["line_total"], d["branch_percent"], cond))
PY
}

row() { ROWS+="| $1 | $2 | $3 | $4 |\n"; }

echo "== [1/3] remote firmware decision core (estop_verdict.c) =="
if have "$CC"; then
  ( cd "$ROOT/firmware/test" && make clean >/dev/null 2>&1 && make CC="$CC" run >/dev/null 2>&1 )
  if "$GCOVR" --root "$ROOT/firmware" "$ROOT/firmware/test" --gcov-executable "$GCOV" \
        --filter '.*/estop_verdict\.c' --json-summary-pretty -o "$OUT/fw.json" >/dev/null 2>&1; then
    read -r L C T B M < <(summarize "$OUT/fw.json")
    echo "   line $L% ($C/$T)  branch $B%  MC/DC $M%"
    row "Firmware decision core (\`estop_verdict.c\`)" "$L% ($C/$T)" "$B%" "$M%"
  else echo "   gcovr failed"; rc=1; fi
  ( cd "$ROOT/firmware/test" && make clean >/dev/null 2>&1 )
else echo "   SKIP: $CC not installed"; row "Firmware decision core" "skipped (no $CC)" "-" "-"; fi

echo "== [2/3] host machine (machine_app_runner.c) =="
if have "$CC"; then
  ( cd "$ROOT/host" && rm -f ./*.gcda ./*.gcno ./*.o
    "$CC" -O0 -g -Wno-unused-parameter -pthread \
      -I../pstop_c/pstop/include -I../pstop_c/transport/include -I../common \
      -c ../pstop_c/pstop/src/pstop/*.c ../pstop_c/transport/src/transport/udp/udp_transport.c
    "$CC" -O0 -g --coverage -fcondition-coverage -Wno-unused-parameter -pthread \
      -I../pstop_c/pstop/include -I../pstop_c/transport/include -I../common -c machine_app_runner.c clock_guard.c
    "$CC" --coverage -pthread -o machine_app_runner ./*.o )
  python3 "$ROOT/tools/pstop_multi_remote_test.py" >/dev/null 2>&1 || rc=1
  python3 "$ROOT/tools/test_config_floor.py"       >/dev/null 2>&1 || rc=1
  if "$GCOVR" --root "$ROOT" "$ROOT/host" --gcov-executable "$GCOV" \
        --filter 'host/machine_app_runner\.c' --json-summary-pretty -o "$OUT/host.json" >/dev/null 2>&1; then
    read -r L C T B M < <(summarize "$OUT/host.json")
    echo "   line $L% ($C/$T)  branch $B%  MC/DC $M%"
    row "Host machine (\`machine_app_runner.c\`)" "$L% ($C/$T)" "$B%" "$M%"
  else echo "   gcovr failed"; rc=1; fi
  ( cd "$ROOT/host" && rm -f ./*.gcda ./*.gcno ./*.o && make >/dev/null 2>&1 )   # restore prod binary
else echo "   SKIP: $CC not installed"; row "Host machine" "skipped (no $CC)" "-" "-"; fi

echo "== [3/3] ROS2 machine (protective_stop_machine) =="
ROS_SETUP=${ROS_SETUP:-$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1)}
if [ "${SKIP_ROS2:-0}" = "1" ] || [ -z "$ROS_SETUP" ] || ! have colcon; then
  echo "   SKIP: no ROS 2 environment (set ROS_SETUP or install colcon)"
  row "ROS2 machine (hand-written)" "skipped (no ROS 2)" "-" "-"
else
  # shellcheck disable=SC1090
  ( cd "$ROOT/ros2" && set +u && . "$ROS_SETUP" && set -u
    rm -rf build-cov install-cov
    colcon build --packages-up-to protective_stop_machine --build-base build-cov --install-base install-cov \
      --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-O0 -g --coverage" \
      -DCMAKE_EXE_LINKER_FLAGS="--coverage" >/dev/null 2>&1
    colcon test --packages-select protective_stop_machine --build-base build-cov --install-base install-cov >/dev/null 2>&1 )
  if "$GCOVR" --root "$ROOT/ros2/protective_stop_machine" "$ROOT/ros2/build-cov/protective_stop_machine" \
        --filter '.*/protective_stop_machine/(src|include)/.*' --exclude '.*/test/.*' \
        --json-summary-pretty -o "$OUT/ros2.json" >/dev/null 2>&1; then
    read -r L C T B M < <(summarize "$OUT/ros2.json")
    echo "   line $L% ($C/$T)  branch $B%"
    row "ROS2 machine (hand-written)" "$L% ($C/$T)" "$B%" "n/a (gcc-11)"
  else echo "   gcovr failed"; rc=1; fi
fi

# ---- combined summary ----
DATE=$(date -u +%Y-%m-%d 2>/dev/null || echo "unknown")
{
  echo "<!-- generated by scripts/coverage.sh -->"
  echo "# Structural coverage — aggregate summary"
  echo
  echo "_Generated $DATE. \`pstop_c\` is pre-qualified (own Bullseye CI), excluded._"
  echo
  echo "| Codebase | Line | Branch | MC/DC |"
  echo "|---|---|---|---|"
  printf "%b" "$ROWS"
} > "$OUT/SUMMARY.md"

echo
echo "==== aggregate summary ($OUT/SUMMARY.md) ===="
printf "%b" "$ROWS" | sed 's/^/  /'
[ $rc -eq 0 ] && echo "coverage.sh: OK" || echo "coverage.sh: some stages failed (rc=$rc)"
exit $rc
