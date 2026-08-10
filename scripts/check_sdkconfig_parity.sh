#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# sdkconfig parity — keep the shared remote/machine defaults from silently
# diverging (David, #84 review).
#
# firmware/sdkconfig.defaults (remote) and machn/sdkconfig.defaults (machine)
# are two separate files but share the great majority of their settings
# (lwIP tuning, WireGuard/DERP knobs, partition layout, log level, TinyUSB,
# crypto, etc.). They MUST stay identical for every setting the two roles
# have in common — a value that drifts in one file but not the other is a
# silent, build-passing regression (e.g. a pbuf-pool or handshake-pacing knob
# tuned on the remote but not the machine). This check asserts that: for every
# `CONFIG_*=...` key present in BOTH files, the values match.
#
# Keys that are legitimately role-specific are listed in EXCEPTIONS below and
# skipped. Keys present in only one file are allowed (role-only settings).
# Add to EXCEPTIONS — with a comment saying why — when a value is deliberately
# per-role; anything else that diverges FAILS.
#
# Usage: scripts/check_sdkconfig_parity.sh
# Exit:  0 = in parity, 1 = a shared key diverged, 2 = a defaults file missing.
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
FW="$ROOT/firmware/sdkconfig.defaults"
MC="$ROOT/machn/sdkconfig.defaults"

for f in "$FW" "$MC"; do
  [ -f "$f" ] || { echo "check_sdkconfig_parity: defaults file not found: $f"; exit 2; }
done

# Keys that are intentionally per-role. Keep this list SHORT and justified.
EXCEPTIONS=(
  # OTA cross-lineage guard: each role only accepts its own image project name
  # (remote = pstop_remote, machine = machn_machine). Divergence is the point.
  CONFIG_ML_OTA_REQUIRE_PROJECT
)

is_exception() {
  local k="$1"
  for e in "${EXCEPTIONS[@]}"; do
    [ "$k" = "$e" ] && return 0
  done
  return 1
}

# Value of a set CONFIG_ key in a file (empty if unset / commented-out).
value_of() {
  # Match "CONFIG_KEY=..." at line start; strip the "CONFIG_KEY=" prefix.
  grep -E "^$1=" "$2" | head -1 | cut -d= -f2-
}

fails=0
# Iterate keys that are set (uncommented) in the remote defaults.
while IFS= read -r key; do
  is_exception "$key" && continue
  fv=$(value_of "$key" "$FW")
  mv=$(value_of "$key" "$MC")
  # Only compare keys that are set in BOTH files (role-only keys are allowed).
  [ -z "$mv" ] && continue
  if [ "$fv" != "$mv" ]; then
    echo "DIVERGE $key: firmware=$fv  machn=$mv"
    fails=$((fails + 1))
  fi
done < <(grep -oE '^CONFIG_[A-Za-z0-9_]+' "$FW" | sort -u)

if [ "$fails" -ne 0 ]; then
  echo "check_sdkconfig_parity: $fails shared key(s) diverged between firmware/ and machn/ sdkconfig.defaults."
  echo "  Fix the drift, or (if the difference is intentional and per-role) add the key to EXCEPTIONS in this script with a justifying comment."
  exit 1
fi

echo "check_sdkconfig_parity: OK — all shared sdkconfig.defaults keys are in parity."
exit 0
