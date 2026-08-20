#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# fleet_cycle.sh — the release-to-fleet cycle as ONE command.
#
# Encodes the bench-proven sequence that was previously run by hand (six times
# in one week, ~15 min + one typo-class mistake each):
#   1. fullclean build of BOTH roles (incremental builds keep a stale
#      git-describe — fleet lineage purity requires clean builds)
#   2. verify the expected version string is inside BOTH binaries
#   3. verify credentials made it into BOTH sdkconfigs (a worktree/clone with
#      a missing sdkconfig.credentials builds fine but homes the wrong region)
#   4. OTA the machine FIRST, wait for it back, then every remote
#   5. verify fw_ver on EVERY device (a mixed fleet invalidated run-32a)
#   6. upload both binaries to the firmware registry
#   7. archive both ELFs per lineage (coredump decode needs the EXACT build;
#      clean rebuilds silently destroy the only copy)
#
# Fleet addresses and credentials are PROPRIETARY and live in a gitignored
# env file — nothing sensitive is in this script. Create fleet.env next to
# this script (see fleet.env.example) before running.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ENV_FILE="${FLEET_ENV:-$HERE/fleet.env}"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "fleet_cycle: $ENV_FILE not found — copy tools/fleet.env.example and fill it in" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ENV_FILE"
: "${MACHINE_IP:?fleet.env must set MACHINE_IP}"
: "${REMOTE_IPS:?fleet.env must set REMOTE_IPS (space-separated)}"
: "${DEVICE_AUTH:?fleet.env must set DEVICE_AUTH (user:pass)}"
# REGISTRY_URL / REGISTRY_AUTH / ELF_ARCHIVE_DIR are optional.

VERSION="$(git -C "$REPO" describe --always --dirty)"
if [[ "$VERSION" == *-dirty ]]; then
  echo "fleet_cycle: refusing to flash a dirty tree ($VERSION)" >&2
  exit 1
fi
echo "== lineage $VERSION"

for role in firmware machn; do
  echo "== fullclean build: $role"
  (cd "$REPO/$role" && idf.py fullclean >/dev/null && idf.py build >/dev/null)
done

echo "== verify version string + credentials"
strings "$REPO/firmware/build/pstop_remote.bin" | grep -q "$VERSION" || {
  echo "remote binary missing $VERSION" >&2
  exit 1
}
strings "$REPO/machn/build/machn_machine.bin" | grep -q "$VERSION" || {
  echo "machine binary missing $VERSION" >&2
  exit 1
}
for role in firmware machn; do
  grep -q "CONFIG_ML_FLEET_SERVER_IP=\"..*\"" "$REPO/$role/sdkconfig" || {
    echo "$role/sdkconfig has empty fleet credentials" >&2
    exit 1
  }
done

ota() { # ota <ip> <binary>
  curl -sf -m 180 -u "$DEVICE_AUTH" --data-binary @"$2" "http://$1/admin/api/ota" >/dev/null
}
wait_ver() { # wait_ver <ip>
  for _ in $(seq 1 60); do
    got="$(curl -sf -m 5 "http://$1/state.json" | python3 -c 'import json,sys;print(json.load(sys.stdin).get("fw_ver",""))' 2>/dev/null || true)"
    [[ "$got" == *"$VERSION"* ]] && return 0
    sleep 5
  done
  echo "device $1 never reported $VERSION" >&2
  return 1
}

echo "== flash machine first: $MACHINE_IP"
ota "$MACHINE_IP" "$REPO/machn/build/machn_machine.bin"
wait_ver "$MACHINE_IP"

for ip in $REMOTE_IPS; do
  echo "== flash remote $ip"
  ota "$ip" "$REPO/firmware/build/pstop_remote.bin"
done
failed=""
for ip in $REMOTE_IPS; do
  # NOT `wait_ver && echo`: under set -e a failing non-final &&-element is
  # errexit-EXEMPT, so a mixed fleet would sail through silently — the exact
  # failure this script exists to prevent (review red).
  if wait_ver "$ip"; then
    echo "   $ip OK"
  else
    failed="$failed $ip"
  fi
done
if [[ -n "$failed" ]]; then
  echo "fleet_cycle: version verification FAILED on:$failed" >&2
  exit 1
fi

if [[ -n "${REGISTRY_URL:-}" ]]; then
  echo "== registry upload"
  curl -sf -m 60 -u "$REGISTRY_AUTH" -F "file=@$REPO/firmware/build/pstop_remote.bin" \
    -F "notes=$VERSION (fleet_cycle)" "$REGISTRY_URL/api/firmware" >/dev/null
  curl -sf -m 60 -u "$REGISTRY_AUTH" -F "file=@$REPO/machn/build/machn_machine.bin" \
    -F "notes=$VERSION machine (fleet_cycle)" "$REGISTRY_URL/api/firmware" >/dev/null
fi

if [[ -n "${ELF_ARCHIVE_DIR:-}" ]]; then
  mkdir -p "$ELF_ARCHIVE_DIR/$VERSION"
  cp "$REPO/firmware/build/pstop_remote.elf" "$REPO/machn/build/machn_machine.elf" "$ELF_ARCHIVE_DIR/$VERSION/"
  echo "== ELFs archived to $ELF_ARCHIVE_DIR/$VERSION"
fi

echo "== fleet uniform on $VERSION"
