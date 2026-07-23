#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# LEGACY bench script (v15 era) — prefer docs/RECOVERY_PLAYBOOK.md patterns.
# Wait for chip to be reachable + ml=3 or 4, then immediately pause Tailscale and OTA
# Usage: [CHIP=<chip-ip>] ./aggressive_ota.sh
cd "$(dirname "$0")/../firmware"
BIN=build/pstop_remote.bin
CHIP="${CHIP:-10.42.0.80}"
while true; do
  start_ts=$(date +%s)
  # quick probe
  out=$(curl -m 1.5 -s http://$CHIP/state.json 2>/dev/null)
  rc=$?
  if [ "$rc" = "0" ] && [ -n "$out" ]; then
    t=$(echo "$out" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['t0'])" 2>/dev/null)
    bc=$(echo "$out" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['boot_count'])" 2>/dev/null)
    echo "$(date +%H:%M:%S) chip up t=$t bc=$bc — pausing+OTA in 1s"
    # Pause Tailscale fast
    curl -m 1 -X POST http://$CHIP/api/derp 2>/dev/null >/dev/null &
    curl -m 1 -X POST http://$CHIP/api/wg 2>/dev/null >/dev/null &
    wait
    sleep 0.3
    # OTA
    echo "$(date +%H:%M:%S) starting OTA"
    out=$(curl -m 30 -u admin:microlink -H "Content-Type: application/octet-stream" \
        --data-binary "@$BIN" http://$CHIP/admin/api/ota \
        -w "\nHTTP %{http_code} %{time_total}s" 2>&1)
    echo "$out" | tail -3
    if echo "$out" | grep -q '"ok":true'; then
      echo "$(date +%H:%M:%S) OTA SUCCESS"
      exit 0
    else
      echo "$(date +%H:%M:%S) OTA failed — retry"
    fi
  fi
  sleep 0.5
done
