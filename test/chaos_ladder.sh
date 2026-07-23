#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# pstop chaos ladder — sweeps impairments through tools/pstop_chaos_proxy.py.
# Emits phase-stamped chip samples to chaos.csv and phase markers to
# phases.log (both next to this script); machine transitions land in the
# runner's stderr, proxy stats in the proxy's stdout.
#
# Prereqs: pstop_chaos_proxy.py running (default ctrl 127.0.0.1:8892),
# chip's pstop peer pointed at the proxy, machine ARMED (so any
# unscheduled STOP = false trip).
#
# Usage: CHIP=<chip-ip> [CTRL_HOST=127.0.0.1] [CTRL_PORT=8892] ./chaos_ladder.sh
set -u
S="$(dirname "$0")"
CHIP="${CHIP:?set CHIP=<chip-ip>}"
CTRL_HOST="${CTRL_HOST:-127.0.0.1}"
CTRL_PORT="${CTRL_PORT:-8892}"
CTRL() { echo "$1" > "/dev/udp/$CTRL_HOST/$CTRL_PORT"; }

sample() { # $1 = phase name
  local NOW STATE
  NOW=$(date +%s)
  STATE=$(timeout 5 curl -s "http://$CHIP/state.json" 2>/dev/null || true)
  python3 - "$NOW" "$1" "$STATE" >> "$S/chaos.csv" 2>/dev/null <<'EOF' || echo "$(date +%s),$1,UNREACHABLE" >> "$S/chaos.csv"
import json, sys
now, phase, state = sys.argv[1], sys.argv[2], sys.argv[3]
d = json.loads(state)
print(','.join(str(x) for x in [now, phase, d['uptime_ms']//1000, d['boot_count'],
    d['pstop_sent'], d['pstop_replies'], d['pstop_send_fail'], d['pstop_rtt_ms'],
    d['pstop_rebonds'], d['pstop_mismatch'], d['pstop_last_msg']]))
EOF
}

phase() { # $1 name, $2 ctrl line, $3 duration_s
  echo "$(date +%s) PHASE-START $1 [$2]" >> "$S/phases.log"
  CTRL "clear=1"; sleep 1; CTRL "$2"
  local END=$(( $(date +%s) + $3 ))
  while [ "$(date +%s)" -lt "$END" ]; do sample "$1"; sleep 10; done
  echo "$(date +%s) PHASE-END $1" >> "$S/phases.log"
}

outage() { # $1 seconds of 100% loss
  echo "$(date +%s) OUTAGE-START ${1}s" >> "$S/phases.log"
  CTRL "loss=1.0"
  sleep "$1"
  CTRL "clear=1"
  echo "$(date +%s) OUTAGE-END ${1}s" >> "$S/phases.log"
  local END=$(( $(date +%s) + 75 ))   # recovery observation window
  while [ "$(date +%s)" -lt "$END" ]; do sample "recover-${1}s"; sleep 5; done
}

echo "epoch,phase,uptime_s,bc,sent,replies,send_fail,rtt_ms,rebonds,mismatch,last_msg" > "$S/chaos.csv"
: > "$S/phases.log"

phase baseline-proxy "clear=1"                                   180
phase loss-1        "loss=0.01"                                  180
phase loss-2        "loss=0.02"                                  180
phase loss-5        "loss=0.05"                                  240
phase loss-10       "loss=0.10"                                  240
phase loss-20       "loss=0.20"                                  300
phase loss-30       "loss=0.30"                                  300
phase delay-50      "delay_ms=50"                                180
phase delay-100     "delay_ms=100"                               180
phase delay-200     "delay_ms=200"                               180
phase delay-500     "delay_ms=500"                               240
phase jitter-reorder "delay_ms=50 jitter_ms=150"                 240
phase dup-5         "dup=0.05"                                   180
phase dup-20        "dup=0.20"                                   180
phase corrupt-2     "corrupt=0.02"                               180
phase combo-bad     "loss=0.05 delay_ms=100 jitter_ms=80 dup=0.05 corrupt=0.01" 300
outage 0.5
outage 1
outage 2
outage 5
CTRL "clear=1"
echo "$(date +%s) LADDER-COMPLETE" >> "$S/phases.log"
echo "CHAOS LADDER COMPLETE"
