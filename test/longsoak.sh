#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Long-soak logger. Records every state.json sample with timestamp.
# Detects:
#  - wedge (unreach)
#  - reboot (t0 decreases)
#  - boot_count change
# Writes to /tmp/longsoak.log

LOG=/tmp/longsoak.log
> $LOG
prev_t=0
prev_bc=-1
prev_sent=0
echo "$(date +%s) START long-soak" | tee -a $LOG
while true; do
  start=$(date +%s)
  out=$(curl -m 3 -s http://10.42.0.80/state.json 2>/dev/null)
  code=$?
  ts=$(date +%H:%M:%S)
  epoch=$(date +%s)
  if [ "$code" = "0" ] && [ -n "$out" ]; then
    line=$(echo "$out" | python3 -c "
import json,sys
try:
    d=json.loads(sys.stdin.read())
    print('t=%d ml=%d bc=%d rr=%d sent=%d rep=%d sfail=%d pbuf=%d hmin=%d heap=%d' % (
      d['t0'], d['ml_state'], d['boot_count'], d['reset_reason'],
      d['pstop_sent'], d['pstop_replies'], d['pstop_send_fail'],
      d.get('wg_pbuf_fails',0), d.get('heap_min_int',0)//1024, d['free_heap']//1024
    ))
except Exception as e:
    print('PARSE_ERROR: %s' % e)
" 2>&1)
    echo "$epoch $ts $line" | tee -a $LOG
    t=$(echo $line | sed -n 's/.*t=\([0-9]*\).*/\1/p')
    bc=$(echo $line | sed -n 's/.*bc=\([0-9]*\).*/\1/p')
    sent=$(echo $line | sed -n 's/.*sent=\([0-9]*\).*/\1/p')
    if [ -n "$t" ] && [ -n "$prev_t" ] && [ "$prev_t" != "0" ]; then
      if [ "$t" -lt "$prev_t" ]; then
        echo "$epoch $ts *** REBOOT t went $prev_t -> $t ***" | tee -a $LOG
      fi
    fi
    if [ -n "$bc" ] && [ "$prev_bc" != "-1" ] && [ "$bc" != "$prev_bc" ]; then
      echo "$epoch $ts *** boot_count $prev_bc -> $bc ***" | tee -a $LOG
    fi
    prev_t=$t
    prev_bc=$bc
    prev_sent=$sent
  else
    echo "$epoch $ts UNREACH code=$code prev_t=$prev_t prev_sent=$prev_sent" | tee -a $LOG
  fi
  # Maintain ~5s cadence accounting for curl time
  elapsed=$(($(date +%s) - start))
  sleep_amt=$((5 - elapsed))
  [ $sleep_amt -gt 0 ] && sleep $sleep_amt
done
