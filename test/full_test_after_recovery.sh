#!/bin/bash
# Full test suite to run AFTER chip recovery (post-brick), per user's
# original ask:
#   - "test all features that have been worked on"
#   - "do multiple reset cycles"
#   - "run lots of pings back and forth to test the Tailscale IP reply
#      rate and track P level latencies"
#   - "test WiFi and USB network connection"

set -u
CHIP=10.42.0.80
ADMIN="admin:microlink"
LOG=/tmp/full_test.log
> $LOG

log() { echo "$(date +%H:%M:%S) $*" | tee -a $LOG; }

wait_for_http() {
  local timeout=${1:-90}
  local deadline=$(($(date +%s) + timeout))
  while [ $(date +%s) -lt $deadline ]; do
    out=$(curl -m 2 -s http://$CHIP/state.json 2>/dev/null)
    if [ -n "$out" ]; then return 0; fi
    sleep 2
  done
  return 1
}

pct() { python3 -c "import sys; xs=sorted(float(l) for l in sys.stdin if l.strip()); p=float(sys.argv[1]); n=len(xs); print(xs[max(0,min(n-1,int(n*p)))]) if n else print('nan')" "$1"; }

# Phase 1: stable USB-NCM baseline (ts_boot=0 is default in v15.3.3)
log "===== PHASE 1: USB-NCM baseline ====="
if ! wait_for_http 90; then log "PHASE 1 abort: chip not reachable"; exit 1; fi
state=$(curl -m 3 -s http://$CHIP/state.json)
echo "$state" | python3 -c "
import json,sys
d=json.loads(sys.stdin.read())
print(f'  ts_boot_en={d.get(\"ts_boot_en\",\"?\")} derp={d.get(\"derp_paused\",\"?\")} wg={d.get(\"wg_paused\",\"?\")} ml={d[\"ml_state\"]} bc={d[\"boot_count\"]}')
print(f'  heap_min_int={d.get(\"heap_min_int\",\"?\")} wg_pbuf_fails={d.get(\"wg_pbuf_fails\",\"?\")}')" | tee -a $LOG

log "PHASE 1.1 — USB-NCM ICMP latency, 600 packets at 5/s"
ping -c 600 -i 0.2 -W 1 $CHIP 2>&1 | tee /tmp/ping_usb.txt | tail -3 | tee -a $LOG
N=$(grep -c "time=" /tmp/ping_usb.txt)
loss=$(grep "packet loss" /tmp/ping_usb.txt | head -1)
log "  USB-NCM: $loss, samples=$N"
if [ "$N" -gt 0 ]; then
  rtts=$(grep "time=" /tmp/ping_usb.txt | sed -n 's/.*time=\([0-9.]*\) ms/\1/p')
  p50=$(echo "$rtts" | pct 0.5)
  p90=$(echo "$rtts" | pct 0.9)
  p95=$(echo "$rtts" | pct 0.95)
  p99=$(echo "$rtts" | pct 0.99)
  log "  P50=${p50}ms P90=${p90}ms P95=${p95}ms P99=${p99}ms"
fi

log "PHASE 1.2 — pstop heartbeat over USB-NCM, 120s window"
out=$(curl -m 3 -s http://$CHIP/state.json)
s0=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_sent'])")
r0=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_replies'])")
f0=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_send_fail'])")
sleep 120
out=$(curl -m 3 -s http://$CHIP/state.json)
s1=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_sent'])")
r1=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_replies'])")
f1=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['pstop_send_fail'])")
log "  pstop delta: sent=$((s1-s0)) rep=$((r1-r0)) fail=$((f1-f0))"

# Phase 2: runtime knob sweep
log "===== PHASE 2: Runtime knobs ====="
for ep in /api/derp /api/wg; do
  log "  POST $ep — toggle x2"
  curl -m 3 -s -X POST http://$CHIP$ep | head -c 80; echo
  curl -m 3 -s -X POST http://$CHIP$ep | head -c 80; echo
done
log "  POST /api/derp_delay?ms=1"
curl -m 3 -s -X POST "http://$CHIP/api/derp_delay?ms=1" | tee -a $LOG; echo
log "  POST /api/pstop_peer (default)"
curl -m 3 -s -u $ADMIN -X POST "http://$CHIP/api/pstop_peer?ip=10.42.0.1&port=8890" | tee -a $LOG; echo

# Phase 3: 5x soft restart
log "===== PHASE 3: 5x soft restart cycle ====="
for i in 1 2 3 4 5; do
  curl -m 3 -s -u $ADMIN -X POST http://$CHIP/admin/api/restart >/dev/null
  sleep 18
  wait_for_http 30 || { log "  reset #$i: chip did NOT come back"; continue; }
  state=$(curl -m 3 -s http://$CHIP/state.json | python3 -c "
import json,sys
d=json.loads(sys.stdin.read())
print(f't={d[\"t0\"]/10}s ml={d[\"ml_state\"]} bc={d[\"boot_count\"]} rr={d[\"reset_reason\"]} ts_boot={d.get(\"ts_boot_en\",\"?\")}')")
  log "  reset #$i: $state"
done

# Phase 4: Tailscale-direct ping latency
log "===== PHASE 4: Tailscale-direct ICMP latency ====="
log "  Flipping ts_boot_en to 1 + admin restart"
curl -m 3 -s -X POST http://$CHIP/api/ts_boot | tee -a $LOG; echo
sleep 1
curl -m 3 -s -u $ADMIN -X POST http://$CHIP/admin/api/restart >/dev/null
sleep 25
wait_for_http 45 || { log "PHASE 4 abort: chip not reachable after ts_boot=1 restart"; }
state=$(curl -m 3 -s http://$CHIP/state.json | python3 -c "
import json,sys
d=json.loads(sys.stdin.read())
print(f't={d[\"t0\"]/10}s ml={d[\"ml_state\"]} bc={d[\"boot_count\"]} derp={d[\"derp_paused\"]} wg={d[\"wg_paused\"]} ts_boot={d.get(\"ts_boot_en\",\"?\")}')")
log "  post-restart: $state"
# Wait for ml_state=4 (Tailscale CONNECTED)
log "  waiting for Tailscale CONNECTED..."
for i in $(seq 1 30); do
  out=$(curl -m 2 -s http://$CHIP/state.json 2>/dev/null)
  if [ -n "$out" ]; then
    ml=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['ml_state'])" 2>/dev/null)
    if [ "$ml" = "4" ]; then log "  Tailscale UP at t=$i"; break; fi
  fi
  sleep 2
done
log "  ping 600 packets to chip's Tailscale IP 100.97.180.43..."
ping -c 600 -i 0.2 -W 1 100.97.180.43 2>&1 | tee /tmp/ping_ts.txt | tail -3 | tee -a $LOG
N=$(grep -c "time=" /tmp/ping_ts.txt)
loss=$(grep "packet loss" /tmp/ping_ts.txt | head -1)
log "  Tailscale: $loss, samples=$N/600"
if [ "$N" -gt 0 ]; then
  rtts=$(grep "time=" /tmp/ping_ts.txt | sed -n 's/.*time=\([0-9.]*\) ms/\1/p')
  p50=$(echo "$rtts" | pct 0.5)
  p90=$(echo "$rtts" | pct 0.9)
  p95=$(echo "$rtts" | pct 0.95)
  p99=$(echo "$rtts" | pct 0.99)
  log "  P50=${p50}ms P90=${p90}ms P95=${p95}ms P99=${p99}ms"
fi

# Phase 5: WiFi mode (toggle USB off)
log "===== PHASE 5: WiFi mode ====="
log "  Flipping ts_boot back to 0 first"
curl -m 3 -s -X POST http://$CHIP/api/ts_boot | tee -a $LOG; echo
sleep 1
log "  Flipping usb_enable off — chip reboots into WiFi"
curl -m 3 -s -u $ADMIN -X POST http://$CHIP/api/usb_enable | tee -a $LOG; echo
log "  Sleep 30s for WiFi association"
sleep 30
log "  Chip's WiFi LAN IP (from sdkconfig.credentials: polymath-2G):"
# Try common LAN IPs the chip might get
for ip in 192.168.107.131 192.168.107.130 192.168.107.132 192.168.107.150; do
  out=$(curl -m 2 -s -o /dev/null -w '%{http_code}' http://$ip/state.json 2>/dev/null)
  if [ "$out" = "200" ]; then
    log "    found chip at $ip"
    curl -m 3 -s http://$ip/state.json | python3 -c "
import json,sys
d=json.loads(sys.stdin.read())
print(f'      t={d[\"t0\"]/10}s ml={d[\"ml_state\"]} ts_boot={d.get(\"ts_boot_en\",\"?\")} rssi={d[\"rssi\"]} heap={d[\"free_heap\"]//1024}KB')"
    log "    pinging chip on WiFi..."
    ping -c 100 -i 0.2 -W 1 $ip 2>&1 | tail -2 | tee -a $LOG
    break
  fi
done
log "  Restoring USB mode for further testing"
log "  (need to find the chip on WiFi first to call /api/usb_enable again — manual step if WiFi probe failed)"

log "===== full_test complete ====="
