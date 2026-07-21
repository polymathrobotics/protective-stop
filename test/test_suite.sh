#!/bin/bash
# Comprehensive test suite for v15.2 dual_core_safety
# Assumes chip is at 10.42.0.80 with ts_boot=0 (Tailscale paused at boot)

set -u
CHIP=10.42.0.80
ADMIN="admin:microlink"
LOG=/tmp/test_suite.log
> $LOG

log() { echo "$(date +%H:%M:%S) $*" | tee -a $LOG; }

# Wait for chip with ts_boot effective
wait_for_stable() {
  local timeout=180
  local deadline=$(($(date +%s) + timeout))
  while [ $(date +%s) -lt $deadline ]; do
    out=$(curl -m 2 -s http://$CHIP/state.json 2>/dev/null)
    if [ -n "$out" ]; then
      derp=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['derp_paused'])" 2>/dev/null)
      wg=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['wg_paused'])" 2>/dev/null)
      tboot=$(echo "$out" | python3 -c "import json,sys;print(json.loads(sys.stdin.read())['ts_boot_en'])" 2>/dev/null)
      if [ "$derp" = "1" ] && [ "$wg" = "1" ] && [ "$tboot" = "0" ]; then
        log "chip stable: ts_boot=0 derp/wg paused"
        return 0
      fi
    fi
    sleep 2
  done
  log "TIMEOUT waiting for stable"
  return 1
}

# Test 1: extended HTTP soak with ts_boot=0
test_http_soak() {
  log "=== TEST 1: HTTP soak (60s, every 1s) ==="
  local ok=0 fail=0 start=$(date +%s)
  for i in $(seq 1 60); do
    out=$(curl -m 1.5 -s -o /dev/null -w '%{http_code}' http://$CHIP/state.json)
    if [ "$out" = "200" ]; then ok=$((ok+1)); else fail=$((fail+1)); fi
    sleep 1
  done
  log "TEST 1 result: $ok OK / $fail FAIL in 60s"
}

# Test 2: every runtime knob
test_knobs() {
  log "=== TEST 2: Runtime knobs ==="
  log "  /api/derp toggle x2"
  curl -m 3 -s -X POST http://$CHIP/api/derp | head -c 80; echo | tee -a $LOG
  curl -m 3 -s -X POST http://$CHIP/api/derp | head -c 80; echo | tee -a $LOG
  log "  /api/wg toggle x2"
  curl -m 3 -s -X POST http://$CHIP/api/wg | head -c 80; echo | tee -a $LOG
  curl -m 3 -s -X POST http://$CHIP/api/wg | head -c 80; echo | tee -a $LOG
  log "  /api/derp_delay ms=10,50,1"
  for ms in 10 50 1; do
    curl -m 3 -s -X POST "http://$CHIP/api/derp_delay?ms=$ms" | head -c 80; echo | tee -a $LOG
  done
  log "  /api/wifi_tx_power q=20,52,84"
  for q in 20 52 84; do
    curl -m 3 -s -X POST "http://$CHIP/api/wifi_tx_power?q=$q" | head -c 80; echo | tee -a $LOG
  done
  log "  /api/pstop_peer change to 10.42.0.1:8890 (default)"
  curl -m 3 -s -u $ADMIN -X POST "http://$CHIP/api/pstop_peer?ip=10.42.0.1&port=8890" | head -c 120; echo | tee -a $LOG
}

# Test 3: pstop over USB-NCM 60s
test_pstop_usb() {
  log "=== TEST 3: pstop over USB-NCM (60s) ==="
  out1=$(curl -m 3 -s http://$CHIP/state.json)
  s1=$(echo "$out1" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_sent'])")
  r1=$(echo "$out1" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_replies'])")
  f1=$(echo "$out1" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_send_fail'])")
  log "start: sent=$s1 rep=$r1 fail=$f1"
  sleep 60
  out2=$(curl -m 3 -s http://$CHIP/state.json)
  s2=$(echo "$out2" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_sent'])")
  r2=$(echo "$out2" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_replies'])")
  f2=$(echo "$out2" | python3 -c "import json,sys;d=json.loads(sys.stdin.read());print(d['pstop_send_fail'])")
  log "end: sent=$s2 rep=$r2 fail=$f2 (delta sent=$((s2-s1)) rep=$((r2-r1)) fail=$((f2-f1)))"
}

# Test 4: 5 admin soft-restart cycles
test_resets() {
  log "=== TEST 4: 5x admin soft restart ==="
  for i in 1 2 3 4 5; do
    log "  reset #$i — calling /admin/api/restart"
    curl -m 3 -s -u $ADMIN -X POST http://$CHIP/admin/api/restart | head -c 120; echo | tee -a $LOG
    sleep 18  # wait for reboot + ml_app init
    out=$(curl -m 3 -s http://$CHIP/state.json)
    if [ -n "$out" ]; then
      info=$(echo "$out" | python3 -c "
import json,sys
d=json.loads(sys.stdin.read())
print(f't={d[\"t0\"]/10}s ml={d[\"ml_state\"]} bc={d[\"boot_count\"]} rr={d[\"reset_reason\"]} ts_boot_en={d[\"ts_boot_en\"]}')")
      log "  after reset #$i: $info"
    else
      log "  after reset #$i: UNREACHABLE"
    fi
  done
}

# Test 5: latency to chip on USB-NCM (large sample)
test_latency_usb() {
  log "=== TEST 5: USB-NCM ping latency (300 packets, P50/P90/P95/P99) ==="
  out=$(ping -c 300 -i 0.2 -W 1 10.42.0.80 2>&1)
  loss=$(echo "$out" | grep "packet loss" | head -1)
  rtts=$(echo "$out" | grep "bytes from 10.42.0.80" | sed -n 's/.*time=\([0-9.]*\) ms/\1/p')
  log "  $loss"
  n=$(echo "$rtts" | wc -l)
  log "  samples=$n"
  if [ $n -gt 0 ]; then
    p50=$(echo "$rtts" | sort -n | awk -v n=$n 'NR==int(n*0.5+1) {print; exit}')
    p90=$(echo "$rtts" | sort -n | awk -v n=$n 'NR==int(n*0.9+1) {print; exit}')
    p95=$(echo "$rtts" | sort -n | awk -v n=$n 'NR==int(n*0.95+1) {print; exit}')
    p99=$(echo "$rtts" | sort -n | awk -v n=$n 'NR==int(n*0.99+1) {print; exit}')
    log "  P50=${p50}ms P90=${p90}ms P95=${p95}ms P99=${p99}ms"
  fi
}

main() {
  log "===== test_suite starting ====="
  wait_for_stable || { log "abort: not stable"; exit 1; }
  test_http_soak
  test_pstop_usb
  test_latency_usb
  test_knobs
  test_resets
  log "===== test_suite complete ====="
}

main
