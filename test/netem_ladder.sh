#!/bin/bash
# WG/Tailscale-layer chaos v2 — self-installing, self-verifying, self-arming.
#   - (re)installs tc scaffolding at start (the tether iface loses qdiscs on
#     every chip reboot/USB re-enumeration)
#   - logs netem qdisc packet counters per phase to PROVE impairment applied
#   - re-arms the machine via the pstop_sim pulse — TEST BUILDS ONLY: the
#     endpoint was removed from production firmware (404s harmlessly there);
#     on production, arm with a physical press of the E-stop switch instead.
# Needs sudo (tc/ifb). Outputs netem2.csv + netem_phases2.log next to this
# script.
#
# Usage: IF=<bench-iface> CHIP=<chip-ip> CHIP_TS=<chip-tailscale-ip> ./netem_ladder.sh
set -u
S="$(dirname "$0")"
IF="${IF:?set IF=<bench interface carrying the WG underlay>}"
CHIP="${CHIP:?set CHIP=<chip-ip>}"
CHIP_TS="${CHIP_TS:?set CHIP_TS=<chip tailscale ip>}"

install_scaffolding() {
  sudo tc qdisc del dev $IF root 2>/dev/null
  sudo tc qdisc del dev $IF ingress 2>/dev/null
  sudo tc qdisc del dev ifb0 root 2>/dev/null
  sudo tc qdisc add dev $IF root handle 1: prio bands 4 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  sudo tc qdisc add dev $IF parent 1:4 handle 40: netem loss 0%
  sudo tc filter add dev $IF parent 1: protocol ip prio 1 u32 \
       match ip protocol 17 0xff match ip dport 51820 0xffff flowid 1:4
  sudo modprobe ifb numifbs=1 2>/dev/null
  sudo ip link set ifb0 up
  sudo tc qdisc add dev $IF handle ffff: ingress
  sudo tc filter add dev $IF parent ffff: protocol ip prio 1 u32 \
       match ip protocol 17 0xff match ip sport 51820 0xffff \
       action mirred egress redirect dev ifb0
  sudo tc qdisc add dev ifb0 root handle 2: netem loss 0%
}

netem_pkts() { # echo "<egress_pkts> <ingress_pkts>"
  local E I
  E=$(sudo tc -s qdisc show dev $IF handle 40: 2>/dev/null | grep -o 'Sent [0-9]* bytes [0-9]* pkt' | awk '{print $4}')
  I=$(sudo tc -s qdisc show dev ifb0 2>/dev/null | grep -o 'Sent [0-9]* bytes [0-9]* pkt' | awk '{print $4}')
  echo "${E:-MISSING} ${I:-MISSING}"
}

set_netem() {
  sudo tc qdisc change dev $IF parent 1:4 handle 40: netem $1 || echo "$(date +%s) TC-CHANGE-FAILED egress [$1]" >> "$S/netem_phases2.log"
  sudo tc qdisc change dev ifb0 root handle 2: netem $1 || echo "$(date +%s) TC-CHANGE-FAILED ingress [$1]" >> "$S/netem_phases2.log"
}

arm() { # simulated protective-stop pulse -> machine STOP_RECEIVED -> OK
        # (test builds only; production firmware 404s — press the switch)
  curl -s -m 5 -X POST "http://$CHIP/api/pstop_sim?ms=800" >/dev/null
  sleep 4
}

sample() {
  local NOW STATE PATHL
  NOW=$(date +%s)
  STATE=$(timeout 4 curl -s "http://$CHIP/state.json" 2>/dev/null || true)
  PATHL=$(tailscale status 2>/dev/null | grep "$CHIP_TS" | grep -o 'direct [^,]*\|relay "[a-z0-9]*"' | head -1)
  python3 - "$NOW" "$1" "$STATE" "$PATHL" >> "$S/netem2.csv" 2>/dev/null <<'EOF' || echo "$NOW,$1,UNREACHABLE,,,,,,,,$PATHL" >> "$S/netem2.csv"
import json, sys
now, phase, state, path = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
d = json.loads(state)
print(','.join(str(x) for x in [now, phase, d['uptime_ms']//1000, d['boot_count'],
    d['pstop_sent'], d['pstop_replies'], d['pstop_send_fail'], d['pstop_rtt_ms'],
    d['pstop_rebonds'], d['pstop_last_msg'], path]))
EOF
}

phase() { # $1 name, $2 netem args, $3 duration
  local P0 P1
  P0=$(netem_pkts)
  echo "$(date +%s) NETEM-PHASE $1 [$2] counters-start: $P0" >> "$S/netem_phases2.log"
  set_netem "$2"
  local END=$(( $(date +%s) + $3 ))
  while [ "$(date +%s)" -lt "$END" ]; do sample "$1"; sleep 5; done
  P1=$(netem_pkts)
  echo "$(date +%s) NETEM-PHASE-END $1 counters-end: $P1" >> "$S/netem_phases2.log"
}

echo "epoch,phase,uptime_s,bc,sent,replies,send_fail,rtt_ms,rebonds,last_msg,path" > "$S/netem2.csv"
: > "$S/netem_phases2.log"

install_scaffolding
echo "$(date +%s) SCAFFOLDING-INSTALLED $(netem_pkts)" >> "$S/netem_phases2.log"

arm
phase n0-baseline   "loss 0%"      120
phase n1-loss10     "loss 10%"     180
phase n2-delay150   "delay 150ms"  180
phase n3-blackhole  "loss 100%"    300
set_netem "loss 0%"
sleep 20 && arm                      # let re-bond settle, then re-arm
phase n4-restore    "loss 0%"      240
echo "$(date +%s) NETEM-COMPLETE $(netem_pkts)" >> "$S/netem_phases2.log"
echo "NETEM LADDER v2 COMPLETE"
