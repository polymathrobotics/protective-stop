#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# LEGACY bench script (v15 era) — prefer docs/RECOVERY_PLAYBOOK.md patterns.
# Auto-recovery: detect when chip re-enumerates, then either OTA (if 4001)
# or esptool-flash (if 1001 ROM mode). Either way push the current build.
# Usage: [CHIP=<chip-ip>] ./auto_recover.sh
cd "$(dirname "$0")/../firmware"
BIN=build/pstop_remote.bin
CHIP="${CHIP:-10.42.0.80}"
LOG=/tmp/auto_recover.log
> $LOG
log() { echo "$(date +%H:%M:%S) $*" | tee -a $LOG; }

log "auto_recover armed, watching for 303a:4001 or 303a:1001..."
while true; do
  rom=$(lsusb -d 303a:1001 2>/dev/null | head -1)
  prod=$(lsusb -d 303a:4001 2>/dev/null | head -1)

  if [ -n "$prod" ]; then
    log "DETECTED production firmware (4001): $prod"
    # Pause Tailscale (just in case it's running), then OTA
    sleep 2
    curl -m 2 -X POST http://$CHIP/api/derp 2>/dev/null >/dev/null
    curl -m 2 -X POST http://$CHIP/api/wg 2>/dev/null >/dev/null
    sleep 1
    log "OTA-ing current binary..."
    out=$(curl -m 60 -u admin:microlink -H "Content-Type: application/octet-stream" \
        --data-binary "@$BIN" http://$CHIP/admin/api/ota \
        -w "\nHTTP %{http_code}" 2>&1)
    if echo "$out" | grep -q '"ok":true'; then
      log "OTA SUCCESS — chip will reboot into the new build"
      log "auto-recovery complete"
      exit 0
    else
      log "OTA failed: $(echo "$out" | tail -3)"
      log "retrying in 5s"
      sleep 5
      continue
    fi

  elif [ -n "$rom" ]; then
    log "DETECTED ROM USB-JTAG (1001): $rom"
    # Find the ttyACM that maps to it
    sleep 2
    for tty in /dev/ttyACM*; do
      info=$(udevadm info "$tty" 2>/dev/null | grep ID_VENDOR_ID=303a)
      if [ -n "$info" ]; then
        log "esptool flashing on $tty"
        source ~/esp-idf-5.5/export.sh > /dev/null 2>&1
        log "erase-flash + flash..."
        idf.py -p "$tty" -B build erase-flash 2>&1 | tail -3 | tee -a $LOG
        idf.py -p "$tty" -B build flash 2>&1 | tail -3 | tee -a $LOG
        log "flash complete — chip should boot the new build cleanly"
        exit 0
      fi
    done
    log "ROM device found but no ttyACM matched. /dev/ttyACM* candidates:"
    for tty in /dev/ttyACM*; do udevadm info "$tty" 2>/dev/null | grep -E "DEVNAME|ID_VENDOR_ID" | tr '\n' ' '; echo; done | tee -a $LOG
    sleep 3
  fi
  sleep 2
done
