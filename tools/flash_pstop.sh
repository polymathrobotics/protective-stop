#!/usr/bin/env bash
#
# flash_pstop.sh — provision a pstop remote over USB from a known-good image.
#
# Flashes the pre-built binaries in tools/production_image/ (bootloader,
# partition table, otadata, app) to an ESP32-S3 over USB. Use this to bring up
# new units in production from ONE compiled image — no ESP-IDF or rebuild
# needed, just esptool. After flashing, the chip self-provisions (per-unit ID
# from its MAC, auto-joins Tailscale, checks in to the fleet) and takes all
# future updates over OTA from the fleet monitor.
#
# The image in tools/production_image/ contains secrets (Tailscale key, WiFi
# creds, admin password, fleet key) and is git-ignored — never commit it.
#
# Usage:
#   tools/flash_pstop.sh                 # auto-detect a chip in download mode
#   tools/flash_pstop.sh /dev/ttyACM0    # explicit port
#   tools/flash_pstop.sh --from-ip 10.42.0.106   # reflash a RUNNING unit
#   tools/flash_pstop.sh --erase /dev/ttyACM0    # full chip erase first
#
# A NEW/blank chip is already in download mode — just plug in USB and run.
# An ALREADY-CONFIGURED unit has no serial port (USB-NCM); either pass
# --from-ip <admin-ip> to kick it into download mode over HTTP, or hold BOOT
# and tap RESET, then run with its port.
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMG="$HERE/production_image"
BAUD=460800
PORT=""
FROM_IP=""
ERASE=0
ADMIN_USER="admin"

usage() { sed -n '2,32p' "$0"; exit "${1:-0}"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --from-ip) FROM_IP="$2"; shift 2 ;;
    --erase)   ERASE=1; shift ;;
    --baud)    BAUD="$2"; shift 2 ;;
    -h|--help) usage 0 ;;
    -*)        echo "unknown option: $1" >&2; usage 1 ;;
    *)         PORT="$1"; shift ;;
  esac
done

# --- image present? -------------------------------------------------------
for f in bootloader.bin partition-table.bin ota_data_initial.bin pstop_remote.bin; do
  [ -f "$IMG/$f" ] || { echo "ERROR: missing $IMG/$f — stage a build into production_image/ first." >&2; exit 1; }
done

# --- esptool available? ---------------------------------------------------
if   command -v esptool.py >/dev/null 2>&1; then ESPTOOL=(esptool.py)
elif command -v esptool     >/dev/null 2>&1; then ESPTOOL=(esptool)
elif python3 -c "import esptool" >/dev/null 2>&1; then ESPTOOL=(python3 -m esptool)
else echo "ERROR: esptool not found — 'pip install esptool'." >&2; exit 1; fi

# --- find a chip in DOWNLOAD mode. Match Espressif VID 303a in any
#     download/JTAG variant (S3 reports 0009, others 1001/0002) but skip the
#     RUNNING app, which enumerates as 303a:4001 (TinyUSB) and cannot be
#     flashed without a reset into download mode. ---------------------------
detect_port() {
  local p vid pid
  for p in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] || continue
    vid="$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')"
    pid="$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p')"
    [ "$vid" = "303a" ] || continue
    [ "$pid" = "4001" ] && continue      # running pstop app, not download mode
    echo "$p"; return 0
  done
  return 1
}

# admin password for the enter_download call: $ADMIN_PASSWORD env wins,
# else read it from the (git-ignored) build credentials on this machine.
admin_pw() {
  if [ -n "${ADMIN_PASSWORD:-}" ]; then printf '%s' "$ADMIN_PASSWORD"; return; fi
  local cred="$HERE/../firmware/sdkconfig.credentials"
  [ -f "$cred" ] && sed -n 's/^CONFIG_ML_ADMIN_PASSWORD="\(.*\)"/\1/p' "$cred" | head -1
}

# --- reflash a running unit: force it into download mode over HTTP --------
if [ -n "$FROM_IP" ]; then
  PW="$(admin_pw)"
  [ -n "$PW" ] || { echo "ERROR: --from-ip needs the admin password. Set ADMIN_PASSWORD=... or keep firmware/sdkconfig.credentials on this machine." >&2; exit 1; }
  echo ">> forcing unit at $FROM_IP into download mode (admin auth + confirm)..."
  resp="$(curl -s -m6 -u "$ADMIN_USER:$PW" -X POST "http://$FROM_IP/api/enter_download?confirm=1" 2>/dev/null || true)"
  case "$resp" in
    *'"ok":true'*) echo "   $resp" ;;
    *) echo "ERROR: enter_download refused: ${resp:-<no response>}" >&2
       echo "       (check the admin password, or hold BOOT + tap RESET and pass the port)." >&2
       exit 1 ;;
  esac
  echo ">> waiting for the download-mode serial port..."
  for _ in $(seq 1 30); do
    sleep 1
    if p="$(detect_port)"; then PORT="$p"; sleep 1; break; fi   # settle after enumerate
  done
  [ -n "$PORT" ] || { echo "ERROR: no download-mode port appeared (try BOOT+RESET, then pass the port)." >&2; exit 1; }
fi

# --- resolve the port -----------------------------------------------------
if [ -z "$PORT" ]; then
  PORT="$(detect_port || true)"
  [ -n "$PORT" ] || { echo "ERROR: no Espressif chip in download mode found. Plug in a blank chip, or hold BOOT + tap RESET, or pass the port explicitly." >&2; exit 1; }
  echo ">> auto-detected port: $PORT"
fi

echo "=========================================================="
echo " Flashing pstop -> $PORT"
grep -E '^(version|app sha256):' "$IMG/MANIFEST.txt" 2>/dev/null || true
echo "=========================================================="

if [ "$ERASE" = "1" ]; then
  echo ">> erasing flash (full chip)..."
  "${ESPTOOL[@]}" --chip esp32s3 -p "$PORT" -b "$BAUD" erase_flash
fi

"${ESPTOOL[@]}" --chip esp32s3 -p "$PORT" -b "$BAUD" \
  --before default_reset --after hard_reset \
  write_flash --flash_mode dio --flash_size 8MB --flash_freq 80m \
  0x0     "$IMG/bootloader.bin" \
  0x8000  "$IMG/partition-table.bin" \
  0x19000 "$IMG/ota_data_initial.bin" \
  0x20000 "$IMG/pstop_remote.bin"

echo
echo ">> DONE. The unit will reboot, derive its ID from its MAC, join Tailscale,"
echo "   and check in to the fleet. It should appear on the fleet monitor within"
echo "   ~30-60 s; future updates roll out from there over OTA."
