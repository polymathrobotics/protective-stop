#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# flash_pstop.sh — provision a pstop device (remote OR machine) over USB from a
# known-good image.
#
# Flashes the pre-built binaries (bootloader, partition table, otadata, app) to
# an ESP32-S3 over USB. Use this to bring up new units in production from ONE
# compiled image per role — no ESP-IDF or rebuild needed, just esptool. After
# flashing, the chip self-provisions (per-unit ID from its MAC, auto-joins
# Tailscale, checks in to the OTA backend if one is configured) and takes all
# future updates over the network.
#
# Two device roles, each with its own staged image directory:
#   remote  -> tools/production_image/        (app: pstop_remote.bin)
#   machine -> tools/production_image_machn/   (app: machn_machine.bin)
# If you don't pass --remote/--machine, the tool ASKS which one to flash.
#
# The image directories contain secrets (Tailscale key, WiFi creds, admin
# password, OTA API key) and are git-ignored — never commit them.
#
# Usage:
#   tools/flash_pstop.sh                 # ask role, auto-detect a chip in download mode
#   tools/flash_pstop.sh --remote        # flash the remote build (no prompt)
#   tools/flash_pstop.sh --machine       # flash the machine build (no prompt)
#   tools/flash_pstop.sh /dev/ttyACM0    # explicit port (still asks role)
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
BAUD=460800
PORT=""
FROM_IP=""
ERASE=0
ROLE=""
ADMIN_USER="admin"

usage() { sed -n '2,35p' "$0"; exit "${1:-0}"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --remote)  ROLE=remote; shift ;;
    --machine) ROLE=machine; shift ;;
    --from-ip) FROM_IP="$2"; shift 2 ;;
    --erase)   ERASE=1; shift ;;
    --baud)    BAUD="$2"; shift 2 ;;
    -h|--help) usage 0 ;;
    -*)        echo "unknown option: $1" >&2; usage 1 ;;
    *)         PORT="$1"; shift ;;
  esac
done

# --- device role: which build for the plugged-in chip? --------------------
# Ask if not told. Each role has its own staged image dir + app binary; the
# bootloader/partition-table/otadata come from the same dir so the layout
# always matches the app.
if [ -z "$ROLE" ]; then
  if [ -t 0 ]; then
    printf 'Flash which build for the plugged-in device?\n  [r] remote  (pstop_remote)\n  [m] machine (machn_machine)\n> ' >&2
    read -r ans
    case "$ans" in
      r|R|remote|Remote|REMOTE)    ROLE=remote ;;
      m|M|machine|Machine|MACHINE) ROLE=machine ;;
      *) echo "ERROR: answer 'r' (remote) or 'm' (machine)." >&2; exit 1 ;;
    esac
  else
    echo "ERROR: no device role given — pass --remote or --machine (stdin isn't a TTY to ask)." >&2
    exit 1
  fi
fi
case "$ROLE" in
  remote)  IMG="$HERE/production_image";        APP="pstop_remote.bin" ;;
  machine) IMG="$HERE/production_image_machn";   APP="machn_machine.bin" ;;
  *) echo "ERROR: bad role '$ROLE'." >&2; exit 1 ;;
esac

# --- image present? -------------------------------------------------------
for f in bootloader.bin partition-table.bin ota_data_initial.bin "$APP"; do
  [ -f "$IMG/$f" ] || { echo "ERROR: missing $IMG/$f — stage a $ROLE build into $(basename "$IMG")/ first." >&2; exit 1; }
done

# --- esptool available? (prefer the modern 'esptool' entrypoint) ----------
if   command -v esptool     >/dev/null 2>&1; then ESPTOOL=(esptool)
elif command -v esptool.py  >/dev/null 2>&1; then ESPTOOL=(esptool.py)
elif python3 -c "import esptool" >/dev/null 2>&1; then ESPTOOL=(python3 -m esptool)
else echo "ERROR: esptool not found — 'pip install esptool'." >&2; exit 1; fi

# esptool v5 renamed flags to hyphens and deprecated the underscore forms;
# v4 (the ESP-IDF one) only knows underscores. Pick the right set so we neither
# spam deprecation warnings on v5 nor break on v4.
EVER="$("${ESPTOOL[@]}" version 2>&1 | grep -oiE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -1)"
if [ "${EVER%%.*}" -ge 5 ] 2>/dev/null; then
  O_WF=write-flash O_EF=erase-flash O_MODE=--flash-mode O_SIZE=--flash-size
  O_FREQ=--flash-freq O_BEFORE=default-reset O_AFTER=hard-reset
else
  O_WF=write_flash O_EF=erase_flash O_MODE=--flash_mode O_SIZE=--flash_size
  O_FREQ=--flash_freq O_BEFORE=default_reset O_AFTER=hard_reset
fi

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

# A RUNNING unit exposes a CDC console at 303a:4001 but cannot be flashed —
# esptool times out talking ROM protocol to live firmware. Catch this early
# (esp. when a port was passed explicitly, bypassing detect_port).
rpid="$(udevadm info -q property -n "$PORT" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p')"
if [ "$rpid" = "4001" ]; then
  echo "ERROR: $PORT is a RUNNING pstop (not in download mode) — esptool can't flash it." >&2
  echo "       Force download mode first:" >&2
  echo "         $(basename "$0") --from-ip <admin-ip>      # over the network, or" >&2
  echo "         hold BOOT, tap RESET, then rerun with the port." >&2
  exit 1
fi

echo "=========================================================="
echo " Flashing pstop $ROLE ($APP) -> $PORT"
grep -E '^(version|app sha256):' "$IMG/MANIFEST.txt" 2>/dev/null || true
echo "=========================================================="

if [ "$ERASE" = "1" ]; then
  echo ">> erasing flash (full chip)..."
  "${ESPTOOL[@]}" --chip esp32s3 -p "$PORT" -b "$BAUD" "$O_EF"
fi

"${ESPTOOL[@]}" --chip esp32s3 -p "$PORT" -b "$BAUD" \
  --before "$O_BEFORE" --after "$O_AFTER" \
  "$O_WF" "$O_MODE" dio "$O_SIZE" 8MB "$O_FREQ" 80m \
  0x0     "$IMG/bootloader.bin" \
  0x8000  "$IMG/partition-table.bin" \
  0x19000 "$IMG/ota_data_initial.bin" \
  0x20000 "$IMG/$APP"

echo
echo ">> DONE. The unit will reboot, derive its ID from its MAC, join Tailscale,"
echo "   and check in to the OTA backend (if configured). It should appear there within"
echo "   ~30-60 s; future updates roll out from there over OTA."
