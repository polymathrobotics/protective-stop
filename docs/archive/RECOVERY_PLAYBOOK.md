# Recovery playbook — bench operations without button presses

The chip is in production firmware mode: USB-OTG is owned by TinyUSB
(so it can be a USB-NCM ethernet device), which means the ROM
USB-JTAG isn't visible to the host. **You cannot esptool-flash this
chip without holding BOOT during reset.** That defeats the no-buttons
contract.

So every operation has to go over the network. This playbook is the
set of patterns that has worked reliably across hundreds of OTA cycles
this session.

## Pre-flight check (do this first, every session)

```sh
# 1. Is the chip reachable on USB-NCM?
curl -m 3 -s http://10.42.0.80/state.json | jq '.t0, .ml_state, .heap_free'

# Expected: a number, 0-4, a number. If you get nothing:
#   - is the USB cable plugged in?
#   - `ip addr show | grep 10.42.0.1` — does the host have the NCM
#     interface up? If not, NetworkManager is probably handling it;
#     `nmcli c | grep enx` should show a connection.
#   - if no NCM interface at all, the chip's TinyUSB stack didn't
#     enumerate. Power-cycle the chip (yank+reinsert USB) once.

# 2. Is the admin UI reachable?
curl -m 3 -s -o /dev/null -w "%{http_code}\n" \
    -u admin:microlink http://10.42.0.80/admin/

# Expected: 200. If 401: password mismatch (default is `microlink`).
# If 000: chip is wedged; see "Recover from wedge" below.
```

## OTA deploy pattern (the canonical one)

As of v15.20+ the lwIP core-locking fix makes OTA-while-Tailscale-
active safe. **No need to pause first** — just push:

```sh
curl -m 120 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/microlink_dual_core_safety.bin" \
     http://10.42.0.80/admin/api/ota
```

Expected response:
```json
{"ok":true,"message":"Firmware updated. Rebooting..."}
```

Bench-verified: 16 s for ~1.1 MB binary under sustained 5 Hz Tailscale
ping + 10 Hz pstop load. Chip back `ml_state=4` in ~8 s post-reboot.

If you'd rather pause Tailscale anyway (e.g. to avoid the 30 s
heartbeat blip during the reboot), the historical two-step is still
valid:

```sh
# Optional: pause TS first so OTA doesn't compete with WG traffic
curl -m 5 -X POST http://10.42.0.80/api/derp
curl -m 5 -X POST http://10.42.0.80/api/wg
# ...upload...
# Resume not required — chip reboots into clean TS state after OTA.
```

If the OTA POST hangs past 90 s or returns `HTTP 000`, the chip is
either (a) crashing during the OTA write (look for a panic on
`/dev/ttyUSB1` UART debug) or (b) unreachable. Wait 3 min for the
safety chain to fire, then retry. **Do not** assume a partial OTA
landed — the bootloader either accepts the full image atomically
or rejects it; there is no half-updated state.

## Recover from wedge

If the chip is unreachable for >30 s:

```sh
# Wait for liveness watchdog to fire (max 180 s post-boot)
# You can monitor by polling and seeing when /state.json comes back:
while ! curl -m 2 -s http://10.42.0.80/state.json > /dev/null; do
    echo "$(date +%H:%M:%S) unreachable"
    sleep 5
done
echo "back up"

# After it's back, check boot_count to confirm the chain fired:
curl -s http://10.42.0.80/state.json | jq '.boot_count, .reset_reason'
# `boot_count` of 1 = first crash, normal recovery
# `boot_count` of 3+ = approaching rollback threshold, ship a fix soon
# `reset_reason` should be "PANIC" or "TASK_WDT" after a wedge
```

If `boot_count` ever hits 4: the next reboot will roll back to the
previous OTA slot. That's by design — the chain is doing its job.

## Force a rollback (i.e. recover from a botched OTA you just pushed)

If you OTA'd a build that wedges in pre-Tailscale init and you don't
want to wait for 3 crash cycles:

```sh
# Option A: just wait. The chain triggers rollback after 3 reboots
# (~6 min worst case).

# Option B: force-trigger if the chip is still responsive between crashes
curl -u admin:microlink -X POST http://10.42.0.80/admin/api/rollback
```

`Option B` calls `esp_ota_mark_app_invalid_rollback_and_reboot()` immediately.
Useful if Layer 2 (liveness) is firing slowly and you want to skip ahead.

## Wholly bricked? (chain failed somehow)

This shouldn't happen, but if both OTA slots are bad:

1. Hold BOOT, tap RESET — chip enters ROM USB-JTAG.
2. `idf.py -p /dev/ttyACM0 flash` (or `esptool.py` directly).
3. After flash, chip auto-reboots into normal mode.

You can do this without disconnecting the chip from the host's
NetworkManager session — the JTAG and USB-NCM are different USB
endpoints on the S3.

We have NOT had to do this once in this entire session. The safety
chain has caught every failure mode.

## Useful one-liners

```sh
# Watch state.json live (5 s refresh)
watch -n 5 -d "curl -m 3 -s http://10.42.0.80/state.json | jq"

# Monitor pstop liveness
while true; do
    curl -m 2 -s http://10.42.0.80/state.json \
        | jq -c '{t:.t0, ml:.ml_state, sent:.pstop_sent, rep:.pstop_replies, fail:.pstop_send_fail}'
    sleep 1
done

# Pause Tailscale (e.g. before a long debug session)
curl -m 5 -X POST http://10.42.0.80/api/derp
curl -m 5 -X POST http://10.42.0.80/api/wg

# Resume Tailscale (same endpoints toggle)
curl -m 5 -X POST http://10.42.0.80/api/derp
curl -m 5 -X POST http://10.42.0.80/api/wg

# Repoint pstop client at a new responder
curl -u admin:microlink -X POST \
    "http://10.42.0.80/api/pstop_peer?ip=10.42.0.1&port=8890"

# Toggle USB tether on/off (NVS-persisted)
curl -u admin:microlink -X POST http://10.42.0.80/api/usb_enable
```

## Auto-OTA-and-monitor pattern (for iterative debugging)

When iterating on a fix, this loop builds, OTAs, and watches
liveness in one go. v15.20+ no longer needs the Tailscale pause —
included below as a comment for the historical case (large tailnets
where DISCO chatter still saturates lwIP, only if you actually hit it).

```sh
#!/bin/bash
set -e
cd examples/dual_core_safety
source ~/esp-idf-5.5/export.sh
idf.py build

# Optional (pre-v15.20 holdout): pause TS to free lwIP for OTA upload.
# curl -m 5 -X POST http://10.42.0.80/api/derp || true
# curl -m 5 -X POST http://10.42.0.80/api/wg || true
# sleep 1
curl -m 300 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/microlink_dual_core_safety.bin" \
     http://10.42.0.80/admin/api/ota

# Wait for reboot
sleep 12

# Watch for 5 min, log to file
for i in $(seq 1 60); do
    line=$(curl -m 2 -s http://10.42.0.80/state.json \
        | jq -c '{t:.t0, ml:.ml_state, bc:.boot_count, rr:.reset_reason, sent:.pstop_sent, rep:.pstop_replies}' \
        || echo "UNREACHABLE")
    echo "$(date +%H:%M:%S) $line"
    sleep 5
done | tee "/tmp/ota_attempt_$(date +%s).log"
```

If after 5 min `boot_count` is still 0 and `pstop_sent` is growing
monotonically, the wedge is gone. Promote that attempt log file with
a meaningful name and commit it.

## Required tools on the bench host

- `curl` (any modern version)
- `jq`
- IDF 5.5: `source ~/esp-idf-5.5/export.sh` before any `idf.py` command
- Python 3 with stdlib `socket` and `struct` (for `pstop_responder.py`)

No proprietary tools, no Anthropic-specific dependencies. Anyone with a
fresh Linux box and the source checked out should be able to follow
this playbook end-to-end.
