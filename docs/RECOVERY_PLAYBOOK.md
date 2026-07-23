# Recovery playbook — bench operations without button presses

When the USB tether is enabled, USB-OTG is owned by TinyUSB (the chip
enumerates as a USB-NCM ethernet device), so the ROM USB-JTAG isn't
visible to the host: **you cannot esptool-flash without holding BOOT
during reset.** That defeats the no-buttons contract, so every routine
operation goes over the network. These patterns have held across
hundreds of OTA cycles.

Conventions: `$CHIP` = the unit's IP on whatever interface is up
(bench addresses are environment-specific). `/admin/*` needs basic auth
`admin:microlink` (default password, `CONFIG_ML_ADMIN_PASSWORD`);
`/state.json` and `/api/*` need none.

## Pre-flight check (every session)

```sh
# 1. Is the chip reachable?
curl -m 3 -s http://$CHIP/state.json | jq '.uptime_ms, .ml_state, .free_heap'

# Expected: a number, 0-4, a number. If nothing:
#   - Ethernet: link light? DHCP on that LAN?
#   - USB-NCM: does the host have the NCM interface up?
#     (`nmcli c | grep enx` — NetworkManager usually handles the share.)
#     No NCM interface at all => TinyUSB didn't enumerate; power-cycle
#     the chip. If it's on the LC USB relay: `usb-switch cycle`. A wedged
#     tether (chip enumerates as 303a:4001 but takes no IP on 10.42.0.0/24)
#     also clears with a power-cycle.

# 2. Is the admin UI reachable?
curl -m 3 -s -o /dev/null -w "%{http_code}\n" \
    -u admin:microlink http://$CHIP/admin/
# 200 = good. 401 = password mismatch. 000 = wedged; see below.
```

## OTA deploy pattern (the canonical one)

OTA is safe while Tailscale is fully loaded — just push:

```sh
cd firmware
curl -m 120 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/pstop_remote.bin" \
     http://$CHIP/admin/api/ota
```

Expected: `{"ok":true,...}` and a reboot; back with `ml_state=4` in
~10 s. The ring shows the dim-purple comet while the image is being
flashed. Check the result:

```sh
curl -u admin:microlink -s http://$CHIP/admin/api/ota/status | jq
```

Rules learned the hard way:

- **Do not OTA with a flapping interface enabled** — route churn kills
  the upload TCP session. Disable a bouncing WiFi first
  (`curl -X POST http://$CHIP/api/iface/wifi`).
- If the POST hangs past 90 s or returns `HTTP 000`: the chip is either
  crashing during the write (check `/api/last_log` after it returns) or
  unreachable. Wait ~3 min for the safety chain, then retry. A partial
  OTA never half-applies — the bootloader accepts the full image
  atomically or rejects it.
- Optional quiesce (rarely needed): `curl -X POST http://$CHIP/api/derp`
  and `curl -X POST http://$CHIP/api/wg` pause Tailscale before the
  upload; the reboot after OTA restores clean state.
- If the upstream pstop_c submodule was bumped, remember the wire rule:
  a CRC change means chip and machine MUST be updated together.

## Recover from a wedge (chip unreachable > 30 s)

The safety chain does this on its own — wait for it:

```sh
while ! curl -m 2 -s http://$CHIP/state.json > /dev/null; do
    echo "$(date +%H:%M:%S) unreachable"; sleep 5
done
echo "back up"
curl -s http://$CHIP/state.json | jq '.boot_count, .reset_reason, .rst_hist'
```

- The network-liveness watchdog aborts (and reboots) only after the
  gateway AND pstop have both been silent 180 s on a previously healthy
  uplink — and that reboot does NOT count toward rollback.
- `boot_count` 1 = one crash boot, normal recovery (this boot runs
  DERP-only, auto-restores after 120 s healthy).
- `boot_count` 3 = one more crash-class boot triggers rollback. Ship a
  fix soon.
- Grab `/api/last_log` before the next reboot overwrites context.

## Recover a unit in the boot-safety pause

If the unit is reachable but Tailscale is paused — `/state.json` shows
`wg_paused=1`, `derp_paused=1`, `ts_boot_en=0`, and `/admin/api/monitor` shows
the `wg_mgr` task **Suspended** with `derp_connected=false` — the boot-safety
ladder disabled Tailscale after repeated crash-class boots (`boot_count>=2`).
Re-enable it and clean-restart:

```sh
curl -u admin:microlink -X POST http://$CHIP/api/ts_boot        # confirm "ts_boot_en":1
curl -u admin:microlink -X POST http://$CHIP/admin/api/restart  # CLEAN restart
```

Use the SW restart, **not** a power-cycle — a hard power cut is a crash-class
boot and re-arms the pause. (Aggressive OTA + hard power-cycling on the bench is
the usual cause; go easy on the power relay while iterating.)

## Fleet can't reach / update a unit

The direct `/admin/api/ota` upload above is also the **fleet-bypass recovery**:
if a unit is unreachable from the fleet (e.g. wedged on the wrong DERP region so
its check-in/OTA fails), push a known-good `pstop_remote.bin` directly to any
address you *can* reach it on (LAN, USB tether `10.42.0.1`, or its Tailscale IP
from a same-region peer). It flashes, reboots, and self-heals. See
`TROUBLESHOOTING.md` → "Reachable over USB/LAN but not via its Tailscale IP".

## Recover from a botched OTA you just pushed

There is no force-rollback endpoint; you have two options:

- **Wait.** If the new image crashes before init completes, the
  bootloader's PENDING_VERIFY rollback restores the previous image on
  the very next reset (~17 s). If it crashes after init, the crash
  counter rolls back after the 4th crash boot (~a few minutes).
- **Re-OTA** the last known-good `pstop_remote.bin` in any healthy
  window between crashes.

## Wholly bricked? (both slots bad)

Shouldn't happen — the never-brick guard refuses a rollback that would
leave no bootable image — but if it does:

1. Hold BOOT, tap RESET — chip enters ROM download mode.
2. `cd firmware && idf.py -p /dev/ttyACM0 flash` (or esptool directly).
   For production/bulk provisioning from a pre-built image, use
   `tools/flash_pstop.sh` (auto-detects a download-mode chip; `--from-ip`
   forces a running unit into download mode). See `README.md`.
3. The chip auto-reboots into normal mode. NVS (settings, pstop peer,
   reset history) survives — the partition table keeps stable offsets.

## Useful one-liners

```sh
# Watch state live
watch -n 5 -d "curl -m 3 -s http://$CHIP/state.json | jq"

# Monitor pstop liveness
while true; do
    curl -m 2 -s http://$CHIP/state.json \
      | jq -c '{ml:.ml_state, sent:.pstop_sent, rep:.pstop_replies,
                fail:.pstop_send_fail, mm:.pstop_mismatch, rb:.pstop_rebonds}'
    sleep 1
done

# Repoint the pstop client at a new machine (NVS-persisted, applies within one tick)
curl -X POST "http://$CHIP/api/pstop_peer?ip=<machine-ip>&port=8890"

# Interface toggles (supervisor re-picks the default route within 1 s)
curl -X POST http://$CHIP/api/iface/eth
curl -X POST http://$CHIP/api/iface/wifi
curl -X POST http://$CHIP/api/iface/usb    # NB: enabling USB installs TinyUSB
                                           # (flashing then needs OTA until next reboot)

# Pause / resume Tailscale (both are toggles)
curl -X POST http://$CHIP/api/derp
curl -X POST http://$CHIP/api/wg

# Previous boot's log tail (survives crashes, not power cycles)
curl -s http://$CHIP/api/last_log
```

## Build-OTA-watch loop (iterating on a fix)

```sh
#!/bin/bash
set -e
CHIP="${CHIP:?set CHIP=<ip>}"
cd firmware
source ~/esp-idf-5.5/export.sh
idf.py build
curl -m 300 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/pstop_remote.bin" \
     http://$CHIP/admin/api/ota
sleep 12
for i in $(seq 1 60); do
    line=$(curl -m 2 -s http://$CHIP/state.json \
        | jq -c '{ml:.ml_state, bc:.boot_count, rr:.reset_reason,
                  sent:.pstop_sent, rep:.pstop_replies}' \
        || echo "UNREACHABLE")
    echo "$(date +%H:%M:%S) $line"
    sleep 5
done
```

If after 5 min `boot_count` is 0 and `pstop_sent` grows monotonically at
10 Hz, the build is healthy.

## Required tools on the bench host

`curl`, `jq`, Python 3 (stdlib only, for the `tools/` scripts), and
ESP-IDF 5.5 (`source ~/esp-idf-5.5/export.sh`) for builds. The machine
side needs only `cc` + `make` (see `host/README.md`).
