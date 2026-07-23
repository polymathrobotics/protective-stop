# Troubleshooting — pstop remote

Field guide for diagnosing a sick unit. Verified against the current
firmware (`firmware/main/main.c` + `firmware/components/dcs_support/`).
`$CHIP` below is the unit's IP on whatever interface is up — bench
addresses are environment-specific. `/state.json` and `/api/*` need no
auth; `/admin/*` uses HTTP basic auth (`admin` / `CONFIG_ML_ADMIN_PASSWORD`,
default `microlink`).

---

## First contact: the LED ring, then `/state.json`

The 16-LED WS2812 ring (GPIO17) shows the pstop link as seen from the
machine's replies:

| Ring | Meaning |
|---|---|
| WHITE | idle — no pstop peer (machine) configured; nothing to connect to (fresh unit, or peer cleared) |
| RED (slow pulse) | a peer IS configured but no fresh reply — never bonded, machine down/unreachable, or the comparator stopped after a lockstep fault. Distinct from a solid-red commanded STOP |
| BLUE | bonded, not armed — last reply was BOND/UNBOND |
| GREEN | armed — last reply OK, robot cleared to run |
| RED (solid) | STOP — commanded stop (switch pressed, or NEED_STOP arming cycle not yet completed) |
| PURPLE | lockstep MISMATCH — the two cores disagreed recently (e.g. ONE loop channel open/faulted). Takes priority over all other colours; held 2 s so single blips show |
| dim purple comet | boot sign-of-life, or an OTA image being flashed |

The onboard single LED (GPIO21) shows network state, not pstop state.

```sh
curl -s http://$CHIP/state.json | python3 -m json.tool
```

| Field | Healthy | If not |
|---|---|---|
| `ml_state` | **4** (CONNECTED) | 0–3 = Tailscale not up; check `ts_boot_en`, `wg_paused`, `derp_paused` |
| `boot_count` | **0** | 1 = last boot crashed (this boot is DERP-only); ≥2 = Tailscale paused this boot; >3 = rollback fires |
| `reset_reason` | 1 or 3 | 4/5/6/7 = crash-class, 11 = brownout — see table below |
| `rst_hist` | mostly 1/3 | ring of the 16 most recent reset reasons — repeated 4s/6s = crash pattern |
| `ota_state` | valid | pending-verify right after an OTA until finalize marks it |
| `active_iface` | 1 (eth) / 2 (usb) / 3 (wifi) | 0 = no uplink, 4 = SoftAP setup mode |
| `pstop_sent` / `pstop_replies` | grow together at 10 Hz | replies lagging = path issue; `pstop_send_fail` climbing = chip can't TX (e.g. Tailscale peer while VPN down — fail-safe) |
| `pstop_mismatch` | **0** | climbing = the cores disagree — usually one loop channel open/shorted; ring shows purple |
| `pstop_rebonds` | 0 (rare 1s) | climbing = reply loss >1.5 s bursts (link quality) |
| `e_hi0`/`e_lo0`/`e_hi1`/`e_lo1` | all 1 | per-channel loop diagnostics: `hi=0` = loop open (press/broken wire), `lo=0` = sense stuck high |
| `heap_min_int` | ≥ 20–30 KiB | low-water internal heap; ~6 KiB lows only in USB-NCM mode |
| `wg_pbuf_fails` | 0 | lwIP couldn't alloc for a WG packet (silent drop) |
| `wg_dedup_drops` | climbs slowly | machine's tailscaled hedging direct + DERP simultaneously |
| `gw_rtt_ms` / `gw_ok` / `gw_loss` | 0–few ms / climbing / ~0 | the active uplink's own gateway ping (device-side link health) |
| `rssi` | −50…−75 dBm | below −80 = poor WiFi link, expect drops (0 when WiFi is off) |

## `reset_reason` codes

| Code | Name | Meaning | Look at |
|---|---|---|---|
| 1 | POWERON | Cold boot | nothing — normal |
| 3 | SW | Clean `esp_restart()` (OTA, admin restart, ladder auto-restart) | nothing — normal |
| 4 | PANIC | Stack overflow, hard fault, `abort()` (including a deliberate liveness abort — check the log) | `/api/last_log` |
| 5 | INT_WDT | Interrupts disabled too long | long critical sections |
| 6 | TASK_WDT | TWDT starved 30 s | `/api/last_log`, task list |
| 11 | BROWNOUT | Supply dipped | power supply / cable (not counted toward rollback) |

`reset_reason` is sticky from the last boot. Reason 4 with
`boot_count = 0` means the 120 s age-out already cleared the counter —
the chip is currently healthy.

## Getting the crash log

Primary (remote, no cabling): `GET /api/last_log` serves a 7 KiB
RTC_NOINIT-backed tail of the previous boot's log. Survives
crash/WDT/soft restart — not a power cycle.

Serial options:
- **USB tether disabled** (`usb_enabled=0`): the native USB-Serial-JTAG
  console works over the same USB-C port (`/dev/ttyACM0`).
- **USB tether enabled**: TinyUSB replaces the native console with the
  NCM composite — wire a USB-UART adapter to the UART0 pins to capture
  panics live (`stty -F /dev/ttyUSBx 115200 raw -echo; cat /dev/ttyUSBx`).

Decode backtraces:

```sh
source ~/esp-idf-5.5/export.sh
xtensa-esp32s3-elf-addr2line -e firmware/build/pstop_remote.elf 0x4037.... 0x4037....
```

## What the safety layers look like when they fire

- **Bootloader rollback (bad OTA):**
  `curl -u admin:microlink http://$CHIP/admin/api/ota/status` →
  `"rollback_occurred": true` with the partition it rolled back from.
  ~17 s from bad OTA push to back on the good image, no user action.
- **Crash counter:** `boot_count` in `/state.json`; counter clears after
  120 s healthy uptime. `boot_count > 3` invokes rollback (with a
  never-brick guard if the other slot is also invalid).
- **Degradation ladder:** `boot_count==1` → `derp_only=1` for that boot
  (relay-level latency, auto-restarts back to full speed after the
  age-out); `boot_count>=2` → Tailscale paused (`wg_paused=1`).
- **Liveness abort:** log line "lwIP wedge, rebooting to recover (NOT a
  rollback trigger)" in `/api/last_log`; fires only when the active
  gateway AND pstop have both been silent 180 s after being healthy.

## Common scenarios

### `tailscale ping $CHIP_TS` shows tx climbing, rx 0

Almost always the chip's peer allowlist doesn't include your host's
Tailscale IP (the chip drops DISCO from unknown peers):

```sh
MY_TS_IP=$(tailscale ip -4)
curl -u admin:microlink -X POST http://$CHIP/admin/api/peers/allowed \
     -H "Content-Type: application/json" \
     -d "{\"peers\":[{\"ip\":\"$MY_TS_IP\",\"label\":\"$(hostname)\"}]}"
curl -u admin:microlink -X POST http://$CHIP/admin/api/settings \
     -H "Content-Type: application/json" \
     -d "{\"priority_peer_ip\":\"$MY_TS_IP\"}"
curl -u admin:microlink -X POST http://$CHIP/admin/api/restart
```

Set `priority_peer_ip` to the machine host — it gets the tightened 3 s
disco heartbeat and the ≤10 s direct→DERP failover.

### Tunnel latency ~100–200 ms instead of ~5–25 ms

The chip is riding a DERP relay instead of the direct UDP path:
1. `derp_only=1` in `/state.json` → post-crash ladder; wait ~2 min for
   the auto-restart, or restart manually.
2. Direct path never formed → NAT/firewall/different LAN between chip
   and peer. DERP works, just slower; the pstop link stays up either way.

> **Do not use `pstop_rtt_ms` to judge direct-vs-relay.** It is
> counter-lag-inflated and can read ~100 ms on a ~10 ms direct path
> (`main.c:619`). Judge the path with `tailscale status` (`direct <ip>` vs
> `relay <region>`) from the machine, or the chip's `/admin/api/monitor`
> `pp_has_direct` / `pp_best_ip` (the endpoint the chip picked for the machine).

### Same-LAN chip and machine still relay through DERP

Both on one LAN but the pstop link won't go direct. Check, in order:
1. **Chip advertises its LAN endpoint** — `/admin/api/monitor` `advert_lan_ip`
   should equal the chip's active-uplink IP (not 0). Fixed 2026-07-22 so eth/USB
   units advertise it, not just WiFi.
2. **Machine host isn't hijacking the chip's LAN IP into Tailscale** — on the
   machine, `ip route get <chip-LAN-IP>` must NOT return `dev tailscale0`. If a
   tailnet subnet router advertises the LAN, add `sudo ip rule add to
   <chip-LAN-IP>/32 lookup main priority 1000` (see the subnet-route caveat
   below). This was the dominant real blocker in bench testing.
3. **AP/client isolation** — some APs block client-to-client traffic; no
   firmware change overcomes it, only DERP will work there.

### Machine stays in STOP after the switch was released

Expected if the STOP episode was shorter than the arming policy minimum
(`min_stop_ms = 500` in `host/machine.toml`) — the runner logs
"ANOMALY: arming VETOED". Re-arm with a deliberate press-and-hold
(≥0.5 s) then release. Also expected after any link outage ≥1 s: the
machine latches NEED_STOP and requires a fresh press→release.

### `pstop_mismatch` climbing / ring purple

The two cores' encodings disagree — check the loop diagnostics
(`e_hi*/e_lo*`): one channel open (broken wire, connector) or sense
stuck high while the other is healthy. A genuine both-pole press shows
RED (agreed STOP), not purple.

### Machine log shows "ANOMALY: counter gap"

The chip increments its counter every tick but only sends on lockstep
agreement, so a gap means messages were dropped OR withheld
(mismatch/boot-priming hold) — not necessarily wire loss. Persistent
gaps + `max_lost_messages` (10) exceeded → MSG_LOST STOP.

### Tailscale reachable from some hosts, not the operator laptop

A tailnet subnet router advertising the robot's LAN steals traffic to
the chip's LAN IP (policy rule beats the connected route). Bench
workaround: an `ip rule` preference for the connected route. Subnet-router
interaction and fleet isolation are covered in `TAILSCALE_ISOLATION.md`.

### Reachable over USB/LAN but not via its Tailscale IP (fleet's remote buttons fail)

The unit checks in fine but can't be reached *inbound* over Tailscale — its
single DERP connection isn't homed on the fleet's region, so remote peers relay
to the wrong region and packets never arrive. Critical for NAT'd units (USB
tether / symmetric NAT) where DERP relay is the only inbound path (no direct
hole-punch). Diagnose via `/admin/api/monitor`:

- `fleet_peer_region` — the fleet's DERP region as learned by the chip (2 = sfo).
- `derp_home_region` / `priority_peer_region` — where DERP is homed / the region
  the chip decided on. Both should equal `fleet_peer_region` (or the priority
  peer's region, which wins when known).
- `derp_connected` — must be true for relay reachability.
- `rehome_calls` / `rehome_applied` — did the re-home run. **`rehome_calls = 0`
  means the WG task isn't running at all** → see the boot-safety pause below.

A wrong region self-heals within a few seconds (the firmware re-homes to the
fleet's region and re-advertises it in every MapRequest). If it doesn't recover,
OTA the latest build.

### Tailscale paused / `wg_mgr` task Suspended (boot-safety)

Symptom: `wg_paused=1`, `derp_paused=1`, `ts_boot_en=0`, DERP down, `rehome_calls=0`,
and the `wg_mgr` task shows **Suspended** in `/admin/api/monitor`. The boot-safety
ladder pauses Tailscale after repeated crash-class boots (`boot_count>=2`) to
escape a perceived boot-loop — easily tripped on the bench by aggressive OTA +
hard power-cycling. Recover:

1. `curl -u admin:PW -X POST http://$CHIP/api/ts_boot` — confirm it returns `"ts_boot_en":1`.
2. `curl -u admin:PW -X POST http://$CHIP/admin/api/restart` — a **clean** SW
   restart. Do **not** power-cycle: a hard cut is a crash-class boot and re-arms
   the pause.

For a wedged USB-NCM tether (chip enumerates as `303a:4001` but takes no IP on
`10.42.0.0/24`), power-cycle it via the LC relay: `usb-switch cycle`.

> Note: on these units `/api/last_log` (RTC ring) is unreliable/previous-boot and
> the USB-CDC console emits nothing — debug with the `/admin/api/monitor`
> **counters**, not logs.

### OTA upload times out / hangs

- Don't OTA with a flapping interface enabled — route churn kills the
  TCP session (disable WiFi first if it's bouncing).
- WiFi RSSI below −80 dBm makes large uploads flaky regardless.
- Check `heap_min_int` and `/api/last_log` (chip may be crashing during
  the upload).
- A partial upload never half-applies: the bootloader accepts the full
  image atomically or rejects it.

## Field escalation ladder

1. Soft reboot: `curl -u admin:microlink -X POST http://$CHIP/admin/api/restart`
2. Toggle an interface (each returns JSON, supervisor re-picks the route
   within 1 s): `curl -X POST http://$CHIP/api/iface/eth` (`/wifi`, `/usb`)
3. Toggle Tailscale-at-boot: `curl -X POST http://$CHIP/api/ts_boot`
   (NVS flag, takes effect next reboot)
4. OTA the latest known-good `pstop_remote.bin` (see
   `RECOVERY_PLAYBOOK.md`)
5. Physical reflash: hold BOOT, tap RST → ROM download mode →
   `idf.py -p /dev/ttyACM0 flash`. Last resort; NVS (settings, peer,
   history) survives a reflash because the partition offsets are stable.
