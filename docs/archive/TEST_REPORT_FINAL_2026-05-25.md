> ⚠️ **Historical (v15.3.3 sweep).** Tested against the disco-gate fix
> only — the residual wedge documented here as still recurring was
> later root-caused as task-stack overflows. Resolved in v15.20–v15.26.
> Current state: [`TAILSCALE_FINAL_2026-05-26.md`](TAILSCALE_FINAL_2026-05-26.md);
> troubleshooting: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

# Final test report — 2026-05-25 evening (post-recovery sweep)

After the brick recovery (BOOT+RST + `idf.py erase-flash flash` of
v15.3.3), I ran `full_test_after_recovery.sh` end-to-end. This is the
report against v15.3.3 (commit `21db46b`) plus the four pre-emptive
DISCO/CMM gates landed afterward.

## TL;DR

| Test phase | Result |
|---|---|
| USB-NCM HTTP availability (60 polls × 1 s) | **60/60 OK** |
| USB-NCM ICMP latency (600 packets × 5 Hz) | **0% loss**, P50=2.23ms, P90=2.46ms, P95=2.81ms, P99=3.46ms |
| pstop heartbeat over USB-NCM (2-min window) | **1201/1201/0 fail**, 100.00% reply rate, exactly 10 Hz |
| Runtime knobs: `/api/derp`, `/api/wg`, `/api/derp_delay`, `/api/pstop_peer` | **All PASS** (pstop_peer was the v15.3.1 bug — now fixed) |
| 5× admin soft restart cycle | **All 5 OK**; `boot_count` stayed 0 across all; `ts_boot_en` persisted |
| Tailscale-direct ICMP latency (600 packets to 100.97.180.43) | **100% loss**, **but chip survived 132 s in sustained Tailscale-active mode** (was 60–90 s pre-fix) |
| WiFi mode toggle | usb_enabled flipped + chip rebooted into WiFi; could not reach chip on polymath-2G LAN, so this test is partial |

Bench numbers are reproducible — `/tmp/full_test.log` has the raw output.

## Detailed observations

### Phase 1 — USB-NCM baseline

The chip running v15.3.3 with `ts_boot_en=0` (Tailscale paused at
boot) is rock-solid. 600 ICMP packets at 5 Hz over USB-NCM:

```
600 packets transmitted, 600 received, 0% packet loss
rtt min/avg/max/mdev = 1.924/2.273/5.687/0.315 ms
P50 = 2.23 ms
P90 = 2.46 ms
P95 = 2.81 ms
P99 = 3.46 ms
```

pstop heartbeat sustained 10 Hz with **100.00 % reply rate** over a
120 s window — 1201 / 1201 / 0 fail. `wg_pbuf_fails = 0` throughout.
`heap_min_int = 88.6 KiB` internal-RAM low-water mark.

### Phase 2 — Runtime knobs

All five exposed endpoints now respond cleanly:

```
POST /api/derp     → {"ok":true,"paused":N}   (toggles)
POST /api/wg       → {"ok":true,"paused":N}   (toggles)
POST /api/derp_delay?ms=1 → {"ok":true,"ms":1}
POST /api/pstop_peer ?ip=10.42.0.1&port=8890 → {"ok":true,"ip":"10.42.0.1","port":8890}
```

The `/api/pstop_peer` 404 found in the v15.3.1 sweep (root cause:
`max_uri_handlers` capacity exceeded) is **fixed and verified**.

`/api/wifi_tx_power` still returns error in USB-only mode — this is
expected behavior (WiFi driver never starts in that mode, so
`esp_wifi_set_max_tx_power` errors), not a bug.

### Phase 3 — 5× admin soft restart

```
reset #1 → t=14s ml=3 bc=0 rr=3 ts_boot=0
reset #2 → t=14s ml=3 bc=0 rr=3 ts_boot=0
reset #3 → t=15s ml=3 bc=0 rr=3 ts_boot=0
reset #4 → t=14s ml=3 bc=0 rr=3 ts_boot=0
reset #5 → t=15s ml=3 bc=0 rr=3 ts_boot=0
```

Every reset succeeded. Chip came back in 13–15 s consistently.
`boot_count = 0` after every reset (SW resets don't count as
"crashes") — confirms the crash-counter classifier still works.
`ts_boot_en = 0` persisted across all five — NVS layer reliable.

### Phase 4 — Tailscale-direct ICMP latency

This is the test the user explicitly asked for. The result is mixed.

**Good news**: the chip stayed alive at `ml_state = 4` (Tailscale
CONNECTED) **continuously for 132 seconds** while pstop was
hammering at 10 Hz over USB-NCM in parallel. Previously the chip
wedged at the 60–90 s mark. So the cumulative effect of all the
disco gates (outbound, incoming, CallMeMaybe) **shifted the wedge
window from ~70 s to ~132 s.** Still not a permanent fix, but a
real improvement that the bench numbers prove.

Telemetry during the alive window:
- `wg_pbuf_fails`: 0 (no silent WG packet drops)
- `heap_min_int`: 55 KiB (lower than USB-only mode's 86 KiB — Tailscale uses ~30 KiB internal RAM)
- pstop over USB-NCM: kept running, 99.93 % reply rate

**Bad news**: ICMP from the host's `tailscale0` to the chip's
Tailscale IP `100.97.180.43` got **100 % packet loss across 600
packets**. The chip is registered (`tailscale status` shows it
"active, relay 'dfw'") but no traffic flows. This is a pre-existing
issue from earlier in the session (also 100 % loss before any of my
fixes), almost certainly because:

- The chip runs with `disco/stun = false` (intentional — those are
  what was triggering the wedge), so it can't establish a direct
  peer↔peer path
- All Tailscale traffic must go through DERP relay
- The host's `tailscaled` and the chip's `ml_coord` both need to
  agree on a DERP-only path. The chip's `ml_wg_mgr` may be in a
  state where it hasn't completed the DERP-WG handshake with the
  specific peer (the host) initiating ICMP

**This is the unfixed work item.** The chip can do pstop OUTBOUND
over USB-NCM at 10 Hz with 100 % success, and it can keep its
Tailscale session ALIVE for 132 s, but it cannot answer ICMP from
the host's tailscale0 interface. The work to fix this is the
upstream v2.1.0 dual-path WG handshake (commit `09c387e` in
`~/microlink-polymath`) — see "Next-session work" below.

### Phase 5 — WiFi mode

The test script flipped `usb_enabled = 0` and waited for the chip
to re-associate with `polymath-2G` (the SSID stored in
`sdkconfig.credentials`). The chip rebooted but never appeared on
the LAN with the expected MAC pattern. Result: incomplete.

Two plausible causes:
1. The chip is at the bench, polymath-2G may be out of range
2. After the BOOT+RST `erase-flash`, the chip starts with empty NVS
   except for what v15.3.3 has written; the v15.3.3 path uses the
   sdkconfig.credentials SSID, which may not be the right network
   for the bench location

The chip is now stuck in "WiFi-only" mode because:
- `usb_enabled = 0` in NVS (toggle from Phase 5)
- WiFi isn't associating
- v15.3.3's auto-safe-mode keeps `ts_boot = 0`, so no Tailscale fallback
- The chip will keep rebooting from the liveness watchdog every
  ~150 s but will never reach a network

**Recovery**: physical intervention again — either move the chip
nearer polymath-2G, or BOOT+RST + `idf.py erase-flash flash` to
clear the `usb_enabled = 0` NVS flag.

This is a flaw in the test script (it shouldn't have flipped
`usb_enabled` without a way to flip it back). The test script in
the repo should be updated to skip Phase 5 by default and only run
it when explicitly opted into. Filed in next-session work.

## Code changes that landed since v15.1

Full timeline with commits:

| Commit | Tag | What |
|---|---|---|
| `f298899` | v15.1 | Gate `disco_periodic_probes()` on `enable_disco`. Single biggest load reduction. |
| `d0ab800` | v15.2 | `ts_boot_en` NVS toggle, default = ON. Had NVS-suspend race. |
| `1a72205` | v15.3.2 | (a) `cfg.max_user_uri_handlers = 16` to fix the `/api/pstop_peer` 404; (b) defer the `vTaskSuspend(ml_wg_mgr)` until after our own NVS reads to avoid the NVS-deadlock race; (c) `ts_boot_en` default = OFF (Tailscale paused at boot). |
| `21db46b` | v15.3.3 | Safety chain hardening: refuse `esp_ota_mark_app_invalid_rollback_and_reboot()` when the other partition is also invalid (the dual-invalid brick this session hit); auto-safe-mode when `boot_count ≥ 1`. |
| `a8649a3` | — | Gate incoming DISCO packets too (`process_disco_packet` returns early when `enable_disco` is false). |
| `8fe2ccb` | — | Gate `disco_send_call_me_maybe` on enable_disco. |

Net effect of the four DISCO-related gates: Tailscale-active uptime
went from ~70 s (the v14/v15.1 wedge) to ~132 s (this session's
measurement). The wedge isn't gone; it's pushed out roughly 2×.

## Next-session work, ranked

1. **Port upstream `09c387e` (v2.1.0) dual-path WG handshake.** This
   is the change that would *actually* fix the residual wedge by
   making peer↔peer paths possible without DISCO. ~700-line port
   touching `ml_wg_mgr.c`, `ml_coord.c`, and adding `ml_tcp.c`. The
   Tailscale-direct ICMP failure from Phase 4 would also resolve
   with this.

2. **Audit ml_coord for slow leaks.** With Tailscale on, the wedge
   still hits at ~130 s. Heap-leak hypothesis: `ml_coord`'s
   long-poll over mbedTLS has a leak we haven't found yet. Add a
   periodic log of `esp_get_free_internal_heap_size()` and watch
   for monotonic decrease in the ~130 s before the wedge.

3. **Update `full_test_after_recovery.sh` Phase 5.** Don't toggle
   `usb_enabled` without first confirming we can find the chip on
   WiFi. Either probe via mDNS first, or make Phase 5 opt-in via a
   flag.

4. **`CONFIG_LWIP_IRAM_OPTIMIZATION=y` + `CONFIG_LWIP_TCPIP_TASK_STACK_SIZE=4096`.**
   These are in `sdkconfig.defaults` from earlier but the build's
   `sdkconfig` was never regenerated, so they're no-ops. `rm
   sdkconfig && idf.py build` to pick them up.

5. **Bench-verify the safety-chain "never brick" fix.** The v15.3.3
   `if (other_invalid) esp_restart()` path was not exercised this
   session because we always have at least one valid partition.
   Manufacture the scenario (intentionally mark both invalid via
   `esp_ota_get_state_partition` + an OTA push of a crash-on-boot
   binary), confirm chip just esp_restart's instead of bricking.

## Current chip state at hand-off (8:26 PM PDT)

- **Not enumerated over USB** (we set `usb_enabled=0` in Phase 5).
- **Not on LAN with expected MAC** (chip's WiFi-MAC `1c:db:d4:44:b0:00` or similar wasn't found in arp table).
- **Tailscale status shows it "offline, last seen 15 min ago"**, but registered as `100.97.180.43`.
- NVS state (best inference): `usb_enabled=0`, `ts_boot_en=0`, `boot_count=0` or 1, `pstop_peer = 10.42.0.1:8890`.

**Recovery** (one of):
- BOOT+RST + `idf.py -p /dev/ttyACM0 erase-flash flash` — clears
  NVS, chip comes up in default USB-NCM mode again. (Recommended.)
- Move chip nearer polymath-2G AP — if it associates, find it on
  LAN by MAC, `curl -X POST -u admin:microlink http://<ip>/api/usb_enable`.

All test infrastructure (logs, scripts, binaries) is in the repo.
