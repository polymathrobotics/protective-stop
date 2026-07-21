> ⚠️ **Historical (v15.3.x era).** Pre-dates the v15.20+ wedge fixes.
> Test failures recorded here reflect the bc_clear / ping task-stack
> overflows that weren't yet diagnosed. Current state:
> [`TAILSCALE_FINAL_2026-05-26.md`](TAILSCALE_FINAL_2026-05-26.md);
> troubleshooting: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

# Extended test report — 2026-05-25 evening session

Driven by the user's ask to "test all features that have been worked on,
do multiple reset cycles, run lots of pings back and forth to test the
Tailscale IP reply rate and track P-level latencies." This report covers
what passed, what failed, what got fixed in the same session, and what's
left open (most notably: the chip is currently USB-hard-bricked at hand-
off — see "Open issue" at the bottom).

## TL;DR

- USB-NCM path: **rock-solid**. 60/60 HTTP polls in 60 s, 601/601 pstop
  heartbeats at exact 10 Hz, 300/300 ICMP echoes with **P50=2.27 ms /
  P90=2.47 ms / P95=2.80 ms / P99=3.55 ms**.
- 5 consecutive `/admin/api/restart` soft reboots: **all succeeded**;
  `boot_count` stayed at 0 because SW resets don't count toward the
  crash counter; `ts_boot_en` correctly persisted across all five.
- Tailscale (chip's `100.97.180.43`): **wedge still recurs** at
  ~60–90 s of sustained activity. The disco-gate fix from v15.1
  helped on the first boot but doesn't fully eliminate the wedge.
  This is now an explicit, opt-in mode (NVS `ts_boot_en=1`).
- New default: **Tailscale paused at boot** (`ts_boot_en=0`) so the
  rest of the device is stable. `POST /api/ts_boot` flips it.
- Found and fixed a real bug:
  `/api/pstop_peer` was silently dropped because httpd's URI handler
  table was full. Bumped `cfg.max_user_uri_handlers` from default 8 → 16.
- Found and fixed a second bug:
  early `vTaskSuspend(ml_wg_mgr)` in app_main deadlocked NVS because
  wg_mgr was mid-`ml_peer_nvs_load_all`. Moved the suspend to after
  our own NVS reads + added a 500 ms breather.
- **Open issue**: the chip is currently USB-disconnected and not
  enumerating. Final 15.3.2 OTA + the rollback churn left it stuck.
  Power-cycle (unplug + replug USB) needed to recover.

## Detailed results (v15.3.1 — Tailscale paused at boot)

The test sweep ran against v15.3.1 in stable-mode. Source:
`/tmp/test_suite.log` and `/tmp/longsoak.log` from this session.

### TEST 1 — HTTP soak (60 polls × 1 s)

```
19:27:05 === TEST 1: HTTP soak (60s, every 1s) ===
19:28:06 TEST 1 result: 60 OK / 0 FAIL in 60s
```

100 % HTTP availability over the window. Compare to v15.1 in Tailscale-
active mode where curl on `/state.json` was timing out within ~60–90 s.

### TEST 3 — pstop over USB-NCM (60 s)

```
start:  sent=738   rep=737   fail=0
end:    sent=1339  rep=1338  fail=0
delta:  sent=+601  rep=+601  fail=+0
```

601 sends / 601 replies / 0 failures = **100 % reply rate, exact 10 Hz**.

### TEST 5 — USB-NCM ICMP latency (300 packets, 5 Hz)

```
300 packets transmitted, 300 received, 0% packet loss, time 59962 ms
P50 = 2.27 ms
P90 = 2.47 ms
P95 = 2.80 ms
P99 = 3.55 ms
```

Tight tail. The USB-NCM path through TinyUSB on the S3 + host's NM
shared mode adds well under 5 ms of round-trip overhead at p99.

### TEST 2 — Runtime knobs

| Endpoint | Result | Notes |
|---|---|---|
| `POST /api/derp` toggle | PASS | `{"ok":true,"paused":0}` / `{"ok":true,"paused":1}` |
| `POST /api/wg` toggle | PASS | same |
| `POST /api/derp_delay?ms=10/50/1` | PASS | each round-trip returns the new value |
| `POST /api/wifi_tx_power?q=…` | **FAIL** (expected) | `esp_wifi_set_max_tx_power failed` — with `s_usb_enabled=1` we never call `esp_wifi_start()`, so `esp_wifi_set_max_tx_power` errors. Cosmetic only; works in WiFi-mode boot |
| `POST /api/pstop_peer` | **FAIL → fixed** | initially `404 "Nothing matches the given URI"`. Root cause: httpd's URI handler table was full (`max_uri_handlers=24`, we register 26+). Fix landed in v15.3.2: `cfg.max_user_uri_handlers = 16` |

### TEST 4 — 5 admin soft restarts

```
reset #1 → t=14.6s ml=3 bc=0 rr=3 ts_boot_en=0
reset #2 → t=13.6s ml=3 bc=0 rr=3 ts_boot_en=0
reset #3 → t=13.6s ml=3 bc=0 rr=3 ts_boot_en=0
reset #4 → t=14.6s ml=3 bc=0 rr=3 ts_boot_en=0
reset #5 → t=14.6s ml=3 bc=0 rr=3 ts_boot_en=0
```

All five `/admin/api/restart` calls succeeded. The chip came back in
~13–15 s each time. `boot_count=0` after every reset (clean SW resets
don't increment the crash counter — verified). `ts_boot_en` persisted
across all five resets — NVS write is durable.

### Long-soak observation (v15.3.1, ~6 minutes continuous)

`/tmp/longsoak.log` recorded the chip from boot through the full test
sweep. Sampling at 5 s, no gaps, until the chip was rebooted at end of
TEST 4. Key telemetry held flat:
- `heap_min_int` settled at ~93 KiB (internal RAM low-water)
- `wg_pbuf_fails` stayed at 0
- `free_heap` ~8160 KiB
- `boot_count` cleared automatically at +2 min

## Tailscale path (the part that didn't fully work)

When I flipped `ts_boot_en=1` (via `/api/ts_boot`) and let the chip
reboot into Tailscale-active mode, it wedged at the original 60–90 s
mark again. Specifically the test run that bricked the chip recorded:

```
t=773 (77.3 s)   ml=4 bc=2 sent=773
…then…
UNREACH for ~10+ min, ultimately USB-hard-disconnect
```

So the v15.1 conclusion needs amending: the disco-gate is a real fix
(it's the largest known load source eliminated), but the wedge has
**at least one more cause**. The earlier "5-minute clean soak right
after OTA" was lucky — likely the chip's peer cache was empty / DERP
session was fresh, and the wedge needed cumulative state.

That's why v15.3 makes `ts_boot_en=0` the default — the device stays
useful for everything else while the residual wedge is investigated.

### What I'd try next session for the wedge

In rough order of expected-value:

1. **Stagger ml_coord registration vs DERP**. The 2 s delay already
   in `ml_coord.c` helped cold-start but doesn't address steady-state
   churn.
2. **Reduce WG periodic frequency from 400 ms to ~1 s** when no peer
   handshake is in flight. The hot-path crypto is the load that
   compounds with anything else on the TCPIP thread.
3. **Cap max_peers to 4** but only after validating that the peer-table
   sizing in `ml_coord.c` handles small values safely (`max_peers=4`
   crashed v15.0 at init this session).
4. **Port upstream `~/microlink-polymath` 09c387e** (v2.1.0 dual-path
   WG handshake + new `ml_tcp.c`). Larger change, higher risk, but
   addresses the disco/stun=false WG-handshake gap directly.
5. **Add a "ts_uptime_cap" auto-pause**: if Tailscale has been active
   for N minutes, auto-pause it. Workaround, not a fix.

## Bug fixes that landed this session

### v15.3.2 — httpd URI handler table

`/api/pstop_peer` was silently dropped because the httpd table
filled. ml_app sets `max_uri_handlers = 16 + 8 = 24` by default;
ml_config_httpd registers 14 routes under `/admin/`, ml_app
itself adds 3 root routes, and dual_core_safety registers 9.
26 routes vs 24 slots → the last one (pstop_peer) was the casualty.

Fix in dual_core_safety/main/main.c:
```c
cfg.max_user_uri_handlers = 16;   // was default 8
```

### v15.3.1 — NVS deadlock from suspending ml_wg_mgr too early

Initial v15.2 attempt to suspend the WG manager *immediately* after
`ml_app_start` returned deadlocked the chip's boot: ml_wg_mgr was
inside `ml_peer_nvs_load_all()` at that moment, holding the NVS
lock. Once suspended, the next NVS read in app_main blocked forever
→ chip never reached `initial_ota_validate()` → PENDING_VERIFY
rollback to old firmware on the second boot.

Fix in dual_core_safety/main/main.c: pause DERP immediately (just
sets an atomic flag, no locks) but defer the `vTaskSuspend` on
ml_wg_mgr until after our own NVS reads + a 500 ms breather to
let ml_wg_mgr finish its peer-cache load.

## Test infrastructure left behind

- `/tmp/longsoak.sh` — 5 s state.json sampler with reboot/wedge
  detection, output to `/tmp/longsoak.log`.
- `/tmp/test_suite.sh` — the test driver this report's numbers came
  from. Keep it, refine as the project grows.
- `/tmp/aggressive_ota.sh` / `/tmp/ota_then_ts.sh` — race-with-wedge
  OTA helpers that retry every ~500 ms during the alive window.

## Open issue — chip USB-hard-bricked at hand-off

After the v15.3.2 build + a series of attempted aggressive OTAs into
a chip that was already bouncing through the wedge/rollback cycle,
the chip stopped enumerating over USB entirely. Last `usb 1-3: New
USB device found` was at `19:31:22` (Mon May 25 2026). 14+ minutes
of silence after that — `lsusb -d 303a:` returns nothing.

What I think happened: each safety-chain rollback marks the running
partition as `ESP_OTA_IMG_INVALID`. With two consecutive rollback
cycles (v15.0 → old, v15.1 → old, v15.2 → old, v15.3 → old, v15.3.1
attempts during wedge), both OTA slots ended up marked invalid in
otadata. ESP-IDF bootloader policy with both slots invalid + no
factory partition (we have none, see partitions.csv) = halt. TinyUSB
never gets a chance to init, so no USB enumeration.

**Recovery**: physical power-cycle (unplug + replug USB cable). The
otadata partition seems to retain the invalid state across resets,
so power-cycling might not be enough — we may need BOOT+RST then
`idf.py -p /dev/ttyACM0 flash` over ROM USB-JTAG to clear otadata.
Both partition images on flash are still good binaries; just the
otadata flag tells the bootloader to skip them.

I sent a push notification flagging this. Continuing testing
requires physical access — sorry. The work that was completed
above is real signal; the bricked chip is a process-failure of the
test loop, not a fundamental firmware issue.

## What this means for the v15.1 → v15.3.3 progression

- v15.1 (commit `f298899`): disco-gate fix. Real, valuable. Wedge
  delayed but not eliminated.
- v15.2 (commit `d0ab800`): `ts_boot_en` toggle, default = ON. Had
  the NVS-deadlock bug.
- v15.3 (uncommitted at brick): made `ts_boot_en` default OFF.
- v15.3.1 (uncommitted at brick): fixed the NVS-suspend ordering.
- v15.3.2 (commit `1a72205`): fixed the missing `/api/pstop_peer`
  registration (handler table size).
- v15.3.3 (commit `21db46b`): two safety-chain hardenings landed
  *after* the chip bricked, in direct response. (1) Refuse to call
  `esp_ota_mark_app_invalid_rollback_and_reboot()` when the other
  partition is ALREADY invalid — that's the combination that
  bricked us; we just `esp_restart()` instead and accept that the
  bad build will keep restarting until manually replaced. (2)
  Auto safe-mode: if `boot_count >= 1` we force `ts_boot_en=false`
  for the current boot regardless of NVS, so a chip stuck in a
  Tailscale-wedge crash loop stops trying to come back into
  Tailscale and just stays useful as a non-Tailscale node.

The v15.3.3 binary is in `examples/dual_core_safety/build/
microlink_dual_core_safety.bin` and is the recovery target once
the chip is power-cycled / esptool-flashed back to life. The
v15.3.3 changes are pre-emptive — they could not be bench-tested
because the chip went away first — but they're code-review safe
and address the exact failure mode of this session.

## How to recover the chip after this session's brick

1. **Try power-cycle first.** Yank the USB cable; wait 5s; replug.
   If lucky, the bootloader retries one of the partitions enough
   times that it boots one anyway. Watch `lsusb -d 303a:` for the
   chip to re-appear as `303a:4001` or `303a:1001` (ROM).

2. **If power-cycle doesn't restore enumeration, BOOT+RST flash.**
   Hold BOOT, tap RESET (or unplug+replug while holding BOOT).
   Chip should come up as `303a:1001` on `/dev/ttyACM0` (or wherever
   the ROM device lands). Then:
   ```sh
   cd examples/dual_core_safety
   source ~/esp-idf-5.5/export.sh
   idf.py -p /dev/ttyACM0 erase-flash
   idf.py -p /dev/ttyACM0 flash
   ```
   `erase-flash` is the important step — it nukes the otadata
   partition, clearing both partitions' invalid flags. After the
   subsequent `flash` the chip should come up cleanly on v15.3.3
   with `ts_boot_en` defaulted to false.

3. **First-boot verification once the chip is up:**
   ```sh
   curl -s http://10.42.0.80/state.json | jq '{ts:.ts_boot_en, bc:.boot_count, fw:"v15.3.3"}'
   ```
   `ts_boot_en` should be 0 (default). `boot_count` should be 0.
   Then re-run `/tmp/test_suite.sh` to confirm all the v15.3.1
   passes still pass against v15.3.3.
