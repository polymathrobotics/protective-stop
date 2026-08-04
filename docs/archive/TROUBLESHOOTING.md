# Troubleshooting — dual_core_safety

Field guide for diagnosing a sick chip. Written against v15.26 (2026-05-26).

---

## First contact: read `/state.json`

```sh
# WiFi mode (default at boot):
curl http://192.168.107.131/state.json | python3 -m json.tool

# USB-NCM mode (host on NetworkManager esp-tether share):
curl http://10.42.0.80/state.json | python3 -m json.tool
```

Look at these fields in this order:

| Field | Healthy | What it means if not |
|---|---|---|
| `ml_state` | **4** (CONNECTED) | 0/1/2/3 = Tailscale not up; check `ts_boot_en`, `wg_paused`, `derp_paused` |
| `boot_count` | **0** | 1 = previous boot crashed (DERP-only mode); 2 = TS auto-paused; 4+ = bootloader rollback imminent |
| `reset_reason` | **3** (SW restart) | 4 (PANIC), 5/6/7 (various WDTs), 11 (brownout) — see below |
| `ts_boot_en` | **1** | 0 = Tailscale intentionally paused (NVS bit) or auto-paused by ladder |
| `derp_only` | **0** | 1 = chip in DERP-only fallback (DISCO/STUN dropped) after a prior crash |
| `wg_paused` / `derp_paused` | **0** / 0 | 1 = the v15.19 ladder paused these because bc≥1 (DERP-only) or bc≥2 (full pause) |
| `usb_enabled` | depends on deployment | 1 = USB-NCM tether path, 0 = WiFi STA path |
| `heap_min_int` | **≥ 30 KiB** typical | Drops to ~6 KiB in USB-NCM mode (extra TinyUSB buffers); below 1 KiB = trouble |
| `wg_pbuf_fails` | **0** | Increments = lwIP couldn't satisfy a `pbuf_alloc(PBUF_RAM)` for a WG packet (silent drop) |
| `wg_dedup_drops` | climbs slowly | Counts WG data packets dropped as duplicates from Tailscale dual-pathing; high rate = host's tailscaled is hedging direct + DERP simultaneously |
| `pstop_sent` / `pstop_replies` | grow together at 10 Hz | Reply lag = network path issue; `pstop_send_fail` increments = chip can't TX |
| `rssi` | -50 to -75 dBm | Below -80 = poor WiFi link, expect drops |

## What `reset_reason` tells you

| Code | Name | Meaning | Look at |
|---|---|---|---|
| 1 | POWERON | Cold boot (power applied) | nothing — normal |
| 3 | SW | Clean `esp_restart()` (OTA, admin restart, auto-recovery) | nothing — normal |
| 4 | PANIC | Stack overflow, hard fault, abort() | UART0 serial log for the banner |
| 5 | INT_WDT | Interrupt watchdog (interrupts disabled > 300 ms) | Look for long critical sections, ISR-heavy code |
| 6 | TASK_WDT | A subscribed task didn't feed TWDT in 30 s | `app_heartbeat_task` stuck; check task list |
| 11 | BROWNOUT | Supply voltage dipped below threshold | power supply / USB cable |

`reset_reason` is sticky from the last boot. If it shows 4 (PANIC)
but `boot_count = 0`, the 2-min `clear_boot_count_task` has already
cleared the counter — chip is currently healthy.

---

## Get the panic banner via UART0 serial debug

Without serial, the chip looks like it silently reboots after a
crash — every backtrace dies on UART0 with no listener. The CP2102
USB-UART adapter wired to the chip's UART0 pins surfaces them.

```sh
# Serial device — typically /dev/ttyUSB0 or /dev/ttyUSB1 (CP2102):
udevadm info -q property -n /dev/ttyUSB1 | grep ID_MODEL
# ID_MODEL=CP2102_USB_to_UART_Bridge_Controller

stty -F /dev/ttyUSB1 115200 raw -echo
cat /dev/ttyUSB1 > /tmp/chip_serial.log &
```

Trigger the wedge, then look for:

```
***ERROR*** A stack overflow in task <name> has been detected.
Backtrace: 0x4037d8d5:0x3fce8910 0x4037d89d:0x3fce8930 ...
```

The task name is the first thing to read — that tells you which
`xTaskCreate` is sized too small. Decode the PCs with:

```sh
source ~/esp-idf-5.5/export.sh
xtensa-esp32s3-elf-addr2line -e build/microlink_dual_core_safety.elf 0x4037d8d5 0x4037d89d ...
```

UART0 is independent of TinyUSB on USB-OTG, so it keeps working
even when the USB net interface is down or the chip is in download
mode.

---

## What the four safety layers look like when they fire

### Layer 1 — Bootloader PENDING_VERIFY rollback

Triggered by: new OTA crashes (any reset reason) before
`esp_ota_mark_app_valid_cancel_rollback()` is called.

What you see:

```sh
curl -u admin:microlink http://CHIP/admin/api/ota/status | python3 -m json.tool
```

```json
{
  "running_partition": "ota_1",
  "rollback_occurred": true,
  "rolled_back_from": "ota_0",
  "rollback_reason": "ABORTED"
}
```

Recovery time: ~17 s from bad OTA push to chip back on the previous
good partition. **No user action required.** Verified live v15.26.

### Layer 2 — Persistent crash counter (`boot_count`)

Triggered by: CRASH-class reset reasons (PANIC / INT_WDT / TASK_WDT
/ WDT / BROWNOUT). Stored in NVS at `dcs_app/boot_cnt`.

What you see:

- `boot_count` = 1 in `/state.json` → previous boot crashed once
- `boot_count` = 2 → crashed twice
- `boot_count` ≥ 4 → bootloader rollback on next boot (via
  `esp_ota_mark_app_invalid_rollback_and_reboot()`)

Counter clears to 0 automatically after 2 min of healthy uptime
(`clear_boot_count_task`).

### Layer 3 — v15.19 graded ladder

Triggered by: `boot_count ≥ 1` at boot, scoped to this boot only
(NVS isn't touched).

| bc at boot | Mode for this boot |
|---|---|
| 0 | Direct UDP + DISCO + STUN (full Tailscale) |
| 1 | **DERP-only** — `derp_only=1` in `/state.json`; DISCO/STUN dropped |
| ≥ 2 | **Tailscale paused** — `ts_boot_en=0`, `wg_paused=1`, `derp_paused=1` |

If bc=1 and the chip stays healthy for 2 min, `clear_boot_count_task`
clears bc to 0 in NVS **and** triggers `esp_restart()` to retry the
direct-UDP fast path. So a single wedge results in ~2 min of DERP-
level latency, then full speed comes back automatically.

### Layer 4 — Task watchdog + network liveness

- `app_heartbeat_task` subscribes the app to ESP-IDF's TWDT.
  Any 30 s starvation → panic → reset_reason=TASK_WDT → counter
  bumps → eventual rollback.
- `net_liveness_task` watches INET_PING progress to 8.8.8.8. No
  reply for 60 s past a 90 s grace window → `abort()` → same
  cascade.

---

## Common scenarios

### "Tailscale ping shows `rx 0`"

Almost always: **the chip's peer allowlist doesn't include your
host's Tailscale IP**. The chip drops incoming DISCO from unknown
peers. Fix:

```sh
MY_TS_IP=$(tailscale ip -4)
curl -u admin:microlink -X POST http://CHIP/admin/api/peers/allowed \
     -H "Content-Type: application/json" \
     -d "{\"peers\":[{\"ip\":\"$MY_TS_IP\",\"label\":\"$(hostname)\"}]}"
curl -u admin:microlink -X POST http://CHIP/admin/api/settings \
     -H "Content-Type: application/json" \
     -d "{\"priority_peer_ip\":\"$MY_TS_IP\"}"
curl -u admin:microlink -X POST http://CHIP/admin/api/restart
```

Wait ~15 s for the chip to add the peer after its MapResponse,
then retry the ping. See README §First-time Tailscale setup.

### "Latency is ~165 ms instead of ~25 ms"

The chip is routing via DERP relay (Dallas region) instead of the
direct UDP path. Two possible causes:

1. **`derp_only=1` in `/state.json`** — chip is in the v15.19 ladder's
   post-crash fallback. Wait 2 min for `clear_boot_count_task` to
   trigger an auto-restart that retries direct UDP, or restart manually:
   `curl -u admin:microlink -X POST http://CHIP/admin/api/restart`.
2. **Direct UDP path never formed** — chip's `add_peer` couldn't
   reach the host directly (NAT, firewall, different LAN). Check
   that your host has a Tailscale IP on the same physical network
   as the chip (or any other path with DISCO-able UDP). DERP is
   the fallback and works; just slower.

### "Chip reboots every ~2 min with `boot_count` cycling 0→1→2→0"

If you see this on v15.26+, **something is genuinely crashing the
chip every ~2 min**. The 2-min healthy clear fires, then crash, then
2-min clear, then crash. Get the panic banner from UART0 (see above)
and look for the offending task name.

If you see this on v15.20 or earlier: it's almost certainly the
`bc_clear` task stack overflow, fixed in v15.25. Upgrade.

### "Panic during a WiFi↔USB-NCM mode transition"

Observed once pre-v15.27 (backtrace lost — no UART0 capture on the
test rig). Suspected race between in-flight WG packet processing on
`ml_wg_mgr` and `esp_restart_noos`'s task-kill sequence during the
500 ms window before reset.

v15.27 mitigation: an `esp_register_shutdown_handler` registered in
`dcs_support_init` calls `microlink_pause_derp(true)` +
`vTaskSuspend(ml_wg_mgr)` on every graceful `esp_restart()`, so the
WG pipeline is quiet before the reset fires. Stress-tested 30 rapid
WiFi↔USB cycles with a concurrent 800-packet Tailscale ping flood —
no crashes, all transitions came back `ml_state=4` within ~10 s.

If you see this panic again post-v15.27: get a UART0 trace
(`stty -F /dev/ttyUSB1 115200 raw -echo; cat /dev/ttyUSB1 > log &`)
and reproduce — the shutdown hook only fires for graceful restarts,
so a panic during init (not transition itself) would still surface
on the next boot.

### "Chip stops responding around 30-40 min into a soak"

On v15.20-v15.25: IDF `ping` task stack overflow. Fixed in v15.26
(`cfg.task_stack_size = 4096` in `start_internet_ping`). Upgrade.

On v15.26+: this should not happen. Get the panic banner.

### "OTA upload times out / hangs"

v15.20+ is safe to OTA while Tailscale is fully loaded — verified
16 s for 1.1 MB during sustained 5 Hz ping + 10 Hz pstop. If it hangs:

- Check WiFi RSSI — anything below -80 dBm makes large HTTP uploads
  flaky regardless of chip health
- Check `heap_min_int` — if it's very low, the chip might be too
  memory-pressured to accept the OTA buffer (rare)
- Check serial for a panic banner — the chip may be crashing during
  the upload and the curl is just timing out

### "`/api/state` returns but admin pages 404"

Check the URI registration count. v15.3.2 caught the bug where the
httpd handler table only had 24 slots and `/api/pstop_peer`
registered last got silently dropped. The fix is
`cfg.max_user_uri_handlers = 16` in `ml_app_start`. If you've added
more endpoints and seeing 404s on the most-recently-added one, bump
this.

---

## Field-replaceable failure modes

If you've ruled out everything above and the chip is still
misbehaving, in escalation order:

1. **Soft reboot:** `curl -u admin:microlink -X POST http://CHIP/admin/api/restart`
2. **Toggle TS off + on:** `curl -X POST http://CHIP/api/ts_boot` twice
   (it's a toggle — first call disables, second re-enables — then restart)
3. **Toggle USB ↔ WiFi mode:** `curl -X POST http://CHIP/api/usb_enable`
   (forces a reboot into the other path; rules out path-specific WiFi
   driver / TinyUSB-NCM driver issues)
4. **Clear NVS state via admin panel** (`/admin/`) → settings reset.
   Then re-do the README §First-time Tailscale setup curl sequence.
5. **OTA latest known-good binary** (this build has been bench-stable
   for 4+ hours of continuous direct-UDP load).
6. **Erase-flash + factory reflash** — requires USB cable + BOOT+RST
   to enter download mode. Last resort.

---

## Telemetry counters added in this work

| Field | Source | When you'd watch it |
|---|---|---|
| `heap_min_int` | `heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)` | OOM debugging — internal RAM is the small budget, PSRAM is large |
| `wg_pbuf_fails` | `wireguardif_pbuf_alloc_fails` | Silent WG drops; signals memory pressure on the WG netif |
| `wg_dedup_drops` | `ml_wg_mgr.c::wg_dedup_is_duplicate` | How aggressively Tailscale is dual-pathing this peer; high rate = host's tailscaled is sending direct + DERP simultaneously |
| `derp_only` | `s_derp_only_mode` in main.c | The v15.19 ladder is in DERP-only fallback |
| Per-task stack high-water | `/admin/api/monitor` → `tasks[*].stack_free` | Spot tight stacks before they overflow |

Also: `/api/last_log` serves a 7 KiB RTC_NOINIT-backed tail of the
previous boot's ESP_LOG output. Survives crash/WDT/esp_restart (not
power cycle). Not as good as serial but works remotely.
