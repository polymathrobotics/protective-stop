# Safety chain — how the remote recovers from any failure mode

The remote is a node in a protective-stop system. The premise of the
safety chain is: **any way the firmware can wedge or crash, it must
reboot to a known-good state without anyone pressing a button.** The
operator has walked away; the chip is on its own. (While the chip is
down the machine sees silence and holds STOP — the chain is about
restoring availability, never about masking a fault.)

Implementation: `firmware/components/dcs_support/src/dcs_safety.c`,
`dcs_net_liveness.c`, `dcs_nvs.c`, `panic_log.c`. Constants:
`dcs_internal.h` (`DCS_SAFETY_MAX_RAPID_BOOTS = 3`,
`DCS_SAFETY_CLEAR_AFTER_MS = 120000`).

## Layer 1 — Task Watchdog Timer (TWDT)

`CONFIG_ESP_TASK_WDT_TIMEOUT_S = 30` in `firmware/sdkconfig.defaults`.
The TWDT is fed by the IDLE tasks on both cores plus an explicitly
subscribed `app_heartbeat_task` (1 Hz feed). 30 s of starvation →
panic + reboot (`reset_reason = TASK_WDT`, crash-class).

Catches: kernel-level infinite loops, priority inversion that locks out
IDLE, scheduler-lock hangs. Doesn't catch: a deadlocked lwIP stack
(TCPIP task starved while IDLE is fine) — that's Layer 2.

## Layer 2 — Network liveness watchdog

`net_liveness_task` (`dcs_net_liveness.c`). Pings the **active uplink's
own gateway** every 5 s (not an internet host — a PoE board on an
isolated LAN must not false-abort, and the WG overlay netif's "gateway"
never answers ICMP). The target is re-resolved when the net supervisor
fails over between Ethernet/USB/WiFi.

Arming is deliberately conservative — the watchdog can only fire for a
link that WAS healthy and went silent:

- the CURRENT uplink's gateway must have replied since the last
  failover (re-armed per interface, never sticky), AND
- pstop must have bonded at least once, AND
- before aborting, a cross-check: a fresh pstop reply within the window
  is hard proof lwIP is alive (gateways routinely deprioritise ICMP), so
  only gateway-silent AND pstop-silent for
  `NET_LIVENESS_TIMEOUT_MS = 180 s` counts as a genuine wedge.

On a genuine wedge it calls `abort()` (so the panic backtrace is
captured) — but first stamps an RTC_NOINIT flag
(`dcs_safety_mark_liveness_abort`) so the next boot does **NOT** count
this network-class reboot toward the rollback ladder. A bad network must
never downgrade a good image.

## Layer 3 — Persistent crash counter + OTA rollback

`dcs_safety_account_boot()` runs first thing in `dcs_support_init()`:

- `boot_count` (NVS `dcs_app/boot_cnt`) increments only on
  **genuine-code-fault** reset reasons: `PANIC` (unless it was a marked
  liveness abort), `INT_WDT`, `TASK_WDT`, `WDT`.
- Excluded: clean resets (`POWERON`/`SW`), deep-sleep, `BROWNOUT` (power
  quality is not a firmware fault), and liveness aborts.
- Every boot's reset reason is also pushed into a 16-entry NVS ring
  (`rst_hist`, shown in `/state.json`) for crash-pattern diagnosis.
- `boot_count > 3` → `esp_ota_mark_app_invalid_rollback_and_reboot()` —
  the bootloader boots the previous known-good image. **Never-brick
  guard:** if the OTHER partition is also marked INVALID/ABORTED, the
  rollback is refused (it would leave no bootable image); the counter is
  cleared and the chip just reboots and keeps trying until a new OTA
  replaces it.

The counter also drives a graded degradation ladder for the boot it's
observed on (NVS untouched):

| boot_count at boot | Mode for this boot |
|---|---|
| 0 | full Tailscale (direct UDP + DISCO + STUN) |
| 1 | **DERP-only** — DISCO/STUN suppressed (`derp_only=1` in `/state.json`) |
| ≥ 2 | **Tailscale paused** — `ml_wg_mgr` suspended, DERP TX paused |

Age-out: `bc_clear_task` clears the counter after **120 s** of healthy
uptime; if the boot was DERP-only, it also triggers a clean
`esp_restart()` to retry the direct-UDP fast path. So a single wedge
costs ~2 min of relay-level latency, then full speed returns
automatically.

## Layer 4 — Bootloader rollback (PENDING_VERIFY)

`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. After OTA, the new image
starts in `PENDING_VERIFY`. `dcs_support_finalize()` — called at the end
of `app_main()`, only after the lockstep safety tasks have spawned —
calls `esp_ota_mark_app_valid_cancel_rollback()`. If init crashes before
that line, the image is never marked valid and the bootloader rolls back
to the previous slot on the next reset. Marking valid deliberately does
NOT clear `boot_count`, so the crash-loop signal survives until the
Layer-3 age-out.

## Crash forensics (not a recovery layer, but part of the chain)

- `panic_log.c`: a 7 KiB RTC_NOINIT ring captures the tail of the
  previous boot's ESP_LOG output — survives crash/WDT/`esp_restart()`
  (not a power cycle). Served at `GET /api/last_log`.
- `rst_hist` (16 most recent reset reasons) + `boot_count` +
  `reset_reason` + `ota_state` in `/state.json`.

## How the layers compose

| Failure mode | Caught by |
|---|---|
| Hard CPU hang / starved IDLE | 1 (TWDT) → 3 if recurrent |
| Pre-init crash (e.g. nvs_init fault) | 4 (PENDING_VERIFY) |
| Init succeeds, app crashes shortly after | 4 on the first post-OTA boot; 3 if recurrent |
| lwIP wedge (net dead, scheduler fine) | 2 (liveness abort — recovers, never rolls back) |
| Slow memory leak → panic | 1/heap panic → 3 |
| Bad config (e.g. wrong WG key) | none — chip stays reachable in degraded mode; fix the config |

## Deliberately NOT in the chain

- **No automatic factory reset.** Corrupt NVS → log and continue with
  defaults. Wiping config on a perceived fault would cost Tailscale auth
  and reachability — worse than the fault.
- **No remote-triggered auto-reboot.** The admin UI has a manual restart
  (`POST /admin/api/restart`), but nothing cloud-driven can reboot the
  chip on its own.

## Verification checklist for any change to the chain

1. **Crash-loop rollback:** deploy a build that `abort()`s in steady
   state. Verify the 4th crash boot triggers rollback into known-good.
2. **PENDING_VERIFY rollback:** deploy a build that `abort()`s *before*
   `dcs_support_finalize()`. Verify one boot → bootloader rolls back.
3. **Liveness abort is not a rollback trigger:** induce a wedge (or call
   the abort path); verify the next boot logs "NOT counted toward
   rollback" and `boot_count` doesn't climb.
4. **Counter age-out:** after a single crash boot, verify `boot_count`
   returns to 0 after ~120 s healthy uptime (and a DERP-only boot
   auto-restarts to full Tailscale).
5. **NVS settings persist:** toggle USB enable via the admin UI,
   soft-reboot, verify the state survived (exercises the NVS paths the
   chain depends on).
