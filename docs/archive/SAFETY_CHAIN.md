# Safety chain — how the chip recovers from any failure mode

The chip is a node in a safety system. The premise of the safety chain
is: **any way the firmware can wedge or crash, it must reboot to a
known-good state without anyone pressing a button.** The user has
walked away. The chip is on its own.

The chain is four layers, each catching what the layer above missed.

## Layer 1 — Task Watchdog Timer (TWDT)

`CONFIG_ESP_TASK_WDT_TIMEOUT_S = 30` in `sdkconfig.defaults`. The
TWDT is fed implicitly by the IDLE tasks on both cores. If either
IDLE task is starved for 30 s straight, panic + reboot.

This catches: kernel-level infinite loops, priority inversion that
locks out the IDLE task, tasks that grab the scheduler lock and don't
let go.

Doesn't catch: lwIP wedge (TCPIP task is starved but IDLE is fine),
application-level liveness loss.

## Layer 2 — Network liveness watchdog

`net_liveness_task` in `main.c`. The check:

```
if (last_ml_state == ML_STATE_CONNECTED && time_since_ping > 60 s)
    && uptime > 90 s grace:
        abort();
```

`time_since_ping` is updated by an internal "INET_PING" task that
pings `8.8.8.8` over the WG netif every 5 s. Once Tailscale is up,
INET_PING traffic should always flow; if it stops for 60 s while we
think we're CONNECTED, lwIP has wedged and we need to reboot.

`abort()` triggers a PANIC reset which is classified as a CRASH (see
Layer 3).

This catches: the microlink wedge described in `MICROLINK_WEDGE.md`.
We confirmed bench-side that this layer fires reliably 150–180 s after
boot when the wedge hits.

Doesn't catch: pre-Tailscale wedges (we're not "CONNECTED" yet so the
check is skipped). For that we rely on Layer 1 and the 90 s grace.

## Layer 3 — Boot counter + OTA invalidate

In `app_main()`, we increment `boot_count` in NVS *only on
crash-class reset reasons*: `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT`,
`BROWNOUT`. Clean resets (`SW`, `POWERON`, `DEEPSLEEP`) reset the
counter to 0.

```c
uint32_t boot_count = nvs_get_u32(USR_NVS_KEY_BOOT_COUNT);
if (crash_reset) boot_count++;
else             boot_count = 0;
nvs_set_u32(..., boot_count);

if (boot_count > 3) {
    ESP_LOGE(TAG, "Crash loop detected — rolling back");
    esp_ota_mark_app_invalid_rollback_and_reboot();
}
```

Three crashes in a row → mark current OTA slot invalid → bootloader
boots the previous-known-good image on reboot. This is the layer that
saves us from shipping a botched OTA build.

This is **critical** to the no-buttons promise. If we ship a build
that wedges in pre-Tailscale init (Layer 2 doesn't catch it because
grace period hasn't elapsed), Layer 3 still rolls us back after 3
power-on attempts.

## Layer 4 — Bootloader rollback (PENDING_VERIFY)

`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` in `sdkconfig.defaults`.
After OTA, the new image starts in `PENDING_VERIFY` state. If it
reboots before marking itself valid (`esp_ota_mark_app_valid_cancel_rollback`),
the bootloader rolls back to the previous slot on next boot.

In `main.c`, we call `esp_ota_mark_app_valid_cancel_rollback()` at the
end of `app_main()`, immediately before the main loop. This is the
"I survived init, my image is good" signal. If init crashes before we
reach that line, we never mark valid, and the bootloader saves us.

Earlier we tried a 5-minute validation timer (so Tailscale had to come
up before we marked valid). We removed that because it conflicted with
the USB toggle: a user disabling USB during the window would prevent
mark-valid even though the build was fine. Now we rely on Layer 3 for
crash-loop detection of post-init failures.

## How the layers compose

| Failure mode | Catches layer |
|---|---|
| Hard CPU hang | 1 (TWDT) |
| Pre-init crash (e.g. nvs_init fault) | 4 (PENDING_VERIFY) |
| Init succeeds, but app crashes 1 s later | 4 (PENDING_VERIFY) on first reboot, 3 if recurrent |
| Tailscale wedge (lwIP stops) | 2 (liveness) → 3 (counter) if recurrent |
| Slow memory leak | eventually 1 (heap exhaust → panic) → 3 |
| Build is OK but config is wrong (e.g. bad WG key) | 2 if it manifests as no liveness; otherwise users must change config |

## What's deliberately NOT in the chain

- **No automatic factory reset.** If we boot from a corrupt NVS, we
  log and continue with defaults. Wiping user config on a perceived
  fault would be more dangerous than the fault — the chip would lose
  Tailscale auth and stop being reachable.
- **No remote-trigger reboots.** The admin UI has a reboot button but
  no auto-reboot-from-cloud. Same reason: if the cloud is wrong, we
  shouldn't honor it.

## Verification checklist for any future change to the chain

When touching layers 2/3/4, re-run these tests before declaring done:

1. **Crash-loop rollback**: deploy a build that calls `abort()` in
   `app_main`. Verify 3 boots → counter triggers `esp_ota_mark_app_invalid_rollback_and_reboot`
   → boots back into known-good.
2. **PENDING_VERIFY rollback**: deploy a build that calls `abort()`
   *before* `esp_ota_mark_app_valid_cancel_rollback()` is reached.
   Verify 1 boot → reset → bootloader rolls back.
3. **Liveness fires under wedge**: induce wedge (start Tailscale, wait
   60–90 s). Verify `abort()` log line appears 150–180 s post-boot,
   chip reboots cleanly.
4. **Counter resets on clean reboot**: deploy a build, let it run 10
   s, soft-reboot via admin UI. Verify `boot_count` is 0 on next boot.
5. **NVS toggle persists**: toggle USB enable via admin UI, soft-reboot,
   verify state preserved. (Tests NVS write/read paths the chain
   depends on.)

All five should pass before any safety-chain-modifying change ships.
