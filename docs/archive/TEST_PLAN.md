# Dual-Core Safety — Test Plan (W5500 / Waveshare ESP32-S3-ETH)

A regression test plan oriented around the **subtle failure modes** this build
has actually hit, not just happy-path smoke tests. Each test names the specific
bug class it guards against so a failure is diagnosable.

All telemetry is read from `GET /state.json` (served on every active interface,
no auth). Fields referenced below: `active_iface` (0 none / 1 eth / 2 usb /
3 wifi / 4 softap), `eth_en`/`wifi_en`/`usbncm_en`, `eth_link`,
`eth_ip`/`usb_ip`/`wifi_ip`, `wifi_conn`/`wifi_disc`/`rssi`, `vpn_ip`,
`boot_count`, `reset_reason` (1 POWERON, 3 SW, 4 PANIC, 6 TASK_WDT, 11 USB),
`uptime_ms`, `heap_min_int`, `free_heap`, `wg_pbuf_fails`, `rgb_cycles`,
`pstop_sent`/`_replies`/`_mismatch`/`_rebonds`/`_rtt_ms`,
`derp_delay_ms`/`derp_paused`/`wg_paused`/`derp_only`.

Bench setup: chip on a wired LAN (W5500 RJ45), a host on the same LAN running
the pstop machine (`cd host && make && ./machine_app_runner 8890`), pstop peer
pointed at the host (`curl -X POST "http://<chip>/api/pstop_peer?ip=<host>&port=8890"`).

---

## T0 — Build & flash sanity
**Catches:** build breakage, wrong IDF, partition/flash-size drift.
1. `source ~/esp/esp-idf/export.sh` (must be **v5.5.4+**), `idf.py build`.
2. `idf.py -p /dev/ttyACM1 flash` (chip's built-in USB-Serial-JTAG; **not**
   `/dev/ttyACM0` on this bench — that's an unrelated USB device).
3. **Pass:** build clean, flash verifies, `/state.json` answers within ~15 s.

## T1 — W5500 Ethernet bring-up
**Catches:** SPI pinout/clock regressions, missing W5500 Kconfig, DHCP wiring.
1. Plug RJ45 into a DHCP LAN, power on.
2. **Pass:** within ~6 s, `active_iface=1`, `eth_link=1`, `eth_ip` non-zero;
   chip pingable at that IP. RGB LED **blue**, blinking the IP's last octet.

## T2 — Interface priority & failover (Eth > USB > WiFi > SoftAP)
**Catches:** route_prio inversion, supervisor not switching, `if_key` drift.
1. With Ethernet up (`active_iface=1`, blue), **unplug** the RJ45.
2. **Pass:** chip fails over to USB-NCM (`active_iface=2`, **green**) if a USB
   host with DHCP is attached, else WiFi (`active_iface=3`, **yellow**), else
   SoftAP setup (`active_iface=4`, **red**, SSID `microlink-XXYYZZ`).
3. Re-plug Ethernet → within ~1 s `active_iface` returns to 1 (blue). The 1 Hz
   `dcs_net_supervisor` must promote the higher-priority link automatically.
   > **Runtime failover ladder (`dcs-w5500-ethernet`):** the supervisor drives the
   > whole Eth → USB → WiFi ladder on demand — USB-NCM ~5 s after Ethernet goes
   > unusable (before WiFi, so a USB host wins), WiFi ~20 s after both Ethernet
   > and USB are unusable — and drops an auto-enabled tier ~15 s after a higher
   > one returns. Ethernet keeps requesting DHCP, so a late lease (robot boot) is
   > promoted automatically. It only revokes a tier it itself enabled. See T12
   > and README "Runtime failover ladder".

## T3 — Dual-core lockstep pstop
**Catches:** encoding divergence, comparator logic, the `g_done` stale-give race.
1. Run the host machine, point the peer at it, watch `/state.json` for ~2 min.
2. **Pass:** `pstop_sent ≈ pstop_replies` and both advancing ~10/s;
   `pstop_mismatch = 0`; `pstop_rtt_ms` single/low-double digits; machine logs
   `chip alive: BOND`. **Any non-zero `pstop_mismatch` on a quiet link is a
   regression** (cores disagreed, or a stale-buffer read).

## T4 — `/state.json` robustness (buffer-overflow regression)
**Catches:** the snprintf-append `cap - n` underflow that corrupted memory and
panicked the chip when the JSON grew past the buffer.
1. Hammer it: `for i in $(seq 1 50); do curl -s http://<chip>/state.json >/dev/null; done`
2. **Pass:** all 50 return valid JSON; `boot_count` does **not** increase and
   `uptime_ms` does **not** reset across the burst. (Pre-fix this rebooted the
   chip whenever the task-breakdown made the response exceed the buffer.)

## T5 — Internal-RAM headroom (heap-exhaustion regression)
**Catches:** internal-SRAM exhaustion from the DERP TLS handshake (cert-chain
verify) and leaving the WiFi driver up unnecessarily — both left the system one
allocation from a panic. **The 8 MB PSRAM does not help here:** DMA buffers and
task stacks are pinned to the ~168 KB internal SRAM, so `heap_min_int` is the
gate. Fixed by `CONFIG_MBEDTLS_DYNAMIC_BUFFER` (frees SSL I/O buffers + CA copy
+ config after each handshake), the 2+2 USB-NCM NTB buffers, and moving the
panic-log snapshot to PSRAM.
1. On the Ethernet path (Tailscale off), read `heap_min_int`.
2. **Pass:** `heap_min_int` ≳ **60 KB** (was ~9 KB before the mbedTLS dynamic-
   buffer fix). With Tailscale active it drops to ~32 KB — still healthy.
   **`heap_min_int` trending toward 0 over a soak, or `wg_pbuf_fails > 0`, is
   the wedge precursor — fail.**

## T6 — net_liveness watchdog (false-abort regression — HIGH VALUE)
**Catches:** the watchdog `abort()`ing the chip because it followed the default
netif's gateway (which Tailscale flaps to the un-pingable WireGuard overlay),
and the gateway-ICMP-flakiness false positive.
1. Enable Tailscale (see README), confirm `derp_only=0`, direct path up.
2. Drive sustained tunnel load: `ping -i 0.2 100.x.y.z` (chip's Tailscale IP)
   for **≥10 min**.
3. **Pass:** `boot_count=0`, `uptime_ms` climbs monotonically, no `reset_reason=4`.
   (Pre-fix the chip rebooted at ~120-160 s, every time.)
4. **Real-wedge check (must still fire):** the watchdog must still abort on a
   *genuine* lwIP wedge. It aborts only when the active uplink's **gateway is
   silent AND pstop has no fresh reply** for `NET_LIVENESS_TIMEOUT_MS` (90 s).
   Verify by code review / fault injection that a total-network-loss still
   triggers `reset_reason=4` → boot-counter → rollback.
   > Disarmed (by design) when: no STA-class upstream, the gateway never
   > answered ICMP at all, or SoftAP setup mode — so a no-ICMP gateway won't
   > false-reboot. Time math is uint64 (no ~49.7-day ms-wrap mis-fire).

## T7 — pstop re-bond self-heal (permanent-wedge regression)
**Catches:** a dropped burst of replies desyncing `received_counter` →
machine rejects every message as MSG_LOST → link wedged forever (chip bonds
once). Fixed by the reply-loss watchdog (`REBOND_AFTER_MS = 1500`).
1. With pstop flowing, cut replies for ~5 s (host: `sudo iptables -I OUTPUT -d
   <chip> -j DROP`; then delete the rule).
2. **Pass:** during the cut the machine goes to STOP (fail-safe); after restore,
   `pstop_rebonds` increments and `pstop_replies` resumes advancing, `rtt` back
   to normal, `pstop_mismatch` unchanged. The link recovers without a reboot.

## T8 — Core-task priority under Tailscale load (starvation regression)
**Catches:** safety core tasks (was prio 5) starved by the WG tasks (prio 7),
showing as a slow `pstop_mismatch` climb (core-publish-timeouts).
1. Full Tailscale + sustained tunnel load (as T6) for ~5 min, with pstop running.
2. **Pass:** `pstop_mismatch` stays at **0** (cores are now prio 8, above the
   microlink tasks). A creeping mismatch under load is the regression.

## T9 — OTA-rollback safety chain
**Catches:** a bad OTA bricking or crash-looping without recovery.
1. OTA a deliberately-crashing build (or induce ≥4 PANIC/WDT reboots):
   `curl -u admin:microlink -H "Content-Type: application/octet-stream"
   --data-binary @build/microlink_dual_core_safety.bin http://<chip>/admin/api/ota`
2. **Pass:** after 4 crash-class reboots (`boot_count` 1→4) the bootloader rolls
   back to the previous partition automatically; after 2 min healthy uptime the
   counter ages out to 0. A `ts_boot`-enabled chip drops to `derp_only` at
   `boot_count==1`, full Tailscale pause at `>=2`, then auto-recovers.

## T10 — RGB status LED liveness
**Catches:** WS2812 driver wedge / "stuck LED", wrong colour, sparse-pattern
"looks frozen".
1. Watch the LED and `rgb_cycles` over ~1 min.
2. **Pass:** `rgb_cycles` advances every cycle (~3.5 s); colour matches
   `active_iface` (blue/green/yellow/red); the octet blink is visibly periodic.
   A frozen `rgb_cycles` = the LED task wedged.

## T11 — WiFi + Tailscale RAM coexistence (`dcs-w5500-ethernet`)
**Catches:** the DERP TLS handshake (cert-chain verify) crushing the internal
heap to ~9 KB and OOM-crashing the chip while WiFi is also up. Fixed by
`CONFIG_MBEDTLS_DYNAMIC_BUFFER` (+ `..._FREE_CA_CERT` / `..._FREE_CONFIG_DATA`).
1. Boot on WiFi (or auto-failover to it), enable Tailscale (see README), and
   force a fresh DERP TLS handshake (toggle DERP via `/api/derp`, or reboot with
   `ts_boot` on). Keep pstop running.
2. **Pass:** `heap_min_int` stays **> ~25 KB** straight through the handshake
   (no dip toward 0), `pstop_mismatch` (`mm`) stays **0**, no reboot
   (`boot_count=0`, `uptime_ms` monotonic). A dip toward 0 or an OOM reset is the
   regression. **Cross-check:** the same DERP handshake on Ethernet-only should
   show `heap_min_int` ≳ 60 KB (WiFi down).

## T12 — Runtime failover ladder Eth → USB → WiFi (`dcs-w5500-ethernet`)
**Catches:** the supervisor skipping a tier (e.g. jumping straight to WiFi),
failing to bring a tier up on demand, churning a driver on a flapping link,
auto-revoking a manual enable, or not promoting Ethernet when its lease returns.

**A — USB is preferred over WiFi.** Start on Ethernet (`active_iface=1`, blue)
with a USB-NCM host attached (laptop with the `dcs-usb` shared connection) and
Tailscale up.
1. **Unplug (or admin-disable) Ethernet.** Within **~5 s** USB-NCM auto-enables,
   `active_iface` becomes 2 (green), `usb_ip` gets a `10.42.0.x` lease, and
   **`wifi_en` stays 0** (WiFi must NOT come up while USB carries). Tailscale +
   pstop ride USB, `mm=0`.
2. **Restore Ethernet.** It re-requests DHCP, `active_iface` returns to 1 (blue)
   within ~1 s with a fresh `eth_ip`, and **~15 s** later USB auto-drops
   (`usbncm_en→0`; `heap_min_int` recovers).

**B — WiFi only when USB has no host.** Repeat with **no** USB host attached.
After Ethernet loss USB enables but gets no lease; **~20 s** after Ethernet went
down WiFi auto-enables (`active_iface=3`, yellow) and carries Tailscale.

**C — late Ethernet (robot boot).** Ethernet always requests DHCP, so a lease
appearing minutes after boot promotes Ethernet on the next tick — simulate by
keeping Ethernet unplugged at boot (fails over to USB/WiFi) then plugging it in.

1. **Pass:** correct tier order (USB before WiFi), both-direction switching,
   fresh DHCP on Ethernet return, `mm=0` throughout, no reboot. The asymmetric
   up/down hysteresis must hold (no churn on a quick re-plug), a link-local
   `169.254.x` lease must NOT count as usable, and **a tier enabled by the admin
   toggle must NOT be auto-disabled** (enable USB/WiFi manually, restore
   Ethernet, confirm it stays enabled).

## T13 — Chaos / soak stress (stability gate)
**Catches:** any latent crash, leak, lockstep desync, or failover gap that only
shows under sustained mixed-fault load. This is the branch's overall stability
gate.
1. Bring the chip up with Tailscale on and pstop flowing, then run the harness:

   ```sh
   python3 tools/chaos_soak.py [duration_seconds]
   ```

   It probes reachability + `/state.json` telemetry while injecting reboots,
   Ethernet drops, WiFi failover, and USB + WireGuard toggles, then reports ping
   success rate, RTT, `pstop_mismatch`, and crash count.
   > Space induced reboots **> ~120 s** apart — the safety layer counts rapid
   > boots (`DCS_SAFETY_MAX_RAPID_BOOTS=3`, clears after 120 s stable uptime) and
   > will roll firmware back on a crash-loop. The harness already paces them.
2. **Pass:** **zero unexpected reboots/crashes**, `pstop_mismatch` bounded, the
chip **recovers from every injected chaos event** (each Ethernet drop /
   failover / toggle returns to a reachable, pstop-flowing state), and a **high
pstop reply rate** (~99 %+) overall.

---

## Debugging a crash (no UART adapter needed)
Console/panic output goes to UART0 (GPIO43/44), which isn't wired on this bench.
Use the chip's **built-in USB-JTAG** instead for full backtraces:

```sh
openocd -f board/esp32s3-builtin.cfg &                 # GDB server on :3333
xtensa-esp32s3-elf-gdb -batch -x catch.gdb build/microlink_dual_core_safety.elf
# catch.gdb: target remote :3333; break esp_panic_handler; break abort; continue
```

On a panic the breakpoint halts the chip *before* it reboots → `bt` /
`thread apply all bt` show the faulting task. To reproduce a full-Tailscale-only
crash, temporarily set `DCS_DEBUG_FORCE_FULL_TS` in `dcs_support.c` to bypass the
boot-count `derp_only` ladder (revert before commit). Note: once the app's USB-NCM
tether (TinyUSB) is active the USB-JTAG is replaced — reflash via OTA then.

## Troubleshooting (lessons from the bench)
+ **`4WAY_HANDSHAKE_TIMEOUT` at workable signal (e.g. -71 dBm) with a known-good
  PSK** points to the **station's RX buffering/config**, not the AP or the
  password. On this board it was an over-trimmed WiFi buffer config
  (`static_rx=4`) starving the RX ring and dropping EAPOL frames — restoring the
  stock `WIFI_INIT_CONFIG_DEFAULT()` buffers fixed association. Validate the PSK
  independently on a laptop first (`nmcli device wifi connect <ssid> password
  <pw>`, then revert) before suspecting credentials.
+ **OTA over Ethernet occasionally returns `curl: HTTP 000`** — this is a
  transient; just retry the upload.
+ **Space reboots > ~120 s apart.** The safety layer counts rapid boots
  (`DCS_SAFETY_MAX_RAPID_BOOTS=3`, clears after 120 s stable uptime) and will
  roll firmware back if it sees a crash-loop — back-to-back manual reboots can
  trip a false rollback.
