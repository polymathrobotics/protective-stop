# microlink lwIP wedge — root cause + fix (2026-05-26)

> ✅ **Resolved as of v15.26.** This file's earlier analysis (lwIP pbuf
> pool exhaustion, kept below for context) turned out to be wrong. The
> real cause was three separate bugs hiding behind a silent panic
> because the chip had no reachable console. Once we wired a CP2102
> USB-UART to UART0 and captured `stderr`, the actual failures became
> obvious within minutes.

## What the wedge actually was

Three independent issues whose symptoms overlapped into "the chip
silently dies somewhere between 1 and 80 minutes":

### 1. lwIP cross-thread access without core lock (v15.20)

`process_wg_packet` in `ml_wg_mgr.c` calls `wireguardif_network_rx`
on the `ml_wg_mgr` FreeRTOS task. That function decrypts the packet
then calls `ip_input(pbuf, device->netif)`, which mutates lwIP state
directly. Meanwhile the TCPIP thread is also operating on the same
state. Without `CONFIG_LWIP_TCPIP_CORE_LOCKING=y` and explicit
`LOCK_TCPIP_CORE()`/`UNLOCK_TCPIP_CORE()` brackets, the two threads
silently corrupted pbuf/netif structures under sustained direct-UDP
load. This is upstream microlink issue
[#15](https://github.com/CamM2325/microlink/issues/15) /
PR [#14](https://github.com/CamM2325/microlink/pull/14).

**Fix:** enable `LWIP_TCPIP_CORE_LOCKING`, wrap every cross-thread
lwIP call. See `components/microlink/src/ml_wg_mgr.c` and
`components/microlink/components/wireguard_lwip/src/wireguardif.c`,
both gated on `sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER)` so we
don't recursively lock when already on the TCPIP thread.

### 2. `bc_clear` task stack overflow (~2 min, v15.25)

```
***ERROR*** A stack overflow in task bc_clear has been detected.
Backtrace: 0x4037d8d5:0x3fce8910 0x4037d89d:0x3fce8930 ...
```

`clear_boot_count_task` (the 2-min healthy-uptime crash-counter
clear-out) was created with `xTaskCreate(..., 2048, ...)`. The
combined cost of `vTaskDelay` + `nvs_write_boot_count` + `ESP_LOGI`
+ v15.24's added `esp_restart` branch drained the stack past the
canary. GCC pre-allocates the function frame for the worst-case
branch even when not taken, so v15.24's addition contributed even
on paths where its branch never executed.

**Fix:** bump to 4096.

### 3. ESP-IDF `ping_sock` task stack overflow (~33 min, v15.26)

```
I (1972901) INET_PING: REPLY seq=395 ttl=111 8.8.8.8 len=64 time=7ms
***ERROR*** A stack overflow in task ping has been detected.
Backtrace: 0x4037d8d5:0x3fce89c0 0x4037d89d:0x3fce89e0 ...
```

The IDF `ping_sock` task runs on `ESP_TASK_PING_STACK` =
`2048 + TASK_EXTRA_STACK_SIZE` (~2.5 K). After ~395 ICMP iterations,
the cumulative pressure of socket I/O + lwIP ICMP + our log
callback (with `vsnprintf` for the REPLY line) drained it.

**Fix:** set `cfg.task_stack_size = 4096` before `esp_ping_new_session`
in `start_internet_ping`.

## How to debug a new wedge if one appears

The serial debug cable is the leverage point. ESP-IDF's
`CONFIG_ESP_CONSOLE_UART_DEFAULT=y` puts every panic banner on
UART0 (`/dev/ttyUSB1` on the bench's CP2102 adapter). UART is
independent of the TinyUSB CDC composite running on USB-OTG, so it
keeps working even when the USB net interface is down. Capture:

```sh
stty -F /dev/ttyUSB1 115200 raw -echo
cat /dev/ttyUSB1 > /tmp/chip_serial.log &
```

Trigger the suspected wedge, then `grep stack /tmp/chip_serial.log`
for the offending task name. Decode any PC offsets with:

```sh
xtensa-esp32s3-elf-addr2line -e build/microlink_dual_core_safety.elf <addr>
```

Without the serial cable, the chip will look like it "silently"
reboots after a wedge — every backtrace and panic message dies on
UART0 with no listener. The `/api/last_log` endpoint helps a bit
(RTC_NOINIT ringbuffer keeps ~7 KiB of ESP_LOG across reboots) but
won't capture the panic banner itself.

## Bench evidence (v15.26)

| Mode | Uptime | Crashes | pstop reply rate | P50 / min latency |
|---|---|---|---|---|
| USB-NCM (10.42.0.80) | 44 min | 0 | 99.99 % | 11.7 / 11.4 ms |
| WiFi (192.168.107.131) | 49+ min | 0 | 99.98 % | 24 / 12 ms |

Sustained 10 Hz pstop + 5 Hz Tailscale ping over the entire window.

---

# Historical analysis (pre-v15.20, kept for context)

The pbuf-pool-exhaustion hypothesis below is **wrong** as a root
cause but the diagnostic technique (heap watermark, pbuf_alloc fail
counter) is still useful for unrelated memory issues. The "60-90 s"
timing was actually the v15.25 `bc_clear` overflow firing at the
2-min healthy timer — the timing matched coincidentally.

## Symptom in detail

What we observe, deterministically, on every fresh boot once Tailscale
registers and the WG peer table is populated:

1. ~T+15 s: `ml_state` transitions to `4` (CONNECTED), `vpn_ip` populated,
   peers visible in `/state.json`.
2. ~T+15 → T+~80 s: traffic flows. We've seen 20+ sustained pings over
   the WG netif at 5–19 ms RTT, normal `coord/derp` keepalives, normal
   admin UI responses.
3. ~T+60–90 s: silently, all of:
   + `tailscale ping 100.97.180.43` from host returns `no answer`
   + `curl http://10.42.0.80/` over USB-NCM hangs (connect succeeds,
     read times out)
   + `curl http://10.42.0.1/` from chip side (pstop UDP) `EAGAIN`s
     repeatedly
   + UART log goes quiet (no microlink log lines)
4. ~T+150–180 s: liveness watchdog (`net_liveness_task` in `main.c`)
   notices no INET_PING progress for 60 s past its 90 s grace, calls
   `abort()`, system reboots, cycle repeats.

Critically, *all three netifs* (USB-NCM, WiFi STA shell, WG) go silent
at the same instant. The chip itself is alive (core dumps would show a
crash; we get none) but **the lwIP single TCPIP thread has stopped
processing packets**.

That's the key fingerprint. Whatever is wedging this, it's at the lwIP
layer, not at the WireGuard or DERP layers individually.

## Three hypotheses, ranked

### #1 — pbuf pool exhaustion (most likely)

`CONFIG_LWIP_PBUF_POOL_SIZE` defaults to 16 entries in ESP-IDF 5.5. We
currently inherit that default. With three netifs concurrent — USB-NCM
TX/RX, WiFi STA shell RX, WG netif TX/RX going through the wireguardif
crypto path — plus a 16-peer Tailscale ingestion at startup with
`CallMeMaybe` and DISCO pings going to every peer in parallel, we are
plausibly hitting the pool ceiling. Once `pbuf_alloc` returns NULL on
the RX path, packets are dropped silently and TCP rwnd never recovers
until something times out, which is many seconds later.

The all-netifs-silent-at-once fingerprint fits this hypothesis better
than anything else: a single shared resource starves, and every netif
that needs it stalls together.

**The fix is one line:**

```
CONFIG_LWIP_PBUF_POOL_SIZE=64
```

in `examples/dual_core_safety/sdkconfig.defaults`. Costs ~3 KB of
internal RAM (we have plenty of headroom on the S3).

**Verification:** add an `ESP_LOGW` log in `wireguardif_output`
(`components/microlink/components/wireguard_lwip/src/wireguardif.c`)
when `pbuf_alloc` returns NULL. Should be silent on the bigger pool.

### #2 — DISCO probe burst saturating TCPIP thread

In `components/microlink/src/ml_wg_mgr.c`, the disco probe loop fires
periodic probes to every peer to discover NAT mappings. With 16 peers
in the table, that's a burst of ~16 outbound UDP encrypts every probe
cycle, each going through the WireGuard crypto path on the TCPIP
thread. The crypto path is slow (~10–20 ms per packet on S3 without
HW accel). 16 × 15 ms = 240 ms of TCPIP thread time per probe cycle.
During that window, USB-NCM RX is parked.

If probes coincide with a DERP keepalive timeout, the keepalive isn't
processed in time, the DERP TCP gets RST, ml_derp reconnects, that
reconnect bursts more peer info, the cycle deepens — until the
liveness watchdog reboots us.

**Mitigation:** cap concurrent in-flight DISCO probes at 2 in
`disco_periodic_probes()`. Throttle the queue instead of fanning out.

### #3 — mbedTLS connection leak we didn't fully catch

We fixed an obvious DERP-reconnect leak in `ml_derp.c` (the
`DERP_CONNECT_FAIL_CLEANUP()` macro). But the DERP `read_packet` /
`write_packet` paths can also bail mid-loop with a partial allocation
state we don't unwind. Symptom would be slowly growing heap usage
ending in `mbedtls_ssl_handshake` returning `MBEDTLS_ERR_SSL_ALLOC_FAILED`.

**Verification:** log `esp_get_free_heap_size()` and
`esp_get_free_internal_heap_size()` every 5 s for the first 2 min
post-boot. If heap monotonically decreases until wedge, this is real.
If heap stays flat, rule this out.

Currently `state.json` shows `heap_free` (the largest free block) but
doesn't include `internal_free` or the trend. Worth adding a min-heap
watermark field.

## Three small fixes from upstream we haven't pulled

`~/microlink-polymath` (user's home dir on the previous host) is a
newer fork than what's in this repo's `components/microlink/`. Two
commits we *have* pulled, two we haven't:

| upstream | applied? | what it fixes |
|---|---|---|
| `502d0b1` partial-read in coord/noise_recv | yes | large-tailnet partial socket reads |
| `f4558d4` MapResponse 60s timeout | yes | long polls being killed at 30 s |
| `09c387e` v2.1.0: TCP API + dual-path WG handshake + EarlyNoise slow-path | **NO** | makes WG handshake work without disco/stun, removes a 30-s slow-path window |
| `1c49194` ml_socket AF_INET6 → AT bridge | NO (probably N/A — we don't use AT) | IPv6 socket routing |

The `09c387e` set is the interesting one. It adds "dual-path"
handshake initiation: WireGuard INIT message goes out via DERP relay
AND via direct UDP simultaneously, with the first to land winning.
That removes the dependency on DISCO `CallMeMaybe` round-trips for
the initial handshake, which is exactly the part of our stack we
turned off (because turning it on broke registration). If we land
this, we can keep disco/stun=false *and* get reliable handshakes,
which may also reduce the load that produces the wedge.

It's a non-trivial port (touches `ml_wg_mgr`, `ml_derp`, peer
state machine) — try the pbuf fix first.

## What's already in place that we should not regress

These mitigations are committed and help, but aren't enough on their
own:

+ `bf499bd`: `wifi_idle_init()` doesn't call `esp_wifi_start()` →
  eliminated the WiFi scan storm (hundreds of "Haven't to connect to a
  suitable AP" logs/sec that were lwip-starving by themselves).
+ `0471d8c`: `DERP_CONNECT_FAIL_CLEANUP()` macro on every DERP connect
  failure path → plugs the obvious mbedTLS leak (hypothesis #3 covers
  the *non-obvious* paths still potentially leaking).
+ `8bb88e9`: `[TIMING-DERP]` logs at every connect phase → makes wedge
  bisection possible from UART alone.
+ `ml_coord.c`: 2 s `vTaskDelay` before signalling `ML_EVT_DERP_CONNECT_REQ`
  → staggers DERP connect after peer-list ingestion settles. Helped
  measurably with the cold-start phase but doesn't address the steady-state
  wedge.
+ `sdkconfig.defaults`: `CONFIG_LWIP_TCP_RECVMBOX_SIZE=32` (was 6) →
  bigger TCP RX backlog per socket; helped DERP HTTP-upgrade success
  rate from ~50% to ~95% on cold boot.

## Verification protocol for any wedge fix

The chip is deterministic enough that 3 × 5-min boots is sufficient to
say "wedge gone".

```sh
# Build, OTA, then for each of 3 attempts:
#   1. wait 30 s for boot + Tailscale up
#   2. POST /api/wg and /api/derp paused-OFF (so they're ON)
#   3. start `tailscale ping 100.97.180.43 -c 600` from host (10 min)
#   4. monitor /state.json for ticks/heap every 5 s
# Pass criteria:
#   - all 600 pings reply
#   - boot_count stays at 1
#   - heap_free min watermark > 50 KB
#   - no ml_state regressions (must stay 4=CONNECTED throughout)
```

If we get one clean 10-minute run, do one 1-hour run before declaring
the wedge fixed. The bug has a 60–90 s recurrence so anything past 5
min represents real signal.
