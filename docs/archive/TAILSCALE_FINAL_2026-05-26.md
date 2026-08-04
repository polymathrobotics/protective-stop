# Tailscale on dual_core_safety — final status 2026-05-26

After the v15.8 changes Tailscale-direct ICMP between the host and the
chip works end-to-end over DERP, with the chip's safety chain intact.
v15.10 (2026-05-26) re-confirmed the same behavior after a brief
direct-UDP-path experiment in v15.9 — see "Direct UDP path attempt" at
the bottom for what we learned.

## TL;DR

| Metric | Result |
|---|---|
| 500-packet ping at 2 Hz | **500/500 OK (0% loss)** over 250 s |
| 1000-packet ping at 5 Hz | **998/1000 OK (0.2% loss)** over 200 s |
| Tailscale ICMP latency (P50 / P90 / P95 / P99) | **164 / 189 / 193 / 198 ms** (DERP relay 'dfw') |
| 5× admin restart cycle | **5/5 chip + Tailscale back up**, 48/50 ping post-reset (96 %) |
| Chip uptime with Tailscale active + pings flowing | **10+ min sustained**, heap stable, no wedge |

## What ended up working

Three pieces compose to give a reliable WG session **without re-enabling
DISCO** (which still wedges the chip on busy tailnets):

1. **`priority_peer_ip`** in NVS / settings — flags the device we want
   reachable (here: the host PC at `100.110.35.58`). Existing LRU
   eviction code in `ml_wg_mgr.c` ensures the priority peer stays in
   the 16-slot WG table when the chip ingests >16 peers from MapResponse.

2. **Peer allowlist** containing just the priority peer — set via
   `POST /admin/api/peers/allowed`. With the allowlist, the chip won't
   spend NaCl box-opens on incoming DISCO from the other peers. The
   guard at the top of `process_disco_packet()` early-returns based on
   `ml_config_peer_is_allowed()`.

3. **Active WG handshake to the priority peer on `add_peer` success**
   (v15.8 — `ml_wg_mgr.c` ~line 535). Once the host's peer entry
   lands in our wireguardif table, we call
   `wireguardif_connect_derp()` on it. That sends a single WG INIT
   message out via DERP. The host's magicsock receives it via
   `receiveIPv4()` and calls `noteRecvActivity()` →
   `maybeReconfigWireguardLocked()` — i.e., it lazy-adds us to its
   wireguard-go. wireguard-go then processes the INIT and sends a
   RESPONSE back via DERP. Session established. ICMP flows after that.

The key insight: with DISCO off, the chip is "invisible" to peers'
magicsocks (they don't add it to wireguard-go unless they receive
*something* from it). The active handshake gives the peer that
"something" without any DISCO machinery — and *one* INIT per peer is
cheap (versus DISCO's per-second probe storm).

## Setup commands (one-time after fresh erase-flash)

> ⚠️ **Substitute your own values.** The IP `100.110.35.58` below is
> the host this doc was written from — it's an *example*, not a magic
> number. **Use `tailscale ip -4` on the laptop you're sitting at to
> get the IP that must go into the chip's allowlist.** Copy-pasting
> `100.110.35.58` will silently leave you unreachable from your own
> machine. Same goes for `10.42.0.80` — newer firmware DHCPs from
> NetworkManager-shared and may be at `10.42.0.106` or similar; check
> `ip neigh show dev enx<host-mac>` to find it.

```sh
HOST_TS_IP=$(tailscale ip -4)                        # YOUR Tailscale IP
CHIP_IP=10.42.0.106                                  # check ip neigh
HOST_LABEL=$(hostname)

# 1. Add the host's Tailscale IP to the peer allowlist:
curl -u admin:microlink -X POST http://$CHIP_IP/admin/api/peers/allowed \
     -H "Content-Type: application/json" \
     -d "{\"peers\":[{\"ip\":\"$HOST_TS_IP\",\"label\":\"$HOST_LABEL\"}]}"

# 2. Mark the same IP as priority:
curl -u admin:microlink -X POST http://$CHIP_IP/admin/api/settings \
     -H "Content-Type: application/json" \
     -d "{\"priority_peer_ip\":\"$HOST_TS_IP\"}"

# 3. Enable Tailscale at boot (NVS — persists across reboots):
curl -X POST http://$CHIP_IP/api/ts_boot
#    {"ok":true,"ts_boot_en":1,"message":"takes effect on next reboot"}

# 4. Reboot for the changes to take effect cleanly:
curl -u admin:microlink -X POST http://$CHIP_IP/admin/api/restart
```

The chip reads all three NVS values on the next boot. As soon as
MapResponse delivers the host as a peer, the chip's `add_peer`
auto-triggers `wireguardif_connect_derp()` on the host. Session up
in ~5–15 s post-boot.

## Bench evidence (verbatim from this session)

```
$ ping -c 500 -i 0.5 -W 3 -I tailscale0 100.68.91.75
--- 100.68.91.75 ping statistics ---
500 packets transmitted, 500 received, 0% packet loss, time 249784ms
rtt min/avg/max/mdev = 98.170/179.247/7794.799/341.517 ms
P50=168.0ms  P90=185.0ms  P95=187.0ms  P99=192.0ms

$ ping -c 1000 -i 0.2 -W 2 -I tailscale0 100.68.91.75
--- 100.68.91.75 ping statistics ---
1000 packets transmitted, 998 received, 0.2% packet loss, time 200181ms
rtt min/avg/max/mdev = 99.716/161.017/203.066/23.894 ms
P50=164.0ms  P90=189.0ms  P95=193.0ms  P99=198.0ms

# 5× admin restart cycle (Tailscale comes up clean each time):
reset #1: 10/10 ping
reset #2: 9/10  ping  (1 lost during handshake re-establishment)
reset #3: 10/10 ping
reset #4: 10/10 ping
reset #5: 9/10  ping
```

Chip's heap held at 42 KiB internal floor throughout. `wg_pbuf_fails = 0`.
No wedge in 10+ min of Tailscale-active uptime with sustained 5 Hz ICMP.

## Latency note: ~165 ms via DERP

That's the DERP-relay round trip — Dallas (region 'dfw') is the
chip's home region, and traffic is going chip → 'dfw' → host →
'dfw' → chip. With DISCO off the chip can't establish a direct
LAN-local peer→peer path even though the host is on the same WiFi
network. If you want sub-10 ms latency, you'd need to re-enable
DISCO/STUN and solve the residual load problem. For most safety
heartbeat use cases, 165 ms via DERP is fine.

## Caveats

- **Chip's NVS keeps `ts_boot_en=1` once set** — auto safe-mode
  (force `ts_boot=0` if `bc≥1`) will engage on a crash, and
  `clear_boot_count_task` needs 2 min of healthy uptime to clear the
  counter before Tailscale comes back. That's the intended safety
  chain behavior; if the chip ever genuinely crashes during a soak,
  it will drop into stable-no-Tailscale mode for one cycle.
- **DERP is the only path** with this configuration. A fully-routable
  direct UDP path between the chip and the host needs DISCO/STUN,
  which currently wedge the chip on tailnets larger than ~8 peers
  even with the inbound-DISCO allowlist filter (`process_disco_packet`
  early-return). Diagnosing that is a future-session task —
  `enable_disco = true` in main.c is the entry point.
- **Per-cycle 90% loss seen on 2 of 5 resets** is the very first
  packet of a 10-packet burst, before the WG session is rebuilt
  post-reboot. For a 10 Hz pstop heartbeat use case the first
  100 ms post-boot would be lossy; everything after is steady.

## Direct UDP path attempt (v15.9 → v15.10 revert)

v15.9 (2026-05-26) flipped `cfg.enable_disco = true` and
`cfg.enable_stun = true` in main.c to let Tailscale form a direct
UDP path between chip and host on the shared USB-NCM subnet
(10.42.0.0/24). With the allowlist filter applied to both
`process_disco_packet` (incoming, ml_wg_mgr.c ~line 1062) and
`disco_periodic_probes` (outgoing, ml_wg_mgr.c ~line 1520), only the
priority peer's DISCO traffic survives the early-return gates.

**The direct path does form.** Immediately after v15.9 OTA the first
`tailscale ping` came back as:

```
pong from esp32-1cdbd444b000-1 (100.68.91.75) via 10.42.0.80:51820 in 69ms
```

`10.42.0.80:51820` is the chip's WireGuard listener on the USB-NCM
interface, so traffic was crossing the local L2 link directly — no
DERP hop. Subsequent measured single packets dropped to **4.5 ms min /
10.7 ms P50 / 27.7 ms P99** on the packets that arrived.

**But the chip wedged inside ~3 s of sustained ping load.** A
100-packet @ 5 Hz ping saw 13/100 returned, all of those within the
first ~2.6 s; the remaining 87 % timed out. State on the next
state.json scrape showed `bc=1`, auto-safe-mode engaged
(`ts_boot_en=0`, `wg_paused=1`, `derp_paused=1`). A manual
`/api/wg`+`/api/derp` resume crashed the chip a second time. Heap was
still healthy on both reads (`heap_min_int ≥ 71 KiB`,
`wg_pbuf_fails = 0`) — the failure mode isn't memory exhaustion.

Best current hypothesis: the WG endpoint-rewrite the first time a
peer's path flips from DERP to direct UDP exercises a code path in
wireguard-lwIP / ml_wg_mgr that isn't safe under sustained traffic on
this 16-peer table size. Diagnosing requires a serial-attached panic
trace, which we don't have during unattended bench work.

**v15.10 revert.** Both flags back to `false`. Verified afterwards
with a fresh 500-packet @ 2 Hz soak: **500/500 OK, P50 = 165 ms,
P99 = 193 ms, heap floor 42 KiB, no pbuf fails, no wedge.** Identical
to v15.8 baseline.

**Future-work entry point.** The diff that proves the direct path
works is just main.c:1247 — flipping `enable_disco` and `enable_stun`
back to `true`. The allowlist filter is already in place; the wedge
is downstream. Next session should attach to USB-Serial, OTA v15.9
again, ping at low rate (1 Hz), and capture the panic trace when the
chip crashes. With the trace, the offending code path becomes a
targeted fix.

## v15.11–v15.19 — direct UDP path restored (≥ 60× longer life)

After the v15.10 revert we kept hunting. Three findings stacked up:

**1. The big stacks were on the wrong tasks.** v15.11 bumped
`LWIP_TCPIP_TASK_STACK_SIZE` 4 K → 16 K under the theory that the
recursive `wireguardif_network_rx → decrypt → ip_input → icmp_echo
→ wireguardif_output → encrypt → udp_sendto` chain was overflowing
TCPIP. That stretched the wedge from ~3 s to ~64 s but didn't fix
it. v15.15 bumped `ML_TASK_WG_MGR_STACK` 8 K → 16 K — same chain
runs on this task with zero-copy *off* — and got us to ~100 s.
With both bumps, monitoring confirmed `tcpip` and `wg_mgr` never
came close to their high-water marks (15 K / 13 K free out of 16 K),
so stack-on-the-data-path wasn't the root cause.

**2. IDF's internal task stacks were the actual choke point.** v15.17
bumped `ESP_SYSTEM_EVENT_TASK_STACK_SIZE` (2304 → 4096) and
`ESP_TIMER_TASK_STACK_SIZE` (3584 → 6144). Wedge moved from ~100 s to
~180 s. These tasks (sys_evt, esp_timer) handle WiFi events and timer
callbacks; they're invisible to ESP_LOG and have no path for their
panic banner to reach the host (TinyUSB blocks USB-Serial-JTAG, UART
isn't pinned out on the XIAO S3), which is why every wedge looked
identical and silent. /admin/api/monitor now reports all eight
watchable tasks so this dead-zone is at least visible.

**3. Some residual wedge stays.** Repeated 5 min @ 2 Hz Tailscale-
ping soaks still trip a panic around 100–180 s of sustained direct-
UDP. No captured ESP_LOG WARN or ERROR fires before the silence —
the chip is steadily processing WG RX/TX, then dies. Verbose INFO
logging actually *delays* the wedge by ~3× (a vsnprintf-cost-per-
packet yield somewhere), so the bug feels rate-correlated, not
purely time-based. The current best guess is a WiFi-driver internal
or PSRAM cache issue we can't reach from the application.

**Bench evidence (v15.17, chip on prime_iot WiFi 192.168.107.131,
host at 192.168.107.66):**

```
$ tailscale ping -c 2 100.68.91.75
pong from esp32-1cdbd444b000-1 (100.68.91.75) via 192.168.107.131:51820 in 71ms

$ ping -c 250 -i 0.5 -W 3 -I tailscale0 100.68.91.75
250 packets transmitted, 248 received, 0.8% packet loss
rtt min/avg/max/mdev = 11.5/27.0/241.6/20.5 ms
P50 = 27 ms  (vs DERP P50 = 165 ms — 7× faster)
```

## v15.19 — graded safety net

Now that we know the direct-UDP wedge exists but is bounded, the
chip stops treating it as catastrophic. The boot_count ladder in
`app_main` got an extra rung:

```
bc == 0  : direct-UDP enabled, full Tailscale          (default)
bc == 1  : DERP-only for this boot — drop DISCO+STUN,  (auto-fallback
           keep TS up so pstop heartbeat continues       after wedge)
bc >= 2  : Tailscale paused                            (genuine safe-mode)
```

NVS isn't touched in the first two cases. After the 2 min healthy
uptime window clears `bc` back to 0, the next reboot tries direct-
UDP again — so the chip eventually re-attempts the fast path on
its own. `s_derp_only_mode` is surfaced in /state.json as `derp_only`
so it's obvious which mode the chip is running in.

**Verified end-to-end:**
- Triggered crash with sustained Tailscale-ping flood (bc 0→1, rr=4 panic)
- Chip rebooted, came back ml=4 at t ≈ 9 s with `derp_only=1`
- 200-pkt @ 2 Hz under DERP-only fallback: **200/200 OK, 0 % loss,
  P50 = 23 ms, P99 = 76 ms** (host on same LAN as chip)
- pstop heartbeat **2076/2077** (99.95 %) — survived the crash +
  reboot + mode-flip with one missed beat

So the net story: direct UDP works and is fast when it works, the
chip self-recovers when it doesn't, and pstop heartbeat — the
actual control-loop primitive this chip exists to serve — stays
above 99 % delivery through the failure mode.

## v15.20 — wedge ACTUALLY FIXED (lwIP core locking)

Found the root cause in upstream microlink issues **#14** (closed,
unmerged) and **#15** (open):

> MicroLink was calling lwIP core APIs like `netif_set_up()` from the
> `wg_mgr` task instead of the TCPIP context. That can corrupt pbuf,
> socket, and netif state, which matched your earlier reboot/crash
> symptoms.

Exact same code path we hit. `process_wg_packet()` (ml_wg_mgr task) →
`wireguardif_network_rx()` → `wireguardif_process_data_message()` →
`ip_input(pbuf, device->netif)` — all running on the wg_mgr task while
the TCPIP thread concurrently touches the same pbuf/netif state. Under
sustained direct-UDP load that race corrupts state and the chip silently
panics ~100-180 s in. That's exactly what we observed.

### Fix

Mirrors upstream PR #14 (closed without merge, but the diff applies the
right pattern). v15.20 manually ports the relevant changes:

1. Enable `CONFIG_LWIP_TCPIP_CORE_LOCKING=y` in sdkconfig.defaults.
2. Wrap every direct lwIP API call made from a non-TCPIP thread with
   `LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()`. The wrap is gated on
   `sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER)` so we don't
   recursively lock when the call is already on the TCPIP thread:
   - `ml_wg_mgr.c wg_udp_output_cb` — `udp_sendto`
   - `ml_wg_mgr.c process_wg_packet` — `wireguardif_network_rx`
   - `wireguardif.c` — `netif_set_link_up` (handshake response path,
     data-packet path) + `netif_set_link_down` (periodic timer ×2)

### Bench evidence (chip 192.168.107.131, host on same LAN)

Set chip's pstop_peer to host's Tailscale IP 100.110.35.58:8890, ran
host-side `pstop_responder.py`. Tailscale-direct UDP path active
(`tailscale ping` confirms `pong via 192.168.107.131:51820`).

```
22.5 min uptime, bc=0, no crash
13 372 pstop sent / 11 374 replies   (85.1 % reply rate)
Tailscale-direct latency: P50 = 27 ms  P90 = 69 ms  P99 = 132 ms
                          min = 13.8 ms  loss = 0/50 in a 50-pkt @ 5 Hz burst
heap floor: 34.9 KiB internal, wg_pbuf_fails = 0
all task stacks stable end-to-end
```

Previous v15.19 baseline: wedged after 100-180 s of the same load,
every run. v15.20 ran 22+ minutes with no degradation and no crashes.

### Bottom line

- Direct UDP works AND is stable.
- Latency P50 is 27 ms (vs 165 ms via DERP — **6× faster**).
- pstop bidirectional heartbeat over Tailscale-direct UDP: 85 % reply
  rate sustained over 13 000+ messages (the loss-to-responder is at the
  host side / WG path, not chip wedging).
- The v15.19 graded-safety-net ladder stays in place as defense in
  depth, but the trigger condition (~3 min direct-UDP crashes) is now
  gone, so in practice the chip will sit at `bc=0` / `derp_only=0`
  indefinitely.

## v15.25 + v15.26 — residual wedge actually root-caused

After v15.20 we still saw multi-minute wedges at varying times. Hooking
up the CP2102 USB-UART debug cable to UART0 (`/dev/ttyUSB1`) made the
crashes visible, and what looked like one mystery turned out to be
**two distinct task-stack overflows**:

**v15.25: `bc_clear` stack overflow at t ≈ 2 min**

```
***ERROR*** A stack overflow in task bc_clear has been detected.
Backtrace: 0x4037d8d5:0x3fce8910 0x4037d89d:0x3fce8930 ...
```

The `clear_boot_count_task` was created on a 2048-byte stack. The
combined cost of `vTaskDelay` + `nvs_write_boot_count` + `ESP_LOGI` +
the v15.24 auto-recovery branch (`ESP_LOGW` + `esp_restart`) pushed
past the canary. GCC pre-allocates the function frame for the
worst-case branch even when not taken, so v15.24's added code
contributed even on the "no derp_only" path. Bumped to 4096.

**v15.26: `ping` task stack overflow at t ≈ 33 min**

```
I (1972901) INET_PING: REPLY seq=395 ttl=111 8.8.8.8 len=64 time=7ms
***ERROR*** A stack overflow in task ping has been detected.
Backtrace: 0x4037d8d5:0x3fce89c0 0x4037d89d:0x3fce89e0 ...
```

The IDF `ping_sock` task runs on `ESP_TASK_PING_STACK` =
2048 + TASK_EXTRA_STACK_SIZE (~2.5 K). After ~395 ping iterations
the cumulative pressure of socket I/O + lwIP ICMP + our success
callback (with vsnprintf for the REPLY log line) drained the stack
past the canary. Set `cfg.task_stack_size = 4096` explicitly before
`esp_ping_new_session`.

This is consistent with every multi-minute "wedge" we observed:
- v15.20: ~3 min crashes (occasionally ping after very few iterations)
- v15.21+: 33–80 min crashes (later ping firings under accumulated
  stack pressure, exact timing depending on network jitter)

Together with v15.20's lwIP core-locking and v15.22's WG dedup,
the chip now has no known stack-tight tasks under sustained load.
Bench targets: USB-NCM Tailscale-direct P50 = 11.7 ms, min = 11.4 ms;
WiFi Tailscale-direct P50 = 22 ms, min = 12 ms.

### How we got the trace

A CP2102 USB-UART adapter wired to the chip's UART0 pins, host side
at `/dev/ttyUSB1`. The chip's `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`
routes all ESP-IDF panic/log output to UART0 (independent of the
TinyUSB CDC composite that consumes USB-OTG). Capture with:

```sh
stty -F /dev/ttyUSB1 115200 raw -echo
cat /dev/ttyUSB1 > /tmp/chip_serial.log &
```

The panic banner is in cleartext, no decoding needed for the task
name. To resolve PC offsets:

```sh
xtensa-esp32s3-elf-addr2line -e build/microlink_dual_core_safety.elf <addr>
```

## What `priority_peer_ip` + allowlist mean for a multi-device setup

Right now we whitelist exactly one peer (the host). If you wanted the
chip to be reachable from a different machine on the Polymath tailnet
(say a robot), add that machine's Tailscale IP to the allowlist via
the same POST endpoint and reboot. The allowlist holds up to 512
entries; only the first one matching `priority_peer_ip` gets the
active handshake — others get the passive treatment but will still
accept handshakes initiated by them.
