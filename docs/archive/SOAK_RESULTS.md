# dual_core_safety — chaos/soak stability results

Board: Waveshare ESP32-S3-ETH (ESP32-S3R8, W5500 Ethernet, 8 MB octal PSRAM).
Branch: `dcs-w5500-ethernet`. Harness: [`tools/chaos_soak.py`](../tools/chaos_soak.py).

## Run: 2026-06-22, 1 hour, 38 chaos events

Device on Ethernet + Tailscale; host probes reachability + telemetry every 2 s
while the harness injects chaos. Raw per-sample data: `soak-2026-06-22.csv`
(1621 rows), full log: `soak-2026-06-22.log`.

### Reliability — VERDICT: PASS

| metric | result |
|---|---|
| Unexpected reboots / crashes | **0** (9 reboots, all intentional, all recovered) |
| Reachability (HTTP /state.json) | **100.00 %** (1621/1621) |
| Longest outage | **0 s** — reboots recover HTTP-responsive in ~3 s (within one 2 s probe + 4 s timeout) |
| pstop rebonds (safety-link re-syncs) | **+0** over the whole hour |
| Lockstep mismatches (`pstop_mismatch`) | **max 1** (one transient in 60 min) |
| Internal heap floor (`heap_min_int`) | **28 KB** (worst dip during a WiFi-failover + DERP reconnect; no leak, no OOM) |
| Tailscale | healthy throughout (`derp_delay=1 ms`), carried over both Ethernet and WiFi |

### ICMP ping latency

| window | n | p50 | p95 | p99 | max |
|---|---|---|---|---|---|
| All (incl. chaos transitions) | 1621 | 5.1 ms | 122 ms | 214 ms | 267 ms |
| Steady (Ethernet active) | 1508 | 5.0 ms | 114 ms | 176 ms | 233 ms |
| During WiFi failover | 113 | 19.1 ms | 214 ms | 230 ms | 267 ms |

Note: the host reaches the device over its own WiFi link (laptop on `prime` 5 GHz →
LAN → device W5500), so host-side WiFi jitter is in every RTT sample. See the
steady-state tail isolation below.

### Chaos coverage (every event recovered)

9× reboot · 6× Ethernet drop 12 s · 4× Ethernet drop 40 s (→ WiFi auto-failover)
· 4× cable flap · 4× USB-NCM toggle · 5× WireGuard pause · 5× manual WiFi enable.
The supervisor failed over to WiFi and back on every uplink loss; reboots stayed
spaced so the rapid-boot rollback never tripped.

## Steady-state ping-tail isolation

The soak's steady-state ICMP tail (p95 ≈ 114 ms) looked high, so a clean 3-minute
test (no chaos, device idle on Ethernet) pinged the **gateway (wired router)** and
the **device** in parallel from the host, at 2 Hz:

| target | n | loss | p50 | p95 | p99 | max |
|---|---|---|---|---|---|---|
| gateway `192.168.107.1` (wired router) | 360 | 0 | 35.5 ms | 152 ms | 214 ms | 450 ms |
| device `192.168.107.101` (W5500) | 360 | 0 | 41.2 ms | 154 ms | 208 ms | 449 ms |

The gateway and the device have **effectively identical tails** — so the latency
is the **host laptop's WiFi link to the router** (the host reaches both targets
over WiFi/`prime`), not the device. The device adds only ~5 ms of median and **no
tail of its own**.

Confirmed device-side: the device's own wired gateway ping (`net_liveness`
GW_PING) is **`time=0 ms`** (sub-millisecond) — its W5500 Ethernet + lwIP path is
already at the floor.

**Conclusion:** there is no device-side change that lowers this tail; it is the
measurement path (host WiFi). For a true device-latency figure, measure from a
wired host or read the device's sub-ms GW_PING. The safety path is unaffected —
`pstop_rebonds=0` and `pstop_mismatch ≤ 1` across the entire soak.
