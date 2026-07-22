# Testing

How to exercise the pstop link, the machine's arming policy, and the
remote's robustness. All tools are stdlib-Python or bash + `curl`/`jq`;
nothing needs the chip's IDE. Bench IPs are environment-specific — every
script takes them as environment variables or flags.

## 1. Machine arming-policy suite — `tools/pstop_test_remote.py`

A scripted pstop remote that speaks the real wire protocol (40-byte
messages, CRC16 poly 0x8D95, pstop_c @ d9f4970) well enough to bond and
run timed STOP/OK sequences — verifies the wrapper's min-STOP-duration
arming policy without a chip. Run it against a **dedicated** runner
instance (never the one bonded to a real remote):

```sh
cd host && make
./machine_app_runner machine.toml 8893 &      # dedicated port
python3 ../tools/pstop_test_remote.py --port 8893
```

Asserts (exit 0 = all pass):
1. bond + OK stream → machine replies STOP (NEED_STOP, nothing armed yet)
2. 200 ms STOP blip → arming VETOED (replies stay STOP)
3. 800 ms STOP press → ARMS (replies turn OK)
4. blip while armed → robot stops, short release does NOT re-arm
5. press again → re-arms

Run this after any change to `host/machine_app_runner.c` or a pstop_c
submodule bump (it will catch a CRC/wire break immediately).

## 1b. Many-remotes-to-one-machine matrix — `tools/pstop_multi_remote_test.py`

Drives N scripted software remotes (each its own device_id + counter handshake)
against a dedicated `machine_app_runner` to validate the many-to-one logic
across the corner-case matrix: independent arming, the any-STOP OR, arming
ownership, heartbeat loss, unbond, capacity overflow, allowlist deny, stop-only
operators, malformed/bad-CRC/invalid-type traffic, and network chaos (via §2).
32 assertions covering the fail-safe invariants (I1–I7).

```sh
python3 tools/pstop_multi_remote_test.py     # exit 0 = all invariants held
```

Full write-up + invariant list: `MULTI_REMOTE_VALIDATION_2026-07-22.md`.

## 2. Protocol-level impairment — `tools/pstop_chaos_proxy.py`

UDP proxy interposed in the pstop path
(chip → proxy:8891 → machine:8890) that injects loss / delay / jitter
(reordering) / duplication / corruption per direction, re-tunable at
runtime via a UDP control port (8892) without re-bonding:

```sh
python3 tools/pstop_chaos_proxy.py --listen 8891 --fwd 8890 --ctrl 8892
curl -X POST "http://$CHIP/api/pstop_peer?ip=<proxy-host>&port=8891"
echo 'loss=0.05 delay_ms=100 jitter_ms=50' > /dev/udp/127.0.0.1/8892
echo 'clear=1'                             > /dev/udp/127.0.0.1/8892
```

Direction-specific overrides: `loss_up` (chip→machine), `loss_down`,
same suffixes for `delay_ms`/`jitter_ms`/`dup`/`corrupt`. Stats print
every 10 s.

## 3. Chaos ladder — `test/chaos_ladder.sh`

Sweeps the proxy through the standard impairment ladder (baseline,
loss 1→30 %, delay 50→500 ms, jitter-reorder, dup, corrupt, combo, then
0.5/1/2/5 s full outages) while sampling the chip's `/state.json` every
10 s. Requires the proxy from §2 running and the machine ARMED (any
unscheduled STOP in the logs = false trip).

```sh
CHIP=<chip-ip> [CTRL_HOST=127.0.0.1] [CTRL_PORT=8892] test/chaos_ladder.sh
```

Outputs (next to the script): `chaos.csv` (phase-stamped samples),
`phases.log` (phase markers); machine transitions land in the runner's
stderr, proxy stats in its stdout.

Pass criteria (measured baseline, 2026-07-20): zero false STOPs, zero
rebonds, zero crashes (`boot_count` stays 0) through the entire ladder
up to 30 %/direction loss + 500 ms delay + 20 % dup + 2 % corruption;
outages <1 s ride through; outages ≥1 s STOP on the heartbeat timeout
and re-bond automatically, with arming requiring a fresh press.

## 4. WG-underlay ladder — `test/netem_ladder.sh`

tc/netem on the *encrypted* WireGuard flows (UDP 51820) of the bench
interface — validates the tunnel layer itself: loss, delay, a 5-min
direct-path blackhole (must fail over to DERP), restore (must return to
the direct path). Self-installing and self-verifying: it logs netem
qdisc packet counters per phase to PROVE impairment applied, because the
tether interface silently loses all qdiscs on every chip reboot /
USB re-enumeration.

```sh
IF=<bench-iface> CHIP=<chip-ip> CHIP_TS=<chip-tailscale-ip> test/netem_ladder.sh
```

Needs sudo (tc/ifb). NOTE: the script's `arm()` step uses the
`/api/pstop_sim` pulse, which exists only in test builds — on production
firmware (pstop_sim removed) it 404s harmlessly; arm with a physical
press of the E-stop switch instead.

Baselines (2026-07-20/21): 10 %/dir underlay loss → ~19 % reply loss, no
false STOP; blackhole → STOP on timeout, heartbeats resume via DERP at
~201 ms RTT; dark window ≤10 s with the priority-peer pong watchdog
(measured 9.3 s; was 10–66 s); direct path back within one sample of
restore.

## 5. Soak methodology

A transport claim needs, per transport (Ethernet / USB-NCM / WiFi):

1. **30–40 min steady-state soak** at 10 Hz over Tailscale, machine
   armed. Watch `/state.json`: `pstop_sent`/`pstop_replies` ratio,
   `pstop_rebonds`, `pstop_mismatch`, `boot_count`, `heap_min_int`.
2. The §3 protocol ladder and §4 underlay ladder.
3. A wire-level audit: tcpdump the underlay and confirm no plaintext
   application ports (this is what caught the critical plaintext
   downgrade — see `archive/CHAOS_RESULTS_2026-07-20.md`).

Measured baselines to compare against
(`archive/TRANSPORT_TEST_REPORT_2026-07-20.md`):

| Metric | USB-NCM | Ethernet (PoE) | WiFi (PS off) |
|---|---|---|---|
| Soak reply rate | 99.983 % | 100.000 % | 98.09 % |
| Rebonds / false stops | 0 / 0 | 0 / 0 | ~1 burst per 10–15 min (fail-safe) |
| Steady-state pstop RTT over tunnel | ~5 ms | ~5 ms | tens of ms, spiky |
| Crashes over the whole campaign | 0 | 0 | 0 |

Long-run harnesses in `tools/` (edit the IP constants at the top —
they are bench snapshots): `tools/chaos_soak.py` (reachability +
telemetry probe at 2 s with injected chaos events; reboot-gated so it
never trips the rapid-boot ladder) and `tools/robust_test.py` (repeated
reboot: Tailscale-up time, crash-free boots, admin-page latency).

## 6. Legacy scripts

`test/longsoak.sh`, `test/test_suite.sh`, `test/full_test_after_recovery.sh`,
`test/aggressive_ota.sh`, `test/auto_recover.sh` are v15-era bench
scripts kept for reference; they predate the current firmware and still
assume old bench addresses/flows. Prefer the tools above.
