# dual_core_safety — robustness + performance goal

The device must be **robust and performant** as a safety-critical networking
client on the Waveshare ESP32-S3-ETH. This is the acceptance gate the work
drives toward.

## Acceptance gate (all must hold on hardware)

| # | Criterion | Verified by |
|---|---|---|
| 1 | **0 crash reboots** (no `reset_reason == 4` = `ESP_RST_PANIC`/abort) across ≥20 reboots and a ≥2 h chaos soak | `tools/robust_test.py`, `tools/chaos_soak.py` |
| 2 | **Tailscale up < 15 s every boot** (`vpn_ip` assigned, DERP connected) | `tools/robust_test.py` |
| 3 | **Eth→USB→WiFi→SoftAP failover** works both directions, `pstop_mismatch ≤ 1` | `tools/chaos_soak.py` + manual cable tests |
| 4 | **Admin/main pages < 250 ms p95** over Ethernet | `tools/robust_test.py` |
| 5 | Running **OTA image marked valid**; rollback chain intact | `/state.json`, panic log |
| 6 | **Cold power-cycle** reaches Tailscale, no loop | manual (bench) |

## Operating rule (safety device)

Validate and diagnose autonomously; **gate every firmware change + OTA on human
review.** The reverted dead-uplink demote (a plausible feature that boot-looped
the chip) is the cautionary tale — measurable validation before shipping, always.

## Backlog (folded into the gate)

- **Crash diagnostics** — sticky last-crash log surviving clean reboots, so the
  next TWDT (esp. cold-boot) is diagnosable.
- **PSTOP unit number** — editable admin field, NVS-backed, default = chip-ID hash.
- **Dead-USB detection (redo)** — a starvation-proof signal that can never
  false-fire and drop a live lease (the bug that caused the boot loop).
- **WG lwIP-lock latency** — reduce the occasional ~150 ms `/state.json` spikes.

## Current status (2026-06-22)

Boot loop **fixed** (dead-uplink demote reverted, `f80cb28`): 8 reboots,
0 panics (`reset_reason=4`), Tailscale up 8–9 s each, page 41 ms / state.json
~15 ms. Root cause was the demote killing connectivity → `net_liveness`'s
lwIP-wedge `abort()` (or an exception in the churn) → panic loop. Gate items
above are the remaining work.
