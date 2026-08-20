<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: Apache-2.0
-->

# Monitoring field glossary

Semantics of the diagnostics surfaced by `/state.json` and `/admin/api/monitor`.
This file exists because four separate bench incidents produced WRONG
mid-incident conclusions from misread fields — each trap below cost real
debugging time before a source-check corrected it. **Rule: verify a field's
semantics here (or in source) before building a monitor or a conclusion on it.**

## Traps (read these first)

| Field | Trap |
|-------|------|
| `pp_*` (`pp_best_ip`, `pp_endpoint_count`, `pp_has_direct`, …) | Describe the peer at `config.priority_peer_ip` — on bench remotes that is NOT the machine link. All-zero `pp_*` on a healthily direct-bonded remote is NORMAL. Do not use as machine-link evidence. |
| `direct_regains` | GLOBAL counter across all peers, not per-peer. A regain burst says nothing about WHICH peer regained. |
| `wg_stall_events` / `derp_stall_events` | 8-deep RINGS. `len == 8` means "full", not "8 total" — check the newest entry's `at_s` for freshness; old entries rotate out silently. |
| `rx_worst_gap_ms` / `tx_worst_gap_ms` | Since-boot high-water marks that usually SATURATE during the boot/flash reconvergence window. Always read the paired `*_at_s` before treating a large value as a live event. |
| `nvs_flush_count` | NVS housekeeping commits — NOT network/conn flushes. |
| `probe_tbl_hw` | Since-boot high-water of disco probe-table occupancy. Records the size of the REGION that overflowed: 48 = general region full (safety reserve untouched), 64 = true exhaustion including the reserve. |
| `rtt_ms` (pstop machine slots) | Loop round-trip of the LAST reply — a delayed burst can show tens of seconds without the link being down. Not a path-health signal on its own. |
| `silence_resume_worst_ms` | Check `silence_resume_at_s` — boot-window values saturate it. Pairing is per-safety-peer (8 slots). |

## Crash / reset visibility (`/state.json`)

| Field | Meaning |
|-------|---------|
| `reset_reason` | `esp_reset_reason()` of this boot (3 = SW, 5 = INT_WDT, …). |
| `ctrl_reset_cause` | One-shot crumb: this boot followed `dcs_controlled_reset()` — 1 clock-fault, 2 XCHECK, 3 pad-cfg. Zero unless `reset_reason` is SW. |
| `xcheck_last_detail` | `(fault_code << 4) \| stalled_core`, rides only an XCHECK-labeled boot. Fault codes: 1 peer task/heartbeat stalled, 2 peer clock frozen/backward. |
| `crash_present` / `crash_pc` / `crash_task` / `crash_sha` | Boot-time summary of the coredump in flash (persists until the NEXT crash — vintage may predate this boot's build; `crash_sha` names the crashing build, match it against `ops/elf-archive/`). |
| `log_lines_s` / `log_lines_s_peak` / `log_console_skipped` | ESP_LOG line rate (last full second / since-boot peak / console lines dropped by the 40-line/s budget). Storm context: 147 lines/s was measured at a core-stall; UART0 drains ~115 lines/s at 100 chars — the budget exists so logging can never block a core again. |

## Path / DERP diagnostics (`/admin/api/monitor`)

| Field | Meaning |
|-------|---------|
| `derp_reconnects` + `derp_reconnect_last_ms`/`_worst_ms` | HOME-conn reconnect count and durations (RST-to-reconnected). Aux churn is not counted here. |
| `derp_reconn_causes` `[5]` | Home-reconnect attribution: `[tx write fail, rx read error, tx-active/rx-silent reap, interface rebind, region rehome]`. Sum should track `derp_reconnects`; a persistent shortfall means an unattributed trigger site. |
| `derp_kicks_spaced` | Non-home connect kicks refused by the 1.5 s spacing gate (flush-storm damping). The home slot and the pool-dark rescue kick bypass refusal. |
| `peer_table_full_count` | Over-cap peer-add refusals (128-slot table vs the tailnet). Counted always; the WARN log is rate-limited to 1/10 s. Refusals are zero-work: they do not consume add-pacing slots. |
| `disco_silence_pings` / `silence_resume_last_ms`/`_worst_ms`/`_at_s` | rx-silence ping-now machinery: pings sent to direct-silent safety peers, and the last/worst ping→direct-pong resume delta (≈ RTT when healthy). |
| `readd_skipfail[7]` | Why a peer re-add could not be skipped: `[!existing, no-wg-slot, vpn_ip, disco_key, hostname, region, endpoints]`. |

## Wire-truth companion

Firmware self-reported network metrics are not wire evidence (bench rule since
2026-08-09). `tools/udp_canary.py` is the independent reference: run it on a
host behind the same edge; `CANARY_REBIND` = the public mapping changed (NAT
rebind / multi-WAN failover — the run-46 root cause), `CANARY_GAP_*` = the
edge's UDP state was actually down. Correlate its millisecond stamps against
the device gauges above before attributing an event to firmware.
