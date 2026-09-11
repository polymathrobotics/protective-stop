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

## Lifetime health / wear (`/api/health`, `health` in `/state.json`)

Counters that survive reboot, `idf.py flash` and OTA (NVS blob
`dcs_app/health`). Purpose: decide when a unit is retired. Full field list in
[`API.md`](API.md#health-lifetime-counters-and-warnings).

| Field | Meaning |
|-------|---------|
| `health` (`/state.json`) / `level` | Worst active warning: 0 ok, 1 warn, 2 critical. Poll this one field fleet-wide; open `/api/health` only when it is nonzero. |
| `presses` | Physical stop operations: rising edges of BOTH cores reading STOP. Compare against `button_rated_ops` (NKK FF01: 100,000). |
| `mismatch_events` | Rising edges of the two cores disagreeing on the same tick (one loop open, the other closed). The leading indicator of switch/wiring wear; it climbs before `presses` gets anywhere near the rating. Distinct from `pstop_mismatch` in `/state.json`, which is this boot's comparator count including publish timeouts. |
| `uptime_s` | Cumulative powered seconds. Trap: lags live time by up to `CONFIG_DCS_HEALTH_FLUSH_MAX_S` (600 s) after a power pull. Not wall-clock; no SNTP involved. |
| `boots` | Lifetime boot count. Trap: NOT `boot_count` in `/state.json`, which is the crash-ladder counter and is zeroed after 120 s of healthy uptime. |
| `flashes` / `otas` | Boots on which the image SHA changed / on which the image was OTA-delivered. `flashes - otas` = cable flashes. A brand-new unit shows `flashes: 1`. |
| `nvs_flushes` / `last_flush_age_s` | This boot's successful blob writes and the seconds since the last one. `last_flush_age_s` stuck above 600 with `nvs_flush_fails` climbing = NVS trouble. |
| `warnings[]` | One entry per publishing source (`src`), with `count` (re-publishes) and `age_s` (since first raised this boot). Built-in: `button_wear`, `loop_mismatch`. |

**Retire when:** `level` is 2 (button at or past rated life), or `loop_mismatch`
persists after the wiring has been inspected and reseated. **Data loss:** an
`erase-flash`, a partition-table change that moves NVS, or the boot-time
NVS auto-erase on `ESP_ERR_NVS_NO_FREE_PAGES`/`NEW_VERSION_FOUND` (an IDF
major upgrade) zeroes everything; record `/api/health` before those.

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
