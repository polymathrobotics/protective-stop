<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Network-Stack Robustness — USB-NCM + W5500 (2026-08-03)

Build `fa8b442`. Hardens the transport under the safety UDP link, driven by the
6 h soak finding that a USB-NCM remote intermittently dropped a heartbeat frame
(lwip `ERR_IF`, `g_sf_txdrv`) and — after enough consecutive drops — opened a
>1.2 s gap that would trip the machine's heartbeat STOP and force an operator
re-arm. Two research passes (USB-NCM + W5500) produced the proposals; the
low-risk, safety-latency-neutral subset was implemented and tested.

## Root cause (USB-NCM)

`sess_sendto()` did a single `sendto()`; on a transient NCM refusal (all IN NTBs
momentarily in-flight waiting for the USB host to poll bulk-IN) it dropped the
frame and waited a full 200 ms tick. On W5500 Ethernet this never happens (the
SPI driver queues the frame and returns OK). It is USB-NCM-specific, under TX load.

## Changes implemented

**USB-NCM**
- **NTB free-list 2 → 4** (`CONFIG_TINYUSB_NCM_IN_NTB_BUFFS_COUNT`, both projects) —
  widens the pool `tud_network_can_xmit()` draws from. +6.4 KB DRAM (heap verified
  fine, ~7.9 MB free).
- **Bounded yielding retry-on-ERR_IF** in `sess_sendto()` — up to 3 retries,
  `vTaskDelay(1)` between, so the lower-priority (5) TinyUSB task can free an NTB;
  the resend lands in the SAME 200 ms tick, no gap. **Gated on `errno == -1`
  (ERR_IF) only** — a genuinely dead link (route / ENOMEM) still fails on the first
  attempt, so **dead-link detection latency is unchanged** (the machine's 1.2 s
  heartbeat STOP and the 1.5 s local rebond threshold are untouched).
- **`pstop_sf_txdrv_recovered`** counter added to `/state.json` — counts transients
  absorbed by the retry, so the effect is measurable.

**W5500 Ethernet** (flawless in the soak; proactive margin only)
- SPI **`input_delay_ns = 20`** — sampling-edge margin (freq limit 40 MHz vs the
  20 MHz clock, so pure signal-integrity headroom, no behavioural change).
- **`LWIP_DHCP_RESTORE_LAST_IP`** — fast, deterministic IP reacquire after a link
  bounce instead of a full re-DHCP.

## Deferred (documented for a fault-injection-tested follow-up)

Higher-value W5500 items from the research that need hardware fault injection
(cable pull, SPI glitch, black-hole) to validate — shipping them untested onto the
currently-flawless Ethernet path was judged the wrong call for safety firmware:

- **W5500 health-watchdog + reset-recovery ladder** (stop/start → RST pulse →
  driver reinstall) for the documented "driver wedges, link still reports UP,
  traffic black-holes, no disconnect event" field failure. *Highest deferred value.*
- **Instant demote-on-link-down** (event-driven failover kick; key failover off
  link state with asymmetric debounce).
- TinyUSB task-priority bump (interacts with lockstep timing); Ethernet TX mutex
  (priority-inversion risk on the safety send); full static IP (deployment choice).

## Test results

**USB-NCM fix — PSTOP54 (USB-NCM, live safety link to machn), 40 min on `fa8b442`:**

| Metric | Result |
|---|---|
| `txdrv` (UNrecovered drops) | **0** |
| `txdrv_recovered` (absorbed by retry) | **5** |
| `pstop_rebonds` (steady-state) | **0** (2 total, both OTA-reboot settling) |
| reply rate (35 min window) | **100 %** (+10,500 / +10,500) |
| free heap | stable ~7.9 MB |

Before the fix (6 h soak): 17 `txdrv` drops + ~9 steady-state rebonds. After: 0
drops, 0 steady-state rebonds, 5 transients cleanly recovered. The nuisance-stop
precursor is eliminated, and because the retry is ERR_IF-gated the dead-link STOP
latency is unchanged.

**Armed safety-function regression — DUT (`0x01d7f344`) via HIL relays on `fa8b442`:**

| Step | Result |
|---|---|
| loops open (pressed) | both-phase sampling detects open (e_hi → 0) |
| arm (press → release) | machine → **OK (armed)** |
| press while armed | machine → **STOP** |
| re-arm | machine → **OK (armed)** |
| ~105 s armed hold | **no spurious STOP** |

The transport changes did not regress the safety logic: arm / STOP / re-arm all
correct, and every STOP transition mapped to a deliberate press.

## Fleet

`fa8b442` uploaded to the fleet as the verified build; all reachable units
(PSTOP54, PSTOP42, DUT, machn) already OTA-updated; offline units adopt it on
next check-in.
