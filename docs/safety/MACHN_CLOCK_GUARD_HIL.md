<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# machn Clock-Guard — Hardware-in-the-Loop Validation

**SR-H-04b (machine side) / FMEA DU-2.** Closes the deferred "machn clock-guard
HW validation (needs live machine node + relay HIL)" item. Performed 2026-08-02
against the live machine node **`machn-01d77a1c`** (tailnet `100.84.155.111`, LAN
`192.168.107.192`), an ESP32-S3 driving two series safety relays with feedback.

## What was validated

The independent-timebase clock-sanity guard added in `fe29fa1`
(`machn/main/main.c` — esp_timer vs FreeRTOS-tick cross-check, TWDT/IWDT/RTC-WDT):
a frozen/backward `esp_timer` would blind every heartbeat timeout (a dead remote
would look alive → relays stay energized), so on a detected clock fault the guard
opens both relays, forces STOP on both cores, and does a controlled reset.

## Method

The unit was found running `6e7492f` (2026-08-01) — the commit **immediately
before** the guard, and the guard is the *only* change to `machn/main/main.c`
since. So the guard had **never run on this hardware**. It was built from HEAD
(`3dcd08d`, IDF v5.5.4), the guard code was confirmed present in the ELF
(`CLOCK FAULT: esp_timer frozen/backward…` strings), and deployed by OTA
(`POST /admin/api/ota`, 1.38 MB) to the A/B partition (`ota_1`→`ota_0`, valid-mark
rollback as the safety net). State observed via the unit's `/state.json`.

## Results

**Deployment + clean boot.** OTA accepted, unit rebooted into `3dcd08d` on `ota_0`,
`ota_state=VALID`, `rollback_occurred=false`. Boot was clean — `reset_reason=3`
(the OTA reboot), `boot_count=0`, cores in lockstep, and **no clock-fault trip**.

**Sustained non-false-trip (the core question).** Watched ~5.5 min. Uptime climbed
monotonically (9 s → 339 s), `boot_count` stayed 0, `reset_reason`/`rst_hist`
showed no new fault-reset, cores agreed every sample (`t0==t1`, `l0==l1`),
`pstop_mismatch=0`, relay feedback healthy (`relay_fault_*=0`). The guard runs
every tick and did **not** false-trip on real silicon. The bench remote
(`0x01d7eed0`) re-bonded on its own — normal comms unaffected.

**Functional safety function under the guard** (soft remote `0x01AA0001` →
machn `:8890`, machine_id `0x01020304`, bench `allow_unlisted`):

| Step | l0 | l1 | e_hi0 | relay_stop | verdict |
|---|---|---|---|---|---|
| baseline (bonded) | 0 | 0 | 0 | 0 | STOP |
| arm (hold STOP, release OK) | 1 | 1 | 1 | 0 | **RUN after 0.53 s** (= min-stop delay) |
| STOP asserted | 0 | 0 | 0 | 0 | **STOP, relays open** |
| re-arm | 1 | 1 | 1 | 0 | **RUN after 0.54 s** |
| soft-remote unbond | 0 | 0 | 0 | 0 | safe STOP |

Dual-core lockstep agreed on every transition; the real series relays energized
and opened with correct feedback; `boot_count`/`mismatch` stayed 0. The guard does
not interfere with arming, stopping, or re-arming.

## Scope / honesty

Injecting a genuine `esp_timer` freeze is not possible on unmodified silicon, so
the guard's **detection** of an actual freeze was not exercised here. That
detection was already fault-injection-proven on the functionally identical remote
cross-check (`347de6f`) and on the host `clock_guard` unit tests
(`host/test_clock_guard.c`, 5117 checks). This HIL run validates the remaining
open question: that the guard-bearing build **runs on real machn hardware, boots
clean, does not false-trip under sustained normal operation, and leaves the full
safety function intact**.

## Post-state

The machn is left running the current guard build `3dcd08d` (an improvement over
the pre-guard `6e7492f` it shipped with), in fail-safe STOP awaiting a normal
arming gesture from its bench remote.
