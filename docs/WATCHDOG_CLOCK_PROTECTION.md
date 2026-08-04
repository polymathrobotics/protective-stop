# Watchdog + Dual-Core Clock Cross-Check — SR-H-04 / FMEA DU-2

**Requirement:** SR-H-04 — the device must detect a frozen clock or a frozen /
wedged task (FMEA hazard **DU-2**, a *dangerous-undetected* fault) and drive the
system to the fail-safe state (**STOP + reset**).

**Branch:** `feat/watchdog-clock-crosscheck`
**Target:** ESP32-S3, ESP-IDF v5.5
**Validated on:** local unit `10.42.0.41` (pstop-01d7eed0), OTA-only, fault
injection reverted.

---

## 1. Why the existing lockstep is not enough

The device already runs two safety tasks pinned to core 0 and core 1 in
lockstep: each encodes the same `pstop_msg_t` and a comparator byte-compares the
two encodings, transmitting only on agreement (`main.c`). That design already
fail-safes several faults:

- **A safety task that wedges mid-tick** stops publishing its encoding →
  the comparator's `both_in` check fails → nothing is sent → every machine's
  heartbeat times out → STOP.
- **RAM / cache / ALU corruption during encoding** → the two encodings differ →
  `memcmp` mismatch → send nothing → STOP.

What it does **not** catch is the **DU-2 frozen-clock** case: if
`esp_timer_get_time()` freezes while the FreeRTOS scheduler keeps running, both
cores keep looping and keep producing **byte-identical** encodings (they read the
same frozen clock), so `pstop_mismatch` stays 0. But the per-session counter
still advances on every transmitting tick, so the machine keeps *accepting* the
heartbeat even though the message `stamp` (taken from `esp_timer`) is frozen —
the robot keeps moving on a stale-timestamp OK. That is a dangerous *undetected*
fault. This work closes it with two layers: the built-in watchdogs (recovery /
backstop) and a software dual-core mutual clock cross-check (fast detection).

---

## 2. Built-in watchdogs (`firmware/sdkconfig.defaults`)

Researched against the ESP-IDF v5.5 ESP32-S3 watchdog documentation
([Watchdogs — ESP-IDF v5.5, ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/system/wdts.html)
and [Startup / RTC WDT](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-guides/startup.html)).

### 2.1 Task Watchdog Timer (TWDT)

| Kconfig | Value | Rationale |
|---|---|---|
| `CONFIG_ESP_TASK_WDT_EN` | `y` | Enable the TWDT. |
| `CONFIG_ESP_TASK_WDT_INIT` | `y` | Auto-init at startup and subscribe both idle tasks, so a safety task that *spins* (never yields) starves its core's idle task and trips the WDT. |
| `CONFIG_ESP_TASK_WDT_PANIC` | `y` | On timeout, **panic → reset** (reset reason `ESP_RST_TASK_WDT`) instead of only printing a warning. That reset reason is already counted toward the OTA rollback ladder in `dcs_safety.c`, so a persistent wedge eventually rolls firmware back. |
| `CONFIG_ESP_TASK_WDT_TIMEOUT_S` | `5` | See timeout rationale below. |
| `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/CPU1` | `y` | Watch idle on both cores. |

**Subscribers (explicit, in `main.c`):** the two lockstep core tasks and the
comparator each call `esp_task_wdt_add()` and `esp_task_wdt_reset()` once per
100 ms tick.

- The cores subscribe on their **first** notification (not at creation): the
  comparator delays ~5 s at boot before it starts notifying, and a
  subscribed-but-unfed task would otherwise trip the 5 s TWDT during that window.
- Because a core feeds only when the comparator notifies it, a **wedged
  comparator** also stops the cores' feed → TWDT reset — a second, independent
  path to catching a comparator wedge.
- The pre-existing `app_heartbeat_task` (`dcs_safety.c`) remains a TWDT
  subscriber, feeding every 1 s.

**Timeout rationale (5 s):** The *fast* safety reactions are the 0.5 s clock
cross-check (§3) and the 1 s machine heartbeat timeout — those bring the system
to STOP long before the TWDT. The TWDT's job is the **recovery reset** of a
genuinely wedged task. 5 s is 50 missed 100 ms feeds of margin — short enough to
recover a wedge promptly, long enough that a transient TLS / WireGuard
preemption burst does not cause a false reset. (The previous value was 30 s,
tuned only as a coarse hang-catcher; 5 s makes it safety-relevant without being
twitchy. The safety tasks themselves block/yield, so tightening the idle-task
timeout does not risk false trips from our own code.)

### 2.2 Interrupt Watchdog (IWDT)

| Kconfig | Value | Rationale |
|---|---|---|
| `CONFIG_ESP_INT_WDT` | `y` | Enable the IWDT. |
| `CONFIG_ESP_INT_WDT_TIMEOUT_MS` | `300` | IDF default; comfortably longer than any critical section on the safety path (the E-stop loop uses only two `esp_rom_delay_us(10)` settles). |

The IWDT is **fed by the FreeRTOS tick ISR**. It catches interrupts being
blocked too long — a runaway ISR or an over-long critical section (e.g. a lwIP
core-lock section, a wedged WG-crypto path). Two important properties:

1. It also catches a **total systimer freeze**: if the systimer stops, the
   FreeRTOS tick interrupt stops firing, the IWDT stops being fed, and it fires.
2. Its two-stage design — stage 1 invokes the panic handler; **stage 2 is wired
   to the RTC watchdog**, which hard-resets the chip if the panic handler itself
   hangs.

### 2.3 RTC (main system) Watchdog

| Kconfig | Value | Rationale |
|---|---|---|
| `CONFIG_BOOTLOADER_WDT_ENABLE` | `y` | Protects the **boot path** — a stalled boot auto-resets and retries. |

At runtime the RTC WDT is the **INT-WDT's stage-2 hard-reset backstop** (§2.2),
which is the ultimate last resort on the S3. Note: the user-code `rtc_wdt_feed()`
API (`rtc_wdt.h`) is guarded to **ESP32 / ESP32-S2 only** — there is no public
manual RTC-WDT feed on the ESP32-S3 — so keeping the RTC WDT running in user code
with an app-side feed loop is not an option here. The INT-WDT / RTC-WDT coupling
provides the equivalent independent hardware backstop instead.

### Watchdog coverage summary

| Fault | Caught by |
|---|---|
| Safety task spins (never yields) | TWDT (idle starvation) → reset |
| Safety task blocks forever | TWDT (explicit subscriber stops feeding) → reset; **and** existing lockstep `both_in` → STOP |
| ISR / critical section too long | IWDT → panic → reset |
| Systimer totally freezes (scheduler dead) | IWDT (tick ISR stops feeding it) → reset |
| Panic handler itself hangs | RTC WDT (INT-WDT stage 2) → hard reset |
| Stalled boot | Bootloader RTC WDT → reset + retry |

---

## 3. Dual-core mutual clock cross-check (`firmware/main/main.c`)

The software layer that specifically closes the DU-2 frozen-clock gap while the
scheduler is still alive.

### Mechanism

Each core, **every tick**, publishes two independent liveness signals for the
*other* core to observe (`xcheck_publish_and_verify`):

- **`hb`** — a heartbeat counter this core increments itself each loop pass.
  *Task-liveness*: freezes iff this core's task stops running.
- **`us`** — an independent `esp_timer_get_time()` reading.
  *Clock-liveness*: freezes iff `esp_timer` stops advancing.

Each core then verifies the **peer's** two signals are advancing, **gating on
`xTaskGetTickCount()`** — deliberately a *different* time base from `esp_timer`,
so a frozen `esp_timer` (which also freezes this core's own `esp_timer` reads) is
still measurable against the advancing scheduler tick. Two **independent** stall
timers:

- peer `hb` frozen > `XCHECK_STALL_TICKS` (500 ms) → **`FAULT_PEER_HB`** (peer task wedged)
- peer `us` frozen 500 ms, or ever moves backward → **`FAULT_PEER_CLK`** (peer clock frozen/backward)

Checking them **separately** is the whole point: if `esp_timer` freezes while the
task still loops, `hb` keeps advancing (no HB fault — the task *is* alive) but
`us` freezes → the clock timer fires. This is precisely the case the lockstep
`memcmp` cannot see.

### Timeout rationale (500 ms)

At 10 Hz each core should advance both signals every 100 ms; 500 ms is 5 missed
ticks of tolerance — immune to the normal scheduling jitter that also underlies
the 80 ms core-publish window — while still well under the 1 s machine heartbeat
timeout and the 5 s TWDT, so the cross-check is the *first* line to react.

### Fail-safe reaction

The detecting core latches `g_xcheck_fault` (a single `atomic_compare_exchange`,
lock-free). The comparator polls it at the top of every tick and, on fault:

1. **Halts all transmission** — sends nothing to any machine, so every session's
   heartbeat times out and the machine goes to **STOP** (the primary safety
   reaction).
2. Publishes the fault to `/state.json`, keeps feeding the TWDT so *our*
   controlled sequence stays in charge.
3. After a `XCHECK_RESET_GRACE_MS` (500 ms) settle — measured in **FreeRTOS
   ticks, not `esp_timer`**, since the fault may *be* a frozen `esp_timer` — does
   a **controlled `esp_restart()`** for recovery.

`esp_restart()` is a *clean* reset (reason `ESP_RST_SW`), deliberately **not**
counted toward the OTA rollback ladder: a frozen clock is a hardware/silicon-
level fault that a firmware rollback cannot fix, and while any such fault
persists the device simply restarts and stays silent — every machine stays
safely STOPped throughout. (A wedge that instead manifests as a task not feeding
the TWDT *does* reset via `ESP_RST_TASK_WDT`, which *is* counted — so genuine
firmware wedges still escalate to rollback.)

### Concurrency

`g_xcheck_hb[]` / `g_xcheck_us[]` are single-writer-per-core atomics (same
discipline as `g_tick_roll`); the per-observer memory is private to each core
task. A torn read across the un-paired `hb`/`us` atomics can only *delay*
detection by one tick, never cause a false fault, because each signal has its own
500 ms stall timer.

### Telemetry

`/state.json` gains `xcheck_fault` (0 healthy / 1 task-stall / 2 clock-frozen),
`xcheck_hb0`, `xcheck_hb1` — published every tick, so the advancing heartbeats
are positive evidence the cross-check is running, not just a fault flag.

---

## 4. Test & fault-injection evidence (`10.42.0.41`, OTA)

**Injection (throwaway build, reverted):** after ~15 s of uptime, core 1 keeps
looping (its `hb` keeps advancing) but publishes a **frozen** `esp_timer` stamp —
the exact frozen-clock DU-2 case. Core 0 must raise `FAULT_PEER_CLK`.

`/state.json` polled at 0.7 s cadence (fields: uptime_ms, reset_reason, xfault,
hb0, hb1, t0, t1, pstop_mismatch):

```
uptime=27588  rr=3 xfault=0 hb0=132 hb1=132 t0=133 t1=132 mism=0   healthy, in lockstep
uptime=28729  rr=3 xfault=0 hb0=142 hb1=142 t0=143 t1=143 mism=0
uptime=29570  rr=3 xfault=0 hb0=152 hb1=152 t0=152 t1=152 mism=0   hb>150 → core1 clock frozen
uptime=30391  rr=3 xfault=2 hb0=158 hb1=158 t0=158 t1=158 mism=0   FAULT_PEER_CLK detected
   (no response — controlled reset firing)
   (no response)
   (no response)
uptime=10713  rr=3 xfault=0 hb0=0   hb1=0   t0=0   t1=0   mism=0   back up, clean SW reset, boot_count=0
```

Key proof points:

- **`xcheck_fault` → 2 (`FAULT_PEER_CLK`)** the moment core 1's clock froze,
  while **`hb1` kept advancing (158)** — confirming detection of a frozen clock
  on a task that is *still running* (the DU-2 signature).
- **`pstop_mismatch` stayed 0** the entire time — the existing lockstep
  byte-compare **never** saw the frozen clock. Only the new cross-check caught
  it. This is the coverage gap being closed, demonstrated directly.
- **Controlled reset fired** (device went unresponsive, then returned with
  uptime reset). `reset_reason = 3` (`ESP_RST_SW`, clean) and `boot_count = 0`,
  exactly as designed — the fault path does not churn the rollback ladder.

**Injection reverted → clean build stable past the freeze point:**

```
uptime=14388  xfault=0 hb0=0   hb1=0   t0=0   t1=0   mism=0 boot=0
uptime=19432  xfault=0 hb0=50  hb1=50  t0=51  t1=51  mism=0 boot=0
uptime=24523  xfault=0 hb0=101 hb1=101 t0=102 t1=102 mism=0 boot=0
uptime=29615  xfault=0 hb0=152 hb1=152 t0=153 t1=153 mism=0 boot=0   past old freeze point
uptime=34659  xfault=0 hb0=203 hb1=203 t0=203 t1=203 mism=0 boot=0   no reset, healthy
```

The heartbeats advance in lockstep (`hb0 == hb1`, `t0 == t1`), `xfault` stays 0,
and the device runs continuously past the old 150-tick freeze point — proving the
cross-check runs every tick and does not false-trip in healthy operation.

---

## 5. Residual coverage

| Fault | Detected by | Notes |
|---|---|---|
| One core's `esp_timer` clock frozen, scheduler alive | **Cross-check** `FAULT_PEER_CLK` → STOP + reset | The DU-2 case the lockstep misses; proven in §4. |
| One core's task heartbeat frozen, scheduler alive | **Cross-check** `FAULT_PEER_HB` → STOP + reset; **also** lockstep `both_in` → STOP | Two independent detections. |
| Both cores' `esp_timer` frozen simultaneously (single shared `esp_timer`) | **Cross-check** — each core sees the *other's* published `us` stop advancing against the FreeRTOS-tick gate → fault | The gate is a different time base, so a shared-clock freeze is still caught **as long as the scheduler runs**. |
| **Total systimer freeze → scheduler stops** | **Hardware WDTs only** | The cores stop running, so the software cross-check cannot execute. The FreeRTOS tick ISR stops → IWDT (300 ms) is no longer fed → reset; the app_hb / core TWDT feeds also stop → TWDT (5 s) → reset; RTC WDT (INT-WDT stage 2) hard-resets if the panic handler hangs. This is the documented residual: the software layer degrades to the hardware layer. |
| ISR / critical section hang | IWDT | §2.2 |
| Panic handler hang | RTC WDT (INT-WDT stage 2) | §2.3 |

**Bottom line:** while the scheduler runs, the dual-core cross-check gives *fast*
(0.5 s) fail-safe STOP + reset for a frozen clock or frozen task, including the
DU-2 case the lockstep cannot see. When the scheduler itself dies, the built-in
hardware watchdogs (IWDT → TWDT → RTC WDT) are the backstop. Every layer's
reaction is fail-safe: STOP (via halted TX / heartbeat timeout) plus a reset.

---

## 6. How this discharges SR-H-04 / DU-2

- **Detection of a frozen clock:** cross-check `FAULT_PEER_CLK`, proven to fire
  while the existing lockstep (`pstop_mismatch`) stays 0 (§4).
- **Detection of a frozen / wedged task:** cross-check `FAULT_PEER_HB` +
  lockstep `both_in` + TWDT.
- **Fail-safe reaction = STOP + reset:** the comparator halts all TX (machines
  heartbeat-time-out to STOP) and does a controlled reset; the hardware
  watchdogs reset the chip when the scheduler is gone.
- **No dangerous-undetected residual while the scheduler runs;** the only
  residual (total systimer / scheduler death) is covered by the hardware
  watchdogs, documented in §5.

## Files changed

- `firmware/sdkconfig.defaults` — TWDT / IWDT / bootloader-RTC-WDT configuration.
- `firmware/main/main.c` — dual-core clock cross-check + TWDT subscribe/feed on
  the cores and comparator + fault reaction.
- `firmware/components/dcs_support/src/dcs_support.c`,
  `.../src/dcs_internal.h`, `.../include/dcs_support.h` — `dcs_publish_xcheck`
  sink + atomics.
- `firmware/components/dcs_support/src/dcs_admin_pages.c` — `xcheck_*` fields in
  `/state.json`.
</content>
