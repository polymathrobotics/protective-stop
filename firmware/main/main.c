// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file main.c
 * @brief Dual-Core Safety — lockstep pstop client, one remote to up to
 * PSTOP_MAX_MACHINES machines.
 *
 * Two pinned tasks on cores 0 and 1 each independently build the SAME
 * outgoing pstop_msg_t per bonded machine and encode each to a 40-byte
 * buffer. A comparator task does a bytewise memcmp of the two encodings
 * for every machine; only if ALL agree does it transmit anything this
 * tick. Disagreement → send nothing to anyone → every machine's heartbeat
 * times out → STOP. Lockstep integrity is a property of the DEVICE, not
 * of one link, so a single mismatch silences all sessions.
 *
 * The whole `pstop_msg_t` is compared, including the CRC, so the safety
 * check catches RAM/cache/ALU corruption during encoding — not just a
 * mismatch in the OK/STOP verdict.
 *
 * Machines are independent sessions: each has its own socket (so egress
 * binding is per peer type), its own pstop_c protocol_data_t counter
 * state, its own non-blocking bond state machine and its own reply-loss
 * watchdog. One unreachable machine NEVER stalls heartbeats to the
 * others — the mirror of the machine side's many-remote fail-safe OR.
 * The arming gesture (STOP→OK) is broadcast: one press-and-release arms
 * every bonded machine; each machine still enforces its own min_stop_ms
 * veto.
 *
 * Everything *not* part of the safety pattern (HTTP admin UI, OTA-rollback
 * ladder, NVS, telemetry, network-liveness watchdog, USB/WiFi boot path)
 * lives in components/dcs_support/.
 *
 * Modelled on pstop_c/examples/client/client_app.c:
 *   bond per machine → loop send_msg(verdict) → never unbond.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dcs_identity.h"
#include "dcs_support.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "estop_verdict.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h" /* gpio_ll_get_io_config — pad-config read-back (SR-R-09) */
#include "lwip/sockets.h"
#include "pstop/protocol_data.h"
#include "pstop/pstop_msg.h"
#include "soc/gpio_struct.h" /* GPIO peripheral instance for gpio_ll_* */

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define TICK_HZ 10
#define TICK_MS (1000u / TICK_HZ)
#define TICK_PERIOD pdMS_TO_TICKS(1000 / TICK_HZ)

/* === Machine-governed heartbeat rate =======================================
 * The MACHINE decides the update rate its safety case needs: pstop_c puts the
 * per-operator heartbeat_ms (machine.toml: default_heartbeat_ms / per-operator
 * heartbeat_ms) into the heartbeat_timeout field of every reply, including
 * the bond ack (machine.c: resp->heartbeat_timeout = remote_data.heartbeat_ms;
 * the field is only valid machine→operator). The remote adopts it per session:
 * it transmits every heartbeat_timeout/2 (2x margin, so normal jitter can
 * never cost the machine a whole heartbeat window), clamped between the 10 Hz
 * lockstep tick (the E-stop sampling cadence is a safety property and never
 * slows down) and PSTOP_MAX_SEND_PERIOD_MS. Sends are DECIMATED ticks — the
 * per-session counter advances only on transmitting ticks, so the machine
 * sees contiguous counters (protocol.c rejects gaps > max_lost_messages+1).
 * A reply carrying heartbeat_timeout=0 (reject paths) changes nothing. */
#define PSTOP_MAX_SEND_PERIOD_MS 1000u

/* Each core publishes its encoded buffers within 80 ms of the tick. If a core
 * doesn't, treat that tick as a mismatch (send nothing). 80ms leaves the
 * comparator 20ms of slack to drain RX and re-align to vTaskDelayUntil's
 * 100ms tick — and gives the cores enough headroom that being briefly
 * preempted by ml_wg_mgr or httpd doesn't cause spurious mismatches. */
#define CORE_PUBLISH_TIMEOUT pdMS_TO_TICKS(80)

/* Machine sessions. Must not exceed DCS_PSTOP_MAX_MACHINES (the telemetry
 * and peer-slot plumbing in dcs_support is sized to it). */
#define PSTOP_MAX_MACHINES DCS_PSTOP_MAX_MACHINES

/* Local UDP source ports: slot i binds PSTOP_LOCAL_PORT + i (8891..8894).
 * One socket per machine keeps the reply path trivially demultiplexed and
 * lets each socket carry its own egress binding (VPN vs LAN). */
#define PSTOP_LOCAL_PORT 8891

#define HEARTBEAT_TIMEOUT_MS 1000u

/* Reply-loss watchdog, per session. If a machine stops replying for longer
 * than this, that link has desynced and won't recover on its own — re-bond
 * that session to re-sync counters. Other sessions are untouched.
 *
 * MUST exceed the machine's heartbeat timeout + jitter, so the remote only
 * tears the session AFTER machn would itself have dropped the bond — never on
 * a sub-timeout reply blip. That timeout is `heartbeat_ms × max_missed`
 * = 400 × 5 = 2000 ms with today's defaults (max_missed raised 3→5 on
 * 2026-08-04). The old value (1500 ms) was set against the legacy 1000 ms
 * timeout and was NOT updated with max_missed, so it dropped BELOW the machine
 * timeout: a ~1.6 s reply lag (e.g. a peer-join X25519/DISCO crypto burst
 * time-sharing the wg_mgr decrypt task) forced a nuisance rebond even though
 * machn still held the bond.
 *
 * This constant is only the FLOOR. A hard-coded 2500 ms held the invariant
 * for the 400 ms default alone; the actual threshold is derived per session
 * from the machine-adopted heartbeat in sess_rebond_after_ms() so any
 * configured heartbeat keeps the remote's watchdog above the machine's.
 * The floor also covers the pre-first-reply state (heartbeat not yet
 * learned). Safety is unaffected — machn's heartbeat timeout remains the
 * STOP authority; this only governs connectivity re-sync. */
#define REBOND_AFTER_MS 2500u

/* Missed-heartbeat multiplier in the machine's bond-drop timeout. max_missed
 * is NOT advertised on the wire (replies carry only heartbeat_timeout), so
 * this is a hard-coded coupling to machn's MACHN_MAX_MISSED_HEARTBEATS = 5
 * (machn/main/main.c) — if that constant changes, this one MUST change with
 * it. Long-term fix: advertise the machine's absolute timeout in the reply
 * instead of making the remote reconstruct it. */
#define REBOND_MACHINE_MAX_MISSED 5u
#define REBOND_JITTER_MARGIN_MS 500u

/* Bond handshake, per session: one BOND in flight at a time; retry after
 * this long without a reply. Long spacing means we never have more than one
 * BOND outstanding — critical because the pstop protocol rejects duplicate
 * counters with MSG_LOST once a client is registered — and is long enough
 * that the machine's check_heartbeats can age out a half-bonded stale
 * client between attempts. */
#define BOND_RETRY_MS 5000u

/* How long the comparator waits for the first reply after a send burst.
 * Bounded so the 10 Hz tick always holds. */
#define REPLY_WAIT_MS 50

static const char * TAG = "dcs_main";

/* Last message TYPE received from a machine, aggregated worst-of across
 * bonded sessions (STOP beats BOND/UNBOND beats OK), published for the LED
 * ring's legacy consumers and /state.json. The ring's per-segment display
 * reads the per-machine arrays instead. */
extern atomic_uint_fast32_t g_dcs_pstop_last_msg;
extern atomic_uint_fast32_t g_dcs_pstop_replies;
extern atomic_uint_fast32_t g_dcs_pstop_rebonds;

/* ============================================================================
 * Lockstep state — one tick's worth of inputs for every active session,
 * snapshotted by the comparator before notifying the cores. Cores read this
 * read-only and encode one buffer per active slot.
 * ========================================================================== */

typedef struct
{
  bool active; /* encode this slot this tick (session is BONDED) */
  uint64_t stamp_ms;
  uint32_t counter;
  uint32_t received_counter;
  uint64_t received_stamp;
  uint32_t receiver_id; /* this machine's device id */
} tick_input_t;

static tick_input_t g_tick[PSTOP_MAX_MACHINES];
/* E-stop rolling-level source: advances EVERY tick regardless of which
 * sessions transmit, so the drive-high/drive-low sampling pattern never
 * stalls under send decimation. Written by the comparator before it
 * notifies the cores; read-only for the cores (same discipline as g_tick). */
static uint32_t g_tick_roll;
static uint8_t g_encoded[2][PSTOP_MAX_MACHINES][PSTOP_MESSAGE_SIZE];
static uint8_t g_verdict[2]; /* for telemetry */
static SemaphoreHandle_t g_done[2]; /* core_id → comparator */
static TaskHandle_t g_core_h[2]; /* comparator → core_id */

/* ============================================================================
 * Dual-channel hardware E-stop loops.
 *
 * Two independent loopback channels run through the two poles of an external
 * DPST normally-closed E-stop switch. Each lockstep core owns ONE channel and
 * reads it independently — this is the cross-check:
 *
 *     core 0 (channel A):  GPIO39 (drive) --> DPST pole 1 --> GPIO40 (sense)
 *     core 1 (channel B):  GPIO41 (drive) --> DPST pole 2 --> GPIO42 (sense)
 *
 * Each tick the owning core drives its OUT pin with a rolling level (the tick
 * counter's LSB) and verifies the echo on its IN pin, which is pulled DOWN so
 * an OPEN loop reads 0:
 *     drive 1, read 0  -> loop OPEN (button pressed) or wire broken
 *     drive 0, read 1  -> IN shorted high
 * The loop counts as CLOSED only when the most recent drive-high echoed high
 * (proves continuity) AND the most recent drive-low echoed low (rules out a
 * stuck-high short). Until BOTH phases have been sampled the channel reads
 * OPEN, so the device is fail-safe (STOP) at boot and the rolling pattern
 * makes a stuck/shorted pin detectable rather than latching a false OK.
 *
 * Pressing the button opens BOTH poles at once: both cores independently reach
 * STOP, agree, and a STOP message is sent. A SINGLE-channel fault (one loop
 * open/short, the other intact) makes the two cores DISAGREE -> the
 * comparator's memcmp fails -> nothing is sent -> every machine heartbeat-
 * times out and stops. No single fault can mask a real stop.
 * ========================================================================== */

#define ESTOP_SETTLE_US 10 /* propagation through the wire + switch contacts */

/* The debounce/boot constants (LOOP_RECLOSE_DEBOUNCE_TICKS,
 * LOOP_BOOT_OPEN_CONFIRM_TICKS), the per-core state struct (estop_state_t), and
 * the verdict/debounce/priming logic now live in estop_verdict.{h,c} so the
 * SIL-critical decision core is host-unit-testable to branch + MC/DC (the
 * on-target Xtensa build can't be gcov'd — JTAG pins ARE the loop pins). See
 * firmware/test/ and docs/safety/COVERAGE.md. */

static const struct
{
  gpio_num_t out;
  gpio_num_t in;
} g_estop_ch[2] = {
  {GPIO_NUM_39, GPIO_NUM_40}, /* core 0 — channel A */
  {GPIO_NUM_41, GPIO_NUM_42}, /* core 1 — channel B */
};

static estop_state_t g_estop_st[2]; /* per-core state; type in estop_verdict.h */

static void estop_init(void)
{
  for (int c = 0; c < 2; c++) {
    gpio_config_t out_cfg = {
      .pin_bit_mask = 1ULL << g_estop_ch[c].out,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_config_t in_cfg = {
      .pin_bit_mask = 1ULL << g_estop_ch[c].in,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, /* OPEN loop -> reads 0 -> STOP */
      .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);
    gpio_set_level(g_estop_ch[c].out, 0);
  }
  (void)memset(g_estop_st, 0, sizeof(g_estop_st));
  ESP_LOGI(
    TAG,
    "E-stop loops: chA=GPIO%d->GPIO%d  chB=GPIO%d->GPIO%d",
    g_estop_ch[0].out,
    g_estop_ch[0].in,
    g_estop_ch[1].out,
    g_estop_ch[1].in);
}

/* Drive THIS core's loop with the tick's rolling level, sample the echo, and
 * return true iff the loop is proven closed-and-healthy. Each core touches only
 * its own pin pair, so the two cores never contend. Called exactly ONCE per
 * tick per core — the sampled verdict is then reused for every machine's
 * encoding, so all sessions carry the same tick verdict. */
static uint8_t estop_channel_closed(int core_id, uint32_t counter)
{
  const gpio_num_t out = g_estop_ch[core_id].out;
  const gpio_num_t in = g_estop_ch[core_id].in;
  (void)counter; /* both phases are sampled every tick; no rolling phase */

  /* SAFETY — button integral to the message (external reviewer, 2026-08).
   * Sample BOTH loop phases THIS tick: drive HIGH then LOW, reading each back.
   * A healthy closed loop conducts as driven → rb_hi==1 AND rb_lo==0. */
  gpio_set_level(out, 1);
  esp_rom_delay_us(ESTOP_SETTLE_US);
  const int rb_hi = gpio_get_level(in);
  gpio_set_level(out, 0);
  esp_rom_delay_us(ESTOP_SETTLE_US);
  const int rb_lo = gpio_get_level(in);

  /* Decide the verdict + update debounce/priming in the pure, host-testable
   * core (estop_verdict.c — carries the full safety rationale). This function
   * stays thin HAL glue: drive/read both phases (above), decide, publish. */
  const uint8_t msg = estop_decide(&g_estop_st[core_id], core_id, rb_hi, rb_lo);
  dcs_publish_estop(core_id, g_estop_st[core_id].high_ok, g_estop_st[core_id].low_ok);
  return msg;
}

/* True once BOTH cores have sampled their loop on both rolling phases. The
 * comparator holds off sending until then (see comparator_task), so the
 * one-time boot-priming STOP never reaches any machine. That artifact STOP
 * would otherwise complete the machine's NEED_STOP arming (a STOP->OK cycle)
 * and arm the robot to OK with NO operator action — arming MUST be a deliberate
 * manual press->release. After priming, the only STOP->OK on the wire comes
 * from a real E-stop press then release. (Primed flags latch true for the run,
 * so this gate only affects the ~2-tick startup window.) */
static bool estop_primed(void)
{
  return estop_channels_primed(g_estop_st);
}

/* ============================================================================
 * Safety verdict — each core reads its OWN E-stop channel. OK only while the
 * loop is proven closed; otherwise STOP. The two cores' verdicts are
 * cross-checked downstream by the comparator's byte-compare (disagreement =>
 * send nothing => machines stop), so a per-core read here is the intended
 * design, not a lockstep hazard: identical inputs (both loops closed, or both
 * open) produce identical messages; only a genuine channel split diverges.
 * ========================================================================== */
static uint8_t compute_verdict(uint32_t counter, int core_id)
{
  /* estop_channel_closed now returns the codeword directly (physical-image
   * derived; OK only on a live sample that matched the driven level). */
  return estop_channel_closed(core_id, counter);
}

/* ============================================================================
 * E-stop GPIO pad-config re-verification — SR-R-09 / FMEA DU-1.
 *
 * The E-stop IN pins are configured ONCE in estop_init() as INPUT with the
 * PULL-DOWN ENABLED, so an OPEN loop discharges to 0 and reads STOP. If that
 * pull-down is lost at runtime (an SEU on the IO_MUX config register, a stray
 * pad reconfig, ESD), an open loop FLOATS and can echo 1 on the drive-high
 * phase -> false CLOSED -> false OK, with NO existing runtime detection: this
 * is DU-1, the highest-priority dangerous-undetected gap. PSTOP_SAFETY_DESIGN
 * claimed a runtime "config-integrity check"; the code never had one. This is
 * it.
 *
 * Every ~1 s the comparator reads back each IN pad's configuration registers
 * and confirms the safe setup is intact. Read-back primitive:
 * gpio_ll_get_io_config() (ESP-IDF v5.5 hal/gpio_ll.h) decodes the very
 * IO_MUX/GPIO registers gpio_config() wrote:
 *   .pd = FUN_WPD (IO_MUX pull-down)     MUST be 1  <- the DU-1 bit
 *   .ie = FUN_IE  (IO_MUX input-enable)  MUST be 1  (else the read is dead)
 *   .oe = GPIO output-enable (direction) MUST be 0  (input, not driven output)
 *   .pu = FUN_WPU (IO_MUX pull-up)       MUST be 0  (a pull-UP would actively
 *                                                    hold an open loop CLOSED)
 * Any mismatch latches a STICKY per-channel fault bit; the comparator then
 * fail-safes exactly like the clock cross-check: it halts all pstop TX (every
 * machine heartbeat-times-out to STOP), then after a short STOP-settle grace
 * does a CONTROLLED, CLEAN reset (esp_restart, reset_reason ESP_RST_SW — NOT
 * counted toward the OTA rollback ladder). On reboot estop_init re-writes the
 * pad config, so a TRANSIENT corruption (an SEU-flipped FUN_WPD bit) is
 * recovered; a PERSISTENT hardware fault simply re-trips and loops, and the
 * device stays safely silent (machines STOPped) the whole time. Using our own
 * controlled reset preempts the cores' unfed-TWDT reset (which would otherwise
 * fire ~5 s later as the rollback-counted ESP_RST_TASK_WDT).
 * ========================================================================== */

#define ESTOP_PADCFG_REVERIFY_TICKS 10u /* ~1 s at the 100 ms comparator tick */
#define ESTOP_PADCFG_RESET_GRACE_MS 1500u /* STOP-settle (> heartbeat timeout) before the controlled reset */

/* Sticky bitmask: bit c set once channel c's pad config is seen bad. */
static atomic_uint_fast32_t g_gpio_cfg_fault;

/* True iff this IN pad still holds the safe E-stop config (input + pull-down). */
static bool estop_padcfg_ok(gpio_num_t in)
{
  gpio_io_config_t cfg;
  gpio_ll_get_io_config(&GPIO, (uint32_t)in, &cfg);
  return cfg.pd && cfg.ie && !cfg.oe && !cfg.pu;
}

/* Re-verify BOTH E-stop IN pads; latch a sticky fault bit per bad channel.
 * Returns the current latched fault bitmask (0 = healthy). */
static uint32_t estop_padcfg_reverify(void)
{
  uint32_t bad = 0u;
  for (int c = 0; c < 2; c++) {
    if (!estop_padcfg_ok(g_estop_ch[c].in)) {
      bad |= (1u << c);
    }
  }
  if (bad != 0u) {
    uint32_t prev = (uint32_t)atomic_fetch_or(&g_gpio_cfg_fault, bad);
    if ((prev | bad) != prev) { /* newly-set bit(s) — log the transition once */
      ESP_LOGE(
        TAG,
        "GPIO PADCFG FAULT mask 0x%lx (E-stop IN pins GPIO%d/GPIO%d): input-dir/"
        "pull-down integrity LOST -> an open loop could read false-OK. FAIL-SAFE "
        "STOP: halting all pstop TX (every machine heartbeat-times-out).",
        (unsigned long)bad,
        g_estop_ch[0].in,
        g_estop_ch[1].in);
    }
  }
  return (uint32_t)atomic_load(&g_gpio_cfg_fault);
}

/* ============================================================================
 * Dual-core mutual clock cross-check — SR-H-04 / FMEA DU-2 (frozen clock /
 * frozen task).
 *
 * The existing lockstep already fail-safes a core whose task WEDGES MID-TICK:
 * it stops publishing its encoding, the comparator's `both_in` fails, nothing
 * is sent, every machine STOPs. What it does NOT catch is a frozen esp_timer
 * CLOCK while the scheduler keeps running: the per-session counter advances on
 * every transmitting tick regardless of the clock, so the machine keeps
 * accepting heartbeats even though the pstop `stamp` (taken from esp_timer)
 * has frozen — a dangerous-undetected fault. This cross-check closes that gap.
 *
 * Each core, every tick, PUBLISHES two independent liveness signals for the
 * OTHER core to observe:
 *   - hb   : a heartbeat counter this core increments itself each loop pass
 *            (task-liveness — freezes iff this core's task stops running);
 *   - us   : an independent esp_timer_get_time() reading
 *            (clock-liveness — freezes iff esp_timer stops advancing).
 * Each core then VERIFIES the peer's two signals are advancing, gating on the
 * FreeRTOS tick count (xTaskGetTickCount) — a DIFFERENT time base from
 * esp_timer, so a frozen esp_timer is still measurable against the scheduler.
 * Two independent stall timers:
 *   - peer hb frozen  > XCHECK_STALL_TICKS  -> peer TASK wedged   (FAULT_PEER_HB)
 *   - peer us frozen/backward               -> peer CLOCK frozen  (FAULT_PEER_CLK)
 * Checking them separately is the point: if esp_timer freezes while the task
 * still loops, hb keeps advancing (no HB fault, the task IS alive) but us
 * freezes -> the clock timer fires. On a fault the detecting core latches
 * g_xcheck_fault; the comparator then FAIL-SAFES: send nothing (machines
 * heartbeat-time-out to STOP) then a controlled reset for recovery.
 *
 * Lock-free: g_xcheck_hb/us are single-writer-per-core atomics (same
 * discipline as g_tick_roll); the per-core observation memory is private to
 * each core task. A torn read across the un-paired hb/us atomics can only
 * DELAY detection by a tick, never cause a false fault, because each signal
 * has its own stall timer over a 500 ms window.
 *
 * Residual (documented): a total systimer freeze stops the scheduler itself,
 * so the cores stop running and this software check cannot execute — that case
 * is caught only by the hardware watchdogs (TWDT feed stops -> reset; the
 * RTC WDT on its independent slow clock is the ultimate backstop). See
 * docs/WATCHDOG_CLOCK_PROTECTION.md.
 * ========================================================================== */

#define XCHECK_STALL_TICKS pdMS_TO_TICKS(500) /* 5 ticks of no-advance tolerance */
#define XCHECK_RESET_GRACE_MS 500u /* silent-STOP settle before controlled reset */
#define XCHECK_FAULT_NONE 0u
#define XCHECK_FAULT_PEER_HB 1u /* peer task/heartbeat stalled */
#define XCHECK_FAULT_PEER_CLK 2u /* peer esp_timer clock frozen or ran backward */

/* Published by core c, observed by core (1-c). Single-writer-per-core. */
static atomic_uint_fast32_t g_xcheck_hb[2];
static atomic_uint_fast64_t g_xcheck_us[2];
/* Latched on the first detected fault; polled by the comparator. */
static atomic_uint_fast32_t g_xcheck_fault;

/* Private per-observer memory of the peer's last-seen signals + the tick at
 * which each last advanced. Indexed by the OBSERVING core id. */
static uint32_t s_xc_seen_hb[2];
static uint64_t s_xc_seen_us[2];
static TickType_t s_xc_hb_tick[2];
static TickType_t s_xc_clk_tick[2];
static bool s_xc_init[2];

static void xcheck_raise(uint32_t code, int core_id, int peer, uint32_t peer_hb, uint64_t peer_us)
{
  uint32_t expect = XCHECK_FAULT_NONE;
  if (atomic_compare_exchange_strong(&g_xcheck_fault, &expect, code)) {
    ESP_LOGE(
      TAG,
      "XCHECK core%d: peer core%d %s (hb=%lu clk=%llu us) — FAIL-SAFE STOP + reset",
      core_id,
      peer,
      (code == XCHECK_FAULT_PEER_HB) ? "TASK STALLED (heartbeat frozen)" : "CLOCK FROZEN/BACKWARD (esp_timer stalled)",
      (unsigned long)peer_hb,
      (unsigned long long)peer_us);
  }
}

/* Publish MY liveness, then verify the PEER's. Called once per tick per core. */
static void xcheck_publish_and_verify(int core_id, uint32_t hb, uint64_t now_us)
{
  atomic_store(&g_xcheck_hb[core_id], hb);
  atomic_store(&g_xcheck_us[core_id], now_us);

  const int peer = 1 - core_id;
  const uint32_t peer_hb = (uint32_t)atomic_load(&g_xcheck_hb[peer]);
  const uint64_t peer_us = (uint64_t)atomic_load(&g_xcheck_us[peer]);
  const TickType_t now_tick = xTaskGetTickCount();

  if (!s_xc_init[core_id]) {
    s_xc_seen_hb[core_id] = peer_hb;
    s_xc_seen_us[core_id] = peer_us;
    s_xc_hb_tick[core_id] = now_tick;
    s_xc_clk_tick[core_id] = now_tick;
    s_xc_init[core_id] = true;
    return;
  }

  /* Task-liveness: the peer's self-incremented heartbeat counter. */
  if (peer_hb != s_xc_seen_hb[core_id]) {
    s_xc_seen_hb[core_id] = peer_hb;
    s_xc_hb_tick[core_id] = now_tick;
  } else if ((TickType_t)(now_tick - s_xc_hb_tick[core_id]) > XCHECK_STALL_TICKS) {
    xcheck_raise(XCHECK_FAULT_PEER_HB, core_id, peer, peer_hb, peer_us);
  }

  /* Clock-liveness: the peer's esp_timer microsecond stamp. Must be monotonic
   * AND advancing. Gated on the FreeRTOS tick, so a frozen esp_timer (which
   * also freezes THIS core's own esp_timer reads) is still caught. */
  if (peer_us > s_xc_seen_us[core_id]) {
    s_xc_seen_us[core_id] = peer_us;
    s_xc_clk_tick[core_id] = now_tick;
  } else if (peer_us < s_xc_seen_us[core_id]) {
    xcheck_raise(XCHECK_FAULT_PEER_CLK, core_id, peer, peer_hb, peer_us); /* backward */
  } else if ((TickType_t)(now_tick - s_xc_clk_tick[core_id]) > XCHECK_STALL_TICKS) {
    xcheck_raise(XCHECK_FAULT_PEER_CLK, core_id, peer, peer_hb, peer_us); /* frozen */
  }
}

/* ============================================================================
 * Per-core task — pinned to a single ESP32-S3 core.
 *
 * On each notification from the comparator: sample MY E-stop channel ONCE,
 * then build + encode one pstop_msg_t per active session slot from the
 * shared per-slot tick inputs, and signal completion.
 * ========================================================================== */

static void core_task(void * arg)
{
  int core_id = (int)(intptr_t)arg;
  uint32_t hb_local = 0; /* this core's private heartbeat (task-liveness) */
  bool wdt_subscribed = false;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Subscribe to the TWDT on the FIRST notification (not before — the
         * comparator delays ~5 s at boot before it starts notifying, and a
         * subscribed-but-unfed task would trip the 5 s TWDT during that window).
         * From here on a wedged comparator (no notifications) also stops this
         * feed -> TWDT resets the chip, a second path to catching a wedge. */
    if (!wdt_subscribed) {
      (void)esp_task_wdt_add(NULL);
      wdt_subscribed = true;
    }

    /* Snapshot the tick inputs the comparator just published. */
    tick_input_t in[PSTOP_MAX_MACHINES];
    (void)memcpy(in, g_tick, sizeof(in));

    /* Sample the loop exactly once per tick (the drive/echo pattern is
         * stateful), clocked by the dedicated roll counter so priming/debounce
         * advances every tick even when session sends are decimated. */
    uint32_t roll = g_tick_roll;
    uint8_t verdict = compute_verdict(roll, core_id);

    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      if (!in[i].active) {
        continue;
      }
      pstop_msg_t msg;
      pstop_message_init(&msg);
      msg.message = verdict;
      msg.id.data = dcs_identity_device_id();
      msg.receiver_id.data = in[i].receiver_id;
      msg.stamp = in[i].stamp_ms;
      msg.received_stamp = in[i].received_stamp;
      msg.counter = in[i].counter;
      msg.received_counter = in[i].received_counter;
      msg.heartbeat_timeout = HEARTBEAT_TIMEOUT_MS;
      /* pstop_message_encode computes the CRC over bytes [0..37] and
             * writes it to bytes [38..39] — both cores produce byte-identical
             * buffers given identical input fields. */
      pstop_message_encode(&msg, g_encoded[core_id][i]);
    }

    g_verdict[core_id] = verdict;
    dcs_publish_core_tick(core_id, roll, verdict);

    /* Dual-core clock cross-check: publish MY heartbeat + an independent
         * esp_timer read, then verify the PEER core's are advancing. Runs every
         * tick regardless of session activity, so clock monitoring is continuous
         * even with no machines bonded. */
    hb_local++;
    xcheck_publish_and_verify(core_id, hb_local, (uint64_t)esp_timer_get_time());

    /* Feed the TWDT — proves THIS core task ran to completion this tick. */
    (void)esp_task_wdt_reset();

    xSemaphoreGive(g_done[core_id]);
  }
}

/* ============================================================================
 * Machine sessions — one per configured peer slot. Each session owns its
 * socket, its pstop_c protocol_data_t counter state, and a NON-BLOCKING bond
 * state machine driven from the comparator's 10 Hz loop. Nothing in here may
 * block for longer than a socket syscall: one dead machine must never stall
 * the heartbeats to the others.
 * ========================================================================== */

typedef enum
{
  SESS_IDLE = 0, /* slot not configured */
  SESS_BONDING = 1, /* configured; (re)bonding — heartbeats not flowing */
  SESS_BONDED = 2 /* counter handshake done; heartbeats flowing */
} sess_state_t;

typedef struct
{
  /* Configuration snapshot (detects live /api/pstop_peers changes). */
  bool configured;
  uint32_t ip; /* host order */
  uint16_t port;
  uint32_t machine_id;

  /* Socket, source-bound per peer type (see sess_ensure_socket). */
  int sock;
  uint32_t sock_local_ip; /* host order; 0 = INADDR_ANY */

  sess_state_t state;
  protocol_data_t pd; /* pstop_c remote handshake state (counters) */
  uint32_t req_hb_ms; /* machine-requested heartbeat window (reply
                                 * heartbeat_timeout); 0 = not yet learned */
  uint64_t next_send_ms; /* next scheduled transmit for this session */
  uint32_t bond_counter;
  uint64_t bond_sent_ms; /* 0 = no BOND in flight */
  uint64_t last_reply_ms;
  uint64_t tx_stamp_history[16];

  /* Sustained-ENOTCONN escalation state (cold-bond far-side gap,
     * 2026-08-08 bench): the machine rebooted without its netmap, forgot our
     * WG key, and silently ignored our handshake initiations — every send hit
     * peer-present-but-no-keypair (lwip ERR_CONN -> errno ENOTCONN) for
     * 27 min; restarting THIS remote did not help. See step 8b in the
     * comparator loop for the bounded escalation these fields drive. */
  uint64_t enotconn_since_ms; /* 0 = not in an ENOTCONN streak */
  uint64_t enotconn_next_kick_ms; /* earliest next escalation */
  uint32_t enotconn_backoff_ms; /* current backoff period (0 = at initial) */

  /* Telemetry. */
  uint32_t sent;
  uint32_t replies;
  uint32_t send_fail;
  uint32_t rebonds;
  uint32_t rtt_ms;
  uint8_t last_msg;
} pstop_sess_t;

static pstop_sess_t g_sess[PSTOP_MAX_MACHINES];

/* Send period this session has adopted: half the machine's requested window
 * (2x margin so jitter never costs the machine a whole heartbeat window),
 * clamped to [lockstep tick, PSTOP_MAX_SEND_PERIOD_MS]. Window not yet
 * learned (0) = every tick, the safe default. */
static uint32_t sess_send_period_ms(const pstop_sess_t * s)
{
  if (s->req_hb_ms == 0u) {
    return TICK_MS;
  }
  uint32_t p = s->req_hb_ms / 2u;
  if (p < TICK_MS) {
    p = TICK_MS;
  }
  if (p > PSTOP_MAX_SEND_PERIOD_MS) {
    p = PSTOP_MAX_SEND_PERIOD_MS;
  }
  return p;
}

/* Reply-loss watchdog threshold: the machine's bond-drop timeout is
 * req_hb_ms × max_missed, so derive from the per-session adopted heartbeat
 * plus jitter margin. This keeps the invariant (the remote tears a session
 * only AFTER the machine would have dropped the bond) for ANY configured
 * heartbeat — the previous hard-coded 2500 ms only exceeded the machine
 * timeout for the 400 ms / max_missed=5 default. The ×5 multiplier is a
 * hard-coded mirror of machn's MACHN_MAX_MISSED_HEARTBEATS because
 * max_missed is not on the wire (see REBOND_MACHINE_MAX_MISSED). Floored at
 * REBOND_AFTER_MS, which also covers req_hb_ms == 0 (no reply adopted yet). */
static uint64_t sess_rebond_after_ms(const pstop_sess_t * s)
{
  uint64_t t = ((uint64_t)s->req_hb_ms * (uint64_t)REBOND_MACHINE_MAX_MISSED) + REBOND_JITTER_MARGIN_MS;
  return (t < (uint64_t)REBOND_AFTER_MS) ? (uint64_t)REBOND_AFTER_MS : t;
}

static struct sockaddr_in sess_addr(const pstop_sess_t * s)
{
  struct sockaddr_in a = {
    .sin_family = AF_INET,
    .sin_port = htons(s->port),
    .sin_addr.s_addr = htonl(s->ip),
  };
  return a;
}

static void sess_close_socket(pstop_sess_t * s)
{
  if (s->sock >= 0) {
    close(s->sock);
    s->sock = -1;
  }
}

/* (Re)start a session's bond from scratch. Heartbeats to this machine stop
 * (it fail-safes on its own heartbeat timeout); other sessions unaffected. */
static void sess_start_bonding(pstop_sess_t * s)
{
  s->state = SESS_BONDING;
  s->bond_counter = 0;
  s->bond_sent_ms = 0;
  protocol_data_init(&s->pd);
  s->pd.remote_id.data = s->machine_id;
  s->pd.heartbeat_ms = HEARTBEAT_TIMEOUT_MS;
}

/* Reset a session for a (new) configuration. */
static void sess_reconfigure(pstop_sess_t * s, bool configured, uint32_t ip, uint16_t port, uint32_t machine_id)
{
  sess_close_socket(s);
  (void)memset(s, 0, sizeof(*s));
  s->sock = -1;
  s->configured = configured;
  s->ip = ip;
  s->port = port;
  s->machine_id = machine_id;
  if (configured) {
    sess_start_bonding(s);
  } else {
    s->state = SESS_IDLE;
  }
}

/* Make sure the session's socket exists and is bound correctly for its peer.
 * NON-BLOCKING: if the right binding isn't available yet (Tailscale peer,
 * VPN not registered), the socket stays closed and the caller skips this
 * session for the tick — the machine sees silence, which is a STOP.
 *
 * Tailscale peers (100.64.0.0/10) get a socket source-bound to our VPN IP
 * so lwIP's source-based routing pins egress to the WireGuard netif. This
 * is a SAFETY property, not an optimization: with an INADDR_ANY socket,
 * lwIP routes per packet by destination, and whenever the WG session
 * lapses (netif link down) the CGNAT destination silently falls through
 * to the default route — observed 2026-07-20 on the bench as the pstop
 * heartbeat riding the USB tether IN PLAINTEXT. Source-bound, a downed
 * tunnel makes sendto FAIL instead: send_fail climbs, that machine times
 * out to STOP. Fail safe, never fail open. */
static bool sess_ensure_socket(pstop_sess_t * s, int slot)
{
  uint32_t want = 0;
  if ((s->ip & 0xFFC00000u) == 0x64400000u) {
    want = dcs_get_vpn_ip();
    if (want == 0u) {
      /* Tailscale peer but VPN not up: hold this session dark. */
      sess_close_socket(s);
      return false;
    }
  }

  if ((s->sock >= 0) && (want == s->sock_local_ip)) {
    return true;
  }
  sess_close_socket(s);

  int sk = socket(AF_INET, SOCK_DGRAM, 0);
  if (sk < 0) {
    ESP_LOGE(TAG, "m%d socket(): errno=%d", slot, errno);
    return false;
  }
  struct sockaddr_in local = {
    .sin_family = AF_INET,
    .sin_port = htons((uint16_t)(PSTOP_LOCAL_PORT + slot)),
    .sin_addr.s_addr = (want != 0u) ? htonl(want) : INADDR_ANY,
  };
  if (bind(sk, (struct sockaddr *)&local, sizeof(local)) < 0) {
    /* Transient (VPN IP racing netif assignment, fd pressure): retry next
         * tick rather than blocking the safety loop. */
    ESP_LOGW(TAG, "m%d bind(): errno=%d — retrying next tick", slot, errno);
    close(sk);
    return false;
  }
  int flags = fcntl(sk, F_GETFL, 0);
  fcntl(sk, F_SETFL, flags | O_NONBLOCK);
  s->sock = sk;
  s->sock_local_ip = want;
  ESP_LOGI(
    TAG,
    "m%d socket local=%lu.%lu.%lu.%lu:%u peer=%lu.%lu.%lu.%lu:%u id=0x%08lx",
    slot,
    (unsigned long)((want >> 24) & 0xFF),
    (unsigned long)((want >> 16) & 0xFF),
    (unsigned long)((want >> 8) & 0xFF),
    (unsigned long)(want & 0xFF),
    (unsigned)(PSTOP_LOCAL_PORT + slot),
    (unsigned long)((s->ip >> 24) & 0xFF),
    (unsigned long)((s->ip >> 16) & 0xFF),
    (unsigned long)((s->ip >> 8) & 0xFF),
    (unsigned long)(s->ip & 0xFF),
    s->port,
    (unsigned long)s->machine_id);
  return true;
}

/* Drain anything queued on a session's socket. Used before each BOND attempt
 * so late responses from a previous attempt don't fool us into thinking the
 * CURRENT bond succeeded. */
static void sess_drain(pstop_sess_t * s)
{
  for (;;) {
    uint8_t junk[PSTOP_MESSAGE_SIZE];
    if (recvfrom(s->sock, junk, sizeof(junk), 0, NULL, NULL) <= 0) {
      break;
    }
  }
}

/* Count of transient ERR_IF refusals absorbed by a same-tick retry (i.e. sends
 * that would previously have been dropped as g_sf_txdrv and could have opened a
 * heartbeat gap). Published as pstop_sf_txdrv_recovered for soak verification. */
static uint32_t g_sf_txdrv_recovered;

/* Max immediate retries on a TRANSIENT uplink-driver refusal (ERR_IF, errno -1
 * — e.g. USB-NCM all IN NTBs momentarily in-flight). Each retry yields so the
 * lower-priority (prio 5, core 1) TinyUSB task can complete a bulk-IN transfer
 * and return an NTB to the free list; the resend then succeeds within the SAME
 * send tick, so the machine never sees a heartbeat gap. Raised 3 -> 6 retries
 * (7 attempts total; 1 ms→2 ms backoff between attempts, ~9 ms worst case —
 * no delay after the final failure) after a soak at ~70 % core-1 load showed
 * bursts outrunning the old ~3 ms window (pstop_sf_txdrv=9): at high load the
 * prio-5 USB task is slow to drain, so a wider window is needed to absorb the
 * transient. Still ≪ the 200 ms period and ≪ the machine's ~2.0 s timeout
 * (400 ms × max_missed 5). Gated on ERR_IF ONLY — a genuinely dead link
 * (route/ENOMEM) still fails on the first attempt, so dead-link detection
 * latency is UNCHANGED. */
#define PSTOP_TX_RETRY_MAX 6

static bool sess_sendto(pstop_sess_t * s, const uint8_t * bytes)
{
  struct sockaddr_in a = sess_addr(s);
  for (int attempt = 0; attempt <= PSTOP_TX_RETRY_MAX; attempt++) {
    int n = sendto(s->sock, bytes, PSTOP_MESSAGE_SIZE, 0, (struct sockaddr *)&a, sizeof(a));
    if (n == PSTOP_MESSAGE_SIZE) {
      if (attempt > 0) {
        g_sf_txdrv_recovered++; /* a transient ERR_IF absorbed without a gap */
      }
      /* A send got through the WG session — keypair exists again, so the
             * sustained-ENOTCONN escalation (step 8b) stands down. */
      s->enotconn_since_ms = 0u;
      s->enotconn_next_kick_ms = 0u;
      s->enotconn_backoff_ms = 0u;
      return true;
    }
    if (errno != -1) {
      return false; /* not ERR_IF (route / ENOMEM / other): fail fast, never spin */
    }
    if (attempt < PSTOP_TX_RETRY_MAX) { /* no send follows the last failure — don't burn a pointless delay */
      vTaskDelay((attempt < 3) ? 1 : 2); /* yield so the prio-5 USB task frees an IN NTB; widen as a burst persists */
    }
  }
  return false; /* still ERR_IF after the bounded retries -> counted as g_sf_txdrv */
}

/* Aggregate send-failure causes (errno buckets), published to /state.json so
 * a climbing send_fail can be attributed without guesswork: ENOMEM = TX
 * queue/pbuf pressure (typically the DERP relay path under load); route =
 * no path to the peer (tunnel down / no session); other = the rest. */
static uint32_t g_sf_nomem;
static uint32_t g_sf_route;
static uint32_t g_sf_txdrv; /* errno -1 = lwip ERR_IF: the uplink DRIVER refused the
                             * frame (e.g. USB-NCM transmit busy) — path is up,
                             * one send slot lost, retried next tick */
static uint32_t g_sf_other;
static uint32_t g_sf_enotconn; /* ENOTCONN = WG peer known but NO keypair: the far
                                * side doesn't know our key and ignores our
                                * handshakes (cold-bond gap, 2026-08-08) */
static uint32_t g_sf_enotconn_kicks; /* sustained-ENOTCONN escalations fired */
static int g_sf_last_errno;

/* Sustained-ENOTCONN escalation tuning (comparator step 8b). 30 s before the
 * first kick: long enough that microlink's own per-peer health wake (3 s
 * cadence re-handshakes, already driven by dcs_notify_peer_health above) got
 * a fair chance — if ENOTCONN persists past that, the far side genuinely
 * does not know our key and only a coordination re-announce can fix it. */
#define PSTOP_ENOTCONN_ESCALATE_MS 30000u
#define PSTOP_ENOTCONN_BACKOFF_CAP_MS 300000u

static void sess_count_send_fail(pstop_sess_t * s)
{
  int e = errno;
  s->send_fail++;
  g_sf_last_errno = e;
  if (e == ENOMEM) {
    g_sf_nomem++;
  } else if ((e == EHOSTUNREACH) || (e == ENETUNREACH) || (e == EADDRNOTAVAIL)) {
    g_sf_route++;
  } else if (e == -1) {
    g_sf_txdrv++; /* lwip ERR_IF maps to errno -1 (err.c table) */
  } else if (e == ENOTCONN) {
    g_sf_enotconn++;
    if (s->enotconn_since_ms == 0u) {
      /* Streak start — the escalation clock (step 8b) runs from here. */
      s->enotconn_since_ms = (uint64_t)(esp_timer_get_time() / 1000);
    }
  } else {
    g_sf_other++;
  }
  if (e != ENOTCONN) {
    /* Different failure mode: not a keypair problem — restart the streak. */
    s->enotconn_since_ms = 0u;
    s->enotconn_next_kick_ms = 0u;
    s->enotconn_backoff_ms = 0u;
  }
}

/* Fire one BOND if none is in flight (or the last one timed out). Never
 * blocks; the reply is picked up by the shared drain pass. */
static void sess_bond_step(pstop_sess_t * s, int slot, uint64_t now_ms)
{
  if ((s->bond_sent_ms != 0u) && ((now_ms - s->bond_sent_ms) < BOND_RETRY_MS)) {
    return; /* BOND in flight, not yet timed out */
  }
  if (s->bond_sent_ms != 0u) {
    ESP_LOGW(TAG, "m%d BOND no response in %lums — retrying", slot, (unsigned long)BOND_RETRY_MS);
  }
  sess_drain(s);
  s->bond_counter++;

  pstop_msg_t req;
  pstop_message_init(&req);
  req.message = PSTOP_MESSAGE_BOND;
  req.id.data = dcs_identity_device_id();
  req.receiver_id.data = s->machine_id;
  req.stamp = now_ms;
  req.counter = s->bond_counter;
  req.heartbeat_timeout = HEARTBEAT_TIMEOUT_MS;
  uint8_t req_bytes[PSTOP_MESSAGE_SIZE];
  pstop_message_encode(&req, req_bytes);

  ESP_LOGI(
    TAG,
    "m%d BOND attempt counter=%lu -> %lu.%lu.%lu.%lu:%u",
    slot,
    (unsigned long)s->bond_counter,
    (unsigned long)((s->ip >> 24) & 0xFF),
    (unsigned long)((s->ip >> 16) & 0xFF),
    (unsigned long)((s->ip >> 8) & 0xFF),
    (unsigned long)(s->ip & 0xFF),
    s->port);

  if (!sess_sendto(s, req_bytes)) {
    ESP_LOGW(TAG, "m%d BOND sendto failed (errno=%d)", slot, errno);
    sess_count_send_fail(s);
    /* Leave bond_sent_ms at now anyway so we back off BOND_RETRY_MS —
         * a dead route fails instantly and would otherwise spam every tick. */
  }
  s->bond_sent_ms = now_ms;
}

/* Drain every queued reply on a session's socket, newest wins. A transient
 * path change (e.g. peer switched to a Tailscale IP whose first packets ride
 * DERP before the direct path forms) can queue a burst of late replies;
 * reading only one per tick leaves that backlog standing forever — the link
 * then runs with a permanent counter lag that inflates rtt_ms and silently
 * eats the machine's max_lost_messages margin (observed 2026-07-20: constant
 * 600 ms "rtt" on a 5 ms direct path). Returns the number of valid replies. */
static uint32_t sess_drain_replies(pstop_sess_t * s, int slot, uint64_t now_ms)
{
  uint32_t got = 0;
  for (;;) {
    uint8_t buf[PSTOP_MESSAGE_SIZE];
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);
    int n = recvfrom(s->sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &srclen);
    if (n <= 0) {
      break;
    }
    if (n != PSTOP_MESSAGE_SIZE) {
      continue;
    }
    /* Per-session sockets only ever talk to one machine, but verify the
         * source anyway — a stray sender must not be able to feed counters
         * into a safety session. */
    if ((src.sin_addr.s_addr != htonl(s->ip)) || (src.sin_port != htons(s->port))) {
      continue;
    }
    pstop_msg_t resp;
    pstop_message_decode(&resp, buf);
    if (resp.checksum != resp.calculated_checksum) {
      continue;
    }

    /* The machine governs the update rate its safety case needs: adopt the
         * heartbeat window it advertises in every accepted reply (0 = reject
         * paths, ignored). Takes effect on the next scheduled send. */
    if ((resp.heartbeat_timeout != 0u) && (resp.heartbeat_timeout != s->req_hb_ms)) {
      s->req_hb_ms = resp.heartbeat_timeout;
      ESP_LOGI(
        TAG,
        "m%d machine requests heartbeat window %lu ms -> sending every %lu ms",
        slot,
        (unsigned long)s->req_hb_ms,
        (unsigned long)sess_send_period_ms(s));
    }

    if (s->state == SESS_BONDING) {
      /* Bond response: adopt the machine's counter/stamp so the first OK
             * heartbeat carries the right context (mirrors the handshake in
             * pstop_c/examples/client/client_app.c). */
      s->pd.last_received_counter = resp.counter;
      s->pd.last_timestamp = resp.stamp;
      s->pd.msg_counter = s->bond_counter + 1u;
      s->state = SESS_BONDED;
      s->bond_sent_ms = 0;
      ESP_LOGI(
        TAG,
        "m%d BOND ok: machine counter=%lu stamp=%llu",
        slot,
        (unsigned long)resp.counter,
        (unsigned long long)resp.stamp);
    } else {
      s->pd.last_received_counter = resp.counter;
      s->pd.last_timestamp = resp.stamp;
      if ((resp.received_counter > 0u) && (resp.received_counter <= s->pd.msg_counter)) {
        uint64_t sent_ms = s->tx_stamp_history[resp.received_counter & 15];
        if ((sent_ms > 0u) && (now_ms >= sent_ms)) {
          s->rtt_ms = (uint32_t)(now_ms - sent_ms);
        }
      }
    }
    s->last_reply_ms = now_ms;
    s->last_msg = resp.message;
    s->replies++;
    got++;
  }
  return got;
}

/* ============================================================================
 * Comparator task — drives the 10 Hz tick, dispatches to the cores,
 * memcmps the two encodings per session, transmits on match, and runs every
 * session's bond FSM + reply-loss watchdog. Never blocks on any one machine.
 * ========================================================================== */

static void comparator_task(void * arg)
{
  (void)arg;

  /* Wait for at least one netif to be up. Tether/Tailscale bring-up runs
     * concurrently in ml_app — 5 s is enough on every bench config. */
  vTaskDelay(pdMS_TO_TICKS(5000));

  /* Subscribe to the TWDT now (after the startup delay, so the unfed window
     * above can't trip it). Fed every tick below — a wedged comparator stops
     * the feed and the 5 s TWDT resets the chip. */
  (void)esp_task_wdt_add(NULL);

  for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
    g_sess[i].sock = -1;
    g_sess[i].state = SESS_IDLE;
  }

  uint32_t mismatch = 0;
  uint64_t last_unhealthy_kick_ms = 0;
  bool xc_announced = false;
  TickType_t xc_fault_tick = 0;
  uint32_t padcfg_tick = 0; /* decimates the ~1 s GPIO pad-config re-verify */
  bool padcfg_announced = false;
  TickType_t padcfg_fault_tick = 0;

  TickType_t next = xTaskGetTickCount();
  for (;;) {
    (void)esp_task_wdt_reset();
    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);

    /* 0. Dual-core clock cross-check reaction (SR-H-04 / DU-2). A core latched
         *    a fault: the peer core's task or esp_timer clock stalled. FAIL-SAFE
         *    — send NOTHING to any machine (each heartbeat-times-out to STOP),
         *    let that STOP settle, then a controlled reset for recovery. The
         *    grace deadline is measured in FreeRTOS ticks, NOT esp_timer, since
         *    the fault may BE a frozen esp_timer. esp_restart() is a CLEAN reset
         *    (reset_reason SW): a frozen clock is a hardware fault that an OTA
         *    rollback cannot fix, so it is deliberately not counted toward the
         *    rollback ladder — the device simply restarts, and while any fault
         *    persists it stays silent and every machine stays safely STOPped. */
    uint32_t xc_fault = (uint32_t)atomic_load(&g_xcheck_fault);
    if (xc_fault != XCHECK_FAULT_NONE) {
      if (!xc_announced) {
        xc_announced = true;
        xc_fault_tick = xTaskGetTickCount();
        ESP_LOGE(
          TAG,
          "XCHECK FAULT %lu — halting all pstop TX (machines will STOP); controlled reset in %lu ms",
          (unsigned long)xc_fault,
          (unsigned long)XCHECK_RESET_GRACE_MS);
      }
      dcs_publish_xcheck(xc_fault, (uint32_t)atomic_load(&g_xcheck_hb[0]), (uint32_t)atomic_load(&g_xcheck_hb[1]));
      if ((TickType_t)(xTaskGetTickCount() - xc_fault_tick) >= pdMS_TO_TICKS(XCHECK_RESET_GRACE_MS)) {
        ESP_LOGE(TAG, "XCHECK: controlled reset now");
        dcs_controlled_reset(DCS_CTRL_RST_XCHECK); /* crumb + flush + restart */
      }
      vTaskDelayUntil(&next, TICK_PERIOD);
      continue; /* send nothing this tick */
    }

    /* 0b. E-stop GPIO pad-config re-verification (SR-R-09 / DU-1). Read back
         *     both IN pads ~once per second; if the input-direction/pull-down
         *     integrity is lost (an open loop could then float to a false-OK),
         *     latch a sticky fault. React the same way as the clock cross-check:
         *     send NOTHING to any machine so every heartbeat times out to STOP,
         *     hold that silence for ESTOP_PADCFG_RESET_GRACE_MS, then take a
         *     controlled reset — estop_init's re-init recovers an SEU-corrupted
         *     pad config; a persistent silicon fault re-trips the check on the
         *     next boot and the unit cycles silent-STOP again (never false-OK). */
    if (++padcfg_tick >= ESTOP_PADCFG_REVERIFY_TICKS) {
      padcfg_tick = 0;
      (void)estop_padcfg_reverify();
    }
    uint32_t padcfg_fault = (uint32_t)atomic_load(&g_gpio_cfg_fault);
    dcs_publish_gpio_cfg(padcfg_fault);
    if (padcfg_fault != 0u) {
      if (!padcfg_announced) {
        padcfg_announced = true;
        padcfg_fault_tick = xTaskGetTickCount();
        ESP_LOGE(
          TAG,
          "GPIO PADCFG FAULT 0x%lx — halting all pstop TX (machines will STOP); controlled reset in %lu ms",
          (unsigned long)padcfg_fault,
          (unsigned long)ESTOP_PADCFG_RESET_GRACE_MS);
      }
      if ((TickType_t)(xTaskGetTickCount() - padcfg_fault_tick) >= pdMS_TO_TICKS(ESTOP_PADCFG_RESET_GRACE_MS)) {
        ESP_LOGE(TAG, "GPIO PADCFG FAULT: controlled reset now (re-init recovers an SEU; a persistent fault re-trips)");
        dcs_controlled_reset(DCS_CTRL_RST_PADCFG); /* crumb + flush + restart */
      }
      vTaskDelayUntil(&next, TICK_PERIOD);
      continue; /* fail-safe: send nothing -> machines STOP */
    }

    /* 1. Refresh session configs from the peer slots (live /api updates)
         *    and run socket bring-up + the bond FSM for each. */
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      bool cfg = false;
      uint32_t ip = 0;
      uint32_t id = 0;
      uint16_t port = 0;
      dcs_get_pstop_peer_slot(i, &cfg, &ip, &port, &id);
      if ((cfg != s->configured) || (cfg && ((ip != s->ip) || (port != s->port) || (id != s->machine_id)))) {
        ESP_LOGI(TAG, "m%d peer config changed — resetting session", i);
        sess_reconfigure(s, cfg, ip, port, id);
      }
      if (!s->configured) {
        continue;
      }
      if (!sess_ensure_socket(s, i)) {
        continue; /* binding unavailable (VPN down): dark = fail-safe */
      }
      if (s->state == SESS_BONDING) {
        sess_bond_step(s, i, now_ms);
      }
    }

    /* 2. Publish this tick's per-session inputs for the cores. A session is
         *    active this tick only when its machine-governed send period has
         *    elapsed (sess_send_period_ms) — sends are decimated ticks. The
         *    E-stop rolling level advances every tick regardless. */
    g_tick_roll++;
    bool any_active = false;
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      g_tick[i].active = (s->configured && (s->state == SESS_BONDED) && (s->sock >= 0) && (now_ms >= s->next_send_ms));
      if (g_tick[i].active) {
        g_tick[i].stamp_ms = now_ms;
        g_tick[i].counter = s->pd.msg_counter;
        g_tick[i].received_counter = s->pd.last_received_counter;
        g_tick[i].received_stamp = s->pd.last_timestamp;
        g_tick[i].receiver_id = s->machine_id;
        s->tx_stamp_history[s->pd.msg_counter & 15] = now_ms;
        any_active = true;
      }
    }

    /* 3. Drain any stale completion signal before notifying. If a core
         *    published just *after* CORE_PUBLISH_TIMEOUT on a previous tick,
         *    its g_done give is still pending and would otherwise satisfy this
         *    tick's xSemaphoreTake immediately — before the core has re-encoded
         *    for the new g_tick — so the comparator would memcmp a stale (or
         *    half-written) buffer. Clearing first guarantees each take below
         *    corresponds to THIS tick's encode. */
    xSemaphoreTake(g_done[0], 0);
    xSemaphoreTake(g_done[1], 0);

    /* 4. Notify both cores; they sample their loop, encode every active
         *    slot, and signal back. */
    xTaskNotifyGive(g_core_h[0]);
    xTaskNotifyGive(g_core_h[1]);

    bool both_in = (xSemaphoreTake(g_done[0], CORE_PUBLISH_TIMEOUT) == pdTRUE) &
                   (xSemaphoreTake(g_done[1], CORE_PUBLISH_TIMEOUT) == pdTRUE);

    /* 5. Lockstep check across ALL active slots. Any disagreement taints
         *    the DEVICE — send nothing to anyone this tick; every machine
         *    fail-safes independently on its heartbeat timeout. */
    bool lockstep_ok = both_in;
    if (both_in) {
      for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
        if (g_tick[i].active && (memcmp(g_encoded[0][i], g_encoded[1][i], PSTOP_MESSAGE_SIZE) != 0)) {
          lockstep_ok = false;
        }
      }
    }

    bool sent_any = false;
    if (!both_in) {
      mismatch++;
      ESP_LOGW(TAG, "core publish timeout — sending nothing (mismatch=%lu)", (unsigned long)mismatch);
    } else if (!lockstep_ok) {
      mismatch++;
      ESP_LOGW(TAG, "ENCODING MISMATCH v0=%u v1=%u — sending nothing to any machine", g_verdict[0], g_verdict[1]);
    } else if (!estop_primed()) {
      /* Boot-priming hold: both E-stop channels haven't yet been sampled
             * on both phases. Send NOTHING so the artifact priming-STOP never
             * reaches a machine — otherwise it would auto-complete the
             * machine's NEED_STOP arming and run the robot without an operator
             * press. Not a fault; the machines stay safely stopped meanwhile. */
    } else if (any_active) {
      /* 6. Cores agreed and the E-stop loops are live: transmit to every
             *    bonded machine on its own socket. */
      for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
        pstop_sess_t * s = &g_sess[i];
        if (!g_tick[i].active) {
          continue;
        }
        if (sess_sendto(s, g_encoded[0][i])) {
          s->sent++;
          sent_any = true;
        } else {
          sess_count_send_fail(s);
        }
      }
    } else {
      /* No bonded machines this tick — nothing to send. */
    }

    /* 7. Reply pass. Wait briefly (bounded) so the NEXT tick's
         *    received_counter/received_stamp reflect the latest machine state
         *    — the pstop protocol checks `req.received_counter ==
         *    machine.msg_counter`, and being even one off triggers MSG_LOST on
         *    every subsequent message. One shared select() over every open
         *    socket keeps the wait bounded regardless of machine count; the
         *    per-session drains then read every queued reply, newest wins. */
    bool bond_pending = false;
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      if (g_sess[i].configured && (g_sess[i].state == SESS_BONDING) && (g_sess[i].sock >= 0)) {
        bond_pending = true;
      }
    }
    if (sent_any || bond_pending) {
      fd_set rfds;
      FD_ZERO(&rfds);
      int maxfd = -1;
      for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
        if (g_sess[i].sock >= 0) {
          FD_SET(g_sess[i].sock, &rfds);
          if (g_sess[i].sock > maxfd) {
            maxfd = g_sess[i].sock;
          }
        }
      }
      if (maxfd >= 0) {
        struct timeval tv = {.tv_sec = 0, .tv_usec = REPLY_WAIT_MS * 1000};
        (void)select(maxfd + 1, &rfds, NULL, NULL, &tv);
      }
    }
    uint32_t got_any = 0;
    uint64_t drain_now = (uint64_t)(esp_timer_get_time() / 1000);
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      if (g_sess[i].configured && (g_sess[i].sock >= 0)) {
        got_any += sess_drain_replies(&g_sess[i], i, drain_now);
      }
    }

    /* 8. Per-session reply-loss watchdog + counter advance + send schedule.
         *    A session whose machine went silent re-bonds ALONE; its machine
         *    sits in fail-safe STOP until heartbeats resume. The counter
         *    advances only on TRANSMITTING ticks (send success or not — a
         *    failed send leaves a single-step gap, the correct lost-message
         *    signal), so the machine sees contiguous counters regardless of
         *    the adopted send period. */
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      if (!s->configured || (s->state != SESS_BONDED)) {
        continue;
      }
      if (g_tick[i].active) {
        s->pd.msg_counter++;
        s->next_send_ms = now_ms + (uint64_t)sess_send_period_ms(s);
      }
      if ((drain_now - s->last_reply_ms) > sess_rebond_after_ms(s)) {
        ESP_LOGW(
          TAG,
          "m%d no reply for %llu ms — re-bonding that session",
          i,
          (unsigned long long)(drain_now - s->last_reply_ms));
        s->rebonds++;
        atomic_fetch_add(&g_dcs_pstop_rebonds, 1);
        sess_start_bonding(s);
      }
    }

    /* 8b. Sustained-ENOTCONN escalation (cold-bond far-side gap, bench
         *     2026-08-08): every send to a machine failing with ENOTCONN for
         *     30+ s while the tailnet is otherwise CONNECTED means the FAR
         *     side has a WG entry for us but no keypair and is silently
         *     ignoring our handshake initiations — classically a machine that
         *     rebooted before its netmap re-arrived, so it no longer knows our
         *     static key. Restarting this remote cannot fix that (observed:
         *     27 min wedge). Kick two levers, once per backoff period
         *     (30 s first, then doubling to a 300 s cap):
         *       - dcs_notify_peer_health(ip,false): forces WG re-handshakes
         *         toward that peer (covers the transient side);
         *       - dcs_request_announce(): a coordination re-announce so the
         *         control plane re-pushes US into the machine's map stream —
         *         the only remote-side action that can restore the far side's
         *         knowledge of our key.
         *     Everything here is non-blocking (queue post + RAM flag) and
         *     bounded per tick; counters land in /state.json
         *     (pstop_sf_enotconn / pstop_sf_enotconn_kicks) so a bench can
         *     watch it fire. */
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      if (!s->configured || (s->enotconn_since_ms == 0u)) {
        continue;
      }
      if ((drain_now - s->enotconn_since_ms) < (uint64_t)PSTOP_ENOTCONN_ESCALATE_MS) {
        continue;
      }
      if (drain_now < s->enotconn_next_kick_ms) {
        continue;
      }
      if (!dcs_tailnet_connected()) {
        continue; /* no control plane: the announce can't help; WG wake already runs */
      }
      if (s->enotconn_backoff_ms == 0u) {
        s->enotconn_backoff_ms = PSTOP_ENOTCONN_ESCALATE_MS;
      }
      /* Schedule the next kick, then double toward the cap. */
      s->enotconn_next_kick_ms = drain_now + (uint64_t)s->enotconn_backoff_ms;
      s->enotconn_backoff_ms = (s->enotconn_backoff_ms >= (PSTOP_ENOTCONN_BACKOFF_CAP_MS / 2u))
                                 ? PSTOP_ENOTCONN_BACKOFF_CAP_MS
                                 : (s->enotconn_backoff_ms * 2u);
      dcs_notify_peer_health(s->ip, false);
      dcs_request_announce();
      g_sf_enotconn_kicks++;
      ESP_LOGW(
        TAG,
        "m%d ENOTCONN for %llu ms — far side may not know our key; "
        "forcing re-handshake + coord re-announce (kick %lu, next in %lu ms)",
        i,
        (unsigned long long)(drain_now - s->enotconn_since_ms),
        (unsigned long)g_sf_enotconn_kicks,
        (unsigned long)(s->enotconn_next_kick_ms - drain_now));
    }

    /* 9. Telemetry: per-machine + legacy aggregates. Aggregate health for
         *    the transport: the priority link is "healthy" while ANY machine
         *    is replying; only when every configured session is silent do we
         *    ask the transport to force a fresh WG handshake. */
    uint32_t agg_sent = 0;
    uint32_t agg_fail = 0;
    uint32_t agg_replies = 0;
    uint32_t agg_rtt = 0;
    uint64_t agg_last_reply = 0;
    uint32_t agg_msg = PSTOP_MESSAGE_OK;
    bool any_cfg = false;
    bool any_fresh = false;
    bool any_stop = false;
    bool any_nonok = false;
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      if (!s->configured) {
        dcs_publish_pstop_machine(i, 0, 0, 0, 0, 0, 0, 0, 0, (uint8_t)SESS_IDLE);
        continue;
      }
      any_cfg = true;
      agg_sent += s->sent;
      agg_fail += s->send_fail;
      agg_replies += s->replies;
      if (s->last_reply_ms > agg_last_reply) {
        agg_last_reply = s->last_reply_ms;
        agg_rtt = s->rtt_ms;
      }
      if ((s->state == SESS_BONDED) && ((drain_now - s->last_reply_ms) <= sess_rebond_after_ms(s))) {
        any_fresh = true;
      }
      if (s->state == SESS_BONDED) {
        if (s->last_msg == PSTOP_MESSAGE_STOP) {
          any_stop = true;
        } else if (s->last_msg != PSTOP_MESSAGE_OK) {
          any_nonok = true;
        } else {
          /* OK: no aggregate escalation */
        }
      }
      dcs_publish_pstop_machine(
        i,
        s->sent,
        s->replies,
        s->send_fail,
        s->rebonds,
        s->req_hb_ms,
        s->last_reply_ms,
        s->rtt_ms,
        s->last_msg,
        (uint8_t)s->state);
      /* Per-machine transport health (multi-machine cold recovery): while
             * THIS target is bonded-and-fresh report healthy; while it is
             * configured but silent report unhealthy so microlink's wake
             * forces fresh handshakes at that peer only. Fixes the >30 s
             * re-bond after power-cycling one machine while another stayed
             * healthy (the aggregate kick below deliberately never fires in
             * the partial-health case). */
      bool slot_fresh = (s->state == SESS_BONDED) && ((drain_now - s->last_reply_ms) <= sess_rebond_after_ms(s));
      dcs_notify_peer_health(s->ip, slot_fresh);
    }
    if (any_stop) {
      agg_msg = PSTOP_MESSAGE_STOP; /* worst-of: any STOP shows as STOP */
    } else if (any_nonok) {
      agg_msg = PSTOP_MESSAGE_BOND;
    } else {
      /* all OK: agg_msg stays OK */
    }
    dcs_publish_pstop_sf_causes(
      g_sf_nomem,
      g_sf_route,
      g_sf_txdrv,
      g_sf_txdrv_recovered,
      g_sf_other,
      g_sf_enotconn,
      g_sf_enotconn_kicks,
      g_sf_last_errno);
    /* Dual-core cross-check liveness: publish the per-core heartbeats every
         * tick (healthy or not) so /state.json shows them advancing — positive
         * evidence the cross-check is running, not just a fault indicator. */
    dcs_publish_xcheck(
      (uint32_t)atomic_load(&g_xcheck_fault),
      (uint32_t)atomic_load(&g_xcheck_hb[0]),
      (uint32_t)atomic_load(&g_xcheck_hb[1]));
    atomic_store(&g_dcs_pstop_last_msg, agg_msg);
    atomic_store(&g_dcs_pstop_replies, agg_replies);
    dcs_publish_comparator(agg_sent, mismatch, agg_fail, agg_last_reply, agg_rtt);

    /* Aggregate PRIORITY-peer health (kept alongside the per-slot
         * notifications above, which cover each machine target individually):
         * healthy when any reply arrived; unhealthy at bond-retry cadence
         * only when EVERY configured session is silent. */
    if (got_any > 0u) {
      dcs_notify_priority_health(true);
    } else if (any_cfg && !any_fresh && ((drain_now - last_unhealthy_kick_ms) >= (uint64_t)BOND_RETRY_MS)) {
      dcs_notify_priority_health(false);
      last_unhealthy_kick_ms = drain_now;
    } else {
      /* partial health or recent kick: no notification this tick */
    }

    vTaskDelayUntil(&next, TICK_PERIOD);
  }
}

/* ============================================================================
 * app_main — bring up the support component, then spawn the lockstep tasks.
 * ========================================================================== */

void app_main(void); /* referenced by the IDF startup code */

void app_main(void)
{
  dcs_boot_state_t bs = dcs_support_init();

  /* Per-core completion semaphores. Cores give → comparator takes. These
     * allocate from internal heap, which can run as low as ~18 KB when WiFi +
     * USB-NCM + a Tailscale TLS handshake coincide. A silent NULL here would
     * later be passed to xSemaphoreGive/Take in the SAFETY path → assert/panic.
     * Retry briefly to ride out a transient coincident allocation; if the heap
     * is genuinely exhausted, a CLEAN reboot (does NOT count toward rollback)
     * beats a NULL-deref panic (which would). */
  for (int i = 0; (i < 20) && ((g_done[0] == NULL) || (g_done[1] == NULL)); i++) {
    if (g_done[0] == NULL) {
      g_done[0] = xSemaphoreCreateBinary();
    }
    if (g_done[1] == NULL) {
      g_done[1] = xSemaphoreCreateBinary();
    }
    if ((g_done[0] != NULL) && (g_done[1] != NULL)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (!g_done[0] || !g_done[1]) {
    ESP_LOGE(
      TAG,
      "FATAL: g_done semaphore alloc failed (internal heap exhausted) "
      "— cannot run lockstep; clean reboot to retry");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  /* Priority for the safety lockstep tasks. Must sit ABOVE the microlink/
     * Tailscale tasks (ml_net_io & ml_wg_mgr run at prio 7, ml_derp_tx &
     * ml_coord at 5) so the cores always publish their encoding within the
     * comparator's 80 ms window even under sustained WireGuard load. At prio 5
     * (== the old value) the cores were preempted by the prio-7 WG tasks,
     * which showed up as a slow climb in pstop_mismatch (core-publish-timeouts)
     * during Tailscale soaks. These tasks are tiny — compute a verdict, encode,
     * then block — so running them above WG costs the radio path nothing.
     * Stays well below the TCPIP thread (prio 18) the sockets depend on. */
#define SAFETY_TASK_PRIO 8

  /* Bring up the dual-channel E-stop GPIOs before the cores start reading
     * them. Done here (not in dcs_support) because the loops ARE the safety
     * input — they belong with the lockstep, in the auditable file. */
  estop_init();

  /* Spawn the per-core tasks first (their notify handles will be used by
     * the comparator). Creation can fail under the internal-heap floor; a NULL
     * g_core_h would later be passed to xTaskNotifyGive() → panic, so treat a
     * failed create as fatal and clean-reboot to retry (not a rollback trigger). */
  BaseType_t r0 =
    xTaskCreatePinnedToCore(core_task, "core0", 3072, (void *)(intptr_t)0, SAFETY_TASK_PRIO, &g_core_h[0], 0);
  BaseType_t r1 =
    xTaskCreatePinnedToCore(core_task, "core1", 3072, (void *)(intptr_t)1, SAFETY_TASK_PRIO, &g_core_h[1], 1);

  /* Comparator task — unpinned, the FreeRTOS scheduler can place it on
     * whichever core has slack. Higher stack budget than the cores since
     * it owns the sockets + drain logic. */
  BaseType_t rc = xTaskCreate(comparator_task, "comparator", 5120, NULL, SAFETY_TASK_PRIO, NULL);

  if ((r0 != pdPASS) || (r1 != pdPASS) || (rc != pdPASS) || (g_core_h[0] == NULL) || (g_core_h[1] == NULL)) {
    ESP_LOGE(TAG, "FATAL: safety lockstep task create failed (heap) — clean reboot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  dcs_support_finalize(&bs);
}
