// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file main.c — ESP32-S3 pstop MACHINE node ("machn").
 *
 * The machine role of pstop_c on the remote's platform, inverting the
 * remote's lockstep: the remote's two cores independently ENCODE and the
 * comparator transmits only on agreement; here the two cores independently
 * DECODE + DECIDE and the comparator actuates/replies only on agreement.
 *
 *   - Each core runs its OWN pstop_c machine instance (own state, own
 *     remotes table). Every received datagram is processed by BOTH.
 *   - The comparator compares (library verdict, reply bytes). Agreement →
 *     reply sent, relays driven to the agreed state. Disagreement →
 *     internal fault: NO reply (remotes see silence and stop), both relay
 *     outputs commanded open, mismatch counter incremented. Transient by
 *     design — agreement on a later tick resumes normally, like the remote.
 *   - Each core drives ITS OWN relay from ITS OWN instance's verdict; the
 *     two relay contacts are wired in series in the robot's stop circuit,
 *     so either core alone stops the robot. Energized = run; any reset,
 *     brownout, or hang fails to STOP.
 *   - Feedback: a resistor divider on each relay's switched output, read
 *     as a digital input by the comparator every tick. A persistent
 *     commanded-vs-observed contradiction raises a (self-clearing) relay
 *     fault indication; safety never depends on it — the series partner
 *     carries the stop.
 *     NOTE (2026-08): relay-feedback monitoring is DESCOPED by default via
 *     the Kconfig gate CONFIG_MACHN_RELAY_FEEDBACK (default n; see the gate
 *     comment below). A new machine-specific hardware revision + wiring
 *     diagram is coming in which the divider feedback voltages cannot be read
 *     sanely; the read-back path is therefore gated OFF (relays are still
 *     DRIVEN exactly as before). Rationale, reversal steps, and safety-case
 *     impact: docs/RELAY_FEEDBACK_DESCOPE.md. Set the Kconfig to y to restore.
 *
 * pstop_c (certification track) is UNMODIFIED — this file is the shell.
 * Design: docs/MACHINE_ESP32_DESIGN.md. Scaffold status: timing config is
 * compile-time constants; operator authorization is now NVS-backed (the
 * operator allowlist via dcs_operator_*, managed at /api/operators) so unlisted
 * remotes are STOP-ONLY by default. Telemetry reuses the dcs comparator/core
 * atomics (machn-specific /state.json fields are a follow-up).
 */

#include <stdatomic.h>
#include <stdlib.h> /* strtoul for the operator-hostname parse */
#include <string.h>

#include "dcs_support.h"
#include "esp_log.h"
#include "esp_system.h" /* esp_restart on a clock fault */
#include "esp_task_wdt.h" /* TWDT subscribe/feed for the comparator */
#include "esp_timer.h"
#include "microlink.h" /* peer-wanted hook: pin operator remotes past the peer cap */
#include "sdkconfig.h" /* CONFIG_MACHN_RELAY_FEEDBACK — relay-feedback gate */
// clang-format off
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// clang-format on
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "pstop/machine.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"
#include "pstop_aux_channel.h"

static const char * TAG = "machn";

/* === Role configuration (compile-time scaffold; NVS/admin UI later) ====== */
#define MACHN_LISTEN_PORT 8890
#define MACHN_MACHINE_ID 0x01020304u /* what remotes put in receiver_id */
#define MACHN_MAX_REMOTES 8
#define MACHN_MAX_LOST_MESSAGES 10u
#define MACHN_MAX_MISSED_HEARTBEATS \
  5u /* was 3; raised to ride through ~90min control-plane re-sync RTT spikes (2026-08-04) */
#define MACHN_MIN_STOP_MS 500u /* -> library delay_between_stop_ms */
#define MACHN_DEFAULT_HEARTBEAT_MS 400u /* advertised: remotes publish at /2 */

/* === Relay I/O (reuses the remote's E-stop loop pins) ===================== */
#define RELAY_A_DRIVE 39 /* core 0 -> relay 1 coil driver */
#define RELAY_A_SENSE 40 /* divider on relay 1's switched output (feedback; gated, see below) */
#define RELAY_B_DRIVE 41 /* core 1 -> relay 2 coil driver */
#define RELAY_B_SENSE 42 /* divider on relay 2's switched output (feedback; gated, see below) */

/* ========================================================================= *
 *  RELAY FEEDBACK MONITORING — Kconfig GATE (default: DISABLED)              *
 * ------------------------------------------------------------------------- *
 *  Gated by CONFIG_MACHN_RELAY_FEEDBACK (machn/main/Kconfig.projbuild,       *
 *  default n; pinned to n in machn/sdkconfig.defaults). Set it to y          *
 *  (menuconfig -> "machn (machine node) configuration", or                   *
 *  CONFIG_MACHN_RELAY_FEEDBACK=y in sdkconfig.defaults) to RE-ENABLE relay   *
 *  read-back (feedback) monitoring. When n the macro is undefined, so        *
 *  `#if CONFIG_MACHN_RELAY_FEEDBACK` evaluates to 0.                         *
 *                                                                            *
 *  The relay DRIVE path is UNAFFECTED either way — both coils are still      *
 *  commanded from each core's own verdict exactly as before, and the        *
 *  de-energize-to-safe series-relay stop is fully intact.                   *
 *                                                                            *
 *  WHY DISABLED (2026-08): a new machine-specific hardware revision + wiring *
 *  diagram is incoming on which the resistor-divider feedback voltages       *
 *  cannot be read sanely, so the read-back would produce meaningless levels  *
 *  and spurious relay faults. Self-monitoring relays are expected to return  *
 *  on the new hardware, hence this is a REVERSIBLE gate, not a deletion:     *
 *  the entire feedback implementation is retained below under `#if`.         *
 *                                                                            *
 *  WHAT THE GATE CONTROLS when disabled (feedback OFF):                      *
 *    - the RELAY_x_SENSE GPIOs are not configured or read;                   *
 *    - no commanded-vs-observed contradiction is computed;                   *
 *    - no relay-feedback fault is raised and the feedback-driven             *
 *      STOP-on-contradiction (machine_stop_robot) is NOT armed;              *
 *    - telemetry reports relay_fault_a/b = 0, relay_stop = 0 and a new       *
 *      relay_feedback_monitored = 0 flag so consumers can tell "no fault"    *
 *      apart from "not monitored" (see dcs_publish_relay_fault).             *
 *                                                                            *
 *  TO RE-ENABLE on the new hardware: set CONFIG_MACHN_RELAY_FEEDBACK=y (and  *
 *  re-verify the SENSE pin numbers / divider ratios for the new wiring). No  *
 *  other change needed.                                                      *
 *                                                                            *
 *  Full rationale + safety-case impact: docs/RELAY_FEEDBACK_DESCOPE.md       *
 * ========================================================================= */

#define RELAY_FEEDBACK_MS 100u /* commanded->observed settle allowance (TBD by part) */
#define RELAY_FAULT_STOP_TICKS \
  10u /* persistent-fault stop: this many CONSECUTIVE contradiction ticks \
       * (~1 s) forces the robot STOPPED — both relays open, replies STOP, \
       * re-arming refused — until the contradiction clears (no latch). \
       * Degraded redundancy = don't run (operator decision 2026-07-31). */

#define TICK_MS 100u /* comparator/liveness cadence, matches the remote */
#define CORE_WINDOW_MS 80u /* per-core processing budget within a tick */
#define SAFETY_TASK_PRIO 8 /* above WG tasks, below TCPIP — remote rationale */

/* Independent-timebase clock sanity (SR-H-04b machine-side / FMEA DU-2). The
 * machn's entire liveness case (machine_validate_heartbeats) runs on esp_timer
 * via g_tick_now_ms — a SINGLE latched clock (FMEDA MC-4, beta≈1). If esp_timer
 * freezes or runs backward while the FreeRTOS scheduler keeps ticking, heartbeat
 * timeouts stop firing and a dead/silent remote keeps looking alive -> the
 * series relays stay ENERGIZED (RUN) = dangerous-undetected. The comparator
 * cross-checks esp_timer against xTaskGetTickCount() (a DIFFERENT time base):
 * on a frozen or backward esp_timer it forces STOP (opens both relays +
 * machine_stop_robot on both cores) and does a controlled reset for recovery —
 * the machine analogue of the remote's dual-core clock cross-check
 * (docs/WATCHDOG_CLOCK_PROTECTION.md). */
#define MACHN_CLK_STALL_TICKS pdMS_TO_TICKS(500) /* esp_timer no-advance tolerance */
#define MACHN_CLK_RESET_GRACE_MS 500u /* STOP settle before the controlled reset */
#define MACHN_TWDT_STARTUP_DELAY_MS 5000u /* match the remote: don't subscribe unfed */

/* === Per-core machine instances =========================================== */
typedef struct
{
  pstop_application_t app;
  pstop_machine_t machine;
  pstop_remote_data_t remotes[MACHN_MAX_REMOTES];
  /* Outputs of the last processed op, read by the comparator after done. */
  pstop_error_t err;
  pstop_msg_t resp;
  uint8_t resp_bytes[PSTOP_MESSAGE_SIZE];
  int robot_state; /* ROBOT_STATE_OK / ROBOT_STATE_STOPPED after the op */
  uint32_t seq; /* cycle these outputs belong to (g_cycle_seq) */
} machn_core_t;

static machn_core_t g_core[2];
static TaskHandle_t g_core_h[2];
static SemaphoreHandle_t g_done[2];

/* Datagram staged for both cores (comparator writes, cores read). */
static uint8_t g_rx_bytes[PSTOP_MESSAGE_SIZE];
static volatile int g_rx_pending; /* 1 = staged datagram this tick */

/* Relay command/observe state (comparator-owned). g_relay_cmd[] is part of the
 * DRIVE path (records what each core commanded) and stays live regardless of
 * the feedback gate; the fault counters below exist only for feedback. */
static volatile int g_relay_cmd[2]; /* last commanded level per channel */
static uint64_t g_relay_cmd_ms[2]; /* when it was commanded (settle window) */
#if CONFIG_MACHN_RELAY_FEEDBACK
static uint32_t g_relay_fault[2]; /* consecutive contradiction ticks */
static uint32_t g_relay_fault_total;
static volatile int g_relay_fault_stop; /* persistent-fault stop engaged */
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */

/* Live bonded-remote view for the landing page (comparator-owned; read
 * against core 0's instance BETWEEN core windows so it never races the
 * cores). rtt is the stamp-echo delta: our reply's stamp comes back in the
 * remote's next message's received_stamp, so (now - received_stamp)
 * bounds the true RTT plus the remote's inter-send hold (<= its period). */
typedef struct
{
  uint32_t id;
  uint32_t ip; /* UDP source (host order) = the remote's tailnet IP */
  uint64_t last_rx_ms;
  uint32_t rtt_ms;
} machn_seen_t;

static machn_seen_t g_seen[DCS_MACHN_MAX_REMOTES];

/* Remote-announced role map (aux uplink). The safety cores read it from
 * details_default when a remote bonds; the comparator is the single writer and
 * always writes BEFORE it notifies the cores for the tick, so the FreeRTOS
 * notify barrier makes every core read see a consistent id/role pair. 32-bit
 * atomics (natively lock-free on Xtensa), mirroring the operator allowlist
 * cache. id 0 = empty slot. */
static atomic_uint_fast32_t g_claimed_id[DCS_MACHN_MAX_REMOTES];
static atomic_uint_fast32_t g_claimed_role[DCS_MACHN_MAX_REMOTES];

/* The announced role for id, or UNSPECIFIED if unknown. */
static pstop_aux_role_t claimed_role_get(uint32_t id)
{
  if (id == 0u) {
    return PSTOP_AUX_ROLE_UNSPECIFIED;
  }
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    if ((uint32_t)atomic_load(&g_claimed_id[i]) == id) {
      return (pstop_aux_role_t)atomic_load(&g_claimed_role[i]);
    }
  }
  return PSTOP_AUX_ROLE_UNSPECIFIED;
}

/* Record id's latest announced role. Returns true iff id was already known with
 * a DIFFERENT role (a transition the caller must contain). Comparator-only. */
static bool claimed_role_note(uint32_t id, pstop_aux_role_t role)
{
  if (id == 0u) {
    return false;
  }
  int free_slot = -1;
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    if ((uint32_t)atomic_load(&g_claimed_id[i]) == id) {
      bool changed = ((pstop_aux_role_t)atomic_load(&g_claimed_role[i]) != role);
      atomic_store(&g_claimed_role[i], role);
      return changed;
    }
    if ((free_slot < 0) && ((uint32_t)atomic_load(&g_claimed_id[i]) == 0u)) {
      free_slot = i;
    }
  }
  if (free_slot >= 0) {
    atomic_store(&g_claimed_role[free_slot], role); /* role before id: reader keys off id */
    atomic_store(&g_claimed_id[free_slot], id);
  }
  return false; /* brand-new remote: bonds fresh with the right role, no reset */
}

/* Release id's slot when the remote ages out. Comparator-only. */
static void claimed_role_forget(uint32_t id)
{
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    if ((uint32_t)atomic_load(&g_claimed_id[i]) == id) {
      atomic_store(&g_claimed_id[i], 0u);
      return;
    }
  }
}

static void seen_note(uint32_t id, uint32_t ip, uint64_t now, uint64_t received_stamp)
{
  int free_slot = -1;
  int slot = -1;
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    if (g_seen[i].id == id) {
      slot = i;
      break;
    }
    if ((free_slot < 0) && (g_seen[i].id == 0u)) {
      free_slot = i;
    }
  }
  if (slot < 0) {
    slot = free_slot;
  }
  if (slot < 0) {
    return; /* table full — telemetry only, never blocks the protocol */
  }
  g_seen[slot].id = id;
  g_seen[slot].ip = ip;
  g_seen[slot].last_rx_ms = now;
  if ((received_stamp != 0u) && (now >= received_stamp) && ((now - received_stamp) < 60000u)) {
    g_seen[slot].rtt_ms = (uint32_t)(now - received_stamp);
  }
}

static void seen_publish(uint64_t now)
{
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    if (g_seen[i].id == 0u) {
      dcs_publish_machn_remote(i, 0u, 0u, 0u, 0u, 0u, false, PSTOP_AUX_ROLE_UNSPECIFIED);
      continue;
    }
    uint64_t age = (now > g_seen[i].last_rx_ms) ? (now - g_seen[i].last_rx_ms) : 0u;
    if (age > 30000u) { /* gone half a minute: drop from the page */
      dcs_untrack_peer_health(g_seen[i].ip); /* stop heartbeat-pinging it */
      claimed_role_forget(g_seen[i].id); /* release its aux-role slot */
      g_seen[i].id = 0u;
      dcs_publish_machn_remote(i, 0u, 0u, 0u, 0u, 0u, false, PSTOP_AUX_ROLE_UNSPECIFIED);
      continue;
    }
    /* Keep the disco heartbeat (and thus the path-RTT sample) alive toward
         * every remote we're actively serving; 'healthy' while its messages
         * are fresher than the library timeout, which also gives the
         * machine->remote direction the zombie-session recovery kick. */
    dcs_notify_peer_health(g_seen[i].ip, age < 2000u);
    uint32_t st = 0u;
    bool stop_only = false;
    device_id_t rid = {.data = g_seen[i].id};
    const pstop_remote_data_t * rd = machine_get_remote_data(&g_core[0].machine, &rid);
    if (rd != NULL) {
      st = (uint32_t)rd->remote_state;
      stop_only = rd->is_stop_only; /* library's per-remote authorization (latched at bond) */
    }
    dcs_publish_machn_remote(
      i, g_seen[i].id, g_seen[i].ip, st, (uint32_t)age, g_seen[i].rtt_ms, stop_only, claimed_role_get(g_seen[i].id));
  }
}

static uint64_t now_ms(void)
{
  return (uint64_t)(esp_timer_get_time() / 1000);
}

/* Tick-latched wall clock. The library stamps REPLY BYTES with
 * get_time_cb() and takes every time-boundary decision (heartbeat
 * timeouts, the min STOP->OK delay) from it; if each core reads the live
 * clock they straddle millisecond boundaries ~25% of the time and the
 * comparator sees divergent replies — a mismatch storm that withheld a
 * quarter of all replies and re-bonded the remote every few seconds
 * (bench, 2026-07-31). The comparator latches the time ONCE per cycle;
 * both cores compute from the identical value, like the remote's TX
 * lockstep. */
static volatile uint64_t g_tick_now_ms;

/* Sequence tag: outputs are only comparable if both cores processed the
 * SAME cycle. A core that missed its window leaves a stale done-semaphore
 * give behind; without the tag the next cycle compares one core's fresh
 * result against the other's previous one — permanently desynced. */
static volatile uint32_t g_cycle_seq;

/* === pstop_c callbacks (per-core trampolines — the lib passes no ctx) ==== */

static remote_details_t details_default(const device_id_t * device_id)
{
  remote_details_t d;
  /* Authorization policy. Every bonding remote is ACCEPTED (allowed=true) and
     * heartbeat-monitored, but STOP-ONLY by default: it may command STOP, never
     * re-arm (STOP->OK). Only a remote whose 32-bit pstop id is on the
     * NVS-backed operator allowlist (dcs_operator_*, empty on blank NVS) is a
     * full operator (stop_only=false). This REPLACES the old bench scaffold that
     * accepted any remote as a full operator. Read is lock-free RAM (safe on the
     * safety cores). Latched onto the client at bond (pstop_c machine.c
     * add_new_client) — an operator added later applies on the remote's next
     * bond. */
  const bool is_listed = (device_id != NULL) && dcs_operator_is_listed(device_id->data);
  /* AND-rule: a remote may re-arm only if it BOTH announces the operator role
     * (aux uplink) AND is on the machine allowlist. A stop-only/unspecified
     * claim, or an unlisted id, stays stop-only. Strictly more conservative than
     * the allowlist alone: a provisioning gap surfaces as stop-only, never as an
     * unexpected operator. The role is latched with is_stop_only at bond; a later
     * role change forces a re-bond (see role_change_reset). */
  const pstop_aux_role_t claimed = (device_id != NULL) ? claimed_role_get(device_id->data) : PSTOP_AUX_ROLE_UNSPECIFIED;
  const bool is_operator = is_listed && pstop_aux_role_is_operator(claimed);
  remote_detail_set(&d, true, MACHN_DEFAULT_HEARTBEAT_MS, !is_operator);
  return d;
}

/* status_cb fires inside machine_process/validate on the calling core's
 * task; robot_state is snapshotted after the call instead, so these only
 * exist to satisfy the non-NULL callback contract. */
static void status_noop_core0(pstop_status_message_t status)
{
  (void)status;
}

static void status_noop_core1(pstop_status_message_t status)
{
  (void)status;
}

static void log_noop(uint64_t timestamp, const device_id_t * client, uint8_t message, pstop_error_t error)
{
  (void)timestamp;
  (void)client;
  (void)message;
  (void)error;
}

static uint64_t lib_now_core(void)
{
  uint64_t t = g_tick_now_ms;
  return (t != 0u) ? t : now_ms();
}

static void core_instance_init(int c)
{
  machn_core_t * mc = &g_core[c];
  memset(mc, 0, sizeof(*mc));
  pstop_application_init(&mc->app);
  mc->app.app_config.max_lost_messages = MACHN_MAX_LOST_MESSAGES;
  mc->app.app_config.max_missed_heartbeats = MACHN_MAX_MISSED_HEARTBEATS;
  mc->app.app_config.delay_between_stop_ms = MACHN_MIN_STOP_MS;
  mc->app.remote_details_cb = details_default;
  mc->app.status_cb = (c == 0) ? status_noop_core0 : status_noop_core1;
  mc->app.log_message_cb = log_noop;
  mc->app.env.get_time_cb = lib_now_core;
  machine_init(&mc->machine, &mc->app, mc->remotes, MACHN_MAX_REMOTES);
  device_id_t mid = {.data = MACHN_MACHINE_ID};
  device_id_copy(&mc->machine.application->machine_device_id, &mid);
}

/* Contain a remote's role transition (invariant 3): drop its bond on both
 * instances and force STOP, so it must re-bond — which re-runs details_default
 * and re-latches is_stop_only from the NEW role — and a fresh stop/OK gesture is
 * required before anything can re-arm. Public pstop_c API only; runs on the
 * comparator between core windows (cores idle), like the seen_publish read. */
static void role_change_reset(uint32_t id)
{
  device_id_t rid = {.data = id};
  for (int c = 0; c < 2; c++) {
    /* Cast away const: the record lives in our own g_core[c].remotes[] and the
       * accessor is the only public way to reach it by id. */
    pstop_remote_data_t * client = (pstop_remote_data_t *)machine_get_remote_data(&g_core[c].machine, &rid);
    if (client != NULL) {
      pstop_remote_deactivate(client);
    }
    machine_stop_robot(&g_core[c].machine);
  }
  ESP_LOGW(TAG, "remote 0x%08lx role changed — bond dropped, STOP forced, re-bond required", (unsigned long)id);
}

/* === Per-core task: process staged op, run liveness, drive own relay ===== */

static void core_task(void * arg)
{
  const int c = (int)(intptr_t)arg;
  machn_core_t * mc = &g_core[c];
  const gpio_num_t drive = (c == 0) ? RELAY_A_DRIVE : RELAY_B_DRIVE;
  uint32_t tick = 0;

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tick++;
    const uint32_t seq = g_cycle_seq;

    mc->err = PSTOP_ERROR_INVALID_ID; /* placeholder; meaningful only when a datagram was staged */
    if (g_rx_pending != 0) {
      pstop_msg_t req;
      pstop_message_decode(&req, g_rx_bytes);
      memset(&mc->resp, 0, sizeof(mc->resp));
      mc->err = machine_process_message(&mc->machine, &req, &mc->resp);
      if (mc->err == PSTOP_OK) {
        /* Aux downlink (machine -> remote feedback): reserved today, emit zero.
             * Both cores write the same value, so resp_bytes stay identical. */
        pstop_aux_encode_feedback_none(&mc->resp);
        pstop_message_encode(&mc->resp, mc->resp_bytes);
      } else {
        memset(mc->resp_bytes, 0, sizeof(mc->resp_bytes));
      }
    }

    /* Native liveness on this core's own instance/clock. */
    (void)machine_validate_heartbeats(&mc->machine);

#if CONFIG_MACHN_RELAY_FEEDBACK
    /* Persistent relay-fault stop: force STOPPED via the library's public
         * API every tick while the contradiction persists. Replies become
         * STOP, arming gestures cannot complete, and this core's relay drive
         * follows the STOPPED verdict below. Identical flag on both cores =
         * identical verdicts = no comparator divergence. Clears with the
         * fault (no latch).
         * GATED: this is the ONLY point at which relay feedback feeds back
         * into the safety verdict. With feedback descoped it is compiled out,
         * so a feedback contradiction can no longer force STOP. The l0/l1
         * lockstep verdict + series-relay drive below are unchanged. */
    if (g_relay_fault_stop != 0) {
      machine_stop_robot(&mc->machine);
    }
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */

    /* This core's verdict drives THIS core's relay, independent of the
         * other core: the series wiring makes any single-core STOP a real
         * stop, so a disagreement is never less safe than agreement. */
    mc->robot_state = mc->machine.robot_state.robot_state;
    (void)gpio_set_level(drive, (mc->robot_state == ROBOT_STATE_OK) ? 1 : 0);

    dcs_publish_core_tick(c, tick, (mc->robot_state == ROBOT_STATE_OK) ? 1u : 0u);
    mc->seq = seq;
    (void)xSemaphoreGive(g_done[c]);
  }
}

/* === Relay feedback (comparator tick) ===================================== */

static void relay_feedback_check(void)
{
#if !CONFIG_MACHN_RELAY_FEEDBACK
  /* Feedback DESCOPED (default). No SENSE read, no fault computation. The
   * commanded coil levels are still published (a DRIVE-path signal, not
   * feedback) so operators can see what the firmware commanded; the observed
   * half is reported false and flagged not-monitored by the comparator's
   * dcs_publish_relay_fault(monitored=false) call, so nothing mistakes it for
   * a live divider reading. See the gate comment and
   * docs/RELAY_FEEDBACK_DESCOPE.md. */
  dcs_publish_estop(0, g_relay_cmd[0] != 0, false);
  dcs_publish_estop(1, g_relay_cmd[1] != 0, false);
#else
  const gpio_num_t sense[2] = {RELAY_A_SENSE, RELAY_B_SENSE};
  uint64_t t = now_ms();
  /* SERIES-CHAIN expectations (supply -> relay A -> relay B -> load):
     * the divider after A sees A alone; the divider after B sees the whole
     * chain, i.e. A AND B. A naive per-channel compare would raise a
     * phantom fault on B every time A opens while B is still closed. */
  int expected[2];
  int observed[2];
  expected[0] = g_relay_cmd[0];
  expected[1] = ((g_relay_cmd[0] != 0) && (g_relay_cmd[1] != 0)) ? 1 : 0;
  for (int ch = 0; ch < 2; ch++) {
    observed[ch] = gpio_get_level(sense[ch]);
    if (observed[ch] == expected[ch]) {
      g_relay_fault[ch] = 0; /* matches — indication self-clears */
      continue;
    }
    /* Settle window applies after EITHER channel's transition (a change
         * on A moves B's expectation too). */
    uint64_t last_cmd = (g_relay_cmd_ms[0] > g_relay_cmd_ms[1]) ? g_relay_cmd_ms[0] : g_relay_cmd_ms[1];
    if ((t - last_cmd) < RELAY_FEEDBACK_MS) {
      continue;
    }
    g_relay_fault[ch]++;
    g_relay_fault_total++;
    if (g_relay_fault[ch] == 1u || (g_relay_fault[ch] % 50u) == 0u) {
      ESP_LOGE(
        TAG,
        "RELAY FAULT ch%c: chain-expected %d, divider reads %d (x%u) — series partner carries the stop",
        'A' + ch,
        expected[ch],
        observed[ch],
        (unsigned)g_relay_fault[ch]);
    }
  }
  /* Publish commanded/observed per channel into /state.json NOW by
     * reusing the (machine-role-unused) E-stop channel atomics:
     *   e_hi0 = relay A commanded   e_lo0 = relay A divider observed
     *   e_hi1 = relay B commanded   e_lo1 = relay B divider observed
     * Lets bench/HIL tests assert relay state over HTTP. Deliberate,
     * clearly-labeled reuse until machn-specific state fields land. */
  dcs_publish_estop(0, g_relay_cmd[0] != 0, observed[0] != 0);
  dcs_publish_estop(1, g_relay_cmd[1] != 0, observed[1] != 0);
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */
}

/* Track what the cores commanded this tick (for the feedback window). */
static void relay_note_commands(void)
{
  for (int c = 0; c < 2; c++) {
    int level = (g_core[c].robot_state == ROBOT_STATE_OK) ? 1 : 0;
    if (level != g_relay_cmd[c]) {
      g_relay_cmd[c] = level;
      g_relay_cmd_ms[c] = now_ms();
    }
  }
}

/* === Comparator: socket, staging, agreement, reply ======================= */

static void comparator_task(void * arg)
{
  (void)arg;

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "FATAL: socket() failed (%d) — clean reboot", errno);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }
  struct sockaddr_in bind_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(MACHN_LISTEN_PORT),
    .sin_addr.s_addr = htonl(INADDR_ANY),
  };
  if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
    ESP_LOGE(TAG, "FATAL: bind(%d) failed (%d) — clean reboot", MACHN_LISTEN_PORT, errno);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }
  ESP_LOGI(TAG, "MACHINE node listening on 0.0.0.0:%d (machine_id=0x%08x)", MACHN_LISTEN_PORT, MACHN_MACHINE_ID);

  uint32_t processed = 0;
  uint32_t mismatch = 0;
  uint64_t last_rx_ms = 0;

  /* Subscribe the comparator to the TWDT after a startup delay (so the unfed
   * bring-up window can't trip it), then feed it every tick below. A wedged
   * comparator stops the feed and the TWDT resets the chip — the recovery
   * backstop under the fast clock-sanity/heartbeat reactions. */
  vTaskDelay(pdMS_TO_TICKS(MACHN_TWDT_STARTUP_DELAY_MS));
  (void)esp_task_wdt_add(NULL);

  /* Independent-timebase clock-sanity state (SR-H-04b machine-side / DU-2). */
  uint64_t clk_last_us = 0;
  TickType_t clk_last_adv_tick = 0;
  bool clk_init = false;
  bool clk_fault = false;
  bool clk_announced = false;
  TickType_t clk_fault_tick = 0;

  for (;;) {
    (void)esp_task_wdt_reset();

    /* Clock sanity FIRST: esp_timer (the liveness time base) vs the FreeRTOS
         * tick (an independent base). A frozen or backward esp_timer disables
         * every heartbeat timeout -> a dead remote would look alive -> relays
         * stay energized (RUN). Fail-safe: open both relays now, force STOP on
         * both cores, and do a controlled reset after a short STOP-settle. */
    {
      uint64_t us = (uint64_t)esp_timer_get_time();
      TickType_t nt = xTaskGetTickCount();
      if (!clk_init) {
        clk_last_us = us;
        clk_last_adv_tick = nt;
        clk_init = true;
      } else if (us < clk_last_us) {
        clk_fault = true; /* esp_timer ran backward */
      } else if (us > clk_last_us) {
        clk_last_us = us;
        clk_last_adv_tick = nt;
      } else if ((TickType_t)(nt - clk_last_adv_tick) > MACHN_CLK_STALL_TICKS) {
        clk_fault = true; /* esp_timer frozen while the scheduler still ticks */
      }
    }
    if (clk_fault) {
      TickType_t nt = xTaskGetTickCount();
      if (!clk_announced) {
        clk_announced = true;
        clk_fault_tick = nt;
        ESP_LOGE(
          TAG,
          "CLOCK FAULT: esp_timer frozen/backward vs FreeRTOS tick — heartbeat "
          "watchdog is BLIND. FAIL-SAFE STOP (relays open); controlled reset in %lu ms",
          (unsigned long)MACHN_CLK_RESET_GRACE_MS);
      }
      /* Force STOP directly (do not depend on the core round-trip, which shares
             * the suspect clock): open both series relays and stop both cores. */
      (void)gpio_set_level(RELAY_A_DRIVE, 0);
      (void)gpio_set_level(RELAY_B_DRIVE, 0);
      g_relay_cmd[0] = 0;
      g_relay_cmd[1] = 0;
      machine_stop_robot(&g_core[0].machine);
      machine_stop_robot(&g_core[1].machine);
      if ((TickType_t)(nt - clk_fault_tick) >= pdMS_TO_TICKS(MACHN_CLK_RESET_GRACE_MS)) {
        ESP_LOGE(TAG, "CLOCK FAULT: controlled reset now");
        dcs_controlled_reset(DCS_CTRL_RST_CLOCK_FAULT); /* crumb + flush + restart */
      }
      vTaskDelay(pdMS_TO_TICKS(TICK_MS));
      continue; /* no reply, no re-arm while the clock is untrustworthy */
    }

    /* Wait up to one tick for a datagram; a quiet tick still runs
         * liveness + relay supervision on both cores. */
    struct timeval tv = {.tv_sec = 0, .tv_usec = (int)(TICK_MS * 1000u)};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    int sel = select(sock + 1, &rfds, NULL, NULL, &tv);

    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    g_rx_pending = 0;
    if ((sel > 0) && FD_ISSET(sock, &rfds)) {
      int n = recvfrom(sock, g_rx_bytes, sizeof(g_rx_bytes), 0, (struct sockaddr *)&from, &from_len);
      if (n == PSTOP_MESSAGE_SIZE) {
        g_rx_pending = 1;
        last_rx_ms = now_ms();
      }
      /* Short/garbage datagrams are dropped silently (fail-safe: no
              * reply). Additional queued datagrams are handled on subsequent
              * ticks — remotes publish at >= 200 ms spacing per unit. */
    }

    /* Aux uplink: record the sender's announced role BEFORE the cores process
         * this datagram, so details_default sees the right role when a bond is
         * accepted this tick. A role transition for an already-bonded remote is
         * contained here (drop bond + STOP) so the demoted/promoted remote must
         * re-bond with the new authorization.
         *
         * Only a currently-bonded remote (or an incoming BOND, which is about to
         * become one) may touch the role table or trigger the forced STOP: the
         * CRC is a public integrity check, not authentication, so acting on an
         * arbitrary/spoofed id would let any sender on the path force a STOP or
         * pollute the table. Mirrors the host + ROS 2 backends' `known` gate. */
    if (g_rx_pending != 0) {
      pstop_msg_t rx;
      pstop_message_decode(&rx, g_rx_bytes);
      if (rx.checksum == rx.calculated_checksum) {
        device_id_t rid = {.data = rx.id.data};
        bool known = machine_get_remote_data(&g_core[0].machine, &rid) != NULL;
        if (known || (rx.message == PSTOP_MESSAGE_BOND)) {
          if (claimed_role_note(rx.id.data, pstop_aux_decode_role(&rx)) && known) {
            role_change_reset(rx.id.data);
          }
        }
      }
    }

    /* Run both cores against the staged op (or a bare liveness tick).
         * Latch the cycle clock FIRST (identical time for both instances),
         * drain any stale done-gives from a previously missed window, and
         * tag the cycle so stale outputs can never be compared as fresh. */
    g_tick_now_ms = now_ms();
    g_cycle_seq++;
    while (xSemaphoreTake(g_done[0], 0) == pdTRUE) {
    }
    while (xSemaphoreTake(g_done[1], 0) == pdTRUE) {
    }
    (void)xTaskNotifyGive(g_core_h[0]);
    (void)xTaskNotifyGive(g_core_h[1]);
    bool ok0 = (xSemaphoreTake(g_done[0], pdMS_TO_TICKS(CORE_WINDOW_MS)) == pdTRUE);
    bool ok1 = (xSemaphoreTake(g_done[1], pdMS_TO_TICKS(CORE_WINDOW_MS)) == pdTRUE);
    ok0 = ok0 && (g_core[0].seq == g_cycle_seq);
    ok1 = ok1 && (g_core[1].seq == g_cycle_seq);

    if (!ok0 || !ok1) {
      /* A core missed its window — internal fault, treat as disagreement:
             * no reply this tick; the hung core's relay is unchanged but the
             * healthy core keeps enforcing its own verdict, and a truly hung
             * core eventually trips the task WDT into a (safe) reset. */
      mismatch++;
    } else if (g_rx_pending != 0) {
      bool agree =
        (g_core[0].err == g_core[1].err) &&
        ((g_core[0].err != PSTOP_OK) || (memcmp(g_core[0].resp_bytes, g_core[1].resp_bytes, PSTOP_MESSAGE_SIZE) == 0));
      if (agree && (g_core[0].err == PSTOP_OK)) {
        (void)sendto(sock, g_core[0].resp_bytes, PSTOP_MESSAGE_SIZE, 0, (struct sockaddr *)&from, from_len);
        processed++;
      } else if (!agree) {
        /* Cores diverged on the SAME input: withhold the reply (the
                 * remote sees silence and stops) and count it. Each core has
                 * already driven its own relay from its own verdict — the
                 * series chain resolves any split toward STOP. */
        mismatch++;
        ESP_LOGW(
          TAG, "LOCKSTEP MISMATCH: core0 err=%d core1 err=%d (reply withheld)", (int)g_core[0].err, (int)g_core[1].err);
      } else {
        /* Both cores rejected identically (bad CRC, unknown sender, ...):
                 * the library contract is no reply. */
      }
    } else {
      /* Quiet tick: liveness + relays already handled by the cores. */
    }

    if (g_rx_pending != 0) {
      /* Landing-page peer view — decode is cheap (40 B) and phase-safe
             * here: both cores are idle between windows. */
      pstop_msg_t peek;
      pstop_message_decode(&peek, g_rx_bytes);
      if (peek.checksum == peek.calculated_checksum) {
        /* Real arrival time (not the tick-latched clock): the telemetry
                 * table is comparator-only, and the latched clock would inflate
                 * the echo-RTT bound by up to a whole tick. */
        seen_note(peek.id.data, ntohl(from.sin_addr.s_addr), now_ms(), peek.received_stamp);
      }
    }
    seen_publish(now_ms()); /* REAL clock: entries are stamped with now_ms() in seen_note; mixing in
                                  * the tick-latched clock made age negative -> u64 wrap -> instant ageout */

    /* Arming/restart telemetry for /state.json (remote_stop_id +
         * restart_state). Read from CORE 0's instance, like the g_seen view
         * above, and in the same comparator-only phase (both cores idle
         * between windows, so nothing races the read). Core 0 is
         * representative because the instances run in RX lockstep on
         * identical inputs and an identical latched clock: on any tick that
         * produced agreement their robot_state blocks are equal, and on a
         * divergent tick the comparator has already withheld the reply and
         * each core's relay enforces its own verdict — this read-only
         * diagnostic can lag truth by at most one 100 ms tick and feeds
         * nothing back into the safety path. */
    /* robot_state.remote_stop_id is pstop_c's LOCAL slot handle
         * (client->local_remote_id — a per-bond monotonic counter), NOT the
         * wire device id. Publishing it raw surfaced meaningless values like
         * "17" (2026-08-09). Translate to the WIRE id (same domain as
         * bonded_remotes[].id) by scanning the remotes table; 0 = no owner
         * or owner slot no longer present (both mean "no arming owner"). */
    uint32_t owner_wire_id = 0u;
    uint32_t owner_local = g_core[0].machine.robot_state.remote_stop_id;
    if (owner_local != 0u) {
      for (uint16_t ri = 0u; ri < g_core[0].machine.remotes.max_remotes; ++ri) {
        const pstop_remote_data_t * rc = &g_core[0].machine.remotes.remotes[ri];
        if ((rc->remote_state != PSTOP_REMOTE_UNKNOWN) && (rc->local_remote_id == owner_local)) {
          owner_wire_id = rc->remote_data.remote_id.data;
          break;
        }
      }
    }
    dcs_publish_machn_arm(owner_wire_id, (uint32_t)g_core[0].machine.robot_state.restart_state);

    relay_note_commands();
    relay_feedback_check();

#if CONFIG_MACHN_RELAY_FEEDBACK
    bool fault_now = (g_relay_fault[0] >= RELAY_FAULT_STOP_TICKS) || (g_relay_fault[1] >= RELAY_FAULT_STOP_TICKS);
    if (fault_now && (g_relay_fault_stop == 0)) {
      ESP_LOGE(
        TAG,
        "RELAY FAULT persistent (A=%u B=%u ticks) — forcing STOP until it clears",
        (unsigned)g_relay_fault[0],
        (unsigned)g_relay_fault[1]);
    } else if (!fault_now && (g_relay_fault_stop != 0)) {
      ESP_LOGW(TAG, "relay fault cleared — STOP force released (re-arm via remote gesture)");
    }
    g_relay_fault_stop = fault_now ? 1 : 0;
    dcs_publish_relay_fault(g_relay_fault[0], g_relay_fault[1], fault_now, /*monitored=*/true);

    dcs_publish_comparator(processed, mismatch, g_relay_fault_total, last_rx_ms, 0);
#else
    /* Feedback DESCOPED: no fault is computed and none is forced. Report the
     * fields as deterministically no-fault (relay_fault_a/b = 0, relay_stop =
     * 0 so no consumer spuriously stops — the ROS 2 machine bridge derives
     * need_stop from relay_stop) and flag relay_feedback_monitored = 0 so
     * "no fault" is distinguishable from "not monitored". */
    dcs_publish_relay_fault(0u, 0u, false, /*monitored=*/false);
    dcs_publish_comparator(processed, mismatch, 0u, last_rx_ms, 0);
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */
  }
}

/* === Bring-up ============================================================== */

static void relay_gpio_init(void)
{
  gpio_config_t out = {
    .pin_bit_mask = (1ULL << RELAY_A_DRIVE) | (1ULL << RELAY_B_DRIVE),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE, /* default-low = de-energized = STOP */
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&out));
  (void)gpio_set_level(RELAY_A_DRIVE, 0);
  (void)gpio_set_level(RELAY_B_DRIVE, 0);

#if CONFIG_MACHN_RELAY_FEEDBACK
  /* SENSE inputs are configured ONLY when relay feedback is enabled. With
   * feedback descoped (default) the RELAY_x_SENSE pads are left untouched —
   * the incoming machine hardware revision reworks this wiring, and the pin
   * numbers/divider ratios must be re-verified before the gate is flipped
   * back on. See the CONFIG_MACHN_RELAY_FEEDBACK comment block. */
  gpio_config_t in = {
    .pin_bit_mask = (1ULL << RELAY_A_SENSE) | (1ULL << RELAY_B_SENSE),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&in));
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */
}

/* microlink "keep this peer past the ML_MAX_PEERS cap" hook.
 *
 * The tailnet can exceed ML_MAX_PEERS, so every chip trims its netmap. A machn
 * pins nothing by default, so a freshly-added operator remote (no recent
 * activity) gets dropped from the netmap and machn never learns its WG key ->
 * the remote's handshakes get no session and it never bonds. Fix: when the peer
 * table is full, keep any incoming peer whose device identity is on the
 * OPERATOR allowlist (bounded, <= DCS_MAX_OPERATORS). Stop-only accept-all
 * remotes are intentionally NOT pinned here and remain best-effort.
 *
 * Identity scheme: the remote's tailnet hostname is "pstop-01<mac24>" — the 8
 * lowercase hex digits after "pstop-" ARE the 32-bit pstop id (e.g.
 * "pstop-01d7f344" -> 0x01d7f344), the exact value dcs_operator_* stores and
 * compares, so parsing the hex as a uint32_t needs no byte-order fixup.
 * Robust to any hostname that doesn't match the pattern (returns false).
 *
 * ALSO consulted by microlink's peer-NVS cache (ml_wg_mgr save path,
 * cold-bond recovery 2026-08-08): operator remotes returning true here are
 * protected from LRU eviction in the on-flash peer cache, because those
 * cached WG keys are what the boot-time preseed uses to answer a remote's
 * re-bond handshakes BEFORE this machine's netmap re-arrives (previously the
 * WG layer silently ignored them and the remote wedged ENOTCONN for 27 min). */
static bool machn_peer_wanted_cb(void * ctx, const char * hostname, uint32_t vpn_ip)
{
  (void)ctx;
  (void)vpn_ip;
  static const char kPrefix[] = "pstop-";
  const size_t plen = sizeof(kPrefix) - 1u;
  if (hostname == NULL) {
    return false;
  }
  if (strncmp(hostname, kPrefix, plen) != 0) {
    return false;
  }
  const char * hex = hostname + plen;
  /* Require exactly 8 hex digits (the pstop-01<mac24> form) then end-of-string
   * OR a '.' — netmap hostnames are FQDNs like pstop-01d7f344.tail13f8.ts.net,
   * so the id is followed by the tailnet domain suffix, not a NUL. */
  for (int i = 0; i < 8; i++) {
    const char c = hex[i];
    const bool is_hex = ((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'f')) || ((c >= 'A') && (c <= 'F'));
    if (!is_hex) {
      return false;
    }
  }
  if ((hex[8] != '\0') && (hex[8] != '.')) {
    return false;
  }
  const uint32_t id = (uint32_t)strtoul(hex, NULL, 16);
  return dcs_operator_is_listed(id);
}

void app_main(void);

void app_main(void)
{
  dcs_boot_state_t bs = dcs_support_init();

  /* Register the operator-remote pin hook now that microlink is up. Keeps a
     * newly-added operator remote from being trimmed out of machn's netmap. */
  microlink_set_peer_wanted_cb(bs.ml_handle, machn_peer_wanted_cb, NULL);

  relay_gpio_init(); /* relays open (STOP) before anything else runs */
  core_instance_init(0);
  core_instance_init(1);

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
  if ((g_done[0] == NULL) || (g_done[1] == NULL)) {
    ESP_LOGE(TAG, "FATAL: semaphore alloc failed — clean reboot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  BaseType_t r0 =
    xTaskCreatePinnedToCore(core_task, "core0", 4096, (void *)(intptr_t)0, SAFETY_TASK_PRIO, &g_core_h[0], 0);
  BaseType_t r1 =
    xTaskCreatePinnedToCore(core_task, "core1", 4096, (void *)(intptr_t)1, SAFETY_TASK_PRIO, &g_core_h[1], 1);
  BaseType_t rc = xTaskCreate(comparator_task, "comparator", 5120, NULL, SAFETY_TASK_PRIO, NULL);
  if ((r0 != pdPASS) || (r1 != pdPASS) || (rc != pdPASS) || (g_core_h[0] == NULL) || (g_core_h[1] == NULL)) {
    ESP_LOGE(TAG, "FATAL: task create failed — clean reboot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  /* Finalize platform bring-up: marks the running OTA image VALID (else
     * the bootloader rolls back on the next reboot — bit us on the bench:
     * every machn OTA bounced straight back), starts the boot-count
     * age-out, and applies any deferred Tailscale pause. */
  dcs_support_finalize(&bs);

#if CONFIG_MACHN_RELAY_FEEDBACK
  ESP_LOGI(
    TAG,
    "machn up: dual-core RX lockstep, relays A=%d/%d B=%d/%d (feedback ON)",
    RELAY_A_DRIVE,
    RELAY_A_SENSE,
    RELAY_B_DRIVE,
    RELAY_B_SENSE);
#else
  ESP_LOGI(
    TAG,
    "machn up: dual-core RX lockstep, relay drive A=%d B=%d (feedback DESCOPED — see RELAY_FEEDBACK_DESCOPE.md)",
    RELAY_A_DRIVE,
    RELAY_B_DRIVE);
#endif /* CONFIG_MACHN_RELAY_FEEDBACK */
}
