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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "pstop/protocol_data.h"
#include "pstop/pstop_msg.h"

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define TICK_HZ 10
#define TICK_PERIOD pdMS_TO_TICKS(1000 / TICK_HZ)

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
 * that session to re-sync counters. Must exceed the machine heartbeat
 * timeout (1000 ms) plus normal jitter so healthy single-reply drops never
 * trigger it. Other sessions are untouched. */
#define REBOND_AFTER_MS 1500u

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

/* Release-direction debounce: after ANY unhealthy read, this many consecutive
 * healthy ticks are required before the channel reports closed again. The
 * open->STOP edge stays SINGLE-TICK — the stop path is never filtered; this
 * only extends how long a STOP episode lasts, so EMC-induced blips (measured
 * 100-200 ms both-channel flaps under WiFi TX, 2026-07-21) produce a >=300 ms
 * episode instead of chattering — defence-in-depth under the machine-side
 * min-STOP-duration arming policy (which is the enforcement point). */
#define LOOP_RECLOSE_DEBOUNCE_TICKS 3U

/* Boot warm-up: the comparator sends NOTHING until each channel has settled —
 * either one full closed-debounce cycle (loops proven healthy) or this many
 * CONSECUTIVE open reads (button genuinely held at boot -> STOP flows, just
 * ~500 ms later, well before the pstop bond completes). Without the
 * consecutive-open requirement, the first loop sample glitching open for a
 * tick (observed on every boot of this board) put a ~200 ms STOP->OK episode
 * on the wire — the arming gesture — at every power-on. */
#define LOOP_BOOT_OPEN_CONFIRM_TICKS 5U

static const struct
{
  gpio_num_t out;
  gpio_num_t in;
} g_estop_ch[2] = {
  {GPIO_NUM_39, GPIO_NUM_40}, /* core 0 — channel A */
  {GPIO_NUM_41, GPIO_NUM_42}, /* core 1 — channel B */
};

static struct
{
  bool high_ok; /* most recent drive-high tick read IN==1 (closed)      */
  bool low_ok; /* most recent drive-low  tick read IN==0 (not shorted) */
  bool primed_high;
  bool primed_low;
  uint8_t closed_streak; /* consecutive healthy ticks (release debounce) */
  uint8_t open_streak; /* consecutive unhealthy ticks (boot warm-up only) */
  bool settled; /* debounce warm-up done: one full closed-debounce
                             * cycle observed, OR LOOP_BOOT_OPEN_CONFIRM_TICKS
                             * consecutive open reads (button held at boot).
                             * Until BOTH channels settle, the comparator's
                             * boot-priming hold sends nothing — the first
                             * boot samples glitch open on this board, and
                             * without the hold that put a STOP->OK episode
                             * (the arming gesture) on the wire every boot. */
} g_estop_st[2];

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
static bool estop_channel_closed(int core_id, uint32_t counter)
{
  const gpio_num_t out = g_estop_ch[core_id].out;
  const gpio_num_t in = g_estop_ch[core_id].in;
  const int level = ((counter & 1u) != 0u) ? 1 : 0;

  gpio_set_level(out, level);
  esp_rom_delay_us(ESTOP_SETTLE_US);
  const int rb = gpio_get_level(in);

  if (level != 0) {
    g_estop_st[core_id].high_ok = (rb == 1);
    g_estop_st[core_id].primed_high = true;
  } else {
    g_estop_st[core_id].low_ok = (rb == 0);
    g_estop_st[core_id].primed_low = true;
  }

  dcs_publish_estop(core_id, g_estop_st[core_id].high_ok, g_estop_st[core_id].low_ok);

  const bool raw_closed = g_estop_st[core_id].primed_high && g_estop_st[core_id].primed_low &&
                          g_estop_st[core_id].high_ok && g_estop_st[core_id].low_ok;

  /* Asymmetric release debounce: open reports IMMEDIATELY (streak reset),
     * closed only after LOOP_RECLOSE_DEBOUNCE_TICKS consecutive healthy
     * ticks. Both cores run this identically on their own channel, so the
     * lockstep encodings stay in agreement through the debounce window. */
  if (raw_closed) {
    if (g_estop_st[core_id].closed_streak < (uint8_t)255U) {
      g_estop_st[core_id].closed_streak++;
    }
    g_estop_st[core_id].open_streak = 0U;
    if (g_estop_st[core_id].closed_streak >= LOOP_RECLOSE_DEBOUNCE_TICKS) {
      g_estop_st[core_id].settled = true;
    }
  } else {
    g_estop_st[core_id].closed_streak = 0U;
    if (g_estop_st[core_id].open_streak < (uint8_t)255U) {
      g_estop_st[core_id].open_streak++;
    }
    if (g_estop_st[core_id].open_streak >= LOOP_BOOT_OPEN_CONFIRM_TICKS) {
      g_estop_st[core_id].settled = true; /* held open: STOP flows */
    }
  }

  return raw_closed && (g_estop_st[core_id].closed_streak >= LOOP_RECLOSE_DEBOUNCE_TICKS);
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
  return g_estop_st[0].primed_high && g_estop_st[0].primed_low && g_estop_st[1].primed_high &&
         g_estop_st[1].primed_low && g_estop_st[0].settled && g_estop_st[1].settled;
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
  return estop_channel_closed(core_id, counter) ? PSTOP_MESSAGE_OK : PSTOP_MESSAGE_STOP;
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

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Snapshot the tick inputs the comparator just published. */
    tick_input_t in[PSTOP_MAX_MACHINES];
    (void)memcpy(in, g_tick, sizeof(in));

    /* Sample the loop exactly once per tick (the drive/echo pattern is
         * stateful) — use the first active slot's counter for the rolling
         * level, or the tick LSB the comparator provides in slot 0 even when
         * no session is bonded, so priming/debounce always advances. */
    uint32_t roll = in[0].counter;
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
  uint32_t bond_counter;
  uint64_t bond_sent_ms; /* 0 = no BOND in flight */
  uint64_t last_reply_ms;
  uint64_t tx_stamp_history[16];

  /* Telemetry. */
  uint32_t sent;
  uint32_t replies;
  uint32_t send_fail;
  uint32_t rebonds;
  uint32_t rtt_ms;
  uint8_t last_msg;
} pstop_sess_t;

static pstop_sess_t g_sess[PSTOP_MAX_MACHINES];

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

static bool sess_sendto(pstop_sess_t * s, const uint8_t * bytes)
{
  struct sockaddr_in a = sess_addr(s);
  int n = sendto(s->sock, bytes, PSTOP_MESSAGE_SIZE, 0, (struct sockaddr *)&a, sizeof(a));
  return n == PSTOP_MESSAGE_SIZE;
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
    s->send_fail++;
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

    if (s->state == SESS_BONDING) {
      /* Bond response: adopt the machine's counter/stamp so the first OK
             * heartbeat carries the right context (mirrors the handshake in
             * pstop_c/examples/client/client_app.c). */
      s->pd.last_sent_counter = resp.counter;
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
      s->pd.last_sent_counter = resp.counter;
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

  for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
    g_sess[i].sock = -1;
    g_sess[i].state = SESS_IDLE;
  }

  uint32_t mismatch = 0;
  uint32_t tick_lsb = 0; /* rolling level source while no session is bonded */
  uint64_t last_unhealthy_kick_ms = 0;

  TickType_t next = xTaskGetTickCount();
  for (;;) {
    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);

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

    /* 2. Publish this tick's per-session inputs for the cores. Slot 0's
         *    counter doubles as the E-stop rolling-level source, so keep it
         *    advancing even when nothing is bonded. */
    bool any_active = false;
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      g_tick[i].active = (s->configured && (s->state == SESS_BONDED) && (s->sock >= 0));
      if (g_tick[i].active) {
        g_tick[i].stamp_ms = now_ms;
        g_tick[i].counter = s->pd.msg_counter;
        g_tick[i].received_counter = s->pd.last_sent_counter;
        g_tick[i].received_stamp = s->pd.last_timestamp;
        g_tick[i].receiver_id = s->machine_id;
        s->tx_stamp_history[s->pd.msg_counter & 15] = now_ms;
        any_active = true;
      }
    }
    if (!g_tick[0].active) {
      g_tick[0].stamp_ms = now_ms;
      g_tick[0].counter = tick_lsb; /* rolling level only; slot stays inactive */
    }
    tick_lsb++;

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
          s->send_fail++;
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

    /* 8. Per-session reply-loss watchdog + counter advance. A session whose
         *    machine went silent re-bonds ALONE; its machine sits in fail-safe
         *    STOP until heartbeats resume. Counter advances once per tick per
         *    bonded session (send success or not), matching the single-machine
         *    behaviour the protocol was validated with. */
    for (int i = 0; i < PSTOP_MAX_MACHINES; i++) {
      pstop_sess_t * s = &g_sess[i];
      if (!s->configured || (s->state != SESS_BONDED)) {
        continue;
      }
      s->pd.msg_counter++;
      if ((drain_now - s->last_reply_ms) > REBOND_AFTER_MS) {
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
        dcs_publish_pstop_machine(i, 0, 0, 0, 0, 0, 0, 0, (uint8_t)SESS_IDLE);
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
      if ((s->state == SESS_BONDED) && ((drain_now - s->last_reply_ms) <= REBOND_AFTER_MS)) {
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
        i, s->sent, s->replies, s->send_fail, s->rebonds, s->last_reply_ms, s->rtt_ms, s->last_msg, (uint8_t)s->state);
    }
    if (any_stop) {
      agg_msg = PSTOP_MESSAGE_STOP; /* worst-of: any STOP shows as STOP */
    } else if (any_nonok) {
      agg_msg = PSTOP_MESSAGE_BOND;
    } else {
      /* all OK: agg_msg stays OK */
    }
    atomic_store(&g_dcs_pstop_last_msg, agg_msg);
    atomic_store(&g_dcs_pstop_replies, agg_replies);
    dcs_publish_comparator(agg_sent, mismatch, agg_fail, agg_last_reply, agg_rtt);

    /* Healthy: replies arrived this tick. Unhealthy: EVERY configured
         * session is silent — kick the transport on the old bond-retry cadence
         * (a repeated false is what forces WG past a zombie keypair when the
         * far peer forgot us; a single edge notification is not enough — see
         * the cold-recovery notes in docs/TROUBLESHOOTING.md). A partially
         * healthy device (one machine up, another down) does NOT kick: that
         * would churn the shared transport under the healthy session. */
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
