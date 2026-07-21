/**
 * @file main.c
 * @brief Dual-Core Safety — lockstep pstop client.
 *
 * Two pinned tasks on cores 0 and 1 each independently build the SAME
 * outgoing pstop_msg_t and encode it to a 40-byte buffer. A comparator
 * task does a bytewise memcmp of the two encodings; only if they agree
 * does it transmit. Disagreement → send nothing → machine-side heartbeat
 * timeout → STOP.
 *
 * The whole `pstop_msg_t` is compared, including the CRC, so the safety
 * check catches RAM/cache/ALU corruption during encoding — not just a
 * mismatch in the OK/STOP verdict.
 *
 * Everything *not* part of the safety pattern (HTTP admin UI, OTA-rollback
 * ladder, NVS, telemetry, network-liveness watchdog, USB/WiFi boot path)
 * lives in components/dcs_support/.
 *
 * Modelled on pstop_c/examples/client/client_app.c:
 *   bond once at startup → loop send_msg(verdict) → never unbond.
 */

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "dcs_support.h"
#include "pstop/pstop_msg.h"
#include "pstop/protocol_data.h"

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define TICK_HZ                10
#define TICK_PERIOD            pdMS_TO_TICKS(1000 / TICK_HZ)

/* Each core publishes its encoded buffer within 80 ms of the tick. If a core
 * doesn't, treat that tick as a mismatch (send nothing). 80ms leaves the
 * comparator 20ms of slack to drain RX and re-align to vTaskDelayUntil's
 * 100ms tick — and gives the cores enough headroom that being briefly
 * preempted by ml_wg_mgr or httpd doesn't cause spurious mismatches. */
#define CORE_PUBLISH_TIMEOUT   pdMS_TO_TICKS(80)

/* Local UDP source port. */
#define PSTOP_LOCAL_PORT       8891

/* Device IDs — wire format expects 32-bit IDs. The chip's is fixed at build
 * time; the machine's is whatever machine_app_runner.c uses (default in
 * upstream is 0x01020304). */
#define DEVICE_ID_THIS         0x01020380u
#define DEVICE_ID_MACHINE      0x01020304u

#define HEARTBEAT_TIMEOUT_MS   1000u

/* Reply-loss watchdog. If the machine stops replying for longer than this,
 * the link has desynced and won't recover on its own (see the re-bond logic
 * in comparator_task) — re-bond to re-sync counters. Must exceed the machine
 * heartbeat timeout (1000 ms) plus normal jitter so healthy single-reply
 * drops never trigger it. */
#define REBOND_AFTER_MS        1500u

/* Bond handshake: a single BOND per attempt, then block until the machine
 * answers. Long timeout means we never have more than one BOND in flight —
 * critical because the pstop protocol rejects duplicate counters with
 * MSG_LOST once a client is registered. The 5 s wait is also long enough
 * that, between failed attempts, the machine's check_heartbeats can age
 * out a half-bonded stale client and accept our next attempt fresh. */
#define BOND_RECV_TIMEOUT_MS   5000

static const char *TAG = "dcs_main";

/* Last message TYPE received from the machine (PSTOP_MESSAGE_*), published for
 * the LED ring (dcs_pstop_ring.c): yellow=disconnected, blue=BOND/UNBOND,
 * green=OK, red=STOP. Written on every machine reply (bond + heartbeat). */
extern atomic_uint_fast32_t g_dcs_pstop_last_msg;

/* ============================================================================
 * Lockstep state — single tick's worth of inputs, snapshotted by the
 * comparator before notifying the cores. Cores read this read-only.
 * ========================================================================== */

typedef struct {
    uint64_t stamp_ms;
    uint32_t counter;
    uint32_t received_counter;
    uint64_t received_stamp;
} tick_input_t;

static tick_input_t      g_tick;
static uint8_t           g_encoded[2][PSTOP_MESSAGE_SIZE];
static uint8_t           g_verdict[2];          /* for telemetry */
static SemaphoreHandle_t g_done[2];             /* core_id → comparator */
static TaskHandle_t      g_core_h[2];           /* comparator → core_id */
static int               g_sock = -1;
static struct sockaddr_in g_peer;               /* set during bond */

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
 * comparator's memcmp fails -> nothing is sent -> the machine heartbeat-times
 * out and stops. No single fault can mask a real stop.
 * ========================================================================== */

#define ESTOP_SETTLE_US  10   /* propagation through the wire + switch contacts */

/* Release-direction debounce: after ANY unhealthy read, this many consecutive
 * healthy ticks are required before the channel reports closed again. The
 * open->STOP edge stays SINGLE-TICK — the stop path is never filtered; this
 * only extends how long a STOP episode lasts, so EMC-induced blips (measured
 * 100-200 ms both-channel flaps under WiFi TX, 2026-07-21) produce a >=300 ms
 * episode instead of chattering — defence-in-depth under the machine-side
 * min-STOP-duration arming policy (which is the enforcement point). */
#define LOOP_RECLOSE_DEBOUNCE_TICKS  3U

/* Boot warm-up: the comparator sends NOTHING until each channel has settled —
 * either one full closed-debounce cycle (loops proven healthy) or this many
 * CONSECUTIVE open reads (button genuinely held at boot -> STOP flows, just
 * ~500 ms later, well before the pstop bond completes). Without the
 * consecutive-open requirement, the first loop sample glitching open for a
 * tick (observed on every boot of this board) put a ~200 ms STOP->OK episode
 * on the wire — the arming gesture — at every power-on. */
#define LOOP_BOOT_OPEN_CONFIRM_TICKS 5U

static const struct { gpio_num_t out; gpio_num_t in; } g_estop_ch[2] = {
    { GPIO_NUM_39, GPIO_NUM_40 },   /* core 0 — channel A */
    { GPIO_NUM_41, GPIO_NUM_42 },   /* core 1 — channel B */
};

static struct {
    bool high_ok;       /* most recent drive-high tick read IN==1 (closed)      */
    bool low_ok;        /* most recent drive-low  tick read IN==0 (not shorted) */
    bool primed_high;
    bool primed_low;
    uint8_t closed_streak;  /* consecutive healthy ticks (release debounce) */
    uint8_t open_streak;    /* consecutive unhealthy ticks (boot warm-up only) */
    bool settled;           /* debounce warm-up done: one full closed-debounce
                             * cycle observed, OR LOOP_BOOT_OPEN_CONFIRM_TICKS
                             * consecutive open reads (button held at boot).
                             * Until BOTH channels settle, the comparator's
                             * boot-priming hold sends nothing — the first
                             * boot samples glitch open on this board, and
                             * without the hold that put a STOP->OK episode
                             * (the arming gesture) on the wire every boot. */
} g_estop_st[2];

static void estop_init(void) {
    for (int c = 0; c < 2; c++) {
        gpio_config_t out_cfg = {
            .pin_bit_mask = 1ULL << g_estop_ch[c].out,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&out_cfg);
        gpio_config_t in_cfg = {
            .pin_bit_mask = 1ULL << g_estop_ch[c].in,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* OPEN loop -> reads 0 -> STOP */
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&in_cfg);
        gpio_set_level(g_estop_ch[c].out, 0);
    }
    (void)memset(g_estop_st, 0, sizeof(g_estop_st));
    ESP_LOGI(TAG, "E-stop loops: chA=GPIO%d->GPIO%d  chB=GPIO%d->GPIO%d",
             g_estop_ch[0].out, g_estop_ch[0].in,
             g_estop_ch[1].out, g_estop_ch[1].in);
}

/* Drive THIS core's loop with the tick's rolling level, sample the echo, and
 * return true iff the loop is proven closed-and-healthy. Each core touches only
 * its own pin pair, so the two cores never contend. */
static bool estop_channel_closed(int core_id, uint32_t counter) {
    const gpio_num_t out = g_estop_ch[core_id].out;
    const gpio_num_t in  = g_estop_ch[core_id].in;
    const int level = ((counter & 1u) != 0u) ? 1 : 0;

    gpio_set_level(out, level);
    esp_rom_delay_us(ESTOP_SETTLE_US);
    const int rb = gpio_get_level(in);

    if (level != 0) { g_estop_st[core_id].high_ok = (rb == 1); g_estop_st[core_id].primed_high = true; }
    else            { g_estop_st[core_id].low_ok  = (rb == 0); g_estop_st[core_id].primed_low  = true; }

    dcs_publish_estop(core_id, g_estop_st[core_id].high_ok, g_estop_st[core_id].low_ok);

    const bool raw_closed =
        g_estop_st[core_id].primed_high && g_estop_st[core_id].primed_low &&
        g_estop_st[core_id].high_ok    && g_estop_st[core_id].low_ok;

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
            g_estop_st[core_id].settled = true;   /* held open: STOP flows */
        }
    }

    return raw_closed &&
           (g_estop_st[core_id].closed_streak >= LOOP_RECLOSE_DEBOUNCE_TICKS);
}

/* True once BOTH cores have sampled their loop on both rolling phases. The
 * comparator holds off sending until then (see comparator_task), so the
 * one-time boot-priming STOP never reaches the machine. That artifact STOP
 * would otherwise complete the machine's NEED_STOP arming (a STOP->OK cycle)
 * and arm the robot to OK with NO operator action — arming MUST be a deliberate
 * manual press->release. After priming, the only STOP->OK on the wire comes
 * from a real E-stop press then release. (Primed flags latch true for the run,
 * so this gate only affects the ~2-tick startup window.) */
static bool estop_primed(void) {
    return g_estop_st[0].primed_high && g_estop_st[0].primed_low &&
           g_estop_st[1].primed_high && g_estop_st[1].primed_low &&
           g_estop_st[0].settled     && g_estop_st[1].settled;
}

/* ============================================================================
 * Safety verdict — each core reads its OWN E-stop channel. OK only while the
 * loop is proven closed; otherwise STOP. The two cores' verdicts are
 * cross-checked downstream by the comparator's byte-compare (disagreement =>
 * send nothing => machine stops), so a per-core read here is the intended
 * design, not a lockstep hazard: identical inputs (both loops closed, or both
 * open) produce identical messages; only a genuine channel split diverges.
 * ========================================================================== */
static uint8_t compute_verdict(const tick_input_t *in, int core_id) {
    return estop_channel_closed(core_id, in->counter) ? PSTOP_MESSAGE_OK
                                                       : PSTOP_MESSAGE_STOP;
}

/* ============================================================================
 * Per-core task — pinned to a single ESP32-S3 core.
 *
 * On each notification from the comparator: read the shared tick_input,
 * compute MY verdict, build pstop_msg_t, encode to per-core 40-byte slot,
 * signal completion.
 * ========================================================================== */

static void core_task(void *arg) {
    int core_id = (int)(intptr_t)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Snapshot the tick inputs the comparator just published. */
        tick_input_t in = g_tick;
        uint8_t verdict = compute_verdict(&in, core_id);

        pstop_msg_t msg;
        pstop_message_init(&msg);
        msg.message           = verdict;
        msg.id.data           = DEVICE_ID_THIS;
        msg.receiver_id.data  = DEVICE_ID_MACHINE;
        msg.stamp             = in.stamp_ms;
        msg.received_stamp    = in.received_stamp;
        msg.counter           = in.counter;
        msg.received_counter  = in.received_counter;
        msg.heartbeat_timeout = HEARTBEAT_TIMEOUT_MS;
        /* pstop_message_encode computes the CRC over bytes [0..37] and
         * writes it to bytes [38..39] — both cores will produce
         * byte-identical buffers given identical input fields. */
        pstop_message_encode(&msg, g_encoded[core_id]);

        g_verdict[core_id] = verdict;
        dcs_publish_core_tick(core_id, in.counter, verdict);

        xSemaphoreGive(g_done[core_id]);
    }
}

/* ============================================================================
 * Socket bring-up + bond handshake.
 * ========================================================================== */

static int open_socket(uint32_t local_ip_host) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        ESP_LOGE(TAG, "socket(): errno=%d", errno);
        return -1;
    }
    struct sockaddr_in local = {
        .sin_family      = AF_INET,
        .sin_port        = htons(PSTOP_LOCAL_PORT),
        .sin_addr.s_addr = local_ip_host ? htonl(local_ip_host) : INADDR_ANY,
    };
    if (bind(s, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind(): errno=%d", errno);
        close(s);
        return -1;
    }
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    return s;
}

/* Local address the current g_sock is bound to (host order; 0 = ANY). */
static uint32_t s_sock_local_ip;

static void refresh_peer(void);

/* Make sure g_sock exists and is bound correctly for the CURRENT peer.
 *
 * Tailscale peers (100.64.0.0/10) get a socket source-bound to our VPN IP
 * so lwIP's source-based routing pins egress to the WireGuard netif. This
 * is a SAFETY property, not an optimization: with an INADDR_ANY socket,
 * lwIP routes per packet by destination, and whenever the WG session
 * lapses (netif link down) the CGNAT destination silently falls through
 * to the default route — observed 2026-07-20 on the bench as the pstop
 * heartbeat riding the USB tether IN PLAINTEXT after a boot where the
 * bond preceded Tailscale registration. Source-bound, a downed tunnel
 * makes sendto FAIL instead: send_fail climbs, the machine times out to
 * STOP, and bond_with_retry blocks here until the VPN is back. Fail safe,
 * never fail open. */
static void ensure_socket_for_peer(void) {
    uint32_t want = 0;
    for (;;) {
        refresh_peer();
        uint32_t peer_ip = ntohl(g_peer.sin_addr.s_addr);
        if ((peer_ip & 0xFFC00000u) != 0x64400000u) { want = 0u; break; }
        want = dcs_get_vpn_ip();
        if (want != 0u) { break; }
        ESP_LOGW(TAG, "pstop peer is a Tailscale IP but VPN is not up — "
                      "holding bond (machine sees silence = fail-safe STOP)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if ((g_sock >= 0) && (want == s_sock_local_ip)) { return; }
    if (g_sock >= 0) { close(g_sock); g_sock = -1; }
    /* Retry forever, not exit: transient fd exhaustion or a VPN-IP bind
     * racing the netif address assignment must not kill the safety task. */
    for (int tries = 1; g_sock < 0; tries++) {
        g_sock = open_socket(want);
        if (g_sock < 0) {
            ESP_LOGE(TAG, "pstop socket bring-up failed (try %d) — retry in 2s", tries);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    s_sock_local_ip = want;
    ESP_LOGI(TAG, "pstop socket local=%lu.%lu.%lu.%lu port=%u (peer %s:%u)",
             (unsigned long)((want >> 24) & 0xFF), (unsigned long)((want >> 16) & 0xFF),
             (unsigned long)((want >> 8) & 0xFF),  (unsigned long)(want & 0xFF),
             PSTOP_LOCAL_PORT, inet_ntoa(g_peer.sin_addr), ntohs(g_peer.sin_port));
}

/* Refresh the peer endpoint from the dcs_support-published NVS-backed
 * value. Called at boot and on every bond attempt so /api/pstop_peer
 * changes take effect without a reboot. */
static void refresh_peer(void) {
    uint32_t ip;
    uint16_t port;
    dcs_get_initial_pstop_peer(&ip, &port);
    g_peer.sin_family      = AF_INET;
    g_peer.sin_port        = htons(port);
    g_peer.sin_addr.s_addr = htonl(ip);
}

/* Send a single message (already encoded into `bytes`). Returns true if
 * the send succeeded; doesn't wait for a reply. */
static bool send_encoded(const uint8_t *bytes) {
    int n = sendto(g_sock, bytes, PSTOP_MESSAGE_SIZE, 0,
                   (struct sockaddr *)&g_peer, sizeof(g_peer));
    return n == PSTOP_MESSAGE_SIZE;
}

/* Blocking receive with timeout, returns true if a valid pstop_msg was
 * decoded into *out. */
static bool recv_with_timeout(pstop_msg_t *out, uint32_t timeout_ms) {
    uint64_t start_ms = (uint64_t)(esp_timer_get_time() / 1000);
    for (;;) {
        uint8_t buf[PSTOP_MESSAGE_SIZE];
        int n = recvfrom(g_sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n == PSTOP_MESSAGE_SIZE) {
            pstop_message_decode(out, buf);
            return out->checksum == out->calculated_checksum;
        }
        uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
        if (now_ms - start_ms >= timeout_ms) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* Drain anything queued on the socket. Used before each BOND attempt so
 * late responses from a previous boot or attempt don't fool us into
 * thinking the *current* BOND succeeded. */
static void drain_socket(void) {
    for (;;) {
        uint8_t junk[PSTOP_MESSAGE_SIZE];
        if (recvfrom(g_sock, junk, sizeof(junk), 0, NULL, NULL) <= 0) { break; }
    }
}

/* Send BOND, wait for response. Retries forever — chip is useless without
 * a bonded peer. Each attempt uses a fresh counter; stale packets are
 * drained before each send so the machine sees exactly one BOND per cycle.
 * Updates *pd's handshake state from the response — last_sent_counter and
 * last_timestamp, plus msg_counter set to the next counter to send — so the
 * first OK heartbeat carries the right context. Mirrors the counter handshake
 * in pstop_c/examples/client/client_app.c, run over the device's own
 * failover/Tailscale-aware socket rather than pstop_c's POSIX transport. */
static void bond_with_retry(protocol_data_t *pd) {
    uint32_t counter = 0;
    for (;;) {
        ensure_socket_for_peer();   /* refreshes peer + rebinds if it changed */
        drain_socket();
        counter++;

        pstop_msg_t req;
        pstop_message_init(&req);
        req.message           = PSTOP_MESSAGE_BOND;
        req.id.data           = DEVICE_ID_THIS;
        req.receiver_id.data  = pd->remote_id.data;
        req.stamp             = (uint64_t)(esp_timer_get_time() / 1000);
        req.counter           = counter;
        req.heartbeat_timeout = HEARTBEAT_TIMEOUT_MS;
        uint8_t req_bytes[PSTOP_MESSAGE_SIZE];
        pstop_message_encode(&req, req_bytes);

        ESP_LOGI(TAG, "BOND attempt counter=%lu -> %s:%u",
                 (unsigned long)counter,
                 inet_ntoa(g_peer.sin_addr), ntohs(g_peer.sin_port));

        if (!send_encoded(req_bytes)) {
            ESP_LOGW(TAG, "BOND sendto failed (errno=%d) — retrying", errno);
            vTaskDelay(pdMS_TO_TICKS(BOND_RECV_TIMEOUT_MS));
            continue;
        }

        pstop_msg_t resp;
        if (!recv_with_timeout(&resp, BOND_RECV_TIMEOUT_MS)) {
            /* The 5 s wait IS the backoff — no extra delay, just loop back
             * and try again. */
            ESP_LOGW(TAG, "BOND no response in %dms — retrying", BOND_RECV_TIMEOUT_MS);
            continue;
        }
        ESP_LOGI(TAG, "BOND ok: machine counter=%lu stamp=%llu",
                 (unsigned long)resp.counter, (unsigned long long)resp.stamp);

        pd->last_sent_counter = resp.counter;
        pd->last_timestamp    = resp.stamp;
        pd->msg_counter       = counter + 1u; /* next counter for the first OK heartbeat */
        atomic_store(&g_dcs_pstop_last_msg, resp.message);
        return;
    }
}

/* ============================================================================
 * Comparator task — drives the 10 Hz tick, dispatches to the cores,
 * memcmps the two encodings, transmits on match.
 * ========================================================================== */

static void comparator_task(void *arg) {
    (void)arg;

    /* Wait for at least one netif to be up. Tether/Tailscale bring-up runs
     * concurrently in ml_app — 5 s is enough on every bench config. */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Socket bring-up is peer-aware: a Tailscale peer waits for the VPN and
     * source-binds to it (egress pinned to the WG netif — see
     * ensure_socket_for_peer). Retries internally forever; the machine sees
     * silence (= STOP) until the right transport is actually available. */
    ensure_socket_for_peer();

    /* pstop_c remote handshake state (counters + timestamps). Adopted from
     * pstop_c's protocol_data_t so the device tracks the upstream remote data
     * model; transport stays the device's own (failover/Tailscale-aware). */
    protocol_data_t pd;
    protocol_data_init(&pd);
    pd.remote_id.data = DEVICE_ID_MACHINE;   /* the machine we heartbeat to */
    pd.heartbeat_ms   = HEARTBEAT_TIMEOUT_MS;
    bond_with_retry(&pd);

    uint32_t sent      = 0;
    uint32_t replies   = 0;
    uint32_t mismatch  = 0;
    uint32_t send_fail = 0;
    uint32_t rtt_ms    = 0;
    /* Seed the reply baseline at bond time so the reply-loss watchdog doesn't
     * fire before the first heartbeat round-trip completes. */
    uint64_t last_reply_ms = (uint64_t)(esp_timer_get_time() / 1000);
    uint64_t tx_stamp_history[16] = {0};

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        /* 1. Capture this tick's inputs (single source of truth for both
         *    cores). Re-read the peer endpoint so /api/pstop_peer updates
         *    take effect within one tick. */
        refresh_peer();
        uint64_t stamp_ms = (uint64_t)(esp_timer_get_time() / 1000);
        g_tick.stamp_ms         = stamp_ms;
        g_tick.counter          = pd.msg_counter;
        g_tick.received_counter = pd.last_sent_counter;
        g_tick.received_stamp   = pd.last_timestamp;
        tx_stamp_history[pd.msg_counter & 15] = stamp_ms;

        /* 2. Drain any stale completion signal before notifying. If a core
         *    published just *after* CORE_PUBLISH_TIMEOUT on a previous tick,
         *    its g_done give is still pending and would otherwise satisfy this
         *    tick's xSemaphoreTake immediately — before the core has re-encoded
         *    for the new g_tick — so the comparator would memcmp a stale (or
         *    half-written) buffer. Clearing first guarantees each take below
         *    corresponds to THIS tick's encode. (With the cores at priority 8 a
         *    late publish is now rare, but this closes the race for good.) */
        xSemaphoreTake(g_done[0], 0);
        xSemaphoreTake(g_done[1], 0);

        /* 3. Notify both cores; they read g_tick, encode, signal back. */
        xTaskNotifyGive(g_core_h[0]);
        xTaskNotifyGive(g_core_h[1]);

        /* 3. Wait for both publications. Either core failing to deliver
         *    in CORE_PUBLISH_TIMEOUT counts as mismatch — send nothing. */
        bool both_in =
            (xSemaphoreTake(g_done[0], CORE_PUBLISH_TIMEOUT) == pdTRUE) &
            (xSemaphoreTake(g_done[1], CORE_PUBLISH_TIMEOUT) == pdTRUE);

        bool sent_this_tick = false;
        if (!both_in) {
            mismatch++;
            ESP_LOGW(TAG, "core publish timeout — sending nothing (mismatch=%lu)",
                     (unsigned long)mismatch);
        } else if (memcmp(g_encoded[0], g_encoded[1], PSTOP_MESSAGE_SIZE) != 0) {
            /* Cores disagreed on the encoded message — could be a verdict
             * difference (safety logic split brain) or a corruption bug.
             * Either way, send nothing; machine will STOP on heartbeat
             * timeout. */
            mismatch++;
            ESP_LOGW(TAG,
                     "ENCODING MISMATCH counter=%lu v0=%u v1=%u — sending nothing",
                     (unsigned long)pd.msg_counter,
                     g_verdict[0], g_verdict[1]);
        } else if (!estop_primed()) {
            /* Boot-priming hold: both E-stop channels haven't yet been sampled
             * on both phases. Send NOTHING so the artifact priming-STOP never
             * reaches the machine — otherwise it would auto-complete the
             * machine's NEED_STOP arming and run the robot without an operator
             * press. Not a fault; the machine stays safely stopped meanwhile. */
        } else {
            /* Cores agreed and the E-stop loops are live. Transmit. */
            if (send_encoded(g_encoded[0])) {
                sent++;
                sent_this_tick = true;
            } else {
                send_fail++;
            }
        }

        /* 4. If we sent, synchronously wait briefly for the machine's
         *    response so the NEXT tick's received_counter/received_stamp
         *    reflect the latest machine state. The pstop protocol checks
         *    `req.received_counter == machine.msg_counter` — being even
         *    one off triggers MSG_LOST on every subsequent message. A
         *    short blocking recv here keeps the chip and machine
         *    counters in lockstep without depending on socket-drain
         *    timing across tick boundaries. */
        if (sent_this_tick) {
            /* Read EVERY queued reply, newest wins. A transient path change
             * (e.g. pstop peer switched to a Tailscale IP whose first packets
             * ride DERP before the direct path forms) can queue a burst of
             * late replies; reading only one per tick leaves that backlog
             * standing forever — the link then runs with a permanent
             * ~backlog-sized counter lag that inflates rtt_ms and silently
             * eats the machine's max_lost_messages margin (observed
             * 2026-07-20: constant 600 ms "rtt" on a 5 ms direct path).
             * Draining re-syncs pd.last_sent_counter to the newest reply
             * within one tick. */
            pstop_msg_t resp;
            bool got = recv_with_timeout(&resp, 50);
            while (got) {
                pd.last_sent_counter = resp.counter;
                pd.last_timestamp    = resp.stamp;
                last_reply_ms        = (uint64_t)(esp_timer_get_time() / 1000);
                replies++;
                atomic_store(&g_dcs_pstop_last_msg, resp.message);
                if ((resp.received_counter > 0u) && (resp.received_counter <= pd.msg_counter)) {
                    uint64_t sent_ms = tx_stamp_history[resp.received_counter & 15];
                    uint64_t now_ms  = last_reply_ms;
                    if ((sent_ms > 0u) && (now_ms >= sent_ms)) {
                        rtt_ms = (uint32_t)(now_ms - sent_ms);
                    }
                }
                got = recv_with_timeout(&resp, 0);   /* non-blocking drain */
            }
        }

        /* Publish both the comparator counters and the reply counter via
         * the g_dcs_pstop_replies atomic exposed by dcs_support. */
        extern atomic_uint_fast32_t g_dcs_pstop_replies;
        atomic_store(&g_dcs_pstop_replies, replies);
        dcs_publish_comparator(sent, mismatch, send_fail, last_reply_ms, rtt_ms);

        pd.msg_counter++;

        /* Reply-loss watchdog. A dropped burst of replies leaves our
         * received_counter behind the machine's msg_counter; after that the
         * machine rejects every message as MSG_LOST and the link is wedged
         * permanently (we otherwise bond only once). Re-bond to re-sync so the
         * link self-heals after a transient comms glitch. We send nothing while
         * re-bonding, so the machine stays in STOP throughout — the intended
         * fail-safe — and resumes once heartbeats return. */
        uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
        if ((now_ms - last_reply_ms) > REBOND_AFTER_MS) {
            ESP_LOGW(TAG, "no machine reply for %llu ms — re-bonding to re-sync",
                     (unsigned long long)(now_ms - last_reply_ms));
            extern atomic_uint_fast32_t g_dcs_pstop_rebonds;
            atomic_fetch_add(&g_dcs_pstop_rebonds, 1);
            bond_with_retry(&pd);
            last_reply_ms = (uint64_t)(esp_timer_get_time() / 1000);
            next = xTaskGetTickCount();   /* re-align cadence after blocking bond */
            continue;
        }

        vTaskDelayUntil(&next, TICK_PERIOD);
    }
}

/* ============================================================================
 * app_main — bring up the support component, then spawn the lockstep tasks.
 * ========================================================================== */

void app_main(void);   /* referenced by the IDF startup code */

void app_main(void) {
    dcs_boot_state_t bs = dcs_support_init();

    /* Per-core completion semaphores. Cores give → comparator takes. These
     * allocate from internal heap, which can run as low as ~18 KB when WiFi +
     * USB-NCM + a Tailscale TLS handshake coincide. A silent NULL here would
     * later be passed to xSemaphoreGive/Take in the SAFETY path → assert/panic.
     * Retry briefly to ride out a transient coincident allocation; if the heap
     * is genuinely exhausted, a CLEAN reboot (does NOT count toward rollback)
     * beats a NULL-deref panic (which would). */
    for (int i = 0; (i < 20) && ((g_done[0] == NULL) || (g_done[1] == NULL)); i++) {
        if (g_done[0] == NULL) { g_done[0] = xSemaphoreCreateBinary(); }
        if (g_done[1] == NULL) { g_done[1] = xSemaphoreCreateBinary(); }
        if ((g_done[0] != NULL) && (g_done[1] != NULL)) { break; }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!g_done[0] || !g_done[1]) {
        ESP_LOGE(TAG, "FATAL: g_done semaphore alloc failed (internal heap exhausted) "
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
#define SAFETY_TASK_PRIO  8

    /* Bring up the dual-channel E-stop GPIOs before the cores start reading
     * them. Done here (not in dcs_support) because the loops ARE the safety
     * input — they belong with the lockstep, in the auditable file. */
    estop_init();

    /* Spawn the per-core tasks first (their notify handles will be used by
     * the comparator). Creation can fail under the internal-heap floor; a NULL
     * g_core_h would later be passed to xTaskNotifyGive() → panic, so treat a
     * failed create as fatal and clean-reboot to retry (not a rollback trigger). */
    BaseType_t r0 = xTaskCreatePinnedToCore(core_task, "core0", 3072,
                            (void *)(intptr_t)0, SAFETY_TASK_PRIO, &g_core_h[0], 0);
    BaseType_t r1 = xTaskCreatePinnedToCore(core_task, "core1", 3072,
                            (void *)(intptr_t)1, SAFETY_TASK_PRIO, &g_core_h[1], 1);

    /* Comparator task — unpinned, the FreeRTOS scheduler can place it on
     * whichever core has slack. Higher stack budget than the cores since
     * it owns the socket + drain logic. */
    BaseType_t rc = xTaskCreate(comparator_task, "comparator", 5120, NULL,
                                SAFETY_TASK_PRIO, NULL);

    if ((r0 != pdPASS) || (r1 != pdPASS) || (rc != pdPASS) ||
        (g_core_h[0] == NULL) || (g_core_h[1] == NULL)) {
        ESP_LOGE(TAG, "FATAL: safety lockstep task create failed (heap) — clean reboot");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    }

    dcs_support_finalize(&bs);
}
