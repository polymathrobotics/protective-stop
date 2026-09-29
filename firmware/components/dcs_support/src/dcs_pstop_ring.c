// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_pstop_ring.c
 * @brief 16-LED WS2812 ring on GPIO17 — shows the PSTOP link state as seen from
 *        the MACHINE's replies (NOT the network state; the onboard single LED
 *        on GPIO21, dcs_rgb.c, does network).
 *
 * The ring is divided evenly among the unit's pstop peers (one peer = the
 * whole ring, two = halves, etc., starting at LED 1) and each segment shows
 * that link's state. Both roles show the MACHINE's state, so a remote's ring
 * and its machine's ring agree:
 *   - remote (firmware/): one segment per CONFIGURED machine slot, from the
 *     per-machine telemetry the comparator publishes
 *     (g_dcs_pstop_m_state/_last_msg/_last_reply_ms);
 *   - machine (machn/, built with DCS_PAGE_MACHINE): one segment per ASSIGNED
 *     remote (allowlisted or pinned ids, then every remote served since boot),
 *     from the last reply the machine sent it (dcs_publish_machn_reply).
 * Device-level conditions (lockstep MISMATCH, OTA, locate, no peers at all)
 * override the whole ring. Per segment / ring:
 *
 *     WHITE  = IDLE          no pstop peer: no machine configured (remote) or
 *                            no remote assigned (machine) — nothing to connect
 *                            to (fresh unit, or peer cleared).
 *     AMBER  = UNREACHABLE   a peer IS configured but no fresh reply — AMBER
 *              (N blinks)     blinking, where the blink COUNT names the deepest
 *                            broken network layer (1 = bonded peer down / net OK;
 *                            2 = no Tailscale; 3 = no Internet). Amber (r>g) is
 *                            clearly distinct from a solid-red commanded STOP and
 *                            from yellow. A REACHABLE machine still shows its real
 *                            safety colour (same-LAN-direct needs no Internet).
 *     BLUE   = BOND/UNBOND   connected; the machine's last reply was BOND or
 *                            UNBOND — just (re)bonded, not yet armed to OK.
 *     GREEN  = OK            connected; the machine's last reply was OK — the
 *                            robot is cleared to run.
 *     RED    = STOP          connected; the machine's last reply was STOP — a
 *                            commanded stop (E-stop pressed, or the NEED_STOP
 *                            arming cycle not yet completed).
 *     PURPLE = MISMATCH      the two lockstep cores are PERSISTENTLY
 *                            disagreeing (their encoded messages differ) —
 *                            e.g. ONE E-stop loop channel opened/faulted while
 *                            the other stayed closed. The comparator sends
 *                            nothing on a mismatch, so the link would
 *                            otherwise just time out to yellow; purple flags
 *                            the real cause. Paints only after the
 *                            disagreement persists ~3 consecutive ring frames
 *                            (~750 ms); momentary blips never flash the ring
 *                            (see pstop_mismatch in /state.json for those).
 *                            Takes priority over every other colour.
 *
 * Additionally, a DIM PURPLE comet (much dimmer than the mismatch purple)
 * spins around the ring from the very top of boot via
 * dcs_pstop_ring_bootsign() — a power-on sign of life that runs while the
 * network and the rest of bring-up are still being established, i.e. before
 * the ring task exists. The ring task stops it on startup.
 *
 * Driven over a SECOND RMT TX channel (the S3 has 4); same WS2812 bit timing as
 * dcs_rgb.c, but all 16 pixels are streamed in one transmit and the inter-frame
 * task delay provides the >50us WS2812 reset.
 *
 * The ring can be installed in any of 16 rotations, so which physical pixel is
 * "LED 1" is a per-device NVS setting (ring_off): frames are composed in
 * logical coordinates and rotated at transmit time (ring_show). An installer
 * locate mode (POST /api/ring_led1) paints only logical LED 1 white so the
 * offset can be dialled in from the provisioning tooling.
 */

#include <stdatomic.h>
#include <string.h>

#include "dcs_internal.h"
#include "dcs_ring_logic.h" /* segment colour, blink count, machine assigned-remote set */
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ml_config_httpd.h" /* ml_config_ota_in_progress() */

static const char * TAG = "dcs_ring";

#define RING_GPIO 17
#define RING_LEDS 16
#define RING_RES_HZ 10000000 /* 0.1 us per RMT tick (same as dcs_rgb) */
#define RING_BRIGHTNESS 40 /* per channel; 16 LEDs — visible, low current/glare */
/* White lights all three channels, so at RING_BRIGHTNESS it reads ~3x brighter
 * than the single-channel state colours (green/red/blue/amber) — glaring in the
 * IDLE state. Dim the idle white to 50% so it's comfortable. */
#define RING_WHITE_BRIGHTNESS (RING_BRIGHTNESS / 2)

#define RING_REFRESH_MS 250 /* re-evaluate state + repaint; also the WS2812 reset gap */
#define LINK_FRESH_MS 2000u /* no machine reply within this -> not connected (re-bond is 1.5s) */
#define MISMATCH_PERSIST_FRAMES \
  3u /* PURPLE trigger threshold: the mismatch counter must advance across \
      * this many CONSECUTIVE ring frames (3 x 250 ms ~= 750 ms of continuous \
      * core disagreement) before the ring paints purple. Momentary blips \
      * (1-2 disagreeing ticks) never flash the ring — they stay visible in \
      * the pstop_mismatch counter only. */
#define MISMATCH_HOLD_MS \
  300u /* PURPLE bridging window once triggered: just over one ring frame, \
                                * so a persistent fault paints solid purple with no flicker and the \
                                * ring clears within ~one frame of the disagreement ending. */
#define PULSE_PERIOD_MS 1000u /* flashing YELLOW cycle (50% duty) when a configured host is unreachable */
#define PULSE_FRAME_MS 60 /* repaint step while flashing, keeps edges crisp */

#define RING_BOOT_BRIGHTNESS \
  10 /* dim power-on sign-of-life; well below the state colours (40)
                                       * so it can't be mistaken for a full-brightness MISMATCH purple */
#define BOOT_SPIN_FRAME_MS 70 /* comet step period: ~1.1 s per revolution of the 16 LEDs */

#define DLY(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_enc = NULL;
static uint8_t s_grb[RING_LEDS * 3]; /* WS2812 wants GRB order, per pixel — LOGICAL order (index 0 = LED 1) */

/* Rotation offset (0..15): the PHYSICAL pixel that the installed bezel makes
 * "LED 1". The ring can be mounted in any of 16 orientations; every frame is
 * rotated by this at transmit time, so all patterns (comet, locate, future
 * per-LED states) stay in logical coordinates. Loaded from NVS (ring_off) in
 * dcs_pstop_ring_start(); the boot spinner runs before NVS is up and simply
 * spins unrotated — it's a sign of life, orientation is irrelevant. */
static atomic_uint_fast32_t s_ring_offset;

/* Master brightness (0..100%): scales EVERY pixel at transmit time so all ring
 * states dim proportionally. Seeded to the default so the boot sign-of-life
 * spinner (paints before NVS is up) is visible; dcs_pstop_ring_start() replaces
 * it with the persisted led_bri. */
static atomic_uint_fast32_t s_ring_brightness_pct = DCS_LED_BRIGHTNESS_DEFAULT;

/* Locate mode: ms-uptime deadline until which ONLY logical LED 1 is painted
 * white (0 = off). Set via dcs_pstop_ring_locate(); auto-expires so a
 * forgotten locate can't mask the safety state colours indefinitely. */
static atomic_uint_fast64_t s_locate_until_ms;

/* Stream the current s_grb frame, rotated so logical pixel 0 lands on the
 * physical LED-1 position. Static tx buffer: rmt_transmit is async until
 * rmt_tx_wait_all_done, and the two painting tasks never overlap (bootsign
 * handoff in ring_task). */
static void ring_show(void)
{
  if ((s_chan == NULL) || (s_enc == NULL)) {
    return;
  }
  static uint8_t tx_grb[RING_LEDS * 3];
  uint32_t off = (uint32_t)atomic_load(&s_ring_offset);
  /* Master-brightness scale, applied HERE (the single WS2812 write for every
   * ring state) so all colours dim by the same factor. */
  uint32_t bri = (uint32_t)atomic_load(&s_ring_brightness_pct);
  for (int i = 0; i < RING_LEDS; i++) {
    int p = (int)(((uint32_t)i + off) % RING_LEDS);
    for (int c = 0; c < 3; c++) {
      tx_grb[(p * 3) + c] = (uint8_t)(((uint32_t)s_grb[(i * 3) + c] * bri) / 100u);
    }
  }
  rmt_transmit_config_t tx = {.loop_count = 0};
  if (rmt_transmit(s_chan, s_enc, tx_grb, sizeof(tx_grb), &tx) == ESP_OK) {
    (void)rmt_tx_wait_all_done(s_chan, 100);
  }
}

/* Fill all 16 pixels one colour and stream the frame. */
static void ring_fill(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < RING_LEDS; i++) {
    s_grb[i * 3] = g;
    s_grb[(i * 3) + 1] = r;
    s_grb[(i * 3) + 2] = b;
  }
  ring_show();
}

/* === Boot sign-of-life spinner ==============================================
 * A dim purple comet chases around the ring from the top of boot until the
 * ring task takes over — visual flair that says "powering on" while the
 * network and the rest of bring-up are still being established.
 *
 * Handoff: s_boot_anim_run is the run request, s_boot_anim_alive tracks the
 * spinner task's lifetime. ring_task clears the run flag and then waits for
 * alive to drop before painting, so the two tasks never interleave frames on
 * the shared s_grb / RMT channel. */

static atomic_bool s_boot_anim_run;
static atomic_bool s_boot_anim_alive;

/* One comet frame: head at RING_BOOT_BRIGHTNESS with a fading 4-pixel tail,
 * dim purple. Shared by the boot spinner and the OTA-in-progress indicator. */
static void ring_comet_frame(int head)
{
  static const uint8_t comet[4] = {RING_BOOT_BRIGHTNESS, 5, 2, 1};
  (void)memset(s_grb, 0, sizeof(s_grb));
  for (int t = 0; t < 4; t++) {
    int i = ((head - t) + RING_LEDS) % RING_LEDS;
    s_grb[(i * 3) + 1] = comet[t]; /* R */
    s_grb[(i * 3) + 2] = comet[t]; /* B — R+B = purple */
  }
  ring_show();
}

static void ring_bootsign_task(void * arg)
{
  (void)arg;
  int head = 0;
  while (atomic_load(&s_boot_anim_run)) {
    ring_comet_frame(head);
    head = (head + 1) % RING_LEDS;
    DLY(BOOT_SPIN_FRAME_MS);
  }
  atomic_store(&s_boot_anim_alive, false);
  vTaskDelete(NULL);
}

/* One-shot RMT bring-up, shared by the early boot sign-of-life and the ring
 * task (whichever runs first does the work). Attempted once; a failure is
 * latched so the task path can report "disabled" without retry loops. */
static bool ring_hw_init(void)
{
  static enum { HW_UNTRIED, HW_OK, HW_FAILED } s_hw = HW_UNTRIED;

  if (s_hw != HW_UNTRIED) {
    return (s_hw == HW_OK);
  }
  s_hw = HW_FAILED;

  rmt_tx_channel_config_t chan_cfg = {
    .gpio_num = RING_GPIO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = RING_RES_HZ,
    .mem_block_symbols = 64, /* bytes-encoder streams the 384-bit frame */
    .trans_queue_depth = 4,
  };
  if (rmt_new_tx_channel(&chan_cfg, &s_chan) != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_tx_channel(GPIO%d) failed — PSTOP ring disabled", RING_GPIO);
    s_chan = NULL;
    return false;
  }
  rmt_bytes_encoder_config_t enc_cfg = {
    .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
    .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
    .flags = {.msb_first = 1},
  };
  if ((rmt_new_bytes_encoder(&enc_cfg, &s_enc) != ESP_OK) || (rmt_enable(s_chan) != ESP_OK)) {
    ESP_LOGE(TAG, "rmt encoder/enable failed — PSTOP ring disabled");
    s_enc = NULL;
    return false;
  }
  s_hw = HW_OK;
  return true;
}

/* === Liveness comet wave (OK / STOP segments) ==============================
 * A steady green or red ring is indistinguishable from a wedged display, so
 * a subtle comet-shaped brightness wave (same design as the boot spinner)
 * chases around the ring through OK and STOP segments: head boosted above
 * the base brightness with a fading tail, everything else slightly below
 * base so the motion reads clearly. Percent-of-base per distance-from-head;
 * base far pixels at 85% keep the state colour dominant — the wave is a
 * liveness cue, not a pattern of its own. */
#define WAVE_FRAME_MS 70 /* comet step period: ~1.1 s per revolution (matches boot spinner) */

static int s_wave_head;

static uint32_t wave_scale_pct(int dist)
{
  /* Head well above base with a longer fading tail, and the rest of the
   * segment dimmed to 55% — the deeper dip is what makes the wave read
   * clearly at a glance (85% base was too subtle on the bench). */
  static const uint32_t k_scale[5] = {170u, 135u, 105u, 80u, 65u}; /* head, then tail */
  return (dist < 5) ? k_scale[dist] : 55u;
}

typedef enum
{
  RING_IDLE, /* no pstop peer configured — nothing to connect to (white) */
  RING_UNREACHABLE, /* peer configured but no fresh reply (amber, N-blink layer)  */
  RING_BOND,
  RING_OK,
  RING_STOP,
  RING_MISMATCH
} ring_state_t;

/* Connectivity-loss indication (ring-only, decided 2026-08-03): an UNREACHABLE
 * segment blinks AMBER, and the blink COUNT names the DEEPEST broken network
 * layer so a tech sees WHERE the break is at a glance:
 *   1 blink  = bonded peer down (Internet + Tailscale OK — go to the machine)
 *   2 blinks = Tailscale / control-plane down (Internet OK)
 *   3 blinks = no Internet at all (nothing reachable)
 * "More blinks = deeper problem." The safety colours (purple mismatch, red STOP,
 * green OK, blue bond, white idle) are unchanged and always win — this only
 * refines the old generic yellow flash into a diagnostic. */
#define AMBER_ON_MS 160u /* one blink on-time              */
#define AMBER_OFF_MS 200u /* gap between blinks in a group  */
#define AMBER_REST_MS 900u /* gap between blink groups (rest)*/
#define AMBER_G_PCT 40u /* green as % of red -> amber hue (r>g; distinct from yellow r==g) */

/* On (1) / off (0) for an N-blink amber group at time `now` (ms): N blinks then
 * a REST gap, repeating. Device-level layer probe picks N; every unreachable
 * segment shares it, so they blink in unison showing the same root cause. */
static uint8_t amber_on(uint64_t now, int n)
{
  uint32_t group = AMBER_ON_MS + AMBER_OFF_MS;
  uint32_t period = ((uint32_t)n * group) + AMBER_REST_MS;
  uint32_t ph = (uint32_t)(now % period);
  if (ph >= ((uint32_t)n * group)) {
    return 0u; /* rest gap between groups */
  }
  return ((ph % group) < AMBER_ON_MS) ? 1u : 0u;
}

/* Link segments for this role, in display order; returns the count (0 = no
 * peers, ring shows white). At most RING_LEDS so every segment gets a pixel.
 * Decision logic is in dcs_ring_logic.c (host-tested). */
#ifdef DCS_PAGE_MACHINE
/* Machine: one segment per assigned remote (allowlisted or pinned ids, then
 * every remote served since boot), coloured by the last reply the machine
 * sent it. That is exactly what the remote's own ring shows for this machine,
 * so the two rings agree. */
static int ring_collect_segments(uint64_t now, dcs_ring_seg_t seg[RING_LEDS])
{
  uint32_t allow[DCS_MAX_LIST_IDS];
  uint32_t pin[DCS_MAX_LIST_IDS];
  uint32_t deny[DCS_MAX_LIST_IDS];
  int nallow = dcs_list_get(DCS_LIST_ALLOW, allow);
  int npin = dcs_list_get(DCS_LIST_PIN, pin);
  int ndeny = dcs_list_get(DCS_LIST_DENY, deny);

  dcs_ring_reply_t rep[DCS_MACHN_MAX_REMOTES];
  for (int i = 0; i < DCS_MACHN_MAX_REMOTES; i++) {
    rep[i].id = (uint32_t)atomic_load(&g_dcs_machn_rep_id[i]);
    rep[i].last_msg = (uint8_t)atomic_load(&g_dcs_machn_rep_msg[i]);
    rep[i].last_ms = (uint64_t)atomic_load(&g_dcs_machn_rep_ms[i]);
  }

  dcs_ring_peer_t peers[RING_LEDS];
  int n = dcs_ring_machine_peers(
    allow, nallow, pin, npin, deny, ndeny, rep, DCS_MACHN_MAX_REMOTES, now, LINK_FRESH_MS, peers, RING_LEDS);
  for (int i = 0; i < n; i++) {
    seg[i] = dcs_ring_segment(peers[i].connected, peers[i].last_msg);
  }
  return n;
}
#else
/* Remote: one segment per CONFIGURED machine slot, in slot order, coloured by
 * that machine's last reply to this remote. */
static int ring_collect_segments(uint64_t now, dcs_ring_seg_t seg[RING_LEDS])
{
  int n = 0;
  for (int i = 0; i < DCS_PSTOP_MAX_MACHINES; i++) {
    bool cfg = false;
    dcs_get_pstop_peer_slot(i, &cfg, NULL, NULL, NULL);
    if (!cfg) {
      continue;
    }
    uint32_t st8 = (uint32_t)atomic_load(&g_dcs_pstop_m_state[i]);
    uint64_t last = (uint64_t)atomic_load(&g_dcs_pstop_m_last_reply_ms[i]);
    uint8_t lastmsg = (uint8_t)atomic_load(&g_dcs_pstop_m_last_msg[i]);
    bool connected = (st8 == 2u) && dcs_ring_fresh(last, now, LINK_FRESH_MS);
    seg[n] = dcs_ring_segment(connected, lastmsg);
    n++;
  }
  return n;
}
#endif

static void ring_task(void * arg)
{
  (void)arg;

  if (!ring_hw_init()) {
    vTaskDelete(NULL);
    return;
  }

  /* Stop the boot spinner and wait for it to exit before touching the
     * shared frame buffer / RMT channel. */
  atomic_store(&s_boot_anim_run, false);
  for (int i = 0; (i < 50) && atomic_load(&s_boot_anim_alive); i++) {
    DLY(10);
  }

  ESP_LOGI(TAG, "WS2812 PSTOP ring (%d LEDs) on GPIO%d", RING_LEDS, RING_GPIO);

  uint32_t prev_mismatch = atomic_load(&g_dcs_pstop_mismatch);
  uint32_t mm_streak = 0; /* consecutive frames the mismatch counter advanced */
  uint64_t mismatch_until = 0;
  int last_logged = -1;
  int ota_head = 0;
  bool ota_showing = false;
  bool locate_showing = false;

  for (;;) {
    /* OTA upload being flashed: reuse the boot comet so the operator sees
         * "updating" instead of a stale state colour. On success the device
         * reboots out of this; on failure the flag drops and the state
         * repaint resumes (last_logged reset so the transition is logged). */
    if (ml_config_ota_in_progress()) {
      if (!ota_showing) {
        ESP_LOGI(TAG, "ring -> OTA spinner");
        ota_showing = true;
        last_logged = -1;
      }
      ring_comet_frame(ota_head);
      ota_head = (ota_head + 1) % RING_LEDS;
      DLY(BOOT_SPIN_FRAME_MS);
      continue;
    }
    ota_showing = false;

    uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;

    /* Locate mode (installer aid): only logical LED 1 in white, rotated by
         * the offset in ring_show(), so the installer sees exactly which
         * physical pixel the current offset calls "LED 1". Overrides the state
         * colours while active; the deadline (set by dcs_pstop_ring_locate)
         * bounds how long the safety display can be masked. */
    if (now < (uint64_t)atomic_load(&s_locate_until_ms)) {
      if (!locate_showing) {
        ESP_LOGI(TAG, "ring -> LOCATE (LED 1 white, offset=%u)", (unsigned)atomic_load(&s_ring_offset));
        locate_showing = true;
        last_logged = -1;
      }
      (void)memset(s_grb, 0, sizeof(s_grb));
      s_grb[0] = RING_BRIGHTNESS; /* G */
      s_grb[1] = RING_BRIGHTNESS; /* R */
      s_grb[2] = RING_BRIGHTNESS; /* B — white */
      ring_show();
      DLY(RING_REFRESH_MS);
      continue;
    }
    if (locate_showing) {
      ESP_LOGI(TAG, "ring -> locate off");
      locate_showing = false;
    }

    /* PURPLE: only for a PERSISTENT core disagreement. A sustained fault
         * increments the mismatch counter every 100 ms comparator tick, so
         * every 250 ms ring frame sees a change; require the counter to have
         * advanced across MISMATCH_PERSIST_FRAMES consecutive frames (~750 ms
         * of continuous disagreement) before painting. Momentary blips (a
         * tick or two of disagreement) no longer flash the ring at all —
         * they remain visible in /state.json's pstop_mismatch counter. */
    uint32_t mm = atomic_load(&g_dcs_pstop_mismatch);
    if (mm != prev_mismatch) {
      prev_mismatch = mm;
      if (mm_streak < MISMATCH_PERSIST_FRAMES) {
        mm_streak++;
      }
      if (mm_streak >= MISMATCH_PERSIST_FRAMES) {
        mismatch_until = now + MISMATCH_HOLD_MS;
      }
    } else {
      mm_streak = 0;
    }
    bool recent_mismatch = now < mismatch_until;

    const uint8_t B = RING_BRIGHTNESS;
    uint32_t frame_ms = RING_REFRESH_MS;

    /* A recent core disagreement (e.g. one E-stop loop channel faulted) is a
         * DEVICE fault, not a link state — it silences every session — so it
         * paints the whole ring purple, overriding the per-machine segments. */
    if (recent_mismatch) {
      ring_fill(B, 0, B);
      if (last_logged != (int)RING_MISMATCH) {
        ESP_LOGI(TAG, "ring -> MISMATCH (mm=%lu)", (unsigned long)mm);
        last_logged = (int)RING_MISMATCH;
      }
      DLY(frame_ms);
      continue;
    }

    /* Link segments, one per peer, in display order (one peer = the whole
         * ring). Both roles show the MACHINE's state: the remote paints each
         * configured machine from that machine's last reply, the machine paints
         * each assigned remote from the last reply it sent (see
         * ring_collect_segments). Per segment:
         *   amber N-blink = no fresh reply (N = deepest broken network layer)
         *   blue          = last reply BOND/UNBOND
         *   green         = last reply OK
         *   red           = last reply STOP
         * No peers at all = whole ring white (IDLE). */
    dcs_ring_seg_t seg[RING_LEDS];
    int nseg = ring_collect_segments(now, seg);

    ring_state_t worst = RING_IDLE; /* for transition logging only */
    if (nseg == 0) {
      ring_fill(
        RING_WHITE_BRIGHTNESS, RING_WHITE_BRIGHTNESS, RING_WHITE_BRIGHTNESS); /* white (50%): nothing to connect to */
    } else {
      /* Deepest broken network layer -> amber blink count, shared by every
             * unreachable segment (device-level probes). 3 = no Internet,
             * 2 = no Tailscale, 1 = the peer is silent while the net is fine. */
      int conn_blinks = dcs_ring_blinks(
        dcs_net_inet_down(), (g_dcs.ml_handle != NULL) && (microlink_get_state(g_dcs.ml_handle) != ML_STATE_CONNECTED));
      uint8_t amber = amber_on(now, conn_blinks) ? B : 0u;

      (void)memset(s_grb, 0, sizeof(s_grb));
      for (int j = 0; j < nseg; j++) {
        uint8_t r = 0, g = 0, b = 0;
        bool wave = false; /* overlay the liveness comet on this segment */
        ring_state_t st;
        switch (seg[j]) {
          case DCS_RING_SEG_STOP:
            st = RING_STOP;
            r = B;
            wave = true;
            break;
          case DCS_RING_SEG_OK:
            st = RING_OK;
            g = B;
            wave = true;
            break;
          case DCS_RING_SEG_BOND:
            st = RING_BOND;
            b = B;
            break;
          case DCS_RING_SEG_UNREACHABLE:
          default:
            st = RING_UNREACHABLE;
            r = amber; /* amber = full red + low green (r>g): distinct from yellow (r==g) */
            g = (uint8_t)(((uint32_t)amber * AMBER_G_PCT) / 100u);
            frame_ms = PULSE_FRAME_MS;
            break;
        }
        if ((int)st > (int)worst) {
          worst = st;
        }

        int seg_start = (j * RING_LEDS) / nseg;
        int seg_end = ((j + 1) * RING_LEDS) / nseg;
        for (int p = seg_start; p < seg_end; p++) {
          uint8_t rr = r, gg = g, bb = b;
          if (wave) {
            /* Liveness comet: same shape as the boot spinner, overlaid as a
                         * subtle brightness wave on the segment's base colour so a
                         * steady OK/STOP visibly "breathes" instead of looking frozen.
                         * Head brightest with a fading tail; pixels far from the head
                         * sit slightly BELOW base so the wave reads as movement, not a
                         * static brightness change. Chases the whole ring so multiple
                         * segments share one coherent wave. */
            int d = ((s_wave_head - p) + RING_LEDS) % RING_LEDS;
            uint32_t scale = wave_scale_pct(d);
            rr = (uint8_t)(((uint32_t)r * scale) / 100u);
            gg = (uint8_t)(((uint32_t)g * scale) / 100u);
            bb = (uint8_t)(((uint32_t)b * scale) / 100u);
            frame_ms = WAVE_FRAME_MS;
          }
          s_grb[p * 3] = gg;
          s_grb[(p * 3) + 1] = rr;
          s_grb[(p * 3) + 2] = bb;
        }
      }
      s_wave_head = (s_wave_head + 1) % RING_LEDS;
      ring_show();
    }

    if ((int)worst != last_logged) {
      static const char * N[] = {"IDLE", "UNREACHABLE", "BOND/UNBOND", "OK", "STOP", "MISMATCH"};
      ESP_LOGI(TAG, "ring -> %s (peers=%d mm=%lu)", N[worst], nseg, (unsigned long)mm);
      last_logged = (int)worst;
    }

    DLY(frame_ms);
  }
}

void dcs_pstop_ring_bootsign(void)
{
  /* Called first thing in dcs_support_init(), long before the network (and
     * therefore the ring task) comes up: spin a DIM PURPLE comet around the
     * ring as a power-on sign of life. Without this the ring stays dark until
     * ml_app_start() returns — up to 30 s on a slow alt-network probe — which
     * reads as "dead board". The ring task stops the spinner and takes over
     * with the real link state (yellow = waiting for machine) as soon as it
     * starts. */
  if (!ring_hw_init()) {
    return;
  }

  atomic_store(&s_boot_anim_run, true);
  atomic_store(&s_boot_anim_alive, true);
  /* Internal-RAM stack, tiny: memset + RMT transmit only. Priority 2 (same
     * as the ring task — cosmetic, must never contend with safety tasks). */
  if (xTaskCreate(ring_bootsign_task, "ring_boot", 2048, NULL, 2, NULL) == pdPASS) {
    ESP_LOGI(TAG, "boot sign-of-life: dim purple spinner");
  } else {
    /* No task, no spinner — fall back to the static dim-purple fill. */
    atomic_store(&s_boot_anim_run, false);
    atomic_store(&s_boot_anim_alive, false);
    const uint8_t B = RING_BOOT_BRIGHTNESS;
    ring_fill(B, 0, B);
    ESP_LOGI(TAG, "boot sign-of-life: ring dim purple (static fallback)");
  }
}

void dcs_pstop_ring_start(void)
{
  /* Load the persisted rotation HERE (caller's internal stack, NVS already
     * up) — the ring task lives on a PSRAM stack and must not touch NVS. */
  atomic_store(&s_ring_offset, dcs_nvs_read_ring_offset());
  atomic_store(&s_ring_brightness_pct, dcs_nvs_read_led_brightness());
  /* PSRAM stack: LED ring is non-safety, does no flash/NVS. */
  (void)dcs_task_spawn_psram(ring_task, "pstop_ring", 4096, NULL, 2, tskNO_AFFINITY);
}

void dcs_pstop_ring_set_offset(uint8_t off)
{
  atomic_store(&s_ring_offset, (uint32_t)(off & 0x0Fu));
  /* Next repaint (<=250 ms) — or the next locate frame — picks it up. */
}

uint8_t dcs_pstop_ring_get_offset(void)
{
  return (uint8_t)atomic_load(&s_ring_offset);
}

void dcs_pstop_ring_set_brightness(uint8_t pct)
{
  /* Clamp BOTH bounds: DCS_LED_BRIGHTNESS_MIN keeps the safety colours
   * (RED STOP / GREEN OK / PURPLE MISMATCH) visible — 0%% would blank the
   * whole ring and persist across reboots with no visible way back. */
  uint32_t v = dcs_led_brightness_floor((pct > 100u) ? 100u : pct);
  atomic_store(&s_ring_brightness_pct, v);
  /* Next repaint (<=250 ms) — or the next locate/comet frame — picks it up. */
}

uint8_t dcs_pstop_ring_get_brightness(void)
{
  return (uint8_t)atomic_load(&s_ring_brightness_pct);
}

void dcs_pstop_ring_locate(bool on)
{
  if (on) {
    uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;
    atomic_store(&s_locate_until_ms, now + DCS_RING_LOCATE_TIMEOUT_MS);
  } else {
    atomic_store(&s_locate_until_ms, 0u);
  }
}

bool dcs_pstop_ring_locate_active(void)
{
  uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;
  return now < (uint64_t)atomic_load(&s_locate_until_ms);
}
