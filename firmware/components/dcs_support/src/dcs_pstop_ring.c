// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_pstop_ring.c
 * @brief 16-LED WS2812 ring on GPIO17 — shows the PSTOP link state as seen from
 *        the MACHINE's replies (NOT the network state; the onboard single LED
 *        on GPIO21, dcs_rgb.c, does network).
 *
 * The ring is divided evenly among the CONFIGURED machine slots (one machine
 * = the whole ring, two = halves, etc., in slot order starting at LED 1) and
 * each segment shows ITS machine's link state from the per-machine telemetry
 * the comparator publishes (g_dcs_pstop_m_state/_last_msg/_last_reply_ms).
 * Device-level conditions (lockstep MISMATCH, OTA, locate, no machines at
 * all) override the whole ring. Per segment / ring:
 *
 *     WHITE  = IDLE          no pstop peer (machine) configured — the remote has
 *                            nothing to connect to (fresh unit, or peer cleared).
 *     RED    = UNREACHABLE   a peer IS configured but no fresh reply — SLOW RED
 *              (pulsing)      PULSE (never bonded / machine down / went away /
 *                            comparator stopped after a lockstep fault). Distinct
 *                            from a solid-red commanded STOP.
 *     BLUE   = BOND/UNBOND   connected; the machine's last reply was BOND or
 *                            UNBOND — just (re)bonded, not yet armed to OK.
 *     GREEN  = OK            connected; the machine's last reply was OK — the
 *                            robot is cleared to run.
 *     RED    = STOP          connected; the machine's last reply was STOP — a
 *                            commanded stop (E-stop pressed, or the NEED_STOP
 *                            arming cycle not yet completed).
 *     PURPLE = MISMATCH      the two lockstep cores disagreed recently (their
 *                            encoded messages differed) — e.g. ONE E-stop loop
 *                            channel opened/faulted while the other stayed
 *                            closed. The comparator sends nothing on a mismatch,
 *                            so the link would otherwise just time out to
 *                            yellow; purple flags the real cause. Held briefly
 *                            after each mismatch so single blips show. Takes
 *                            priority over every other colour.
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
 * offset can be dialled in from the fleet-setup tooling.
 */

#include <stdatomic.h>
#include <string.h>

#include "dcs_internal.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ml_config_httpd.h" /* ml_config_ota_in_progress() */
#include "pstop/pstop_msg.h" /* PSTOP_MESSAGE_OK / _STOP / _BOND / _UNBOND */

static const char * TAG = "dcs_ring";

#define RING_GPIO 17
#define RING_LEDS 16
#define RING_RES_HZ 10000000 /* 0.1 us per RMT tick (same as dcs_rgb) */
#define RING_BRIGHTNESS 40 /* per channel; 16 LEDs — visible, low current/glare */

#define RING_REFRESH_MS 250 /* re-evaluate state + repaint; also the WS2812 reset gap */
#define LINK_FRESH_MS 2000u /* no machine reply within this -> not connected (re-bond is 1.5s) */
#define MISMATCH_HOLD_MS 2000u /* show PURPLE this long after a lockstep mismatch increments */
#define PULSE_PERIOD_MS 2400u /* slow RED pulse period when a configured host is unreachable */
#define PULSE_FRAME_MS 60 /* repaint step while pulsing, for a smooth ramp */

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
  for (int i = 0; i < RING_LEDS; i++) {
    int p = (int)(((uint32_t)i + off) % RING_LEDS);
    (void)memcpy(&tx_grb[p * 3], &s_grb[i * 3], 3);
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
  static const uint32_t k_scale[4] = {160u, 130u, 110u, 95u}; /* head, then tail */
  return (dist < 4) ? k_scale[dist] : 85u;
}

typedef enum
{
  RING_IDLE, /* no pstop peer configured — nothing to connect to (white) */
  RING_UNREACHABLE, /* peer configured but no fresh reply (slow red pulse)       */
  RING_BOND,
  RING_OK,
  RING_STOP,
  RING_MISMATCH
} ring_state_t;

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

    /* PURPLE hold: latch on any new lockstep mismatch so even a single
         * transient blip is visible; a sustained single-channel fault keeps
         * re-extending it (the cores disagree every tick). */
    uint32_t mm = atomic_load(&g_dcs_pstop_mismatch);
    if (mm != prev_mismatch) {
      prev_mismatch = mm;
      mismatch_until = now + MISMATCH_HOLD_MS;
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

    /* Per-machine segments: the ring is divided evenly among the CONFIGURED
         * peer slots, in slot order (one machine = the whole ring, matching the
         * old single-machine display). Per segment, the classic colours apply:
         *   red pulse = configured but no fresh reply (bonding / unreachable)
         *   blue      = bonded, machine's last reply was BOND/UNBOND
         *   green     = machine's last reply was OK
         *   red       = machine's last reply was STOP
         * No machines configured at all = whole ring white (IDLE). */
    int cfg_slots[DCS_PSTOP_MAX_MACHINES];
    int ncfg = 0;
    for (int i = 0; i < DCS_PSTOP_MAX_MACHINES; i++) {
      bool cfg = false;
      dcs_get_pstop_peer_slot(i, &cfg, NULL, NULL, NULL);
      if (cfg) {
        cfg_slots[ncfg] = i;
        ncfg++;
      }
    }

    ring_state_t worst = RING_IDLE; /* for transition logging only */
    if (ncfg == 0) {
      ring_fill(B, B, B); /* white: nothing to connect to */
    } else {
      /* Pulse level shared by every unreachable segment: triangle ramp
             * 0..B..0 over PULSE_PERIOD_MS; repaint fast so it is smooth. */
      uint32_t ph = (uint32_t)(now % PULSE_PERIOD_MS);
      uint32_t half = PULSE_PERIOD_MS / 2u;
      uint32_t pulse = (ph < half) ? (ph * B / half) : ((PULSE_PERIOD_MS - ph) * B / half);

      (void)memset(s_grb, 0, sizeof(s_grb));
      for (int j = 0; j < ncfg; j++) {
        int slot = cfg_slots[j];
        uint32_t st8 = (uint32_t)atomic_load(&g_dcs_pstop_m_state[slot]);
        uint64_t last = (uint64_t)atomic_load(&g_dcs_pstop_m_last_reply_ms[slot]);
        uint8_t lastmsg = (uint8_t)atomic_load(&g_dcs_pstop_m_last_msg[slot]);
        bool connected = (st8 == 2u) && (last > 0u) && (now >= last) && ((now - last) < LINK_FRESH_MS);

        uint8_t r = 0, g = 0, b = 0;
        bool wave = false; /* overlay the liveness comet on this segment */
        ring_state_t st;
        if (!connected) {
          st = RING_UNREACHABLE;
          r = (uint8_t)pulse;
          frame_ms = PULSE_FRAME_MS;
        } else if (lastmsg == PSTOP_MESSAGE_STOP) {
          st = RING_STOP;
          r = B;
          wave = true;
        } else if (lastmsg == PSTOP_MESSAGE_OK) {
          st = RING_OK;
          g = B;
          wave = true;
        } else {
          st = RING_BOND;
          b = B;
        }
        if ((int)st > (int)worst) {
          worst = st;
        }

        int seg_start = (j * RING_LEDS) / ncfg;
        int seg_end = ((j + 1) * RING_LEDS) / ncfg;
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
      ESP_LOGI(TAG, "ring -> %s (machines=%d mm=%lu)", N[worst], ncfg, (unsigned long)mm);
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
