/**
 * @file dcs_pstop_ring.c
 * @brief 16-LED WS2812 ring on GPIO17 — shows the PSTOP link state as seen from
 *        the MACHINE's replies (NOT the network state; the onboard single LED
 *        on GPIO21, dcs_rgb.c, does network).
 *
 * The whole ring is filled one solid colour, driven by the machine's most
 * recent reply — the comparator publishes the last received message type
 * (g_dcs_pstop_last_msg) and timestamp (g_dcs_pstop_last_reply_ms) in main.c:
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
 */

#include "dcs_internal.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pstop/pstop_msg.h"   /* PSTOP_MESSAGE_OK / _STOP / _BOND / _UNBOND */
#include "ml_config_httpd.h"   /* ml_config_ota_in_progress() */

static const char *TAG = "dcs_ring";

#define RING_GPIO          17
#define RING_LEDS          16
#define RING_RES_HZ        10000000   /* 0.1 us per RMT tick (same as dcs_rgb) */
#define RING_BRIGHTNESS    40         /* per channel; 16 LEDs — visible, low current/glare */

#define RING_REFRESH_MS    250        /* re-evaluate state + repaint; also the WS2812 reset gap */
#define LINK_FRESH_MS      2000u      /* no machine reply within this -> not connected (re-bond is 1.5s) */
#define MISMATCH_HOLD_MS   2000u      /* show PURPLE this long after a lockstep mismatch increments */
#define PULSE_PERIOD_MS    2400u      /* slow RED pulse period when a configured host is unreachable */
#define PULSE_FRAME_MS     60         /* repaint step while pulsing, for a smooth ramp */

#define RING_BOOT_BRIGHTNESS 10       /* dim power-on sign-of-life; well below the state colours (40)
                                       * so it can't be mistaken for a full-brightness MISMATCH purple */
#define BOOT_SPIN_FRAME_MS   70       /* comet step period: ~1.1 s per revolution of the 16 LEDs */

#define DLY(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_enc  = NULL;
static uint8_t s_grb[RING_LEDS * 3];   /* WS2812 wants GRB order, per pixel */

/* Stream the current s_grb frame. */
static void ring_show(void) {
    if ((s_chan == NULL) || (s_enc == NULL)) {
        return;
    }
    rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_chan, s_enc, s_grb, sizeof(s_grb), &tx) == ESP_OK) {
        (void)rmt_tx_wait_all_done(s_chan, 100);
    }
}

/* Fill all 16 pixels one colour and stream the frame. */
static void ring_fill(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < RING_LEDS; i++) {
        s_grb[i * 3]       = g;
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
static void ring_comet_frame(int head) {
    static const uint8_t comet[4] = { RING_BOOT_BRIGHTNESS, 5, 2, 1 };
    (void)memset(s_grb, 0, sizeof(s_grb));
    for (int t = 0; t < 4; t++) {
        int i = ((head - t) + RING_LEDS) % RING_LEDS;
        s_grb[(i * 3) + 1] = comet[t];   /* R */
        s_grb[(i * 3) + 2] = comet[t];   /* B — R+B = purple */
    }
    ring_show();
}

static void ring_bootsign_task(void *arg) {
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
static bool ring_hw_init(void) {
    static enum { HW_UNTRIED, HW_OK, HW_FAILED } s_hw = HW_UNTRIED;
    if (s_hw != HW_UNTRIED) {
        return (s_hw == HW_OK);
    }
    s_hw = HW_FAILED;

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = RING_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RING_RES_HZ,
        .mem_block_symbols  = 64,    /* bytes-encoder streams the 384-bit frame */
        .trans_queue_depth = 4,
    };
    if (rmt_new_tx_channel(&chan_cfg, &s_chan) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel(GPIO%d) failed — PSTOP ring disabled", RING_GPIO);
        s_chan = NULL;
        return false;
    }
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags = { .msb_first = 1 },
    };
    if ((rmt_new_bytes_encoder(&enc_cfg, &s_enc) != ESP_OK) ||
        (rmt_enable(s_chan) != ESP_OK)) {
        ESP_LOGE(TAG, "rmt encoder/enable failed — PSTOP ring disabled");
        s_enc = NULL;
        return false;
    }
    s_hw = HW_OK;
    return true;
}

typedef enum {
    RING_IDLE,         /* no pstop peer configured — nothing to connect to (white) */
    RING_UNREACHABLE,  /* peer configured but no fresh reply (slow red pulse)       */
    RING_BOND, RING_OK, RING_STOP, RING_MISMATCH
} ring_state_t;

static void ring_task(void *arg) {
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

    uint32_t prev_mismatch  = atomic_load(&g_dcs_pstop_mismatch);
    uint64_t mismatch_until = 0;
    int  last_logged = -1;
    int  ota_head    = 0;
    bool ota_showing = false;

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

        /* PURPLE hold: latch on any new lockstep mismatch so even a single
         * transient blip is visible; a sustained single-channel fault keeps
         * re-extending it (the cores disagree every tick). */
        uint32_t mm = atomic_load(&g_dcs_pstop_mismatch);
        if (mm != prev_mismatch) { prev_mismatch = mm; mismatch_until = now + MISMATCH_HOLD_MS; }
        bool recent_mismatch = now < mismatch_until;

        /* "Connected" = a fresh reply from the machine within LINK_FRESH_MS.
         * Otherwise disconnected (never bonded, the machine went away, or the
         * comparator stopped sending after a lockstep fault — which the machine
         * then heartbeat-times-out, so replies stop). */
        uint32_t replies = atomic_load(&g_dcs_pstop_replies);
        uint64_t last    = atomic_load(&g_dcs_pstop_last_reply_ms);
        uint8_t  lastmsg = (uint8_t)atomic_load(&g_dcs_pstop_last_msg);
        uint32_t peer_ip = atomic_load(&g_dcs_pstop_peer_ip);
        bool connected   = (replies > 0u) && (last > 0u) && (now >= last) &&
                           ((now - last) < LINK_FRESH_MS);
        bool host_set    = (peer_ip != 0u);   /* a pstop peer (machine) target is configured */

        /* A recent core disagreement (e.g. one E-stop loop channel faulted)
         * takes priority. When there's no fresh reply, distinguish IDLE (no
         * peer configured -> nothing to connect to -> white) from UNREACHABLE
         * (peer configured but not answering -> slow red pulse). Otherwise the
         * colour reflects the machine's most recent reply. */
        ring_state_t st;
        if      (recent_mismatch)               { st = RING_MISMATCH; }     /* purple    */
        else if (!connected && !host_set)       { st = RING_IDLE; }         /* white     */
        else if (!connected)                    { st = RING_UNREACHABLE; }  /* red pulse */
        else if (lastmsg == PSTOP_MESSAGE_STOP) { st = RING_STOP; }         /* red       */
        else if (lastmsg == PSTOP_MESSAGE_OK)   { st = RING_OK; }           /* green     */
        else                                    { st = RING_BOND; }         /* blue: BOND/UNBOND */

        const uint8_t B = RING_BRIGHTNESS;
        uint32_t frame_ms = RING_REFRESH_MS;
        switch (st) {
        case RING_IDLE:         ring_fill(B, B, B); break;   /* white  */
        case RING_UNREACHABLE: {                             /* slow red pulse */
            /* Triangle ramp 0..B..0 over PULSE_PERIOD_MS; repaint fast (frame_ms)
             * so the pulse is smooth rather than stepped. */
            uint32_t ph   = (uint32_t)(now % PULSE_PERIOD_MS);
            uint32_t half  = PULSE_PERIOD_MS / 2u;
            uint32_t level = (ph < half) ? (ph * B / half)
                                         : ((PULSE_PERIOD_MS - ph) * B / half);
            ring_fill((uint8_t)level, 0, 0);
            frame_ms = PULSE_FRAME_MS;
            break;
        }
        case RING_BOND:         ring_fill(0, 0, B); break;   /* blue   */
        case RING_OK:           ring_fill(0, B, 0); break;   /* green  */
        case RING_STOP:         ring_fill(B, 0, 0); break;   /* red    */
        case RING_MISMATCH:     ring_fill(B, 0, B); break;   /* purple */
        default:                                    break;   /* unreachable */
        }

        if ((int)st != last_logged) {
            static const char *N[] = { "IDLE", "UNREACHABLE", "BOND/UNBOND", "OK", "STOP", "MISMATCH" };
            ESP_LOGI(TAG, "ring -> %s (machine_msg=%u replies=%lu mm=%lu host_set=%d)",
                     N[st], lastmsg, (unsigned long)replies, (unsigned long)mm, (int)host_set);
            last_logged = (int)st;
        }

        DLY(frame_ms);
    }
}

void dcs_pstop_ring_bootsign(void) {
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

void dcs_pstop_ring_start(void) {
    /* PSRAM stack: LED ring is non-safety, does no flash/NVS. */
    (void)dcs_task_spawn_psram(ring_task, "pstop_ring", 4096, NULL, 2, tskNO_AFFINITY);
}
