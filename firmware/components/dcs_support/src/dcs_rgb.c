/**
 * @file dcs_rgb.c
 * @brief Onboard WS2812 RGB status LED — blinks the local IPv4 last octet in
 *        a colour that encodes the active network interface.
 *
 * Waveshare ESP32-S3-ETH has a single WS2812B addressable LED whose data line
 * (RGB_DIN) is jumpered to GPIO21 (schematic: WS_L1, R15=0R -> GP21). The old
 * ml_ip_blink toggled GPIO21 as a plain on/off pin, which cannot clock a
 * WS2812 — so this module drives it properly over the RMT peripheral.
 *
 * Colour = active interface (read live from g_dcs_active_iface, which the
 * net supervisor refreshes every second, so the colour tracks runtime mode
 * switches, not just the boot-time choice):
 *
 *     blue   = Ethernet (W5500)
 *     green  = USB-Ethernet (CDC-NCM)
 *     yellow = WiFi STA
 *     red    = WiFi SoftAP (local setup mode)
 *     off    = no usable interface yet
 *
 * Blink pattern = last octet of that interface's IPv4, digit by digit
 * (e.g. .109 -> 1 blink, gap, "0" long blink, gap, 9 blinks). Same scheme as
 * the previous ml_ip_blink, just in colour.
 *
 * EXCEPTION — internet-fault strobe: when a local uplink is up but the public
 * internet is unreachable (dcs_net_inet_down(): neither 1.1.1.1 nor 8.8.8.8
 * answering), the per-octet blink is replaced by a rapid RED strobe. Tailscale
 * lives on the internet, so this is the "link green but offline to peers"
 * alarm. SoftAP/none states keep their own colour (no uplink to probe through).
 */

#include "dcs_internal.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "dcs_rgb";

#define RGB_GPIO         21
#define RGB_RES_HZ       10000000   /* 0.1 us per RMT tick */
#define RGB_BRIGHTNESS   48         /* 0..255 per channel — comfortable, not blinding */

#define BLINK_ON_MS      180
#define BLINK_OFF_MS     220
#define BLINK_ZERO_MS    550        /* long blink represents digit 0 */
#define BLINK_DIGIT_GAP  550
#define BLINK_CYCLE_GAP  1500        /* shorter dark gap so a sparse octet
                                      * (e.g. ".101") still reads as "alive"
                                      * rather than looking frozen */
#define BLINK_WAIT_MS    1000        /* poll interval while no interface is up */

#define DLY(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_enc  = NULL;

/* Push one WS2812 pixel (GRB order). */
static void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    if ((s_chan == NULL) || (s_enc == NULL)) {
        return;
    }
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &tx) == ESP_OK) {
        (void)rmt_tx_wait_all_done(s_chan, 100);
    }
}

/* Colour (already brightness-scaled) for the active interface. */
static void colour_for(dcs_iface_t a, uint8_t *r, uint8_t *g, uint8_t *b) {
    const uint8_t v = RGB_BRIGHTNESS;
    switch (a) {
    case DCS_IFACE_ETH:  *r = 0; *g = 0; *b = v; break;   /* blue   */
    case DCS_IFACE_USB:  *r = 0; *g = v; *b = 0; break;   /* green  */
    case DCS_IFACE_WIFI: *r = v; *g = v; *b = 0; break;   /* yellow */
    case DCS_IFACE_AP:   *r = v; *g = 0; *b = 0; break;   /* red (setup) */
    default:             *r = 0; *g = 0; *b = 0; break;   /* off    */
    }
}

/* Last dotted octet of a netif's IPv4 (0 if none). esp_ip4_addr_t.addr is in
 * network byte order, so on the little-endian S3 the last octet is the high
 * byte of the integer (e.g. 10.42.0.109 -> 0x6D002A0A -> 0x6D = 109). */
static uint8_t octet_of(const char *ifkey) {
    if (ifkey == NULL) {
        return 0u;
    }
    esp_netif_t *n = esp_netif_get_handle_from_ifkey(ifkey);
    if (n == NULL) {
        return 0u;
    }
    esp_netif_ip_info_t ip;
    if ((esp_netif_get_ip_info(n, &ip) != ESP_OK) || (ip.ip.addr == 0u)) {
        return 0u;
    }
    return (uint8_t)((ip.ip.addr >> 24) & 0xFFu);
}

static void blink_digit(int count, uint8_t r, uint8_t g, uint8_t b) {
    if (count == 0) {                 /* zero = one long blink */
        rgb_set(r, g, b);
        DLY(BLINK_ZERO_MS);
        rgb_set(0, 0, 0);
        return;
    }
    for (int i = 0; i < count; i++) {
        rgb_set(r, g, b);
        DLY(BLINK_ON_MS);
        rgb_set(0, 0, 0);
        if (i < (count - 1)) {
            DLY(BLINK_OFF_MS);
        }
    }
}

static void rgb_task(void *arg) {
    (void)arg;

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = RGB_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RGB_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    if (rmt_new_tx_channel(&chan_cfg, &s_chan) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed — RGB LED disabled");
        vTaskDelete(NULL);
        return;
    }
    /* WS2812 bit timing at 0.1 us/tick: 0-bit = 0.3us high / 0.9us low,
     * 1-bit = 0.9us high / 0.3us low. MSB first. */
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags = { .msb_first = 1 },
    };
    if ((rmt_new_bytes_encoder(&enc_cfg, &s_enc) != ESP_OK) ||
        (rmt_enable(s_chan) != ESP_OK)) {
        ESP_LOGE(TAG, "rmt encoder/enable failed — RGB LED disabled");
        vTaskDelete(NULL);
        return;
    }
    rgb_set(0, 0, 0);
    ESP_LOGI(TAG, "WS2812 RGB status LED on GPIO%d", RGB_GPIO);

    int last_logged = -1;
    for (;;) {
        (void)atomic_fetch_add(&g_dcs_rgb_cycles, 1);   /* liveness heartbeat */
        dcs_iface_t a = (dcs_iface_t)atomic_load(&g_dcs_active_iface);
        uint8_t r, g, b;
        colour_for(a, &r, &g, &b);
        uint8_t octet = octet_of(dcs_iface_ifkey(a));

        if ((int)a != last_logged) {
            ESP_LOGI(TAG, "iface=%d octet=%u colour=(%u,%u,%u)",
                     (int)a, octet, r, g, b);
            last_logged = (int)a;
        }

        /* Internet unreachable while a local uplink IS up: override the normal
         * per-octet blink with a rapid RED alarm strobe. Tailscale's DERP relays
         * and WireGuard peers all live on the public internet, so "link up but no
         * internet" means this unit is offline to its pstop peers even though a
         * local light is green — the operator needs to see that immediately.
         * Gated on a real uplink: SoftAP/none keep their own colour, since
         * there's no upstream to even attempt the internet through. */
        if (((a == DCS_IFACE_ETH) || (a == DCS_IFACE_USB) || (a == DCS_IFACE_WIFI)) &&
            dcs_net_inet_down()) {
            for (int i = 0; i < 6; i++) {
                rgb_set(RGB_BRIGHTNESS, 0, 0);   /* red */
                DLY(120);
                rgb_set(0, 0, 0);
                DLY(120);
            }
            DLY(500);
            continue;
        }

        /* No usable interface — LED dark, poll again shortly. */
        if ((a == DCS_IFACE_NONE) || ((r == 0u) && (g == 0u) && (b == 0u))) {
            rgb_set(0, 0, 0);
            DLY(BLINK_WAIT_MS);
            continue;
        }

        /* Interface up but no readable address (rare) — steady colour pulse
         * so the mode is still indicated. */
        if (octet == 0u) {
            rgb_set(r, g, b);
            DLY(BLINK_ZERO_MS);
            rgb_set(0, 0, 0);
            DLY(BLINK_CYCLE_GAP);
            continue;
        }

        int oct = (int)octet;
        int digits[3] = { oct / 100, (oct / 10) % 10, oct % 10 };
        int start = 0;
        if (digits[0] == 0) { start = 1; }
        if ((digits[0] == 0) && (digits[1] == 0)) { start = 2; }

        for (int d = start; d < 3; d++) {
            if (d > start) {
                DLY(BLINK_DIGIT_GAP);
            }
            blink_digit(digits[d], r, g, b);
        }

        rgb_set(0, 0, 0);
        DLY(BLINK_CYCLE_GAP);   /* dark gap; iface + IP re-read next cycle */
    }
}

void dcs_rgb_start(void) {
    /* PSRAM stack: status LED is non-safety, does no flash/NVS. */
    (void)dcs_task_spawn_psram(rgb_task, "rgb_blink", 4096, NULL, 2, tskNO_AFFINITY);
}
