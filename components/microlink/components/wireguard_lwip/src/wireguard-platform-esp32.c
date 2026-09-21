/**
 * @file wireguard-platform-esp32.c
 * @brief ESP32 platform implementation for wireguard-lwip
 */

#include "wireguard-platform.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "lwip/sys.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Time Functions
 * ========================================================================== */

uint32_t wireguard_sys_now() {
    // Use lwIP's built-in time function
    return sys_now();
}

// Handshake timestamps must increase across REBOOTS, not just within one: the
// peer keeps the greatest TAI64N it accepted from our static key and silently
// drops any initiation that is not newer (wireguard-go compares the 12 bytes
// as opaque big-endian values; it never checks them against wall time). With
// uptime alone, a rebooted device could not re-initiate until the host forgot
// the peer (#157: two 9-min tether outages). No RTC/SNTP here, so a per-boot
// epoch from NVS, incremented once per boot, provides the monotonic high bits:
// seconds = TAI(0) + (epoch << 32) + uptime_s.
static uint64_t s_boot_epoch_s; // (epoch << 32); 0 = not initialised (uptime-only, pre-#157 behaviour)
static bool s_epoch_failed;     // init attempted and NVS refused: degraded (reported as UINT32_MAX)

// Call once at startup, before the WireGuard interface comes up: one NVS
// read-increment-commit, kept off the handshake path. Returns false (and logs)
// if the epoch could not be persisted: handshakes then fall back to uptime-only
// timestamps rather than taking the tunnel down.
bool wireguard_tai64n_epoch_init(void) {
    static bool done; // once per boot even if the WG interface init is retried
    if (done) return s_boot_epoch_s != 0;
    done = true;
    nvs_handle_t h;
    uint32_t epoch = 0;
    esp_err_t err = nvs_open("ml_wg", NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, "epoch", &epoch);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK; // first boot: start at 1
        // any other read error must NOT restart the counter at 1 (that would roll the epoch back)
        if (err == ESP_OK) {
            epoch++;
            err = nvs_set_u32(h, "epoch", epoch);
            if (err == ESP_OK) err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        s_epoch_failed = true;
        printf("[WG] ERROR: handshake epoch not persisted (%d) - post-reboot initiations may be rejected as replays\n", (int)err);
        return false;
    }
    s_boot_epoch_s = (uint64_t)epoch << 32;
    return true;
}

// 0 = not initialised yet (WG interface not up); UINT32_MAX = init failed (degraded: device-initiated
// handshakes may be rejected as replays); otherwise the persisted per-boot epoch.
uint32_t wireguard_tai64n_epoch(void) {
    if (s_epoch_failed) return 0xFFFFFFFFu;
    return (uint32_t)(s_boot_epoch_s >> 32);
}

void wireguard_tai64n_now(uint8_t *output) {
    // TAI64N format: 8 bytes seconds + 4 bytes nanoseconds
    uint64_t now_us = esp_timer_get_time();
    uint64_t uptime_s = now_us / 1000000ULL;
    uint64_t seconds = uptime_s + s_boot_epoch_s;
    uint32_t nanoseconds = (now_us % 1000000ULL) * 1000;

    // Log uptime and the boot epoch separately (only every ~5s to avoid spam)
    static uint64_t last_log_s = 0;
    if (uptime_s - last_log_s >= 5) {
        extern volatile int wg_verbose_logging;
        if (wg_verbose_logging) printf("[TAI64N] uptime=%llu s, epoch=%lu, nano=%lu\n", (unsigned long long)uptime_s, (unsigned long)(s_boot_epoch_s >> 32), (unsigned long)nanoseconds);
        last_log_s = uptime_s;
    }

    // TAI64 starts at 1970-01-01 00:00:10 TAI (Unix epoch + 10 seconds)
    // Add TAI offset: 2^62 + Unix time
    seconds += 0x400000000000000AULL;

    // Write in big-endian format
    output[0] = (seconds >> 56) & 0xFF;
    output[1] = (seconds >> 48) & 0xFF;
    output[2] = (seconds >> 40) & 0xFF;
    output[3] = (seconds >> 32) & 0xFF;
    output[4] = (seconds >> 24) & 0xFF;
    output[5] = (seconds >> 16) & 0xFF;
    output[6] = (seconds >> 8) & 0xFF;
    output[7] = seconds & 0xFF;

    output[8] = (nanoseconds >> 24) & 0xFF;
    output[9] = (nanoseconds >> 16) & 0xFF;
    output[10] = (nanoseconds >> 8) & 0xFF;
    output[11] = nanoseconds & 0xFF;
}

/* ============================================================================
 * Random Number Generation
 * ========================================================================== */

void wireguard_random_bytes(void *bytes, size_t size) {
    // Use ESP32 hardware RNG
    esp_fill_random(bytes, size);
}

/* ============================================================================
 * Load Management
 * ========================================================================== */

bool wireguard_is_under_load() {
    // For now, always return false (not under load)
    // Could be enhanced to check:
    // - Free heap memory
    // - CPU usage
    // - Number of active connections
    return false;
}
