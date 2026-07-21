/**
 * @file dcs_safety.c
 * @brief 4-layer safety chain — boot-counter accounting, TWDT-fed heartbeat,
 * OTA mark-valid, and the Layer-4 age-out task. Verbatim from pre-refactor
 * main.c apart from being routed through the shared dcs state.
 */

#include "dcs_internal.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "dcs_safety";

/* RTC_NOINIT survives a reset (but is garbage on a cold power-on — guarded by
 * the 32-bit magic + the fact a POWERON is not a crash anyway). net_liveness
 * stamps this right before its deliberate wedge-recovery abort(); account_boot
 * consumes it so the recovery reboot is NOT counted toward the rollback ladder. */
#define DCS_LIVENESS_ABORT_MAGIC 0x4C495645u   /* "LIVE" */
RTC_NOINIT_ATTR static uint32_t s_liveness_abort_flag;

void dcs_safety_mark_liveness_abort(void) {
    s_liveness_abort_flag = DCS_LIVENESS_ABORT_MAGIC;
}

void dcs_safety_account_boot(void) {
    g_dcs.reset_reason = (uint8_t)esp_reset_reason();
    g_dcs.boot_count   = dcs_nvs_read_boot_count();
    dcs_nvs_push_reset_reason(g_dcs.reset_reason);   /* crash-pattern history */

    /* Was the last reboot a DELIBERATE net_liveness wedge-recovery abort? It
     * uses abort() (so the panic backtrace is captured) but a network/upstream
     * condition must NOT count toward firmware rollback. Consume the flag once. */
    bool was_liveness_abort = (s_liveness_abort_flag == DCS_LIVENESS_ABORT_MAGIC);
    s_liveness_abort_flag = 0;

    /* Only GENUINE CODE FAULTS count toward the rollback ladder — rollback means
     * "this image is bad," not "the environment is bad." Excluded: clean reboots
     * (POWERON/SW/EXT) and deep-sleep; BROWNOUT (power quality); and deliberate
     * net_liveness wedge-aborts (network/upstream). */
    bool crash_reset =
        ((g_dcs.reset_reason == (uint8_t)ESP_RST_PANIC) && !was_liveness_abort) ||
        (g_dcs.reset_reason == (uint8_t)ESP_RST_INT_WDT)  ||
        (g_dcs.reset_reason == (uint8_t)ESP_RST_TASK_WDT) ||
        (g_dcs.reset_reason == (uint8_t)ESP_RST_WDT);
    if (was_liveness_abort) {
        ESP_LOGW(TAG, "SAFETY: last reboot was a net_liveness wedge-recovery "
                 "(reset_reason=%u) — NOT counted toward rollback", g_dcs.reset_reason);
    }
    if (crash_reset) {
        g_dcs.boot_count++;
        if (dcs_nvs_write_boot_count(g_dcs.boot_count) != ESP_OK) {
            ESP_LOGW(TAG, "SAFETY: boot_count NVS write failed");
        }
        ESP_LOGW(TAG, "SAFETY: CRASH boot #%u (reset_reason=%u)",
                 g_dcs.boot_count, g_dcs.reset_reason);
    } else {
        ESP_LOGI(TAG, "SAFETY: clean boot (reset_reason=%u, crash_count=%u)",
                 g_dcs.reset_reason, g_dcs.boot_count);
    }

    if (g_dcs.boot_count <= (uint16_t)DCS_SAFETY_MAX_RAPID_BOOTS) return;

    /* Boot-count exceeded the ladder. v15.3.3 guard: NEVER mark the running
     * partition invalid if the OTHER partition is ALSO marked invalid — that
     * would leave the bootloader with no bootable image and brick the chip.
     * Just esp_restart() and let the bad firmware keep trying until a new
     * OTA replaces it. */
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    esp_ota_img_states_t other_state = ESP_OTA_IMG_UNDEFINED;
    bool other_invalid = false;
    if (next && esp_ota_get_state_partition(next, &other_state) == ESP_OK) {
        other_invalid = ((other_state == ESP_OTA_IMG_INVALID) ||
                         (other_state == ESP_OTA_IMG_ABORTED));
    }
    if (dcs_nvs_write_boot_count(0) != ESP_OK) {
        ESP_LOGW(TAG, "SAFETY: boot_count clear failed");
    }
    if (other_invalid) {
        ESP_LOGE(TAG,
                 "SAFETY: boot count %u > %u BUT other partition (%s) is also "
                 "marked invalid (state=%d). Refusing to brick — just rebooting; "
                 "safety-chain rollback intentionally skipped.",
                 g_dcs.boot_count, DCS_SAFETY_MAX_RAPID_BOOTS,
                 next ? next->label : "?", (int)other_state);
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
        /* unreachable */
    }
    ESP_LOGE(TAG, "SAFETY: boot count %u > %u — invoking rollback+reboot "
             "(other partition '%s' state=%d, safe to roll back)",
             g_dcs.boot_count, DCS_SAFETY_MAX_RAPID_BOOTS,
             next ? next->label : "?", (int)other_state);
    esp_err_t rb = esp_ota_mark_app_invalid_rollback_and_reboot();
    /* unreachable unless the rollback call itself failed */
    ESP_LOGE(TAG, "SAFETY: rollback+reboot returned: %s", esp_err_to_name(rb));
}

/* === TWDT-fed app heartbeat task ========================================== */

static void app_heartbeat_task(void *arg) {
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "SAFETY: heartbeat TWDT subscribe failed");
    }
    while (1) {
        if (esp_task_wdt_reset() != ESP_OK) {
            ESP_LOGW(TAG, "SAFETY: TWDT reset failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void dcs_safety_start_heartbeat(void) {
    if (xTaskCreate(app_heartbeat_task, "app_hb", 2048, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "SAFETY: app_hb task create failed");
    }
}

/* === OTA validate ========================================================= */

void dcs_safety_mark_ota_valid(void) {
    esp_err_t r = esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "SAFETY: OTA valid-mark @startup: %s", esp_err_to_name(r));
}

/* === Layer-4 age-out + v15.24 auto-recovery =============================== */

static void bc_clear_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(DCS_SAFETY_CLEAR_AFTER_MS));
    if (dcs_nvs_write_boot_count(0) != ESP_OK) {
        ESP_LOGW(TAG, "SAFETY: boot_count clear failed");
    }
    g_dcs.boot_count = 0;
    ESP_LOGI(TAG, "SAFETY: crash counter cleared after %lus healthy uptime",
             (unsigned long)(DCS_SAFETY_CLEAR_AFTER_MS / 1000));

    /* v15.24: if this boot was forced into derp-only mode by the safety
     * ladder, retry the fast direct-UDP path now that we've been healthy
     * for the age-out window. */
    if (g_dcs.derp_only_mode) {
        ESP_LOGW(TAG, "SAFETY: DERP-only mode survived %lus healthy — "
                 "auto-restart to retry direct UDP",
                 (unsigned long)(DCS_SAFETY_CLEAR_AFTER_MS / 1000));
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
    vTaskDelete(NULL);
}

void dcs_safety_start_bc_clear(void) {
    /* v15.25 stack: 4 KiB — bc_clear hits NVS write + ESP_LOG + esp_restart.
     * 2 KiB tripped the canary in pre-refactor testing. */
    if (xTaskCreate(bc_clear_task, "bc_clear", 4096, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "SAFETY: bc_clear task create failed");
    }
}
