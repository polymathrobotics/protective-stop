// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_health.c
 * @brief Lifetime wear counters (NVS-backed) + the health-warning board.
 *
 * Data flow:
 *   core0/core1 -> dcs_publish_core_tick() -> dcs_health_note_core_tick()
 *     -> edge detector (dcs_health_logic.c) -> atomic counters, dirty flag
 *   health task (1 Hz, INTERNAL stack because it writes NVS):
 *     -> flush when due (dirty >= FLUSH_MIN_S, or every FLUSH_MAX_S)
 *     -> built-in publishers every 60 s (button wear, loop mismatch rate)
 *   shutdown handler -> best-effort flush on controlled restart
 *   /api/health, /state.json -> dcs_health_get_snapshot(), dcs_health_overall()
 *
 * The edge hook runs on the safety cores' tick, so it is a short critical
 * section and nothing else: no logging, no allocation, no NVS.
 */

#include "dcs_health.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "dcs_internal.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pstop/pstop_msg.h" /* PSTOP_MESSAGE_STOP */

static const char * TAG = "dcs_health";

/* --- Counters ------------------------------------------------------------ */

/* Values loaded from NVS at boot; uptime_s here is the base, live uptime is
 * base + this boot's esp_timer. Written only by init/reset under s_lock. */
static dcs_health_counters_t s_base;

/* Deltas accumulated this boot by the edge hook (safety-core context). */
static atomic_uint_fast32_t s_presses_delta;
static atomic_uint_fast32_t s_mismatch_delta;
static atomic_bool s_dirty;

#ifndef DCS_PAGE_MACHINE
/* Edge detector state + the latest tick/verdict per core. Guarded by a
 * spinlock because both cores publish concurrently. Remote only: on machn the
 * per-core "verdict" is robot state, not a button. */
static portMUX_TYPE s_edge_mux = portMUX_INITIALIZER_UNLOCKED;
static dcs_health_edge_t s_edge;
#endif

/* Flush bookkeeping (health task + shutdown hook + admin reset). */
static SemaphoreHandle_t s_lock; /* serializes NVS writes + s_base edits */
static atomic_uint_fast32_t s_flushes;
static atomic_uint_fast32_t s_flush_fails;
static atomic_uint_fast64_t s_last_flush_ms;
static uint32_t s_uptime_in_base_s; /* seconds of THIS boot already folded into s_base.uptime_s */
static bool s_inited;

/* --- Warning board -------------------------------------------------------- */

static dcs_health_warning_t s_warn[DCS_HEALTH_MAX_WARNINGS];
static int s_warn_n;
static atomic_uint_fast32_t s_dropped;
static atomic_int s_overall; /* dcs_health_level_t, cheap read for /state.json */

static uint64_t now_ms(void)
{
  return (uint64_t)esp_timer_get_time() / 1000ULL;
}

static uint32_t this_boot_uptime_s(void)
{
  return (uint32_t)((uint64_t)esp_timer_get_time() / 1000000ULL);
}

/* Live counters = base + this boot's deltas. Caller holds s_lock or accepts a
 * momentarily inconsistent (but monotone) read. */
static void snapshot_counters(dcs_health_counters_t * out)
{
  *out = s_base;
  out->presses += (uint32_t)atomic_load(&s_presses_delta);
  out->mismatch_events += (uint32_t)atomic_load(&s_mismatch_delta);
  out->uptime_s += this_boot_uptime_s() - s_uptime_in_base_s;
}

static void recompute_overall(void)
{
  int worst = (int)DCS_HEALTH_OK;
  for (int i = 0; i < s_warn_n; i++) {
    if ((int)s_warn[i].level > worst) {
      worst = (int)s_warn[i].level;
    }
  }
  atomic_store(&s_overall, worst);
}

/* --- Persistence ---------------------------------------------------------- */

/* Fold deltas into s_base, write the blob. Holds s_lock. Deltas are moved,
 * not zeroed, so a failed write keeps them for the next attempt. */
static esp_err_t flush_locked(void)
{
  dcs_health_counters_t c = s_base;
  uint32_t p = (uint32_t)atomic_exchange(&s_presses_delta, 0);
  uint32_t m = (uint32_t)atomic_exchange(&s_mismatch_delta, 0);
  c.presses += p;
  c.mismatch_events += m;
  /* Uptime: fold in only the part of this boot not yet in the base. */
  uint32_t up_now = this_boot_uptime_s();
  c.uptime_s += (up_now - s_uptime_in_base_s);

  atomic_store(&s_dirty, false);
  esp_err_t r = dcs_nvs_write_health(&c);
  if (r == ESP_OK) {
    s_base = c;
    s_uptime_in_base_s = up_now;
    atomic_fetch_add(&s_flushes, 1);
    atomic_store(&s_last_flush_ms, now_ms());
  } else {
    /* Put the deltas back so nothing is lost. */
    atomic_fetch_add(&s_presses_delta, p);
    atomic_fetch_add(&s_mismatch_delta, m);
    atomic_store(&s_dirty, true);
    atomic_fetch_add(&s_flush_fails, 1);
  }
  return r;
}

static esp_err_t flush(TickType_t wait)
{
  if (!s_inited || (s_lock == NULL)) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_lock, wait) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t r = flush_locked();
  xSemaphoreGive(s_lock);
  return r;
}

/* esp_restart() path only (not panic). No blocking beyond a short lock wait;
 * NVS commit itself is a few ms. */
static void health_flush_on_shutdown(void)
{
  (void)flush(pdMS_TO_TICKS(50));
}

/* --- Boot accounting ------------------------------------------------------ */

void dcs_health_init(void)
{
  s_lock = xSemaphoreCreateMutex();
  if (s_lock == NULL) {
    ESP_LOGE(TAG, "mutex create failed; health counters disabled");
    return;
  }

  (void)memset(&s_base, 0, sizeof(s_base));
  bool had = dcs_nvs_read_health(&s_base);

  /* Every boot counts. */
  s_base.boots++;

  /* Flash detection: the running image's ELF SHA vs the last one we saw. A
   * fresh blob (had == false) has an all-zero sha, so the very first boot of a
   * unit registers as flash #1 — which it is. */
  const esp_app_desc_t * desc = esp_app_get_description();
  if (desc != NULL) {
    if (memcmp(s_base.fw_sha, desc->app_elf_sha256, sizeof(s_base.fw_sha)) != 0) {
      s_base.flashes++;
      (void)memcpy(s_base.fw_sha, desc->app_elf_sha256, sizeof(s_base.fw_sha));

      /* OTA vs cable: an OTA-delivered image boots PENDING_VERIFY (rollback
       * enabled) until something marks it valid. dcs_support_init calls us
       * before ml_app_start, so nothing can have marked it yet. A cable
       * flash writes otadata "valid" directly. */
      esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
      const esp_partition_t * run = esp_ota_get_running_partition();
      if ((run != NULL) && (esp_ota_get_state_partition(run, &st) == ESP_OK) && (st == ESP_OTA_IMG_PENDING_VERIFY)) {
        s_base.otas++;
      }
    }
  }

  s_inited = true;
  atomic_store(&s_last_flush_ms, now_ms());

  /* Persist the boot/flash accounting now: a unit that never reaches the
   * first periodic flush (crash loop) still records its boots. */
  esp_err_t r = flush(pdMS_TO_TICKS(200));
  ESP_LOGI(
    TAG,
    "lifetime: boots=%lu flashes=%lu otas=%lu presses=%lu mismatch=%lu uptime=%lus (%s, flush %s)",
    (unsigned long)s_base.boots,
    (unsigned long)s_base.flashes,
    (unsigned long)s_base.otas,
    (unsigned long)s_base.presses,
    (unsigned long)s_base.mismatch_events,
    (unsigned long)s_base.uptime_s,
    had ? "from NVS" : "new blob",
    esp_err_to_name(r));

  esp_err_t hook = esp_register_shutdown_handler(health_flush_on_shutdown);
  if (hook != ESP_OK) {
    ESP_LOGW(TAG, "shutdown handler: %s", esp_err_to_name(hook));
  }
}

/* --- Edge hook (safety-core context) -------------------------------------- */

void dcs_health_note_core_tick(int core_id, uint32_t tick, uint8_t verdict)
{
#ifdef DCS_PAGE_MACHINE
  /* machn: the "verdict" is robot state, not a button. Nothing to count. */
  (void)core_id;
  (void)tick;
  (void)verdict;
#else
  if ((core_id < 0) || (core_id > 1) || !s_inited) {
    return;
  }
  dcs_health_edge_out_t e;
  portENTER_CRITICAL(&s_edge_mux);
  e = dcs_health_edge_step(&s_edge, core_id, tick, verdict, (uint8_t)PSTOP_MESSAGE_STOP);
  portEXIT_CRITICAL(&s_edge_mux);
  if (e.press == true) {
    atomic_fetch_add(&s_presses_delta, 1);
    atomic_store(&s_dirty, true);
  }
  if (e.mismatch == true) {
    atomic_fetch_add(&s_mismatch_delta, 1);
    atomic_store(&s_dirty, true);
  }
#endif
}

/* --- Warning board -------------------------------------------------------- */

static int warn_find(const char * src)
{
  for (int i = 0; i < s_warn_n; i++) {
    if (strncmp(s_warn[i].src, src, DCS_HEALTH_SRC_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

static void warn_remove_locked(int idx)
{
  if ((idx < 0) || (idx >= s_warn_n)) {
    return;
  }
  for (int i = idx; i < (s_warn_n - 1); i++) {
    s_warn[i] = s_warn[i + 1];
  }
  s_warn_n--;
  (void)memset(&s_warn[s_warn_n], 0, sizeof(s_warn[0]));
}

esp_err_t dcs_health_publish(const char * src, dcs_health_level_t level, const char * msg)
{
  if (!s_inited || (s_lock == NULL)) {
    return ESP_ERR_INVALID_STATE;
  }
  if ((src == NULL) || (src[0] == '\0')) {
    return ESP_ERR_INVALID_ARG;
  }
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t r = ESP_OK;
  int idx = warn_find(src);
  if (level == DCS_HEALTH_OK) {
    warn_remove_locked(idx);
  } else if (idx >= 0) {
    dcs_health_warning_t * w = &s_warn[idx];
    if (w->level != level) {
      ESP_LOGW(
        TAG,
        "%s: %s -> %s: %s",
        src,
        (w->level == DCS_HEALTH_CRIT) ? "CRIT" : "WARN",
        (level == DCS_HEALTH_CRIT) ? "CRIT" : "WARN",
        (msg != NULL) ? msg : "");
    }
    w->level = level;
    w->count++;
    w->last_ms = now_ms();
    strlcpy(w->msg, (msg != NULL) ? msg : "", sizeof(w->msg));
  } else if (s_warn_n < DCS_HEALTH_MAX_WARNINGS) {
    dcs_health_warning_t * w = &s_warn[s_warn_n];
    s_warn_n++;
    (void)memset(w, 0, sizeof(*w));
    strlcpy(w->src, src, sizeof(w->src));
    strlcpy(w->msg, (msg != NULL) ? msg : "", sizeof(w->msg));
    w->level = level;
    w->count = 1;
    w->first_ms = now_ms();
    w->last_ms = w->first_ms;
    ESP_LOGW(TAG, "%s: %s: %s", src, (level == DCS_HEALTH_CRIT) ? "CRIT" : "WARN", w->msg);
  } else {
    atomic_fetch_add(&s_dropped, 1);
    r = ESP_ERR_NO_MEM;
  }
  recompute_overall();
  xSemaphoreGive(s_lock);
  return r;
}

void dcs_health_clear(const char * src)
{
  (void)dcs_health_publish(src, DCS_HEALTH_OK, NULL);
}

dcs_health_level_t dcs_health_overall(void)
{
  return (dcs_health_level_t)atomic_load(&s_overall);
}

/* --- Snapshot / admin ------------------------------------------------------ */

void dcs_health_get_snapshot(dcs_health_snapshot_t * out)
{
  (void)memset(out, 0, sizeof(*out));
  if (!s_inited || (s_lock == NULL)) {
    return;
  }
  bool locked = (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE);
  snapshot_counters(&out->counters);
  out->button_rated_ops = (uint32_t)CONFIG_DCS_BUTTON_RATED_OPS;
  (void)dcs_health_button_level(
    out->counters.presses, out->button_rated_ops, (uint32_t)CONFIG_DCS_BUTTON_WARN_PCT, &out->button_wear_pct);
  out->level = dcs_health_overall();
  out->nvs_flushes = (uint32_t)atomic_load(&s_flushes);
  out->nvs_flush_fails = (uint32_t)atomic_load(&s_flush_fails);
  uint64_t lf = atomic_load(&s_last_flush_ms);
  uint64_t n = now_ms();
  out->last_flush_age_s = (uint32_t)((n > lf) ? ((n - lf) / 1000ULL) : 0ULL);
  out->dropped = (uint32_t)atomic_load(&s_dropped);
  if (locked) {
    out->n_warnings = s_warn_n;
    (void)memcpy(out->warnings, s_warn, sizeof(s_warn[0]) * (size_t)s_warn_n);
    xSemaphoreGive(s_lock);
  }
}

esp_err_t dcs_health_reset_button(void)
{
  if (!s_inited || (s_lock == NULL)) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  atomic_store(&s_presses_delta, 0);
  atomic_store(&s_mismatch_delta, 0);
  s_base.presses = 0;
  s_base.mismatch_events = 0;
  if (s_base.button_swaps < 255u) {
    s_base.button_swaps++;
  }
  esp_err_t r = flush_locked();
  xSemaphoreGive(s_lock);
  dcs_health_clear("button_wear");
  dcs_health_clear("loop_mismatch");
  ESP_LOGW(TAG, "button counters reset (swap #%u): %s", (unsigned)s_base.button_swaps, esp_err_to_name(r));
  return r;
}

esp_err_t dcs_health_reset_all(void)
{
  if (!s_inited || (s_lock == NULL)) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  atomic_store(&s_presses_delta, 0);
  atomic_store(&s_mismatch_delta, 0);
  uint8_t sha[8];
  (void)memcpy(sha, s_base.fw_sha, sizeof(sha));
  (void)memset(&s_base, 0, sizeof(s_base));
  (void)memcpy(s_base.fw_sha, sha, sizeof(sha)); /* keep: otherwise next boot counts a phantom flash */
  s_base.boots = 1; /* this boot */
  esp_err_t r = flush_locked();
  xSemaphoreGive(s_lock);
  dcs_health_clear("button_wear");
  dcs_health_clear("loop_mismatch");
  ESP_LOGW(TAG, "all lifetime counters reset: %s", esp_err_to_name(r));
  return r;
}

/* --- Built-in publishers ----------------------------------------------------- */

static void check_button_wear(const dcs_health_counters_t * c)
{
#ifdef DCS_PAGE_MACHINE
  (void)c;
#else
  uint32_t pct = 0;
  int lvl = dcs_health_button_level(
    c->presses, (uint32_t)CONFIG_DCS_BUTTON_RATED_OPS, (uint32_t)CONFIG_DCS_BUTTON_WARN_PCT, &pct);
  if (lvl == 0) {
    dcs_health_clear("button_wear");
    return;
  }
  char msg[DCS_HEALTH_MSG_LEN];
  (void)snprintf(
    msg,
    sizeof(msg),
    "%lu%% of rated %lu ops%s",
    (unsigned long)pct,
    (unsigned long)CONFIG_DCS_BUTTON_RATED_OPS,
    (lvl >= 2) ? "; replace switch" : "");
  (void)dcs_health_publish("button_wear", (lvl >= 2) ? DCS_HEALTH_CRIT : DCS_HEALTH_WARN, msg);
#endif
}

static void check_loop_mismatch(const dcs_health_counters_t * c)
{
#ifdef DCS_PAGE_MACHINE
  (void)c;
#else
  int lvl = dcs_health_mismatch_level(
    c->mismatch_events,
    c->presses,
    (uint32_t)CONFIG_DCS_MISMATCH_WARN_MIN,
    (uint32_t)CONFIG_DCS_MISMATCH_WARN_PER_1000);
  if (lvl == 0) {
    dcs_health_clear("loop_mismatch");
    return;
  }
  char msg[DCS_HEALTH_MSG_LEN];
  (void)snprintf(
    msg,
    sizeof(msg),
    "%lu loop mismatches over %lu presses; check switch wiring",
    (unsigned long)c->mismatch_events,
    (unsigned long)c->presses);
  (void)dcs_health_publish("loop_mismatch", DCS_HEALTH_WARN, msg);
#endif
}

/* --- Task ------------------------------------------------------------------- */

static void health_task(void * arg)
{
  (void)arg;
  uint32_t secs = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    secs++;

    uint64_t lf = atomic_load(&s_last_flush_ms);
    uint64_t n = now_ms();
    uint32_t since = (uint32_t)((n > lf) ? ((n - lf) / 1000ULL) : 0ULL);
    bool due = dcs_health_flush_due(
      atomic_load(&s_dirty), since, (uint32_t)CONFIG_DCS_HEALTH_FLUSH_MIN_S, (uint32_t)CONFIG_DCS_HEALTH_FLUSH_MAX_S);
    if (due == true) {
      esp_err_t r = flush(pdMS_TO_TICKS(500));
      if (r != ESP_OK) {
        ESP_LOGW(TAG, "flush: %s", esp_err_to_name(r));
      }
    }

    /* Built-in checks: once shortly after boot, then every minute. */
    if ((secs == 5u) || ((secs % 60u) == 0u)) {
      dcs_health_counters_t c;
      snapshot_counters(&c);
      check_button_wear(&c);
      check_loop_mismatch(&c);
    }
  }
}

void dcs_health_start(void)
{
  if (!s_inited) {
    return;
  }
  /* INTERNAL stack (plain xTaskCreate): this task writes NVS, and a PSRAM
   * stack is unreadable while the flash cache is down (dcs_internal.h). 4 KiB:
   * NVS commit + snprintf + ESP_LOG, same budget as bc_clear. */
  if (xTaskCreate(health_task, "health", 4096, NULL, 1, NULL) != pdPASS) {
    ESP_LOGE(TAG, "task create failed; counters will only flush on restart");
  }
}
