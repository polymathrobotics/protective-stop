// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_telemetry.c
 * @brief Per-core CPU load + per-bucket top-N task breakdown.
 *
 * Sampled once per second via uxTaskGetSystemState. Per-core load is
 * derived from idle-task runtime deltas. Tasks are bucketed by xCoreID
 * (0=core0, 1=core1, tskNO_AFFINITY=shared) and the top-DCS_TOP_N
 * delta-runtime tasks per bucket are exposed in the snapshot.
 *
 * Double-buffered so the /state.json reader never blocks the sampler.
 */

#include <stdatomic.h>
#include <string.h>

#include "dcs_internal.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char * TAG = "dcs_telem";

#define LOAD_SAMPLE_MS 1000
#define MAX_TASKS \
  64 /* headroom over the live task count (was 40); the overflow guard in the sampler still protects if exceeded */

static dcs_task_breakdown_t s_bd[2];
static atomic_int s_bd_cur = 0;

static int affinity_bucket(BaseType_t xCoreID)
{
  if (xCoreID == 0) {
    return 0;
  }
  if (xCoreID == 1) {
    return 1;
  }
  return 2;
}

static uint32_t prev_runtime(const TaskStatus_t * prev, UBaseType_t prev_n, TaskHandle_t h, bool * found)
{
  for (UBaseType_t i = 0; i < prev_n; i++) {
    if (prev[i].xHandle == h) {
      *found = true;
      return prev[i].ulRunTimeCounter;
    }
  }
  *found = false;
  return 0;
}

typedef struct
{
  const char * name;
  uint32_t delta_us;
} top_entry_t;

static void top_insert(top_entry_t * top, int * n, const char * name, uint32_t delta)
{
  int pos = *n;
  for (int i = 0; i < *n; i++) {
    if (delta > top[i].delta_us) {
      pos = i;
      break;
    }
  }
  if (pos >= DCS_TOP_N) {
    return;
  }
  int end = (*n < DCS_TOP_N) ? *n : (DCS_TOP_N - 1);
  for (int i = end; i > pos; i--) {
    top[i] = top[i - 1];
  }
  top[pos].name = name;
  top[pos].delta_us = delta;
  if (*n < DCS_TOP_N) {
    (*n)++;
  }
}

static void load_sampler(void * arg)
{
  (void)arg;
  static TaskStatus_t snap_a[MAX_TASKS];
  static TaskStatus_t snap_b[MAX_TASKS];
  TaskStatus_t *cur_snap = snap_a, *prev_snap = snap_b;
  UBaseType_t cur_n = 0, prev_n = 0;

  uint32_t prev_idle[2] = {0, 0};
  int64_t prev_us = esp_timer_get_time();
  bool primed = false;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(LOAD_SAMPLE_MS));

    atomic_store(&g_dcs_heap_min_internal, (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));

    cur_n = uxTaskGetSystemState(cur_snap, MAX_TASKS, NULL);
    if (cur_n == 0u) {
      /* Array too small — uxTaskGetSystemState returns 0 when the live
             * task count exceeds MAX_TASKS. Left unguarded this made idle[]
             * read 0 and load compute as a bogus value (often 0%), i.e. the
             * sampler silently reported an IDLE system when it was the
             * opposite. Skip this sample and keep prev_* so the next valid
             * sample computes a correct (if longer-interval) delta. */
      ESP_LOGW(TAG, "task snapshot overflow (>%d tasks) — load sample skipped", MAX_TASKS);
      continue;
    }

    uint32_t idle[2] = {0, 0};
    for (UBaseType_t i = 0; i < cur_n; i++) {
      const char * name = cur_snap[i].pcTaskName;
      if (name == NULL) {
        continue;
      }
      if (strcmp(name, "IDLE0") == 0) {
        idle[0] = cur_snap[i].ulRunTimeCounter;
      } else if (strcmp(name, "IDLE1") == 0) {
        idle[1] = cur_snap[i].ulRunTimeCounter;
      } else {
        /* not an idle task */
      }
    }

    int64_t now_us = esp_timer_get_time();
    if (primed) {
      int64_t d_wall_us = now_us - prev_us;
      uint32_t d_wall = (uint32_t)d_wall_us;
      if (d_wall == 0u) {
        d_wall = 1u;
      }

      for (int c = 0; c < 2; c++) {
        uint32_t d_idle = idle[c] - prev_idle[c];
        uint32_t load = (d_wall > d_idle) ? (uint32_t)(100u - ((uint64_t)d_idle * 100u) / d_wall) : 0u;
        if (load > 100u) {
          load = 100u;
        }
        atomic_store(&g_dcs_load_pct[c], load);
      }

      top_entry_t top[3][DCS_TOP_N] = {0};
      int top_n[3] = {0};
      uint32_t total[3] = {0};

      for (UBaseType_t i = 0; i < cur_n; i++) {
        bool found = false;
        uint32_t before = prev_runtime(prev_snap, prev_n, cur_snap[i].xHandle, &found);
        if (!found) {
          continue;
        }
        uint32_t delta = cur_snap[i].ulRunTimeCounter - before;
        if (delta == 0u) {
          continue;
        }

        int bkt = affinity_bucket(cur_snap[i].xCoreID);
        total[bkt] += delta;
        top_insert(top[bkt], &top_n[bkt], cur_snap[i].pcTaskName, delta);
      }

      int next = 1 - atomic_load(&s_bd_cur);
      dcs_task_breakdown_t * out = &s_bd[next];
      (void)memset(out, 0, sizeof(*out));

      for (int bkt = 0; bkt < 3; bkt++) {
        uint32_t accounted = 0;
        out->b[bkt].n = (uint8_t)top_n[bkt];
        for (int i = 0; i < top_n[bkt]; i++) {
          uint32_t pct = (uint32_t)(((uint64_t)top[bkt][i].delta_us * 100u) / d_wall);
          out->b[bkt].tasks[i].pct = pct;
          (void)strncpy(out->b[bkt].tasks[i].name, top[bkt][i].name, sizeof(out->b[bkt].tasks[i].name) - 1u);
          accounted += top[bkt][i].delta_us;
        }
        uint32_t rem = (total[bkt] > accounted) ? (total[bkt] - accounted) : 0u;
        out->b[bkt].other_pct = (uint32_t)(((uint64_t)rem * 100u) / d_wall);
      }
      atomic_store(&s_bd_cur, next);
    }

    TaskStatus_t * tmp = prev_snap;
    prev_snap = cur_snap;
    cur_snap = tmp;
    prev_n = cur_n;

    prev_idle[0] = idle[0];
    prev_idle[1] = idle[1];
    prev_us = now_us;
    primed = true;
  }
}

void dcs_telemetry_start_sampler(void)
{
  /* PSRAM stack: load sampler is non-safety, does no flash/NVS. */
  (void)dcs_task_spawn_psram(load_sampler, "load_sampler", 4096, NULL, 3, tskNO_AFFINITY);
}

void dcs_telemetry_snapshot(dcs_task_breakdown_t * out)
{
  int cur = atomic_load(&s_bd_cur);
  *out = s_bd[cur];
}
