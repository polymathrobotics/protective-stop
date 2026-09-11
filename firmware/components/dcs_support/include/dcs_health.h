// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_health.h
 * @brief Lifetime wear/health counters + a process-wide health-warning board.
 *
 * Two things live here:
 *
 * 1. Persistent counters (button presses, lockstep mismatch events, cumulative
 *    uptime, boots, flashes, OTAs) kept in RAM and flushed to one NVS blob.
 *    They survive reboots, `idf.py flash` and OTA; they do not survive
 *    erase-flash. Read them at GET /api/health. The purpose is to know when a
 *    unit should be retired: the stop switch on the BOM (NKK FF01) is rated
 *    for 100,000 operations.
 *
 * 2. dcs_health_publish(): any task on the chip can raise (or clear) a named
 *    warning. The board keeps one entry per source, tracks how often it was
 *    re-raised and for how long, and /api/health + /state.json expose the
 *    worst level. The first publishers are the built-in button-wear and
 *    loop-mismatch checks in dcs_health.c; add others (heap floor, flash
 *    write failures, RSSI, ...) from wherever the evidence lives.
 *
 * Not a safety function. Nothing here influences the pstop verdict or
 * transmission; it observes the telemetry the safety cores already publish.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dcs_health_logic.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    DCS_HEALTH_OK = 0,
    DCS_HEALTH_WARN = 1,
    DCS_HEALTH_CRIT = 2,
  } dcs_health_level_t;

#define DCS_HEALTH_MAX_WARNINGS 8
#define DCS_HEALTH_SRC_LEN 16 /* incl. NUL */
#define DCS_HEALTH_MSG_LEN 64 /* incl. NUL */

  typedef struct
  {
    char src[DCS_HEALTH_SRC_LEN];
    char msg[DCS_HEALTH_MSG_LEN];
    dcs_health_level_t level;
    uint32_t count; /* publishes for this source since first raised */
    uint64_t first_ms; /* esp_timer ms when first raised (this boot) */
    uint64_t last_ms; /* esp_timer ms of the latest publish */
  } dcs_health_warning_t;

  /* Snapshot for the HTTP layer. */
  typedef struct
  {
    dcs_health_counters_t counters; /* live RAM values (may be ahead of NVS) */
    dcs_health_level_t level; /* worst active warning */
    uint32_t button_rated_ops;
    uint32_t button_wear_pct;
    uint32_t nvs_flushes; /* successful flushes this boot */
    uint32_t nvs_flush_fails;
    uint32_t last_flush_age_s;
    uint32_t dropped; /* publishes refused: table full */
    int n_warnings;
    dcs_health_warning_t warnings[DCS_HEALTH_MAX_WARNINGS];
  } dcs_health_snapshot_t;

  /* --- Publish API (task context only; not ISR-safe) ----------------------- */

  /** Raise or update a warning. One entry per @p src (upsert); a repeated
   * publish bumps count/last_ms and replaces level/msg. Level OK clears the
   * entry (same as dcs_health_clear). Returns ESP_ERR_NO_MEM when the table is
   * full (counted in `dropped`), ESP_ERR_INVALID_STATE before init. */
  esp_err_t dcs_health_publish(const char * src, dcs_health_level_t level, const char * msg);

  /** Remove a source's warning, if present. */
  void dcs_health_clear(const char * src);

  /** Worst level currently on the board. Cheap (atomic read); safe from any
   * task. Mirrored into /state.json as `health`. */
  dcs_health_level_t dcs_health_overall(void);

  /* --- Snapshot / admin -------------------------------------------------- */

  void dcs_health_get_snapshot(dcs_health_snapshot_t * out);

  /** Zero the button counters (presses, mismatch_events) and bump
   * button_swaps; flushes immediately. For a switch replacement. */
  esp_err_t dcs_health_reset_button(void);

  /** Zero every counter (refurbished unit); flushes immediately. */
  esp_err_t dcs_health_reset_all(void);

#ifdef __cplusplus
}
#endif
