// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure logic behind the lifetime health counters: the press / mismatch edge
// detectors, the NVS blob codec, and the wear thresholds. No ESP-IDF, no
// FreeRTOS, no I/O, so firmware/test/ can unit-test it on the host. The
// ESP-IDF glue (atomics, NVS, task, HTTP) lives in src/dcs_health.c.

#ifndef DCS_HEALTH_LOGIC_H
#define DCS_HEALTH_LOGIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Persisted counters. Lifetime = survives reboot, idf.py flash and OTA
   * (stable NVS partition offsets); lost on erase-flash / NVS auto-erase. */
  typedef struct
  {
    uint32_t presses; /* both cores OK->STOP rising edges (remote only) */
    uint32_t mismatch_events; /* same-tick core disagreement rising edges (remote only) */
    uint32_t uptime_s; /* cumulative powered seconds */
    uint32_t boots; /* every boot, never auto-cleared */
    uint32_t flashes; /* boots where the ELF SHA changed (cable + OTA) */
    uint32_t otas; /* boots where the image was PENDING_VERIFY (OTA-delivered) */
    uint8_t fw_sha[8]; /* first 8 bytes of the last-seen ELF SHA-256 */
    uint8_t button_swaps; /* admin button-counter resets (switch replaced) */
  } dcs_health_counters_t;

/* Blob layout v1: ver(1) + 6 x u32 BE (24) + sha(8) + swaps(1) = 34 bytes. */
#define DCS_HEALTH_BLOB_VER 1u
#define DCS_HEALTH_BLOB_LEN 34u

  /* Serialize / parse. decode returns false on bad length/version (caller keeps
   * zeroed counters = "new unit", never a hard failure). */
  void dcs_health_encode(const dcs_health_counters_t * c, uint8_t out[DCS_HEALTH_BLOB_LEN]);
  bool dcs_health_decode(const uint8_t * blob, size_t len, dcs_health_counters_t * out);

  /* Edge detectors over the two lockstep cores' per-tick verdicts. Fed from
   * the dcs_publish_core_tick sink, one call per core per tick. The detector
   * only evaluates when both cores have published the SAME tick, and the
   * first such aligned observation sets a baseline without counting, so a
   * button already latched at power-on (or the boot-priming STOP the cores
   * emit for their first ticks) is never counted as a press. */
  typedef struct
  {
    uint8_t seen; /* bit per core: has published at least once */
    bool primed; /* baseline taken from an aligned observation */
    bool stop_both; /* last aligned state: both cores STOP */
    bool mismatch; /* last aligned state: verdicts differ */
    uint32_t tick[2];
    uint8_t verdict[2];
  } dcs_health_edge_t;

  typedef struct
  {
    bool press; /* both-STOP rising edge this call */
    bool mismatch; /* same-tick disagreement rising edge this call */
  } dcs_health_edge_out_t;

  /* stop_code is the STOP verdict value (PSTOP_MESSAGE_STOP) so this file does
   * not depend on the pstop headers. A verdict is either OK or STOP; anything
   * else is treated as "not STOP". */
  dcs_health_edge_out_t dcs_health_edge_step(
    dcs_health_edge_t * st, int core_id, uint32_t tick, uint8_t verdict, uint8_t stop_code);

  /* Wear thresholds. Returns 0 ok / 1 warn / 2 critical.
   * button: percent of rated operations consumed; warn at warn_pct, critical at 100.
   * pct_out (optional) receives the clamped 0..999 percentage. */
  int dcs_health_button_level(uint32_t presses, uint32_t rated_ops, uint32_t warn_pct, uint32_t * pct_out);

  /* loop mismatch: warn once mismatch_events >= min_events AND the rate exceeds
   * per_1000 mismatches per 1000 presses (presses floored at 1000 so a brand
   * new unit with a handful of presses cannot trip it). */
  int dcs_health_mismatch_level(uint32_t mismatch_events, uint32_t presses, uint32_t min_events, uint32_t per_1000);

  /* Flush policy: true if a flush should happen now.
   * dirty -> after min_interval_s; always after max_interval_s. */
  bool dcs_health_flush_due(bool dirty, uint32_t since_last_flush_s, uint32_t min_interval_s, uint32_t max_interval_s);

#ifdef __cplusplus
}
#endif

#endif /* DCS_HEALTH_LOGIC_H */
