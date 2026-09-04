// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Pure logic for the lifetime health counters. See dcs_health_logic.h.
// Host-tested in firmware/test/test_dcs_health_logic.c.

#include "dcs_health_logic.h"

#include <string.h>

static void put_u32(uint8_t * p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t get_u32(const uint8_t * p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

void dcs_health_encode(const dcs_health_counters_t * c, uint8_t out[DCS_HEALTH_BLOB_LEN])
{
  out[0] = (uint8_t)DCS_HEALTH_BLOB_VER;
  put_u32(&out[1], c->presses);
  put_u32(&out[5], c->mismatch_events);
  put_u32(&out[9], c->uptime_s);
  put_u32(&out[13], c->boots);
  put_u32(&out[17], c->flashes);
  put_u32(&out[21], c->otas);
  (void)memcpy(&out[25], c->fw_sha, sizeof(c->fw_sha));
  out[33] = c->button_swaps;
}

bool dcs_health_decode(const uint8_t * blob, size_t len, dcs_health_counters_t * out)
{
  if ((blob == NULL) || (out == NULL) || (len != DCS_HEALTH_BLOB_LEN) || (blob[0] != (uint8_t)DCS_HEALTH_BLOB_VER)) {
    return false;
  }
  out->presses = get_u32(&blob[1]);
  out->mismatch_events = get_u32(&blob[5]);
  out->uptime_s = get_u32(&blob[9]);
  out->boots = get_u32(&blob[13]);
  out->flashes = get_u32(&blob[17]);
  out->otas = get_u32(&blob[21]);
  (void)memcpy(out->fw_sha, &blob[25], sizeof(out->fw_sha));
  out->button_swaps = blob[33];
  return true;
}

dcs_health_edge_out_t dcs_health_edge_step(
  dcs_health_edge_t * st, int core_id, uint32_t tick, uint8_t verdict, uint8_t stop_code)
{
  dcs_health_edge_out_t out = {false, false};
  if ((core_id < 0) || (core_id > 1)) {
    return out;
  }
  st->tick[core_id] = tick;
  st->verdict[core_id] = verdict;
  st->seen |= (core_id == 0) ? 1u : 2u;

  /* Only a same-tick comparison is meaningful: the two cores publish a few
   * microseconds apart, so between the two publishes the ticks differ by one.
   * Skewed observations carry no information; wait for the partner. */
  if ((st->seen != 3u) || (st->tick[0] != st->tick[1])) {
    return out;
  }
  const bool s0 = (st->verdict[0] == stop_code);
  const bool s1 = (st->verdict[1] == stop_code);
  const bool stop_both = s0 && s1;
  const bool mismatch = (s0 != s1);

  if (st->primed == true) {
    out.press = stop_both && !st->stop_both;
    out.mismatch = mismatch && !st->mismatch;
  }
  st->primed = true;
  st->stop_both = stop_both;
  st->mismatch = mismatch;
  return out;
}

int dcs_health_button_level(uint32_t presses, uint32_t rated_ops, uint32_t warn_pct, uint32_t * pct_out)
{
  uint32_t pct = 0;
  if (rated_ops > 0u) {
    /* 64-bit intermediate: presses may exceed 42M before *100 overflows u32. */
    uint64_t p = ((uint64_t)presses * 100u) / rated_ops;
    pct = (p > 999u) ? 999u : (uint32_t)p;
  }
  if (pct_out != NULL) {
    *pct_out = pct;
  }
  if (rated_ops == 0u) {
    return 0; /* rating disabled */
  }
  if (pct >= 100u) {
    return 2;
  }
  if ((warn_pct > 0u) && (pct >= warn_pct)) {
    return 1;
  }
  return 0;
}

int dcs_health_mismatch_level(uint32_t mismatch_events, uint32_t presses, uint32_t min_events, uint32_t per_1000)
{
  if ((per_1000 == 0u) || (mismatch_events < min_events)) {
    return 0;
  }
  const uint64_t basis = (presses < 1000u) ? 1000ULL : (uint64_t)presses;
  const uint64_t allowed = (basis * per_1000) / 1000u;
  return ((uint64_t)mismatch_events > allowed) ? 1 : 0;
}

bool dcs_health_flush_due(bool dirty, uint32_t since_last_flush_s, uint32_t min_interval_s, uint32_t max_interval_s)
{
  if (since_last_flush_s >= max_interval_s) {
    return true;
  }
  return dirty && (since_last_flush_s >= min_interval_s);
}
