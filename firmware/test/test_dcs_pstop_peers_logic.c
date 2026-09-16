// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the ps_peers blob codec and the v1->v2 per-peer-role
// migration. The migration runs once per remote on the OTA that introduces
// per-peer roles and cannot be re-run, so it is covered here rather than on
// target.

#include <stdio.h>
#include <string.h>

#include "dcs_pstop_peers_logic.h"

static int g_fail;

#define CHECK(cond)                                          \
  do {                                                       \
    if (!(cond)) {                                           \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      g_fail++;                                              \
    }                                                        \
  } while (0)

static const uint8_t ROLE_STOP_ONLY = (uint8_t)PSTOP_AUX_ROLE_STOP_ONLY;
static const uint8_t ROLE_OPERATOR = (uint8_t)PSTOP_AUX_ROLE_OPERATOR;

/* Build a v1 blob (11-byte records, no role byte) the way the shipped firmware
 * wrote it, so the migration is tested against the real historical layout. */
static void make_v1(uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN_V1], int slots)
{
  (void)memset(blob, 0, DCS_PSTOP_PEERS_BLOB_LEN_V1);
  blob[0] = DCS_PSTOP_PEERS_VER_V1;
  for (int i = 0; i < slots; i++) {
    uint8_t * rec = &blob[1 + ((size_t)i * DCS_PSTOP_PEERS_REC_LEN_V1)];
    rec[0] = 1u;
    rec[1] = 10u; /* 10.0.0.(1+i) */
    rec[2] = 0u;
    rec[3] = 0u;
    rec[4] = (uint8_t)(1 + i);
    rec[5] = 0x22; /* port 8890 */
    rec[6] = 0xBA;
    rec[10] = (uint8_t)(0x10 + i); /* machine_id */
  }
}

static void test_round_trip(void)
{
  dcs_pstop_peer_rec_t in[DCS_PSTOP_PEERS_MAX] = {0};
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    in[i].role = ROLE_STOP_ONLY; /* what encode narrows an unset slot to */
  }
  in[0].configured = true;
  in[0].ip = 0x0A000001u;
  in[0].port = 8890;
  in[0].machine_id = 0x01020304u;
  in[0].role = ROLE_OPERATOR;
  in[2].configured = true;
  in[2].ip = 0x0A000002u;
  in[2].port = 9999;
  in[2].machine_id = 0xDEADBEEFu;
  in[2].role = ROLE_STOP_ONLY;

  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN];
  dcs_pstop_peers_encode(in, blob);
  CHECK(blob[0] == DCS_PSTOP_PEERS_VER);

  dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX];
  CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_STOP_ONLY, out));
  CHECK(memcmp(in, out, sizeof(in)) == 0);

  /* A per-peer role really is per peer: slot 0 operator, slot 2 stop-only. */
  CHECK(out[0].role == ROLE_OPERATOR);
  CHECK(out[2].role == ROLE_STOP_ONLY);
  /* Unconfigured slots are stop-only, never left as a zero/unspecified byte. */
  CHECK(out[1].role == ROLE_STOP_ONLY);
  CHECK(!out[1].configured);
}

static void test_v1_migration_seeds_from_global_role(void)
{
  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN_V1];
  make_v1(blob, 2);

  /* A remote promoted under the old global key keeps arming: every slot
   * inherits operator. */
  dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX];
  CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_OPERATOR, out));
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    CHECK(out[i].role == ROLE_OPERATOR);
  }
  CHECK(out[0].configured);
  CHECK(out[0].ip == 0x0A000001u);
  CHECK(out[0].port == 8890u);
  CHECK(out[1].configured);
  CHECK(!out[2].configured);

  /* A remote that was stop-only stays stop-only. */
  CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_STOP_ONLY, out));
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    CHECK(out[i].role == ROLE_STOP_ONLY);
  }

  /* An absent/corrupt global key (0 = unspecified) must not grant authority. */
  CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), 0u, out));
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    CHECK(out[i].role == ROLE_STOP_ONLY);
  }
}

static void test_migration_is_one_way(void)
{
  /* Re-encoding a migrated v1 table writes v2, and a second decode no longer
   * consults the global key — so a later demotion cannot be undone by the
   * stale NVS role value. */
  uint8_t v1[DCS_PSTOP_PEERS_BLOB_LEN_V1];
  make_v1(v1, 1);

  dcs_pstop_peer_rec_t recs[DCS_PSTOP_PEERS_MAX];
  CHECK(dcs_pstop_peers_decode(v1, sizeof(v1), ROLE_OPERATOR, recs));
  recs[0].role = ROLE_STOP_ONLY; /* admin demotes slot 0 after the upgrade */

  uint8_t v2[DCS_PSTOP_PEERS_BLOB_LEN];
  dcs_pstop_peers_encode(recs, v2);

  dcs_pstop_peer_rec_t back[DCS_PSTOP_PEERS_MAX];
  CHECK(dcs_pstop_peers_decode(v2, sizeof(v2), ROLE_OPERATOR, back));
  CHECK(back[0].role == ROLE_STOP_ONLY);
}

static void test_unknown_role_byte_is_stop_only(void)
{
  dcs_pstop_peer_rec_t in[DCS_PSTOP_PEERS_MAX] = {0};
  in[0].configured = true;
  in[0].ip = 0x0A000001u;
  in[0].port = 8890;
  in[0].role = ROLE_OPERATOR;

  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN];
  dcs_pstop_peers_encode(in, blob);

  /* Flash bit-rot / a future schema value must fail safe, never to operator. */
  const uint8_t role_off = 1u + 11u;
  const uint8_t bogus[] = {0u, 3u, 0x7Fu, 0xFFu};
  for (size_t i = 0; i < sizeof(bogus); i++) {
    blob[role_off] = bogus[i];
    dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX];
    CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_OPERATOR, out));
    CHECK(out[0].role == ROLE_STOP_ONLY);
  }
}

static void test_encode_narrows_bogus_role(void)
{
  dcs_pstop_peer_rec_t in[DCS_PSTOP_PEERS_MAX] = {0};
  in[0].configured = true;
  in[0].ip = 0x0A000001u;
  in[0].port = 8890;
  in[0].role = 0x42u; /* never written by the API, but never persisted either */

  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN];
  dcs_pstop_peers_encode(in, blob);
  CHECK(blob[1 + 11] == ROLE_STOP_ONLY);
}

static void test_bad_blobs_rejected(void)
{
  dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX];
  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN] = {0};

  blob[0] = DCS_PSTOP_PEERS_VER;
  CHECK(!dcs_pstop_peers_decode(blob, sizeof(blob) - 1u, ROLE_OPERATOR, out)); /* short */
  CHECK(!dcs_pstop_peers_decode(blob, sizeof(blob) + 1u, ROLE_OPERATOR, out)); /* long */

  blob[0] = 99u;
  CHECK(!dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_OPERATOR, out)); /* future version */

  CHECK(!dcs_pstop_peers_decode(NULL, sizeof(blob), ROLE_OPERATOR, out));

  /* A rejected blob leaves every slot unconfigured AND stop-only, so the
   * caller's fallback can never inherit an operator from garbage. */
  for (int i = 0; i < DCS_PSTOP_PEERS_MAX; i++) {
    CHECK(!out[i].configured);
    CHECK(out[i].role == ROLE_STOP_ONLY);
  }
}

static void test_zero_ip_or_port_degrades(void)
{
  dcs_pstop_peer_rec_t in[DCS_PSTOP_PEERS_MAX] = {0};
  in[0].configured = true;
  in[0].ip = 0u; /* cleared */
  in[0].port = 8890;
  in[0].role = ROLE_OPERATOR;
  in[1].configured = true;
  in[1].ip = 0x0A000001u;
  in[1].port = 0u;
  in[1].role = ROLE_OPERATOR;

  uint8_t blob[DCS_PSTOP_PEERS_BLOB_LEN];
  dcs_pstop_peers_encode(in, blob);

  dcs_pstop_peer_rec_t out[DCS_PSTOP_PEERS_MAX];
  CHECK(dcs_pstop_peers_decode(blob, sizeof(blob), ROLE_STOP_ONLY, out));
  CHECK(!out[0].configured);
  CHECK(!out[1].configured);
}

int main(void)
{
  test_round_trip();
  test_v1_migration_seeds_from_global_role();
  test_migration_is_one_way();
  test_unknown_role_byte_is_stop_only();
  test_encode_narrows_bogus_role();
  test_bad_blobs_rejected();
  test_zero_ip_or_port_degrades();

  if (g_fail != 0) {
    printf("test_dcs_pstop_peers_logic: %d FAILURES\n", g_fail);
    return 1;
  }
  printf("test_dcs_pstop_peers_logic: all checks passed\n");
  return 0;
}
