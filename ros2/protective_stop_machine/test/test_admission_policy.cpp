// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Admission policy for the software backend: whether a remote may BOND at all.
// Two optional global lists. Both empty (default) admits everyone; a non-empty
// allowlist admits only listed ids; the denylist always refuses and wins.
// Admission is deliberately separate from re-arm authority, which is the
// remote's own announced role (common/pstop_aux_channel.h) and is not decided
// here. This is the exact decision resolve_remote_details feeds into pstop_c's
// remote_details_t.allowed (a refused id gets an UNBOND reply).
#include <gtest/gtest.h>

#include "protective_stop_machine/software_backend.hpp"

using protective_stop_machine::software_remote_admitted;
using protective_stop_machine::SoftwareConfig;

TEST(AdmissionPolicy, BothListsEmptyAdmitsEveryone)
{
  SoftwareConfig cfg;
  EXPECT_TRUE(cfg.allowlist.empty());
  EXPECT_TRUE(cfg.denylist.empty());
  EXPECT_TRUE(software_remote_admitted(cfg, 0x01d7791cU));
  EXPECT_TRUE(software_remote_admitted(cfg, 0xFFFFFFFFU));
}

TEST(AdmissionPolicy, NonEmptyAllowlistAdmitsOnlyListed)
{
  SoftwareConfig cfg;
  cfg.allowlist = {0x01d7791cU, 0x01aabbccU};
  EXPECT_TRUE(software_remote_admitted(cfg, 0x01d7791cU));
  EXPECT_TRUE(software_remote_admitted(cfg, 0x01aabbccU));
  // off-by-one id
  EXPECT_FALSE(software_remote_admitted(cfg, 0x01d7791dU));
  EXPECT_FALSE(software_remote_admitted(cfg, 0x02000000U));
}

TEST(AdmissionPolicy, DenylistRefusesAndWinsOverAllowlist)
{
  SoftwareConfig cfg;
  cfg.denylist = {0x01d7791cU};
  EXPECT_FALSE(software_remote_admitted(cfg, 0x01d7791cU));
  // open otherwise
  EXPECT_TRUE(software_remote_admitted(cfg, 0x01aabbccU));

  // listed on both: deny wins
  cfg.allowlist = {0x01d7791cU, 0x01aabbccU};
  EXPECT_FALSE(software_remote_admitted(cfg, 0x01d7791cU));
  EXPECT_TRUE(software_remote_admitted(cfg, 0x01aabbccU));
}
