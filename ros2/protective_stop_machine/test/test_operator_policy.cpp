// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Operator-authorization policy for the software backend (SAFETY). Locks in the
// "operator vs stop-only remote" model: a bonded remote is STOP-ONLY (may STOP +
// is heartbeat-monitored, may NEVER re-arm) unless its 32-bit pstop id is on the
// operator allowlist. An EMPTY allowlist therefore makes every remote stop-only
// = maximally safe out of the box. This is the exact decision cb_remote_details
// feeds into pstop_c's is_stop_only, which gates re-arm ownership (machine.c).
#include <gtest/gtest.h>

#include "protective_stop_machine/software_backend.hpp"

using protective_stop_machine::software_remote_is_stop_only;
using protective_stop_machine::SoftwareConfig;

// The out-of-the-box default: no operators listed -> every remote is stop-only,
// i.e. cannot re-arm. This is the hole the fix closes (was accept-all-as-op).
TEST(OperatorPolicy, EmptyAllowlistMakesEveryRemoteStopOnly)
{
  SoftwareConfig cfg;  // defaults: operators empty, default_stop_only true
  EXPECT_TRUE(cfg.operators.empty());
  EXPECT_TRUE(cfg.default_stop_only);
  EXPECT_TRUE(software_remote_is_stop_only(cfg, 0x01d7791cU));
  EXPECT_TRUE(software_remote_is_stop_only(cfg, 0U));
  EXPECT_TRUE(software_remote_is_stop_only(cfg, 0xFFFFFFFFU));
}

// A listed operator id gets full authority (not stop-only -> may re-arm).
TEST(OperatorPolicy, ListedOperatorMayReArm)
{
  SoftwareConfig cfg;
  cfg.operators = {0x01d7791cU, 0x01aabbccU};
  EXPECT_FALSE(software_remote_is_stop_only(cfg, 0x01d7791cU));
  EXPECT_FALSE(software_remote_is_stop_only(cfg, 0x01aabbccU));
}

// A remote NOT on a non-empty allowlist stays stop-only.
TEST(OperatorPolicy, UnlistedRemoteStaysStopOnly)
{
  SoftwareConfig cfg;
  cfg.operators = {0x01d7791cU};
  EXPECT_TRUE(software_remote_is_stop_only(cfg, 0x02000000U));
  EXPECT_TRUE(software_remote_is_stop_only(cfg, 0x01d7791dU));  // off-by-one id
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
