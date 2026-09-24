// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <string>

#include "protective_stop_machine/json_lite.hpp"

using jsonlite::parse;
using jsonlite::Value;

// Parses the flat /state.json shape the hardware backend depends on.
TEST(JsonLite, ParsesMachnStateShape)
{
  Value parsed;
  ASSERT_TRUE(parse(
    R"({"relay_stop":false,"relay_fault_a":false,"pstop_mismatch":3,)"
    R"("bonded_remotes":[{"id":30928592,"state":2,"age_ms":101,"rtt_ms":209}]})",
    parsed));
  EXPECT_TRUE(parsed.is_obj());
  EXPECT_FALSE(parsed.bool_at("relay_stop", true));
  EXPECT_EQ(parsed.num_at("pstop_mismatch"), 3.0);
  const Value * bonded_remotes = parsed.find("bonded_remotes");
  ASSERT_NE(bonded_remotes, nullptr);
  ASSERT_TRUE(bonded_remotes->is_arr());
  ASSERT_EQ(bonded_remotes->arr.size(), 1u);
  EXPECT_EQ(static_cast<uint32_t>(bonded_remotes->arr[0].num_at("id")), 30928592u);
  EXPECT_EQ(static_cast<int>(bonded_remotes->arr[0].num_at("state")), 2);
  EXPECT_EQ(static_cast<int>(bonded_remotes->arr[0].num_at("age_ms")), 101);
}

// The depth cap must reject a hostile deeply-nested document (no stack overflow).
TEST(JsonLite, RejectsPathologicalNesting)
{
  // far past kMaxDepth
  std::string deeply_nested(500, '[');
  Value parsed;
  // fails cleanly, must not crash
  EXPECT_FALSE(parse(deeply_nested, parsed));
}

TEST(JsonLite, RejectsMalformed)
{
  Value first;
  EXPECT_FALSE(parse("{bad", first));
  Value second;
  EXPECT_FALSE(parse("", second));
  Value third;
  EXPECT_FALSE(parse("[1,2", third));
}

TEST(JsonLite, NumbersBoolsNull)
{
  Value parsed;
  ASSERT_TRUE(parse(R"({"a":-1.5,"b":false,"n":null,"z":0})", parsed));
  EXPECT_DOUBLE_EQ(parsed.num_at("a"), -1.5);
  EXPECT_FALSE(parsed.bool_at("b", true));
  // 0 -> false
  EXPECT_FALSE(parsed.bool_at("z", true));
  const Value * null_value = parsed.find("n");
  ASSERT_NE(null_value, nullptr);
  EXPECT_EQ(null_value->type, Value::NUL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
