// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#include <string>

#include <gtest/gtest.h>

#include "protective_stop_machine/json_lite.hpp"

using jsonlite::Value;
using jsonlite::parse;

// Parses the flat /state.json shape the hardware backend depends on.
TEST(JsonLite, ParsesMachnStateShape)
{
  Value v;
  ASSERT_TRUE(parse(
      R"({"relay_stop":false,"relay_fault_a":false,"pstop_mismatch":3,)"
      R"("bonded_remotes":[{"id":30928592,"state":2,"age_ms":101,"rtt_ms":209}]})",
      v));
  EXPECT_TRUE(v.is_obj());
  EXPECT_FALSE(v.bool_at("relay_stop", true));
  EXPECT_EQ(v.num_at("pstop_mismatch"), 3.0);
  const Value * br = v.find("bonded_remotes");
  ASSERT_NE(br, nullptr);
  ASSERT_TRUE(br->is_arr());
  ASSERT_EQ(br->arr.size(), 1u);
  EXPECT_EQ(static_cast<uint32_t>(br->arr[0].num_at("id")), 30928592u);
  EXPECT_EQ(static_cast<int>(br->arr[0].num_at("state")), 2);
  EXPECT_EQ(static_cast<int>(br->arr[0].num_at("age_ms")), 101);
}

// The depth cap must reject a hostile deeply-nested document (no stack overflow).
TEST(JsonLite, RejectsPathologicalNesting)
{
  std::string deep(500, '[');   // far past kMaxDepth
  Value v;
  EXPECT_FALSE(parse(deep, v));  // fails cleanly, must not crash
}

TEST(JsonLite, RejectsMalformed)
{
  Value a; EXPECT_FALSE(parse("{bad", a));
  Value b; EXPECT_FALSE(parse("", b));
  Value c; EXPECT_FALSE(parse("[1,2", c));
}

TEST(JsonLite, NumbersBoolsNull)
{
  Value v;
  ASSERT_TRUE(parse(R"({"a":-1.5,"b":false,"n":null,"z":0})", v));
  EXPECT_DOUBLE_EQ(v.num_at("a"), -1.5);
  EXPECT_FALSE(v.bool_at("b", true));
  EXPECT_FALSE(v.bool_at("z", true));       // 0 -> false
  const Value * n = v.find("n");
  ASSERT_NE(n, nullptr);
  EXPECT_EQ(n->type, Value::NUL);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
