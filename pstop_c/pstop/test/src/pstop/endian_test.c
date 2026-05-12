// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/endian.h"

#include <unity/unity.h>

static
void
test_little_endian_uint16(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U };
    TEST_ASSERT_EQUAL(0x0201U, read_uint16_le(data, &pos));
    TEST_ASSERT_EQUAL(2U, pos);

    uint8_t result[2];
    pos = 0U;
    write_uint16_le(0x0201U, result, &pos);
    TEST_ASSERT_EQUAL(2U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 2U);
}

static
void
test_little_endian_uint32(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U, 0x03U, 0x04U };
    TEST_ASSERT_EQUAL(0x04030201U, read_uint32_le(data, &pos));
    TEST_ASSERT_EQUAL(4U, pos);

    uint8_t result[4];
    pos = 0U;
    write_uint32_le(0x04030201U, result, &pos);
    TEST_ASSERT_EQUAL(4U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 4U);
}

static
void
test_little_endian_uint64(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U, 0x03U, 0x04U, 0x05, 0x06, 0x07, 0x08 };
    TEST_ASSERT_EQUAL(0x0807060504030201ULL, read_uint64_le(data, &pos));
    TEST_ASSERT_EQUAL(8U, pos);

    uint8_t result[8];
    pos = 0U;
    write_uint64_le(0x0807060504030201ULL, result, &pos);
    TEST_ASSERT_EQUAL(8U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 8U);
}

static
void
test_big_endian_uint16(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U };
    TEST_ASSERT_EQUAL(0x0102U, read_uint16_be(data, &pos));
    TEST_ASSERT_EQUAL(2U, pos);

    uint8_t result[2];
    pos = 0U;
    write_uint16_be(0x0102U, result, &pos);
    TEST_ASSERT_EQUAL(2U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 2U);
}

static
void
test_big_endian_uint32(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U, 0x03U, 0x04U };
    TEST_ASSERT_EQUAL(0x01020304U, read_uint32_be(data, &pos));
    TEST_ASSERT_EQUAL(4U, pos);

    uint8_t result[4];
    pos = 0U;
    write_uint32_be(0x01020304U, result, &pos);
    TEST_ASSERT_EQUAL(4U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 4U);
}

static
void
test_big_endian_uint64(void)
{
    size_t pos = 0U;
    uint8_t data[] = { 0x01U, 0x02U, 0x03U, 0x04U, 0x05, 0x06, 0x07, 0x08 };
    TEST_ASSERT_EQUAL(0x0102030405060708ULL, read_uint64_be(data, &pos));
    TEST_ASSERT_EQUAL(8U, pos);

    uint8_t result[8];
    pos = 0U;
    write_uint64_be(0x0102030405060708ULL, result, &pos);
    TEST_ASSERT_EQUAL(8U, pos);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, result, 8U);
}

void
main_endian_test(void)
{
    UnitySetTestFile("endian_test.c");

    RUN_TEST(test_little_endian_uint16);
    RUN_TEST(test_little_endian_uint32);
    RUN_TEST(test_little_endian_uint64);

    RUN_TEST(test_big_endian_uint16);
    RUN_TEST(test_big_endian_uint32);
    RUN_TEST(test_big_endian_uint64);
}
