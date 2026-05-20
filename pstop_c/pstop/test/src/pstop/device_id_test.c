
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/device_id.h"

#include <unity/unity.h>

static
void
test_device_init(void)
{
    device_id_t device;
    device.data = 0x00010203U;

    device_id_init(&device);
    TEST_ASSERT_EQUAL(device.data, 0x00000000U);
}

static
void
test_device_cmp(void)
{
    device_id_t lhs;
    lhs.data = 0x01020304U;

    device_id_t rhs;
    rhs.data = 0x01020304U;

    TEST_ASSERT_EQUAL(0, device_id_cmp(&lhs, &rhs));
    rhs.data = 0x01020305U;
    TEST_ASSERT_NOT_EQUAL(0, device_id_cmp(&lhs, &rhs));
}

void
main_device_id_test(void)
{
    UnitySetTestFile("device_id_test.c");

    RUN_TEST(test_device_init);
    RUN_TEST(test_device_cmp);
}
