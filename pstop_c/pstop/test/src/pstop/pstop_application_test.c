// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_application.h"

#include <unity/unity.h>

static
void
test_init(void)
{
    pstop_application_t app;
    pstop_application_init(&app);

    TEST_ASSERT_NULL(app.operator_details_cb);
    TEST_ASSERT_NULL(app.status_cb);
    TEST_ASSERT_NOT_NULL(app.log_message_cb);
    TEST_ASSERT_NOT_NULL(app.env.get_time_cb);
    TEST_ASSERT_TRUE(app.env.get_time_cb() > 0U);

    app.log_message_cb(PSTOP_OK, "no error");

    operator_details_t details;
    operator_detail_init(&details);
    TEST_ASSERT_EQUAL(1, details.stop_only);
    TEST_ASSERT_EQUAL(1, details.allowed);
}

void
main_pstop_application_test(void)
{
    UnitySetTestFile("pstop_application_test.c");

    RUN_TEST(test_init);
}
