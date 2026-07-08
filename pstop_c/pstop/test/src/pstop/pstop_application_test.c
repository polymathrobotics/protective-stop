// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_application.h"

#include <unity/unity.h>

#include "pstop/pstop_msg.h"

static
void
print_log(uint64_t /* timestamp */, const device_id_t * /* client */, uint8_t /* message */, pstop_error_t /* error */)
{

}

static
void
test_init(void)
{
    pstop_application_t app;
    pstop_application_init(&app);

    TEST_ASSERT_NULL(app.remote_details_cb);
    TEST_ASSERT_NULL(app.status_cb);
    TEST_ASSERT_NOT_NULL(app.log_message_cb);
    TEST_ASSERT_NOT_NULL(app.env.get_time_cb);
    TEST_ASSERT_TRUE(app.env.get_time_cb() > 0U);

    device_id_t id;
    id.data = 0x12345;

    app.log_message_cb(12345, &id, PSTOP_MESSAGE_BOND, PSTOP_OK);

    pstop_application_set_log_cb(&app, print_log);
    TEST_ASSERT_EQUAL(print_log, app.log_message_cb);

    pstop_application_set_machine_id(&app, &id);
    TEST_ASSERT_EQUAL(0U, device_id_cmp(&id, &(app.machine_device_id)));

    remote_details_t details;
    remote_detail_init(&details);
    TEST_ASSERT_EQUAL(1, details.stop_only);
    TEST_ASSERT_EQUAL(1, details.allowed);
}

void
main_pstop_application_test(void)
{
    UnitySetTestFile("pstop_application_test.c");

    RUN_TEST(test_init);
}
