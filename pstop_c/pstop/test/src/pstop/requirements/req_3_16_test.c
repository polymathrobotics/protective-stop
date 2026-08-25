// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"
#include "pstop/requirements/test_utils.h"

#define MACHINE_ID 1236
#define REMOTE_ID 1234
#define REMOTE_2_ID 1235

static
device_id_t MACHINE = {
    .data = MACHINE_ID
};

static
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 3-16-1: Shall reply with STOP message if in a stopped state
// Description: Bond a device and complete the STOP/OK sequence.
// Bond a second device. First device sends STOP. Second device
// sends OK, shall respond with STOP.

static
void
req_3_16_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U);

    device_id_t remote = {
        .data = REMOTE_ID
    };

    // bond first remote
    pstop_msg_t resp;

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp, &remote, &MACHINE, 12, 2000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state

    // bond second remote
    device_id_t remote2 = {
        .data = REMOTE_2_ID
    };

    pstop_msg_t resp2;

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp2, &remote2, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);

    // first device sends STOP
    TEST_ASSERT_EQUAL(PSTOP_OK, send_stop(&machine, &resp, &remote, &MACHINE, 13, 2500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // second device sends OK
    TEST_ASSERT_EQUAL(PSTOP_OK, send_ok(&machine, &resp2, &remote2, &MACHINE, 11, 1500));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // still in stopped state
}

void
main_req_3_16_test(void)
{
    UnitySetTestFile("req_3_16_test.c");

    RUN_TEST(req_3_16_test);
}
