// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"
#include "pstop/requirements/test_utils.h"

#define MACHINE_ID 1236
#define REMOTE_ID 1234

static
device_id_t MACHINE = {
    .data = MACHINE_ID
};

static
pstop_application_t pstop_app;

#define MAX_CLIENTS 2U

static pstop_remote_data_t pstop_clients[MAX_CLIENTS];

// 3-10-1: Shall reject a bonding request if no more operators are
//         allowed to bond and will reply with an UNBOND message.
// Configure library to allow 2 remotes. Do a successful bonding on
// both remotes. Send BOND message from 3rd remote. Should respond with UNBOND message.

static
void
req_3_10_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U); // Allow this device ID

    device_id_t remote = {
        .data = REMOTE_ID
    };

    for(unsigned i = 0; i < MAX_CLIENTS; ++i) {
        remote.data = REMOTE_ID + i;

        pstop_msg_t resp;

        TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote, &MACHINE, 10, 1000));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

        TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());
    }

    // BOND third client. Should be rejected
    remote.data = REMOTE_ID + MAX_CLIENTS;

    pstop_msg_t resp2;

    set_operator_allowed(true, false, 1000U); // Allow this device ID
    TEST_ASSERT_EQUAL(PSTOP_OUT_OF_OPERATOR_SPACE, send_bond(&machine, &resp2, &remote, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp2.message);

    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status());
}

void
main_req_3_10_test(void)
{
    UnitySetTestFile("req_3_10_test.c");

    RUN_TEST(req_3_10_test);
}
