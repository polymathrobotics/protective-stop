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

// 3-01-1: Shall support a configurable heartbeat per remote
// Description: Configure the library to allow two remotes. Configure
// first remote to have a heartbeat timeout of 5 seconds and the second
// remote to have a heartbeat timeout of 10 seconds. Send a bonding request
// for the first remote and validate that the heartbeat attribute of the
// message is 5 seconds. Do the same for the second remote to validate
// that the heartbeat is 10 seconds.

static
void
req_3_01_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 1000U);

    device_id_t remote_id = {
        .data = REMOTE_ID
    };

    pstop_msg_t resp;

    set_operator_allowed(true, false, 5000U); // 5 seconds

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    set_operator_allowed(true, false, 10000U); // 10 seconds

    device_id_t remote_id_2 = {
        .data = REMOTE_ID + 1
    };

    TEST_ASSERT_EQUAL(PSTOP_OK, send_bond(&machine, &resp, &remote_id_2, &MACHINE, 10, 1000));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // validate first remote heartbeat time
    const protocol_data_t *remote = machine_get_protocol_data(&machine, &remote_id);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(5000U, remote->heartbeat_ms);

    // validate second remote heartbeat time
    remote = machine_get_protocol_data(&machine, &remote_id_2);
    TEST_ASSERT_NOT_NULL(remote);
    TEST_ASSERT_EQUAL(10000U, remote->heartbeat_ms);
}

void
main_req_3_01_test(void)
{
    UnitySetTestFile("req_3_01_test.c");

    RUN_TEST(req_3_01_test);
}
