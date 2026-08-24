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
    set_operator_allowed(true, false, 1000U); // this remote is not allowed

    // bond first remote
    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 1000;
    req.id.data = REMOTE_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;
    req.checksum = 10U;
    req.calculated_checksum = 10U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    req.message = PSTOP_MESSAGE_STOP;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // stop then OK
    req.message = PSTOP_MESSAGE_OK;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, get_last_status()); // in OK state

    // bond second remote
    pstop_msg_t req2;
    pstop_message_init(&req2);
    req2.message = PSTOP_MESSAGE_BOND;
    req2.counter = 10;
    req2.stamp = 1000;
    req2.id.data = REMOTE_2_ID;
    req2.receiver_id.data = MACHINE_ID;
    req2.received_counter = 0U;
    req2.received_stamp = 0U;
    req2.checksum = 10U;
    req2.calculated_checksum = 10U;

    pstop_msg_t resp2;
    pstop_message_init(&resp2);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req2, &resp2));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp2.message);

    // first device sends STOP
    req.message = PSTOP_MESSAGE_STOP;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // in a stopped state

    // second device sends OK
    req2.message = PSTOP_MESSAGE_OK;
    req2.counter++;
    req2.stamp += 500;
    req2.received_counter = resp2.counter;
    req2.received_stamp = resp2.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req2, &resp2));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp2.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, get_last_status()); // still in stopped state
}

void
main_req_3_16_test(void)
{
    UnitySetTestFile("req_3_16_test.c");

    RUN_TEST(req_3_16_test);
}
