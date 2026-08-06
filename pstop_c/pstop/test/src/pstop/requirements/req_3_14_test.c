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

// 3-14-1: Shall reply with UNBOND message after UNBOND request.
// Description: Send an UNBOND message, shall reply with an UNBOND response.

static
void
req_3_14_1_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U); // this remote is not allowed

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

    // stop then unbond
    req.message = PSTOP_MESSAGE_UNBOND;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
}

static
void
req_3_14_2_test(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);
    configure_app_defaults(&pstop_app, &MACHINE, 0U, 0U, 200U);

    reset_robot_status();

    set_time(1000);
    set_operator_allowed(true, false, 1000U); // this remote is not allowed

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

    req.message = PSTOP_MESSAGE_OK;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);

    req.message = PSTOP_MESSAGE_UNBOND;
    req.counter++;
    req.stamp += 500;
    req.received_counter = resp.counter;
    req.received_stamp = resp.stamp;
    set_time(get_time() + 500);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine_process_message(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
}

void
main_req_3_14_test(void)
{
    UnitySetTestFile("req_3_14_test.c");

    RUN_TEST(req_3_14_1_test);
    RUN_TEST(req_3_14_2_test);
}
