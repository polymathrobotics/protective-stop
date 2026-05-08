// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/test_utils.h"

static uint64_t current_time;

static
uint64_t
get_time(void)
{
    return current_time++;
}

static int operator_allowed_flag;

static
int
is_operator_allowed(const device_id_t *id)
{
    return operator_allowed_flag;
}

static pstop_status_message_t lastStatus = PSTOP_STATUS_OK;
static int robot_status_counter = 0;

static
int
robot_status(pstop_status_message_t status)
{
    lastStatus = status;

    robot_status_counter++;

    return 0;
}

static
void
log_error(pstop_error_t error, const char *message)
{

}

static
pstop_application_t pstop_app = {
    .env.get_time_cb = get_time,
    .machine_device_id.data = "testing",
    .operator_allowed_cb = is_operator_allowed,
    .status_cb = robot_status,
    .log_message_cb = log_error,
    .app_config.default_timeout_ms = 60U,
    .app_config.max_lost_messages = 1U,
    .app_config.max_missed_heartbeats = 1U
};

#define MAX_CLIENTS 2U

static pstop_client_data_t pstop_clients[MAX_CLIENTS];

static
void
test_protocol_invalid_receiver_id(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t req;
    pstop_msg_t resp;
    pstop_msg_t *handle = &resp;
    device_id_set_str(&req.receiver_id, "incorrect");
    TEST_ASSERT_EQUAL(PSTOP_ERROR_INVALID_ID, machine.handle_protocol_message_cb(&machine, &req, &handle));
}

static
void
test_protocol_operator_not_allowed(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 0;

    pstop_msg_t req;
    pstop_msg_t resp;
    pstop_msg_t *handle = &resp;
    device_id_set_str(&req.receiver_id, "testing");
    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine.handle_protocol_message_cb(&machine, &req, &handle));
}

static
void
test_protocol_bond_request(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    device_id_set_str(&req.id, "client1");

    pstop_msg_t resp;
    pstop_message_init(&resp);
    pstop_msg_t *handle = &resp;
    device_id_set_str(&req.receiver_id, "testing");
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &handle));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(1U, resp.counter);
    TEST_ASSERT_EQUAL(10, resp.received_counter);
    TEST_ASSERT_EQUAL(100, resp.received_stamp);
}

static
void
test_protocol_invalid_counter(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    device_id_set_str(&req.id, "client1");

    pstop_msg_t resp;
    pstop_message_init(&resp);
    pstop_msg_t *handle = &resp;
    device_id_set_str(&req.receiver_id, "testing");

    // setup client with BOND message
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &handle));
    TEST_ASSERT_EQUAL(1U, resp.counter);

    req.message = PSTOP_MESSAGE_OK;
    req.counter = 9;
    TEST_ASSERT_EQUAL(PSTOP_MSG_OUT_OF_ORDER, machine.handle_protocol_message_cb(&machine, &req, &handle));
}

void
main_protocol_test(void)
{
    UnitySetTestFile("protocol_test.c");

    RUN_TEST(test_protocol_invalid_receiver_id);
    RUN_TEST(test_protocol_operator_not_allowed);
    RUN_TEST(test_protocol_bond_request);
    RUN_TEST(test_protocol_invalid_counter);
}
