// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/protocol.h"

#include <unity/unity.h>

#include "pstop/checksum.h"

static uint64_t current_time;

static
uint64_t
get_time(void)
{
    return current_time++;
}

static int operator_allowed_flag;

static
operator_details_t
is_operator_allowed(const device_id_t *id)
{
    operator_details_t details;

    details.allowed = operator_allowed_flag;
    details.stop_only = 1;
    details.heartbeat_ms = 500U;

    return details;
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
log_error(uint64_t timestamp, const device_id_t *client, uint8_t message, pstop_error_t error)
{

}

#define MACHINE_ID 1236
#define PSTOP_ID 1234

static
pstop_application_t pstop_app = {
    .env.get_time_cb = get_time,
    .machine_device_id.data = MACHINE_ID,
    .operator_details_cb = is_operator_allowed,
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
test_protocol_invalid_checksum(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t req;
    pstop_msg_t resp;
    pstop_message_init(&req);
    pstop_message_init(&resp);
    req.checksum = 10U;

    TEST_ASSERT_EQUAL(PSTOP_MSG_INVALID_CHECKSUM, machine.handle_protocol_message_cb(&machine, &req, &resp));
}

static
void
test_protocol_invalid_receiver_id(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t req;
    pstop_msg_t resp;
    pstop_message_init(&req);
    pstop_message_init(&resp);
    req.receiver_id.data = 4567;

    TEST_ASSERT_EQUAL(PSTOP_ERROR_INVALID_ID, machine.handle_protocol_message_cb(&machine, &req, &resp));
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
    pstop_message_init(&req);
    pstop_message_init(&resp);
    req.receiver_id.data = MACHINE_ID;

    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine.handle_protocol_message_cb(&machine, &req, &resp));
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
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(1U, resp.counter);
    TEST_ASSERT_EQUAL(10, resp.received_counter);
    TEST_ASSERT_EQUAL(100, resp.received_stamp);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&req.id, &resp.receiver_id));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&req.receiver_id, &resp.id));
}

static
void
test_protocol_bond_then_unbond(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;
    current_time = 10;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(1U, resp.counter);
    TEST_ASSERT_EQUAL(10, resp.received_counter);
    TEST_ASSERT_EQUAL(100, resp.received_stamp);
    TEST_ASSERT_EQUAL(12, resp.stamp);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&req.id, &resp.receiver_id));
    TEST_ASSERT_EQUAL(0, device_id_cmp(&req.receiver_id, &resp.id));

    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_UNBOND;
    req.counter = 11;
    req.stamp = 110;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 1;
    req.received_stamp = 12;
    pstop_message_init(&resp);
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
}

static
void
test_protocol_invalid_message(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_UNKNOWN;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;
    req.received_counter = 0U;
    req.received_stamp = 0U;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_TYPE_INVALID, machine.handle_protocol_message_cb(&machine, &req, &resp));
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
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // setup client with BOND message
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(1U, resp.counter);

    req.message = PSTOP_MESSAGE_OK;
    req.counter = 9;
    TEST_ASSERT_EQUAL(PSTOP_MSG_OUT_OF_ORDER, machine.handle_protocol_message_cb(&machine, &req, &resp));
}

static
void
test_protocol_bond_invalid_timestamp(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // setup client with BOND message
    current_time = 100;// move clock forward to 100. Next message of 90 will be in the past.
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(1U, resp.counter);

    req.message = PSTOP_MESSAGE_OK;
    req.counter = 11;
    req.stamp = 90;
    TEST_ASSERT_EQUAL(PSTOP_MSG_OUT_OF_ORDER, machine.handle_protocol_message_cb(&machine, &req, &resp));
}

static
void
test_protocol_bond_missed_too_many_messages(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // setup client with BOND message
    current_time = 100;// move clock forward to 100. Next message of 90 will be in the past.
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(1U, resp.counter);

    req.message = PSTOP_MESSAGE_OK;
    req.counter = 12; // missed message 11
    req.stamp = 110;
    TEST_ASSERT_EQUAL(PSTOP_MSG_LOST, machine.handle_protocol_message_cb(&machine, &req, &resp));
}

static
void
test_protocol_bond_invalid_echo_counter(void)
{
    pstop_machine_t machine;
    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    operator_allowed_flag = 1;

    pstop_msg_t req;
    pstop_message_init(&req);
    req.message = PSTOP_MESSAGE_BOND;
    req.counter = 10;
    req.stamp = 100;
    req.id.data = PSTOP_ID;
    req.receiver_id.data = MACHINE_ID;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    // setup client with BOND message
    current_time = 100;// move clock forward to 100. Next message of 90 will be in the past.
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_protocol_message_cb(&machine, &req, &resp));
    TEST_ASSERT_EQUAL(1U, resp.counter);

    req.message = PSTOP_MESSAGE_OK;
    req.counter = 11; // missed message 11
    req.stamp = 110;
    req.received_counter = 50;
    TEST_ASSERT_EQUAL(PSTOP_MSG_LOST, machine.handle_protocol_message_cb(&machine, &req, &resp));
}

void
main_protocol_test(void)
{
    UnitySetTestFile("protocol_test.c");

    RUN_TEST(test_protocol_invalid_checksum);
    RUN_TEST(test_protocol_invalid_receiver_id);
    RUN_TEST(test_protocol_operator_not_allowed);
    RUN_TEST(test_protocol_bond_request);
    RUN_TEST(test_protocol_bond_then_unbond);
    RUN_TEST(test_protocol_invalid_message);
    RUN_TEST(test_protocol_invalid_counter);
    RUN_TEST(test_protocol_bond_invalid_timestamp);
    RUN_TEST(test_protocol_bond_missed_too_many_messages);
    RUN_TEST(test_protocol_bond_invalid_echo_counter);
}
