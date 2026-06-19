
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/machine.h"

#include <unity/unity.h>

#include "pstop/time.h"

static uint64_t current_time;

static
uint64_t
get_time(void)
{
    return current_time++;
}

static operator_details_t details;

static
operator_details_t
is_operator_allowed(const device_id_t *id)
{
    return details;
}

static pstop_status_message_t last_status = PSTOP_STATUS_OK;
static int robot_status_counter = 0;

static
void
robot_status(pstop_status_message_t status)
{
    last_status = status;

    robot_status_counter++;
}

static
void
log_error(uint64_t timestamp, const device_id_t *client, uint8_t message, pstop_error_t error)
{

}

static
pstop_application_t pstop_app = {
    .env.get_time_cb = get_time,
    .machine_device_id.data = 1236,
    .operator_details_cb = is_operator_allowed,
    .status_cb = robot_status,
    .log_message_cb = log_error,
    .app_config.max_lost_messages = 1U,
    .app_config.max_missed_heartbeats = 1U
};

#define MAX_CLIENTS 2U

static pstop_client_data_t pstop_clients[MAX_CLIENTS];

static
void
init_client(device_id_t *id1, pstop_msg_t *msg, uint32_t id)
{
    id1->data = id;
    device_id_copy(&(msg->id), id1);
}

static
void
test_new_client_operator_not_allowed_req_3_11(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t msg;
    pstop_message_init(&msg);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    msg.message = PSTOP_MESSAGE_BOND;

    details.allowed = false;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    TEST_ASSERT_EQUAL(PSTOP_OPERATOR_NOT_ALLOWED, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(0U, resp.heartbeat_timeout);
}

static
void
test_new_client_no_more_clients_req_3_10(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    device_id_t id;
    pstop_msg_t msg;

    // bond first client
    {
        init_client(&id, &msg, 1234);
        msg.message = PSTOP_MESSAGE_BOND;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
        TEST_ASSERT_EQUAL(500U, resp.heartbeat_timeout);
    }
    // bond second client
    {
        init_client(&id, &msg, 1235);
        msg.message = PSTOP_MESSAGE_BOND;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
        TEST_ASSERT_EQUAL(500U, resp.heartbeat_timeout);
    }

    // no more room for this client
    {
        init_client(&id, &msg, 1236);
        msg.message = PSTOP_MESSAGE_BOND;
        TEST_ASSERT_EQUAL(PSTOP_OUT_OF_OPERATOR_SPACE, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
        TEST_ASSERT_EQUAL(0U, resp.heartbeat_timeout);
    }
}

static
void
test_handle_mssage_null_resp(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t msg;
    pstop_message_init(&msg);

    TEST_ASSERT_EQUAL(PSTOP_FATAL, machine.handle_machine_message_cb(&machine, &msg, NULL));
}

static
void
test_handle_mssage_invalid_message(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    pstop_msg_t msg;
    pstop_message_init(&msg);
    msg.message = PSTOP_MESSAGE_UNKNOWN;

    pstop_msg_t resp;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_TYPE_INVALID, machine.handle_machine_message_cb(&machine, &msg, &resp));
}

static
void
test_new_client_operator_allowed(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    id.data = 1234;

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(100U, pstop_clients[0].client_data.last_timestamp);
    TEST_ASSERT_EQUAL(500U, pstop_clients[0].client_data.heartbeat_ms);
    TEST_ASSERT_EQUAL(500U, resp.heartbeat_timeout);
}

static
void
test_bond_req_bond_resp_req_3_12(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));

    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    pstop_client_data_t *client = pstop_client_get(&(machine.pstops), &id);

    TEST_ASSERT_NOT_NULL(client);
}

static
void
test_ok_not_bonded_req_3_03(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    // OK => UNBOND
    device_id_t id;
    id.data = 1234;

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_OK;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    // client isn't bonded so can only send BOND messages
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));

    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
}

static
void
test_bond_unbond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_NOT_NULL(pstop_client_get(&(machine.pstops), &id));

    msg.message = PSTOP_MESSAGE_UNBOND;
    pstop_message_init(&resp);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_NULL(pstop_client_get(&(machine.pstops), &id));
    TEST_ASSERT_EQUAL(0U, pstop_client_num_active(&(machine.pstops)));
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);
}

// send bond then ok. Since no stop/ok cycle has been
// completed, replies with STOP
static
void
test_bond_ok(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    msg.message = PSTOP_MESSAGE_OK;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
}

static
void
test_bond_bond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;

    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_NOT_NULL(pstop_client_get(&(machine.pstops), &id));

    // already bonded, don't rebond
    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
}

static
void
test_bond_ok_stop_req_3_17(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    id.data = 1234;

    pstop_msg_t msg;
    msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&(msg.id), &id);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(0, robot_status_counter);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    for(int i = 0; i < 10; ++i) {
        msg.message = PSTOP_MESSAGE_OK;
        TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
        TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
        TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);
        TEST_ASSERT_EQUAL(i + 2, robot_status_counter);
    }

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);
    TEST_ASSERT_EQUAL(12, robot_status_counter);
}

static
void
test_bond_stop_ok_req_3_04(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    last_status = PSTOP_STATUS_STOP;
    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    TEST_ASSERT_EQUAL(0, robot_status_counter);

    // haven't received the stop message, so still in stopped state
    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);
}

/*
 * Bond a client, then send stop. Verify that the stop state
 * is not attached to that client. Send another stop. Should be the same.
 * Then send OK to signal that the operator is ready.
 */
static
void
test_bond_stop_stop_ok_req_3_08(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, machine.robot_state.restart_state);

    TEST_ASSERT_EQUAL(0, robot_status_counter);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);

    // send STOP again.
    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);

    // And one more time.
    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);

    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, last_status);

    TEST_ASSERT_EQUAL(1U, pstop_client_num_active(&(machine.pstops)));

    // last client is unbonding
    msg.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, last_status);

    TEST_ASSERT_EQUAL(0U, pstop_client_num_active(&(machine.pstops)));
}

/**
 * Send STOP message for an unbonded client.
 * Should respond with UNBOND
 */
static
void
test_unbonded_stop(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);

    TEST_ASSERT_EQUAL(0, robot_status_counter);
}

static
void
test_heartbeat_timeout(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);
    TEST_ASSERT_EQUAL(500U, resp.heartbeat_timeout);
}

static
void
test_bond_stop_ok_stop_only_operator(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id;
    pstop_msg_t msg;
    init_client(&id, &msg, 1234);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 1;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    msg.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(0, robot_status_counter);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // stop-only operator can't transition to OK state
    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    msg.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    msg.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
}

static
void
test_2_clients(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // second node sends OK, should reply stop
    msg2.message = PSTOP_MESSAGE_OK;

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // first node sends OK, should reply OK
    msg1.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);

    // now unbond first client
    msg1.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(0U, machine.robot_state.client_stop_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);
}

static
void
test_2_clients_unbond_before_ok(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // first node unbonds
    msg1.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);

    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(0U, machine.robot_state.client_stop_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);
}

static
void
test_2_clients_unbond_second(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);

    // first node sends OK, should reply OK. First client is in control
    msg1.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);

    // now unbond second client. First client is still in control
    msg2.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_OK, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
}

static
void
test_2_clients_stop_unbond(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);

    // then OK. It's in control
    msg1.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);

    // then first node sends unbond
    msg1.message = PSTOP_MESSAGE_UNBOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_UNBOND, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_NEED_STOP, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(0U, machine.robot_state.client_stop_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(1U, pstop_client_num_active(&(machine.pstops)));
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    // second node sends STOP, then OK
    msg2.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[1].local_client_id);

    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);
}

static
void
test_2_clients_stop_ok(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);

    // second node sends OK. Should respond with stop
    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    // first node sends OK. It's in control
    msg1.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);

    // second node sends OK. Should respond with OK since first node is in control
    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);
}

static
void
test_2_clients_stop_only_stop(void)
{
    pstop_machine_t machine;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    device_id_t id1, id2;
    pstop_msg_t msg1, msg2;

    init_client(&id1, &msg1, 1234);
    init_client(&id2, &msg2, 1235);

    pstop_msg_t resp;
    pstop_message_init(&resp);

    details.allowed = true;
    details.stop_only = 0;
    details.heartbeat_ms = 500U;
    current_time = 100U;

    robot_status_counter = 0;

    // bond both nodes
    msg1.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // stop-only pstop
    details.stop_only = 1;
    msg2.message = PSTOP_MESSAGE_BOND;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_BOND, resp.message);

    // first node sends stop
    msg1.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(ROBOT_RESTART_STATE_STOP_RECEIVED, machine.robot_state.restart_state);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);

    // first node sends OK. it's in control
    msg1.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg1, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_OK, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, pstop_clients[0].local_client_id);
    TEST_ASSERT_EQUAL(ROBOT_STATE_OK, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_OK, last_status);

    // second node sends STOP. But it's a stop-only node
    msg2.message = PSTOP_MESSAGE_STOP;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, 0U);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);

    // second node sends OK. Should respond with STOP since stop-only can't switch to OK
    msg2.message = PSTOP_MESSAGE_OK;
    TEST_ASSERT_EQUAL(PSTOP_OK, machine.handle_machine_message_cb(&machine, &msg2, &resp));
    TEST_ASSERT_EQUAL(PSTOP_MESSAGE_STOP, resp.message);
    TEST_ASSERT_EQUAL(machine.robot_state.client_stop_id, 0U);
    TEST_ASSERT_EQUAL(ROBOT_STATE_STOPPED, machine.robot_state.robot_state);
    TEST_ASSERT_EQUAL(PSTOP_STATUS_STOP, last_status);
}

void
main_machine_test(void)
{
    UnitySetTestFile("machine_test.c");

    RUN_TEST(test_handle_mssage_null_resp);
    RUN_TEST(test_new_client_operator_not_allowed_req_3_11);
    RUN_TEST(test_new_client_operator_allowed);
    RUN_TEST(test_new_client_no_more_clients_req_3_10);
    RUN_TEST(test_handle_mssage_invalid_message);
    RUN_TEST(test_heartbeat_timeout);
    RUN_TEST(test_2_clients_unbond_before_ok);
    RUN_TEST(test_bond_req_bond_resp_req_3_12);
    RUN_TEST(test_ok_not_bonded_req_3_03);
    RUN_TEST(test_bond_unbond);
    RUN_TEST(test_bond_bond);
    RUN_TEST(test_bond_ok_stop_req_3_17);
    RUN_TEST(test_bond_stop_stop_ok_req_3_08);
    RUN_TEST(test_unbonded_stop);
    RUN_TEST(test_2_clients);
    RUN_TEST(test_2_clients_unbond_second);
    RUN_TEST(test_2_clients_stop_unbond);
    RUN_TEST(test_bond_ok);
    RUN_TEST(test_bond_stop_ok_req_3_04);
    RUN_TEST(test_bond_stop_ok_stop_only_operator);
    RUN_TEST(test_2_clients_stop_ok);
    RUN_TEST(test_2_clients_stop_only_stop);
}
