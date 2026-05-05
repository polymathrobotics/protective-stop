
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>

#include <stddef.h>

#include "pstop/machine.h"
#include "pstop/pstop_client.h"
#include "pstop/constants.h"
#include "pstop/protocol.h"

static
pstop_error_t
add_new_client(pstop_machine_t *machine, const device_id_t *id, pstop_client_data_t **client)
{
    // no client found, can we add it?
    if(!machine->application->operator_allowed_cb(id)) {
        // This ID is not allowed
        *client = NULL;
        return PSTOP_OPERATOR_NOT_ALLOWED;
    }

    // we're allowing this ID, do we have room for it?
    *client = pstop_client_get_free_client(&(machine->pstops));

    if(*client == NULL) {
        return PSTOP_OUT_OF_OPERATOR_SPACE;
    }

    return PSTOP_OK;
}

static
void
init_new_client(pstop_application_t *application, pstop_client_data_t *client, const pstop_msg_t *msg)
{
    uint64_t now = application->env.get_time_cb();

    device_id_copy(&(client->client_id), &(msg->id));
    client->last_timestamp = now;
    client->heartbeat_ms = application->app_config.default_timeout_ms;
    client->msg_counter = 1U;
    client->last_counter = 0U;

    if(now >= msg->stamp) {
        client->clock_drift = (int64_t)(now - msg->stamp);
    }
    else {
        client->clock_drift = -(int64_t)(msg->stamp - now);
    }
}

static
pstop_error_t
handle_bond_msg(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    // if we're already bonded, then don't do anything else
    if(client->client_state == PSTOP_CLIENT_BONDED) {
        resp->message = PSTOP_MESSAGE_BOND;
        return PSTOP_OK;
    }

    client->client_state = PSTOP_CLIENT_BONDED;

    if(pstop_client_num_active(&(machine->pstops)) == 1U) {
        // first client connecting then we'll need a stop/start sequence from anyone
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
        machine->robot_state.client_stop_id = 0U;
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    }

    // create BOND response
    resp->message = PSTOP_MESSAGE_BOND;

    return PSTOP_OK;
}

static
pstop_error_t
handle_ok_msg(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    if(client->client_state != PSTOP_CLIENT_BONDED) {
        resp->message = PSTOP_MESSAGE_UNBOND;
        return PSTOP_OK;
    }

    if(machine->robot_state.restart_state == ROBOT_RESTART_STATE_NEED_STOP) {
        resp->message = PSTOP_MESSAGE_STOP;
        return PSTOP_OK;
    }

    // this isn't the client that requested the STOP
    if(machine->robot_state.client_stop_id != 0U) {
        if(machine->robot_state.client_stop_id != client->local_client_id) {
            resp->message = PSTOP_MESSAGE_STOP;
            return PSTOP_OK;
        }
    }
    // either this is the client that started the STOP/OK cycle or we're in a
    // normal case where STOP/OK has already finished.
    machine->robot_state.robot_state = ROBOT_STATE_OK;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    machine->robot_state.client_stop_id = 0U;

    resp->message = PSTOP_MESSAGE_OK;

    machine->application->status_cb(PSTOP_STATUS_OK);

    return PSTOP_OK;
}

static
void
handle_need_stop(pstop_machine_t *machine, pstop_client_data_t *client, pstop_msg_t *resp)
{
    // if another node already started the stop/ok cycle then we'll expect that
    // node to send us the OK.
    if(machine->robot_state.client_stop_id == client->local_client_id) {
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;
    }
    else {
        if(machine->robot_state.client_stop_id == 0U) {
            // we have a new client that sent us a STOP message.
            machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
            machine->robot_state.client_stop_id = client->local_client_id;
            machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;
        }
    }
}

static
pstop_error_t
handle_stop_msg(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    if(client->client_state != PSTOP_CLIENT_BONDED) {
        resp->message = PSTOP_MESSAGE_UNBOND;
        return PSTOP_OK;
    }

    resp->message = PSTOP_MESSAGE_STOP;

    // do we need a stop/ok cycle?
    if(machine->robot_state.restart_state == ROBOT_RESTART_STATE_NEED_STOP) {
        handle_need_stop(machine, client, resp);
    }

    if(machine->robot_state.client_stop_id == 0U) {
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
        machine->robot_state.client_stop_id = client->local_client_id;
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;
    }

    machine->application->status_cb(PSTOP_STATUS_STOP);

    return PSTOP_OK;
}

static
pstop_error_t
handle_unbond_msg(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    resp->message = PSTOP_MESSAGE_UNBOND;

    // if the client unbonding is the same client that needs the stop/ok cycle
    // then we reset the stop/ok cycle
    if(machine->robot_state.client_stop_id == client->local_client_id) {
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
        machine->robot_state.client_stop_id = 0U;
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    }

    pstop_client_deactivate(client);

    // if this is the last client then stop the robot
    if((pstop_client_num_active(&(machine->pstops)) == 0U) || (machine->robot_state.robot_state == ROBOT_STATE_STOPPED)) {
        machine_stop_robot(machine);
    }

    return PSTOP_OK;
}

static
pstop_error_t
machine_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp)
{
    // validate we have a response message
    if(resp == NULL) {
        return PSTOP_FATAL;
    }
    if(*resp == NULL) {
        return PSTOP_FATAL;
    }

    // validate that the message parameters are correct
    pstop_error_t result = pstop_is_message_valid(req);
    if(result != PSTOP_OK) {
       return result;
    }

    (*resp)->heartbeat_timeout = machine->application->app_config.default_timeout_ms;

    // can we find this client?
    pstop_client_data_t *client = pstop_client_get(&(machine->pstops), &(req->id));
    if(client == NULL) {
        // if not, are they requesting something other than bond?
        if(req->message != PSTOP_MESSAGE_BOND) {
            // send back UNBOND message
            (*resp)->message = PSTOP_MESSAGE_UNBOND;
            return PSTOP_OK;
        }
        // can we add this new client?
        result = add_new_client(machine, &(req->id), &client);
        if(result != PSTOP_OK) {
            // send back UNBOND message
            (*resp)->message = PSTOP_MESSAGE_UNBOND;

            return result;
        }
        // brand new client, let's initialize it
        init_new_client(machine->application, client, req);
    }

    uint64_t now = machine->application->env.get_time_cb();
    client->last_timestamp = now;

    switch(req->message) {
    case PSTOP_MESSAGE_BOND:
        return handle_bond_msg(machine, client, req, *resp);
    case PSTOP_MESSAGE_UNBOND:
        return handle_unbond_msg(machine, client, req, *resp);
    case PSTOP_MESSAGE_OK:
        return handle_ok_msg(machine, client, req, *resp);
    default:
        break;
    }

    // by default we'll assume any invalid message means stop
    return handle_stop_msg(machine, client, req, *resp);
}

static
pstop_error_t
machine_check_heartbeats(pstop_machine_t *machine)
{
    uint64_t now = machine->application->env.get_time_cb();

    int needsStop = 0;

    // check all clients in case multiple clients are failing
    for(uint16_t i = 0U; i < machine->pstops.max_clients; ++i) {
        pstop_client_data_t *client = &(machine->pstops.clients[i]);
        if(client->client_state == PSTOP_CLIENT_UNKNOWN) {
            continue;
        }

        // if for some reason now is in the past compared to the last heart beat
        // then there's no need to check if we've heard from this client
        if(now <= client->last_timestamp) {
            continue;
        }

        uint64_t diff = now - client->last_timestamp;

        // if we're still within the heartbeat timeout then this client is still
        // good.
        if(diff <= client->heartbeat_ms) {
            continue;
        }

        // problem! this client hasn't talked to us in a while
        // if we're still within the window of missed heartbeats then we're OK

        client->missed_heartbeats_counter = (uint16_t)(diff / client->heartbeat_ms);
        if(client->missed_heartbeats_counter >= machine->application->app_config.max_missed_heartbeats) {
            // trigger a stop!
            client->client_state = PSTOP_CLIENT_UNKNOWN;
            needsStop = 1;
        }
    }

    if(needsStop != 0) {
        machine_stop_robot(machine);
        return PSTOP_MISSED_HEARTBEATS;
    }

    return PSTOP_OK;
}

void
machine_init(pstop_machine_t *machine, pstop_application_t *app, pstop_client_data_t *clients, uint16_t max_clients)
{
    machine->handle_machine_message_cb = machine_handle_message;
    machine->handle_protocol_message_cb = protocol_handle_message;
    machine->check_heartbeats_cb = machine_check_heartbeats;

    machine->application = app;
    machine->pstops.clients = clients;
    machine->pstops.max_clients = max_clients;

    machine->robot_state.client_stop_id = 0U;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    machine->robot_state.robot_state = ROBOT_STATE_OK;

    pstop_clients_init(&(machine->pstops));
}

void
machine_stop_robot(pstop_machine_t *machine)
{
    machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
    machine->robot_state.client_stop_id = 0U;

    if(machine->application->status_cb != NULL) {
        machine->application->status_cb(PSTOP_STATUS_STOP);
    }
}
