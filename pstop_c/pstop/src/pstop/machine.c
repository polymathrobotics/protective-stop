
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stddef.h>

#include "pstop/machine.h"
#include "pstop/constants.h"
#include "pstop/protocol.h"

static
pstop_error_t
add_new_client(pstop_machine_t *machine, const device_id_t *id, pstop_remote_data_t **client)
{
    // no client found, can we add it?
    operator_details_t details = machine->application->operator_details_cb(id);

    if(details.allowed == false) {
        // This ID is not allowed
        *client = NULL;
        return PSTOP_OPERATOR_NOT_ALLOWED;
    }

    // we're allowing this ID, do we have room for it?
    *client = pstop_remote_get_free_remote(&(machine->remotes));

    if(*client == NULL) {
        return PSTOP_OUT_OF_OPERATOR_SPACE;
    }

    (*client)->is_stop_only = details.stop_only;
    (*client)->remote_data.heartbeat_ms = details.heartbeat_ms;

    return PSTOP_OK;
}

static
void
init_new_client(pstop_application_t *application, pstop_remote_data_t *client, const pstop_msg_t *msg)
{
    uint64_t now = application->env.get_time_cb();

    device_id_copy(&(client->remote_data.remote_id), &(msg->id));
    client->remote_data.last_timestamp = now;
    client->remote_data.msg_counter = 0U;
    client->remote_data.last_sent_counter = 0U;
}

static
pstop_error_t
handle_bond_msg(pstop_machine_t *machine, pstop_remote_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    // if we're already bonded, then don't do anything else
    if(client->remote_state == PSTOP_REMOTE_BONDED) {
        resp->message = PSTOP_MESSAGE_BOND;
        return PSTOP_OK;
    }

    client->remote_state = PSTOP_REMOTE_BONDED;

    if(pstop_remote_num_active(&(machine->remotes)) == 1U) {
        // first client connecting then we'll need a stop/start sequence from anyone
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
        machine->robot_state.remote_stop_id = 0U;
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    }

    // create BOND response
    resp->message = PSTOP_MESSAGE_BOND;

    return PSTOP_OK;
}

static
pstop_error_t
handle_ok_msg(pstop_machine_t *machine, pstop_remote_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    if(machine->robot_state.restart_state == ROBOT_RESTART_STATE_NEED_STOP) {
        resp->message = PSTOP_MESSAGE_STOP;
        return PSTOP_OK;
    }

    // this isn't the client that requested the STOP
    if(machine->robot_state.remote_stop_id != client->local_remote_id) {
        if(machine->robot_state.robot_state == ROBOT_STATE_OK) {
            resp->message = PSTOP_MESSAGE_OK;
        }
        else {
            resp->message = PSTOP_MESSAGE_STOP;
        }

        return PSTOP_OK;
    }

    // either this is the client that started the STOP/OK cycle or we're in a
    // normal case where STOP/OK has already finished.
    machine->robot_state.robot_state = ROBOT_STATE_OK;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    //machine->robot_state.remote_stop_id = 0U;

    resp->message = PSTOP_MESSAGE_OK;

    machine->application->status_cb(PSTOP_STATUS_OK);

    return PSTOP_OK;
}

static
pstop_error_t
handle_stop_msg(pstop_machine_t *machine, pstop_remote_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    resp->message = PSTOP_MESSAGE_STOP;

    // if we're waiting for any client to initiate the stop/ok cycle
    if(machine->robot_state.remote_stop_id == 0U) {
        // but don't allow stop/ok cycle from stop-only clients
        if(client->is_stop_only == false) {
            machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
            machine->robot_state.remote_stop_id = client->local_remote_id;
            machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;
        }
    }
    else {
        // another client has taken control of this machine
        // but this new client wants to stop it.
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
        if(client->is_stop_only == false) {
            machine->robot_state.remote_stop_id = client->local_remote_id;
            machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;
        }
        else {
            machine->robot_state.remote_stop_id = 0U;
        }
    }

    machine->application->status_cb(PSTOP_STATUS_STOP);

    return PSTOP_OK;
}

static
pstop_error_t
handle_unbond_msg(pstop_machine_t *machine, pstop_remote_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    resp->message = PSTOP_MESSAGE_UNBOND;

    // if the client unbonding is the same client that needs the stop/ok cycle
    // then we reset the stop/ok cycle
    if(machine->robot_state.remote_stop_id == client->local_remote_id) {
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
        machine->robot_state.remote_stop_id = 0U;
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    }

    pstop_remote_deactivate(client);

    // if this is the last client then stop the robot
    if((pstop_remote_num_active(&(machine->remotes)) == 0U) || (machine->robot_state.robot_state == ROBOT_STATE_STOPPED)) {
        machine_stop_robot(machine);
    }

    return PSTOP_OK;
}

static
pstop_error_t
dispatch_message(pstop_machine_t *machine, pstop_remote_data_t *client , const pstop_msg_t *req, pstop_msg_t *resp)
{
    switch(req->message) {
    case PSTOP_MESSAGE_BOND:
        return handle_bond_msg(machine, client, req, resp);
    case PSTOP_MESSAGE_UNBOND:
        return handle_unbond_msg(machine, client, req, resp);
    case PSTOP_MESSAGE_OK:
        return handle_ok_msg(machine, client, req, resp);
    default:
        break;
    }

    // by default we'll assume any invalid message means stop
    return handle_stop_msg(machine, client, req, resp);
}

static
pstop_error_t
machine_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t *resp)
{
    // validate we have a response message
    if(resp == NULL) {
        return PSTOP_FATAL;
    }

    // validate that the message parameters are correct
    pstop_error_t result = pstop_is_message_valid(req);
    if(result != PSTOP_OK) {
       return result;
    }

    uint64_t now = machine->application->env.get_time_cb();

    // can we find this client?
    pstop_remote_data_t *client = pstop_remote_get(&(machine->remotes), &(req->id));
    if(client == NULL) {
        // if not, are they requesting something other than bond?
        if(req->message != PSTOP_MESSAGE_BOND) {
            // send back UNBOND message
            resp->message = PSTOP_MESSAGE_UNBOND;
            resp->heartbeat_timeout = 0U;
            machine->application->log_message_cb(now, &(req->id), req->message, PSTOP_NOT_BOND_MESSAGE);
            return PSTOP_OK;
        }
        // can we add this new client?
        result = add_new_client(machine, &(req->id), &client);
        if(result != PSTOP_OK) {
            // send back UNBOND message
            resp->message = PSTOP_MESSAGE_UNBOND;
            resp->heartbeat_timeout = 0U;
            machine->application->log_message_cb(now, &(req->id), req->message, result);

            return result;
        }
        // brand new client, let's initialize it
        init_new_client(machine->application, client, req);
    }
    resp->heartbeat_timeout = client->remote_data.heartbeat_ms;

    client->remote_data.last_timestamp = now;

    result = dispatch_message(machine, client, req, resp);

    if(client->last_message != req->message) {
        client->last_message = req->message;
        machine->application->log_message_cb(now, &(req->id), req->message, result);
    }

    return result;
}

static
void
machine_notify_hal(pstop_machine_t *machine, pstop_status_message_t status)
{
    machine->application->status_cb(status);
}

static
pstop_error_t
machine_check_heartbeats(pstop_machine_t *machine)
{
    uint64_t now = machine->application->env.get_time_cb();

    int needsStop = 0;

    // check all clients in case multiple clients are failing
    for(uint16_t i = 0U; i < machine->remotes.max_remotes; ++i) {
        pstop_remote_data_t *client = &(machine->remotes.remotes[i]);
        if(client->remote_state == PSTOP_REMOTE_UNKNOWN) {
            continue;
        }

        // if for some reason now is in the past compared to the last heart beat
        // then there's no need to check if we've heard from this client
        if(now <= client->remote_data.last_timestamp) {
            continue;
        }

        uint64_t diff = now - client->remote_data.last_timestamp;

        // if we're still within the heartbeat timeout then this client is still
        // good.
        if(diff <= client->remote_data.heartbeat_ms) {
            continue;
        }

        // problem! this client hasn't talked to us in a while
        // if we're still within the window of missed heartbeats then we're OK

        client->missed_heartbeats_counter = (uint16_t)(diff / client->remote_data.heartbeat_ms);
        if(client->missed_heartbeats_counter >= machine->application->app_config.max_missed_heartbeats) {
            // trigger a stop!
            client->remote_state = PSTOP_REMOTE_UNKNOWN;
            machine->application->log_message_cb(now, &(client->remote_data.remote_id), PSTOP_MESSAGE_UNKNOWN, PSTOP_MISSED_HEARTBEATS);
            needsStop = 1;
        }
    }

    if(needsStop != 0) {
        machine_stop_robot(machine);
        return PSTOP_MISSED_HEARTBEATS;
    }

    if(machine->robot_state.robot_state == ROBOT_STATE_STOPPED) {
        machine_notify_hal(machine, PSTOP_STATUS_STOP);
    }
    else {
        machine_notify_hal(machine, PSTOP_STATUS_OK);

    }

    return PSTOP_OK;
}

void
machine_init(pstop_machine_t *machine, pstop_application_t *app, pstop_remote_data_t *remotes, uint16_t max_remotes)
{
    machine->handle_machine_message_cb = machine_handle_message;
    machine->handle_protocol_message_cb = protocol_handle_message;
    machine->check_heartbeats_cb = machine_check_heartbeats;

    machine->application = app;
    machine->remotes.remotes = remotes;
    machine->remotes.max_remotes = max_remotes;

    machine->robot_state.remote_stop_id = 0U;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    machine->robot_state.robot_state = ROBOT_STATE_STOPPED;

    pstop_remotes_init(&(machine->remotes));
}

void
machine_stop_robot(pstop_machine_t *machine)
{
    machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
    machine->robot_state.remote_stop_id = 0U;

    machine_notify_hal(machine, PSTOP_STATUS_STOP);
}
