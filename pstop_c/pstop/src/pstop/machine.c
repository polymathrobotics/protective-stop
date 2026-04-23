
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
    if(!machine->application.operator_allowed_cb(id)) {
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
    uint64_t now = application->get_time_cb();

    device_id_copy(&(client->client_id), &(msg->id));
    client->last_timestamp = now;
    client->last_received_heartbeat = 0U;
    client->heartbeat_ms = application->default_timeout_ms;
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
    // brand new client, let's initialize it
    init_new_client(&(machine->application), client, msg);

    client->client_state = PSTOP_CLIENT_BONDED;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;

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
    if((machine->robot_state.client_stop_id != 0U) && (machine->robot_state.client_stop_id != client->local_client_id)) {
        resp->message = PSTOP_MESSAGE_STOP;
        return PSTOP_OK;
    }

    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    machine->robot_state.client_stop_id = 0U;

    resp->message = PSTOP_MESSAGE_OK;

    machine->application.status_cb(PSTOP_STATUS_OK);

    return PSTOP_OK;
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

    machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
    machine->robot_state.client_stop_id = client->local_client_id;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_STOP_RECEIVED;

    // only if this client can send stop. Might need to go stop -> ok first
    machine->application.status_cb(PSTOP_STATUS_STOP);

    return PSTOP_OK;
}

static
pstop_error_t
handle_unbond_msg(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *msg, pstop_msg_t *resp)
{
    resp->message = PSTOP_MESSAGE_UNBOND;

    pstop_client_remove(&(machine->pstops), &(msg->id));

    // if this is the last client then stop the robot
    if(machine->pstops.num_clients == 0U) {
        machine->robot_state.robot_state = ROBOT_STATE_STOPPED;
        machine->robot_state.restart_state = ROBOT_RESTART_STATE_NEED_STOP;
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
       *resp = NULL;
       return result;
    }

    (*resp)->heartbeat_timeout = machine->application.default_timeout_ms;

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
    }

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
    //const device_id_t *this_client = &machine->application.machine_device_id;

    uint64_t now = machine->application.get_time_cb();

    for(uint16_t i = 0U; i < machine->pstops.num_clients; ++i) {
        pstop_client_data_t *client = &(machine->pstops.clients[i]);

        // if for some reason now is in the past compared to the last heart beat
        // then there's no need to check if we've heard from this client
        if(now <= client->last_received_heartbeat) {
            continue;
        }

        uint64_t diff = client->last_received_heartbeat - now;

        // if we're still within the heartbeat timeout then this client is still
        // good.
        if(diff <= client->heartbeat_ms) {
            continue;
        }

        client->missed_heartbeats_counter++;

        // problem! this client hasn't talked to us in a while
        // if we're still within the window of missed heartbeats then we're OK

        uint16_t missed = (uint16_t)(diff / client->heartbeat_ms);
        if(missed > client->missed_heartbeats_counter) {
            // trigger a stop!
            machine_stop_robot(machine);
            return PSTOP_MISSED_HEARTBEATS;
        }
    }

    return PSTOP_OK;
}

void
machine_init(pstop_machine_t *machine, const pstop_application_t *app, pstop_client_data_t *clients, uint16_t max_clients)
{
    machine->handle_machine_message_cb = machine_handle_message;
    machine->handle_protocol_message_cb = protocol_handle_message;
    machine->check_heartbeats_cb = machine_check_heartbeats;

    machine->application = *app;
    machine->pstops.clients = clients;
    machine->pstops.max_clients = max_clients;
    machine->pstops.num_clients = 0U;

    machine->robot_state.client_stop_id = 0U;
    machine->robot_state.restart_state = ROBOT_RESTART_STATE_OK;
    machine->robot_state.robot_state = ROBOT_STATE_OK;

    pstop_clients_init(&(machine->pstops));
}

void
machine_stop_robot(pstop_machine_t *machine)
{
    machine->application.status_cb(PSTOP_STATUS_STOP);
}
