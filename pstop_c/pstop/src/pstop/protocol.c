
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>

#include "pstop/protocol.h"

static
int
is_checksum_valid(const pstop_msg_t *req)
{
    return req->checksum == req->calculated_checksum;
}

static
pstop_error_t
check_counter(const pstop_application_config_t *app_config, const pstop_client_data_t *client, const pstop_msg_t *req)
{
    if(req->counter <= client->client_data.last_sent_counter) {
        return PSTOP_MSG_OUT_OF_ORDER;
    }
    if((req->counter - client->client_data.last_sent_counter) > app_config->max_lost_messages) {
        return PSTOP_MSG_LOST;
    }
    if(req->received_counter > client->client_data.msg_counter) {
        return PSTOP_MSG_OUT_OF_ORDER;
    }
    if((client->client_data.msg_counter - req->received_counter) > app_config->max_lost_messages) {
        return PSTOP_MSG_LOST;
    }

    return PSTOP_OK;
}

static
pstop_error_t
check_timestamp(const pstop_application_config_t *app_config, const protocol_data_t *client, const pstop_msg_t *req)
{
    if(req->received_stamp == client->last_timestamp) {
        return PSTOP_OK;
    }

    // did the other end miss a message?
    if(req->received_stamp < client->last_timestamp) {
        uint64_t diff = client->last_timestamp - req->received_stamp;

        uint16_t missed = (uint16_t)(diff / client->heartbeat_ms);

        if(missed > app_config->max_missed_heartbeats) {
            return PSTOP_MSG_LOST;
        }
    }
    else {
        return PSTOP_MSG_OUT_OF_ORDER;
    }

    return PSTOP_OK;
}

pstop_error_t
protocol_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t *resp)
{
    // validate checksum
    if(!is_checksum_valid(req)) {
        return PSTOP_MSG_INVALID_CHECKSUM;
    }

    // make sure te target ID is this machine ID
    if(device_id_cmp(&(machine->application->machine_device_id), &(req->receiver_id))) {
        return PSTOP_ERROR_INVALID_ID;
    }

    // no client found, can we add it?
    operator_details_t details = machine->application->operator_details_cb((&req->id));

    if(!details.allowed) {
        return PSTOP_OPERATOR_NOT_ALLOWED;
    }

    pstop_client_data_t *client = pstop_client_get(&(machine->pstops), &(req->id));

    // if we've already seen this client, then validate the counter/timestamps
    if(client != NULL) {
        pstop_error_t err = check_counter(&(machine->application->app_config), client, req);
        if(err != PSTOP_OK) {
            return err;
        }

        err = check_timestamp(&(machine->application->app_config), &(client->client_data), req);
        if(err != PSTOP_OK) {
            return err;
        }
    }

    // now send the message to the machine for pstop handling.
    // This function will create a new client if necessary
    pstop_error_t result = machine->handle_machine_message_cb(machine, req, resp);

    if(result != PSTOP_OK) {
        return result;
    }

    // add in response black channel values
    client = pstop_client_get(&(machine->pstops), &(req->id));

    uint64_t now = machine->application->env.get_time_cb();

    // let's add response black channel values
    resp->counter = 0U; // 0 will be if this operator requested unbond.
    resp->stamp = now;
    resp->received_counter = req->counter;
    resp->received_stamp = req->stamp;
    resp->heartbeat_timeout = 0U;
    device_id_copy(&(resp->id), &(machine->application->machine_device_id));
    device_id_copy(&(resp->receiver_id), &(req->id));

    // null client will happen on unbond
    if(client != NULL) {
        resp->counter = client->client_data.msg_counter + 1U;
        resp->heartbeat_timeout = client->client_data.heartbeat_ms;
        client->client_data.msg_counter++;
        client->client_data.last_sent_counter = req->counter;
        client->client_data.last_timestamp = now;
    }

    return PSTOP_OK;
}
