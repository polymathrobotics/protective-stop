
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>

#include "pstop/protocol.h"
#if 0
static
void
init_response_from_request(const pstop_msg_t *req, pstop_msg_t *resp)
{
    device_id_copy(&(resp->receiver_id), &(req->id));
    device_id_copy(&(resp->id), &(req->receiver_id));
    resp->received_counter = req->counter;
    resp->received_stamp = req->stamp;
}
#endif

static
pstop_error_t
check_message_order(pstop_client_data_t *client, const pstop_msg_t *msg)
{
    if(client->last_timestamp >= msg->stamp) {
        return PSTOP_MSG_OUT_OF_ORDER;
    }
    if(client->msg_counter >= msg->counter) {
        return PSTOP_MSG_REPETITION;
    }

    return PSTOP_OK;
}

static
pstop_error_t
validate_message(pstop_machine_t *machine, pstop_client_data_t *client, const pstop_msg_t *req, pstop_msg_t **resp)
{
    // Check the message and ignore it if it's invalid
    pstop_error_t result = check_message_order(client, req);
    if(result != PSTOP_OK) {
        *resp = NULL;
        return result;
    }

    // check for lost messages
    if(req->counter != (client->msg_counter - 1U)) {
        client->lost_message_counter++;
        if(client->lost_message_counter >= machine->application->app_config.max_lost_messages) {
            // too many lost messages
            // clean up this client
            *resp = NULL;
            machine_stop_robot(machine);
            return PSTOP_MSG_LOST;
        }
    }
    else {
        client->lost_message_counter = 0U;
    }

    client->missed_heartbeats_counter = 0U;

    return PSTOP_OK;
}

pstop_error_t
protocol_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp)
{
    // validate checksum

    pstop_client_data_t *client = pstop_client_get(&(machine->pstops), &(req->id));

    if(client == NULL) {
        // validate that this is a new client message
    }
    else {
        // validate lost/out of order message
        pstop_error_t result = validate_message(machine, client, req, resp);
        if(result != PSTOP_OK) {
            return result;
        }
    }

    // now send the message to the machine for pstop handling.
    // This function will create a new client if necessary
    pstop_error_t result = machine->handle_machine_message_cb(machine, req, resp);

    if(result != PSTOP_OK) {
        return result;
    }

    if(resp != NULL) {
        // add in response black channel values
        client = pstop_client_get(&(machine->pstops), &(req->id));
        if(client == NULL) {
            // problem.
            return PSTOP_FATAL;
        }

        // let's add response black channel values
        client->msg_counter++;
        client->last_timestamp = req->stamp;

        return PSTOP_OK;
    }


    return PSTOP_FATAL;
}
