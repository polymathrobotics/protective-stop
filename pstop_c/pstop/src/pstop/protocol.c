
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>

#include "pstop/protocol.h"

static
int
is_checksum_valid(const pstop_msg_t *req)
{
    return 1;
}

pstop_error_t
protocol_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp)
{
    // validate checksum
    if(!is_checksum_valid(req)) {
        *resp = NULL;
        return PSTOP_MSG_INVALID_CHECKSUM;
    }

    // make sure te target ID is this machine ID
    if(!device_id_cmp(&(machine->application->machine_device_id), &(req->receiver_id))) {
        fprintf(stderr, "Invald receiver ID\n");
        *resp = NULL;
        return PSTOP_ERROR_INVALID_ID;
    }

    if(!machine->application->operator_allowed_cb(&(req->id))) {
        fprintf(stderr, "Operator not allowed\n");
        *resp = NULL;
        return PSTOP_OPERATOR_NOT_ALLOWED;
    }

    pstop_client_data_t *client = pstop_client_get(&(machine->pstops), &(req->id));

    // if we've already seen this client, then validate the counter/timestamps
    if(client != NULL) {
        //fprintf(stderr, "Time = %ld <=> %ld\n", req->stamp, client->last_timestamp);
        if(req->counter <= client->last_counter) {
            *resp = NULL;
            return PSTOP_MSG_OUT_OF_ORDER;
        }
        if((req->counter - client->last_counter) > machine->application->app_config.max_lost_messages) {
            *resp = NULL;
            return PSTOP_MSG_LOST;
        }
        if(req->stamp <= client->last_timestamp) {
            *resp = NULL;
            return PSTOP_MSG_OUT_OF_ORDER;
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

        uint64_t now = machine->application->env.get_time_cb();

        // let's add response black channel values
        (*resp)->counter = req->counter + 1U;
        (*resp)->stamp = now;
        (*resp)->received_counter = req->counter;
        (*resp)->received_stamp = req->stamp;
        device_id_copy(&(*resp)->id, &(machine->application->machine_device_id));
        device_id_copy(&(*resp)->receiver_id, &(req->id));

        // null client will happen on unbond
        if(client != NULL) {
            client->last_counter = (*resp)->counter;
            client->last_timestamp = now;
        }

        return PSTOP_OK;
    }


    return PSTOP_FATAL;
}
