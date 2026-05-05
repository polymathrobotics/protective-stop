
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>

#include "transport/udp/udp_transport.h"
#include "pstop/machine.h"
#include "pstop/pstop_msg.h"

pstop_application_t pstop_app;

#define MAX_CLIENTS 3U

pstop_client_data_t pstop_clients[MAX_CLIENTS];

pstop_machine_t machine;

udp_transport_data_t udp_transport;

int is_operator_allowed(const device_id_t *device_id)
{
    return 1;
}

pstop_status_message_t lastStatus = 0;

static
int
robot_status(pstop_status_message_t status)
{
    if(lastStatus != status) {
        fprintf(stderr, "Status = %d\n", (int)status);
        lastStatus = status;
    }

    return 0;
}

int
main(int argc, char *argv[])
{
    transport_udp_init(&udp_transport);
    pstop_application_init(&pstop_app);

    pstop_app.app_config.default_timeout_ms = 1000U;
    pstop_app.operator_allowed_cb = is_operator_allowed;
    pstop_app.status_cb = robot_status;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    int port = 8890;
    if(argc > 1) {
        port = atoi(argv[1]);
    }
    int result = transport_udp_listen(&udp_transport, "127.0.0.1", port);
    if(result < 0) {
        fprintf(stderr, "Unable to open UDP: %d\n", result);
        return -1;
    }

    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
    uint8_t respbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;
    pstop_msg_t *resp_msg_ptr = &resp_msg;

    fprintf(stderr, "Connected to localhost:%d\n", port);
    while(1) {
        struct sockaddr_storage client;
        result = transport_udp_read(&udp_transport, reqbytes, PSTOP_MESSAGE_SIZE, &client);

        if(result == PSTOP_MESSAGE_SIZE) {
            pstop_message_decode(&req_msg, reqbytes);

            fprintf(stderr, "Got message: %d from %d\n", req_msg.message, req_msg.id.data[15]);
            pstop_error_t error = machine.handle_protocol_message_cb(&machine, &req_msg, &resp_msg_ptr);
            if(resp_msg_ptr != NULL) {
                pstop_message_encode(&resp_msg, respbytes);
                transport_udp_write(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, (struct sockaddr_in *)&client);
            }
            else {
                fprintf(stderr, "Invalid response: %d\n", (int)error);
            }
        }
        //machine.check_heartbeats_cb(&machine);
    }

    transport_udp_close(&udp_transport);

    return 0;
}
