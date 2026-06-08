
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

operator_details_t
is_operator_allowed(const device_id_t *device_id)
{
    operator_details_t details;
    details.allowed = 1;
    details.stop_only = 0;
    details.heartbeat_ms = 1000U;

    return details;
}

pstop_status_message_t lastStatus = 0;

static
void
robot_status(pstop_status_message_t status)
{
    if(lastStatus != status) {
        if(status == PSTOP_STATUS_OK) {
            fprintf(stderr, "Robot Status = OK\n");
        }
        else {
            fprintf(stderr, "Robot Status = STOP\n");
        }
        lastStatus = status;
    }
}

void
dump_bytes(const uint8_t *b)
{
    for(int i = 0U; i < PSTOP_MESSAGE_SIZE; ++i) {
        fprintf(stderr, "%X ", (int)b[i]);
    }
    fprintf(stderr, "\n");
}

static const char * const OK_STR = "OK";
static const char * const STOP_STR = "STOP";
static const char * const BOND_STR = "BOND";
static const char * const UNBOND_STR = "UNBOND";
static const char * const UNKNOWN_STR = "UNKNOWN";
static
const char *
get_message_str(uint8_t message)
{
    switch(message) {
    case PSTOP_MESSAGE_OK: return OK_STR;
    case PSTOP_MESSAGE_STOP: return STOP_STR;
    case PSTOP_MESSAGE_BOND: return BOND_STR;
    case PSTOP_MESSAGE_UNBOND: return UNBOND_STR;
    }
    return UNKNOWN_STR;
}

static
void
simple_log(uint64_t timestamp, const device_id_t *client, uint8_t message, pstop_error_t error)
{
    uint32_t ip = client->data;
    uint8_t ip1 = (uint8_t)((ip >> 24) & 0xFFU);
    uint8_t ip2 = (uint8_t)((ip >> 16) & 0xFFU);
    uint8_t ip3 = (uint8_t)((ip >> 8) & 0xFFU);
    uint8_t ip4 = (uint8_t)((ip) & 0xFFU);
    fprintf(stderr, "[%ld] %d.%d.%d.%d %s %d\n", timestamp, (int)ip1, (int)ip2, (int)ip3, (int)ip4, get_message_str(message), (int)error);
}

int
main(int argc, char *argv[])
{
    transport_udp_init(&udp_transport);
    pstop_application_init(&pstop_app);

    pstop_app.log_message_cb = simple_log;
    pstop_app.operator_details_cb = is_operator_allowed;
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

    device_id_t machine_uuid = {
        .data = 0x01020304U
    };
    device_id_copy(&(machine.application->machine_device_id), &machine_uuid);

    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
    uint8_t respbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    fprintf(stderr, "Connected to localhost:%d\n", port);
    while(1) {
        struct sockaddr_storage client;
        result = transport_udp_read(&udp_transport, reqbytes, PSTOP_MESSAGE_SIZE, &client);

        if(result == PSTOP_MESSAGE_SIZE) {
            //dump_bytes(reqbytes);
            pstop_message_decode(&req_msg, reqbytes);

            fprintf(stderr, "Got message: %d from %X\n", req_msg.message, req_msg.id.data);
            pstop_error_t error = machine.handle_protocol_message_cb(&machine, &req_msg, &resp_msg);
            if(error != PSTOP_OK) {
                fprintf(stderr, "Invalid request: %d\n", (int)error);
                continue;
            }
            //fprintf(stderr, "Sending resp: counter=%d\n", resp_msg.counter);
            pstop_message_encode(&resp_msg, respbytes);
            transport_udp_write(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, (struct sockaddr_in *)&client);
        }
        machine.check_heartbeats_cb(&machine);
    }

    transport_udp_close(&udp_transport);

    return 0;
}
