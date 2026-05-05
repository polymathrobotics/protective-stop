
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>
#include <stdlib.h>

#include "pstop/pstop_msg.h"
#include "transport/udp/udp_transport.h"
#include "pstop/device_id.h"
#include "pstop/os.h"

udp_transport_data_t udp_transport;

int
read_msg(udp_transport_data_t *transport, pstop_os_env *env, pstop_msg_t *resp, uint64_t timeout)
{
    struct sockaddr_storage client;
    uint8_t respbytes[PSTOP_MESSAGE_SIZE];

    uint64_t start = env->get_time_cb();

    while(1) {
        int result = transport_udp_read(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, &client);
        if(result == PSTOP_MESSAGE_SIZE) {
            pstop_message_decode(resp, respbytes);
            return 1;
        }
        uint64_t now = env->get_time_cb();
        if((now - start) >= timeout) {
            fprintf(stderr, "Timeout reading response\n");
            break;
        }
    }

    return 0;
}

void
send_bond(udp_transport_data_t *transport, pstop_os_env *env, const device_id_t *uuid)
{
    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    req_msg.message = PSTOP_MESSAGE_BOND;
    device_id_copy(&req_msg.id, uuid);

    pstop_message_encode(&req_msg, reqbytes);

    transport_udp_write(transport, reqbytes, PSTOP_MESSAGE_SIZE, NULL);

    if(read_msg(transport, env, &resp_msg, 2000)) {
        fprintf(stderr, "Received msg: %d\n", resp_msg.message);
    }
}

void
send_ok(udp_transport_data_t *transport, pstop_os_env *env, const device_id_t *uuid, int is_ok)
{
    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    if(is_ok) {
        req_msg.message = PSTOP_MESSAGE_OK;
    }
    else {
        req_msg.message = PSTOP_MESSAGE_STOP;
    }

    device_id_copy(&req_msg.id, uuid);

    pstop_message_encode(&req_msg, reqbytes);

    transport_udp_write(transport, reqbytes, PSTOP_MESSAGE_SIZE, NULL);

    if(read_msg(transport, env, &resp_msg, 2000)) {
        fprintf(stderr, "Received msg: %d\n", resp_msg.message);
    }
}

void
send_unbond(udp_transport_data_t *transport, pstop_os_env *env, const device_id_t *uuid)
{
    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    req_msg.message = PSTOP_MESSAGE_UNBOND;
    device_id_copy(&req_msg.id, uuid);

    pstop_message_encode(&req_msg, reqbytes);

    transport_udp_write(transport, reqbytes, PSTOP_MESSAGE_SIZE, NULL);

    if(read_msg(transport, env, &resp_msg, 2000)) {
        fprintf(stderr, "Received msg: %d\n", resp_msg.message);
    }
}

int
main(int argc, char *argv[])
{
    transport_udp_init(&udp_transport);

    if(argc < 3) {
        printf("USAGE:\n");
        printf("  %s <port> <uuid-byte>\n\n", argv[0]);
        printf("Example:\n");
        printf("  client_app 8890 15\n");
        return -1;
    }

    int port = atoi(argv[1]);
    int uuid_byte = atoi(argv[2]);

    int result = transport_udp_connect(&udp_transport, "127.0.0.1", port);
    if(result < 0) {
        fprintf(stderr, "Unable to open UDP: %d\n", result);
        return -1;
    }

    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
    uint8_t respbytes[PSTOP_MESSAGE_SIZE];

    pstop_os_env env;
    pstop_os_env_init(&env);

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    device_id_t uuid = {
        .data = {
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xFF
        }
    };
    uuid.data[15] = (uint8_t)(uuid_byte & 0xFFU);

    fprintf(stderr, "uuid=%d\n", uuid.data[15]);
    send_bond(&udp_transport, &env, &uuid);

    for(int i = 0; i < 3; ++i) {
        send_ok(&udp_transport, &env, &uuid, (i % 2));
    }

    send_unbond(&udp_transport, &env, &uuid);

    return 0;
}
