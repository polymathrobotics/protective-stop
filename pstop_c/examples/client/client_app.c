
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include "pstop/pstop_msg.h"
#include "transport/udp/udp_transport.h"
#include "pstop/device_id.h"
#include "pstop/os.h"
#include "pstop/machine_client_data.h"

udp_transport_data_t udp_transport;

void
sleep_ms(uint64_t ms)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = ms * 1000000L;
    nanosleep(&ts, NULL);
}

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

int
send_msg(udp_transport_data_t *transport, pstop_os_env *env, machine_client_data_t *machine, const device_id_t *uuid, uint8_t msg)
{
    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;

    uint64_t now = env->get_time_cb();

    req_msg.message = msg;
    device_id_copy(&req_msg.id, uuid);
    device_id_copy(&req_msg.receiver_id, &(machine->client_id));
    req_msg.counter = machine->last_counter + 1U;
    req_msg.received_counter = machine->last_counter;
    req_msg.received_stamp = machine->last_timestamp;
    req_msg.stamp = now;

    machine->last_timestamp = now;
    machine->last_counter = req_msg.counter;

    pstop_message_encode(&req_msg, reqbytes);

    transport_udp_write(transport, reqbytes, PSTOP_MESSAGE_SIZE, NULL);

    if(!read_msg(transport, env, &resp_msg, 2000)) {
        return 0;
    }
    fprintf(stderr, "Received msg: %d counter=%d\n", resp_msg.message, resp_msg.counter);
    machine->last_counter = resp_msg.counter;
    machine->last_timestamp = resp_msg.stamp;

    return 1;
}

void
send_bond(udp_transport_data_t *transport, pstop_os_env *env, machine_client_data_t *machine, const device_id_t *uuid)
{
    if(send_msg(transport, env, machine, uuid, PSTOP_MESSAGE_BOND)) {
        fprintf(stderr, "BOND Success\n");
    }
}

void
send_ok(udp_transport_data_t *transport, pstop_os_env *env, machine_client_data_t *machine, const device_id_t *uuid, int is_ok)
{
    if(is_ok) {
        if(send_msg(transport, env, machine, uuid, PSTOP_MESSAGE_OK)) {
            fprintf(stderr, "OK Success\n");
        }
    }
    else {
        if(send_msg(transport, env, machine, uuid, PSTOP_MESSAGE_STOP)) {
            fprintf(stderr, "STOP Success\n");
        }
    }
}

void
send_unbond(udp_transport_data_t *transport, pstop_os_env *env, machine_client_data_t *machine, const device_id_t *uuid)
{
    if(send_msg(transport, env, machine, uuid, PSTOP_MESSAGE_UNBOND)) {
        fprintf(stderr, "UNBOND Success\n");
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

    machine_client_data_t machine;
    machine_client_init(&machine);

    pstop_os_env env;
    pstop_os_env_init(&env);

    device_id_t machine_uuid = {
        .data = {
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xFF
        }
    };
    device_id_copy(&(machine.client_id), &machine_uuid);

    device_id_t this_uuid = {
        .data = {
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xFF
        }
    };
    this_uuid.data[15] = (uint8_t)(uuid_byte & 0xFFU);

    fprintf(stderr, "uuid=%d\n", this_uuid.data[15]);
    send_bond(&udp_transport, &env, &machine, &this_uuid);

    sleep_ms(300);

    for(int i = 0; i < 30; ++i) {
        send_ok(&udp_transport, &env, &machine, &this_uuid, (i % 2));
        sleep_ms(300);
    }

    send_unbond(&udp_transport, &env, &machine, &this_uuid);

    return 0;
}
