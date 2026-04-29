
#include <stdio.h>

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

int
main(int argc, char *argv[])
{
    transport_init_udp(&udp_transport);
    pstop_application_init(&pstop_app);

    pstop_app.app_config.default_timeout_ms = 1000U;
    pstop_app.operator_allowed_cb = is_operator_allowed;

    machine_init(&machine, &pstop_app, pstop_clients, MAX_CLIENTS);

    int result = transport_udp_open(&udp_transport, "localhost", 8890);
    if(result < 0) {
        fprintf(stderr, "Unable to open UDP: %d\n", result);
        return -1;
    }

    uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
    uint8_t respbytes[PSTOP_MESSAGE_SIZE];

    pstop_msg_t req_msg;
    pstop_msg_t resp_msg;
    pstop_msg_t *resp_msg_ptr = &resp_msg;

    fprintf(stderr, "Connected to localhost:8890\n");
    while(1) {
        struct sockaddr_storage client;
        result = transport_udp_read(&udp_transport, reqbytes, PSTOP_MESSAGE_SIZE, &client);

        if(result == PSTOP_MESSAGE_SIZE) {
            pstop_message_encode(&req_msg, reqbytes);

            pstop_error_t error = machine.handle_protocol_message_cb(&machine, &req_msg, &resp_msg_ptr);
            if(resp_msg_ptr != NULL) {
                pstop_message_encode(&resp_msg, respbytes);
                transport_udp_write(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, (struct sockaddr_in *)&client);
            }
        }
        machine.check_heartbeats_cb(&machine);
    }

    transport_udp_close(&udp_transport);

    return 0;
}
