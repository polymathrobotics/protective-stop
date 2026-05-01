
#include <stdlib.h>
#include <string.h>

#include "pstop/pstop_client.h"

static uint32_t next_client_id = 0U;

void
pstop_client_init(pstop_client_data_t *client)
{
    device_id_init(&(client->client_id));
    client->last_timestamp = 0U;
    client->heartbeat_ms = 0U;
    client->msg_counter = 0U;
    client->last_counter = 0U;
    client->clock_drift = 0U;
    client->lost_message_counter = 0U;
    client->missed_heartbeats_counter = 0U;
    client->client_state = PSTOP_CLIENT_UNKNOWN;
    client->local_client_id = next_client_id++;
}

void
pstop_clients_init(pstop_clients_t *clients)
{
    for(uint16_t i = 0U; i < clients->max_clients; ++i) {
        pstop_client_init(&(clients->clients[i]));
    }
}

uint16_t
pstop_client_num_active(pstop_clients_t *clients)
{
    uint16_t count = 0U;

    for(uint16_t i = 0U; i < clients->max_clients; ++i) {
        if(clients->clients[i].client_state != PSTOP_CLIENT_UNKNOWN) {
            count++;
        }
    }

    return count;
}

pstop_client_data_t *
pstop_client_get_free_client(pstop_clients_t *clients)
{
    pstop_client_data_t *client = NULL;

    // find an empty client spot
    for(uint16_t i = 0U; i < clients->max_clients; ++i) {
        if(clients->clients[i].client_state == PSTOP_CLIENT_UNKNOWN) {
            client = &(clients->clients[i]);
            break;
        }
    }

    if(client != NULL) {
        pstop_client_init(client);
        client->client_state = PSTOP_CLIENT_INITING;
    }

    return client;
}

void
pstop_client_deactivate(pstop_client_data_t *client)
{
    client->client_state = PSTOP_CLIENT_UNKNOWN;
}

pstop_client_data_t *
pstop_client_get(pstop_clients_t *clients, const device_id_t *client_id)
{
    for(uint16_t i = 0U; i < clients->max_clients; ++i) {
        if(clients->clients[i].client_state != PSTOP_CLIENT_UNKNOWN) {
            if(device_id_cmp(&(clients->clients[i].client_id), client_id) == 0) {
                return &(clients->clients[i]);
            }
        }
    }

    return NULL;
}
