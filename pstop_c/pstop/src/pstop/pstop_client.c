
#include <stdlib.h>
#include <string.h>

#include "pstop/pstop_client.h"

static uint32_t next_client_id = 0U;

void
pstop_client_init(pstop_client_data_t *client)
{
    device_id_init(&(client->client_id));
    client->last_timestamp = 0U;
    client->last_received_heartbeat = 0U;
    client->heartbeat_ms = 0U;
    client->msg_counter = 0U;
    client->last_counter = 0U;
    client->clock_drift = 0U;
    client->lost_message_counter = 0U;
    client->missed_heartbeats_counter = 0U;
    client->client_state = PSTOP_CLIENT_UNKNOWN;
    client->local_client_id = next_client_id++;
}

static
void
pstop_client_copy(pstop_client_data_t *dest, pstop_client_data_t *src)
{
    memcpy(dest, src, sizeof(pstop_client_data_t));
}

void
pstop_clients_init(pstop_clients_t *clients)
{
    for(uint16_t i = 0U; i < clients->max_clients; ++i) {
        pstop_client_init(&(clients->clients[i]));
    }
}

pstop_client_data_t *
pstop_client_get_free_client(pstop_clients_t *clients)
{
    if(clients->num_clients == clients->max_clients) {
        return NULL;
    }

    pstop_client_data_t *client = &(clients->clients[clients->num_clients]);
    clients->num_clients++;

    pstop_client_init(client);

    return client;
}

void
pstop_client_remove(pstop_clients_t *clients, const device_id_t *client_id)
{
    // copies the last position client to replace the one we are removing
    // then clears out the last one
    for(uint16_t i = 0U; i < clients->num_clients; ++i) {
        if(device_id_cmp(&(clients->clients[i].client_id), client_id) == 0) {
            if(clients->num_clients > 1U) {
                pstop_client_copy(&(clients->clients[i]), &(clients->clients[clients->num_clients - 1U]));
            }
            clients->num_clients--;
            pstop_client_init(&(clients->clients[clients->num_clients]));
        }
    }
}

pstop_client_data_t *
pstop_client_get(pstop_clients_t *clients, const device_id_t *client_id)
{
    for(uint16_t i = 0U; i < clients->num_clients; ++i) {
        if(device_id_cmp(&(clients->clients[i].client_id), client_id) == 0) {
            return &(clients->clients[i]);
        }
    }

    return NULL;
}
