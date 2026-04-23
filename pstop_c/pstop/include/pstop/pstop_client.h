#ifndef PSTOP_PSTOP_CLIENT_H
#define PSTOP_PSTOP_CLIENT_H

#include <stdint.h>

#include "pstop/device_id.h"
#include "pstop/constants.h"

typedef enum {

    PSTOP_CLIENT_BONDED = 1,
    PSTOP_CLIENT_UNBONDED = 2,
    PSTOP_CLIENT_STOPPED = 3,
    PSTOP_CLIENT_FAILURE = 4,
    PSTOP_CLIENT_UNKNOWN = 255

} pstop_client_state_t;

typedef struct {

    uint32_t local_client_id;

    device_id_t client_id;

    // last time we've heard from this client
    uint64_t last_timestamp;

    // last time we've heard from this client
    uint64_t last_received_heartbeat;

    // how frequently we should hear from this client
    uint64_t heartbeat_ms;

    // the counter indicating each message we are sending
    uint32_t msg_counter;

    uint32_t last_counter;

    // approx how far off is the client's clock from this clock
    // compare the incoming pstop_msg.stamp to this clock.
    // Ignoring network transit time
    int32_t clock_drift;

    // how many messages have we lost so far?
    uint16_t lost_message_counter;

    // how many heartbeats have we missed?
    uint16_t missed_heartbeats_counter;

    pstop_client_state_t client_state;

} pstop_client_data_t;

typedef struct {

    pstop_client_data_t *clients;
    uint16_t max_clients;
    uint16_t num_clients;

} pstop_clients_t;

/**
 * Initialize the pstop_client_data_t structure to default values.
 */
void pstop_client_init(pstop_client_data_t *client);

/**
 * Initializes a pstop_clients object to default values.
 */
void pstop_clients_init(pstop_clients_t *clients);

/**
 * Returns an empty pstop_client if one is available
 *
 * @Return the client data if successful, NULL if no space available.
 */
pstop_client_data_t *pstop_client_get_free_client(pstop_clients_t *clients);

/**
 * Removes the specified pstop client by the device ID.
 */
void pstop_client_remove(pstop_clients_t *clients, const device_id_t *client_id);

/**
 * Finds the specified pstop client by device ID.
 *
 * @Return the requsted client if found or NULL if not found.
 */
pstop_client_data_t *pstop_client_get(pstop_clients_t *clients, const device_id_t *client_id);

#endif /* PSTOP_PSTOP_CLIENT_H */
