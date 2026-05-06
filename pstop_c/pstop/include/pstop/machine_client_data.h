// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_MACHINE_CLIENT_DATA_H
#define PSTOP_MACHINE_CLIENT_DATA_H

#include <stdint.h>

#include "pstop/device_id.h"

typedef struct {

    device_id_t client_id;

    // last time we've heard from this client
    uint64_t last_timestamp;

    // how frequently we should ping this client
    uint64_t heartbeat_ms;

    // the counter indicating each message we are sending
    uint32_t msg_counter;

    uint32_t last_counter;

} machine_client_data_t;

void machine_client_init(machine_client_data_t *client);

#endif /* PSTOP_MACHINE_CLIENT_DATA_H */
