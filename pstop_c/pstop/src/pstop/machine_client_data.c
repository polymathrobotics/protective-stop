
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/machine_client_data.h"

void
machine_client_init(machine_client_data_t *client)
{
    device_id_init(&(client->client_id));
    client->last_timestamp = 0U;
    client->heartbeat_ms = 0U;
    client->msg_counter = 0U;
    client->last_counter = 0U;
}
