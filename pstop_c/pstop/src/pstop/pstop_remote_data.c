
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include <string.h>

#include "pstop/pstop_remote_data.h"

static uint32_t next_remote_id = 0U;

void
pstop_remote_init(pstop_remote_data_t *remote)
{
    protocol_data_init(&(remote->remote_data));
    remote->last_message = 0U;
    remote->lost_message_counter = 0U;
    remote->missed_heartbeats_counter = 0U;
    remote->remote_state = PSTOP_REMOTE_UNKNOWN;
    remote->local_remote_id = next_remote_id++;
    remote->is_stop_only = true;
}

void
pstop_remotes_init(pstop_remotes_t *remotes)
{
    for(uint16_t i = 0U; i < remotes->max_remotes; ++i) {
        pstop_remote_init(&(remotes->remotes[i]));
    }
}

uint16_t
pstop_remote_num_active(const pstop_remotes_t *remotes)
{
    uint16_t count = 0U;

    for(uint16_t i = 0U; i < remotes->max_remotes; ++i) {
        if(remotes->remotes[i].remote_state != PSTOP_REMOTE_UNKNOWN) {
            count++;
        }
    }

    return count;
}

pstop_remote_data_t *
pstop_remote_get_free_remote(pstop_remotes_t *remotes)
{
    pstop_remote_data_t *remote = NULL;

    // find an empty remote spot
    for(uint16_t i = 0U; i < remotes->max_remotes; ++i) {
        if(remotes->remotes[i].remote_state == PSTOP_REMOTE_UNKNOWN) {
            remote = &(remotes->remotes[i]);
            break;
        }
    }

    if(remote != NULL) {
        pstop_remote_init(remote);
        remote->remote_state = PSTOP_REMOTE_INITING;
    }

    return remote;
}

void
pstop_remote_deactivate(pstop_remote_data_t *remote)
{
    remote->remote_state = PSTOP_REMOTE_UNKNOWN;
}

uint16_t
pstop_remote_num_stopped(const pstop_remotes_t *remotes)
{
    uint16_t count = 0U;

    for(uint16_t i = 0U; i < remotes->max_remotes; ++i) {
        if(remotes->remotes[i].remote_state == PSTOP_REMOTE_STOPPED) {
            count++;
        }
    }

    return count;
}

pstop_remote_data_t *
pstop_remote_get(const pstop_remotes_t *remotes, const device_id_t *remote_id)
{
    for(uint16_t i = 0U; i < remotes->max_remotes; ++i) {
        if(remotes->remotes[i].remote_state != PSTOP_REMOTE_UNKNOWN) {
            if(device_id_cmp(&(remotes->remotes[i].remote_data.remote_id), remote_id) == 0) {
                return &(remotes->remotes[i]);
            }
        }
    }

    return NULL;
}
