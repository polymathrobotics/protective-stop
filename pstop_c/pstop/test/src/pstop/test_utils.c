
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "pstop/test_utils.h"

void
device_id_set(device_id_t *device_id, const char *id)
{
    device_id_init(device_id);

    size_t len = strlen(id);
    if(len < DEVICE_ID_LENGTH) {
        memcpy(device_id->data, id, len);
    }
    else {
        memcpy(device_id->data, id, DEVICE_ID_LENGTH);
    }
}
