
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "pstop/device_id.h"

#if PSTOP_VERSION == 0x00U
void
device_id_init(device_id_t *device_id)
{
    device_id->data = 0U;
}

void
device_id_copy(device_id_t *device_id, const device_id_t *id)
{
    device_id->data = id->data;
}

int
device_id_cmp(const device_id_t *lhs, const device_id_t *rhs)
{
    return (int)(lhs->data - rhs->data);
}

void
device_id_set(device_id_t *device_id, uint32_t id)
{
    device_id->data = id;
}
#else
#   error "Unsupported PSTOP_VERSION"
#endif
