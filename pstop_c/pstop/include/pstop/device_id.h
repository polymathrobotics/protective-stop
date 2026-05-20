// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_DEVICE_ID_H
#define PSTOP_DEVICE_ID_H

#include <stdint.h>

#include "pstop/config.h"

typedef struct {
#if PSTOP_VERSION == 0x00
    // IPv4 address
    uint32_t data;
#else
#   error "Unsupported PSTOP_VERSION"
#endif
} device_id_t;

/**
 * Initializes the specified device to 0.
 */
void device_id_init(device_id_t *device_id);

/**
 * Copies the device ID from id to device_id.
 */
void device_id_copy(device_id_t *device_id, const device_id_t *id);

/**
 * Compares two device ids.
 *
 * @Return 0 if lhs == rhs. Otherwise returns non-zero.
 */
int device_id_cmp(const device_id_t *lhs, const device_id_t *rhs);

#if PSTOP_VERSION == 0x00
void device_id_set(device_id_t *device_id, uint32_t id);
#endif

#endif /* PSTOP_DEVICE_ID_H */
