// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_DEVICE_ID_H
#define PSTOP_DEVICE_ID_H

#include <stdint.h>

#define DEVICE_ID_LENGTH 16

typedef struct {
    uint8_t data[DEVICE_ID_LENGTH];
} device_id_t;

/**
 * Initializes the specified device to 0.
 */
void device_id_init(device_id_t *device_id);

void device_id_set_bytes(device_id_t *device_id, const uint8_t *data);

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

#endif /* PSTOP_DEVICE_ID_H */
