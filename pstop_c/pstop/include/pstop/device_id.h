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
 * @brief Initializes the specified device to an empty ID.
 *
 * @param device_id The device_id to initialize.
 */
void device_id_init(device_id_t *device_id);

/**
 * @brief Copies the device ID from id to device_id.
 *
 * @param device_id The device ID to set
 * @param id The device ID to copy
 */
void device_id_copy(device_id_t *device_id, const device_id_t *id);

/**
 * @brief Compares two device ids.
 *
 * @param lhs The left-hand side of the comparision
 * @param rhs The right-hand side of the comparision
 *
 * @return 0 if lhs == rhs. Otherwise returns non-zero.
 */
int device_id_cmp(const device_id_t *lhs, const device_id_t *rhs);

#if PSTOP_VERSION == 0x00

/**
 * @brief Utility function to set a device ID from a uint32_t
 *
 * @param device_id The device ID to set
 * @param id The integer value to copy
 */
void device_id_set(device_id_t *device_id, uint32_t id);
#endif

#endif /* PSTOP_DEVICE_ID_H */
