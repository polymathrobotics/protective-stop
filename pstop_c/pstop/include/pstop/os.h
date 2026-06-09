// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_OS_H
#define PSTOP_OS_H

#include <stdint.h>

/**
 * A callback to return the current time in milliseconds.
 */
typedef uint64_t (* get_current_time_t)(void);

/**
 * This struct contains a callback function to return the current
 * time in milliseconds. If this code is deployed on a supported OS
 * then this function is already defined. If deploying on an unsupported
 * OS then applications must create their own time function and set this
 * callback appropriately.
 */
typedef struct pstop_os_env_t {

    /**
     * Callback to return the current time.
     */
    get_current_time_t get_time_cb;

} pstop_os_env_t;

/**
 * @brief Initialize an os_env object with a default time function.
 */
void pstop_os_env_init(pstop_os_env_t *env);

#endif /* PSTOP_OS_H */
