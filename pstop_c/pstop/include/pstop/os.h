// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_OS_H
#define PSTOP_OS_H

#include <stdint.h>

typedef uint64_t (* get_current_time_t)(void);

typedef struct pstop_os_env {

    /**
     * Callback to return the current time.
     */
    get_current_time_t get_time_cb;

} pstop_os_env;

void pstop_os_env_init(pstop_os_env *env);

#endif /* PSTOP_OS_H */
