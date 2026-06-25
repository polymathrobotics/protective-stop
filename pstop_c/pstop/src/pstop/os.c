
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/os.h"
#include "pstop/time.h"

void
pstop_os_env_init(pstop_os_env_t *env)
{
    env->get_time_cb = time_get_now;
}

void
pstop_os_env_set_time_cb(pstop_os_env_t *env, get_current_time_t time_cb)
{
    env->get_time_cb = time_cb;
}
