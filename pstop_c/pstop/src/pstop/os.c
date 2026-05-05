
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/os.h"
#include "pstop/time.h"

void
pstop_os_env_init(pstop_os_env *env)
{
    env->get_time_cb = time_get_now;
}
