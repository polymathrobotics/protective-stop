// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once

/* Keep the audited device queue size; owned TX uses at most one drain entry. */
#define ML_USB_TX_EVENT_QUEUE_SIZE 64

/* Keep USB below the safety tasks (checked by both applications), while
 * sharing priority with the WG workers that previously delayed its callbacks. */
#define ML_USB_TASK_PRIORITY 7
