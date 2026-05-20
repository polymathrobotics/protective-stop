// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PSTOP_CONFIG_H
#define PSTOP_CONFIG_H

#ifndef PSTOP_VERSION
#   define PSTOP_VERSION 0x00U
#endif

#if PSTOP_VERSION == 0x00U
#   define PSTOP_MESSAGE_SIZE 40U
#   define PSTOP_DEVICE_ID_LENGTH 4U
#else
#   error "Unsupported PSTOP_VERSION"
#endif

#endif /* PSTOP_CONFIG_H */
