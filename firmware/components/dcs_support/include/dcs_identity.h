#pragma once

#include <stdint.h>

/* ============================================================================
 * Per-unit device identity.
 *
 * Both the pstop black-channel device ID (pstop_msg.id) and the Tailscale node
 * hostname are derived from the SAME per-unit value so the two always match:
 * a node that appears as "pstop-01a1b2c3" on the tailnet bonds as device ID
 * 0x01a1b2c3 in the safety protocol. The value is the low 24 bits of the eFuse
 * MAC (immutable, unique per chip) tagged with 0x01 in the top byte to mark it
 * a pstop remote (the machine uses 0x0102030x). It is stable for the life of
 * the chip and survives reflash/NVS-erase, unlike the WireGuard node key.
 *
 * Machines must accept this ID: run the machine with `allow_unlisted = true`,
 * or add an [[operator]] entry with `device_id = <printed value>`.
 * ========================================================================== */

/* 0x01xxxxxx — pstop remote device ID for the pstop_c black channel. */
uint32_t dcs_identity_device_id(void);

/* "pstop-01xxxxxx" — Tailscale node hostname matching the device ID. */
const char *dcs_identity_hostname(void);
