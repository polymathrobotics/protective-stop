// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_support.h
 * @brief Boilerplate bring-up for the pstop remote firmware.
 *
 * Everything in this component is *not* part of the safety pattern — it's
 * the OTA-rollback chain, the admin/diagnostic web UI, the per-core load
 * sampler, the boot-counter accounting, the USB-tether/Tailscale glue.
 *
 * The whole point of pulling this out of main.c is so the file that holds
 * the actual lockstep safety logic stays small and auditable. Call
 * dcs_support_init() once at the top of app_main(); spawn your own safety
 * tasks; then call dcs_support_finalize() once those tasks are running.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "microlink.h"
#include "ml_app.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /* Resolved boot state, returned by dcs_support_init() and passed back into
 * dcs_support_finalize(). The fields are useful for main.c if it needs to
 * branch on (e.g.) whether Tailscale started paused or which core handles
 * what. Treat as read-only after init returns. */
  typedef struct
  {
    bool usb_enabled; /* NVS dcs_app/usb_en — USB-NCM tether active */
    bool ts_boot_en; /* NVS dcs_app/ts_boot — Tailscale active at boot
                                   * (post-safety-ladder: cleared if boot_count >= 2) */
    bool derp_only_mode; /* boot_count == 1 ladder — DISCO+STUN suppressed */
    uint16_t boot_count; /* CRASH-class boots since last age-out (Layer 2) */
    uint8_t reset_reason; /* esp_reset_reason() value at boot */
    microlink_t * ml_handle; /* For main.c to query ml state if needed */
    ml_app_t * app; /* For main.c to ml_app_add_page() additional URIs */
  } dcs_boot_state_t;

  /**
 * @brief Bring up everything that isn't the safety pattern itself.
 *
 * Side effects (in order):
 *   - panic_log_init()
 *   - nvs_flash_init() with auto-erase on version mismatch
 *   - esp_reset_reason() + boot-counter accounting (may rollback+reboot if
 *     boot_count > SAFETY_MAX_RAPID_BOOTS — does NOT return in that case)
 *   - Reads NVS for usb_en, ts_boot, pstop_peer
 *   - Applies safety ladder: boot_count==1 → derp_only_mode;
 *                             boot_count>=2 → forces ts_boot_en=false
 *   - Spawns app_heartbeat_task (TWDT keep-alive)
 *   - Brings up W5500 Ethernet first (highest route priority), then ml_app_start()
 *     with the right config (dcs_boot_try_tether_or_skip: Ethernet → USB-NCM →
 *     WiFi). Conditionally brings the WiFi driver up idle when an alt network
 *     (USB-NCM) took the slot — but NOT on the wired-Ethernet path, to save
 *     internal RAM.
 *   - Starts dcs_rgb (WS2812 status LED: colour = active interface, blink = IP
 *     last octet) and dcs_net_supervisor (1 Hz Eth>USB>WiFi default-route enforcer)
 *   - Registers all admin pages (/, /state.json, /api/...)
 *   - Starts load_sampler (per-core CPU + top-task breakdown)
 *   - Starts net_liveness_task (pings the ACTIVE uplink's gateway; abort()s only
 *     if the gateway AND pstop are both silent — a real lwIP wedge)
 *   - Sets verbose ESP_LOG levels for ml_* modules
 *
 * Returns the resolved boot state. After this, main.c can spawn its own
 * safety tasks; the chip already has /admin/ and /state.json answering.
 */
  dcs_boot_state_t dcs_support_init(void);

  /**
 * @brief Final post-spawn safety bring-up. Call once after main.c's
 * own tasks (comparator/cores/whatever) are running.
 *
 * Side effects:
 *   - initial_ota_validate() — esp_ota_mark_app_valid_cancel_rollback
 *   - Starts clear_boot_count_task (Layer 4 age-out + v15.24 auto-recovery)
 *   - If ts_boot_en is false, suspends ml_wg_mgr and pauses DERP TX
 *     (kept here so main.c doesn't have to know about the ts_boot ladder)
 */
  void dcs_support_finalize(const dcs_boot_state_t * bs);

  /* ============================================================================
 * Telemetry sinks for /state.json
 *
 * The admin pages and comparator are decoupled by these atomic-backed
 * publish functions. main.c calls them from its safety tasks; the
 * /state.json handler reads from the same atomics. Safe from any task,
 * no locking required.
 * ========================================================================== */

  /**
 * @brief Publish per-tick core state (called by each safety core task).
 * @param core_id  0 or 1.
 * @param tick     monotonic per-core tick counter.
 * @param verdict  PSTOP_MESSAGE_OK or PSTOP_MESSAGE_STOP — this core's
 *                  independent verdict for this tick.
 */
  void dcs_publish_core_tick(int core_id, uint32_t tick, uint8_t verdict);

  /**
 * @brief Publish this core's E-stop loop diagnostics (called by each core task
 * once per tick, alongside dcs_publish_core_tick).
 * @param core_id 0 or 1.
 * @param high_ok true if the last drive-high tick echoed high (loop continuity).
 * @param low_ok  true if the last drive-low tick echoed low (no stuck-high short).
 */
  void dcs_publish_estop(int core_id, bool high_ok, bool low_ok);

  /**
 * @brief Publish the dual-core clock cross-check state (SR-H-04 / DU-2).
 * Called by the comparator when a cross-check fault latches, and reflected in
 * /state.json as xcheck_fault / xcheck_hb0 / xcheck_hb1.
 * @param fault 0=healthy, 1=peer task/heartbeat stalled, 2=peer clock frozen.
 * @param hb0   core 0's last published heartbeat counter.
 * @param hb1   core 1's last published heartbeat counter.
 */
  void dcs_publish_xcheck(uint32_t fault, uint32_t hb0, uint32_t hb1);

  /**
 * @brief Publish the E-stop GPIO pad-config re-verification state (SR-R-09 /
 * DU-1). Called by the comparator's periodic re-verify; reflected in
 * /state.json as gpio_cfg_fault.
 * @param fault 0=both E-stop IN pads verified (input dir + pull-down enabled);
 *              nonzero=latched config-integrity fault (bit0=chA GPIO40,
 *              bit1=chB GPIO42) — a lost pull-down would let an open loop
 *              float to a false-closed/false-OK read.
 */
  void dcs_publish_gpio_cfg(uint32_t fault);

  /**
 * @brief Publish comparator telemetry (called by the comparator task once
 * per tick after the memcmp).
 * @param sent           Total messages successfully sent to the machine.
 * @param mismatch       Total ticks where the two cores' encodings differed.
 * @param send_fail      Total UDP send failures.
 * @param last_reply_ms  esp_timer_get_time()/1000 of the last received reply.
 * @param rtt_ms         Most-recent round-trip-time (request→reply).
 */
  void dcs_publish_comparator(
    uint32_t sent, uint32_t mismatch, uint32_t send_fail, uint64_t last_reply_ms, uint32_t rtt_ms);

  /**
 * @brief Publish the comparator's current peer target (from /api/pstop_peer).
 * @param peer_ip   IPv4 in host byte order.
 * @param peer_port UDP port.
 */
  void dcs_publish_pstop_peer(uint32_t peer_ip, uint16_t peer_port);

  /**
 * @brief Read the configured pstop peer from NVS at boot (called by main.c
 * once before spawning the comparator). Returns defaults if NVS is empty.
 */
  void dcs_get_initial_pstop_peer(uint32_t * peer_ip, uint16_t * peer_port);

/* ============================================================================
 * Multi-machine support: up to DCS_PSTOP_MAX_MACHINES peer slots. Slot 0 is
 * the legacy single peer (mirrored to the old NVS keys + /api/pstop_peer).
 * ========================================================================== */
#define DCS_PSTOP_MAX_MACHINES 4

/* Default pstop_msg.receiver_id — matches machine_app_runner's default
 * machine_device_id (upstream 0x01020304). Per-slot override via
 * /api/pstop_peers?...&id=. */
#define DCS_PSTOP_DEFAULT_MACHINE_ID 0x01020304u

  /**
 * @brief Read one peer slot. Called by the comparator every tick, so
 * /api/pstop_peers changes take effect live. Lock-free: a concurrent update
 * can yield one tick with a mismatched endpoint/machine_id pair, which the
 * machine rejects and the session re-bonds — benign, fail-safe.
 * @param slot        0..DCS_PSTOP_MAX_MACHINES-1 (out-params zeroed if out of range).
 * @param configured  false = slot empty (other out-params zeroed).
 */
  void dcs_get_pstop_peer_slot(
    int slot, bool * configured, uint32_t * peer_ip, uint16_t * peer_port, uint32_t * machine_id);

  /**
 * @brief Publish one machine session's telemetry (comparator, once per tick
 * per configured slot). sess_state: 0=idle, 1=bonding, 2=bonded.
 */
  void dcs_publish_pstop_machine(
    int slot,
    uint32_t sent,
    uint32_t replies,
    uint32_t send_fail,
    uint32_t rebonds,
    uint32_t hb_ms,
    uint64_t last_reply_ms,
    uint32_t rtt_ms,
    uint8_t last_msg,
    uint8_t sess_state);

  /**
 * @brief Publish the aggregate send-failure cause split (errno buckets from
 * the failing sendto): nomem = ENOMEM (TX-queue/pbuf pressure, typically the
 * DERP relay path under load), route = EHOSTUNREACH/ENETUNREACH/
 * EADDRNOTAVAIL (no path), enotconn = ENOTCONN (WG peer known but no
 * keypair — the far side doesn't know our key; cold-bond gap 2026-08-08),
 * other = the rest. enotconn_kicks counts sustained-ENOTCONN escalations
 * (forced re-handshake + coord re-announce). Exposed in /state.json as
 * pstop_sf_nomem / pstop_sf_route / pstop_sf_other / pstop_sf_enotconn /
 * pstop_sf_enotconn_kicks / pstop_sf_errno.
 */
  void dcs_publish_pstop_sf_causes(
    uint32_t nomem,
    uint32_t route,
    uint32_t txdrv,
    uint32_t txdrv_recovered,
    uint32_t other,
    uint32_t enotconn,
    uint32_t enotconn_kicks,
    int last_errno);

  /**
 * @brief True while the tailnet control plane is CONNECTED (registered,
 * long-poll live). Gate for escalations that only make sense with a
 * working coordination path.
 */
  bool dcs_tailnet_connected(void);

  /**
 * @brief Ask microlink to re-announce this node to the control plane (see
 * microlink_request_announce): nudges the server to re-push us into a
 * far peer's netmap when that peer has forgotten our WG key (sustained
 * ENOTCONN sends after the peer rebooted without its netmap, 2026-08-08).
 * Non-blocking; rate-limit at the call site.
 */
  void dcs_request_announce(void);

  /**
 * @brief Publish machine-role relay supervision (consecutive contradiction
 * ticks per channel; stop_active = persistent-fault stop engaged). Shown in
 * /state.json as relay_fault_a / relay_fault_b / relay_stop (0 on remotes).
 */
  void dcs_publish_relay_fault(uint32_t fault_a_ticks, uint32_t fault_b_ticks, bool stop_active);

  /**
 * @brief Publish one machine-role bonded-remote entry (slot 0..7); id 0
 * clears the slot. Shown in /state.json's bonded_remotes array. stop_only
 * reflects the library's per-remote authorization (true = may STOP but may
 * never re-arm; false = full operator) so it is observable over HTTP.
 */
#define DCS_MACHN_MAX_REMOTES 8
  void dcs_publish_machn_remote(
    int slot, uint32_t remote_id, uint32_t ip, uint32_t state, uint32_t age_ms, uint32_t rtt_ms, bool stop_only);

  /**
 * @brief Publish the machine-role arming/restart state (comparator, once per
 * tick; never called on remotes, where both fields read 0). Shown in
 * /state.json as remote_stop_id / restart_state.
 * @param remote_stop_id pstop_c robot_state.remote_stop_id — the remote id
 *        that stopped the robot, or that owns the in-progress stop/OK arming
 *        cycle; 0 = none (fresh boot, or robot running with no cycle open).
 * @param restart_state  pstop_c ROBOT_RESTART_STATE_*: 0 = OK (an OK message
 *        may re-arm), 1 = NEED_STOP (a STOP gesture must come first),
 *        2 = STOP_RECEIVED (stop seen, awaiting the OK half of the gesture).
 */
  void dcs_publish_machn_arm(uint32_t remote_stop_id, uint32_t restart_state);

  /* ============================================================================
 * Operator allowlist (machine-role authorization).
 *
 * A bonded remote is ACCEPTED and heartbeat-monitored but STOP-ONLY by default:
 * it may command STOP, never re-arm (STOP->OK). Only a remote whose 32-bit
 * pstop id is on THIS list is a full operator (stop_only=false, may re-arm).
 * The list is EMPTY on blank NVS, so out of the box every remote is stop-only =
 * maximally safe; operator ids are added during configuration. Persisted in the
 * dcs_app NVS namespace and mirrored to a lock-free RAM cache that the safety
 * cores read from their remote_details callback (never touching NVS/flash).
 * ========================================================================== */
#define DCS_MAX_OPERATORS 16

  /** @brief True if remote_id is a listed operator (RAM-only, lock-free — safe
   * to call from the safety cores' remote_details callback). Empty list => the
   * fail-safe: every id returns false => every remote is stop-only. */
  bool dcs_operator_is_listed(uint32_t remote_id);

  /** @brief Copy the current operator ids into out[] (up to DCS_MAX_OPERATORS);
   * returns the count. Used by the admin API + /state.json. */
  int dcs_operator_get_list(uint32_t out[DCS_MAX_OPERATORS]);

  /** @brief Add an operator id (idempotent) to the RAM cache AND NVS. Returns
   * ESP_ERR_NO_MEM when the list is full, ESP_ERR_INVALID_ARG for id 0. */
  esp_err_t dcs_operator_add(uint32_t remote_id);

  /** @brief Remove an operator id from the RAM cache AND NVS (no error if
   * absent). */
  esp_err_t dcs_operator_del(uint32_t remote_id);

  /**
 * @brief Tell the transport whether the pstop link to the machine is alive.
 *
 * Thin wrapper over microlink_notify_priority_health(). Call with true on every
 * machine reply and false when the reply-loss watchdog / bond retries fire, so
 * the transport can force a fresh WireGuard handshake the moment the link dies
 * instead of waiting out a stale session. No-op before the transport is up.
 */
  void dcs_notify_priority_health(bool healthy);
  /** Per-machine-target variant (see microlink_notify_peer_health). */
  void dcs_notify_peer_health(uint32_t vpn_ip, bool healthy);
  void dcs_untrack_peer_health(uint32_t vpn_ip);

  /**
 * @brief Current Tailscale VPN IP (host byte order), 0 until registered.
 * Used by main.c to source-bind the pstop socket when the peer is a
 * Tailscale address, pinning egress to the WG netif.
 */
  uint32_t dcs_get_vpn_ip(void);

#ifdef __cplusplus
}
#endif
