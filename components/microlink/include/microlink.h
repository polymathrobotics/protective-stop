// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file microlink.h
 * @brief MicroLink v2 - ESP32 Tailscale Client (Public API)
 *
 * Production-ready Tailscale client for ESP32-S3 with:
 * - Queue-based architecture (no mutex deadlocks)
 * - Dedicated tasks for DERP TX, network I/O, coordination, WireGuard
 * - Rate-limited DISCO (matching native tailscaled timing)
 * - Async STUN (non-blocking)
 * - PSRAM-optimized memory layout
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /* Opaque handle */
  typedef struct microlink_s microlink_t;

  /* Configuration */
  typedef struct
  {
    const char * auth_key; /* Tailscale auth key (tskey-auth-...) */
    const char * device_name; /* Device hostname on the tailnet */
    bool enable_derp; /* Enable DERP relay (default: true) */
    bool enable_stun; /* Enable STUN endpoint discovery */
    bool enable_disco; /* Enable DISCO NAT traversal */
    uint8_t max_peers; /* Max simultaneous peers (default: 16) */
    int8_t wifi_tx_power_dbm; /* WiFi TX power in dBm (0 = default 19.5) */

    /* Priority peer: guaranteed a WG slot even when peer table is full.
     * On large tailnets the NVS cache can fill the peer table at boot
     * before the priority peer arrives from MapResponse. When the table
     * is full and a peer matching this IP arrives, the least-recently-used
     * non-priority peer is evicted to make room.
     * Set to 0 to disable (all peers treated equally). */
    uint32_t priority_peer_ip; /* VPN IP in host byte order (e.g., microlink_parse_ip("100.x.y.z")) */

    /* Optional timing overrides (0 = use defaults) */
    uint32_t disco_heartbeat_ms; /* DISCO keepalive interval (default: 3000) */
    uint32_t stun_interval_ms; /* STUN re-probe interval (default: 23000) */
    uint32_t ctrl_watchdog_ms; /* Control plane watchdog timeout (default: 120000) */
  } microlink_config_t;

  /* Peer info (read-only snapshot) */
  typedef struct
  {
    uint32_t vpn_ip;
    char hostname[64];
    uint8_t public_key[32];
    bool online;
    bool direct_path; /* true if communicating via direct UDP */
    uint16_t derp_region; /* learned DERP home region (0 = unknown) — diagnostic */
  } microlink_peer_info_t;

  /* Connection state */
  typedef enum
  {
    ML_STATE_IDLE = 0,
    ML_STATE_WIFI_WAIT,
    ML_STATE_CONNECTING,
    ML_STATE_REGISTERING,
    ML_STATE_CONNECTED,
    ML_STATE_RECONNECTING,
    ML_STATE_ERROR,
  } microlink_state_t;

  /* Callback types */
  typedef void (*microlink_state_cb_t)(microlink_t * ml, microlink_state_t state, void * user_data);
  typedef void (*microlink_peer_cb_t)(microlink_t * ml, const microlink_peer_info_t * peer, void * user_data);
  typedef void (*microlink_data_cb_t)(
    microlink_t * ml, uint32_t src_ip, const uint8_t * data, size_t len, void * user_data);

  /* Application hook asked, when the WG peer table is FULL, whether an incoming
   * peer must be kept anyway. Returning true makes microlink pin the peer (it
   * evicts an LRU non-pinned peer to make room and persists the pin for future
   * netmap syncs). Used by machn to keep operator-allowlist remotes that would
   * otherwise be trimmed by the ML_MAX_PEERS cap and never learn the remote's
   * WG key. Must be side-effect-free and fast (called on the coord->wg path).
   * The application must only return true for a BOUNDED set of peers. */
  typedef bool (*microlink_peer_wanted_cb_t)(void * ctx, const char * hostname, uint32_t vpn_ip);

  /**
 * @brief Factory reset — erase all stored keys and cached peers
 * @return ESP_OK on success
 *
 * Must be called BEFORE microlink_init(). Erases:
 * - Machine key, WireGuard key, DISCO key (NVS namespace "microlink")
 * - Cached peer data (NVS namespace "ml_peers")
 * After reset, next microlink_init() will generate fresh keys.
 */
  esp_err_t microlink_factory_reset(void);

  /**
 * @brief Initialize MicroLink
 * @param config Configuration (copied internally)
 * @return Handle on success, NULL on failure
 *
 * Creates all internal tasks, queues, and event groups.
 * Does NOT start connecting - call microlink_start() for that.
 */
  microlink_t * microlink_init(const microlink_config_t * config);

  /**
 * @brief Start connecting to Tailscale
 * @param ml Handle from microlink_init()
 * @return ESP_OK on success
 *
 * WiFi must be connected before calling this.
 * Connection proceeds asynchronously - use callbacks or poll state.
 */
  esp_err_t microlink_start(microlink_t * ml);

  /**
 * @brief Rebind to a new network interface without destroying the session
 * @param ml Handle
 * @return ESP_OK on success
 *
 * Use this when the active uplink interface changes.
 * Closes and reopens all sockets on the new interface while preserving:
 * - WireGuard peer state and crypto keys
 * - Peer table and DISCO discovery state
 * - VPN IP assignment
 * - Task state machines
 *
 * The coord and DERP connections will reconnect automatically (~5-10s).
 * Much faster than stop/destroy/init/start which requires full re-registration
 * and MapResponse re-download.
 */
  esp_err_t microlink_rebind(microlink_t * ml);

  /**
 * @brief Stop and disconnect from Tailscale
 * @param ml Handle
 * @return ESP_OK on success
 *
 * Gracefully shuts down all tasks and closes connections.
 */
  esp_err_t microlink_stop(microlink_t * ml);

  /**
 * @brief Destroy MicroLink instance and free all resources
 * @param ml Handle (NULL-safe)
 */
  void microlink_destroy(microlink_t * ml);

  /**
 * @brief Get current connection state
 */
  microlink_state_t microlink_get_state(const microlink_t * ml);

  /**
 * @brief Count of control-plane connects. 1 = first connect after boot; each
 * higher value is a re-connect (soft link flap). ml_reconnects in /state.json
 * is this minus 1. Used for soak monitoring of DERP/control-plane stability.
 */
  uint32_t microlink_get_connect_count(const microlink_t * ml);

  /**
 * @brief Check if connected and ready to send/receive
 */
  bool microlink_is_connected(const microlink_t * ml);

  /**
 * @brief Get our assigned VPN IP
 * @return VPN IP in host byte order, 0 if not yet assigned
 */
  uint32_t microlink_get_vpn_ip(const microlink_t * ml);

  /** Public egress IP (host byte order) as seen via STUN; 0 until STUN completes.
 *  Behind NAT this is the site's egress — useful for rough management-side geolocation. */
  uint32_t microlink_get_public_ip(const microlink_t * ml);

  /** Effective DERP home region the single relay connection is actually on
   *  (0 = unset). The runtime lock wins, else the learned/rehomed home. Coarse
   *  locality hint AND the region the UI shows as live. */
  uint16_t microlink_get_derp_region(const microlink_t * ml);

  /** Configured DERP-region LOCK: 0 = auto, else the region the chip is pinned
   *  to home on (the persisted override). Distinct from
   *  microlink_get_derp_region() (the region in use) — the two differ only
   *  transiently while a fresh lock re-homes. Lets a UI show AUTO vs LOCKED-to-N
   *  without guessing. Guards against a repeat of the region-9 (dfw) misroute. */
  uint16_t microlink_get_derp_region_locked(const microlink_t * ml);

  /* ==========================================================================
 * DERP region auto-negotiation (docs/DERP_REGION_AUTONEGOTIATION_DESIGN.md
 * §4.2/§6/§7/§8). EXTENSION of the existing single-home + multi-region-relay
 * + lock machinery: the lock (microlink_get_derp_region_locked) always wins
 * (I2); with no live safety bond the legacy immediate rehome is kept verbatim
 * (§6); only a live-bond region change goes through the make-before-break
 * switch (§7), and every switch is damped (§8).
 * ========================================================================== */

/** Max machine slots the primary-machine callback can report. Sized above
 *  DCS_PSTOP_MAX_MACHINES (4) so the app-side table can grow without an ABI
 *  break here. */
#define MICROLINK_PRIMARY_MAX_MACHINES 8

  /** Snapshot of the app's machine-slot table, filled by the primary-machine
 *  callback. microlink derives the DERP re-home target from it (§4.2):
 *  armed_primary_ip (the LOWEST-index bonded+armed slot — Q1 decision:
 *  deterministic across reboots, unlike arming history) wins; otherwise the
 *  first configured slot whose DERP region is already learned; otherwise the
 *  legacy static priority_peer_ip config. All IPs host byte order, 0 = none. */
  typedef struct
  {
    uint32_t armed_primary_ip; /* lowest bonded+ARMED machine slot; 0 = none armed */
    uint32_t configured_ips[MICROLINK_PRIMARY_MAX_MACHINES]; /* configured slots, slot order */
    int configured_count;
  } microlink_primary_machine_info_t;

  typedef void (*microlink_primary_machine_cb_t)(void * ctx, microlink_primary_machine_info_t * out);

  /**
 * @brief Register the primary-machine feed for DERP region auto-negotiation.
 *
 * Optional; remotes with a machine-slot table register it (dcs_support).
 * Unset (default — machines, plain builds) preserves today's behavior exactly:
 * the re-home target is the single configured priority_peer_ip with the
 * management-server fallback. The callback must be cheap and lock-free (it is
 * invoked from the wg_mgr task's 3 s self-heal cadence — atomics only).
 */
  void microlink_set_primary_machine_cb(microlink_t * ml, microlink_primary_machine_cb_t cb, void * ctx);

  /**
 * @brief Offer a fleet region advisory (Phase 3 auto-apply, design §6/§13.1).
 *
 * Called by the fleet check-in path when the backend response carries a
 * region_advice object. ADVISORY ONLY (I3): the chip gates it on a strictly
 * increasing epoch (enforced here), a TTL (expired advice is inert), AUTO mode
 * (a lock fully disables it), a configured-machine check (a remote with any
 * machine slot ignores advice — the netmap is fresher and machine-
 * authoritative), a 30-min chip-side min-apply interval and full §8 damping +
 * §7 proving. A wrong/malicious advice can at worst waste a few damped,
 * proof-gated aux TLS handshakes. Safe from any task.
 */
  void microlink_offer_region_advice(microlink_t * ml, uint16_t region, uint32_t epoch, uint32_t ttl_s);

  /** Who currently owns the chip's DERP home-region choice (Q2 UI/API surfacing). */
  typedef enum
  {
    MICROLINK_REGION_SRC_AUTO = 0, /* server default / management anchor — no auto-selection applied */
    MICROLINK_REGION_SRC_LOCKED = 1, /* operator lock (derp_region_override) */
    MICROLINK_REGION_SRC_AUTO_PRIMARY = 2, /* auto: primary machine's region (§4.2) */
    MICROLINK_REGION_SRC_AUTO_FLEET = 3, /* auto: fleet region_advice applied (Phase 3) */
  } microlink_region_source_t;

  /** Auto-negotiation status + telemetry (§16), for /state.json, the UI badge
 *  and /admin/api/monitor. All counters since boot. */
  typedef struct
  {
    uint8_t source; /* microlink_region_source_t */
    uint16_t auto_applied_region; /* region last applied by the auto-negotiator (0 = never) */
    uint32_t auto_apply_count; /* auto region applies (legacy rehomes + MBB commits) */
    uint32_t last_auto_apply_s; /* uptime seconds of the last auto apply (0 = never) */
    uint8_t mbb_state; /* 0 IDLE, 1 AUX_OPENING, 2 PROVING */
    uint16_t mbb_pending_region; /* pending MBB target (0 = none) */
    uint32_t mbb_commits;
    uint32_t mbb_rollbacks;
    uint32_t mbb_proofs_ok;
    uint32_t mbb_proofs_failed;
    uint32_t damping_suppressed; /* negotiator passes suppressed by §8 damping */
    uint32_t cooldown_trips; /* hourly circuit-breaker trips */
    uint32_t switches_1h; /* committed switches in the rolling hour */
  } microlink_region_autoneg_t;

  void microlink_get_region_autoneg(microlink_t * ml, microlink_region_autoneg_t * out);

  /** Stable string for microlink_region_source_t:
 *  "auto" | "locked" | "auto-primary" | "auto-fleet". */
  const char * microlink_region_source_str(uint8_t source);

  /**
 * @brief Report application-level health of the priority-peer link.
 *
 * The transport layer cannot see whether an encrypted application protocol is
 * actually being answered — a WireGuard session can read "up" (valid keypair)
 * while the far end has forgotten us after a restart, silently blackholing
 * traffic until the keypair expires (~2-3 min). Call this with @p healthy=false
 * as soon as the app detects the priority peer has stopped responding (e.g. a
 * heartbeat-reply watchdog), and true again once replies resume. On false the
 * priority-peer wake forces a fresh 1-RTT handshake instead of waiting out the
 * stale session, cutting recovery from minutes to seconds. No-op if no
 * priority peer is configured. Safe to call from any task.
 */
  void microlink_notify_priority_health(microlink_t * ml, bool healthy);

  /**
 * @brief Report health of the management/OTA server link.
 *
 * Like microlink_notify_priority_health() but for the management server: call with
 * false when a check-in fails to reach the backend, true on success. On
 * false the management peer's disco-first wake forces a fresh handshake through a
 * stale/zombie session (e.g. after a reboot the far end kept our persisted
 * keypair). No-op if no management server is configured. Safe from any task.
 */
  void microlink_notify_fleet_health(microlink_t * ml, bool healthy);

  /**
 * @brief Get number of known peers
 */
  int microlink_get_peer_count(const microlink_t * ml);

  /**
 * @brief Get peer info by index
 * @param ml Handle
 * @param index Peer index (0 to peer_count-1)
 * @param info Output peer info (copied)
 * @return ESP_OK if valid index
 */
  esp_err_t microlink_get_peer_info(const microlink_t * ml, int index, microlink_peer_info_t * info);

  /**
 * @brief Send UDP data to a peer by VPN IP
 * @param ml Handle
 * @param dest_vpn_ip Destination VPN IP (host byte order)
 * @param data Payload
 * @param len Payload length (max 1400 bytes)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if send queue full
 */
  esp_err_t microlink_send(microlink_t * ml, uint32_t dest_vpn_ip, const uint8_t * data, size_t len);

  /**
 * @brief Register callbacks
 */
  void microlink_set_state_callback(microlink_t * ml, microlink_state_cb_t cb, void * user_data);
  void microlink_set_peer_callback(microlink_t * ml, microlink_peer_cb_t cb, void * user_data);
  void microlink_set_data_callback(microlink_t * ml, microlink_data_cb_t cb, void * user_data);

  /**
 * @brief Register the "keep this peer even when the table is full" hook.
 *
 * Optional. When set, add_peer() consults it before dropping an incoming peer
 * that hit the ML_MAX_PEERS cap; a true return pins the peer (LRU-evicting a
 * non-pinned peer to make room). Unset (default) preserves current behavior.
 */
  void microlink_set_peer_wanted_cb(microlink_t * ml, microlink_peer_wanted_cb_t cb, void * ctx);

  /**
 * @brief Set/get the DERP I/O task's inner-loop yield delay (1..100 ms).
 *
 * Default is 1 ms (highest responsiveness on the DERP relay path, ~9 % core 0
 * CPU steady-state). Larger values cut CPU roughly linearly at the cost of
 * TX queue drain / RX socket polling latency. Useful when responsiveness on
 * other tasks matters more than DERP throughput. Range-checked; returns
 * ESP_ERR_INVALID_ARG outside [1,100].
 */
  esp_err_t microlink_set_derp_loop_delay_ms(int ms);
  int microlink_get_derp_loop_delay_ms(void);

  /**
 * @brief Pause / resume the DERP I/O task at runtime.
 *
 * When paused, the DERP task skips its loop body each iteration (100 ms yield,
 * continue). The connection state and TLS context are left alone — caller can
 * resume cleanly. Safer than vTaskSuspend, which can leave mutexes held.
 *
 * While paused: no DERP heartbeats, no relayed traffic. Direct-path WG still
 * works. Tailscale peers reachable only via direct path or DERP-server-side
 * timeout (~60 s) until resumed.
 */
  void microlink_pause_derp(bool paused);
  bool microlink_is_derp_paused(void);

  /**
 * @brief Lightweight diagnostic snapshots.
 *
 * @return RSSI of the associated AP in dBm, or 0 if not associated.
 */
  int microlink_get_rssi_dbm(void);

  /**
 * @brief Free-heap snapshot. Either argument may be NULL to skip it.
 */
  esp_err_t microlink_get_heap_info(uint32_t * free_total, uint32_t * largest_block);

  /**
 * @brief Convert VPN IP to string
 */
  void microlink_ip_to_str(uint32_t ip, char * buf);

  /**
 * @brief Parse IP string "A.B.C.D" to host byte order uint32
 * @return IP in host byte order, 0 on error
 */
  uint32_t microlink_parse_ip(const char * ip_str);

  /**
 * @brief Get default device name based on MAC address
 * @return Static string like "esp32-a1b2c3"
 */
  const char * microlink_default_device_name(void);

  /**
 * @brief Pin (or unpin) a VPN IP in the WireGuard peer table.
 *
 * Pinned peers survive the ML_MAX_PEERS trim and evict an LRU peer to
 * enter a full table. Register every peer the SAFETY path depends on
 * (e.g. all configured pstop machine targets). Up to 8 extra pins.
 */
  void microlink_pin_peer_ip(microlink_t * ml, uint32_t vpn_ip, bool pin);

  /**
 * @brief Per-peer variant of microlink_notify_priority_health(): report the
 * application-level health of any pinned peer (e.g. each pstop machine
 * target). While false, that peer's session wake forces fresh handshakes,
 * recovering a rebooted/forgotten far end in seconds. Up to 8 peers.
 */
  void microlink_notify_peer_health(microlink_t * ml, uint32_t vpn_ip, bool healthy);

  /** Remove a peer from the health registry (e.g. a remote that unbonded and
 * aged out) so it stops receiving heartbeat pings. */
  void microlink_untrack_peer_health(microlink_t * ml, uint32_t vpn_ip);

  /**
 * @brief Ask the coord task to re-announce this node to the control plane
 * (a Stream=false endpoint-update MapRequest on the existing connection).
 *
 * Use when the app suspects the FAR side no longer knows us — e.g. a
 * machine that rebooted and never received its netmap silently ignores our
 * WG handshakes (sustained ENOTCONN sends, 2026-08-08 incident); the
 * re-announce nudges the control plane to push us to that peer's map
 * stream. Non-blocking and bounded: safe to call from a safety loop, but
 * rate-limit escalations at the call site.
 */
  esp_err_t microlink_request_announce(microlink_t * ml);

  /**
 * @brief Disco-layer path RTT to a peer (txid-matched ping->pong).
 * @return RTT in ms, 0 if unknown/never measured. age_ms_out = staleness
 * of the sample; direct_out = whether it was measured over the direct path.
 */
  uint32_t microlink_get_peer_rtt(microlink_t * ml, uint32_t vpn_ip, uint32_t * age_ms_out, bool * direct_out);

  /** On-device perf micro-benchmark (x25519 + AEAD); writes JSON. */
  int microlink_perf_bench(int iters, char * out, size_t cap);

  /* ============================================================================
 * MagicDNS — Resolve Tailnet hostnames to VPN IPs
 *
 * Resolves short or FQDN hostnames (e.g., "npc1", "npc1.tail12345.ts.net")
 * against the known peer list. No network calls — lookup only.
 * ========================================================================== */

  /**
 * @brief Resolve a tailnet hostname to its VPN IP
 * @param ml Handle
 * @param hostname Short name ("npc1") or FQDN ("npc1.tail12345.ts.net")
 * @return VPN IP in host byte order, 0 if not found
 *
 * Matching rules (in order):
 * 1. Exact match against full peer hostname
 * 2. Prefix match: "npc1" matches "npc1.tail12345.ts.net"
 * 3. Case-insensitive on all matches
 */
  uint32_t microlink_resolve(const microlink_t * ml, const char * hostname);

  /* ============================================================================
 * UDP Socket API
 *
 * Provides simple UDP send/receive over the Tailscale VPN tunnel.
 * Packets are routed through WireGuard for encryption.
 * ========================================================================== */

  /* Opaque UDP socket handle */
  typedef struct microlink_udp_socket microlink_udp_socket_t;

  /* UDP receive callback (called from RX task context) */
  typedef void (*microlink_udp_rx_cb_t)(
    microlink_udp_socket_t * sock,
    uint32_t src_ip,
    uint16_t src_port,
    const uint8_t * data,
    size_t len,
    void * user_data);

  /**
 * @brief Create a UDP socket bound to the WireGuard VPN IP
 * @param ml Handle
 * @param local_port Port to bind (0 = auto-assign)
 * @return Socket handle, NULL on failure
 *
 * On creation, sends CallMeMaybe to all peers to trigger WG handshakes.
 */
  microlink_udp_socket_t * microlink_udp_create(microlink_t * ml, uint16_t local_port);

  /**
 * @brief Close UDP socket and free resources
 */
  void microlink_udp_close(microlink_udp_socket_t * sock);

  /**
 * @brief Send UDP data to a peer
 * @param sock Socket handle
 * @param dest_ip Destination VPN IP (host byte order)
 * @param dest_port Destination port
 * @param data Payload
 * @param len Payload length (max 1400)
 * @return ESP_OK on success
 */
  esp_err_t microlink_udp_send(
    microlink_udp_socket_t * sock, uint32_t dest_ip, uint16_t dest_port, const void * data, size_t len);

  /**
 * @brief Receive UDP data (blocking with timeout)
 * @param sock Socket handle
 * @param src_ip Output source VPN IP (can be NULL)
 * @param src_port Output source port (can be NULL)
 * @param buffer Output buffer
 * @param len In: buffer size, Out: bytes received
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if timed out
 */
  esp_err_t microlink_udp_recv(
    microlink_udp_socket_t * sock,
    uint32_t * src_ip,
    uint16_t * src_port,
    void * buffer,
    size_t * len,
    uint32_t timeout_ms);

  /**
 * @brief Register receive callback for immediate packet handling
 * @param sock Socket handle
 * @param cb Callback (NULL to clear)
 * @param user_data Passed to callback
 */
  esp_err_t microlink_udp_set_rx_callback(microlink_udp_socket_t * sock, microlink_udp_rx_cb_t cb, void * user_data);

  /**
 * @brief Get local bound port
 */
  uint16_t microlink_udp_get_local_port(const microlink_udp_socket_t * sock);

  /* ============================================================================
 * TCP Socket API
 *
 * Provides TCP connections over the Tailscale VPN tunnel.
 * Traffic is routed through WireGuard — standard BSD TCP sockets
 * over the encrypted tunnel. Works with any TCP service on a peer
 * (HTTP, Traccar, MQTT, custom protocols, etc).
 * ========================================================================== */

  /* Opaque TCP socket handle */
  typedef struct microlink_tcp_socket microlink_tcp_socket_t;

  /**
 * @brief Connect TCP to a peer over the VPN tunnel
 * @param ml Handle
 * @param dest_ip Destination VPN IP (host byte order)
 * @param dest_port Destination port
 * @param timeout_ms Connection timeout in ms (0 = default 15s)
 * @return Socket handle, NULL on failure
 *
 * Automatically triggers WG handshake if tunnel is not yet established.
 * Retries once if the initial connect fails due to tunnel not ready.
 */
  microlink_tcp_socket_t * microlink_tcp_connect(
    microlink_t * ml, uint32_t dest_ip, uint16_t dest_port, uint32_t timeout_ms);

  /**
 * @brief Send data over TCP connection
 * @param sock Socket handle
 * @param data Payload
 * @param len Payload length
 * @return ESP_OK on success, ESP_FAIL on error
 *
 * Blocks until all data is sent or an error occurs.
 */
  esp_err_t microlink_tcp_send(microlink_tcp_socket_t * sock, const void * data, size_t len);

  /**
 * @brief Receive data from TCP connection
 * @param sock Socket handle
 * @param buffer Output buffer
 * @param len Buffer size
 * @param timeout_ms Timeout in ms (0 = use socket default)
 * @return Bytes received (>0), 0 on timeout, -1 on error/disconnect
 */
  int microlink_tcp_recv(microlink_tcp_socket_t * sock, void * buffer, size_t len, uint32_t timeout_ms);

  /**
 * @brief Check if TCP connection is still alive
 */
  bool microlink_tcp_is_connected(const microlink_tcp_socket_t * sock);

  /**
 * @brief Close TCP connection and free resources
 */
  void microlink_tcp_close(microlink_tcp_socket_t * sock);

#ifdef __cplusplus
}
#endif
