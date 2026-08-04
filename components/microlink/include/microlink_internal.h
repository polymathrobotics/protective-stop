// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file microlink_internal.h
 * @brief MicroLink v2 Internal Types and Task Communication
 *
 * Architecture: 5 FreeRTOS tasks communicating via queues and event groups.
 * No shared mutable state between tasks - each owns its data exclusively.
 *
 * Tasks:
 *   net_io   (Core 0, pri 6)  - Unified select() on all sockets
 *   derp_tx  (Core 0, pri 7)  - Sole DERP TLS writer
 *   coord    (Core 1, pri 5)  - Control plane (Noise, HTTP/2, registration)
 *   wg_mgr   (Core 1, pri 7)  - WireGuard + DISCO + peer management
 *   app      (unpinned, pri 3) - User application (external, not ours)
 */

#pragma once

#include "esp_heap_caps.h"
// clang-format off
#include "freertos/FreeRTOS.h"  // must precede other freertos/ headers
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// clang-format on
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ssl.h"
#include "microlink.h"
#include "ml_config_httpd.h"
#include "sdkconfig.h"

#ifdef CONFIG_ML_ZERO_COPY_WG
  #include "lwip/pbuf.h"
  #include "lwip/tcpip.h"
  #include "lwip/udp.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

/* Task configuration */
#define ML_TASK_NET_IO_STACK (8 * 1024)
#define ML_TASK_NET_IO_PRIO 7
#define ML_TASK_NET_IO_CORE 0

#define ML_TASK_DERP_TX_STACK (14 * 1024)
#define ML_TASK_DERP_TX_PRIO 5
#define ML_TASK_DERP_TX_CORE 0

#define ML_TASK_COORD_STACK (12 * 1024)
#define ML_TASK_COORD_PRIO 5
#define ML_TASK_COORD_CORE 1

#define ML_TASK_WG_MGR_STACK \
  (CONFIG_ML_WG_MGR_STACK_KB * 1024) /* v15.15: default bumped 8->16 KiB.
 * With ML_ZERO_COPY_WG off, every direct-UDP WG packet runs the full
 * wireguardif_network_rx -> decrypt -> ip_input -> icmp_echo ->
 * ip4_output -> wg_netif_output -> wireguardif_output_to_peer ->
 * encrypt -> udp_sendto chain on this task's stack. 8 KiB tripped the
 * canary around 60 s of sustained 2 Hz Tailscale ping load (verbose
 * logging on). 16 KiB has comfortable headroom. */
#define ML_TASK_WG_MGR_PRIO 7
#define ML_TASK_WG_MGR_CORE 1

/* Queue depths */
#define ML_DERP_TX_QUEUE_DEPTH 16
/* Separate FIFO queue for SAFETY (pinned/priority/health-tracked) + WG-handshake
 * frames, drained FIRST by the DERP task so the 5 Hz pstop heartbeat is never
 * stuck behind, or dropped by, a disco relay burst. Small: a handful of safety
 * peers, each 5 Hz. */
#define ML_DERP_TX_PRIO_DEPTH 12
#define ML_DISCO_RX_QUEUE_DEPTH 8
#define ML_WG_RX_QUEUE_DEPTH \
  32 /* carries the 10 Hz pstop heartbeats;
                                     * 8 was sheddable by one disco burst */
#define ML_STUN_RX_QUEUE_DEPTH 4
#define ML_COORD_CMD_QUEUE_DEPTH 4
#define ML_PEER_UPDATE_QUEUE_DEPTH 400

/* Protocol limits */
#define ML_MAX_PEERS CONFIG_ML_MAX_PEERS
#define ML_MAX_ENDPOINTS 8
#define ML_MAX_PACKET_SIZE 1500
#define ML_DERP_MAX_FRAME (ML_MAX_PACKET_SIZE + 64)

/* DERP */
#define ML_DERP_REGION 9 /* Dallas (dfw) */
#define ML_DERP_HOST "derp9e.tailscale.com"
#define ML_DERP_PORT 443

/* Multi-region DERP relay pool. Slot 0 is the HOME connection (the chip's own
 * PreferredDERP region, kept alive for inbound reachability). Slots 1..N are
 * auxiliary connections opened on demand for the DISTINCT home regions of the
 * few safety peers (pinned / priority / health-tracked) so the chip can relay a
 * WG-init/DISCO-ping to a peer homed on a different region — a DERP server only
 * relays between peers connected to IT (root cause of the cross-region errno-128
 * bond failure). 6 slots covers home + up to 5 distinct safety-peer regions. */
#define ML_DERP_MAX_CONNS 6
#define ML_DERP_AUX_IDLE_REAP_MS 120000 /* reap an unused aux conn after 120 s idle */

/* Tailscale control plane */
#define ML_CTRL_HOST "controlplane.tailscale.com"
#define ML_CTRL_PORT 443
#define ML_CTRL_PROTOCOL_VER 131

/* DISCO timing — slowed ~10x from tailscaled defaults. The original 3 s
 * heartbeat + 15 s upgrade-probe sweep is fine on Linux with surplus CPU,
 * but on ESP32 each fire costs ~36 ms of Curve25519/ChaCha20 plus a UDP
 * round-trip, and during that window lwIP starves ICMP/HTTP traffic to
 * ~400-700 ms latency spikes every 10-15 s. Slowing both ~10x keeps us
 * inside Tailscale's 60 s session-active timeout (peers don't drop) while
 * eliminating most of the spike cadence. Values match the
 * trinity-sign-no-psram-fixes branch, which reported 3x curl reliability
 * improvement on a no-PSRAM ESP32. */
#define ML_DISCO_PING_INTERVAL_MS 5000
#define ML_DISCO_HEARTBEAT_MS 30000
#define ML_DISCO_TRUST_DURATION_MS 60000
#define ML_DISCO_PING_TIMEOUT_MS 5000
#define ML_DISCO_UPGRADE_INTERVAL_MS 60000

/* Priority-peer path liveness (2026-07-21 failover speedup): the peer named
 * by priority_peer_ip gets a tighter disco heartbeat, a pong-recency
 * demotion watchdog (direct->DERP when pongs stop, instead of waiting out
 * the 60 s trust lease), and a faster direct re-probe while demoted. Only
 * the priority peer — the 16-peer tailnet load profile is unchanged. */
#define ML_DISCO_PRIORITY_HB_MS 3000
#define ML_DISCO_PRIORITY_PATH_DEAD_MS \
  4000 /* was 8000: bound autonomous
                                                * direct->DERP failover to <5s.
                                                * One missed 3s pong under jitter
                                                * can demote a live path, but that
                                                * just moves to DERP and re-probe
                                                * restores direct in ~5s — no
                                                * safety impact (dual-path send
                                                * keeps the uplink gapless anyway). */
#define ML_DISCO_PRIORITY_REPROBE_MS 5000

/* 100+ peer scaling bounds (2026-07-21): pace CPU-heavy per-peer work so a
 * large tailnet can never monopolize the wg_mgr loop that also drains the
 * pstop heartbeat queue. The priority peer is exempt from every one of
 * these budgets. See docs/PEER_SCALING_DESIGN (pstop repo). */
#define ML_PEER_ADDS_PER_PASS 4 /* MapResponse ingest: adds per 10ms pass */
#define ML_DISCO_OPENS_PER_PASS 4 /* inbound disco NaCl box-opens per pass */
#define ML_CMM_SENDS_PER_PASS 2 /* post-STUN CallMeMaybe broadcast pacing */
#define ML_HB_SEALS_PER_TICK 8 /* heartbeat pings per 1s probe tick */
#define ML_DISCO_SESSION_ACTIVE_MS 120000

/* STUN servers (Tailscale primary, Google fallback) */
#define ML_STUN_PRIMARY_HOST "derp9.tailscale.com"
#define ML_STUN_PRIMARY_PORT 3478
#define ML_STUN_FALLBACK_HOST "stun.l.google.com"
#define ML_STUN_FALLBACK_PORT 19302
#define ML_STUN_MAX_RETRIES 3
#define ML_STUN_RETRY_INTERVAL_MS 2000

/* STUN timing */
#define ML_STUN_RETRANSMIT_MS 100
#define ML_STUN_TOTAL_TIMEOUT_MS 5000
#define ML_STUN_RESTUN_INTERVAL_MS 23000

/* Control plane timing */
#define ML_CTRL_WATCHDOG_MS 120000
#define ML_CTRL_BACKOFF_MAX_MS 30000
#define ML_CTRL_KEEPALIVE_MS 60000

/* Large tailnet buffer sizes (PSRAM-allocated, configurable via menuconfig) */
#define ML_H2_BUFFER_SIZE (CONFIG_ML_H2_BUFFER_SIZE_KB * 1024)
#define ML_JSON_BUFFER_SIZE (CONFIG_ML_JSON_BUFFER_SIZE_KB * 1024)

/* Noise protocol */
#define ML_NOISE_KEY_LEN 32
#define ML_NOISE_MAC_LEN 16
#define ML_NOISE_NONCE_LEN 12
#define ML_NOISE_HASH_LEN 32

  /* ============================================================================
 * Zero-Copy WG Types (Kconfig: CONFIG_ML_ZERO_COPY_WG)
 *
 * When enabled, the DISCO UDP socket uses a raw lwIP PCB callback instead of
 * a BSD socket. WG packets go directly to wireguardif_network_rx() (zero copy).
 * DISCO packets are buffered in a lock-free SPSC ring for the wg_mgr task.
 * ========================================================================== */

#ifdef CONFIG_ML_ZERO_COPY_WG

  #define ML_ZC_DISCO_RING_SIZE 16 /* Must be power of 2 */
  #define ML_ZC_DISCO_MAX_PKT 256 /* Max DISCO packet size in ring */
  #define ML_ZC_TX_POOL_SIZE 24 /* Outbound send pool depth */

  /* SPSC ring entry for DISCO packets (PCB callback → wg_mgr task) */
  typedef struct
  {
    uint8_t data[ML_ZC_DISCO_MAX_PKT];
    uint16_t len;
    uint32_t src_ip_nbo; /* Network byte order (from lwIP) */
    uint16_t src_port; /* Host byte order (from lwIP) */
  } ml_zc_disco_entry_t;

  /* Outbound TX context for tcpip_callback send */
  typedef struct
  {
    struct udp_pcb * pcb;
    uint8_t data[ML_MAX_PACKET_SIZE];
    uint16_t len;
    ip_addr_t dest;
    uint16_t port;
  } ml_zc_tx_ctx_t;

  /* Zero-copy state embedded in microlink_s */
  typedef struct
  {
    struct udp_pcb * pcb; /* Raw UDP PCB (replaces disco_sock4) */
    uint16_t local_port; /* Bound port */

    /* DISCO RX ring buffer (lock-free SPSC) */
    ml_zc_disco_entry_t rx_ring[ML_ZC_DISCO_RING_SIZE];
    volatile uint8_t rx_head; /* Written by tcpip_thread */
    volatile uint8_t rx_tail; /* Written by wg_mgr task */

    /* TX send pool (microlink → tcpip_thread) */
    ml_zc_tx_ctx_t tx_pool[ML_ZC_TX_POOL_SIZE];
    volatile uint8_t tx_head; /* Written by wg_mgr task */
    volatile uint8_t tx_tail; /* Written by tcpip_thread */
  } ml_zerocopy_t;

#endif /* CONFIG_ML_ZERO_COPY_WG */

  /* ============================================================================
 * Event Group Bits
 * ========================================================================== */

#define ML_EVT_WIFI_CONNECTED BIT0
#define ML_EVT_COORD_REGISTERED BIT1
#define ML_EVT_DERP_CONNECTED BIT2
#define ML_EVT_WG_READY BIT3
#define ML_EVT_PEERS_AVAILABLE BIT4
#define ML_EVT_STUN_COMPLETE BIT5
#define ML_EVT_SHUTDOWN_REQUEST BIT6
#define ML_EVT_DERP_RECONNECT BIT7
#define ML_EVT_DERP_CONNECT_REQ BIT8

  /* ============================================================================
 * Queue Message Types
 * ========================================================================== */

  /* DERP TX queue item - packet to send via DERP relay */
  typedef struct
  {
    uint8_t dest_pubkey[32]; /* Destination peer's public key */
    uint8_t * data; /* Heap-allocated payload (caller frees on failure) */
    size_t len; /* Payload length */
    uint8_t frame_type; /* DERP frame type (0x04 = SendPacket) */
    uint16_t region_id; /* DERP home region of the destination peer; 0 = unknown
                         * -> route on the HOME connection (slot 0). Resolved at
                         * enqueue time from the peer table so the DERP task can
                         * egress the frame on the pool conn homed on THIS region. */
  } ml_derp_tx_item_t;

  /* Received packet (from net_io to disco/wg queues) */
  typedef struct
  {
    uint8_t * data; /* Heap-allocated packet data */
    size_t len; /* Packet length */
    uint32_t src_ip; /* Source IP (for UDP packets) */
    uint16_t src_port; /* Source port (for UDP packets) */
    uint8_t src_pubkey[32]; /* Source peer key (for DERP packets) */
    bool via_derp; /* true if received via DERP, false if direct UDP */
  } ml_rx_packet_t;

  /* Coordination command */
  typedef enum
  {
    ML_CMD_CONNECT, /* Start registration */
    ML_CMD_DISCONNECT, /* Graceful disconnect */
    ML_CMD_UPDATE_ENDPOINTS, /* Send endpoint update to control plane */
    ML_CMD_FORCE_RECONNECT, /* Force reconnection (after DERP failure, etc.) */
  } ml_coord_cmd_t;

  /* Peer update (from coord to wg_mgr) */
  typedef struct
  {
    enum
    {
      ML_PEER_ADD,
      ML_PEER_REMOVE,
      ML_PEER_UPDATE_ENDPOINT,
    } action;

    uint32_t vpn_ip;
    uint8_t public_key[32];
    uint8_t disco_key[32];
    char hostname[64];
    uint16_t derp_region;

    /* Endpoints */
    struct
    {
      uint32_t ip;
      uint16_t port;
      bool is_ipv6;
    } endpoints[ML_MAX_ENDPOINTS];

    int endpoint_count;
  } ml_peer_update_t;

  /* ============================================================================
 * Peer State (owned exclusively by wg_mgr task)
 * ========================================================================== */

  typedef struct
  {
    /* Identity */
    uint32_t vpn_ip;
    uint8_t public_key[32];
    uint8_t disco_key[32];
    char hostname[64];
    bool active;

    /* Endpoints */
    struct
    {
      uint32_t ip;
      uint16_t port;
      bool is_ipv6;
    } endpoints[ML_MAX_ENDPOINTS];

    int endpoint_count;
    uint16_t derp_region;

    /* DISCO state (rate limiting) */
    uint64_t last_ping_sent_ms; /* Last DISCO ping we sent */
    uint64_t last_pong_recv_ms; /* Last DISCO pong we received */
    uint32_t disco_rtt_ms; /* txid-matched ping->pong RTT (true path metric) */
    bool disco_rtt_direct; /* the measured pong came direct (not via DERP) */
    uint64_t trust_until_ms; /* Direct path trusted until */
    uint64_t last_send_ms; /* Last data sent to this peer */
    uint64_t last_upgrade_ms; /* Last path upgrade attempt */

    /* Best direct path */
    uint32_t best_ip;
    uint16_t best_port;
    bool has_direct_path;

    /* DERP-stability flap damping (safety peers only). A hard-NAT peer can get a
     * lucky direct pong (promote) whose reverse NAT mapping then dies, demoting a
     * few seconds later — an endless 5 s promote/demote swap that wobbles the
     * safety heartbeat. If a direct path lived < 15 s we count it as a flap and
     * exponentially back off the direct-UPGRADE re-probe so the peer holds a
     * steady DERP bond. Never suppresses the 3 s liveness wake or a >15 s
     * established failover; only priority/health-tracked peers ever set these. */
    uint64_t direct_promoted_ms; /* ms of last has_direct_path false->true edge */
    uint8_t direct_flap_count; /* consecutive short-lived direct paths (cap 8) */
    uint64_t direct_backoff_until; /* gate direct-upgrade re-probe until this ms */

    /* Throttle the via-DERP-pong DERP-handshake re-fire to >= 5 s apart so a
     * DERP-only safety peer answering every 3 s ping doesn't drive a connect_derp
     * storm. 0 = never fired. */
    uint64_t last_derp_reconnect_ms;

    /* WireGuard peer index in wireguard-lwip */
    int wg_peer_index;

    /* On-demand handshake: tried once on first DISCO direct path discovery */
    bool tried_initial_handshake;
  } ml_peer_t;

  /* ============================================================================
 * DERP Map Types (parsed from MapResponse, used by coord + STUN)
 * ========================================================================== */

#define ML_MAX_DERP_REGIONS 32
#define ML_MAX_DERP_NODES 4

  typedef struct
  {
    char hostname[64];
    char ipv4[16];
    char ipv6[46];
    uint16_t stun_port; /* 0 = default 3478 */
    uint16_t derp_port; /* 0 = default 443 */
    bool stun_only; /* true if node only serves STUN, not DERP */
  } ml_derp_node_t;

  typedef struct
  {
    uint16_t region_id;
    char code[8]; /* e.g. "dfw", "nyc", "sfo" */
    ml_derp_node_t nodes[ML_MAX_DERP_NODES];
    uint8_t node_count;
    bool avoid; /* true if region should be avoided */
  } ml_derp_region_t;

  /* ============================================================================
 * Noise Protocol State (owned exclusively by coord task)
 * ========================================================================== */

  typedef struct
  {
    uint8_t h[ML_NOISE_HASH_LEN];
    uint8_t ck[ML_NOISE_HASH_LEN];
    uint8_t local_static_private[32];
    uint8_t local_static_public[32];
    uint8_t local_ephemeral_private[32];
    uint8_t local_ephemeral_public[32];
    uint8_t remote_static_public[32];
    uint8_t tx_key[ML_NOISE_KEY_LEN];
    uint8_t rx_key[ML_NOISE_KEY_LEN];
    uint64_t tx_nonce;
    uint64_t rx_nonce;
    bool handshake_complete;
  } ml_noise_state_t;

  /* ============================================================================
 * DERP Connection State
 * ========================================================================== */

  typedef struct
  {
    int sockfd; /* Raw TCP socket */
    mbedtls_ssl_context ssl; /* TLS context (owned exclusively by DERP I/O task) */
    mbedtls_ssl_config ssl_conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    bool connected;
    uint64_t last_recv_ms; /* For keepalive watchdog */

    /* Multi-region pool bookkeeping (all owned by the DERP I/O task) */
    uint16_t region_id; /* DERP region this conn serves; 0 = unused/free slot */
    bool tls_inited; /* the four mbedTLS contexts above are live (must be freed) */
    uint64_t last_connect_attempt_ms; /* per-slot connect backoff timestamp */
    uint64_t last_used_ms; /* last time a frame egressed here (aux reap clock) */
  } ml_derp_conn_t;

  /* ============================================================================
 * Main Context
 * ========================================================================== */

  struct microlink_s
  {
    /* Configuration (immutable after init) */
    microlink_config_t config;

    /* State (atomic reads from any task, writes only from coord) */
    volatile microlink_state_t state;
    /* Count of control-plane connects (coord long-poll established). First
     * connect = 1; each later increment is a RE-connect (a soft link flap).
     * Exposed as ml_reconnects (= connect_count - 1) in /state.json for soak
     * tracking of DERP/control-plane stability. Writes only from coord. */
    volatile uint32_t connect_count;
    volatile uint32_t vpn_ip;

    /* Priority-peer application-level health, set by the app (pstop layer) via
     * microlink_notify_priority_health(): true while heartbeat replies flow,
     * false when they stop. The wg_mgr priority wake uses this to detect a
     * "zombie" WG session — the local keypair still reads up, but the peer
     * forgot us after a restart, so wireguardif_peer_is_up() reports a session
     * that no longer carries traffic. On false the wake forces a fresh 1-RTT
     * handshake. Defaults true so non-pstop users see no behaviour change. */
    volatile bool priority_link_healthy;

    /* DERP region of the priority peer, learned from the netmap. microlink
     * holds ONE DERP connection and a DERP server only delivers to peers
     * connected to it, so the chip homes its connection on THIS region (and
     * reports it as PreferredDERP) to guarantee it can relay to the machine.
     * 0 = not yet known → fall back to ML_DERP_REGION. */
    volatile uint16_t priority_peer_region;

    /* Fleet coordination/OTA server link health, set by the app (fleet-OTA
     * check-in) via microlink_notify_fleet_health(): true on a successful
     * check-in, false when a check-in can't reach the backend. Drives the management peer's
     * peer's disco-first wake to force a fresh handshake through a zombie
     * session (same idea as priority_link_healthy for the safety peer).
     * Defaults true. */
    volatile bool fleet_link_healthy;

    /* Event group (cross-task synchronization) */
    EventGroupHandle_t events;

    /* Task handles */
    TaskHandle_t net_io_task;
    TaskHandle_t derp_tx_task;
    TaskHandle_t coord_task;
    TaskHandle_t wg_mgr_task;

    /* Queues */
    QueueHandle_t derp_tx_queue; /* -> derp_tx task (disco/relay frames) */
    QueueHandle_t derp_tx_prio_queue; /* -> derp_tx task, drained FIRST (safety heartbeat + WG handshake) */
    QueueHandle_t disco_rx_queue; /* net_io -> wg_mgr */
    QueueHandle_t wg_rx_queue; /* net_io -> wg_mgr */
    QueueHandle_t stun_rx_queue; /* net_io -> coord */
    QueueHandle_t coord_cmd_queue; /* any -> coord */
    QueueHandle_t peer_update_queue; /* coord -> wg_mgr */

    /* Keys (loaded at init, read-only after) */
    uint8_t machine_private_key[32]; /* Noise machine key */
    uint8_t machine_public_key[32];
    uint8_t wg_private_key[32]; /* WireGuard key */
    uint8_t wg_public_key[32];
    uint8_t disco_private_key[32]; /* DISCO key */
    uint8_t disco_public_key[32];

    /* DERP relay connection POOL (owned exclusively by DERP I/O task after
     * connect). derp[0] is the HOME connection — the chip's own PreferredDERP
     * region, always maintained for inbound reachability. derp[1..N] are aux
     * connections opened on demand for the distinct regions of the safety peers
     * so cross-region relay works (magicsock model). See ML_DERP_MAX_CONNS. */
    ml_derp_conn_t derp[ML_DERP_MAX_CONNS];

    /* Region-pin override for slot 0's home region (repro rig + fleet pin).
     * 0 = auto (use the learned/rehomed derp_home_region). Non-zero forces the
     * chip to home its inbound DERP + PreferredDERP advert on THIS region.
     * Runtime-settable via /api/settings derp_region; see ml_effective_home_region. */
    volatile uint16_t derp_region_override;

    /* Sockets for net_io select() loop */
    int disco_sock4; /* UDP socket for DISCO + direct WG */
    int disco_sock6; /* IPv6 UDP socket (-1 if unavailable) */
    int stun_sock; /* UDP socket for STUN */
    uint16_t disco_local_port; /* Bound port for disco_sock4 */

    /* Coordination socket (owned exclusively by coord task) */
    int coord_sock;
    uint32_t h2_next_stream_id; /* Next H2 stream ID for endpoint updates (odd, starts at 7) */
    uint32_t
      map_stream_id; /* H2 stream id of the active streaming map long-poll; 0 until first poll; a soft refresh moves it to a fresh odd id (an ended H2 stream can't be reused). */

    /* WireGuard netif (owned exclusively by wg_mgr task) */
    void * wg_netif;

    /* Peers (owned exclusively by wg_mgr task) */
    ml_peer_t peers[ML_MAX_PEERS];
    int peer_count;

    /* STUN results (written by coord, read by coord only) */
    uint32_t stun_public_ip;
    uint16_t stun_public_port;

    /* STUN server cache (pre-resolved IPs, host byte order) */
    uint32_t stun_primary_ip; /* derp9.tailscale.com resolved IPv4 */
    uint32_t stun_fallback_ip; /* stun.l.google.com resolved IPv4 */
    uint8_t stun_retry_count; /* Retries on current server */
    bool stun_using_fallback; /* true if probing fallback server */
    uint64_t stun_last_probe_ms; /* Timestamp of last probe sent */

    /* IPv6 STUN */
    int stun_sock6; /* IPv6 UDP socket for STUN (-1 if unavailable) */
    uint8_t stun_primary_ip6[16]; /* derp9.tailscale.com resolved IPv6 */
    uint8_t stun_public_ip6[16]; /* Our public IPv6 from STUN */
    uint16_t stun_public_port6; /* Our public IPv6 port from STUN */
    bool stun_has_ipv6; /* true if IPv6 STUN result available */

    /* Symmetric NAT detection */
    uint16_t stun_secondary_port; /* Mapped port from second STUN server */
    bool stun_nat_checked; /* true if symmetric NAT check completed */
    bool nat_mapping_varies; /* true = symmetric NAT (direct won't work) */

    /* DERP map (parsed from MapResponse, owned by coord task) */
    ml_derp_region_t derp_regions[ML_MAX_DERP_REGIONS];
    uint8_t derp_region_count;
    uint16_t derp_home_region; /* Our PreferredDERP region */

    /* Key expiry (parsed from MapResponse self-node) */
    int64_t key_expiry_epoch; /* Unix epoch seconds, 0 = no expiry */
    bool key_expired; /* true if Node.Expired == true */

    /* Resolved timing (set during init from config, 0 = default) */
    uint32_t t_disco_heartbeat_ms;
    uint32_t t_stun_interval_ms;
    uint32_t t_ctrl_watchdog_ms;

    /* Callbacks */
    microlink_state_cb_t state_cb;
    void * state_cb_data;
    microlink_peer_cb_t peer_cb;
    void * peer_cb_data;
    microlink_data_cb_t data_cb;
    void * data_cb_data;
    /* "Keep this peer when the table is full" hook (NULL = disabled). See
     * microlink_set_peer_wanted_cb() / add_peer() in ml_wg_mgr.c. */
    microlink_peer_wanted_cb_t peer_wanted_cb;
    void * peer_wanted_ctx;

    /* HTTP Config Server (peer allowlist, runtime settings) */
    ml_config_ctx_t * config_httpd;

    /* NVS-backed config string storage (auth_key/device_name pointers in
     * microlink_config_t are redirected here when NVS settings exist) */
    char nvs_auth_key[96];
    char nvs_device_name[48];

    /* Control plane host override (empty = use ML_CTRL_HOST default).
     * Set from NVS at boot for Headscale/Ionscale/custom coordinators. */
    char ctrl_host[64];

    /* Debug flags (bitmask from NVS, checked at runtime for verbose logging) */
    uint8_t debug_flags; /* bit 0: DISCO, bit 1: WG, bit 2: DERP, bit 3: coord */

#ifdef CONFIG_ML_ZERO_COPY_WG
    /* Zero-copy WG: raw PCB replaces disco_sock4 BSD socket */
    ml_zerocopy_t zc;
#endif
  };

  /* ============================================================================
 * Internal Function Declarations (per-module)
 * ========================================================================== */

  /* ml_net_io.c */
  void ml_net_io_task(void * arg);

  /* ml_derp.c */
  void ml_derp_tx_task(void * arg);
  /* Connect a specific pool connection `c` to `region_id`'s DERP node. */
  esp_err_t ml_derp_connect(microlink_t * ml, ml_derp_conn_t * c, uint16_t region_id);
  /* Tear down a specific pool connection (frees its mbedTLS contexts + socket). */
  void ml_derp_disconnect(microlink_t * ml, ml_derp_conn_t * c);
  esp_err_t ml_derp_queue_send(microlink_t * ml, const uint8_t * dest_key, const uint8_t * data, size_t len);

  /* Effective home DERP region for slot 0: the runtime override wins, else the
   * learned/rehomed home region. 0 = neither known (caller falls back to
   * ML_DERP_REGION). Shared by ml_derp.c (slot-0 connect), ml_coord.c
   * (PreferredDERP advert) and ml_stun.c (STUN home region). */
  static inline uint16_t ml_effective_home_region(const microlink_t * ml)
  {
    uint16_t ov = ml->derp_region_override;
    return ov ? ov : ml->derp_home_region;
  }

  /* ml_coord.c */
  void ml_coord_task(void * arg);

  /* ml_wg_mgr.c */
  void ml_wg_mgr_task(void * arg);
  /* DERP region a peer is homed on, looked up by its 32-byte WG public key.
   * 0 = peer unknown or region not yet learned. Called from the enqueue path
   * (same wg_mgr task that owns the peer table) to tag each relayed frame. */
  uint16_t ml_wg_region_for_pubkey(microlink_t * ml, const uint8_t * wg_pubkey);
  /* True if the peer identified by its 32-byte WG public key is a SAFETY peer
   * (pinned incl. priority/fleet/app-pin, OR health-tracked). Used by the DERP
   * TX enqueue path to front-queue and protect the safety heartbeat frame from
   * disco backpressure. Same cross-task read profile as ml_wg_region_for_pubkey:
   * word-sized reads of the peer table, worst case a transiently stale answer. */
  bool ml_wg_is_safety_pubkey(microlink_t * ml, const uint8_t * wg_pubkey);
  /* Collect the DISTINCT DERP regions of the safety peers exempt from the
   * peer-scaling armor (pinned OR priority OR health-tracked). Writes up to
   * `max` region ids into `out`, returns the count. Used by the DERP task to
   * decide which auxiliary pool connections to open. */
  int ml_wg_collect_safety_regions(microlink_t * ml, uint16_t * out, int max);
  void ml_wg_mgr_send_cmm(microlink_t * ml, uint32_t peer_vpn_ip);
  esp_err_t ml_wg_mgr_trigger_handshake(microlink_t * ml, uint32_t dest_vpn_ip);
  bool ml_wg_mgr_peer_is_up(microlink_t * ml, uint32_t vpn_ip);
  void ml_wg_mgr_update_transport(microlink_t * ml);

  /* ml_stun.c */
  esp_err_t ml_stun_resolve_servers(microlink_t * ml);
  esp_err_t ml_stun_send_probe(microlink_t * ml, const char * server, uint16_t port);
  esp_err_t ml_stun_send_probe_to(microlink_t * ml, uint32_t server_ip, uint16_t port);
  esp_err_t ml_stun_send_probe_ipv6(microlink_t * ml, const uint8_t * server_ip6, uint16_t port);
  bool ml_stun_parse_response(const uint8_t * data, size_t len, uint32_t * out_ip, uint16_t * out_port);
  bool ml_stun_parse_response_ipv6(const uint8_t * data, size_t len, uint8_t * out_ip6, uint16_t * out_port);

  /* ml_noise.c */
  void ml_noise_init(
    ml_noise_state_t * state,
    const uint8_t * local_private,
    const uint8_t * local_public,
    const uint8_t * remote_public);
  esp_err_t ml_noise_write_msg1(ml_noise_state_t * state, uint8_t * out, size_t * out_len);
  esp_err_t ml_noise_read_msg2(ml_noise_state_t * state, const uint8_t * msg, size_t len);
  esp_err_t ml_noise_encrypt(
    const uint8_t * key,
    uint64_t nonce,
    const uint8_t * ad,
    size_t ad_len,
    const uint8_t * plaintext,
    size_t pt_len,
    uint8_t * ciphertext);
  esp_err_t ml_noise_decrypt(
    const uint8_t * key,
    uint64_t nonce,
    const uint8_t * ad,
    size_t ad_len,
    const uint8_t * ciphertext,
    size_t ct_len,
    uint8_t * plaintext);

  /* ml_h2.c */
  int ml_h2_build_headers_frame(
    uint8_t * out,
    size_t out_size,
    const char * method,
    const char * path,
    const char * authority,
    const char * content_type,
    uint32_t stream_id,
    bool end_stream);
  int ml_h2_build_data_frame(
    uint8_t * out, size_t out_size, const uint8_t * data, size_t data_len, uint32_t stream_id, bool end_stream);
  int ml_h2_build_preface(uint8_t * out, size_t out_size);
  int ml_h2_build_settings_ack(uint8_t * out, size_t out_size);
  int ml_h2_build_window_update(uint8_t * out, size_t out_size, uint32_t stream_id, uint32_t increment);

  /* ml_wg_mgr.c — active-uplink LAN IPv4 (host byte order), or 0 if none.
   * Used to advertise our local endpoint for same-LAN direct-path discovery. */
  uint32_t ml_active_lan_ip(void);

  /* ml_wg_mgr.c — same-LAN direct-path diagnostics for the priority peer.
   * Exposed via /admin/api/monitor to see, on units with no live log, what
   * candidate endpoints the chip holds for its machine and which one (if any)
   * it selected as the direct path. All IPs host byte order. */
  typedef struct
  {
    uint32_t active_lan_ip; /* what we would advertise as our LAN endpoint  */
    uint32_t pp_vpn_ip; /* priority peer (machine) VPN IP               */
    uint32_t pp_best_ip; /* chosen direct endpoint, 0 = none (on DERP)   */
    uint16_t pp_best_port;
    bool pp_has_direct; /* has_direct_path flag for the priority peer   */
    int pp_endpoint_count;
    uint32_t pp_ep_ip[ML_MAX_ENDPOINTS]; /* candidate endpoints we know     */
    uint16_t pp_ep_port[ML_MAX_ENDPOINTS];
  } ml_direct_diag_t;

  void ml_wg_get_direct_diag(microlink_t * ml, ml_direct_diag_t * out);

  /* ml_peer_nvs.c */
  esp_err_t ml_peer_nvs_init(void);
  void ml_peer_nvs_deinit(void);
  esp_err_t ml_peer_nvs_save(const ml_peer_t * peer);
  int ml_peer_nvs_load_all(ml_peer_t * peers, int max_peers);
  /* Deferred flash flush of the peer cache (writes are debounced: saves only
 * update the PSRAM working copy; call this ~once per wg_mgr pass). */
  esp_err_t ml_peer_nvs_flush_if_due(uint64_t now_ms);
  /* Mark a peer (by VPN IP, host order) as never-LRU-evicted from the cache —
 * used to pin the priority/safety peer so its endpoint survives reboots. */
  void ml_peer_nvs_set_protected(uint32_t vpn_ip);
  esp_err_t ml_peer_nvs_clear(void);

#ifdef CONFIG_ML_ZERO_COPY_WG
  /* ml_zerocopy.c */
  esp_err_t ml_zerocopy_init(microlink_t * ml);
  void ml_zerocopy_deinit(microlink_t * ml);
  esp_err_t ml_zerocopy_send(microlink_t * ml, const uint8_t * data, size_t len, uint32_t dest_ip, uint16_t dest_port);
#endif

  /* Utility */
  uint64_t ml_get_time_ms(void);

  /* ============================================================================
 * Network socket aliases — thin names over the BSD socket API so call sites
 * stay uniform.
 * ========================================================================== */

#define ml_socket socket
#define ml_connect connect
#define ml_send send
#define ml_recv recv
#define ml_sendto sendto
#define ml_recvfrom recvfrom
#define ml_close_sock close
#define ml_bind bind
#define ml_setsockopt setsockopt
#define ml_fcntl fcntl
#define ml_select_fds select
#define ml_getaddrinfo getaddrinfo
#define ml_freeaddrinfo freeaddrinfo
#define ml_write_sock write
#define ml_read_sock read

  /* PSRAM allocation helper */
  static inline void * ml_psram_malloc(size_t size)
  {
    void * ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) ptr = malloc(size);
    return ptr;
  }

  static inline void * ml_psram_calloc(size_t n, size_t size)
  {
    void * ptr = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) ptr = calloc(n, size);
    return ptr;
  }

#ifdef __cplusplus
}
#endif
