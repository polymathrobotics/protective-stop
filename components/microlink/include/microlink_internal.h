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
 * frames, drained FIRST by the DERP task so configured pstop heartbeats are
 * never stuck behind, or dropped by, a disco relay burst. */
#define ML_DERP_TX_PRIO_DEPTH 16 /* headroom for path-diversity leg-2 mirrors */
#define ML_DISCO_RX_QUEUE_DEPTH 8
#define ML_WG_RX_QUEUE_DEPTH 32 /* carries pstop heartbeats; 8 was sheddable by one disco burst */
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
#define ML_DERP_AUX_UNWANTED_REAP_MS \
  30000 /* reap an aux conn whose region is no
                                            * longer a safety region after 30 s,
                                            * regardless of traffic through it */

/* Consecutive failed periodic HOME-region connect attempts before slot 0 is
 * allowed to fall back to a proven-reachable region. DERP design-review finding
 * (2026-08): a chip whose home region's DERP server is TCP-unreachable retried
 * the SAME region forever with no fallback, leaving it offline on the relay
 * path. The periodic-retry cadence for a priority build is 5s,5s,10s,then 60s,
 * so N=4 spends ~80 s honouring the intended home (giving a transient outage or
 * an event-driven re-home a fair chance to recover it) before engaging the
 * fallback. Bounded, so it can never turn into a reconnect storm. */
#define ML_DERP_HOME_FALLBACK_AFTER 4

/* ============================================================================
 * DERP region auto-negotiation (design doc §6/§7/§8).
 *
 * §7 make-before-break (MBB): a live safety bond must NEVER be interrupted by
 * a home-region change (I1). The switch first opens an aux conn on the target
 * region (via the EXISTING derp_manage_aux machinery — want-set union, per-slot
 * backoff, one-blocking-connect-per-loop cap all inherited), PROVES it, and
 * only then swaps ml->derp_home_slot. The old home becomes an ordinary aux and
 * is held/reaped by the existing want-set mechanics (DRAIN_OLD is passive).
 *
 * §8 damping is MANDATORY, not defensive garnish: the MapResponse-guard comment
 * in ml_coord.c documents a real home-region ping-pong that stormed DERP
 * reconnects, and each DERP (re)connect is a full TLS handshake whose CPU burst
 * starves lwIP/httpd (400-700 ms latency spikes; an httpd handler-budget
 * exhaustion took down the admin API during the multi-remote validation). An
 * undamped negotiator is a reconnect-storm generator by construction — every
 * trigger below is therefore rate-limited, hysteretic and MBB.
 *
 * Constants are compile-time for the bench-validation phase; promoting them to
 * runtime /api/settings (NVS-persisted) is deliberately deferred until the
 * defaults survive a soak.
 * ========================================================================== */

/* §7 MBB state machine bounds */
#define ML_MBB_AUX_OPEN_TIMEOUT_MS 90000 /* covers 3 aux connect attempts (5s/10s/60s backoff) */
#define ML_MBB_PROVE_TIMEOUT_MS 30000 /* prove window; >> one 3 s priority disco heartbeat */
#define ML_MBB_PROVE_STABLE_MS 10000 /* conn must hold `connected` this long (§7 gate a) */
#define ML_MBB_PROVE_PING_INTERVAL_MS 5000 /* DERP server PING cadence for the weak proof (Q3) */
/* Prove-race grace: when the target peer is homed on the target region but may
 * have JUST switched there (autoneg follow moves both ends together), prefer its
 * end-to-end echo for this long, then also accept a DERP server PONG so a slow
 * peer echo doesn't spuriously roll back + ban the region. < STABLE so the fall-
 * back can prove before the commit gate. */
#define ML_MBB_PROVE_PEER_GRACE_MS 8000

/* §8 anti-thrash damping */
#define ML_NEG_STABILITY_MS \
  60000 /* target stable this long before MBB starts; absorbs netmap
                                   * patch bursts and coordination lag (> long-poll turnaround) */
#define ML_NEG_MIN_DWELL_MS \
  600000 /* 10 min between committed switches: a switch costs a TLS
                                    * handshake + control-plane propagation; ping-pong at shorter
                                    * periods is the documented §2.6 reconnect storm */
#define ML_NEG_MAX_SWITCHES_PER_HOUR 3 /* circuit breaker: on trip hold current home + log loudly */
#define ML_NEG_REGION_BAN_MS \
  900000 /* 15 min ban after a ROLLBACK: no retry-hammering an
                                     * unprovable region */
#define ML_NEG_ADVICE_MIN_INTERVAL_MS \
  1800000 /* fleet advice honored at most every 30 min,
                                               * chip-side, so even a flapping/buggy fleet cannot
                                               * exceed it (I3) */
#define ML_NEG_REGION_BANS 4 /* banned-region table size (distinct failed regions) */

  /* §7 MBB states (exposed via microlink_region_autoneg_t.mbb_state).
 * SWITCH_ADVERT is instantaneous (the index swap); DRAIN_OLD is passive
 * (want-set reap); ROLLBACK is an action, not a resting state. */
  enum
  {
    ML_MBB_IDLE = 0,
    ML_MBB_AUX_OPENING = 1,
    ML_MBB_PROVING = 2,
  };

  /* MBB executor -> negotiator outcome codes (mbb_outcome). */
  enum
  {
    ML_MBB_OUTCOME_NONE = 0,
    ML_MBB_OUTCOME_COMMITTED = 1,
    ML_MBB_OUTCOME_ROLLED_BACK = 2,
    ML_MBB_OUTCOME_ABORTED = 3, /* lock landed / request cancelled — no ban, no commit */
  };

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
/* Safety peers keep probes matchable far longer: the run-46 wedge's unmatched
 * pongs were replies delayed 5-11 s behind the far end's blocking DERP TLS
 * reconnect — past the 5 s general TTL, so the endpoint learn never completed
 * and direct never re-adopted. A late pong from a safety peer is still a
 * genuine endpoint proof; only these probes get the longer window. */
#define ML_DISCO_SAFETY_PROBE_TTL_MS 15000
/* Tail of the probe table only safety-peer probes may occupy, so general
 * tailnet churn can never starve a safety peer of a probe slot (probe-table
 * exhaustion = the 2026-08-09 regains oscillation; occupancy 12/64 observed
 * during the run-46 wedge, but the reserve makes the guarantee structural). */
#define ML_DISCO_PROBE_RESERVE_SLOTS 16
#define ML_DISCO_UPGRADE_INTERVAL_MS 60000

/* Priority-peer path liveness (2026-07-21 failover speedup): the peer named
 * by priority_peer_ip gets a tighter disco heartbeat, a pong-recency
 * demotion watchdog (direct->DERP when pongs stop, instead of waiting out
 * the 60 s trust lease), and a faster direct re-probe while demoted. Only
 * the priority peer — the 16-peer tailnet load profile is unchanged. */
#define ML_DISCO_PRIORITY_HB_MS 3000
/* Flush recovery (2026-08-16 mechanism): a middlebox conn-state flush kills
 * the router-crossing direct NAT entries AND the DERP relay lattice in the
 * same instant. The machine is a pure replier, so its only path-restoring
 * packet was the 3 s-cadence disco heartbeat — recovery was cadence-bound at
 * 3.0-3.3 s > the 2.0 s heartbeat timeout (one disarm per flush wave, three
 * events measured). When a direct safety peer's rx goes silent past this
 * threshold, ping IMMEDIATELY (~1 Hz while silent) so the punch is re-created
 * inside the timeout window. Additive probes only; never fires at 5 Hz rx. */
#define ML_DISCO_SILENCE_PING_MS 700
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
/* Demote-verification (see ml_demote_verdict.h): the PATH_DEAD_MS comment
 * above assumes a spurious demotion is harmless ("re-probe restores direct
 * in ~5s"). 2026-08-11 falsified that for a symmetric-NAT/USB-NCM safety
 * peer: demotion tears down a direct path that may never re-establish
 * without a reboot, onto a relay too jittery for the 2 s pstop timeout. So:
 * hold the demote while authenticated WG data still arrives DIRECT within
 * this window. 1000 ms = 5 missed 5 Hz heartbeats — a path that quiet is
 * genuinely suspect and may demote; a working path refreshes every 200 ms. */
#define ML_DEMOTE_DIRECT_RX_FRESH_MS 1000
/* Cap on consecutive demote-veto ticks (~1 s apart): past this, the disco
 * side-channel has been dead for ~10 s while data still flows — stop pinning
 * and let the demote proceed (DERP fallback + re-probe is the safer state;
 * see ml_peer_t.demote_veto_ticks). */
#define ML_DEMOTE_VETO_MAX_TICKS 10
/* Periodic idempotent coord re-registration: cures a QUIET registration loss
 * (control plane forgot the node; DERP servers deny relaying to it — 100%
 * inbound relay black-hole, invisible to local gauges) within one period.
 * The reconnect flow is hitless (peers/WG preserved, map re-ingest). */
#define ML_COORD_REREGISTER_MS 300000
/* Reap a CONNECTED pool conn that we actively egress into but that has
 * returned NOTHING for this long: in the safety topology every used conn
 * carries return traffic (replies/mirrors), so tx-active + rx-silent means a
 * server-side black-hole or half-dead TCP. Idle conns (no recent tx) are NOT
 * reaped — nothing is owed back on them. */
#define ML_DERP_RX_STALE_MS 150000
/* Global floor between DERP connect STARTS. A control-plane flush kills every
 * conn at once; back-to-back TLS connect work then starves DIRECT heartbeat
 * rx (run-26 switch drop-gap; run-43: 52 reconnects rode the flush, both
 * disarms inside the burst). Callers all retry on their own cadence, so a
 * refused kick just lands next tick. */
#define ML_DERP_CONNECT_SPACING_MS 1500
#define ML_DERP_RX_STALE_TX_ACTIVE_MS 30000
/* WG-rx handshake budget (docs/STALL_EVENT_CLOSURE_DESIGN.md): an inbound
 * WG INITIATION costs ~4 X25519 ≈ 45 ms measured and is processed holding
 * LOCK_TCPIP_CORE — an unbounded wave stalls heartbeat processing, the DERP
 * task's sockets and local egress at once. Handshakes consume a per-pass
 * budget shared across every wg_rx drain site; excess defers (requeue to
 * tail, capped), then drops — a late rekey fails SAFE. */
#define ML_WG_HANDSHAKES_PER_PASS 3
/* Requeue cap counts POPS WHILE STARVED (a pass has up to 10 drain calls),
 * so it must exceed one pass's worth of pops for a deferral to be
 * guaranteed to reach the next pass's fresh budget; drops are reserved for
 * sustained multi-pass starvation. */
#define ML_WG_HS_REQUEUE_MAX 12
/* Stall-event rings: per-event attribution (high-water gauges are blind
 * under their own mark). Task-owned writer, release-stored index; on wrap
 * the OLDEST entry a reader copies can race the writer's next event (µs
 * window, diagnostics-only — newest entries are always intact). */
#define ML_STALL_RING_LEN 8
#define ML_STALL_THRESH_MS 1000
/* §7b: cap on how long an ingest-busy signal may defer the peer-cache flash
 * flush — durability (boot preseed) must survive sustained storms. */
#define ML_PEER_NVS_FLUSH_MAX_DEFER_MS 30000
/* §7c: learned-region stash TTL — a fleet that MOVED regions while its peer
 * entry was absent must not pin a stale region indefinitely. */
#define ML_REGION_STASH_TTL_MS 1800000
/* Stash capacity = the NVS protected-pin bound (ML_NVS_MAX_PROTECTED): the
 * stash only ever holds pinned peers' regions, so at this size a bulk
 * remove-then-readd of every pin cannot LRU-evict an entry before its
 * re-ADD consumes it. */
#define ML_REGION_STASH_SLOTS 18

  typedef struct
  {
    uint32_t at_s; /* boot-relative seconds */
    uint32_t dur_ms; /* iteration wall-clock */
    uint16_t wg_pkts; /* wg_rx packets processed this pass */
    uint16_t handshakes; /* budget consumed this pass */
    uint16_t peer_adds; /* netmap adds this pass */
    uint16_t disco_opens; /* unknown-key box-opens this pass */
    uint16_t periodic_ms; /* wireguardif_periodic duration this pass */
    uint16_t disco_probe_ms; /* disco_periodic_probes duration this pass */
    uint16_t cmm_sends; /* CallMeMaybe seals this pass (~30 ms each) */
    uint16_t nvs_flush_ms; /* peer-cache flash-flush wall-clock this pass */
    uint16_t ingest_ms; /* process_peer_updates wall-clock this pass — covers
                         * skips/rejects/logging the add counter cannot see */
    uint16_t drain_ms; /* cumulative wg_mgr_drain_wg_rx wall-clock this pass */
  } ml_wg_stall_event_t;

  typedef struct
  {
    uint32_t at_s;
    uint32_t dur_ms;
    uint8_t home_cstate; /* DERP_CS_* at event: separates DNS from rx backlog */
    uint16_t last_dns_ms; /* most recent blocking DNS resolve duration */
    uint16_t rx_gap_ms; /* rx-poll gap measured this pass */
  } ml_derp_stall_event_t;

/* Hitless re-ingest (run-20 green drop, 2026-08-11): a coord/netmap teardown
 * (re-key retire, REMOVE) of a safety peer is VETOED while the WG session
 * shows authenticated data rx (any path) within this window — an immediate
 * wireguardif_remove_peer makes heartbeat sends fail ENOTCONN until a fresh
 * handshake, and >2 s of that is a machine STOP. 2000 ms = the pstop
 * heartbeat timeout: data within it means the safety bond is alive by
 * definition. A genuine re-key/removal goes stale within seconds (the old
 * key stops authenticating) and applies on the next update. */
#define ML_TEARDOWN_RX_FRESH_MS 2000

/* Relay-bound direct-path re-establishment (net/derp-direct-reestablish):
 * while a SAFETY peer's session is alive but riding DERP, periodically re-run
 * the full discovery handshake — CallMeMaybe (so the far side re-learns our
 * CURRENT endpoints and probes back, re-opening a NAT mapping that died) plus
 * a forced candidate sweep. The 5 s reprobe above only re-pings candidates we
 * already hold; it cannot recover when the far side's knowledge of US went
 * stale (NAT rebind, lost endpoint patch). Slow exponential backoff
 * (30 s -> 60 -> 120 -> 240 -> 300 cap) bounds the added load to one NaCl
 * seal + a few UDP sends per due peer, at most one peer per 1 s probe tick;
 * state resets the moment a direct path is regained. Safety peers only —
 * bulk peers keep the peer-scaling armor.
 *
 * Bench falsification of the first cut (2026-08-09, DUT pstop-01d7f344):
 * a 30 s FLOOR before the first CMM round left the safety peer visibly
 * relay-bound for the entire observation window. When the outage also
 * killed the chip's outbound NAT mapping (hostile bench NAT), fix-1's
 * probes to the proven endpoint die in flight — only a CMM (far side
 * probes back, re-punching the hole while our sweep re-punches ours) can
 * recover, so the FIRST round must come fast. First round 5 s after
 * demotion, then 30 s -> 60 -> 120 -> 240 -> 300 cap. The demotion path
 * additionally fires one immediate CMM (see disco_periodic_probes). */
#define ML_DISCO_RELAY_RETRY_FIRST_MS 5000
#define ML_DISCO_RELAY_RETRY_MIN_MS 30000
/* Was 300000 (5 min). This cycle only ever runs for SAFETY peers, and the
 * run-46 wedge showed the grown backoff (observed 153-293 s) BECOMES the
 * outage period: every regain attempt that fails pushes the next one out
 * another multiple. A safety link retries direct at least every 30 s,
 * regardless of history — the retry is one CMM + one ping sweep, cheap. */
#define ML_DISCO_RELAY_RETRY_MAX_MS 30000
/* Min interval between coord re-fetch (reconnect) requests for a symmetric-NAT
 * safety peer that stays relay-bound. The re-fetch pulls the peer's CURRENT
 * endpoints (steady-state coord updates are OmitPeers=true and never do), so the
 * relay-retry ping-sweep can then land and re-hole-punch. Control-plane only. */
#define ML_RELAY_REFETCH_MIN_MS 90000

/* Bench regression of the second cut (2026-08-09, all 3 devices on the fix
 * build): direct_regains oscillated in bursts (machn 131+ over 90 min) and
 * machn's wg_mgr task stalled >2.5 s, disarming the robot. Two compounding
 * causes, both bounded here:
 *
 * 1. The pong-race fix let a via-DERP-answered probe hold its
 *    pending_probes[] slot for the FULL 5 s ping timeout (so a slower direct
 *    pong could still promote). Under a CMM burst that multiplied
 *    steady-state slot demand past the 64-slot table; once full, new pings
 *    were sent UNREGISTERED, their pongs arrived "unmatched", so
 *    last_pong_recv_ms froze and the 4 s pong watchdog demoted perfectly
 *    healthy paths — each demotion firing another CMM: a runaway.
 *    Bound: after a via-DERP match the slot stays alive only
 *    ML_DISCO_DERP_MATCH_GRACE_MS more. The direct pong it is waiting for is
 *    the answer to the SAME dual-sent ping; if the direct path is alive that
 *    pong trails the DERP one by at most ~one direct RTT (<<1 s), so 1 s of
 *    grace preserves the pong-race fix while restoring a bounded lifetime.
 *
 * 2. process_disco_packet answers every received CallMeMaybe with a CMM of
 *    its own (bidirectional NAT traversal). Between TWO microlink chips that
 *    reply rule is a ping-pong chain sustained at DERP RTT until a frame
 *    drops; the demotion-CMM + 5 s retry round re-ignited chains constantly.
 *    Bound: at most one CMM per peer per ML_DISCO_CMM_MIN_INTERVAL_MS at the
 *    single send choke point (disco_send_call_me_maybe). Invariant: the
 *    FIRST CMM in any window passes — >=5 s-spaced retry rounds and isolated
 *    demotion events are never throttled. A re-demotion within the same
 *    window (rapid flap) IS throttled; its recovery falls to the next relay-
 *    retry round — worst case +5 s, still bounded. Chain re-fires are eaten. */
#define ML_DISCO_DERP_MATCH_GRACE_MS 1000
#define ML_DISCO_CMM_MIN_INTERVAL_MS 3000

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
    bool leg2; /* path-diversity mirror (safety frames): route on a CONNECTED
                * conn DISTINCT from the primary leg's; silently skipped when
                * no distinct conn exists. Receiver WG anti-replay dedups. */
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
    uint8_t requeues; /* budgeted-drain deferrals so far (handshakes only) */
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
    uint64_t last_pong_recv_ms; /* Last DISCO pong we received (any path) */
    uint64_t last_direct_pong_recv_ms; /* Last pong received DIRECT — the
                                        * pong-dead demote trigger keys on
                                        * this, not the path-blind stamp: a
                                        * relay-carried pong must not hold a
                                        * dead direct path out of demotion */
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

    /* Relay-bound direct-path retry (safety peers only, see
     * ML_DISCO_RELAY_RETRY_MIN_MS): next CallMeMaybe+sweep due time (0 = not
     * armed) and the attempt count driving the exponential backoff. Both are
     * cleared on every genuine direct promotion. */
    uint64_t relay_retry_next_ms;
    uint8_t relay_retry_count;

    /* Demote-verification veto streak (ml_demote_verdict.h): consecutive
     * maintenance ticks the veto held this peer's direct path. Capped at
     * ML_DEMOTE_VETO_MAX_TICKS — a veto that persists that long means the
     * disco side-channel is durably broken even though data flows; fail
     * toward the DERP demote rather than pin the path indefinitely (the
     * audited a175361 hazard: RX-only evidence can pin a TX-dead path).
     * Reset when no demote trigger fires or a demote executes. */
    uint8_t demote_veto_ticks;

    /* Learn-from-ping ring eviction (run-20/21 finding B-1): with the 8-slot
     * endpoint table full of dead symmetric-NAT candidates, the append-only
     * learn silently dropped the ONE live candidate a rebooted peer presents
     * — a terminal, reboot-surviving wedge on the MACHINE side. When full,
     * learned candidates now overwrite the last two slots round-robin. */
    uint8_t learn_evict_next;

    /* Disco-reset v2: per-peer rate limit (was a single global stamp that let
     * two stuck peers starve each other). 0 = never fired. */
    uint64_t disco_reset_next_ms;

    /* Chip<->chip CMM chain breaker (see ML_DISCO_CMM_MIN_INTERVAL_MS): ms of
     * the last CallMeMaybe SENT to this peer. 0 = never sent. */
    uint64_t last_cmm_sent_ms;

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

  /* Stage-2/3 async connect engine (docs/NONBLOCKING_DERP_TLS_PLAN.md §2): the
   * monolithic blocking connect is decomposed into a per-conn state machine
   * advanced ONE bounded step per DERP-task iteration, so a (re)connect can
   * never starve the rx poll of connected conns or the direct-path heartbeat.
   * All fields are written ONLY by the DERP I/O task (no-mutex invariant). */
  typedef enum
  {
    DERP_CS_IDLE = 0, /* not connecting: slot free, or fully CONNECTED */
    DERP_CS_TCP_CONNECT, /* non-blocking connect() in flight (DNS done at kick) */
    DERP_CS_TLS_HANDSHAKE, /* mbedtls_ssl_handshake() incremental */
    DERP_CS_HTTP_UPGRADE, /* GET /derp sent; reading the 101 incrementally */
    DERP_CS_DERP_HS, /* ServerKey -> ClientInfo -> ServerInfo */
  } ml_derp_cstate_t;

  typedef struct
  {
    int sockfd; /* Raw TCP socket */
    mbedtls_ssl_context ssl; /* TLS context (owned exclusively by DERP I/O task) */
    mbedtls_ssl_config ssl_conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    volatile bool connected; /* volatile: wg_mgr/httpd read it cross-task */
    uint64_t last_recv_ms; /* For keepalive watchdog */
    uint64_t last_relay_rx_ms; /* last RecvPacket (genuine relayed traffic) —
                                * the rx-staleness reap keys on THIS: server
                                * keepalives/pongs refresh last_recv_ms even
                                * while the server denies relaying to us */

    /* Multi-region pool bookkeeping (all owned by the DERP I/O task) */
    volatile uint16_t region_id; /* DERP region this conn serves; 0 = free slot.
                                  * volatile: cross-task readers (word-sized) */
    bool tls_inited; /* the four mbedTLS contexts above are live (must be freed) */
    uint64_t last_connect_attempt_ms; /* per-slot connect backoff timestamp */
    uint64_t last_used_ms; /* last time a frame egressed here (aux reap clock) */
    uint64_t aux_unwanted_since_ms; /* when this aux region left the safety set (0
                                     * = currently wanted). Reap clock decoupled
                                     * from traffic so incidental (non-safety)
                                     * frames can't keep a stale slot pinned. */

    /* §7 PROVING inputs (all written by the DERP I/O task):
     * connected_at_ms — conn-stability clock (gate a);
     * rx_pkts         — RecvPacket frames received on THIS conn: advancing
     *                   during PROVING is the strong end-to-end proof (a safety
     *                   peer's dual-sent disco/heartbeat leg came back through
     *                   the target server);
     * last_pong_ms    — DERP server PONG (reply to our client PING): the weak
     *                   proof when no bonded peer is on the target region (Q3
     *                   opportunistic hybrid). */
    uint64_t connected_at_ms;
    uint64_t last_pong_ms;
    uint32_t rx_pkts;

    /* --- async connect engine (Stage-2/3; DERP-task-owned) ------------------ */
    ml_derp_cstate_t cstate; /* connect progress; IDLE when free or CONNECTED */
    uint64_t cstate_deadline_ms; /* whole-connect deadline (DERP_CONNECT_TIMEOUT_MS) */
    uint16_t cs_region; /* region this in-progress connect targets */
    uint32_t cs_http_len; /* bytes accumulated in cs_buf during HTTP_UPGRADE */
    uint8_t cs_derp_step; /* 0=ServerKey hdr, 1=ServerKey payload, 2=ClientInfo send,
                           * 3=ServerInfo hdr, 4=ServerInfo payload/skip */
    uint8_t cs_hdr[5]; /* partial DERP frame header (resumable) */
    uint32_t cs_hdr_got;
    uint8_t cs_frame_type;
    uint32_t cs_frame_len;
    uint32_t cs_payload_got; /* payload bytes consumed so far (resumable) */
    uint8_t * cs_tx_buf; /* pending outbound (HTTP GET / ClientInfo frame); heap, freed on leave */
    uint32_t cs_tx_len;
    uint32_t cs_tx_sent; /* bytes of cs_tx_buf written so far (resumable) */
    bool cs_used_dns_cache; /* this attempt used the per-region addr cache (invalidate on fail) */
    uint8_t cs_buf[512]; /* HTTP response / ServerKey+ServerInfo scratch (resumable) */
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

    /* Region-pin override for the HOME slot's region (repro rig + fleet pin).
     * 0 = auto (use the learned/rehomed derp_home_region). Non-zero forces the
     * chip to home its inbound DERP + PreferredDERP advert on THIS region.
     * Runtime-settable via /api/settings derp_region; see ml_effective_home_region. */
    volatile uint16_t derp_region_override;

    /* Index of the HOME connection in the derp[] pool. Historically hardcoded
     * slot 0; the §7 MBB commit swaps THIS INDEX (never conn state — sockets/
     * TLS contexts stay put, no mid-I/O copy hazard) so the proven aux becomes
     * home and the old home decays into an ordinary aux (reaped by the
     * existing want-set mechanics). Written ONLY by the DERP I/O task; other
     * tasks (coord/httpd) take word-sized reads — a stale index during the
     * swap instant is benign because both conns are live throughout. */
    volatile int derp_home_slot;

    /* ---- §7 MBB cross-task command/status (negotiator on wg_mgr <-> executor
     * on the DERP I/O task). Command: wg_mgr writes mbb_target_region then
     * bumps mbb_generation; the executor restarts from IDLE whenever the
     * (generation,target) pair changes (§7 preemption: a newer target aborts
     * the in-flight switch). Status/outcome: executor-owned. All word-sized
     * volatiles — a torn intermediate read costs one extra executor restart,
     * never a wrong commit. */
    volatile uint16_t mbb_target_region; /* 0 = no switch pending */
    volatile uint32_t mbb_generation;
    volatile uint8_t mbb_state; /* ML_MBB_* (executor-owned) */
    volatile uint8_t mbb_outcome; /* ML_MBB_OUTCOME_* (executor sets, negotiator clears) */
    volatile uint16_t mbb_outcome_region; /* region the outcome refers to */

    /* ---- Phase-3 fleet region advice (§6, I3-gated). Written by the fleet
     * check-in task via microlink_offer_region_advice (which enforces the
     * strictly-increasing epoch), consumed by the wg_mgr negotiator. Expiry is
     * uptime SECONDS (uint32: no 64-bit tearing on cross-task reads, wraps
     * after 136 years). advice_region==0 or expiry passed => no advice. */
    volatile uint16_t advice_region;
    volatile uint32_t advice_epoch;
    volatile uint32_t advice_expires_s;

    /* ---- Q2 auto-apply surfacing (written by the wg_mgr negotiator, read by
     * /state.json + UI + check-in). derp_region_src is the AUTO-mode source
     * (MICROLINK_REGION_SRC_AUTO*); the getter reports LOCKED when the
     * override is set regardless of this field. NOTHING here is NVS-persisted
     * — the lock stays the only persisted region control (§7). */
    volatile uint8_t derp_region_src;
    volatile uint16_t derp_region_auto_applied;
    volatile uint32_t derp_region_auto_applies;
    volatile uint32_t derp_region_auto_apply_s; /* uptime seconds, 0 = never */

    /* ---- §4.2 primary-machine feed (registered by the app; NULL = legacy
     * single-priority behavior). ctx written before cb so a cross-task reader
     * never sees cb without its ctx. */
    microlink_primary_machine_cb_t primary_cb;
    void * primary_cb_ctx;

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
  /* Stage-2/3 async connect engine (see ml_derp.c engine header). */
  esp_err_t ml_derp_connect_kick(microlink_t * ml, ml_derp_conn_t * c, uint16_t region_id);
  int ml_derp_connect_step(microlink_t * ml, ml_derp_conn_t * c);
  uint32_t ml_derp_get_connect_steps(void);
  /* Pool conns reaped for tx-active rx-silence (server-side black-hole). */
  uint32_t ml_derp_get_rx_stale_reaps(void);
  /* Handshake-budget diag: out[0]=deferred (requeued), out[1]=dropped (cap/full). */
  void ml_wg_get_hs_budget_diag(uint32_t out[2]);
  bool ml_wg_fleet_configured(microlink_t * ml);
  /* §7a/§7c diag: out[0]=unchanged re-adds skipped, out[1]=region stash restores,
 * out[2]=allowlist rejects (near-zero-cost updates exempt from ingest pacing). */
  void ml_wg_get_ingest_diag(uint32_t out[3]);
  void ml_wg_get_skipfail_diag(uint32_t out[7]); /* first-failing §7a clause counts */
  /* wg_rx edge drops at each producer (queue-full sheds, previously silent). */
  uint32_t ml_net_io_get_wg_rx_drops(void);
  uint32_t ml_derp_get_wg_rx_drops(void);
  /* Stall-event rings: copies up to `max` published entries, returns count. */
  int ml_wg_get_stall_events(ml_wg_stall_event_t * out, int max);
  int ml_derp_get_stall_events(ml_derp_stall_event_t * out, int max);
  /* Periodic idempotent coord re-registrations executed. */
  uint32_t ml_coord_get_reregisters(void);
  /* Stage-0 gauges: out[0]=worst single DERP-task iteration ms, out[1]=worst
   * gap between consecutive rx-poll passes ms (both since boot). */
  void ml_derp_get_iter_diag(uint32_t out[2]);
  /* Tear down a specific pool connection (frees its mbedTLS contexts + socket). */
  void ml_derp_disconnect(microlink_t * ml, ml_derp_conn_t * c);
  esp_err_t ml_derp_queue_send(microlink_t * ml, const uint8_t * dest_key, const uint8_t * data, size_t len);
  /* Telemetry: number of times the DERP I/O task fell back off an unreachable
   * home region (or opened a rescue aux for a locked/re-homed dead region).
   * Exposed as derp_home_unreachable_fallbacks via /admin/api/monitor so the
   * design-review home-unreachable-fallback path is observable on a log-less
   * unit. Read from any task (word-sized read of a task-owned counter). */
  uint32_t ml_derp_get_home_fallback_count(void);

  /* §16 MBB executor telemetry: out[0]=commits, out[1]=rollbacks,
   * out[2]=proofs_ok, out[3]=proofs_failed. Counters owned by the DERP I/O
   * task; word-sized cross-task reads (same contract as the diag getters). */
  void ml_derp_get_mbb_diag(uint32_t out[4]);

  /* Stage-1 pin->MBB telemetry (docs/STAGE1_PIN_MBB_DESIGN.md):
   * out[0]=pin MBB requests, out[1]=commits, out[2]=retries (failed proves,
   * retry-forever), out[3]=backoff currently pending (0/1), out[4]=coord
   * full-resyncs forced by a pinned-but-absent peer (f498 heal),
   * out[5]=absent pins restored from the NVS cache. Owned by the wg_mgr
   * task; word-sized cross-task reads. */
  void ml_wg_get_pin_diag(uint32_t out[6]);
  /* Lookup a peer in the NVS cache by VPN IP into an update struct (pin-absent
   * self-heal synthesis path). Returns false when uncached/uninitialized. */
  bool ml_peer_nvs_lookup_update(uint32_t vpn_ip, ml_peer_update_t * out);
  /* One-shot: the next long-poll MapRequest sends OmitPeers=false so the
   * control plane re-delivers the FULL peer list (pin-absent heal fallback). */
  void ml_coord_request_full_peers(void);
  /* One-shot: run the hitless periodic coord re-register on the next coord
   * tick instead of waiting out ML_COORD_REREGISTER_MS (allowlist add). */
  void ml_coord_request_reregister(void);
  /* True when vpn_ip is the priority peer or an extra pin — consumed by
   * ml_config_peer_is_allowed()'s centralized pin exemption. */
  bool ml_wg_ip_is_pinned(uint32_t vpn_ip);
  uint32_t ml_derp_get_kicks_spaced(void); /* connect kicks refused by the spacing gate */
  /* Flush-recovery silence-ping diag: out[0]=pings sent, out[1]=last
   * ping→direct-pong resume delta ms, out[2]=worst resume delta ms,
   * out[3]=uptime s of the last resume record, out[4]=peer-table-full
   * add refusals (rides here to keep the monitor call count down). */
  void ml_wg_get_silence_diag(uint32_t out[5]);
  void ml_derp_note_reconnect_cause(int cause);
  void ml_derp_get_reconnect_causes(uint32_t out[5]);
  /* Stage-0b: worst single wg_mgr loop iteration ms (heartbeat-path stall). */
  uint32_t ml_wg_get_max_iter_ms(void);
  /* Path-diversity telemetry: out[0]=leg-2 mirrors sent, out[1]=mirrors
   * skipped (no distinct conn), out[2]=primary frames rescue-routed. */
  void ml_derp_get_diversity_diag(uint32_t out[3]);

  /* Effective home DERP region for slot 0: the runtime override wins, else the
   * learned/rehomed home region. 0 = neither known (caller falls back to
   * ML_DERP_REGION). Shared by ml_derp.c (slot-0 connect), ml_coord.c
   * (PreferredDERP advert) and ml_stun.c (STUN home region). */
  static inline uint16_t ml_effective_home_region(const microlink_t * ml)
  {
    uint16_t ov = ml->derp_region_override;
    return ov ? ov : ml->derp_home_region;
  }

  /* WG handshake frame (Initiation 0x01 / Response 0x02). The DERP tx priority
   * router and the wg-rx handshake budget must classify identically — one
   * predicate, not two hand-synced copies. */
  static inline bool ml_wg_is_handshake_frame(const uint8_t * data, size_t len)
  {
    return len >= 4 && (data[0] == 0x01 || data[0] == 0x02);
  }

  /* ml_coord.c */
  void ml_coord_task(void * arg);

  /* ml_wg_mgr.c */
  void ml_wg_mgr_task(void * arg);
  /* DERP region a peer is homed on, looked up by its 32-byte WG public key.
   * 0 = peer unknown or region not yet learned. Called from the enqueue path
   * (same wg_mgr task that owns the peer table) to tag each relayed frame. */
  uint16_t ml_wg_region_for_pubkey(microlink_t * ml, const uint8_t * wg_pubkey);
  /* DERP-egress classification for a destination pubkey. Returns the priority-
   * queue bit (pinned OR health-tracked); *mirror_out = heartbeat carriers only
   * (priority OR health-tracked — excludes the fleet pin). Same cross-task read
   * profile as ml_wg_region_for_pubkey. */
  bool ml_wg_tx_class_pubkey(microlink_t * ml, const uint8_t * wg_pubkey, bool * mirror_out);
  /* Collect the DISTINCT DERP regions of the safety peers exempt from the
   * peer-scaling armor (pinned OR priority OR health-tracked). Writes up to
   * `max` region ids into `out`, returns the count. Used by the DERP task to
   * decide which auxiliary pool connections to open. */
  int ml_wg_collect_safety_regions(microlink_t * ml, uint16_t * out, int max);
  /* Diagnostic predicates: does this peer feed collect_safety_regions? */
  bool ml_wg_is_pinned_peer(microlink_t * ml, uint32_t vpn_ip);
  bool ml_wg_is_health_tracked(uint32_t vpn_ip);
  /* §6 live_bond_active(): true while ANY health-tracked safety peer is
   * currently healthy (heartbeat replies flowing). Both roles feed this today:
   * the remote comparator reports per-machine-slot health each tick, and machn
   * reports per-remote health — so it is a faithful "a live safety bond
   * exists" signal on both. Decides legacy-immediate-rehome vs MBB (I1). */
  bool ml_wg_live_bond_active(void);
  /* §16 negotiator damping telemetry: out[0]=damping_suppressed,
   * out[1]=cooldown_trips, out[2]=switches in the rolling hour. */
  void ml_wg_get_neg_diag(uint32_t out[3]);
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

  /* ml_wg_mgr.c — relay-bound direct-path retry counters (log-less bench
   * visibility, same pattern as ml_wg_get_rehome_diag):
   * out[0] = retry rounds fired (CallMeMaybe + forced candidate sweep)
   * out[1] = direct-path regains (has_direct_path false -> true edges)
   * out[2] = safety peers currently relay-bound (active, no direct path)
   * out[3] = ms until the earliest armed retry round (0 = none armed / due
   *          now). Cross-task read of wg_mgr-owned words — worst case a
   *          transiently stale value, same contract as ml_wg_get_direct_diag. */
  void ml_wg_get_direct_retry_diag(microlink_t * ml, uint32_t out[4]);

  /* ml_peer_nvs.c */
  esp_err_t ml_peer_nvs_init(void);
  void ml_peer_nvs_deinit(void);
  esp_err_t ml_peer_nvs_save(const ml_peer_t * peer);
  int ml_peer_nvs_load_all(ml_peer_t * peers, int max_peers);
  /* Deferred flash flush of the peer cache (writes are debounced: saves only
 * update the PSRAM working copy; call this ~once per wg_mgr pass). */
  esp_err_t ml_peer_nvs_flush_if_due(uint64_t now_ms, bool ingest_busy);
  /* Flush timing diag: out[0]=last ms, out[1]=max ms, out[2]=count. */
  void ml_peer_nvs_get_flush_diag(uint32_t out[3]);
  /* Mark a peer (by VPN IP, host order) as never-LRU-evicted from the cache.
 * Bounded set (priority peer, fleet server, app-pinned operator remotes):
 * these keys feed the boot-time WG preseed that answers cold inbound
 * handshakes before the netmap re-arrives (cold-bond gap, 2026-08-08). */
  void ml_peer_nvs_set_protected(uint32_t vpn_ip);
  esp_err_t ml_peer_nvs_clear(void);

  /* microlink.c — persist ml->vpn_ip to NVS (write-on-change; coord task
 * context only, never the safety tick). Enables pre-registration WG
 * bring-up on the next boot. */
  void ml_ident_persist_vpn_ip(microlink_t * ml);

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
