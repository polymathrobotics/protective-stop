// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_wg_mgr.c
 * @brief WireGuard Manager Task - Peer Management + DISCO
 *
 * Owns ALL peer state exclusively. Handles:
 * - Peer add/remove/update from coord task (via peer_update_queue)
 * - DISCO ping/pong with rate limiting (matching tailscaled timing)
 * - WireGuard peer provisioning via wireguard-lwip
 * - Direct path discovery and endpoint switching
 *
 * Reference: tailscale/wgengine/magicsock/magicsock.go
 *            tailscale/disco/disco.go
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "arch/sys_arch.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "lwip/ip.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "mbedtls/base64.h"
#include "microlink_internal.h"
#include "ml_config_httpd.h"
#include "ml_demote_verdict.h"
#include "nacl_box.h"
#include "wireguard.h"
#include "wireguardif.h"

/* Forward declaration for zero-copy path */
extern void wireguardif_network_rx(
  void * arg, struct udp_pcb * pcb, struct pbuf * p, const ip_addr_t * addr, u16_t port);

static const char * TAG = "ml_wg_mgr";

/* v15.20 (upstream PR#14): take lwIP TCPIP core lock around any direct
 * lwIP API call (udp_sendto, wireguardif_network_rx, etc.) made from a
 * non-TCPIP thread. Required by CONFIG_LWIP_TCPIP_CORE_LOCKING=y in
 * sdkconfig.defaults. Without this the chip silently panics after
 * ~100-180 s of sustained direct-UDP traffic — corrupted pbuf/netif
 * state from concurrent access. See upstream microlink issues #14, #15. */
static inline bool ml_lwip_core_lock_needed(void)
{
  return !sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER);
}

/* Forward declarations */
static void disco_send_call_me_maybe(microlink_t * ml, int peer_idx);
static void wg_mgr_drain_wg_rx(microlink_t * ml);

static int s_wg_hs_budget; /* reset each wg_mgr pass */
static uint32_t s_diag_wg_hs_deferred;
static uint32_t s_diag_wg_hs_dropped;
/* Per-pass work counters — inputs to the stall-event ring. */
static uint16_t s_pass_wg_pkts;
static uint16_t s_pass_wg_handshakes;
static uint16_t s_pass_peer_adds;
static uint16_t s_pass_disco_opens;
static uint16_t s_pass_periodic_ms;
static uint16_t s_pass_probe_ms;
static uint16_t s_pass_cmm_sends; /* counted in disco_send_call_me_maybe (6 call sites) */
static uint16_t s_pass_nvs_flush_ms; /* flash-flush wall-clock this pass */

/* Stall-event ring: wg_mgr-owned writer; entries published via a
 * release-stored index so httpd never reads a torn published entry. */
static ml_wg_stall_event_t s_wg_stall_ring[ML_STALL_RING_LEN];
static volatile uint32_t s_wg_stall_widx; /* total events; entry i at i%LEN */

/* v15.22 — WG data-packet dedup ring.
 *
 * Tailscale's host-side tailscaled sends every transport-data packet via
 * BOTH the direct UDP path AND the DERP relay path simultaneously (its
 * normal redundancy posture until it's certain only the direct path is
 * needed). On the chip that doubled the WG processing rate: each packet
 * was decrypted + replay-checked + ip_input'd twice. wireguard_lwip's
 * replay window does eventually reject the second copy, but only AFTER
 * the full ChaCha20-Poly1305 decrypt — wasted work that accumulated
 * pressure on lwIP state and was the residual ~40-80 min wedge trigger
 * we still saw after the v15.20 core-locking fix.
 *
 * Fix: peek at the WG header (receiver_index + counter) before the pbuf
 * copy. If we've seen this (peer, nonce) pair within the last few
 * packets, drop the duplicate.
 *
 * Layout of MESSAGE_TRANSPORT_DATA (RFC §5.4.6):
 *   u8  type        = 4
 *   u8  reserved[3] = 0
 *   u32 receiver    (little-endian, our local peer index)
 *   u64 counter     (little-endian, sender's nonce)
 *   u8  encrypted_data[...]
 *
 * Ring is global (not per-peer): (receiver, counter) is already unique
 * across peers because receiver is *our* per-peer index. 16 slots is
 * way more than the network ever stretches the direct-vs-DERP gap. */
#define WG_DEDUP_RING_SIZE 64

typedef struct
{
  uint32_t receiver;
  uint64_t counter;
} wg_dedup_entry_t;

static wg_dedup_entry_t s_wg_dedup_ring[WG_DEDUP_RING_SIZE];
static int s_wg_dedup_head = 0;
static uint32_t s_wg_dedup_drops = 0;

static inline bool wg_dedup_is_duplicate(uint32_t receiver, uint64_t counter)
{
  for (int i = 0; i < WG_DEDUP_RING_SIZE; i++) {
    if (s_wg_dedup_ring[i].receiver == receiver && s_wg_dedup_ring[i].counter == counter) {
      return true;
    }
  }
  /* Not seen — record it. Ring write is single-threaded (all calls
     * come from ml_wg_mgr task), so no locking needed. */
  s_wg_dedup_ring[s_wg_dedup_head].receiver = receiver;
  s_wg_dedup_ring[s_wg_dedup_head].counter = counter;
  s_wg_dedup_head = (s_wg_dedup_head + 1) % WG_DEDUP_RING_SIZE;
  return false;
}

uint32_t ml_wg_mgr_get_dedup_drops(void)
{
  return s_wg_dedup_drops;
}

static void disco_send_ping_to_peer(microlink_t * ml, int peer_idx, bool force);

/* DISCO message types */
#define DISCO_MSG_PING 0x01
#define DISCO_MSG_PONG 0x02
#define DISCO_MSG_CALL_ME_MAYBE 0x03

/* DISCO magic bytes: "TS" + sparkles emoji UTF-8 */
static const uint8_t DISCO_MAGIC[6] = {'T', 'S', 0xf0, 0x9f, 0x92, 0xac};

#define DISCO_TXID_LEN 12
#define DISCO_NONCE_LEN 24

/* Check if an IP (host byte order) is a LAN address */
static inline bool is_lan_ip(uint32_t ip)
{
  return ((ip >> 24) == 10) || /* 10.x.x.x */
         ((ip >> 16) == 0xC0A8) || /* 192.168.x.x */
         (((ip >> 16) & 0xFFF0) == 0xAC10); /* 172.16-31.x.x */
}

/* Pending DISCO probe tracking */
typedef struct
{
  uint8_t txid[DISCO_TXID_LEN];
  uint32_t dest_ip;
  uint16_t dest_port;
  uint64_t sent_ms;
  /* ms of the FIRST via-DERP pong match (0 = none yet). A via-DERP match no
   * longer consumes the slot (pong-race fix: the direct pong to the same
   * dual-sent ping must still be able to promote), but the slot may then live
   * only ML_DISCO_DERP_MATCH_GRACE_MS more — an unbounded (full 5 s) lifetime
   * is what exhausted this table on the 2026-08-09 bench and demoted healthy
   * paths (see ML_DISCO_DERP_MATCH_GRACE_MS). */
  uint64_t derp_match_ms;
  int peer_index;
  bool active;
} disco_probe_t;

#define MAX_PENDING_PROBES 64
static disco_probe_t pending_probes[MAX_PENDING_PROBES];

/* Small ring of the last few CONSUMED probe txids. The machine answers each ping
 * on BOTH direct and DERP with the same txid; the direct pong consumes the
 * probe, so the DERP duplicate arrives "unmatched". Recognising it here lets us
 * log it at DEBUG ("duplicate via-DERP pong") instead of a misleading WARN.
 * Cosmetic only — never affects probe matching or heartbeat accounting. */
#define CONSUMED_TXID_RING 8
static uint8_t s_consumed_txids[CONSUMED_TXID_RING][DISCO_TXID_LEN];
static int s_consumed_txid_head = 0;

static void remember_consumed_txid(const uint8_t * txid)
{
  memcpy(s_consumed_txids[s_consumed_txid_head], txid, DISCO_TXID_LEN);
  s_consumed_txid_head = (s_consumed_txid_head + 1) % CONSUMED_TXID_RING;
}

static bool txid_recently_consumed(const uint8_t * txid)
{
  for (int i = 0; i < CONSUMED_TXID_RING; i++) {
    if (memcmp(s_consumed_txids[i], txid, DISCO_TXID_LEN) == 0) return true;
  }
  return false;
}

/* ============================================================================
 * Base64 Key Encoding (wireguard-lwip API requires base64 keys)
 * ========================================================================== */

static void key_to_base64(const uint8_t * key, char * b64, size_t b64_size)
{
  size_t olen = 0;
  mbedtls_base64_encode((unsigned char *)b64, b64_size, &olen, key, 32);
  b64[olen] = '\0';
}

/* ============================================================================
 * UDP Send Helper — routes via BSD socket or zero-copy PCB
 *
 * All direct DISCO/WG UDP sends go through this function.
 * dest_ip is HOST byte order, dest_port is HOST byte order.
 * ========================================================================== */

static inline bool disco_has_udp_path(const microlink_t * ml)
{
#ifdef CONFIG_ML_ZERO_COPY_WG
  if (ml->zc.pcb) return true;
#endif
  return (ml->disco_sock4 >= 0);
}

static int disco_udp_sendto(
  microlink_t * ml, const uint8_t * data, size_t len, uint32_t dest_ip_hbo, uint16_t dest_port)
{
#ifdef CONFIG_ML_ZERO_COPY_WG
  if (ml->zc.pcb) {
    return (ml_zerocopy_send(ml, data, len, dest_ip_hbo, dest_port) == ESP_OK) ? len : -1;
  }
#endif
  if (ml->disco_sock4 < 0) return -1;

  struct sockaddr_in dest;
  memset(&dest, 0, sizeof(dest));
  dest.sin_family = AF_INET;
  dest.sin_port = htons(dest_port);
  dest.sin_addr.s_addr = htonl(dest_ip_hbo);

  return ml_sendto(ml->disco_sock4, data, len, MSG_DONTWAIT, (struct sockaddr *)&dest, sizeof(dest));
}

/* ============================================================================
 * WireGuard Output Callbacks (for magicsock mode)
 * ========================================================================== */

/* Called by wireguard-lwip when a peer has no direct endpoint (DERP relay) */
static err_t wg_derp_output_cb(const uint8_t * peer_public_key, const uint8_t * data, size_t len, void * ctx)
{
  microlink_t * ml = (microlink_t *)ctx;
  /* Any pool connection up is enough to accept the relay: ml_derp_queue_send
   * tags the frame with the destination's region and the DERP task routes it to
   * the home OR the matching aux conn (cross-region peers ride an aux slot). */
  bool derp_up = false;
  if (ml) {
    for (int i = 0; i < ML_DERP_MAX_CONNS; i++) {
      if (ml->derp[i].connected) {
        derp_up = true;
        break;
      }
    }
  }
  if (!ml || !derp_up) {
    ESP_LOGW(TAG, "DERP output cb: not connected, dropping %d bytes", (int)len);
    return ERR_CONN;
  }

  /* Log WG handshake initiations with key and one-time hex dump */
  if (len >= 4 && data[0] == 0x01) {
    static int init_dump_count = 0;
    const char * hostname = "?";
    for (int i = 0; i < ml->peer_count; i++) {
      if (memcmp(ml->peers[i].public_key, peer_public_key, 32) == 0) {
        hostname = ml->peers[i].hostname;
        break;
      }
    }
    ESP_LOGI(
      TAG,
      "WG INIT -> %s len=%d key=%02x%02x%02x%02x%02x%02x%02x%02x",
      hostname,
      (int)len,
      peer_public_key[0],
      peer_public_key[1],
      peer_public_key[2],
      peer_public_key[3],
      peer_public_key[4],
      peer_public_key[5],
      peer_public_key[6],
      peer_public_key[7]);

    /* Dump first handshake fully for byte-level verification */
    if (init_dump_count < 1 && len == 148) {
      init_dump_count++;
      /* WG handshake init: type(1) reserved(3) sender(4) ephemeral(32)
             * enc_static(48) enc_timestamp(28) mac1(16) mac2(16) = 148 */
      ESP_LOGI(
        TAG,
        "  type=%02x res=%02x%02x%02x sender=%02x%02x%02x%02x",
        data[0],
        data[1],
        data[2],
        data[3],
        data[4],
        data[5],
        data[6],
        data[7]);
      ESP_LOGI(
        TAG,
        "  ephemeral=%02x%02x%02x%02x...%02x%02x%02x%02x",
        data[8],
        data[9],
        data[10],
        data[11],
        data[36],
        data[37],
        data[38],
        data[39]);
      ESP_LOGI(
        TAG,
        "  mac1=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
        data[116],
        data[117],
        data[118],
        data[119],
        data[120],
        data[121],
        data[122],
        data[123],
        data[124],
        data[125],
        data[126],
        data[127],
        data[128],
        data[129],
        data[130],
        data[131]);
      ESP_LOGI(
        TAG,
        "  mac2=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
        data[132],
        data[133],
        data[134],
        data[135],
        data[136],
        data[137],
        data[138],
        data[139],
        data[140],
        data[141],
        data[142],
        data[143],
        data[144],
        data[145],
        data[146],
        data[147]);
    }
  }

  esp_err_t err = ml_derp_queue_send(ml, peer_public_key, data, len);
  return (err == ESP_OK) ? ERR_OK : ERR_MEM;
}

/* Called by wireguard-lwip when sending via external UDP socket (magicsock).
 * Uses raw lwIP udp_sendto() instead of BSD sendto() to avoid deadlock
 * when called from the TCPIP thread context (via tcpip_input → ip_input →
 * icmp/tcp reply → wireguardif_output → this callback). BSD sendto() posts
 * a message to the TCPIP thread and waits, which deadlocks if we're already
 * on that thread. */
static struct udp_pcb * s_wg_output_pcb = NULL;

static err_t wg_udp_output_cb(uint32_t dest_ip, uint16_t dest_port, const uint8_t * data, size_t len, void * ctx)
{
  microlink_t * ml = (microlink_t *)ctx;
  if (!ml) return ERR_CONN;

  /* Log WG packets sent via direct UDP */
  uint32_t ip_host = ntohl(dest_ip);
  ESP_LOGI(
    TAG,
    "WG UDP TX: %d bytes -> %d.%d.%d.%d:%d type=%d",
    (int)len,
    (int)((ip_host >> 24) & 0xFF),
    (int)((ip_host >> 16) & 0xFF),
    (int)((ip_host >> 8) & 0xFF),
    (int)(ip_host & 0xFF),
    (int)dest_port,
    len >= 1 ? data[0] : -1);

  /* Use raw PCB to send — safe from any thread context */
  if (!s_wg_output_pcb) return ERR_CONN;

  struct pbuf * p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  if (!p) return ERR_MEM;
  memcpy(p->payload, data, len);

  ip_addr_t dst;
  IP_SET_TYPE_VAL(dst, IPADDR_TYPE_V4);
  ip4_addr_set_u32(ip_2_ip4(&dst), dest_ip); /* already network byte order */

  bool need_lock = ml_lwip_core_lock_needed();
  if (need_lock) {
    LOCK_TCPIP_CORE();
  }
  err_t err = udp_sendto(s_wg_output_pcb, p, &dst, dest_port);
  if (need_lock) {
    UNLOCK_TCPIP_CORE();
  }
  pbuf_free(p);
  return err;
}

/* ============================================================================
 * WireGuard Interface Initialization
 * ========================================================================== */

static esp_err_t wg_init_interface(microlink_t * ml)
{
  /* Convert our WG private key to base64 */
  char privkey_b64[64];
  key_to_base64(ml->wg_private_key, privkey_b64, sizeof(privkey_b64));

  /* Allocate lwIP netif */
  struct netif * netif = (struct netif *)calloc(1, sizeof(struct netif));
  if (!netif) {
    ESP_LOGE(TAG, "Failed to allocate WG netif");
    return ESP_FAIL;
  }

  /* Prepare init data */
  struct wireguardif_init_data wg_init = {0};
  wg_init.private_key = privkey_b64;
  wg_init.listen_port = 51820;
  wg_init.bind_netif = NULL;

  /* Disable internal socket binding (we use magicsock mode) */
  wireguardif_disable_socket_bind();

  /* v15.23: take TCPIP core lock around wireguardif_init + netif setup
     * + udp_new. These run at boot — low risk in practice (TCPIP thread
     * isn't processing real traffic yet) but unsafe without the lock per
     * upstream PR#14 audit. */
  bool init_need_lock = ml_lwip_core_lock_needed();
  if (init_need_lock) {
    LOCK_TCPIP_CORE();
  }

  /* Initialize WireGuard netif */
  netif->state = &wg_init;
  err_t err = wireguardif_init(netif);
  if (err != ERR_OK) {
    if (init_need_lock) {
      UNLOCK_TCPIP_CORE();
    }
    ESP_LOGE(TAG, "wireguardif_init failed: %d", err);
    free(netif);
    return ESP_FAIL;
  }

  /* Set IP addresses: our VPN IP (or temporary until we get one) */
  if (ml->vpn_ip != 0) {
    uint8_t a = (ml->vpn_ip >> 24) & 0xFF;
    uint8_t b = (ml->vpn_ip >> 16) & 0xFF;
    uint8_t c = (ml->vpn_ip >> 8) & 0xFF;
    uint8_t d = ml->vpn_ip & 0xFF;
    IP4_ADDR(&netif->ip_addr.u_addr.ip4, a, b, c, d);
  } else {
    IP4_ADDR(&netif->ip_addr.u_addr.ip4, 100, 64, 0, 1); /* temp */
  }
  IP4_ADDR(&netif->netmask.u_addr.ip4, 255, 192, 0, 0); /* /10 */
  IP4_ADDR(&netif->gw.u_addr.ip4, 0, 0, 0, 0);

  /* Use tcpip_input so decrypted packets are posted to the TCPIP thread.
     * Required for TCP (esp_http_server sockets) — ip_input from the wg_mgr
     * thread accesses TCP PCB state without synchronization.  The WG output
     * callback uses raw udp_sendto (not BSD sendto) to avoid deadlock. */
  netif->input = tcpip_input;

  /* Add to lwIP netif list (bypass netif_add which wants init callback) */
  netif->next = netif_list;
  netif_list = netif;

  /* Bring interface up */
  netif_set_up(netif);
  netif_set_link_up(netif);

  /* Create raw UDP PCB for WG output (avoids BSD sendto deadlock on TCPIP
     * thread).  Bind to port 51820 to match the DISCO socket source port.
     * The existing BSD disco_sock4 is only used from the wg_mgr task for
     * DISCO/STUN; this raw PCB is used from the TCPIP thread for WG output. */
  if (!s_wg_output_pcb) {
    s_wg_output_pcb = udp_new();
    if (s_wg_output_pcb) {
      /* Set source port to 51820 — MUST match disco_sock4 so WG handshake
             * responses share the same NAT mapping as DISCO/STUN.  We set
             * local_port directly instead of calling udp_bind() to avoid
             * registering this PCB as a listener (which would steal incoming
             * packets from disco_sock4's BSD socket). */
      s_wg_output_pcb->local_port = 51820;
      /* DSCP 46 (EF) → WMM AC_VO for low-latency WiFi scheduling */
      s_wg_output_pcb->tos = 0xB8;
    }
  }

  if (init_need_lock) {
    UNLOCK_TCPIP_CORE();
  }

  /* Register output callbacks for magicsock mode (these just store fn
     * pointers on the netif state, no lwIP core access) */
  wireguardif_set_derp_output(netif, wg_derp_output_cb, ml);
  wireguardif_set_udp_output(netif, wg_udp_output_cb, ml);

  ml->wg_netif = netif;

  /* Verify WG device public key matches our expected key */
  {
    struct wireguard_device * dev = (struct wireguard_device *)netif->state;
    if (dev) {
      bool match = (memcmp(dev->public_key, ml->wg_public_key, 32) == 0);
      ESP_LOGI(
        TAG,
        "WG device pubkey: %02x%02x%02x%02x... %s ml->wg_public_key",
        dev->public_key[0],
        dev->public_key[1],
        dev->public_key[2],
        dev->public_key[3],
        match ? "MATCHES" : "MISMATCH!");
      if (!match) {
        ESP_LOGE(
          TAG,
          "  Expected: %02x%02x%02x%02x...",
          ml->wg_public_key[0],
          ml->wg_public_key[1],
          ml->wg_public_key[2],
          ml->wg_public_key[3]);
      }
    }
  }

  ESP_LOGI(TAG, "WireGuard interface initialized (magicsock mode)");
  return ESP_OK;
}

static void wg_update_vpn_ip(microlink_t * ml)
{
  if (ml->wg_netif && ml->vpn_ip != 0) {
    struct netif * netif = (struct netif *)ml->wg_netif;
    uint8_t a = (ml->vpn_ip >> 24) & 0xFF;
    uint8_t b = (ml->vpn_ip >> 16) & 0xFF;
    uint8_t c = (ml->vpn_ip >> 8) & 0xFF;
    uint8_t d = ml->vpn_ip & 0xFF;
    IP4_ADDR(&netif->ip_addr.u_addr.ip4, a, b, c, d);
  }
}

/* ============================================================================
 * Peer Management (owned exclusively by this task)
 * ========================================================================== */

static int find_peer_by_key(microlink_t * ml, const uint8_t * pubkey)
{
  for (int i = 0; i < ml->peer_count; i++) {
    if (ml->peers[i].active && memcmp(ml->peers[i].public_key, pubkey, 32) == 0) {
      return i;
    }
  }
  return -1;
}

static int find_peer_by_ip(microlink_t * ml, uint32_t vpn_ip)
{
  for (int i = 0; i < ml->peer_count; i++) {
    if (ml->peers[i].active && ml->peers[i].vpn_ip == vpn_ip) {
      return i;
    }
  }
  return -1;
}

/* Fleet coordination/OTA server VPN IP (CONFIG_ML_FLEET_SERVER_IP), parsed
 * once. 0 if unset. */
static uint32_t fleet_server_ip_cached(void)
{
  static uint32_t ip;
  static bool done;
  if (!done) {
    ip = microlink_parse_ip(CONFIG_ML_FLEET_SERVER_IP);
    done = true;
  }
  return ip;
}

/* Peers that must always keep a WG slot and are never evicted: the priority
 * (safety) peer — the machine — and the management/OTA server. On a
 * large tailnet (100s of peers, trimmed to ML_MAX_PEERS) either could
 * otherwise fall out of the table, leaving the chip unable to reach the
 * machine (unsafe) or the management server (no OTA/coordination). */
/* Additional always-pinned VPN IPs registered by the application — e.g.
 * every configured pstop machine slot. Without this, a machine that joins
 * the tailnet after the chip's last full netmap (or that has no recent
 * activity) gets trimmed by the ML_MAX_PEERS cap and the remote can never
 * bond to it (observed: EHOSTUNREACH on every BOND, bench 2026-07-31). */
#define ML_EXTRA_PINS 16
static uint32_t s_extra_pins[ML_EXTRA_PINS];

/* True path RTT to a peer from the disco layer (txid-matched ping->pong,
 * identical semantics from either end of a link — unlike the pstop loop
 * timings, which are quantized by protocol hold times). Returns 0 if the
 * peer is unknown or never ponged; age_ms_out (optional) = how stale the
 * measurement is; direct_out (optional) = measured over the direct path. */
uint32_t microlink_get_peer_rtt(microlink_t * ml, uint32_t vpn_ip, uint32_t * age_ms_out, bool * direct_out)
{
  if (ml == NULL) return 0;
  int idx = find_peer_by_ip(ml, vpn_ip);
  if (idx < 0) return 0;
  ml_peer_t * p = &ml->peers[idx];
  if (p->last_pong_recv_ms == 0) return 0;
  if (age_ms_out != NULL) {
    uint64_t now = ml_get_time_ms();
    *age_ms_out = (now > p->last_pong_recv_ms) ? (uint32_t)(now - p->last_pong_recv_ms) : 0u;
  }
  if (direct_out != NULL) *direct_out = p->disco_rtt_direct;
  return p->disco_rtt_ms;
}

/* Per-peer application-level health, the multi-machine generalization of
 * microlink_notify_priority_health(): the pstop layer reports, per machine
 * TARGET, whether heartbeat replies are flowing. Each unhealthy entry gets
 * the same zombie-keypair treatment as the priority peer — a disco-first
 * wake forcing a fresh 1-RTT handshake — so a power-cycled machine
 * re-bonds in seconds instead of waiting out WG rekey timers. Without
 * this, only an ALL-machines-silent device recovered fast (the aggregate
 * kick deliberately refused to churn a partially-healthy transport). */
typedef struct
{
  uint32_t ip;
  volatile bool healthy;
} ml_health_ent_t;

static ml_health_ent_t s_health_peers[ML_EXTRA_PINS];

static bool is_health_tracked(uint32_t vpn_ip)
{
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if ((s_health_peers[i].ip != 0) && (s_health_peers[i].ip == vpn_ip)) return true;
  }
  return false;
}

void microlink_untrack_peer_health(microlink_t * ml, uint32_t vpn_ip)
{
  (void)ml;
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if (s_health_peers[i].ip == vpn_ip) {
      s_health_peers[i].ip = 0;
      return;
    }
  }
}

void microlink_notify_peer_health(microlink_t * ml, uint32_t vpn_ip, bool healthy)
{
  (void)ml;
  if (vpn_ip == 0) return;
  int free_slot = -1;
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if (s_health_peers[i].ip == vpn_ip) {
      s_health_peers[i].healthy = healthy;
      return;
    }
    if ((free_slot < 0) && (s_health_peers[i].ip == 0)) free_slot = i;
  }
  if (free_slot >= 0) {
    s_health_peers[free_slot].ip = vpn_ip;
    s_health_peers[free_slot].healthy = healthy;
  }
}

void microlink_pin_peer_ip(microlink_t * ml, uint32_t vpn_ip, bool pin)
{
  (void)ml;
  if (vpn_ip == 0) return;
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if (pin && s_extra_pins[i] == vpn_ip) return; /* already pinned */
    if (!pin && s_extra_pins[i] == vpn_ip) {
      s_extra_pins[i] = 0;
      return;
    }
  }
  if (pin) {
    for (int i = 0; i < ML_EXTRA_PINS; i++) {
      if (s_extra_pins[i] == 0) {
        s_extra_pins[i] = vpn_ip;
        return;
      }
    }
  }
}

static bool is_pinned_peer(microlink_t * ml, uint32_t vpn_ip)
{
  if (vpn_ip == 0) return false;
  if (ml->config.priority_peer_ip != 0 && vpn_ip == ml->config.priority_peer_ip) {
    return true;
  }
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if (s_extra_pins[i] != 0 && vpn_ip == s_extra_pins[i]) {
      return true;
    }
  }
  uint32_t fip = fleet_server_ip_cached();
  return (fip != 0 && vpn_ip == fip);
}

/* Public wrapper so ml_coord can prioritise pinned-peer updates on the
 * coord->wg_mgr queue (their DERP region drives the re-home; dropping it leaves
 * a NAT'd unit relay-unreachable). */
bool ml_wg_is_pinned_peer(microlink_t * ml, uint32_t vpn_ip)
{
  return is_pinned_peer(ml, vpn_ip);
}

/* Public wrapper: is this peer in the health-tracked safety set? Diagnostic use
 * (e.g. /api/peers) to see which peers feed ml_wg_collect_safety_regions. */
bool ml_wg_is_health_tracked(uint32_t vpn_ip)
{
  return is_health_tracked(vpn_ip);
}

/* THE safety-peer membership test — the single definition of "this peer's
 * link carries the safety heartbeat": the configured priority peer OR any
 * health-tracked peer. Used by teardown vetoes, session diagnostics, the
 * rekey-in-flight freeze, LRU-evict protection and the reingest sweep — one
 * predicate, so a future criterion change cannot drift across call sites
 * (6 hand-inlined copies had accumulated before it existed).
 * NOTE: distinct from ml_wg_tx_class_pubkey()'s pinned||health prio rule —
 * pinned covers reachability armor (fleet anchor etc.), not heartbeat carriage. */
static bool is_safety_peer(microlink_t * ml, uint32_t vpn_ip)
{
  return (ml->config.priority_peer_ip != 0 && vpn_ip == ml->config.priority_peer_ip) || is_health_tracked(vpn_ip);
}

/* DERP home region of a peer identified by its 32-byte WG public key.
 * 0 = unknown peer or region not learned. Called from ml_derp_queue_send on the
 * wg_mgr task (the peer-table owner), so this is a same-task read — no lock. */
uint16_t ml_wg_region_for_pubkey(microlink_t * ml, const uint8_t * wg_pubkey)
{
  if (ml == NULL || wg_pubkey == NULL) return 0;
  int idx = find_peer_by_key(ml, wg_pubkey);
  if (idx < 0) return 0;
  return ml->peers[idx].derp_region;
}

/* DERP-egress classification for a destination pubkey, in ONE peer-table walk.
 * Returns the PRIORITY-queue bit: pinned OR health-tracked (their frames never
 * sit behind a disco burst). *mirror_out = heartbeat carriers only (priority OR
 * health-tracked): the fleet pin is reachability armor, not heartbeat carriage,
 * so its frames are not path-diversity mirrored. Safe from the TCPIP thread
 * (wg_derp_output_cb): word-sized reads, worst case a transiently wrong
 * classification, same tolerance as ml_wg_region_for_pubkey. */
bool ml_wg_tx_class_pubkey(microlink_t * ml, const uint8_t * wg_pubkey, bool * mirror_out)
{
  *mirror_out = false;
  if (ml == NULL || wg_pubkey == NULL) return false;
  int idx = find_peer_by_key(ml, wg_pubkey);
  if (idx < 0) return false;
  uint32_t vpn_ip = ml->peers[idx].vpn_ip;
  *mirror_out = is_safety_peer(ml, vpn_ip);
  return *mirror_out || is_pinned_peer(ml, vpn_ip);
}

/* Collect the DISTINCT DERP regions of the safety peers that are exempt from the
 * peer-scaling armor: pinned (incl. priority + fleet + app pins) OR
 * health-tracked. These few peers are the ones the DERP pool must keep an
 * auxiliary connection open for when they home on a non-home region. Writes up
 * to `max` distinct region ids into `out`; returns the count.
 *
 * Concurrency: read from the DERP I/O task while wg_mgr owns/writes the peer
 * table. Reads are word-sized (region ids, active flag) and the worst case of a
 * torn/stale read is a transiently wrong aux region, self-corrected next pass —
 * identical to the existing cross-task diag readers (ml_wg_get_derp_diag). */
int ml_wg_collect_safety_regions(microlink_t * ml, uint16_t * out, int max)
{
  if (ml == NULL || out == NULL || max <= 0) return 0;
  int n = 0;
  for (int i = 0; i < ml->peer_count && n < max; i++) {
    ml_peer_t * p = &ml->peers[i];
    if (!p->active || p->derp_region == 0) continue;
    if (!(is_pinned_peer(ml, p->vpn_ip) || is_health_tracked(p->vpn_ip))) continue;
    bool dup = false;
    for (int k = 0; k < n; k++) {
      if (out[k] == p->derp_region) {
        dup = true;
        break;
      }
    }
    if (!dup) out[n++] = p->derp_region;
  }
  return n;
}

/* Diagnostic: report the DERP region the chip has LEARNED for the management and
 * priority peers (0 = not learned yet). Distinguishes "never learned the server's
 * region -> re-home can't fire" from "learned but re-home/DERP-connect stuck".
 * Exposed via /admin/api/monitor. */
void ml_wg_get_derp_diag(microlink_t * ml, uint16_t * fleet_region, uint16_t * priority_region)
{
  if (fleet_region) {
    *fleet_region = 0;
    uint32_t fip = fleet_server_ip_cached();
    int i = (fip != 0) ? find_peer_by_ip(ml, fip) : -1;
    if (i >= 0) *fleet_region = ml->peers[i].derp_region;
  }
  if (priority_region) {
    *priority_region = 0;
    int i = (ml->config.priority_peer_ip != 0) ? find_peer_by_ip(ml, ml->config.priority_peer_ip) : -1;
    if (i >= 0) *priority_region = ml->peers[i].derp_region;
  }
}

void ml_wg_get_direct_diag(microlink_t * ml, ml_direct_diag_t * out)
{
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  out->active_lan_ip = ml_active_lan_ip();
  if (ml == NULL) return;
  out->pp_vpn_ip = ml->config.priority_peer_ip;
  int i = (ml->config.priority_peer_ip != 0) ? find_peer_by_ip(ml, ml->config.priority_peer_ip) : -1;
  if (i < 0) return;
  ml_peer_t * p = &ml->peers[i];
  out->pp_best_ip = p->best_ip;
  out->pp_best_port = p->best_port;
  out->pp_has_direct = p->has_direct_path;
  out->pp_endpoint_count = p->endpoint_count;
  for (int k = 0; k < p->endpoint_count && k < ML_MAX_ENDPOINTS; k++) {
    out->pp_ep_ip[k] = p->endpoints[k].ip;
    out->pp_ep_port[k] = p->endpoints[k].port;
  }
}

/* DERP re-home diagnostics — counters (not logs) so the re-home path is visible
 * on units with no live-log access. Read via /admin/api/monitor. */
static uint32_t s_diag_selfheal_calls; /* self_heal_rehome invocations      */
static uint32_t s_diag_rehome_calls; /* maybe_rehome invocations          */
static uint32_t s_diag_rehome_ret_region0; /* bailed: peer region unknown (0)   */
static uint32_t s_diag_rehome_ret_notpinned; /* bailed: peer not pinned           */
static uint32_t s_diag_rehome_body; /* passed the guards                 */
static uint32_t s_diag_rehome_applied; /* actually changed the home region  */

void ml_wg_get_rehome_diag(uint32_t out[6])
{
  out[0] = s_diag_selfheal_calls;
  out[1] = s_diag_rehome_calls;
  out[2] = s_diag_rehome_ret_region0;
  out[3] = s_diag_rehome_ret_notpinned;
  out[4] = s_diag_rehome_body;
  out[5] = s_diag_rehome_applied;
}

/* Relay-bound direct-path retry counters — counters (not logs) so the retry
 * loop is verifiable on units with no live-log access, mirroring the re-home
 * diag pattern above. Read via /admin/api/monitor. */
static uint32_t s_diag_relay_retries; /* CMM + forced-sweep rounds fired      */
static uint32_t s_diag_direct_regains; /* has_direct_path false -> true edges */
static uint64_t s_last_relay_refetch_ms; /* rate-limit coord re-fetch on relay-stuck symmetric-NAT safety peer */
static uint32_t s_diag_relay_refetch_reqs; /* coord re-fetch (reconnect) requests issued for endpoint refresh */
static uint32_t s_diag_relay_disco_resets; /* per-peer from-scratch disco resets on a relay-stuck safety peer
                                            * (v2: rate-limited per-peer via p->disco_reset_next_ms) */
static uint32_t s_diag_ep_learn_evictions; /* learn-from-ping ring-evictions on a full endpoint table —
                                            * nonzero = the B-1 wedge trigger occurred and was absorbed */

/* DISCO observability counters — the 2026-08-09 regains-oscillation root cause
 * rested on CALCULATED probe-table/CMM dynamics; these measure them instead.
 * Same counters-not-logs pattern as the diag blocks above; read via
 * /admin/api/monitor. */
static uint32_t s_diag_probe_tbl_hw; /* pending_probes active-slot high water since boot */
static uint32_t s_diag_cmm_rx; /* CallMeMaybe messages received since boot        */
static uint32_t s_diag_regains_safety; /* direct regains on SAFETY peers only (priority
                                        * or health-tracked) — s_diag_direct_regains
                                        * also counts bulk tailnet peers, which masked
                                        * whether the SAFETY path was the one flapping */
static uint32_t s_diag_demote_vetoes; /* direct demotes vetoed by fresh direct WG data
                                       * rx (ml_demote_verdict.h) — counts veto-ticks,
                                       * so it climbs ~1/s for the duration of a
                                       * disco-silent-but-data-alive episode */
static uint64_t s_last_demote_veto_log_ms; /* rate-limit the veto log line */

/* Hitless re-ingest counters (run-20 green drop, 2026-08-11): the three WG
 * session-teardown paths (re-key retire, coord REMOVE, cap-evict) were
 * log-only, so the ENOTCONN green-drop forensics could not tell WHICH fired.
 * Counters-not-logs, read via /admin/api/monitor. The *_vetoes counters mark
 * teardowns of an ACTIVELY-USED safety session that were deferred instead
 * (ml_teardown_veto): nonzero = the guard saved a live safety bond. */
static uint32_t s_diag_rekey_retires; /* stale-entry retires on a key change    */
static uint32_t s_diag_rekey_retire_vetoes; /* ... vetoed: session still live   */
static uint32_t s_diag_peer_removes; /* coord ML_PEER_REMOVE teardowns applied  */
static uint32_t s_diag_remove_vetoes; /* ... vetoed: session still live         */
static uint32_t s_diag_evict_safety_skips; /* LRU evict skipped a safety peer   */

void ml_wg_get_reingest_diag(uint32_t out[6])
{
  out[0] = s_diag_rekey_retires;
  out[1] = s_diag_rekey_retire_vetoes;
  out[2] = s_diag_peer_removes;
  out[3] = s_diag_remove_vetoes;
  out[4] = s_diag_evict_safety_skips;
  out[5] = s_diag_relay_disco_resets; /* from-scratch disco resets on a relay-stuck safety peer (@claude review) */
}

/* WG session-health diag (run-20/21 ENOTCONN forensics): learn-evictions +
 * the WORST safety-peer keypair/init ages. wg_kp_age_max steadily ~<120000
 * (rekey cadence) = healthy; climbing past 120000 = a rekey is starving; a
 * green drop then follows ~60 s later when REJECT_AFTER_TIME kills the key. */
void ml_wg_get_session_diag(microlink_t * ml, uint32_t out[5])
{
  out[0] = s_diag_ep_learn_evictions;
  out[1] = 0; /* max safety keypair age ms (0xFFFFFFFF = a peer has NO valid key) */
  out[2] = 0; /* max safety handshake-init TX age ms */
  out[3] = 0; /* max safety worst any-path rx gap ms (edge-flush receive-stall metric) */
  out[4] = 0; /* WHEN out[3]'s max was recorded (sys_now ms) — dates the gauge */
  if (ml == NULL || ml->wg_netif == NULL) return;
  for (int i = 0; i < ml->peer_count; i++) {
    ml_peer_t * p = &ml->peers[i];
    if (!p->active || p->wg_peer_index < 0) continue;
    bool safety = is_safety_peer(ml, p->vpn_ip);
    if (!safety) continue;
    u32_t kp_age = 0, init_age = 0;
    if (
      wireguardif_peer_handshake_age((struct netif *)ml->wg_netif, (u8_t)p->wg_peer_index, &kp_age, &init_age) ==
      ERR_OK)
    {
      if (kp_age > out[1]) out[1] = kp_age;
      if (init_age != 0xFFFFFFFFu && init_age > out[2]) out[2] = init_age;
    }
    u32_t rx_worst = 0;
    u32_t rx_worst_at = 0;
    if (
      wireguardif_peer_worst_rx_gap((struct netif *)ml->wg_netif, (u8_t)p->wg_peer_index, &rx_worst, &rx_worst_at) ==
        ERR_OK &&
      rx_worst > out[3])
    {
      out[3] = rx_worst;
      out[4] = rx_worst_at;
    }
  }
}

/* True when tearing down this peer's WG session must be deferred: it is a
 * SAFETY peer (priority/health-tracked) and its session shows authenticated
 * data rx within ML_TEARDOWN_RX_FRESH_MS — i.e. the 5 Hz safety heartbeat is
 * flowing RIGHT NOW and a wireguardif_remove_peer would fail those sends
 * ENOTCONN until a fresh handshake (>2 s = machine STOP). See
 * ml_demote_verdict.h for the invariant argument. */
static bool teardown_vetoed(microlink_t * ml, uint32_t vpn_ip, int wg_peer_index)
{
  bool is_safety = is_safety_peer(ml, vpn_ip);
  u32_t age_ms = 0;
  bool valid = ml->wg_netif && wg_peer_index >= 0 &&
               wireguardif_peer_rx_age((struct netif *)ml->wg_netif, (u8_t)wg_peer_index, &age_ms) == ERR_OK;
  return ml_teardown_veto(is_safety, valid, age_ms, ML_TEARDOWN_RX_FRESH_MS);
}

void ml_wg_get_disco_obs_diag(uint32_t out[4])
{
  out[0] = s_diag_probe_tbl_hw;
  out[1] = s_diag_cmm_rx;
  out[2] = s_diag_regains_safety;
  out[3] = s_diag_demote_vetoes;
}

void ml_wg_get_direct_retry_diag(microlink_t * ml, uint32_t out[4])
{
  out[0] = s_diag_relay_retries;
  out[1] = s_diag_direct_regains;
  out[2] = 0;
  out[3] = 0;
  if (ml == NULL) return;
  /* Report the relay-bound safety peers and the earliest armed retry due
   * time, so a bench run can verify the retry ARMED without waiting out the
   * backoff (2026-08-09 falsification: rounds==0 alone cannot distinguish
   * "retry never armed" from "first round simply not due yet"). */
  uint64_t now = ml_get_time_ms();
  uint64_t soonest = 0;
  for (int i = 0; i < ml->peer_count; i++) {
    ml_peer_t * p = &ml->peers[i];
    if (!p->active || p->has_direct_path) continue;
    bool safety = is_safety_peer(ml, p->vpn_ip);
    if (!safety) continue;
    out[2]++;
    if (p->relay_retry_next_ms != 0 && (soonest == 0 || p->relay_retry_next_ms < soonest)) {
      soonest = p->relay_retry_next_ms;
    }
  }
  if (soonest > now) {
    out[3] = (uint32_t)(soonest - now);
  }
}

/* ============================================================================
 * DERP region auto-negotiation — decision side (design §4.2/§6/§8).
 *
 * WHAT CHANGED vs the pre-autoneg code: maybe_rehome_to_priority() used to key
 * the target on the single static config.priority_peer_ip and apply EVERY
 * change break-before-make (derp_home_region write + ML_EVT_DERP_RECONNECT,
 * gapping the relay path for a full TLS handshake). It now (a) derives the
 * target from the app's machine-slot table via the §4.2 primary rule when a
 * callback is registered (Q1: lowest bonded+ARMED slot — deterministic across
 * reboots, unlike arming history), (b) consults TTL/epoch-gated fleet advice
 * ONLY when no machine is configured (§6 — machines and idle remotes), and
 * (c) routes a change through the §7 make-before-break machine whenever a live
 * safety bond exists, under mandatory §8 damping.
 *
 * WHAT STAYED: the pinned-peer guard + diag counters; the management-server
 * anchor fallback ("manageable when no machine online", §4.2); the LOCK path
 * (derp_region_override != 0 preserves today's behavior verbatim — I2); and
 * the no-bond legacy immediate rehome (§6 legacy_rehome — cold-bond must stay
 * fast). All state below is RAM-only: the lock remains the only NVS-persisted
 * region control.
 * ========================================================================== */

/* §8 damping + Phase-3 advice bookkeeping. Owned by the wg_mgr task (the only
 * negotiator writer); exposed cross-task via ml_wg_get_neg_diag (word reads). */
static uint32_t s_neg_damping_suppressed; /* §16: passes suppressed by any §8 guard */
static uint32_t s_neg_cooldown_trips; /* §16: hourly circuit-breaker trips */
static uint64_t s_neg_commit_ring[ML_NEG_MAX_SWITCHES_PER_HOUR]; /* commit timestamps */
static uint16_t s_neg_candidate; /* §8 stability window: current target candidate */
static uint64_t s_neg_candidate_since_ms;
static uint64_t s_neg_last_commit_ms;
static bool s_neg_cooldown_logged;

static struct
{
  uint16_t region;
  uint64_t until_ms;
} s_neg_ban[ML_NEG_REGION_BANS]; /* §8 bad-region cooldown after ROLLBACK */

static uint8_t s_neg_pending_source; /* MICROLINK_REGION_SRC_* of the pending MBB request */
static uint32_t s_neg_advice_applied_epoch; /* last fleet-advice epoch actually applied */
static uint64_t s_neg_last_advice_apply_ms;

static bool neg_region_banned(uint16_t region, uint64_t now)
{
  for (int i = 0; i < ML_NEG_REGION_BANS; i++) {
    if (s_neg_ban[i].region == region && now < s_neg_ban[i].until_ms) return true;
  }
  return false;
}

static void neg_ban_region(uint16_t region, uint64_t now)
{
  int slot = 0;
  for (int i = 0; i < ML_NEG_REGION_BANS; i++) {
    if (s_neg_ban[i].region == region || now >= s_neg_ban[i].until_ms) {
      slot = i;
      break;
    }
  }
  s_neg_ban[slot].region = region;
  s_neg_ban[slot].until_ms = now + ML_NEG_REGION_BAN_MS;
}

static uint32_t neg_switches_last_hour(uint64_t now)
{
  uint32_t n = 0;
  for (int i = 0; i < ML_NEG_MAX_SWITCHES_PER_HOUR; i++) {
    if (s_neg_commit_ring[i] != 0 && (now - s_neg_commit_ring[i]) < 3600000ull) n++;
  }
  return n;
}

static void neg_note_commit(uint64_t now)
{
  /* Overwrite the OLDEST entry — with a ring sized == the hourly cap, the cap
   * check above guarantees a free (aged-out) slot exists whenever we commit. */
  int oldest = 0;
  for (int i = 1; i < ML_NEG_MAX_SWITCHES_PER_HOUR; i++) {
    if (s_neg_commit_ring[i] < s_neg_commit_ring[oldest]) oldest = i;
  }
  s_neg_commit_ring[oldest] = now;
  s_neg_last_commit_ms = now;
}

void ml_wg_get_neg_diag(uint32_t out[3])
{
  out[0] = s_neg_damping_suppressed;
  out[1] = s_neg_cooldown_trips;
  out[2] = neg_switches_last_hour(ml_get_time_ms());
}

/* §6 live_bond_active(): any health-tracked safety peer currently healthy.
 * The remote comparator feeds per-machine-slot health every tick and machn
 * feeds per-remote health, so this is a faithful bond signal on both roles.
 * Builds that never health-track (no pstop app) read false => they keep the
 * legacy immediate-rehome path exactly as today. */
bool ml_wg_live_bond_active(void)
{
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    if (s_health_peers[i].ip != 0 && s_health_peers[i].healthy) return true;
  }
  return false;
}

/* §4.2 primary-machine selection. Returns the VPN IP whose region the chip
 * should home on, 0 = none known. Rules (Q1 decided 2026-08-10):
 *   1/2. the lowest-index bonded+ARMED machine slot (the callback reports it —
 *        dcs owns slots + arming; slot order is NVS config, identical after
 *        power loss, unlike first-armed order);
 *   3.   none armed -> lowest CONFIGURED slot whose region we already learned
 *        (checked here — the app can't know regions);
 *   4.   legacy static priority_peer_ip config (unchanged when no callback is
 *        registered, i.e. machines and non-dcs builds behave as today). */
static uint32_t neg_primary_ip(microlink_t * ml)
{
  microlink_primary_machine_cb_t cb = ml->primary_cb;
  if (cb != NULL) {
    microlink_primary_machine_info_t info;
    memset(&info, 0, sizeof(info));
    cb(ml->primary_cb_ctx, &info);
    if (info.armed_primary_ip != 0) {
      return info.armed_primary_ip;
    }
    for (int i = 0; i < info.configured_count && i < MICROLINK_PRIMARY_MAX_MACHINES; i++) {
      uint32_t ip = info.configured_ips[i];
      int idx = (ip != 0) ? find_peer_by_ip(ml, ip) : -1;
      if (idx >= 0 && ml->peers[idx].derp_region != 0) {
        return ip;
      }
    }
    if (info.configured_count > 0) {
      /* Machines are configured but none armed and no region learned yet:
       * HOLD (self-heal retries in 3 s once the netmap delivers a region)
       * rather than hopping onto some other peer's region meanwhile. */
      return 0;
    }
  }
  return ml->config.priority_peer_ip;
}

/* True when the chip has ANY machine configured — such a chip must ignore
 * fleet advice entirely (§6: the netmap is fresher and machine-authoritative). */
static bool neg_machine_configured(microlink_t * ml)
{
  microlink_primary_machine_cb_t cb = ml->primary_cb;
  if (cb != NULL) {
    microlink_primary_machine_info_t info;
    memset(&info, 0, sizeof(info));
    cb(ml->primary_cb_ctx, &info);
    return info.configured_count > 0 || info.armed_primary_ip != 0;
  }
  return ml->config.priority_peer_ip != 0;
}

/* Fleet advice region if it may drive the target right now, else 0. An
 * ALREADY-APPLIED epoch keeps supplying its region until TTL expiry (so the
 * management-anchor fallback can't fight a committed advice); a NEW epoch is
 * additionally rate-limited to one apply per ML_NEG_ADVICE_MIN_INTERVAL_MS —
 * chip-side, so even a flapping fleet cannot exceed it (I3/§8). */
static uint16_t neg_advice_target(microlink_t * ml, uint64_t now)
{
  uint16_t region = ml->advice_region;
  uint32_t expires_s = ml->advice_expires_s;
  if (region == 0 || expires_s == 0) return 0;
  if ((uint32_t)(now / 1000u) >= expires_s) return 0; /* TTL expired (I3) */
  if (
    ml->advice_epoch != s_neg_advice_applied_epoch && s_neg_last_advice_apply_ms != 0 &&
    (now - s_neg_last_advice_apply_ms) < ML_NEG_ADVICE_MIN_INTERVAL_MS)
  {
    return 0; /* new advice, honored at most every 30 min */
  }
  return region;
}

/* §6 target selection (normative pseudocode in the design):
 *   target = region(primary_machine())            — §4.2
 *   if 0:  fleet advice, ONLY with no machine configured (machines/idle chips)
 *   if 0:  management-server anchor when the invoking peer IS the fleet server
 *          (kept verbatim from the pre-autoneg fallback chain, §4.2)
 *   if 0:  no local choice — today's server-HomeDERP default path stays. */
static uint16_t neg_target_region(microlink_t * ml, const ml_peer_t * p, uint8_t * src_out)
{
  uint64_t now = ml_get_time_ms();
  uint32_t prim = neg_primary_ip(ml);
  if (prim != 0) {
    int idx = find_peer_by_ip(ml, prim);
    if (idx >= 0 && ml->peers[idx].derp_region != 0) {
      *src_out = MICROLINK_REGION_SRC_AUTO_PRIMARY;
      return ml->peers[idx].derp_region;
    }
    /* Primary known but its region not learned yet: fall through to the
     * anchor/advice chain (matches the old fallback shape) — the self-heal
     * cadence re-evaluates once the region arrives. */
  }
  if (!neg_machine_configured(ml)) {
    uint16_t a = neg_advice_target(ml, now);
    if (a != 0) {
      *src_out = MICROLINK_REGION_SRC_AUTO_FLEET;
      return a;
    }
  }
  if (p != NULL) {
    uint32_t fip = fleet_server_ip_cached();
    if (fip != 0 && p->vpn_ip == fip && p->derp_region != 0) {
      /* Management anchor: keeps the chip manageable when no machine is
       * online. Reported as plain "auto" — no machine/advice selection. */
      *src_out = MICROLINK_REGION_SRC_AUTO;
      return p->derp_region;
    }
  }
  return 0;
}

/* Cancel any pending MBB request (lock landed, bond died, target changed).
 * The executor observes the generation bump and aborts to IDLE (§7). */
static void neg_cancel_mbb(microlink_t * ml)
{
  if (ml->mbb_target_region != 0) {
    ml->mbb_target_region = 0;
    ml->mbb_generation++;
  }
}

/* Q2 surfacing: record an auto-applied region change (legacy rehome or MBB
 * commit) for /state.json, the UI badge and the fleet check-in. */
static void neg_note_auto_apply(microlink_t * ml, uint16_t region, uint8_t src, uint64_t now)
{
  ml->derp_region_src = src;
  ml->derp_region_auto_applied = region;
  ml->derp_region_auto_applies++;
  ml->derp_region_auto_apply_s = (uint32_t)(now / 1000u);
}

/* Apply a computed target region (§6 tail). Three paths:
 *   LOCK      — preserve today's behavior verbatim (I2): assert the advert
 *               owner + reconnect; connect/advert use the override anyway.
 *   no bond   — legacy immediate rehome (§6 legacy_rehome): correct and FASTER
 *               than MBB for boot/cold recovery; behavior identical to today.
 *   live bond — §7 MBB only (I1: the bond is never dropped by a switch),
 *               gated by every §8 damping guard. */
/* True while any safety peer's WG session is missing its keypair or a rekey
 * is overdue with initiations actively firing. A region switch in that window
 * re-routes the DERP leg the handshake needs (silent home-conn fallback +
 * old-home reap, audit §1c) — the second-transition trigger behind both
 * run-20/21 green drops (dwell expiry at T+10min, keypair death at T+~14min).
 * Freezing the negotiator here costs only switch latency, never safety. */
static bool safety_rekey_inflight(microlink_t * ml)
{
  if (!ml->wg_netif) return false;
  for (int i = 0; i < ml->peer_count; i++) {
    ml_peer_t * p = &ml->peers[i];
    if (!p->active || p->wg_peer_index < 0) continue;
    bool safety = is_safety_peer(ml, p->vpn_ip);
    if (!safety) continue;
    u32_t kp_age = 0, init_age = 0;
    if (
      wireguardif_peer_handshake_age((struct netif *)ml->wg_netif, (u8_t)p->wg_peer_index, &kp_age, &init_age) !=
      ERR_OK)
    {
      continue;
    }
    if (kp_age == 0xFFFFFFFFu) {
      /* No valid keypair: freeze only if we're actively trying to handshake
       * (a never-connected passive peer must not pin the negotiator). */
      if (init_age != 0xFFFFFFFFu && init_age < 30000u) return true;
      continue;
    }
    /* Keypair older than REKEY_AFTER_TIME (120 s, wireguard.h) with a recent
     * initiation = a rekey is in flight and not completing. */
    if (kp_age > 120000u && init_age < 30000u) return true;
  }
  return false;
}

static void neg_apply_target(microlink_t * ml, uint16_t target, uint8_t src)
{
  uint64_t now = ml_get_time_ms();

  if (ml->derp_region_override != 0) {
    ml->priority_peer_region = target; /* advertised as PreferredDERP (override still wins) */
    /* Pin->MBB: pin convergence is owned by neg_pin_tick — gapless MBB while
     * the home conn is alive, the DERP task's own home-reconnect (straight to
     * the override) when it is not; a teardown here would starve heartbeat rx
     * (docs/STAGE1_PIN_MBB_DESIGN.md). Cancel only a FOREIGN in-flight MBB
     * (autoneg toward some other region — the I2 "lock wins instantly"
     * intent); NEVER the pin's own, or the pin could never complete (this
     * runs every 3 s). */
    if (ml->mbb_target_region != 0 && ml->mbb_target_region != ml->derp_region_override) {
      neg_cancel_mbb(ml);
    }
    return;
  }

  if (ml->derp_home_region == target) {
    /* Fixed point: keep asserting the advert owner (as the old code did) and
     * make sure no stale MBB request lingers. The Q2 surfacing fields track
     * the CURRENT auto-selected region/owner (no counter bump — nothing was
     * applied), so /state.json + the badge stay truthful in steady state. */
    ml->priority_peer_region = target;
    if (src != MICROLINK_REGION_SRC_AUTO) {
      ml->derp_region_src = src; /* ownership label follows the current driver */
      ml->derp_region_auto_applied = target;
    }
    s_neg_candidate = 0;
    neg_cancel_mbb(ml);
    return;
  }

  if (!ml_wg_live_bond_active()) {
    neg_cancel_mbb(ml); /* the immediate rehome supersedes any in-flight MBB */
    ml->priority_peer_region = target;
    s_diag_rehome_applied++;
    ESP_LOGW(
      TAG,
      "Re-homing DERP region %u -> %u (%s; no live bond: legacy immediate reconnect)",
      (unsigned)ml->derp_home_region,
      (unsigned)target,
      microlink_region_source_str(src));
    ml->derp_home_region = target;
    xEventGroupSetBits(ml->events, ML_EVT_DERP_RECONNECT);
    neg_note_auto_apply(ml, target, src, now);
    if (src == MICROLINK_REGION_SRC_AUTO_FLEET) {
      s_neg_advice_applied_epoch = ml->advice_epoch;
      s_neg_last_advice_apply_ms = now;
      /* Q2: an advice apply must be visible to peers + operator immediately —
       * push the new PreferredDERP now instead of waiting for the next
       * periodic MapRequest. */
      (void)microlink_request_announce(ml);
    }
    return;
  }

  /* ---- live bond: §8 damping gates, then request the §7 MBB switch ---- */
  if (neg_region_banned(target, now)) {
    s_neg_damping_suppressed++;
    return;
  }
  if (safety_rekey_inflight(ml)) {
    /* Never start a region transition while a safety session's handshake is
     * struggling — the switch would re-route the very DERP leg the rekey
     * needs (run-20/21 trigger). Re-evaluated next tick; costs latency only. */
    s_neg_damping_suppressed++;
    return;
  }
  if (s_neg_last_commit_ms != 0 && (now - s_neg_last_commit_ms) < ML_NEG_MIN_DWELL_MS) {
    s_neg_damping_suppressed++;
    return;
  }
  if (s_neg_candidate != target) {
    /* New candidate: (re)start the stability window and preempt any MBB
     * toward a stale target (§7: newer target restarts from IDLE via ABORT). */
    s_neg_candidate = target;
    s_neg_candidate_since_ms = now;
    if (ml->mbb_target_region != 0 && ml->mbb_target_region != target) {
      neg_cancel_mbb(ml);
    }
    return;
  }
  if ((now - s_neg_candidate_since_ms) < ML_NEG_STABILITY_MS) {
    return; /* §8: absorbs netmap patch bursts / region flap without churn */
  }
  if (neg_switches_last_hour(now) >= ML_NEG_MAX_SWITCHES_PER_HOUR) {
    if (!s_neg_cooldown_logged) {
      s_neg_cooldown_trips++;
      s_neg_cooldown_logged = true;
      ESP_LOGE(
        TAG,
        "DERP region switch circuit-breaker TRIPPED (%d switches/h): holding home region %u, wanted %u",
        ML_NEG_MAX_SWITCHES_PER_HOUR,
        (unsigned)ml->derp_home_region,
        (unsigned)target);
    }
    s_neg_damping_suppressed++;
    return;
  }
  s_neg_cooldown_logged = false;
  if (ml->mbb_target_region == target) {
    return; /* already requested; the executor is working on it */
  }
  s_neg_pending_source = src;
  ml->mbb_target_region = target;
  ml->mbb_generation++;
  ESP_LOGW(
    TAG,
    "MBB home switch requested: region %u -> %u (%s; bond live, make-before-break)",
    (unsigned)ml->derp_home_region,
    (unsigned)target,
    microlink_region_source_str(src));
}

/* ============================================================================
 * Stage-1 pin->MBB (docs/STAGE1_PIN_MBB_DESIGN.md): drive the operator region
 * pin (derp_region_override) GAPLESSLY. While the home conn is alive, the pin
 * is applied via the §7 make-before-break executor (open target as aux, prove,
 * swap the home index — no teardown, no rx gap). With no live home there is
 * nothing to protect: the DERP task's own home-reconnect path already connects
 * straight to ml_effective_home_region() == the override.
 *
 * Fallback semantics (operator decision 2026-08-13): RETRY FOREVER, NEVER GAP.
 * A pin toward an unprovable region keeps the old (working) home serving and
 * retries with 5 s -> 15 s -> 60 s backoff (60 s cap, RAM-only) — a pin is a
 * command, not advice, so rollbacks neither ban the region nor give up. The
 * unfulfilled pin is visible as pin_pending in /api/status.
 * State + counters owned by the wg_mgr task (single writer); counters read
 * cross-task via ml_wg_get_pin_diag (word reads). */
static struct
{
  uint32_t mbb_requests; /* MBB commands issued for the pin (cumulative) */
  uint32_t mbb_commits; /* pin MBBs that committed (cumulative) */
  uint32_t mbb_retries; /* re-issues after a failed prove (cumulative) */
  uint64_t retry_at_ms; /* 0 = no backoff pending */
  uint32_t backoff_ms; /* current backoff step */
  uint16_t last_target; /* region of the pin MBB we issued (edge 4: cancel-on-clear) */
  uint32_t absent_resyncs; /* coord full-resyncs forced by an absent pinned peer (cumulative) */
  uint64_t absent_next_ms; /* pin-presence heal backoff gate */
  uint32_t absent_backoff_ms;
} s_pin; /* never bulk-reset: counters are cumulative, reset sites clear differing subsets */

/* Stage-0b gauge (run-29 disarm class #4/#10): worst single wg_mgr loop
 * iteration — heartbeats ride THIS task (WG rx -> decrypt -> pstop), so a
 * multi-second stall here gaps every remote simultaneously with the DERP
 * task innocent. Owned by wg_mgr; word-sized cross-task read. */
static uint32_t s_diag_wg_max_iter_ms;

uint32_t ml_wg_get_max_iter_ms(void)
{
  return s_diag_wg_max_iter_ms;
}

void ml_wg_get_pin_diag(uint32_t out[5])
{
  out[0] = s_pin.mbb_requests;
  out[1] = s_pin.mbb_commits;
  out[2] = s_pin.mbb_retries;
  out[3] = (s_pin.retry_at_ms != 0) ? 1u : 0u; /* backoff currently pending */
  out[4] = s_pin.absent_resyncs; /* forced coord resyncs for a pinned-but-absent peer */
}

/* Pin-presence self-heal: an extra-pinned IP
 * (a configured pstop machine / fleet anchor) that is ABSENT from the peer
 * table can never be reached — and with OmitPeers the control plane will not
 * re-send it unprompted, so the outage is PERMANENT until a lucky reboot.
 * How it happens: the initial full-netmap ingest can race ahead of the app's
 * boot-time pstop_slot_pins_sync(); on an over-cap tailnet the machine is
 * then dropped UN-pinned ("Peer table full, cannot add") and no later patch
 * re-delivers it. Heal: detect pinned-but-absent on the 3 s cadence and kick
 * ML_CMD_FORCE_RECONNECT (re-register -> full FETCH_PEERS re-ingests the map;
 * by then the pin is registered, so admission LRU-evicts a non-pinned peer).
 * Escalating backoff 60 s -> 10 min cap: a reconnect is heavier than an
 * announce, and a misconfigured (never-on-tailnet) machine IP must not storm
 * control — it stays visible via the counter + log instead. (Statics live
 * with the other pin counters above, ahead of ml_wg_get_pin_diag.) */
static void pin_presence_heal(microlink_t * ml)
{
  if (!(xEventGroupGetBits(ml->events) & ML_EVT_COORD_REGISTERED)) {
    return; /* boot/reconnect grace: the full map may still be ingesting */
  }
  uint32_t absent_ip = 0;
  for (int i = 0; i < ML_EXTRA_PINS; i++) {
    uint32_t ip = s_extra_pins[i];
    if (ip != 0 && find_peer_by_ip(ml, ip) < 0) {
      absent_ip = ip;
      break;
    }
  }
  uint64_t now = ml_get_time_ms();
  if (absent_ip == 0) {
    s_pin.absent_backoff_ms = 0; /* healthy — reset the backoff ladder */
    s_pin.absent_next_ms = 0; /* and the gate, so a re-absence heals immediately */
    return;
  }
  if (s_pin.absent_next_ms != 0 && now < s_pin.absent_next_ms) {
    return;
  }
  s_pin.absent_backoff_ms =
    (s_pin.absent_backoff_ms == 0) ? 60000 : (s_pin.absent_backoff_ms >= 300000 ? 600000 : s_pin.absent_backoff_ms * 5);
  s_pin.absent_next_ms = now + s_pin.absent_backoff_ms;
  s_pin.absent_resyncs++;
  char ipstr[16];
  microlink_ip_to_str(absent_ip, ipstr);
  ESP_LOGW(
    TAG,
    "Pinned peer %s ABSENT from peer table — forcing coord resync #%u (next retry in %us)",
    ipstr,
    (unsigned)s_pin.absent_resyncs,
    (unsigned)(s_pin.absent_backoff_ms / 1000));
  ml_coord_cmd_t cmd = ML_CMD_FORCE_RECONNECT;
  (void)xQueueSend(ml->coord_cmd_queue, &cmd, 0); /* full = one already pending */
}

static void neg_pin_tick(microlink_t * ml)
{
  uint16_t pin = ml->derp_region_override;
  if (pin == 0) {
    /* Edge 4: pin cleared while its own MBB is in flight — cancel it (the
     * generation bump aborts the executor back to IDLE, no ban). Autoneg
     * resumes ownership on its own cadence. */
    if (s_pin.last_target != 0 && ml->mbb_target_region == s_pin.last_target) {
      neg_cancel_mbb(ml);
    }
    s_pin.last_target = 0;
    s_pin.retry_at_ms = 0;
    s_pin.backoff_ms = 0;
    return;
  }
  /* Cross-task read of the DERP-task-owned home conn: volatile word reads. A
   * torn/stale read is conservative — worst case we wait one 3 s tick. */
  ml_derp_conn_t * home = &ml->derp[ml->derp_home_slot];
  bool home_alive = home->connected;
  if (pin != s_pin.last_target) {
    /* Operator re-pinned to a DIFFERENT region: the old target's backoff must
     * not delay the new command. */
    s_pin.retry_at_ms = 0;
    s_pin.backoff_ms = 0;
  }
  if (home_alive && home->region_id == pin) {
    s_pin.retry_at_ms = 0; /* pin satisfied — clear any backoff */
    s_pin.backoff_ms = 0;
    return;
  }
  if (!home_alive) {
    /* No live home = no gap to protect. The DERP I/O task's home-reconnect
     * path connects to ml_effective_home_region() (= the override) on its
     * own; issuing an MBB here would just race it. Nothing to do. */
    return;
  }
  uint64_t now = ml_get_time_ms();
  if (s_pin.retry_at_ms != 0 && now < s_pin.retry_at_ms) {
    return; /* backing off after a failed prove */
  }
  if (ml->mbb_target_region == pin) {
    return; /* already in flight toward the pin — let the executor work */
  }
  /* Issue the MBB command (same single-writer interface autoneg uses). The
   * §8 damping/ban guards are deliberately NOT consulted: an operator pin is
   * a command; the mbb_target dedupe above is the only rate limit needed. */
  s_neg_pending_source = MICROLINK_REGION_SRC_LOCKED;
  s_pin.retry_at_ms = 0; /* leaving backoff: the re-issued MBB is in flight, not waiting —
                          * keeps pin_backoff_pending telemetry truthful */
  ml->mbb_target_region = pin;
  ml->mbb_generation++;
  s_pin.last_target = pin;
  s_pin.mbb_requests++;
  ESP_LOGW(
    TAG,
    "PIN via MBB: home region %u -> %u (gapless; old home serves until the new conn proves)",
    (unsigned)home->region_id,
    (unsigned)pin);
}

/* Consume the MBB executor's outcome (posted by the DERP I/O task). Runs on
 * the wg_mgr 3 s self-heal cadence — the advert/announce stays single-writer
 * (this task), off the 10 Hz safety send path. */
static void neg_consume_mbb_outcome(microlink_t * ml)
{
  uint8_t oc = ml->mbb_outcome;
  if (oc == ML_MBB_OUTCOME_NONE) return;
  uint16_t region = ml->mbb_outcome_region;
  ml->mbb_outcome = ML_MBB_OUTCOME_NONE;
  neg_cancel_mbb(ml); /* command served either way; executor is back at IDLE */
  uint64_t now = ml_get_time_ms();

  if (oc == ML_MBB_OUTCOME_COMMITTED) {
    bool pin_commit = (ml->derp_region_override != 0 && region == ml->derp_region_override);
    if (pin_commit) {
      s_pin.mbb_commits++; /* Stage-1: the pin landed gaplessly */
      s_pin.retry_at_ms = 0;
      s_pin.backoff_ms = 0;
    }
    /* The commit ring feeds AUTONEG's switches/hour circuit breaker — operator
     * pin commits must not count against it, or a few pins in an hour suppress
     * legitimate autoneg switching for up to an hour after unlock. The advert
     * tail below is still wanted for pins. */
    if (!pin_commit) {
      neg_note_commit(now);
    }
    /* §6/§7 SWITCH_ADVERT tail: an MBB-committed region is treated exactly
     * like priority_peer_region — it owns the PreferredDERP advert and arms
     * the MapResponse authoritative guard so the server can't revert it. */
    ml->priority_peer_region = region;
    s_diag_rehome_applied++;
    neg_note_auto_apply(ml, region, s_neg_pending_source, now);
    if (s_neg_pending_source == MICROLINK_REGION_SRC_AUTO_FLEET) {
      s_neg_advice_applied_epoch = ml->advice_epoch;
      s_neg_last_advice_apply_ms = now;
    }
    /* Live PreferredDERP re-advert — the shipped lock plumbing propagates it;
     * peers migrate to the new region while the OLD conn is still up (§7
     * DRAIN_OLD holds it via the want-set until they do). */
    (void)microlink_request_announce(ml);
    ESP_LOGW(
      TAG,
      "MBB COMMIT: DERP home region now %u (%s); PreferredDERP re-advert queued, old region drains via want-set",
      (unsigned)region,
      microlink_region_source_str(s_neg_pending_source));
  } else if (oc == ML_MBB_OUTCOME_ROLLED_BACK) {
    if (ml->derp_region_override != 0 && region == ml->derp_region_override) {
      /* Stage-1 retry-forever: a pin is a command, not advice — no region ban,
       * no give-up. Back off 5 s -> 15 s -> 60 s (cap) and re-issue; the old
       * (working) home keeps serving throughout, so green is never at risk.
       * The unfulfilled pin is visible as pin_pending in /api/status. */
      s_pin.backoff_ms = (s_pin.backoff_ms == 0) ? 5000 : (s_pin.backoff_ms == 5000 ? 15000 : 60000);
      s_pin.retry_at_ms = now + s_pin.backoff_ms;
      s_pin.mbb_retries++;
      s_neg_candidate = 0;
      ESP_LOGW(
        TAG,
        "PIN MBB ROLLBACK: region %u failed proving — retrying in %us (no ban; old home keeps serving)",
        (unsigned)region,
        (unsigned)(s_pin.backoff_ms / 1000));
      return;
    }
    neg_ban_region(region, now); /* §8: 15-min cooldown before retrying an unprovable region */
    s_neg_candidate = 0; /* restart the stability window from scratch */
    ESP_LOGW(
      TAG,
      "MBB ROLLBACK: region %u failed proving — banned %d min, home untouched (bond never at risk)",
      (unsigned)region,
      (int)(ML_NEG_REGION_BAN_MS / 60000));
  } else {
    /* ABORTED (lock landed / preemption): no ban — nothing was proven bad. */
    s_neg_candidate = 0;
  }
}

/* Home our DERP connection on a pinned peer's region.
 * microlink's HOME DERP connection is where peers reach us, and a Tailscale
 * DERP server only delivers to peers connected to that same server — so if the
 * chip homes on a different region than a peer it must reach, its relayed
 * DISCO/WG-init never arrives and the link only forms after a `tailscale ping`
 * FROM that peer. Target selection is §4.2/§6 (neg_target_region above); the
 * apply path is lock/legacy/MBB (neg_apply_target). Called wherever a pinned
 * peer's derp_region is (re)learned, plus the 3 s self-heal cadence. */
static void maybe_rehome_to_priority(microlink_t * ml, const ml_peer_t * p)
{
  s_diag_rehome_calls++;
  if (p->derp_region == 0) {
    s_diag_rehome_ret_region0++;
    return;
  }
  if (!is_pinned_peer(ml, p->vpn_ip)) {
    s_diag_rehome_ret_notpinned++;
    return;
  }
  s_diag_rehome_body++;
  uint8_t src = MICROLINK_REGION_SRC_AUTO;
  uint16_t target = neg_target_region(ml, p, &src);
  if (target == 0) {
    return; /* no local choice — today's server-HomeDERP default stands (§6) */
  }
  neg_apply_target(ml, target, src);
}

static int find_peer_by_disco_key(microlink_t * ml, const uint8_t * disco_key)
{
  for (int i = 0; i < ml->peer_count; i++) {
    if (ml->peers[i].active && memcmp(ml->peers[i].disco_key, disco_key, 32) == 0) {
      return i;
    }
  }
  return -1;
}

/* Install ml->peers[idx] into wireguard-lwip so inbound handshake
 * initiations from this peer's static key get answered.
 *
 * Shared by the netmap ingest path (add_peer) and the boot-time NVS peer
 * preseed (ml_wg_mgr_task). The preseed exists because of the cold-bond
 * far-side gap (bench 2026-08-08/09): after a machine (machn) reboot, its
 * coordination netmap can be slow — or entirely blocked — to re-arrive, and
 * until each operator remote's key is installed here the WG layer SILENTLY
 * ignores that remote's handshake initiations. The remote then sits at
 * peer-present-but-no-keypair: sess_sendto ERR_CONN -> errno ENOTCONN,
 * sent=0, for 27 min on the bench; restarting the REMOTE did not help, only
 * a machn restart (fresh netmap) did. Installing the NVS-cached peers at
 * boot lets those inbound handshakes succeed immediately, netmap or not. */
static void wg_install_iface_peer(microlink_t * ml, int idx)
{
  ml_peer_t * p = &ml->peers[idx];
  if (!ml->wg_netif || p->wg_peer_index >= 0) {
    return; /* no WG interface yet, or already installed */
  }
  struct netif * netif = (struct netif *)ml->wg_netif;

  /* Convert peer public key to base64 */
  char peer_b64[64];
  key_to_base64(p->public_key, peer_b64, sizeof(peer_b64));

  struct wireguardif_peer wg_peer;
  wireguardif_peer_init(&wg_peer);

  wg_peer.public_key = peer_b64;
  wg_peer.preshared_key = NULL;

  /* Allowed IP: PEER's VPN IP
     * wireguard-lwip uses allowed_ip for TWO purposes:
     * 1. Outbound routing: peer_lookup_by_allowed_ip() matches DESTINATION
     *    IP to find which peer to route to (wireguardif.c:338)
     * 2. Inbound validation: checks decrypted packet SOURCE IP matches
     *    peer's allowed_source_ips (wireguardif.c:507)
     * Must be set to the PEER's VPN IP for both to work correctly. */
  uint8_t ip_a = (p->vpn_ip >> 24) & 0xFF;
  uint8_t ip_b = (p->vpn_ip >> 16) & 0xFF;
  uint8_t ip_c = (p->vpn_ip >> 8) & 0xFF;
  uint8_t ip_d = p->vpn_ip & 0xFF;
  IP4_ADDR(&wg_peer.allowed_ip.u_addr.ip4, ip_a, ip_b, ip_c, ip_d);
  IP4_ADDR(&wg_peer.allowed_mask.u_addr.ip4, 255, 255, 255, 255);

  /* Start every peer DERP-only — NEVER install a netmap-advertised endpoint
   * as the live WG endpoint.
   *
   * Root cause of the DERP-only-unreachable bug (2026-08-02): the
   * netmap/STUN endpoints a peer advertises are UNVALIDATED candidates. We
   * keep them in p->endpoints[] purely for DISCO probing. But
   * wireguardif_add_peer() copies wg_peer.endpoint_ip straight into
   * peer->ip (wireguardif.c:1084-1087), and wireguardif_peer_output()
   * (wireguardif.c:158) then routes ALL WireGuard traffic — including the
   * handshake RESPONSE — to peer->ip via direct UDP, bypassing DERP
   * entirely whenever peer->ip is non-any.
   *
   * Behind a hard / endpoint-dependent NAT that advertised endpoint is
   * unreachable in the peer->chip direction, so every reply the chip sends
   * blackholes and the WG session never forms — while relayed DISCO pongs
   * (which process_disco_ping sends *explicitly* over DERP) still return,
   * producing the pathognomonic "tailscale ping OK / TCP+HTTP data dead"
   * asymmetry (relay rx stuck at a few hundred bytes).
   *
   * Fix: leave the live endpoint blank so the peer is DERP-homed. The live
   * direct endpoint is installed only after PROOF of bidirectional
   * reachability — a txid-matched DISCO direct pong (process_disco_pong ->
   * wireguardif_update_endpoint + wireguardif_connect) or a genuinely
   * received direct WG packet (update_peer_addr roaming). This mirrors
   * tailscaled/magicsock: DERP-home first, upgrade to a direct path only
   * once it is validated both ways. DISCO probing of p->endpoints[]
   * continues to run, so genuinely reachable direct paths still upgrade
   * within one probe interval. */
  ip_addr_set_any(false, &wg_peer.endpoint_ip);
  wg_peer.endport_port = 0;

  wg_peer.keep_alive = 25;

  u8_t wg_peer_idx = WIREGUARDIF_INVALID_INDEX;
  err_t wg_err = wireguardif_add_peer(netif, &wg_peer, &wg_peer_idx);

  if (wg_err == ERR_OK && wg_peer_idx != WIREGUARDIF_INVALID_INDEX) {
    p->wg_peer_index = wg_peer_idx;

    /* Verify the WG internal peer key matches what we passed */
    struct wireguard_device * dev = (struct wireguard_device *)netif->state;
    if (dev && wg_peer_idx < WIREGUARD_MAX_PEERS) {
      struct wireguard_peer * wp = &dev->peers[wg_peer_idx];
      bool key_match = (memcmp(wp->public_key, p->public_key, 32) == 0);
      ESP_LOGI(
        TAG,
        "WG peer added: wg_idx=%d internal_key=%02x%02x%02x%02x %s",
        wg_peer_idx,
        wp->public_key[0],
        wp->public_key[1],
        wp->public_key[2],
        wp->public_key[3],
        key_match ? "KEY_OK" : "KEY_MISMATCH!");
    }

    /* DON'T initiate handshakes to all peers on add — Tailscale's lazy
         * peer config means our INIT to an idle peer gets dropped on their
         * end. EXCEPT for the priority peer (set via priority_peer_ip): for
         * that one we actively send WG INIT via DERP. The peer's
         * magicsock receiveIPv4() will see the DERP-delivered INIT, call
         * noteRecvActivity() → maybeReconfigWireguardLocked() to add us
         * to wireguard-go, then complete the handshake. This is how we
         * get a working WG session WITHOUT enabling DISCO (DISCO load
         * still wedges the chip on a busy tailnet even with allowlist
         * filtering of incoming probes — see v15.7.1 attempt). */
    if (ml->config.priority_peer_ip != 0 && p->vpn_ip == ml->config.priority_peer_ip) {
      wireguardif_connect_derp(netif, (u8_t)wg_peer_idx);
      /* Wake the remote's magicsock so it lazily adds us to
                 * wireguard-go. A bare WG-init relayed over DERP does NOT
                 * drive the peer's lazy reconfig — only an inbound DISCO
                 * ping does (that is why a normal peer otherwise has to
                 * `tailscale ping` us first). Fire one now, unconditionally
                 * (bypasses the enable_disco gate on the paths below); it is
                 * DERP-carried and embeds our node key, so a peer with a
                 * stale/rotated disco key can still map us. The sustained
                 * retry lives in disco_periodic_probes. */
      disco_send_ping_to_peer(ml, idx, true);
      /* Near-zero failover: mirror this peer's encrypted data over
                 * BOTH direct and DERP once a direct path forms. Set here and
                 * persisted in the WG peer struct (survives P7 re-ingests); it
                 * only takes effect while a direct endpoint exists, so it's a
                 * no-op until DISCO upgrades the path. */
      wireguardif_set_dual_path(netif, (u8_t)wg_peer_idx, true);
      ESP_LOGI(TAG, "WG: triggered DERP handshake to priority peer %s (dual-path)", p->hostname);
    } else {
      ESP_LOGI(TAG, "WG peer ready (passive), waiting for peer-initiated handshake");
    }
  } else {
    ESP_LOGW(TAG, "wireguardif_add_peer failed: %d", wg_err);
    p->wg_peer_index = -1;
  }
}

/* §7c learned-region stash: REMOVE→re-ADD of a pinned peer lands in a fresh
 * (or reused) slot with nothing to preserve, and machn's map omits the fleet
 * node's region — so the learned region died on every re-ingest cycle and
 * mirror diversity silently followed. Keyed by vpn_ip (pin identity is
 * IP-keyed; also survives key rotation), TTL-bounded so a fleet that MOVED
 * regions while absent cannot pin a stale one indefinitely. wg_mgr-task-owned. */
typedef struct
{
  uint32_t vpn_ip;
  uint16_t region;
  uint64_t stamp_ms;
} region_stash_t;

static region_stash_t s_region_stash[ML_REGION_STASH_SLOTS];
static uint32_t s_diag_region_stash_restores;
static uint32_t s_diag_readds_skipped;
/* Indexed by first-failing §7a skip clause: 0=!existing 1=no-wg-slot 2=vpn_ip
 * 3=disco_key 4=hostname 5=region 6=endpoints. */
static uint32_t s_diag_skipfail[7];

void ml_wg_get_ingest_diag(uint32_t out[2])
{
  out[0] = s_diag_readds_skipped;
  out[1] = s_diag_region_stash_restores;
}

void ml_wg_get_skipfail_diag(uint32_t out[7])
{
  for (int i = 0; i < 7; i++) out[i] = s_diag_skipfail[i];
}

static void region_stash_put(uint32_t vpn_ip, uint16_t region)
{
  int slot = 0;
  for (int i = 0; i < ML_REGION_STASH_SLOTS; i++) {
    if (s_region_stash[i].vpn_ip == vpn_ip) {
      slot = i;
      break;
    }
    if (s_region_stash[i].stamp_ms < s_region_stash[slot].stamp_ms) slot = i;
  }
  s_region_stash[slot].vpn_ip = vpn_ip;
  s_region_stash[slot].region = region;
  s_region_stash[slot].stamp_ms = ml_get_time_ms();
}

static uint16_t region_stash_lookup(uint32_t vpn_ip)
{
  uint64_t now = ml_get_time_ms();
  for (int i = 0; i < ML_REGION_STASH_SLOTS; i++) {
    if (s_region_stash[i].vpn_ip == vpn_ip && now - s_region_stash[i].stamp_ms < ML_REGION_STASH_TTL_MS) {
      return s_region_stash[i].region;
    }
  }
  return 0;
}

/* Single membership test for the peer candidate table — the learn-from-ping
 * check, the alt-candidate dedup and the §7a re-add compare must all agree on
 * what "already holds this endpoint" means (they had drifted on is_ipv6). */
static bool peer_has_endpoint(const ml_peer_t * p, uint32_t ip, uint16_t port, bool is_ipv6)
{
  for (int i = 0; i < p->endpoint_count && i < ML_MAX_ENDPOINTS; i++) {
    if (p->endpoints[i].ip == ip && p->endpoints[i].port == port && p->endpoints[i].is_ipv6 == is_ipv6) {
      return true;
    }
  }
  return false;
}

/* §7a: does this re-add carry any endpoint we don't already hold? Subset test,
 * order-insensitive: learn-from-ping appends locally-learned endpoints that a
 * coord re-add never carries, so exact-list equality was defeated FOREVER for
 * every peer with active disco — the pinned peers took the full re-add path on
 * each re-ingest, and under re-bond churn those X25519 storms stalled the WG
 * and DERP tasks together (run-36 disarm cascade). Only a genuinely NEW
 * endpoint is a material change. */
static bool readd_endpoints_unchanged(const ml_peer_t * p, const ml_peer_update_t * u)
{
  for (int i = 0; i < u->endpoint_count && i < ML_MAX_ENDPOINTS; i++) {
    if (!peer_has_endpoint(p, u->endpoints[i].ip, u->endpoints[i].port, u->endpoints[i].is_ipv6)) {
      return false;
    }
  }
  return true;
}

static int add_peer(microlink_t * ml, const ml_peer_update_t * update, bool * skipped)
{
  /* Peer allowlist filter: don't waste WG slots on non-allowed peers.
     * Still process updates for existing peers (they may become allowed later). */
  if (!ml_config_peer_is_allowed(ml->config_httpd, update->vpn_ip)) {
    return -1; /* Silently skip — peer not in allowlist */
  }

  /* Check if peer already exists */
  bool existing = false;
  int idx = find_peer_by_key(ml, update->public_key);
  if (idx >= 0) {
    existing = true;
    ESP_LOGI(TAG, "Updating existing peer %s (idx=%d)", update->hostname, idx);
  } else {
    /* Effective cap: the runtime-configured max_peers (NVS/admin) when
         * set and below the compile-time ceiling, else ML_MAX_PEERS. Lets an
         * operator trim the active peer set at runtime — fewer peers probed
         * and box-opened = less embedded load — without a rebuild. The
         * priority peer is exempt (its eviction path below still runs to the
         * full ceiling), so trimming can never lock out the safety peer. */
    int eff_max = ML_MAX_PEERS;
    uint8_t cfg_mp = ml_config_get_max_peers(ml->config_httpd);
    if (cfg_mp > 0u && (int)cfg_mp < eff_max) {
      eff_max = (int)cfg_mp;
    }

    /* Key-rotated peer: the update's key is unknown but its VPN IP may
         * already be occupied by a STALE entry (e.g. an NVS-cached peer
         * preseeded at boot whose far end has since re-keyed). Two live
         * entries with one allowed_ip would make wireguard-lwip's
         * routing-by-IP ambiguous and find_peer_by_ip() would keep returning
         * the dead one — so retire the stale entry and reuse its slot. */
    idx = find_peer_by_ip(ml, update->vpn_ip);
    if (idx >= 0) {
      /* Hitless re-ingest: if the "stale" entry is a SAFETY peer whose session
       * is authenticating data RIGHT NOW, the old key is provably still in use
       * — this update is transient/bogus (netmap churn), not a real re-key.
       * Defer: skip the whole update. A genuine re-key stops authenticating
       * within seconds and the retry lands on the next coord update. */
      if (teardown_vetoed(ml, update->vpn_ip, ml->peers[idx].wg_peer_index)) {
        s_diag_rekey_retire_vetoes++;
        ESP_LOGW(
          TAG, "IGNORING re-key retire for %s: session still passing authenticated data (deferred)", update->hostname);
        return -1;
      }
      s_diag_rekey_retires++;
      ESP_LOGW(TAG, "Peer %s re-keyed — retiring stale entry (idx=%d)", update->hostname, idx);
      if (ml->peers[idx].wg_peer_index >= 0 && ml->wg_netif) {
        wireguardif_remove_peer((struct netif *)ml->wg_netif, (u8_t)ml->peers[idx].wg_peer_index);
      }
      /* §7c: retire is the second door a pinned peer's learned region exits
       * through (a fleet tailscaled restart rotates the key; retire is not
       * vetoed for pinned-but-not-safety peers). Stash it — the re-add with
       * the NEW key consumes it in this same call. */
      if (is_pinned_peer(ml, ml->peers[idx].vpn_ip) && ml->peers[idx].derp_region != 0) {
        region_stash_put(ml->peers[idx].vpn_ip, ml->peers[idx].derp_region);
      }
      ml->peers[idx].active = false;
    }

    /* Find free slot */
    idx = -1;
    for (int i = 0; i < eff_max; i++) {
      if (!ml->peers[i].active) {
        idx = i;
        break;
      }
    }

    /* Peer table full, and this peer isn't pinned yet — but the application
         * (machn) may still need it: an incoming peer whose device identity is on
         * the operator allowlist must be kept, or the machine never learns its WG
         * key and the remote can never bond. Pin it NOW (bounded set: operators
         * are ≤ DCS_MAX_OPERATORS) so is_pinned_peer() below is true and the
         * existing LRU-evict-for-pinned path makes room; pinning also persists it
         * for future netmap syncs. Stop-only accept-all remotes are NOT pinned
         * here and remain best-effort. Null cb (e.g. remotes) = current behavior. */
    if (
      idx < 0 && !is_pinned_peer(ml, update->vpn_ip) && ml->peer_wanted_cb != NULL &&
      ml->peer_wanted_cb(ml->peer_wanted_ctx, update->hostname, update->vpn_ip))
    {
      microlink_pin_peer_ip(ml, update->vpn_ip, true);
    }

    /* Peer table full — evict LRU non-pinned peer if the incoming peer is
         * pinned (priority/safety peer OR the management server). */
    if (idx < 0 && is_pinned_peer(ml, update->vpn_ip)) {
      uint64_t oldest_ms = UINT64_MAX;
      int evict_idx = -1;
      for (int i = 0; i < ML_MAX_PEERS; i++) {
        if (!ml->peers[i].active) continue;
        if (is_pinned_peer(ml, ml->peers[i].vpn_ip)) continue;
        /* Never LRU-evict a safety peer, pinned or not — evicting the peer
         * carrying the 5 Hz heartbeat is a guaranteed machine STOP. */
        if (is_safety_peer(ml, ml->peers[i].vpn_ip)) {
          s_diag_evict_safety_skips++;
          continue;
        }
        uint64_t last_activity = ml->peers[i].last_send_ms;
        if (ml->peers[i].last_pong_recv_ms > last_activity) last_activity = ml->peers[i].last_pong_recv_ms;
        if (last_activity < oldest_ms) {
          oldest_ms = last_activity;
          evict_idx = i;
        }
      }
      if (evict_idx >= 0) {
        char evict_ip[16];
        microlink_ip_to_str(ml->peers[evict_idx].vpn_ip, evict_ip);
        ESP_LOGW(
          TAG,
          "Evicting LRU peer %s (%s) for priority peer %s",
          ml->peers[evict_idx].hostname,
          evict_ip,
          update->hostname);
        if (ml->peers[evict_idx].wg_peer_index >= 0 && ml->wg_netif) {
          wireguardif_remove_peer((struct netif *)ml->wg_netif, ml->peers[evict_idx].wg_peer_index);
        }
        ml->peers[evict_idx].active = false;
        idx = evict_idx;
      }
    }

    if (idx < 0) {
      ESP_LOGW(TAG, "Peer table full (%d slots), cannot add %s", ML_MAX_PEERS, update->hostname);
      return -1;
    }
    if (idx >= ml->peer_count) {
      ml->peer_count = idx + 1;
    }
  }

  ml_peer_t * p = &ml->peers[idx];

  /* §7a skip-unchanged re-add: full re-ingests re-ADD every peer each
   * re-register; when nothing material changed, the only consequential
   * effect below is ml_peer_nvs_save() dirtying the peer cache — whose
   * debounced FLASH flush suspends both cores (the run-34 stall clusters).
   * Skip everything except the two load-bearing RAM-only side effects:
   * LRU protection (rebuilt each boot) and the relay-retry re-arm. Any
   * material change — dropped WG slot, IP/disco-key/hostname/region/
   * endpoint delta — takes the full path. Hostname compared in stored-
   * truncation space so an over-long name cannot defeat the skip forever.
   * skipfail = FIRST failing clause, counted per clause (run-38: the
   * pins' adds=4 persisted after the endpoint subset fix — attribute the
   * defeat with data, not hypotheses). */
  int skipfail = -1;
  if (!existing)
    skipfail = 0;
  else if (p->wg_peer_index < 0)
    skipfail = 1;
  else if (update->vpn_ip != p->vpn_ip)
    skipfail = 2;
  else if (memcmp(update->disco_key, p->disco_key, 32) != 0)
    skipfail = 3;
  else if (strncmp(update->hostname, p->hostname, sizeof(p->hostname) - 1) != 0)
    skipfail = 4;
  else if (update->derp_region != 0 && update->derp_region != p->derp_region)
    skipfail = 5;
  else if (!readd_endpoints_unchanged(p, update))
    skipfail = 6;
  if (skipfail >= 0 && skipfail <= 6) s_diag_skipfail[skipfail]++;

  if (skipfail < 0) {
    if (
      is_pinned_peer(ml, p->vpn_ip) ||
      (ml->peer_wanted_cb != NULL && ml->peer_wanted_cb(ml->peer_wanted_ctx, p->hostname, p->vpn_ip)))
    {
      ml_peer_nvs_set_protected(p->vpn_ip); /* protected set is RAM-only */
    }
    if (
      update->endpoint_count > 0 && !p->has_direct_path &&
      (is_pinned_peer(ml, p->vpn_ip) || is_health_tracked(p->vpn_ip)))
    {
      p->relay_retry_next_ms = 1; /* identical endpoints still re-arm the sweep */
    }
    s_diag_readds_skipped++;
    if (skipped) *skipped = true;
    return idx;
  }

  p->vpn_ip = update->vpn_ip;
  memcpy(p->public_key, update->public_key, 32);
  memcpy(p->disco_key, update->disco_key, 32);
  strlcpy(p->hostname, update->hostname, sizeof(p->hostname));
  /* Preserve a previously-learned DERP region. Full peer re-syncs frequently
     * re-add a peer with derp_region=0 (the region is carried separately, via
     * PeersChangedPatch), and unconditionally overwriting wiped the pinned
     * peer's known region back to 0 — so maybe_rehome() below early-returned and
     * the chip never stayed homed on the server's region. Behind a symmetric NAT
     * (USB tether) DERP relay is the only path, so this left the unit
     * permanently unreachable. Only overwrite when the update actually carries a
     * region. */
  if (update->derp_region != 0) {
    p->derp_region = update->derp_region;
  } else if (!existing) {
    /* Fresh (or reused) slot: nothing to preserve — a reused slot's leftover
     * region from a PREVIOUS occupant must not leak in (slot contamination
     * feeds maybe_rehome + the aux want-set). Consult the pinned-peer stash
     * instead: a re-added pin recovers its learned region across the
     * REMOVE→ADD cycle that re-ingests otherwise lose (task #42). */
    p->derp_region = region_stash_lookup(update->vpn_ip);
    if (p->derp_region != 0) s_diag_region_stash_restores++;
  }
  p->active = true;
  maybe_rehome_to_priority(ml, p);

  /* Copy endpoints — but PRESERVE previously-learned ones when the update
     * carries none. Full peer re-syncs (coord reconnect / long-poll re-sync)
     * can deliver a peer entry with an empty Endpoints array, and blindly
     * copying endpoint_count=0 wiped the stored candidates, leaving the
     * peer DERP-only until the next endpoint patch (observed on the bench:
     * a 46h-uptime unit with an empty candidate list for a peer whose
     * netmap endpoints other units held fine). Same rationale as the
     * derp_region preservation above. */
  if (update->endpoint_count > 0) {
    p->endpoint_count = update->endpoint_count;
    for (int i = 0; i < update->endpoint_count && i < ML_MAX_ENDPOINTS; i++) {
      p->endpoints[i].ip = update->endpoints[i].ip;
      p->endpoints[i].port = update->endpoints[i].port;
      p->endpoints[i].is_ipv6 = update->endpoints[i].is_ipv6;
    }
    /* Fresh endpoints arrived for an EXISTING relay-bound safety peer (e.g. the
     * coord re-fetch triggered by the symmetric-NAT relay-stuck recovery): re-arm
     * the relay-retry sweep to fire on the NEXT tick so we ping the NEW endpoint
     * immediately instead of waiting out the exponential backoff — re-hole-punch
     * fast, shrinking the relay window. nonzero-and-in-the-past => fires next tick. */
    if (existing && !p->has_direct_path && (is_pinned_peer(ml, p->vpn_ip) || is_health_tracked(p->vpn_ip))) {
      p->relay_retry_next_ms = 1;
    }
  } else if (!existing) {
    p->endpoint_count = 0; /* fresh/reused slot: never inherit a previous
                            * occupant's endpoints (slot contamination) */
  }

  /* Initialize DISCO/direct-path state — ONLY for a genuinely new peer.
     * A full "Peers" list is re-ingested (as ML_PEER_ADD per peer) on every
     * coord reconnect / long-poll re-sync / rebind. Resetting an EXISTING
     * peer here dropped its established direct path (has_direct_path=false,
     * best_ip=0, wg_peer_index=-1) and, for the priority peer, forced a
     * connect_derp + full re-discovery on every such event — a needless
     * safety-link gap. Preserve the live path for existing peers; the
     * endpoint delta-patch path (ML_PEER_UPDATE_ENDPOINT) handles real
     * endpoint changes. */
  if (!existing) {
    p->last_ping_sent_ms = 0;
    p->last_pong_recv_ms = 0;
    p->trust_until_ms = 0;
    p->last_send_ms = 0;
    p->last_upgrade_ms = 0;
    p->has_direct_path = false;
    p->best_ip = 0;
    p->best_port = 0;
    p->wg_peer_index = -1;
    /* Direct-path flap-backoff + DERP throttle state (net/derp-stability): reset
     * on slot REUSE too, else an evicted peer's stale direct_backoff_until would
     * spuriously suppress this fresh peer's direct-upgrade probing for up to 60 s. */
    p->direct_promoted_ms = 0;
    p->direct_flap_count = 0;
    p->direct_backoff_until = 0;
    p->last_derp_reconnect_ms = 0;
    p->relay_retry_next_ms = 0;
    p->relay_retry_count = 0;
    /* CMM chain-breaker throttle: an evicted slot's stale stamp would eat the
     * fresh peer's first CallMeMaybe for up to ML_DISCO_CMM_MIN_INTERVAL_MS. */
    p->last_cmm_sent_ms = 0;
  }

  char ip_str[16];
  microlink_ip_to_str(update->vpn_ip, ip_str);
  ESP_LOGI(
    TAG,
    "Peer added: %s (%s) idx=%d endpoints=%d derp=%d key=%02x%02x%02x%02x%02x%02x%02x%02x",
    p->hostname,
    ip_str,
    idx,
    p->endpoint_count,
    p->derp_region,
    p->public_key[0],
    p->public_key[1],
    p->public_key[2],
    p->public_key[3],
    p->public_key[4],
    p->public_key[5],
    p->public_key[6],
    p->public_key[7]);

  /* Add to wireguard-lwip — only for a new peer, or an existing one whose
     * previous WG add failed (wg_peer_index < 0). An existing peer with a
     * live WG slot is left in place so its session/endpoint survives this
     * re-ingest (P7 — see the DISCO-state note above). */
  if (!existing || p->wg_peer_index < 0) {
    wg_install_iface_peer(ml, idx);
  }

  /* Persist to NVS for fast boot next time. Bond-critical peers — the
     * priority/fleet pins AND any peer the app wants kept (machn's operator
     * allowlist via peer_wanted_cb; all bounded sets) — are marked protected
     * so the peer cache's LRU eviction can never drop the very keys the
     * boot-time preseed needs (cold-bond far-side gap, 2026-08-08). Note the
     * pin path above only fires when the RAM table is FULL; the cb is asked
     * again here so operator keys are protected on small tailnets too. */
  if (
    is_pinned_peer(ml, p->vpn_ip) ||
    (ml->peer_wanted_cb != NULL && ml->peer_wanted_cb(ml->peer_wanted_ctx, p->hostname, p->vpn_ip)))
  {
    ml_peer_nvs_set_protected(p->vpn_ip);
  }
  ml_peer_nvs_save(p);

  /* Send CallMeMaybe to trigger peer-initiated handshake (NAT traversal). */
  disco_send_call_me_maybe(ml, idx);

  /* Immediately probe known endpoints (throttled to avoid flooding with 100s of peers).
     * Only force-ping if fewer than 5 peers added in the last second;
     * the periodic probe (every 15s) will handle the rest. */
  {
    static uint64_t last_burst_ms = 0;
    static int burst_count = 0;
    uint64_t add_now = ml_get_time_ms();
    if (add_now - last_burst_ms > 1000) {
      burst_count = 0;
      last_burst_ms = add_now;
    }
    if (burst_count < 5) {
      disco_send_ping_to_peer(ml, idx, true);
      burst_count++;
    }
  }

  /* Notify app via callback */
  if (ml->peer_cb) {
    microlink_peer_info_t info = {
      .vpn_ip = p->vpn_ip,
      .online = true,
      .direct_path = false,
    };
    strlcpy(info.hostname, p->hostname, sizeof(info.hostname));
    memcpy(info.public_key, p->public_key, 32);
    ml->peer_cb(ml, &info, ml->peer_cb_data);
  }

  return idx;
}

static void remove_peer(microlink_t * ml, const ml_peer_update_t * update)
{
  int idx = find_peer_by_key(ml, update->public_key);
  if (idx < 0) return;

  /* Hitless re-ingest: a coord REMOVE for a SAFETY peer whose session is
   * authenticating data RIGHT NOW is presumptively a server glitch (netmap
   * trim/re-sync churn) — applying it fails the 5 Hz heartbeat ENOTCONN and
   * stops the machine. Defer: keep the live session; a genuinely-removed
   * peer's session goes stale within seconds and a later REMOVE (or the
   * peer's own disappearance) applies. Fail toward keeping a working safety
   * bond, never toward a tidy peer table. */
  if (teardown_vetoed(ml, ml->peers[idx].vpn_ip, ml->peers[idx].wg_peer_index)) {
    s_diag_remove_vetoes++;
    ESP_LOGW(
      TAG,
      "IGNORING coord REMOVE for %s: session still passing authenticated data (deferred)",
      ml->peers[idx].hostname);
    return;
  }
  s_diag_peer_removes++;

  /* Remove from wireguard-lwip */
  if (ml->wg_netif && ml->peers[idx].wg_peer_index >= 0) {
    struct netif * netif = (struct netif *)ml->wg_netif;
    wireguardif_remove_peer(netif, (u8_t)ml->peers[idx].wg_peer_index);
  }

  char ip_str[16];
  microlink_ip_to_str(ml->peers[idx].vpn_ip, ip_str);
  ESP_LOGI(TAG, "Peer removed: %s (%s)", ml->peers[idx].hostname, ip_str);

  /* §7c: a pinned peer's learned region survives the REMOVE→re-ADD cycle
   * full re-ingests produce (the re-ADD often carries region 0). */
  if (is_pinned_peer(ml, ml->peers[idx].vpn_ip) && ml->peers[idx].derp_region != 0) {
    region_stash_put(ml->peers[idx].vpn_ip, ml->peers[idx].derp_region);
  }

  ml->peers[idx].active = false;

  /* Compact peer_count */
  while (ml->peer_count > 0 && !ml->peers[ml->peer_count - 1].active) {
    ml->peer_count--;
  }
}

static void process_peer_updates(microlink_t * ml)
{
  ml_peer_update_t * update;
  int adds_this_pass = 0;
  bool paced_out = false;
  while (!paced_out && (xQueueReceive(ml->peer_update_queue, &update, 0) == pdTRUE)) {
    if (!update) continue;
    switch (update->action) {
      case ML_PEER_ADD: {
        bool skipped = false;
        add_peer(ml, update, &skipped);
        /* Each REAL add costs an X25519 (~30 ms) — pace those, and interleave
             * a wg-rx drain after each so a heartbeat is never starved behind a
             * netmap-resync burst. A skipped unchanged re-add is ~µs: no pacing
             * slot and no drain — ML_WG_HS_REQUEUE_MAX sizing assumes at most
             * ML_PEER_ADDS_PER_PASS drains per pass, and a long all-unchanged
             * re-ingest must not pop a budget-deferred handshake to its requeue
             * cap within a single pass. */
        if (!skipped) {
          wg_mgr_drain_wg_rx(ml);
          adds_this_pass++;
          s_pass_peer_adds++;
          if (adds_this_pass >= ML_PEER_ADDS_PER_PASS) {
            paced_out = true;
          }
        }
        break;
      }
      case ML_PEER_REMOVE:
        remove_peer(ml, update);
        break;
      case ML_PEER_UPDATE_ENDPOINT:
        /* Update endpoint/DERP for existing peer (delta patch) */
        {
          int idx = find_peer_by_key(ml, update->public_key);
          if (idx >= 0) {
            ml_peer_t * p = &ml->peers[idx];
            if (update->endpoint_count > 0) {
              p->endpoint_count = update->endpoint_count;
              for (int i = 0; i < update->endpoint_count && i < ML_MAX_ENDPOINTS; i++) {
                p->endpoints[i].ip = update->endpoints[i].ip;
                p->endpoints[i].port = update->endpoints[i].port;
                p->endpoints[i].is_ipv6 = update->endpoints[i].is_ipv6;
              }
            }
            if (update->derp_region > 0) {
              p->derp_region = update->derp_region;
              maybe_rehome_to_priority(ml, p);
            }
            /* Patches are genuine deltas: persist them. With §7a, unchanged
             * full re-adds no longer save, so this is now the ONLY route by
             * which patch-learned endpoints/regions reach the boot preseed. */
            if (update->endpoint_count > 0 || update->derp_region > 0) {
              ml_peer_nvs_save(p);
            }
            ESP_LOGI(TAG, "Peer patched: %s (eps=%d derp=%d)", p->hostname, p->endpoint_count, p->derp_region);
          }
        }
        break;
    }
    free(update);
  }
}

/* ============================================================================
 * DISCO Protocol
 * ========================================================================== */

static void disco_build_ping(microlink_t * ml, int peer_idx, uint8_t * out, size_t * out_len)
{
  ml_peer_t * p = &ml->peers[peer_idx];

  /* Plaintext: [type(1)][version(1)][txid(12)][nodekey(32)] = 46 bytes */
  uint8_t plaintext[46];
  plaintext[0] = DISCO_MSG_PING;
  plaintext[1] = 0; /* version */

  /* Generate random transaction ID */
  uint8_t txid[DISCO_TXID_LEN];
  esp_fill_random(txid, DISCO_TXID_LEN);
  memcpy(plaintext + 2, txid, DISCO_TXID_LEN);
  memcpy(plaintext + 14, ml->wg_public_key, 32);

  /* Generate random nonce */
  uint8_t nonce[DISCO_NONCE_LEN];
  esp_fill_random(nonce, DISCO_NONCE_LEN);

  /* Encrypt with NaCl box: our disco private key -> peer's disco public key */
  uint8_t ciphertext[46 + NACL_BOX_MACBYTES];
  nacl_box(ciphertext, plaintext, sizeof(plaintext), nonce, p->disco_key, ml->disco_private_key);

  /* Build packet: magic(6) + our_disco_pubkey(32) + nonce(24) + ciphertext(62) = 124 bytes */
  size_t pos = 0;
  memcpy(out + pos, DISCO_MAGIC, 6);
  pos += 6;
  memcpy(out + pos, ml->disco_public_key, 32);
  pos += 32;
  memcpy(out + pos, nonce, DISCO_NONCE_LEN);
  pos += DISCO_NONCE_LEN;
  memcpy(out + pos, ciphertext, sizeof(ciphertext));
  pos += sizeof(ciphertext);
  *out_len = pos;

  /* Track pending probe */
  bool registered = false;
  for (int i = 0; i < MAX_PENDING_PROBES; i++) {
    if (!pending_probes[i].active) {
      memcpy(pending_probes[i].txid, txid, DISCO_TXID_LEN);
      pending_probes[i].peer_index = peer_idx;
      pending_probes[i].sent_ms = ml_get_time_ms();
      pending_probes[i].derp_match_ms = 0;
      pending_probes[i].active = true;
      registered = true;
      /* Track the table's occupancy high water (probe_tbl_hw). Registration is
       * the only place occupancy grows, so sampling here catches every peak; a
       * drift-free full scan (64 flag reads) beats a live counter that would
       * need decrements at every retire/expire site. Slots 0..i-1 were all
       * occupied (first-free allocation), slots above i are scanned. */
      {
        uint32_t occupied = (uint32_t)i + 1u;
        for (int k = i + 1; k < MAX_PENDING_PROBES; k++) {
          if (pending_probes[k].active) occupied++;
        }
        if (occupied > s_diag_probe_tbl_hw) s_diag_probe_tbl_hw = occupied;
      }
      ESP_LOGI(
        TAG,
        "Probe registered slot=%d peer=%s txid=%02x%02x%02x%02x",
        i,
        p->hostname,
        txid[0],
        txid[1],
        txid[2],
        txid[3]);
      break;
    }
  }
  if (!registered) {
    /* No free slot = occupancy is the table size — record the saturation
     * (the exhaustion case the 2026-08-09 oscillation hinged on). */
    s_diag_probe_tbl_hw = MAX_PENDING_PROBES;
    ESP_LOGW(TAG, "DISCO probe table full (%d slots), pong will be unmatched", MAX_PENDING_PROBES);
  }
}

static void disco_build_pong(
  microlink_t * ml,
  int peer_idx,
  const uint8_t * txid,
  uint32_t src_ip,
  uint16_t src_port,
  uint8_t * out,
  size_t * out_len)
{
  ml_peer_t * p = &ml->peers[peer_idx];

  /* Plaintext: [type(1)][version(1)][txid(12)][src_addr(18)] = 32 bytes */
  /* src_addr: IPv6-mapped IPv4 (16 bytes) + port (2 bytes big-endian) */
  uint8_t plaintext[32];
  plaintext[0] = DISCO_MSG_PONG;
  plaintext[1] = 0;
  memcpy(plaintext + 2, txid, DISCO_TXID_LEN);

  /* IPv6-mapped IPv4: ::ffff:A.B.C.D */
  memset(plaintext + 14, 0, 10);
  plaintext[24] = 0xff;
  plaintext[25] = 0xff;
  plaintext[26] = (src_ip >> 24) & 0xFF;
  plaintext[27] = (src_ip >> 16) & 0xFF;
  plaintext[28] = (src_ip >> 8) & 0xFF;
  plaintext[29] = src_ip & 0xFF;
  plaintext[30] = (src_port >> 8) & 0xFF;
  plaintext[31] = src_port & 0xFF;

  /* Generate random nonce */
  uint8_t nonce[DISCO_NONCE_LEN];
  esp_fill_random(nonce, DISCO_NONCE_LEN);

  /* Encrypt */
  uint8_t ciphertext[32 + NACL_BOX_MACBYTES];
  nacl_box(ciphertext, plaintext, sizeof(plaintext), nonce, p->disco_key, ml->disco_private_key);

  /* Build packet */
  size_t pos = 0;
  memcpy(out + pos, DISCO_MAGIC, 6);
  pos += 6;
  memcpy(out + pos, ml->disco_public_key, 32);
  pos += 32;
  memcpy(out + pos, nonce, DISCO_NONCE_LEN);
  pos += DISCO_NONCE_LEN;
  memcpy(out + pos, ciphertext, sizeof(ciphertext));
  pos += sizeof(ciphertext);
  *out_len = pos;
}

static void disco_send_ping_to_peer(microlink_t * ml, int peer_idx, bool force)
{
  ml_peer_t * p = &ml->peers[peer_idx];
  uint64_t now = ml_get_time_ms();

  /* Rate limit: don't ping more often than DISCO_PING_INTERVAL_MS (skip if forced) */
  if (!force && now - p->last_ping_sent_ms < ML_DISCO_PING_INTERVAL_MS) {
    return;
  }

  /* Probe dedup: multiple forced call-sites (disco_wake_peer, upgrade, demotion,
   * heartbeat) each register a fresh txid within the 5 s probe lifetime, piling
   * up outstanding probes. If this peer already has an ACTIVE probe younger than
   * 2 s, a fresh one is pure redundancy — skip it. The 2 s floor is well under
   * the 3 s liveness/heartbeat cadence (which also spaces last_ping_sent_ms by
   * >= 3 s), so a genuine liveness ping is never suppressed. */
  for (int pi = 0; pi < MAX_PENDING_PROBES; pi++) {
    if (
      pending_probes[pi].active && pending_probes[pi].peer_index == peer_idx && now - pending_probes[pi].sent_ms < 2000)
    {
      return;
    }
  }

  uint8_t pkt[256];
  size_t pkt_len = 0;
  disco_build_ping(ml, peer_idx, pkt, &pkt_len);

  if (pkt_len == 0) return;

  bool direct_sent = false;
  bool best_sent = false;

  /* If we have a known working direct path, send there FIRST.
     * This is critical for heartbeat pings to renew trust_until_ms.
     * A DERP pong would arrive with via_derp=true and NOT renew trust.
     *
     * Deliberately NOT gated on has_direct_path: after a demotion best_ip
     * still holds the last endpoint that PROVED bidirectionally reachable,
     * and it may no longer appear in endpoints[] (netmap patches replace the
     * candidate list). Keep probing it — a pong from it is exactly how the
     * direct path re-establishes. One bounded extra UDP send per ping. */
  if (p->best_ip != 0 && p->best_port != 0) {
    disco_udp_sendto(ml, pkt, pkt_len, p->best_ip, p->best_port);
    direct_sent = p->has_direct_path; /* demoted: still fall through to DERP */
    best_sent = true;
  }

  /* Also try direct UDP to all known endpoints from MapResponse */
  {
    bool has_udp = disco_has_udp_path(ml);
    if (has_udp) {
      for (int i = 0; i < p->endpoint_count; i++) {
        if (!p->endpoints[i].is_ipv6 && p->endpoints[i].ip != 0) {
          /* Skip if same as best_ip — but ONLY when the best-path branch above
           * actually sent there. After a demotion has_direct_path is false, so
           * that branch is skipped, yet best_ip/best_port still hold the
           * previously-proven endpoint. The unconditional skip here excluded
           * exactly that endpoint — for a NAT'd remote it is typically the ONLY
           * reachable candidate (the others need hairpin), so every re-probe
           * went DERP-only and the peer stayed relay-bound until reboot (which
           * clears best_ip). Root cause of the 2026-08 "stuck on DERP" bug. */
          if (best_sent && p->endpoints[i].ip == p->best_ip && p->endpoints[i].port == p->best_port) continue;
          int ret = disco_udp_sendto(ml, pkt, pkt_len, p->endpoints[i].ip, p->endpoints[i].port);
          if (!direct_sent) { /* Log only first direct send per peer */
            ESP_LOGI(
              TAG,
              "  direct probe -> %d.%d.%d.%d:%d (%d eps, ret=%d)",
              (int)((p->endpoints[i].ip >> 24) & 0xFF),
              (int)((p->endpoints[i].ip >> 16) & 0xFF),
              (int)((p->endpoints[i].ip >> 8) & 0xFF),
              (int)(p->endpoints[i].ip & 0xFF),
              (int)p->endpoints[i].port,
              p->endpoint_count,
              ret);
          }
          direct_sent = true;
        }
      }
    }
    if (!has_udp) {
      ESP_LOGW(TAG, "  no UDP path for %s (sock4=%d)", p->hostname, ml->disco_sock4);
    } else if (!direct_sent && p->endpoint_count > 0) {
      ESP_LOGW(TAG, "  %s: %d eps but none usable (all IPv6?)", p->hostname, p->endpoint_count);
    }
  }

  /* Send via DERP as fallback (or always for initial probes).
     * Skip DERP for heartbeat pings when direct path is active to avoid
     * DERP pong stealing the probe match from the direct pong. */
  if (!p->has_direct_path || !direct_sent) {
    ml_derp_queue_send(ml, p->public_key, pkt, pkt_len);
    ESP_LOGI(TAG, "DISCO PING -> %s via DERP", p->hostname);
  } else {
    ESP_LOGI(
      TAG,
      "DISCO PING -> %s via direct %d.%d.%d.%d:%d",
      p->hostname,
      (int)((p->best_ip >> 24) & 0xFF),
      (int)((p->best_ip >> 16) & 0xFF),
      (int)((p->best_ip >> 8) & 0xFF),
      (int)(p->best_ip & 0xFF),
      (int)p->best_port);
  }

  p->last_ping_sent_ms = now;
}

static void process_disco_ping(
  microlink_t * ml,
  const ml_rx_packet_t * pkt,
  const uint8_t * sender_disco_key,
  const uint8_t * decrypted,
  size_t decrypted_len)
{
  if (decrypted_len < 14) return;

  /* Extract txid from decrypted payload */
  const uint8_t * txid = decrypted + 2;

  /* Find peer by disco key */
  int peer_idx = find_peer_by_disco_key(ml, sender_disco_key);
  if (peer_idx < 0 && decrypted_len >= 46) {
    /* Disco keys rotate on every tailscaled restart, so a stale stored
         * key (from a truncated MapResponse or hardcoded peer config) makes
         * a legitimate peer look unknown. Modern disco pings embed the
         * sender's NODE key at payload[14..46] — stable across restarts.
         * Rescue the mapping and refresh the stored disco key so the pong
         * (boxed to the peer's stored key) and future probes use the
         * current one. This is what finally lets a peer with a rotated
         * disco key traverse to us (2026-07-02 bench: laptop pings arrived
         * and decrypted fine but were dropped right here). */
    const uint8_t * sender_node_key = decrypted + 14;
    for (int i = 0; i < ml->peer_count; i++) {
      if (memcmp(ml->peers[i].public_key, sender_node_key, 32) == 0) {
        peer_idx = i;
        memcpy(ml->peers[i].disco_key, sender_disco_key, 32);
        ESP_LOGW(
          TAG,
          "DISCO key refreshed for %s from inbound ping "
          "(rotated, now %02x%02x%02x%02x...)",
          ml->peers[i].hostname,
          sender_disco_key[0],
          sender_disco_key[1],
          sender_disco_key[2],
          sender_disco_key[3]);
        ml_peer_nvs_save(&ml->peers[i]);
        break;
      }
    }
  }
  if (peer_idx < 0) {
    ESP_LOGW(TAG, "DISCO ping from unknown peer");
    return;
  }

  /* Allowlist enforcement for senders admitted past the pre-decrypt gate
     * on an unknown (rotated) disco key: now that the sender is identified
     * (directly or via node-key rescue), apply the filter before ponging. */
  if (!ml_config_peer_is_allowed(ml->config_httpd, ml->peers[peer_idx].vpn_ip)) {
    ESP_LOGI(TAG, "DISCO ping from %s dropped (not in allowlist)", ml->peers[peer_idx].hostname);
    return;
  }

  ml_peer_t * p = &ml->peers[peer_idx];

  ESP_LOGI(TAG, "DISCO PING from %s (via %s)", p->hostname, pkt->via_derp ? "DERP" : "direct");

  /* Build PONG */
  uint8_t pong[256];
  size_t pong_len = 0;
  disco_build_pong(ml, peer_idx, txid, pkt->src_ip, pkt->src_port, pong, &pong_len);

  if (pong_len == 0) return;

  /* Send PONG via ALL paths for maximum reachability (matching v1 + tailscaled):
     * 1. Direct reply to PING source address (opens NAT hole)
     * 2. All known LAN endpoints (fastest path for same-network)
     * 3. DERP relay (guaranteed delivery) */

  bool direct_sent = false;

  /* 1. Direct reply to PING source (if it was direct UDP) */
  if (!pkt->via_derp && pkt->src_ip != 0 && pkt->src_port != 0) {
    disco_udp_sendto(ml, pong, pong_len, pkt->src_ip, pkt->src_port);
    direct_sent = true;
  }

  /* 2. Send to ALL known LAN endpoints (same-network = fastest path) */
  if (disco_has_udp_path(ml)) {
    for (int i = 0; i < p->endpoint_count; i++) {
      if (p->endpoints[i].is_ipv6 || p->endpoints[i].ip == 0) continue;
      if (!is_lan_ip(p->endpoints[i].ip)) continue;
      /* Skip if this is the same as the ping source (already sent) */
      if (p->endpoints[i].ip == pkt->src_ip && p->endpoints[i].port == pkt->src_port) continue;

      disco_udp_sendto(ml, pong, pong_len, p->endpoints[i].ip, p->endpoints[i].port);
      direct_sent = true;
    }

    /* 2b. Also try public endpoints if no LAN worked */
    if (!direct_sent) {
      for (int i = 0; i < p->endpoint_count; i++) {
        if (p->endpoints[i].is_ipv6 || p->endpoints[i].ip == 0) continue;
        if (is_lan_ip(p->endpoints[i].ip)) continue;

        disco_udp_sendto(ml, pong, pong_len, p->endpoints[i].ip, p->endpoints[i].port);
        direct_sent = true;
        break; /* Only try one public endpoint */
      }
    }
  }

  /* 3. ALWAYS send via DERP (guaranteed delivery, even if direct worked) */
  ml_derp_queue_send(ml, p->public_key, pong, pong_len);

  ESP_LOGI(TAG, "PONG sent to %s (direct=%s, DERP=yes)", p->hostname, direct_sent ? "yes" : "no");

  /* 4. Learn from an inbound DIRECT ping: the source 5-tuple provably
     * reaches us peer->chip, so adopt it as a candidate and immediately
     * race our own ping at it to validate the chip->peer direction (a pong
     * flips has_direct_path). Without this, a chip whose stored candidate
     * list is empty (netmap endpoints missed or wiped) stays DERP-only
     * forever even while the peer pings it directly every few seconds. */
  if (!pkt->via_derp && pkt->src_ip != 0 && pkt->src_port != 0 && !p->has_direct_path) {
    bool known = peer_has_endpoint(p, pkt->src_ip, pkt->src_port, false);
    if (!known) {
      if (p->endpoint_count < ML_MAX_ENDPOINTS) {
        p->endpoints[p->endpoint_count].ip = pkt->src_ip;
        p->endpoints[p->endpoint_count].port = pkt->src_port;
        p->endpoints[p->endpoint_count].is_ipv6 = false;
        p->endpoint_count++;
        ESP_LOGI(TAG, "Learned candidate endpoint for %s from inbound direct ping", p->hostname);
      } else {
        /* Ring-evict (run-20/21 finding B-1): the append-only learn silently
         * dropped the candidate when the 8-slot table was full — and a
         * symmetric NAT mints a new dead candidate per rebind, so the table
         * WILL fill, after which the ONE live mapping a rebooted peer
         * presents (the source of this very ping) could never be stored.
         * Terminal, reboot-surviving wedge on the machine side (only a
         * machine reboot cleared it: NVS persists 2 endpoints). Overwrite
         * the last two slots round-robin — coord-delivered endpoints live at
         * the front and are replaced wholesale by coord updates anyway. */
        int slot = ML_MAX_ENDPOINTS - 2 + (p->learn_evict_next & 1u);
        p->learn_evict_next++;
        p->endpoints[slot].ip = pkt->src_ip;
        p->endpoints[slot].port = pkt->src_port;
        p->endpoints[slot].is_ipv6 = false;
        s_diag_ep_learn_evictions++;
        ESP_LOGW(
          TAG,
          "Endpoint table full for %s: ring-evicted slot %d for learned candidate (evictions=%u)",
          p->hostname,
          slot,
          (unsigned)s_diag_ep_learn_evictions);
      }
    }
    disco_send_ping_to_peer(ml, peer_idx, true);
  }
}

static void process_disco_pong(
  microlink_t * ml,
  const ml_rx_packet_t * pkt,
  const uint8_t * sender_disco_key,
  const uint8_t * decrypted,
  size_t decrypted_len)
{
  if (decrypted_len < 14) return;

  const uint8_t * txid = decrypted + 2;
  uint64_t now = ml_get_time_ms();

  /* Match transaction ID */
  bool matched = false;
  bool consumed = false;
  for (int i = 0; i < MAX_PENDING_PROBES; i++) {
    if (!pending_probes[i].active) continue;
    if (memcmp(pending_probes[i].txid, txid, DISCO_TXID_LEN) != 0) continue;

    int peer_idx = pending_probes[i].peer_index;
    if (peer_idx < 0 || peer_idx >= ml->peer_count) {
      pending_probes[i].active = false;
      continue;
    }

    ml_peer_t * p = &ml->peers[peer_idx];
    uint64_t rtt_ms = now - pending_probes[i].sent_ms;

    ESP_LOGI(
      TAG,
      "DISCO PONG from %s: RTT=%llu ms (via %s)",
      p->hostname,
      (unsigned long long)rtt_ms,
      pkt->via_derp ? "DERP" : "direct");

    p->last_pong_recv_ms = now;
    p->disco_rtt_ms = (uint32_t)rtt_ms;
    p->disco_rtt_direct = !pkt->via_derp;

    /* If direct reply, update best path */
    if (!pkt->via_derp && pkt->src_ip != 0) {
      /* FIX A — sticky safety path. The priority/health-tracked (pstop) peer
       * keeps its proven direct endpoint across a netmap rebond: a pong from a
       * DIFFERENT non-upgrade endpoint must NOT flip best_ip and force a WG
       * re-handshake (that momentary DERP fallback is what dropped the safety
       * heartbeat). Mirrors tailscale magicsock trustBestAddrUntil+betterAddr. */
      bool had_direct = p->has_direct_path;
      bool is_prio = is_safety_peer(ml, p->vpn_ip);

      /* Any direct pong proves the PEER is reachable — renew general liveness.
       * But the pong-dead demote stamp requires a pong from the ESTABLISHED
       * endpoint: one txid fans out to every candidate, and a stray pong from
       * an alternate endpoint must not vouch for the path WG actually sends
       * on (best_ip could be dead while a candidate answers). */
      p->has_direct_path = true;
      bool same_ep = (pkt->src_ip == p->best_ip && pkt->src_port == p->best_port);
      if (p->best_ip == 0 || same_ep) {
        p->last_direct_pong_recv_ms = now;
      }
      p->trust_until_ms = now + ML_DISCO_TRUST_DURATION_MS;

      /* Flap-damping bookkeeping (FIX 2). Timestamp only the genuine false->true
       * promotion edge (NOT every heartbeat pong, else the path never looks
       * "established"). Once a path survives past 15 s, clear the flap counter so
       * a long-stable peer that later legitimately fails isn't penalized. */
      if (!had_direct) {
        p->direct_promoted_ms = now;
        /* Direct regained — disarm the relay-bound retry and reset its backoff
         * so the NEXT outage starts the CMM/sweep cycle from 30 s again. */
        p->relay_retry_next_ms = 0;
        p->relay_retry_count = 0;
        s_diag_direct_regains++;
        if (is_prio) {
          /* SAFETY-peer regains only (priority peer or health-tracked pstop
           * counterpart). The global counter above includes bulk tailnet
           * peers, so an oscillation on the safety path was indistinguishable
           * from ordinary churn elsewhere. */
          s_diag_regains_safety++;
        }
      } else if (now - p->direct_promoted_ms > 15000) {
        p->direct_flap_count = 0;
      }

      /* same_ep computed above (best_ip/best_port unchanged since). A move
       * from a WAN/STUN address to a LAN address is a genuine upgrade
       * (lower latency, no hairpin) and IS allowed to switch even for pinned peers. */
      bool genuine_upgrade = is_lan_ip(pkt->src_ip) && !is_lan_ip(p->best_ip);

      if (is_prio && had_direct && p->best_ip != 0 && !same_ep && !genuine_upgrade) {
        /* Keep the established direct path. Remember the alternate endpoint as
         * a candidate (dedup; bounded by ML_MAX_ENDPOINTS) so a later probe can
         * still find it if the current path actually fails. */
        bool present = peer_has_endpoint(p, pkt->src_ip, pkt->src_port, false);
        if (!present && p->endpoint_count < ML_MAX_ENDPOINTS) {
          p->endpoints[p->endpoint_count].ip = pkt->src_ip;
          p->endpoints[p->endpoint_count].port = pkt->src_port;
          p->endpoints[p->endpoint_count].is_ipv6 = false;
          p->endpoint_count++;
        }
        ESP_LOGI(
          TAG,
          "Keeping established direct path for %s (alt %d.%d.%d.%d:%d recorded as candidate)",
          p->hostname,
          (int)((pkt->src_ip >> 24) & 0xFF),
          (int)((pkt->src_ip >> 16) & 0xFF),
          (int)((pkt->src_ip >> 8) & 0xFF),
          (int)(pkt->src_ip & 0xFF),
          (int)pkt->src_port);
        /* Consume this probe exactly like the normal matched-pong exit below
         * (direct pong -> consuming is correct; see the via_derp note there). */
        pending_probes[i].active = false;
        matched = true;
        consumed = true;
        break;
      }

      p->best_ip = pkt->src_ip;
      p->best_port = pkt->src_port;
      /* The pong that CAUSES an adoption is judged against the pre-update
       * best_ip above and misses the liveness stamp — stamp here too, or a
       * freshly promoted path is pong_dead'd against a stale timestamp on
       * the next probe tick (spurious re-demote of a just-proven path). */
      p->last_direct_pong_recv_ms = now;

      /* Update WireGuard endpoint to direct path.
             * Always update the stored endpoint. Only force a handshake if we
             * already have an active WG session (peer has us in their config).
             * For idle peers, the next incoming initiation will use this endpoint. */
      if (ml->wg_netif && p->wg_peer_index >= 0) {
        struct netif * netif = (struct netif *)ml->wg_netif;
        ip_addr_t ep_ip;
        IP_SET_TYPE_VAL(ep_ip, IPADDR_TYPE_V4);
        ip4_addr_set_u32(ip_2_ip4(&ep_ip), htonl(pkt->src_ip));
        wireguardif_update_endpoint(netif, (u8_t)p->wg_peer_index, &ep_ip, pkt->src_port);

        /* Only call connect (forces handshake) if:
                 * 1. Peer has an active WG session, AND
                 * 2. The endpoint actually changed (avoid re-handshake on every heartbeat PONG) */
        ip_addr_t cur_ip;
        u16_t cur_port;
        err_t is_up = wireguardif_peer_is_up(netif, (u8_t)p->wg_peer_index, &cur_ip, &cur_port);
        if (is_up == ERR_OK) {
          /* Check if endpoint actually changed */
          uint32_t cur_ip_u32 = ip4_addr_get_u32(ip_2_ip4(&cur_ip));
          uint32_t new_ip_u32 = htonl(pkt->src_ip);
          if (cur_ip_u32 != new_ip_u32 || cur_port != pkt->src_port) {
            wireguardif_connect(netif, (u8_t)p->wg_peer_index);
            ESP_LOGI(
              TAG,
              "WG endpoint SWITCHED to direct: %d.%d.%d.%d:%d for %s",
              (int)((pkt->src_ip >> 24) & 0xFF),
              (int)((pkt->src_ip >> 16) & 0xFF),
              (int)((pkt->src_ip >> 8) & 0xFF),
              (int)(pkt->src_ip & 0xFF),
              (int)pkt->src_port,
              p->hostname);
          }
        } else {
          ESP_LOGI(
            TAG,
            "WG endpoint stored (no session): %d.%d.%d.%d:%d for %s",
            (int)((pkt->src_ip >> 24) & 0xFF),
            (int)((pkt->src_ip >> 16) & 0xFF),
            (int)((pkt->src_ip >> 8) & 0xFF),
            (int)(pkt->src_ip & 0xFF),
            (int)pkt->src_port,
            p->hostname);
          /* First direct path discovery — send a one-shot handshake
                     * via direct UDP. Do NOT use wireguardif_connect() which
                     * sets peer->active=true and causes infinite handshake
                     * retries (every 5s) when the peer has us trimmed.
                     * Instead, just fire a single handshake init. If the peer
                     * has us configured, it will respond and establish session.
                     * If not, we stop and wait for them to initiate. */
          if (!p->tried_initial_handshake) {
            p->tried_initial_handshake = true;
            /* Store endpoint so wireguardif_connect sends to it */
            wireguardif_update_endpoint(netif, (u8_t)p->wg_peer_index, &ep_ip, pkt->src_port);
            /* Fire one handshake init but don't leave peer active.
                         * wireguardif_connect sets active=true internally, so
                         * we immediately clear it after to prevent retries. */
            wireguardif_connect(netif, (u8_t)p->wg_peer_index);
            /* Clear active to prevent infinite retry loop.
                         * If handshake succeeds, the response handler will
                         * establish the session regardless of active flag. */
            {
              struct wireguard_device * dev = (struct wireguard_device *)netif->state;
              if (dev && p->wg_peer_index < WIREGUARD_MAX_PEERS) {
                dev->peers[p->wg_peer_index].active = false;
              }
            }
            ESP_LOGI(TAG, "WG one-shot handshake to %s (first direct path)", p->hostname);
          }
        }
      }
    } else if (pkt->via_derp) {
      /* AGGRAVATOR FIX (a): a via-DERP pong proves the RELAY path to this peer is
       * alive. Liveness (last_pong_recv_ms) is already renewed above for every
       * pong. For a SAFETY peer (pinned/priority/health-tracked) with no WG
       * session up, (re)fire a DERP handshake init so the working relay actually
       * drives session establishment — a bond that was stuck errno-128 because
       * our relayed init never reached a cross-region peer now completes once an
       * aux DERP conn homes on that region. We do NOT install a direct endpoint
       * from a DERP pong (would re-break 827386f), and we do NOTHING for
       * non-pinned peers (peer-scaling armor: no retry storm for the bulk peers). */
      bool is_safety = is_pinned_peer(ml, p->vpn_ip) || is_health_tracked(p->vpn_ip);
      if (is_safety && ml->wg_netif && p->wg_peer_index >= 0) {
        struct netif * netif = (struct netif *)ml->wg_netif;
        /* FIX 4: a DERP-only safety peer answers every ~3 s ping via DERP, so this
         * fired connect_derp ~every 3 s — a redundant storm. wireguardif's own
         * periodic init retransmit already provides the steady bring-up cadence;
         * throttle this driver to >= 5 s apart. */
        if (
          (wireguardif_peer_is_up(netif, (u8_t)p->wg_peer_index, NULL, NULL) != ERR_OK) &&
          (now - p->last_derp_reconnect_ms > 5000))
        {
          wireguardif_connect_derp(netif, (u8_t)p->wg_peer_index);
          p->last_derp_reconnect_ms = now;
          ESP_LOGI(TAG, "via-DERP pong from %s: no WG session, (re)firing DERP handshake init", p->hostname);
        }
      }
    }

    /* Consume the probe ONLY for a direct pong. While demoted, every probe is
     * dual-sent (direct + DERP fallback) under ONE txid, and the peer answers
     * BOTH. Consuming on the via-DERP pong (which always arrives) burned the
     * txid, so the direct pong that followed hit the "unmatched" branch below
     * and was DROPPED — the direct path could then never re-promote when DERP
     * won the race (2026-08-09 bench falsification of the stuck-on-DERP fix).
     * A via-DERP match keeps the probe active — but only for a BOUNDED
     * ML_DISCO_DERP_MATCH_GRACE_MS more (stamped here, reclaimed by the sweep
     * in disco_periodic_probes). The direct pong being waited for answers the
     * SAME dual-sent ping, so if the direct path is alive it trails the DERP
     * pong by at most ~one direct RTT; letting the slot ride out the full 5 s
     * window instead is what exhausted the 64-slot table under CMM bursts on
     * the 2026-08-09 bench — pings then went out UNREGISTERED, their pongs
     * arrived unmatched, last_pong_recv_ms froze, and the 4 s pong watchdog
     * demoted healthy paths in a runaway (regains oscillation + robot-disarm
     * waves). Duplicate via-DERP matches keep the FIRST stamp: the window
     * must not be extendable by more relayed pongs. */
    if (!pkt->via_derp) {
      pending_probes[i].active = false;
      consumed = true;
    } else if (pending_probes[i].derp_match_ms == 0) {
      pending_probes[i].derp_match_ms = now;
    }
    matched = true;
    break;
  }

  if (matched && consumed) {
    /* Remember this txid so the machine's DUPLICATE answer on the other transport
     * (direct answered here -> DERP copy arrives next) is logged as
     * a benign duplicate below instead of a scary "unmatched" WARN. */
    remember_consumed_txid(txid);
  } else if (!matched) {
    /* Find peer by disco key for logging */
    int peer_idx = find_peer_by_disco_key(ml, sender_disco_key);
    const char * name = peer_idx >= 0 ? ml->peers[peer_idx].hostname : "?";
    if (txid_recently_consumed(txid)) {
      /* Either the machine's DERP duplicate of a probe the direct pong already
       * consumed, or a straggler direct pong whose slot the post-DERP-match
       * grace window reclaimed (>1 s slower than the relayed answer to the
       * same ping — that path was not going to win promotion anyway). Benign —
       * DEBUG, not WARN. */
      ESP_LOGD(
        TAG,
        "late duplicate pong from %s (via %s) txid=%02x%02x%02x%02x (probe already retired)",
        name,
        pkt->via_derp ? "DERP" : "direct",
        txid[0],
        txid[1],
        txid[2],
        txid[3]);
    } else {
      int active_count = 0;
      for (int i = 0; i < MAX_PENDING_PROBES; i++) {
        if (pending_probes[i].active) active_count++;
      }
      ESP_LOGW(
        TAG,
        "DISCO PONG unmatched from %s (via %s) txid=%02x%02x%02x%02x, active_probes=%d",
        name,
        pkt->via_derp ? "DERP" : "direct",
        txid[0],
        txid[1],
        txid[2],
        txid[3],
        active_count);
    }
  }
}

/* Budget exemption: the priority peer's disco traffic is never rate-limited.
 * Its sender key sits in the cleartext header (offset 6), so this costs one
 * bounded memcmp scan, not crypto. */
static bool disco_pkt_from_priority(microlink_t * ml, const ml_rx_packet_t * pkt)
{
  if ((pkt->len < 38) || (ml->config.priority_peer_ip == 0)) {
    return false;
  }
  for (int i = 0; i < ml->peer_count; i++) {
    if (ml->peers[i].vpn_ip == ml->config.priority_peer_ip) {
      return memcmp(ml->peers[i].disco_key, pkt->data + 6, 32) == 0;
    }
  }
  return false;
}

/* Is the sender's disco key one we already know? Known-key packets open
 * with a CACHED shared key (~1 ms XSalsa20-Poly1305, no DH) — only
 * unknown-key packets pay the full X25519 (the 2026-07-20 rotation
 * rescue), which is what the per-pass budget exists to bound. Budgeting
 * the cheap ones too made every legit peer's ping/pong queue behind
 * tailnet chatter — measured as 300-450 ms disco RTT between two chips
 * whose actual crypto cost was microseconds. */
static bool disco_pkt_known_key(microlink_t * ml, const ml_rx_packet_t * pkt)
{
  if (pkt->len < 38) {
    return false;
  }
  for (int i = 0; i < ml->peer_count; i++) {
    if (ml->peers[i].active && (memcmp(ml->peers[i].disco_key, pkt->data + 6, 32) == 0)) {
      return true;
    }
  }
  return false;
}

static void process_disco_packet(microlink_t * ml, const ml_rx_packet_t * pkt)
{
  if (pkt->len < 62) return; /* magic(6) + key(32) + nonce(24) = 62 minimum */

  /* Verify DISCO magic */
  if (memcmp(pkt->data, DISCO_MAGIC, 6) != 0) return;

  /* Honor the runtime enable_disco config. If we've explicitly opted out
     * of DISCO, drop incoming probes too — they'd otherwise force us to
     * spend a NaCl box-open on every peer's PING and reply with a PONG,
     * which on a 16-peer tailnet is a sustained crypto load on this
     * task / TCPIP thread regardless of whether *we* initiated. */
  if (!ml->config.enable_disco) {
    return;
  }

  /* Extract sender's disco public key */
  const uint8_t * sender_disco_key = pkt->data + 6;

  /* Allowlist filter at the DISCO layer.
     *
     * With enable_disco=true on a tailnet of ≥16 peers (e.g. Polymath's),
     * EVERY peer's magicsock periodically DISCO-pings us. Each ping triggers
     * a NaCl box-open + (if PING) a NaCl box-seal + DERP TX of the PONG.
     * On the ESP32-S3 that's expensive enough to drown the TCPIP thread
     * within ~60 s and wedge lwIP (the same failure mode v15.1 fixed for
     * outbound DISCO). The allowlist-aware filter here mirrors the
     * outbound-side filter in disco_periodic_probes: drop incoming DISCO
     * from peers we don't actually want a direct path with.
     *
     * Lookup is by disco_key (each peer's MapResponse entry carries a
     * disco_key). Unknown disco_key → unknown peer → drop. */
  bool sender_allowed = false;
  bool sender_known = false;
  for (int i = 0; i < ml->peer_count; i++) {
    if (memcmp(ml->peers[i].disco_key, sender_disco_key, 32) == 0) {
      sender_known = true;
      sender_allowed = ml_config_peer_is_allowed(ml->config_httpd, ml->peers[i].vpn_ip);
      break;
    }
  }
  /* Unknown disco key: admit it to the box-open stage even when an
     * allowlist is configured. Disco keys rotate on every tailscaled
     * restart, so a legitimate (even allowlisted) peer's CURRENT key
     * routinely differs from what our MapResponse/NVS recorded — and the
     * node-key rescue that re-identifies such a peer lives in
     * process_disco_ping, which needs the DECRYPTED payload. The allowlist
     * is re-checked there once the sender is identified, so unlisted peers
     * are still dropped before any PONG; the cost is one bounded NaCl
     * box-open per unknown-key packet. (2026-07-20 bench: a rotated laptop
     * key + stale allowlist left tailscale ping unanswered forever —
     * laptop showed tx climbing, rx 0.) */
  if (!sender_known) {
    sender_allowed = true;
  }
  if (!sender_allowed) {
    return; /* silently drop — keeps NaCl box-open count bounded */
  }

  ESP_LOGI(
    TAG,
    "DISCO RX: %d bytes via %s, disco_key=%02x%02x%02x%02x",
    (int)pkt->len,
    pkt->via_derp ? "DERP" : "direct",
    pkt->data[6],
    pkt->data[7],
    pkt->data[8],
    pkt->data[9]);

  /* Extract nonce */
  const uint8_t * nonce = pkt->data + 38;

  /* Decrypt ciphertext */
  const uint8_t * ciphertext = pkt->data + 62;
  size_t ciphertext_len = pkt->len - 62;

  if (ciphertext_len < NACL_BOX_MACBYTES) return;

  size_t plaintext_len = ciphertext_len - NACL_BOX_MACBYTES;
  uint8_t * plaintext = malloc(plaintext_len);
  if (!plaintext) return;

  if (nacl_box_open(plaintext, ciphertext, ciphertext_len, nonce, sender_disco_key, ml->disco_private_key) != 0) {
    ESP_LOGW(TAG, "DISCO decrypt failed");
    free(plaintext);
    return;
  }

  if (plaintext_len < 2) {
    free(plaintext);
    return;
  }

  uint8_t msg_type = plaintext[0];
  switch (msg_type) {
    case DISCO_MSG_PING:
      process_disco_ping(ml, pkt, sender_disco_key, plaintext, plaintext_len);
      break;
    case DISCO_MSG_PONG:
      process_disco_pong(ml, pkt, sender_disco_key, plaintext, plaintext_len);
      break;
    case DISCO_MSG_CALL_ME_MAYBE: {
      /* CallMeMaybe: after type(1)+version(1), payload is N x 18-byte entries
             * Each entry: 16-byte IP (IPv6 or IPv4-mapped) + 2-byte port (big-endian) */
      s_diag_cmm_rx++; /* every received CMM, even unknown-peer: the burst RATE
                        * is the observable (each one cost a box-open already) */
      int peer_idx = find_peer_by_disco_key(ml, sender_disco_key);
      if (peer_idx < 0) {
        ESP_LOGW(TAG, "CallMeMaybe from unknown peer");
        break;
      }

      const uint8_t * ep_data = plaintext + 2;
      size_t ep_data_len = plaintext_len - 2;
      int ep_count = ep_data_len / 18;

      ESP_LOGI(
        TAG,
        "CallMeMaybe from %s: %d endpoints (udp_path=%d)",
        ml->peers[peer_idx].hostname,
        ep_count,
        disco_has_udp_path(ml));

      /* Reply with our own CallMeMaybe (bidirectional NAT traversal). */
      disco_send_call_me_maybe(ml, peer_idx);

      /* Probe the advertised endpoints with ONE DISCO ping (single txid /
       * single pending_probes[] slot, sent to every candidate — the same
       * sweep semantics disco_send_ping_to_peer uses: any endpoint's pong
       * matches the txid). Building a FRESH ping per endpoint burned one
       * probe slot + one seal EACH (up to 8 per received CMM), which is what
       * let a CMM burst blow through the 64-slot table on the 2026-08-09
       * bench. Per received CMM the cost is now exactly one seal + one slot
       * + <=8 UDP sends. */
      if (!disco_has_udp_path(ml)) {
        ESP_LOGW(TAG, "CMM probe skipped: no UDP path (sock4=%d)", ml->disco_sock4);
      } else {
        uint8_t ping_pkt[256];
        size_t ping_len = 0;
        bool ping_built = false;

        for (int i = 0; i < ep_count && i < ML_MAX_ENDPOINTS; i++) {
          const uint8_t * entry = ep_data + (i * 18);
          uint16_t port = (entry[16] << 8) | entry[17];

          /* Check for IPv4-mapped IPv6: ::ffff:A.B.C.D */
          bool is_v4_mapped = true;
          for (int j = 0; j < 10; j++) {
            if (entry[j] != 0) {
              is_v4_mapped = false;
              break;
            }
          }
          if (entry[10] != 0xff || entry[11] != 0xff) is_v4_mapped = false;

          /* DEBUG, not INFO: this 16-byte hex dump per endpoint was ~130
           * blocking UART chars each — real wg_mgr stall time during the CMM
           * bursts this handler must survive. */
          ESP_LOGD(
            TAG,
            "  CMM ep[%d]: v4mapped=%d port=%d ip=%02x%02x%02x%02x",
            i,
            is_v4_mapped,
            port,
            entry[12],
            entry[13],
            entry[14],
            entry[15]);

          if (!is_v4_mapped || port == 0) continue;

          uint32_t ip = ((uint32_t)entry[12] << 24) | ((uint32_t)entry[13] << 16) | ((uint32_t)entry[14] << 8) |
                        (uint32_t)entry[15];

          if (!ping_built) {
            disco_build_ping(ml, peer_idx, ping_pkt, &ping_len);
            ping_built = true;
          }
          if (ping_len > 0) {
            int ret = disco_udp_sendto(ml, ping_pkt, ping_len, ip, port);
            ESP_LOGI(
              TAG,
              "CMM probe -> %d.%d.%d.%d:%d (%d bytes, ret=%d)",
              (int)((ip >> 24) & 0xFF),
              (int)((ip >> 16) & 0xFF),
              (int)((ip >> 8) & 0xFF),
              (int)(ip & 0xFF),
              (int)port,
              (int)ping_len,
              ret);
          }
        }
      }

      /* Also force-ping peer's known endpoints from MapResponse */
      disco_send_ping_to_peer(ml, peer_idx, true);
    } break;
    default:
      ESP_LOGW(TAG, "Unknown DISCO message type: 0x%02x", msg_type);
      break;
  }

  free(plaintext);
  /* pkt->data is freed by the caller */
}

/* ============================================================================
 * WireGuard Packet Processing
 * ========================================================================== */

static void process_wg_packet(microlink_t * ml, const ml_rx_packet_t * pkt)
{
  /* v15.22: dedup MESSAGE_TRANSPORT_DATA (type=4) packets BEFORE the
     * pbuf copy + decrypt. See wg_dedup_is_duplicate() comment block for
     * the rationale (Tailscale dual-pathing this peer). Handshake messages
     * (type 1/2/3) are not deduped — they're rare and the receiver field
     * means different things there. */
  if (pkt->len >= 16 && pkt->data[0] == 4 /* MESSAGE_TRANSPORT_DATA */) {
    uint32_t receiver = (uint32_t)pkt->data[4] | ((uint32_t)pkt->data[5] << 8) | ((uint32_t)pkt->data[6] << 16) |
                        ((uint32_t)pkt->data[7] << 24);
    uint64_t counter = 0;
    for (int i = 0; i < 8; i++) {
      counter |= ((uint64_t)pkt->data[8 + i]) << (i * 8);
    }
    if (wg_dedup_is_duplicate(receiver, counter)) {
      s_wg_dedup_drops++;
      ESP_LOGD(
        TAG,
        "WG DEDUP: dropped via_derp=%d receiver=%lu counter=%llu (total drops=%lu)",
        pkt->via_derp,
        (unsigned long)receiver,
        (unsigned long long)counter,
        (unsigned long)s_wg_dedup_drops);
      free(pkt->data);
      return;
    }
  }

  ESP_LOGI(
    TAG,
    "WG RX: %d bytes, via_derp=%d, type=%d, from=%02x%02x%02x%02x",
    (int)pkt->len,
    pkt->via_derp,
    pkt->len >= 4 ? pkt->data[0] : -1,
    pkt->src_pubkey[0],
    pkt->src_pubkey[1],
    pkt->src_pubkey[2],
    pkt->src_pubkey[3]);
  if (!ml->wg_netif) {
    free(pkt->data);
    return;
  }

  struct netif * netif = (struct netif *)ml->wg_netif;
  void * device = netif->state;
  if (!device) {
    free(pkt->data);
    return;
  }

  /* Allocate PBUF_RAM and copy data so the pbuf OWNS its data.
     * This is required because wireguardif decrypts in-place and then
     * calls ip_input → tcpip_input which posts to the TCPIP thread.
     * With PBUF_REF the backing data would be freed before the TCPIP
     * thread processes the packet. */
  struct pbuf * p = pbuf_alloc(PBUF_RAW, pkt->len, PBUF_RAM);
  if (!p) {
    free(pkt->data);
    return;
  }
  pbuf_take(p, pkt->data, pkt->len);
  free(pkt->data); /* Original data no longer needed — pbuf has its own copy */

  /* Build source address */
  ip_addr_t addr;
  if (pkt->via_derp) {
    ip_addr_set_any(false, &addr);
  } else {
    IP_SET_TYPE_VAL(addr, IPADDR_TYPE_V4);
    ip4_addr_set_u32(ip_2_ip4(&addr), htonl(pkt->src_ip));
  }

  /* Call WG RX handler — pbuf is PBUF_RAM so data survives async delivery.
     * v15.20: wrap in TCPIP core lock since wireguardif_network_rx →
     * wireguardif_process_data_message → ip_input modifies lwIP state that
     * the TCPIP thread is concurrently touching. Per upstream microlink
     * PR#14, this is the root cause of the silent panic at ~100-180 s of
     * sustained direct-UDP traffic. */
  bool need_lock = ml_lwip_core_lock_needed();
  if (need_lock) {
    LOCK_TCPIP_CORE();
  }
  wireguardif_network_rx(device, NULL, p, &addr, pkt->src_port);
  if (need_lock) {
    UNLOCK_TCPIP_CORE();
  }
}

/* THE wg_rx drain — every processing site uses it (docs/
 * STALL_EVENT_CLOSURE_DESIGN.md §2). Data packets (heartbeat decrypt is
 * sub-ms) flow freely; HANDSHAKES (~45 ms of X25519 each, under
 * LOCK_TCPIP_CORE) share one per-pass budget across all call sites so a
 * wave can never stall heartbeat processing. Snapshot-bounded: a deferred
 * handshake requeued to the tail cannot be re-popped in the same call.
 * Deferral fails SAFE: initiators retransmit at 5 s with ~60 s of session
 * margin; worst case is the existing rekey-late nuisance stop. */

int ml_wg_get_stall_events(ml_wg_stall_event_t * out, int max)
{
  uint32_t w = __atomic_load_n(&s_wg_stall_widx, __ATOMIC_ACQUIRE);
  int n = (w < (uint32_t)ML_STALL_RING_LEN) ? (int)w : ML_STALL_RING_LEN;
  if (n > max) n = max;
  for (int i = 0; i < n; i++) {
    out[i] = s_wg_stall_ring[(w - n + i) % ML_STALL_RING_LEN];
  }
  return n;
}

void ml_wg_get_hs_budget_diag(uint32_t out[2])
{
  out[0] = s_diag_wg_hs_deferred;
  out[1] = s_diag_wg_hs_dropped;
}

/* True when a fleet server is configured (its learned region may still be 0
 * — that combination is the task-#42 silent-diversity-loss signature). */
bool ml_wg_fleet_configured(microlink_t * ml)
{
  return fleet_server_ip_cached() != 0;
}

static void wg_mgr_drain_wg_rx(microlink_t * ml)
{
  UBaseType_t pops = uxQueueMessagesWaiting(ml->wg_rx_queue);
  ml_rx_packet_t pkt;
  while (pops-- > 0 && xQueueReceive(ml->wg_rx_queue, &pkt, 0) == pdTRUE) {
    bool hs = ml_wg_is_handshake_frame(pkt.data, pkt.len);
    if (hs && s_wg_hs_budget <= 0) {
      if (pkt.requeues < ML_WG_HS_REQUEUE_MAX) {
        pkt.requeues++;
        if (xQueueSend(ml->wg_rx_queue, &pkt, 0) == pdTRUE) {
          s_diag_wg_hs_deferred++;
          continue;
        }
      }
      free(pkt.data); /* cap hit or queue refilled: shed the handshake */
      s_diag_wg_hs_dropped++;
      continue;
    }
    if (hs) {
      s_wg_hs_budget--;
      s_pass_wg_handshakes++;
    }
    s_pass_wg_pkts++;
    process_wg_packet(ml, &pkt);
  }
}

/* ============================================================================
 * SendCallMeMaybe (client-initiated NAT traversal)
 *
 * Sends our local endpoints (LAN + STUN) to a peer via DERP, telling them
 * "connect to me at these addresses". Peer will then initiate WireGuard
 * handshakes to each endpoint, bypassing NAT asymmetry.
 *
 * Reference: tailscale/wgengine/magicsock/magicsock.go (sendCallMeMaybe)
 * ========================================================================== */

/* Active-uplink LAN IPv4 (host byte order), or 0 if none.
 *
 * The LAN endpoint we advertise (MapRequest endpoints + disco CallMeMaybe) used
 * to be read only from the "WIFI_STA_DEF" netif. On the Ethernet (W5500) and
 * USB-NCM uplinks — the primary production transports — that netif doesn't
 * exist, so we advertised NO local endpoint and same-LAN peers could never form
 * a direct WireGuard path (they only had our shared public STUN endpoint, which
 * needs NAT hairpin). Source it instead from the active uplink: the default-route
 * esp_netif (highest route_prio, up, with a valid IPv4). The WG tunnel is a raw
 * lwIP netif (not an esp_netif) so it is never returned here. Falls back to
 * WIFI_STA_DEF for safety. See docs/SAME_LAN_DIRECT_PATH_PLAN.md. */
uint32_t ml_active_lan_ip(void)
{
  esp_netif_ip_info_t ip;
  esp_netif_t * n = esp_netif_get_default_netif();
  if (n != NULL && esp_netif_get_ip_info(n, &ip) == ESP_OK && ip.ip.addr != 0) {
    return ntohl(ip.ip.addr);
  }
  esp_netif_t * sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (sta != NULL && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0) {
    return ntohl(ip.ip.addr);
  }
  return 0;
}

static void disco_send_call_me_maybe(microlink_t * ml, int peer_idx)
{
  /* Honor enable_disco. With disco off, CMM is meaningless (we'll never
     * answer the resulting direct probes anyway — see
     * process_disco_packet's early-return), and each CMM is an expensive
     * NaCl-box encryption + DERP TX. Sending 16 of these at boot for a
     * 16-peer tailnet is a real load spike on the WG mgr task. */
  if (!ml->config.enable_disco) {
    return;
  }

  ml_peer_t * p = &ml->peers[peer_idx];

  /* Chip<->chip CMM chain breaker (single choke point — EVERY CMM send path
   * runs through here). process_disco_packet answers a received CMM with a
   * CMM of its own; between two microlink chips (remote<->machn) that mutual
   * reply is a ping-pong chain sustained at DERP RTT until a frame drops.
   * The demotion-CMM + 5 s relay-retry rounds re-ignite such chains
   * constantly, and each received CMM costs a reply seal + endpoint probes —
   * the fuel of the 2026-08-09 probe-table exhaustion runaway. At most one
   * CMM per peer per ML_DISCO_CMM_MIN_INTERVAL_MS. Invariant: the FIRST CMM
   * in any window passes (demotion event, >=5 s-spaced retry round, boot
   * broadcast). A re-demotion within the same window IS throttled (rapid
   * flap); its recovery falls to the next relay-retry round — worst case
   * +5 s, still bounded. A suppressed chain reply is harmless — the exchange
   * it would answer already carried our endpoints to that peer within the
   * last window. The stamp is written on the actual-send path below (after
   * the endpoint check), so a no-endpoint attempt — e.g. pre-STUN cold boot —
   * does not burn the window while sending nothing. */
  {
    uint64_t cmm_now = ml_get_time_ms();
    if (p->last_cmm_sent_ms != 0 && cmm_now - p->last_cmm_sent_ms < ML_DISCO_CMM_MIN_INTERVAL_MS) {
      return;
    }
  }

  /* Build plaintext: [type(1)][version(1)][endpoints(N * 18)] */
  s_pass_cmm_sends++; /* ring attribution: each CMM ends in a ~30 ms NaCl seal.
                       * Known noise, accepted (diag-only): the rare no-endpoint
                       * bail over-counts by one, and the ml_wg_mgr_send_cmm
                       * wrapper runs on app tasks (unsynchronized u16 bump). */
  uint8_t plaintext[2 + 3 * 18]; /* Up to 3 endpoints */
  int pt_len = 0;
  int ep_count = 0;

  plaintext[pt_len++] = DISCO_MSG_CALL_ME_MAYBE;
  plaintext[pt_len++] = 0; /* version */

  /* 1. Local LAN IP endpoint (critical for same-network peers). Sourced from
     * the active uplink (eth/usb/wifi), not WiFi-only — see ml_active_lan_ip(). */
  uint32_t local_ip = ml_active_lan_ip();
  uint16_t local_port = ml->disco_local_port;
  if (local_ip != 0 && local_port > 0) {
    /* IPv6-mapped IPv4: ::ffff:A.B.C.D */
    memset(plaintext + pt_len, 0, 10);
    pt_len += 10;
    plaintext[pt_len++] = 0xff;
    plaintext[pt_len++] = 0xff;
    plaintext[pt_len++] = (local_ip >> 24) & 0xFF;
    plaintext[pt_len++] = (local_ip >> 16) & 0xFF;
    plaintext[pt_len++] = (local_ip >> 8) & 0xFF;
    plaintext[pt_len++] = local_ip & 0xFF;
    plaintext[pt_len++] = (local_port >> 8) & 0xFF;
    plaintext[pt_len++] = local_port & 0xFF;
    ep_count++;

    ESP_LOGI(
      TAG,
      "CMM endpoint: LAN %lu.%lu.%lu.%lu:%u",
      (unsigned long)((local_ip >> 24) & 0xFF),
      (unsigned long)((local_ip >> 16) & 0xFF),
      (unsigned long)((local_ip >> 8) & 0xFF),
      (unsigned long)(local_ip & 0xFF),
      local_port);
  }

  /* 2. STUN public endpoint (for cross-NAT peers) */
  if (ml->stun_public_ip != 0) {
    uint32_t pub_ip = ml->stun_public_ip;
    uint16_t pub_port = (ml->stun_public_port != 0) ? ml->stun_public_port : ml->disco_local_port;

    memset(plaintext + pt_len, 0, 10);
    pt_len += 10;
    plaintext[pt_len++] = 0xff;
    plaintext[pt_len++] = 0xff;
    plaintext[pt_len++] = (pub_ip >> 24) & 0xFF;
    plaintext[pt_len++] = (pub_ip >> 16) & 0xFF;
    plaintext[pt_len++] = (pub_ip >> 8) & 0xFF;
    plaintext[pt_len++] = pub_ip & 0xFF;
    plaintext[pt_len++] = (pub_port >> 8) & 0xFF;
    plaintext[pt_len++] = pub_port & 0xFF;
    ep_count++;

    ESP_LOGI(
      TAG,
      "CMM endpoint: STUN %lu.%lu.%lu.%lu:%u",
      (unsigned long)((pub_ip >> 24) & 0xFF),
      (unsigned long)((pub_ip >> 16) & 0xFF),
      (unsigned long)((pub_ip >> 8) & 0xFF),
      (unsigned long)(pub_ip & 0xFF),
      pub_port);
  }

  if (ep_count == 0) {
    ESP_LOGW(TAG, "CMM: no endpoints available for %s", p->hostname);
    return;
  }

  /* Endpoints exist — this attempt really sends, so it may burn the throttle
   * window (see chain-breaker comment above). */
  p->last_cmm_sent_ms = ml_get_time_ms();

  /* Encrypt with NaCl box */
  uint8_t nonce[DISCO_NONCE_LEN];
  esp_fill_random(nonce, DISCO_NONCE_LEN);

  uint8_t ciphertext[sizeof(plaintext) + NACL_BOX_MACBYTES];
  nacl_box(ciphertext, plaintext, pt_len, nonce, p->disco_key, ml->disco_private_key);

  /* Build packet: magic(6) + disco_pubkey(32) + nonce(24) + ciphertext */
  uint8_t pkt[256];
  size_t pos = 0;
  memcpy(pkt + pos, DISCO_MAGIC, 6);
  pos += 6;
  memcpy(pkt + pos, ml->disco_public_key, 32);
  pos += 32;
  memcpy(pkt + pos, nonce, DISCO_NONCE_LEN);
  pos += DISCO_NONCE_LEN;
  size_t ct_len = pt_len + NACL_BOX_MACBYTES;
  memcpy(pkt + pos, ciphertext, ct_len);
  pos += ct_len;

  /* Send via DERP */
  esp_err_t err = ml_derp_queue_send(ml, p->public_key, pkt, pos);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "CallMeMaybe sent to %s (%d endpoints)", p->hostname, ep_count);
  } else {
    ESP_LOGW(TAG, "CallMeMaybe send failed for %s: %d", p->hostname, err);
  }
}

/* Public wrapper for UDP API to trigger CallMeMaybe. */
void ml_wg_mgr_send_cmm(microlink_t * ml, uint32_t peer_vpn_ip)
{
  int idx = find_peer_by_ip(ml, peer_vpn_ip);
  if (idx >= 0) {
    disco_send_call_me_maybe(ml, idx);
  }
}

/* On-demand WG handshake trigger (called from TCP/UDP API when session needed).
 *
 * Dual-path strategy:
 * 1. DERP: Always send via DERP relay (reliable, works through all NATs)
 * 2. Direct: If DISCO has discovered a direct endpoint, ALSO send the
 *    handshake init directly to the peer's UDP port. This is critical
 *    because the direct path hits magicsock's receiveIPv4() handler which
 *    calls noteRecvActivity() → maybeReconfigWireguardLocked() to re-add
 *    trimmed peers to wireguard-go. The DERP path alone does NOT trigger
 *    this re-add for WG handshake packets, so idle peers silently drop
 *    DERP-only handshake inits.
 *
 * The direct handshake arrives at the peer's magicsock UDP socket, wakes
 * the lazy peer, and wireguard-go processes the init and responds. */
esp_err_t ml_wg_mgr_trigger_handshake(microlink_t * ml, uint32_t dest_vpn_ip)
{
  if (!ml || !ml->wg_netif) return ESP_ERR_INVALID_STATE;

  int idx = find_peer_by_ip(ml, dest_vpn_ip);
  if (idx < 0) return ESP_ERR_NOT_FOUND;

  ml_peer_t * p = &ml->peers[idx];
  if (p->wg_peer_index < 0) return ESP_ERR_INVALID_STATE;

  struct netif * netif = (struct netif *)ml->wg_netif;

  /* Don't destroy an existing valid session */
  err_t is_up = wireguardif_peer_is_up(netif, (u8_t)p->wg_peer_index, NULL, NULL);
  if (is_up == ERR_OK) return ESP_OK;

  /* Path 1: DERP (reliable fallback) */
  wireguardif_connect_derp(netif, (u8_t)p->wg_peer_index);
  /* Wake the remote's magicsock via an inbound DISCO ping so it lazily
     * adds us — a bare DERP WG-init alone won't (see disco_periodic_probes).
     * Applies to on-demand bring-up of any peer, not just the priority one. */
  disco_send_ping_to_peer(ml, idx, true);
  ESP_LOGI(TAG, "WG handshake triggered (DERP + disco wake) to %s", p->hostname);

  /* Path 2: Direct UDP (if DISCO has a known endpoint).
     * This wakes the peer's magicsock via receiveIPv4 → noteRecvActivity.
     * Set connect_ip first, then call wireguardif_connect which copies
     * connect_ip → ip and starts a second handshake to the direct endpoint. */
  if (p->best_ip != 0 && p->best_port != 0) {
    ip_addr_t ep_ip;
    IP_SET_TYPE_VAL(ep_ip, IPADDR_TYPE_V4);
    ip4_addr_set_u32(ip_2_ip4(&ep_ip), htonl(p->best_ip));
    wireguardif_update_endpoint(netif, (u8_t)p->wg_peer_index, &ep_ip, p->best_port);
    wireguardif_connect(netif, (u8_t)p->wg_peer_index);
    ESP_LOGI(
      TAG,
      "WG handshake triggered (direct) to %s at %d.%d.%d.%d:%d",
      p->hostname,
      (int)((p->best_ip >> 24) & 0xFF),
      (int)((p->best_ip >> 16) & 0xFF),
      (int)((p->best_ip >> 8) & 0xFF),
      (int)(p->best_ip & 0xFF),
      (int)p->best_port);
  }

  return ESP_OK;
}

bool ml_wg_mgr_peer_is_up(microlink_t * ml, uint32_t vpn_ip)
{
  if (!ml || !ml->wg_netif) return false;
  int idx = find_peer_by_ip(ml, vpn_ip);
  if (idx < 0) return false;
  ml_peer_t * p = &ml->peers[idx];
  if (p->wg_peer_index < 0) return false;
  struct netif * netif = (struct netif *)ml->wg_netif;
  ip_addr_t cur_ip;
  u16_t cur_port;
  bool up = wireguardif_peer_is_up(netif, (u8_t)p->wg_peer_index, &cur_ip, &cur_port) == ERR_OK;
  if (up) {
    struct wireguard_device * dev = (struct wireguard_device *)netif->state;
    if (dev && p->wg_peer_index < WIREGUARD_MAX_PEERS) {
      struct wireguard_peer * wp = &dev->peers[p->wg_peer_index];
      bool key_match = (memcmp(wp->public_key, p->public_key, 32) == 0);
      ESP_LOGI(
        TAG,
        "WG peer UP: %s wg_idx=%d ep=%s:%u key=%02x%02x%02x%02x %s",
        p->hostname,
        p->wg_peer_index,
        ip_addr_isany(&cur_ip) ? "DERP" : ipaddr_ntoa(&cur_ip),
        cur_port,
        wp->public_key[0],
        wp->public_key[1],
        wp->public_key[2],
        wp->public_key[3],
        key_match ? "KEY_OK" : "KEY_MISMATCH!");
    }
  }
  return up;
}

void ml_wg_mgr_update_transport(microlink_t * ml)
{
  (void)ml; /* transport is uniform (eth/usb/wifi all real lwIP sockets) */
}

/* ============================================================================
 * Periodic DISCO probing (rate-limited per tailscaled timing)
 * ========================================================================== */

/* Max upgrade probes per call to spread DISCO load across ticks.
 * With 16 allowed peers: full probe cycle = 8 ticks × 1s = 8s,
 * well within the 15s upgrade interval. */
#define DISCO_PROBES_PER_TICK 2

static int disco_probe_start_idx = 0;

/* Disco-first session wake for a must-reach peer. Initiation MUST be
 * disco-first: a normal tailscaled peer lazily adds us to wireguard-go only on
 * an inbound DISCO ping, never on a bare DERP-relayed WG-init — otherwise the
 * peer has to `tailscale ping` us before a session forms. Re-ping over DERP
 * (node-key-bearing) every ~ML_DISCO_PRIORITY_HB_MS and refresh the WG-init
 * until the session is up. One small DERP frame per 3 s per peer while down.
 * link_healthy: pass the app's view of the link. Even when is_up reads true,
 * a false here forces re-establishment — is_up can't see a peer that forgot us
 * after a restart (zombie keypair); only the app can (no heartbeat replies /
 * failed check-in). */
static void disco_wake_peer(microlink_t * ml, uint32_t vpn_ip, bool link_healthy)
{
  if (vpn_ip == 0 || ml->wg_netif == NULL) return;
  int pidx = find_peer_by_ip(ml, vpn_ip);
  if (pidx < 0 || ml->peers[pidx].wg_peer_index < 0) return;
  struct netif * nif = (struct netif *)ml->wg_netif;
  bool up = (wireguardif_peer_is_up(nif, (u8_t)ml->peers[pidx].wg_peer_index, NULL, NULL) == ERR_OK);
  if (!up || !link_healthy) {
    uint64_t nowp = ml_get_time_ms();
    if (nowp - ml->peers[pidx].last_ping_sent_ms >= ML_DISCO_PRIORITY_HB_MS) {
      disco_send_ping_to_peer(ml, pidx, true);
      wireguardif_connect_derp(nif, (u8_t)ml->peers[pidx].wg_peer_index);
    }
  }
}

static void disco_periodic_probes(microlink_t * ml)
{
  /* Pinned-peer session wakes — run BEFORE the enable_disco gate so
     * the safety peer AND the management/OTA server always (re)establish
     * their WG session disco-first, even on a busy tailnet where general disco is
     * throttled. Without this the management peer can be in the table yet have no live
     * session (check-in silently fails). */
  disco_wake_peer(ml, ml->config.priority_peer_ip, ml->priority_link_healthy);
  disco_wake_peer(ml, fleet_server_ip_cached(), ml->fleet_link_healthy);
  /* App-health-tracked peers (per-machine pstop targets). Skip an entry
     * that duplicates the priority peer — it was already waked above. */
  for (int hp = 0; hp < ML_EXTRA_PINS; hp++) {
    if ((s_health_peers[hp].ip != 0) && (s_health_peers[hp].ip != ml->config.priority_peer_ip)) {
      disco_wake_peer(ml, s_health_peers[hp].ip, s_health_peers[hp].healthy);
    }
  }

  /* Honor the runtime enable_disco config flag. Previously this flag was
     * stored but never read — the chip ran DISCO probes regardless. With a
     * 16-peer table that's a per-second burst of UDP+crypto across every
     * peer, which has been hammering the lwIP TCPIP thread and is the
     * leading hypothesis for the chip-side wedge. Skip everything when
     * disco is disabled — peer→peer direct paths just stay on DERP relay.
     * (The priority-peer wake above is intentionally exempt.) */
  if (!ml->config.enable_disco) {
    return;
  }

  uint64_t now = ml_get_time_ms();
  int upgrade_probes_sent = 0;
  int hb_pings_sent = 0;
  bool relay_retry_fired = false; /* at most one CMM+sweep round per tick */

  /* Rotate start index so we don't always process peers in the same order */
  int start = disco_probe_start_idx;
  if (start >= ml->peer_count) start = 0;

  for (int n = 0; n < ml->peer_count; n++) {
    int i = (start + n) % ml->peer_count;
    ml_peer_t * p = &ml->peers[i];
    if (!p->active) continue;

    /* Peer allowlist filter: check early so we can skip expensive work.
         * Inbound DISCO pings from any peer are still answered (don't break remote). */
    bool peer_allowed = ml_config_peer_is_allowed(ml->config_httpd, p->vpn_ip);

    bool is_priority = is_safety_peer(ml, p->vpn_ip); /* every pstop counterpart gets
                            * 3 s disco heartbeats (fresh RTT on BOTH ends) + pong watchdog. */

    /* EVERY safety peer gets the DERP data+handshake mirror (dual_path) — it
     * was priority-peer-only, so the MACHINE's TX leg to its remotes (data
     * replies AND rekey handshakes, wireguardif_peer_output) was direct-only
     * and could die silently; a starved rekey then surfaces as ENOTCONN
     * 180 s later (run-20/21 audit §1b). Reconciled here (idempotent flag
     * write) because health-tracking can register after peer add. */
    if (is_priority && ml->wg_netif && p->wg_peer_index >= 0) {
      wireguardif_set_dual_path((struct netif *)ml->wg_netif, (u8_t)p->wg_peer_index, true);
    }

    /* Direct-path health. Two triggers:
         *  - trust-lease expiry (all peers, the original behaviour), and
         *  - pong-recency watchdog (priority peer only): liveness is what
         *    last_pong_recv_ms already tracks, but demotion used to rest on
         *    the fixed 60 s lease alone — a dead direct path went dark for
         *    10-60 s (measured) before failover. With the 3 s priority
         *    heartbeat below, two missed pongs demote in ~8-9 s. */
    bool lease_expired = p->has_direct_path && now > p->trust_until_ms;
    /* pong_dead keys on the DIRECT-path pong stamp: a relay-carried pong
     * proves the peer, not this path, and using the path-blind stamp let a
     * half-dead direct path (rx alive, tx dead) evade the veto-streak cap
     * indefinitely — the trigger cleared and reset the streak each time a
     * relay pong landed (2026-08-14 bbd8 wedge). With the direct stamp, a
     * truly dead path keeps the trigger true and the cap demotes in ~10 s. */
    bool pong_dead = p->has_direct_path && is_priority && p->last_direct_pong_recv_ms != 0 &&
                     now - p->last_direct_pong_recv_ms > ML_DISCO_PRIORITY_PATH_DEAD_MS;

    /* Demote-VERIFICATION (ml_demote_verdict.h): both triggers above rest on
     * the DISCO side-channel, whose pings can go silent on a jittery uplink
     * while the WG DATA flow (the 5 Hz safety heartbeat) still arrives on the
     * direct path. Demoting then tears down a WORKING direct path — for a
     * symmetric-NAT peer possibly terminally (2026-08-11: only a reboot
     * re-established it, and the relay fallback is too jittery for the 2 s
     * pstop timeout in ANY region). Authenticated direct data rx within
     * ML_DEMOTE_DIRECT_RX_FRESH_MS outranks a missing pong: renew the lease
     * and hold the path. A truly dead path stops producing direct rx within
     * the window, and demotion proceeds unchanged. Safety peers only. */
    u32_t direct_rx_age_ms = 0;
    bool direct_rx_age_valid =
      (lease_expired || pong_dead) && is_priority && ml->wg_netif && p->wg_peer_index >= 0 &&
      wireguardif_peer_direct_rx_age((struct netif *)ml->wg_netif, (u8_t)p->wg_peer_index, &direct_rx_age_ms) == ERR_OK;
    ml_demote_verdict_t demote = ml_demote_verdict(
      lease_expired, pong_dead, is_priority, direct_rx_age_valid, direct_rx_age_ms, ML_DEMOTE_DIRECT_RX_FRESH_MS);
    /* Veto-streak cap (audited a175361 hazard): the veto's evidence is
     * RX-only and can pin a TX-dead path. Past the cap, fail over to DERP.
     * Streak resets whenever the trigger clears or a demote executes. */
    demote = ml_demote_verdict_capped(demote, p->demote_veto_ticks, ML_DEMOTE_VETO_MAX_TICKS);
    if (demote == ML_DEMOTE_VETO) {
      p->demote_veto_ticks++;
      p->trust_until_ms = now + ML_DISCO_TRUST_DURATION_MS;
      s_diag_demote_vetoes++;
      if (now - s_last_demote_veto_log_ms > 10000) {
        s_last_demote_veto_log_ms = now;
        ESP_LOGW(
          TAG,
          "direct demote VETOED for %s: WG data rx %ums ago on direct (disco %s, streak %u)",
          p->hostname,
          (unsigned)direct_rx_age_ms,
          pong_dead ? "pong-dead" : "lease-expired",
          (unsigned)p->demote_veto_ticks);
      }
    } else {
      p->demote_veto_ticks = 0;
    }
    if (demote == ML_DEMOTE_GO) {
      ESP_LOGI(
        TAG,
        "Direct path to %s %s, reverting to DERP",
        p->hostname,
        pong_dead ? "went silent (pong watchdog)" : "expired");
      p->has_direct_path = false;

      /* FIX 2: hard-NAT flap damping. A direct path that lived < 15 s is a flap
       * (a lucky direct pong promoted, the reverse NAT mapping then died and the
       * pong watchdog demoted). Count it and exponentially back off the
       * direct-UPGRADE re-probe (10 s -> 20 -> 40 -> 60 s cap) so a provably
       * hard-NAT peer holds a STEADY DERP bond instead of swapping every 5 s and
       * wobbling the heartbeat. Only priority/health-tracked peers arm the
       * backoff — bulk peers keep direct_backoff_until == 0, so the peer-scaling
       * armor and the >15 s-established failover are untouched. This changes only
       * WHEN we re-probe; it never installs a direct endpoint (827386f preserved).*/
      if (is_priority && p->direct_promoted_ms != 0 && now - p->direct_promoted_ms < 15000) {
        if (p->direct_flap_count < 8) p->direct_flap_count++;
        uint32_t shift = p->direct_flap_count < 4 ? p->direct_flap_count : 4;
        uint64_t backoff = (uint64_t)5000u << shift;
        if (backoff > 60000u) backoff = 60000u;
        p->direct_backoff_until = now + backoff;
      }

      /* Arm the relay-bound retry (safety peers only). First round FAST
       * (ML_DISCO_RELAY_RETRY_FIRST_MS): when the outage also killed our
       * outbound NAT mapping, the reprobe pings below die in flight and only
       * a CMM round (far side probes back) recovers — a 30 s floor here left
       * the peer visibly relay-bound through the whole bench observation
       * window (2026-08-09). Fresh outage = fresh backoff. */
      if (is_priority) {
        p->relay_retry_next_ms = now + ML_DISCO_RELAY_RETRY_FIRST_MS;
        p->relay_retry_count = 0;
      }

      /* Only do DERP fallback + re-probe for allowed peers.
             * Non-allowed peers just get their state cleaned above. */
      if (peer_allowed) {
        if (ml->wg_netif && p->wg_peer_index >= 0) {
          struct netif * netif = (struct netif *)ml->wg_netif;
          /* Clear the WG endpoint UNCONDITIONALLY (was: only when
                     * the session was up). A session that died mid-outage
                     * left a stale direct endpoint that handshake
                     * retransmits hammer forever — they resolve the target
                     * from peer->ip, and even the 540 s reset path restores
                     * connect_ip, which DISCO overwrote with the same dead
                     * endpoint. connect_derp zeroes the endpoint and fires
                     * a fresh handshake via DERP either way. */
          wireguardif_connect_derp(netif, (u8_t)p->wg_peer_index);
          ESP_LOGI(TAG, "  WG endpoint cleared, %s now via DERP", p->hostname);
        }

        /* Force-ping to try re-establishing the direct path */
        disco_send_ping_to_peer(ml, i, true);

        /* Safety peers: ALSO fire one immediate CallMeMaybe. If the outage
         * that caused this demotion killed our outbound NAT mapping, the
         * force-ping above never reaches the peer — the far side must probe
         * US to re-punch the hole, and it only does that on a CMM. One NaCl
         * seal per demotion EVENT (rare); bulk peers are excluded so the
         * peer-scaling armor is untouched. */
        if (is_priority) {
          disco_send_call_me_maybe(ml, i);
        }
      }
    }

    if (!peer_allowed) continue;

    /* Probe for direct path upgrade (every UPGRADE_INTERVAL when on DERP).
         * Throttled to DISCO_PROBES_PER_TICK to spread load and reduce jitter. */
    uint64_t upgrade_interval = is_priority ? ML_DISCO_PRIORITY_REPROBE_MS : ML_DISCO_UPGRADE_INTERVAL_MS;
    /* FIX 2: while a hard-NAT safety peer is in flap-backoff, hold the direct
     * upgrade re-probe so it stays on its steady DERP bond. Bulk peers keep
     * direct_backoff_until == 0, so this gate is a no-op for them. */
    if (!p->has_direct_path && now - p->last_upgrade_ms > upgrade_interval && now > p->direct_backoff_until) {
      if (upgrade_probes_sent < DISCO_PROBES_PER_TICK) {
        disco_send_ping_to_peer(ml, i, false);
        p->last_upgrade_ms = now;
        upgrade_probes_sent++;
      }
    }

    /* Relay-bound direct-path re-establishment (safety peers only). The 5 s
         * reprobe above re-pings candidates WE hold; it cannot recover when the
         * far side's knowledge of US went stale (its NAT mapping to us died, an
         * endpoint patch got lost server-side). Re-run the candidate EXCHANGE:
         * a CallMeMaybe re-advertising our current LAN + STUN endpoints (so the
         * peer probes back, re-opening the reverse path) plus a forced sweep of
         * everything we hold, incl. the last-proven best endpoint. Slow
         * exponential backoff per peer (30 s -> 300 s cap), at most one round
         * per 1 s tick fleet-wide, honours the hard-NAT flap backoff, and stops
         * the moment direct is regained (state cleared on promotion). Pure
         * disco side-channel: the DERP-carried heartbeat path is untouched. */
    if (is_priority && !p->has_direct_path) {
      if (p->relay_retry_next_ms == 0) {
        /* Cold relay-bound peer (never promoted since boot): arm the cycle. */
        p->relay_retry_next_ms = now + ML_DISCO_RELAY_RETRY_FIRST_MS;
      } else if (!relay_retry_fired && now >= p->relay_retry_next_ms && now > p->direct_backoff_until) {
        relay_retry_fired = true; /* bounded: one NaCl seal + one sweep per tick */
        disco_send_call_me_maybe(ml, i);
        disco_send_ping_to_peer(ml, i, true);
        /* Backoff after round N: 30 s << (N-1), capped at 300 s. With the
         * 5 s first round that gives 5 -> 30 -> 60 -> 120 -> 240 -> 300 cap. */
        if (p->relay_retry_count < 5u) {
          p->relay_retry_count++;
        }
        uint64_t iv = (uint64_t)ML_DISCO_RELAY_RETRY_MIN_MS << (p->relay_retry_count - 1u);
        if (iv > ML_DISCO_RELAY_RETRY_MAX_MS) {
          iv = ML_DISCO_RELAY_RETRY_MAX_MS;
        }
        p->relay_retry_next_ms = now + iv;
        s_diag_relay_retries++;
        ESP_LOGI(
          TAG,
          "relay-bound retry #%u for %s (next in %llu s)",
          (unsigned)p->relay_retry_count,
          p->hostname,
          (unsigned long long)(iv / 1000u));
        /* Relay-stuck recovery: the CMM + ping-sweep above re-probe the endpoints
         * WE hold, but they may be STALE — steady-state coord updates are
         * OmitPeers=true, so we never re-pull the peer's CURRENT endpoints; only a
         * full re-sync does (why a reboot was the only fix). The peer's NAT
         * mapping to us may also have rebound. After the first failed round, ask
         * coord to reconnect (re-register + full peer re-fetch), refreshing this
         * peer's endpoints so the NEXT sweep lands and re-hole-punches. NOT gated
         * on nat_mapping_varies: the stale-endpoint hole is real for ANY relay-
         * bound safety peer (symmetric NAT just makes it terminal), and the chip's
         * own symmetric-NAT flag isn't reliably set on the far peer's behalf.
         * Control-plane only (WG heartbeat/green untouched); only fires when the
         * path is ALREADY relay (no healthy path to disturb); rate-limited so a
         * persistently-relay peer can't storm reconnects. */
        if (
          p->relay_retry_count >= 1u && ml->coord_cmd_queue != NULL &&
          (s_last_relay_refetch_ms == 0 || now - s_last_relay_refetch_ms >= ML_RELAY_REFETCH_MIN_MS))
        {
          s_last_relay_refetch_ms = now;
          s_diag_relay_refetch_reqs++;
          ml_coord_cmd_t rc = ML_CMD_FORCE_RECONNECT;
          (void)xQueueSend(ml->coord_cmd_queue, &rc, 0);
          ESP_LOGW(TAG, "relay-stuck safety peer %s: coord re-fetch (reconnect) for endpoint refresh", p->hostname);
        }
        /* Escalation: the coord re-fetch above refreshes the PEER's endpoints,
         * but a re-fetch re-ingests an EXISTING peer with disco state PRESERVED
         * (the !existing guard in ml_wg_apply_peer_update), so we keep re-probing
         * the stale best_ip that no longer reaches a symmetric-NAT peer whose
         * mapping rebound — which is why only a reboot recovered direct. After a
         * few failed rounds, do the disco half of a per-peer "reboot": clear THIS
         * relay-stuck safety peer's disco session so the next sweep re-runs a
         * from-scratch hole-punch (fresh CMM + unthrottled ping to the just-
         * refreshed endpoints). The peer then pings the machine from its LIVE
         * mapping and the machine's !has_direct_path learn-from-ping adopts it.
         * best_ip/pong timers are disco candidates only — the WG data path and the
         * DERP-carried heartbeat are untouched. Gated to a relay-bound safety peer
         * (no direct path to disturb) and rate-limited. */
        /* v2 (audit of 38fca7d): rate limit is PER-PEER (the global stamp let
         * two stuck peers starve each other), best_ip/best_port are KEPT (the
         * last-proven endpoint is "typically the ONLY reachable candidate" —
         * wiping it destroyed a recovery vector when the peer's mapping had
         * NOT rebound), and the hard-NAT flap backoff is cleared so the
         * re-punch can actually probe. Ping/pong/CMM timers still reset so
         * the next sweep runs unthrottled from scratch. */
        if (p->relay_retry_count >= 3u && (p->disco_reset_next_ms == 0 || now >= p->disco_reset_next_ms)) {
          p->disco_reset_next_ms = now + ML_RELAY_REFETCH_MIN_MS;
          s_diag_relay_disco_resets++;
          p->last_ping_sent_ms = 0;
          p->last_pong_recv_ms = 0;
          p->last_cmm_sent_ms = 0;
          p->trust_until_ms = 0;
          p->direct_backoff_until = 0;
          p->relay_retry_next_ms = 1; /* fire the fresh CMM + ping next sweep tick */
          ESP_LOGW(
            TAG,
            "relay-stuck safety peer %s: disco-session reset v2 (timers+backoff cleared, best_ip kept)",
            p->hostname);
        }
      }
    }

    /* Heartbeat on active direct paths (every HEARTBEAT interval).
         * MUST use force=true because HEARTBEAT_MS (3s) < PING_INTERVAL_MS (5s),
         * so the rate limiter would always block heartbeat pings.
         * Heartbeats are NEVER throttled — they're time-critical for trust_until_ms. */
    uint64_t hb_ms = ml->t_disco_heartbeat_ms;
    if (is_priority && hb_ms > ML_DISCO_PRIORITY_HB_MS) {
      hb_ms = ML_DISCO_PRIORITY_HB_MS;
    }
    /* Per-peer deterministic phase offset (<=~1 s) de-aligns the fleet:
         * a shared trust-renewal moment (e.g. an AP blip) otherwise puts
         * every peer's next heartbeat in the same tick — at 100 peers
         * that's a multi-second NaCl seal burst. Capped per tick as well;
         * the priority peer is exempt from BOTH (its 3 s heartbeat is a
         * failover guarantee). */
    uint64_t hb_due = hb_ms + (uint64_t)((i * 271) % 997);
    if (p->has_direct_path && now - p->last_ping_sent_ms > (is_priority ? hb_ms : hb_due)) {
      if (is_priority || (hb_pings_sent < ML_HB_SEALS_PER_TICK)) {
        disco_send_ping_to_peer(ml, i, true);
        if (!is_priority) {
          hb_pings_sent++;
        }
      }
    }

    /* Sessionless handshake-retransmit cap (non-priority): the
         * trust-lease/demotion path re-arms peer->active, and wireguardif
         * then retries an initiation (~3 X25519-class ops) every 5 s
         * FOREVER for a peer that never answers. Dozens of flapping bulk
         * peers accumulate into a permanent double-digit CPU tax. Clear
         * `active` for non-priority peers with no established session —
         * the same one-shot pattern the first-direct-path handshake uses;
         * inbound initiations from the peer still establish sessions
         * regardless of this flag. The priority peer keeps unlimited
         * retries (its connectivity is the product). */
    /* AGGRAVATOR FIX (b): also exempt PINNED peers (fleet/OTA + app pins), not
     * just priority/health-tracked (is_priority). These few safety peers KEEP
     * `active` so wireguardif retransmits their handshake init between the 3 s
     * disco wakes — critical for a cross-region bond that only completes once an
     * aux DERP conn comes up. The ~120 non-pinned bulk peers still get the clear,
     * so the retry-storm armor (PEER_SCALING_DESIGN.md:39) is preserved. */
    if (!is_priority && !is_pinned_peer(ml, p->vpn_ip) && (ml->wg_netif != NULL) && (p->wg_peer_index >= 0)) {
      struct netif * nif = (struct netif *)ml->wg_netif;
      if (wireguardif_peer_is_up(nif, (u8_t)p->wg_peer_index, NULL, NULL) != ERR_OK) {
        struct wireguard_device * dev = (struct wireguard_device *)nif->state;
        if ((dev != NULL) && (p->wg_peer_index < WIREGUARD_MAX_PEERS) && dev->peers[p->wg_peer_index].active) {
          dev->peers[p->wg_peer_index].active = false;
        }
      }
    }
  }

  /* Advance rotating start index for next call */
  disco_probe_start_idx = (start + DISCO_PROBES_PER_TICK) % (ml->peer_count > 0 ? ml->peer_count : 1);

  /* Expire old pending probes.
     * MUST refresh 'now' because disco_send_ping_to_peer() above may have
     * registered probes with sent_ms NEWER than our stale 'now' from the top
     * of this function. Without refresh, now - sent_ms underflows to ~UINT64_MAX
     * which is always > PING_TIMEOUT_MS, causing immediate false expiry. */
  now = ml_get_time_ms();
  for (int i = 0; i < MAX_PENDING_PROBES; i++) {
    if (!pending_probes[i].active) continue;
    bool timed_out = now - pending_probes[i].sent_ms > ML_DISCO_PING_TIMEOUT_MS;
    /* Bounded post-DERP-match window (see disco_probe_t.derp_match_ms): a
     * via-DERP-answered probe waits only ML_DISCO_DERP_MATCH_GRACE_MS for its
     * slower direct twin, not the full 5 s timeout — unbounded lifetimes here
     * exhausted the table and demoted healthy paths (2026-08-09 bench). */
    bool grace_over =
      pending_probes[i].derp_match_ms != 0 && now - pending_probes[i].derp_match_ms > ML_DISCO_DERP_MATCH_GRACE_MS;
    if (timed_out || grace_over) {
      pending_probes[i].active = false;
      if (grace_over) {
        /* A direct pong may still straggle in; let it log as a benign
         * duplicate instead of a scary "unmatched" WARN. */
        remember_consumed_txid(pending_probes[i].txid);
      }
    }
  }
}

/* ============================================================================
 * WG Manager Task
 * ========================================================================== */

/* Self-heal: re-run the re-home against the pinned peers' CURRENT table state.
 * maybe_rehome_to_priority is event-driven (fires while ingesting a peer update),
 * so an ingestion-ordering race (region learned in a step that didn't re-invoke
 * it, or its update processed before the region was known) can leave the chip
 * homed on the wrong region even though the pinned peer's region is now known —
 * observed on a tethered unit: fleet_peer_region=2 (sfo) but derp_home_region=9
 * (dfw), so DERP relay (the only path behind a symmetric NAT) never worked.
 * Re-invoking maybe_rehome here is idempotent: it only re-homes when the target
 * region differs from the current home, so once corrected it stops (no churn). */
static void self_heal_rehome(microlink_t * ml)
{
  s_diag_selfheal_calls++;
  /* §7: consume any MBB outcome first — commit => advert owner + announce;
   * rollback => region ban. Runs here (3 s cadence, wg_mgr task) so the
   * negotiator stays single-writer and off the 10 Hz safety send path. */
  neg_consume_mbb_outcome(ml);
  /* Stage-1 pin->MBB: converge the operator region pin gaplessly (same
   * cadence, same single-writer command interface as autoneg). */
  neg_pin_tick(ml);
  /* Pinned-but-absent peer => force a coord full resync (f498 heal). */
  pin_presence_heal(ml);
  /* Heal against the DERIVED §4.2 primary (falls back to the legacy static
   * priority_peer_ip when no callback is registered — machn/plain builds). */
  uint32_t prim = neg_primary_ip(ml);
  if (prim != 0) {
    int i = find_peer_by_ip(ml, prim);
    if (i >= 0) maybe_rehome_to_priority(ml, &ml->peers[i]);
  }
  uint32_t fip = fleet_server_ip_cached();
  if (fip != 0 && fip != prim) {
    int i = find_peer_by_ip(ml, fip);
    if (i >= 0) maybe_rehome_to_priority(ml, &ml->peers[i]);
  }
  /* Phase-3 advice with an EMPTY peer table (idle chip, fleet peer not yet
   * ingested): neither invocation above fires, so evaluate advice directly.
   * Same gates as everywhere else (§6: no machine configured; TTL/epoch/
   * interval inside neg_advice_target; lock/damping inside neg_apply_target). */
  if (prim == 0 && !neg_machine_configured(ml)) {
    uint16_t a = neg_advice_target(ml, ml_get_time_ms());
    if (a != 0) {
      neg_apply_target(ml, a, MICROLINK_REGION_SRC_AUTO_FLEET);
    }
  }
}

void ml_wg_mgr_task(void * arg)
{
  microlink_t * ml = (microlink_t *)arg;
  ESP_LOGI(TAG, "WG Manager task started (Core %d)", xPortGetCoreID());

  /* Initialize probe tracking */
  memset(pending_probes, 0, sizeof(pending_probes));

  /* Load cached peers from NVS for fast boot */
  int cached = ml_peer_nvs_load_all(ml->peers, ML_MAX_PEERS);
  if (cached > 0) {
    ml->peer_count = cached;
    ESP_LOGI(TAG, "Pre-loaded %d cached peers from NVS", cached);
  }

  /* Cold-bond far-side recovery (bench 2026-08-08): a rebooted machine whose
     * coordination netmap hasn't re-arrived used to SILENTLY ignore WG
     * handshake initiations from its operator remotes (their static keys were
     * only installed on netmap ingest) — the remote wedged at
     * peer-present-but-no-keypair (ENOTCONN, sent=0) for 27 min and only a
     * machine restart recovered it. If our VPN IP is known from NVS (persisted
     * on every registration), bring the WG interface up NOW and preseed the
     * cached peers into wireguard-lwip so those inbound handshakes succeed
     * immediately — before, or entirely without, coordination connectivity. */
  bool wg_up_early = false;
  if (ml->vpn_ip != 0 && cached > 0) {
    if (wg_init_interface(ml) == ESP_OK) {
      wg_up_early = true;
      xEventGroupSetBits(ml->events, ML_EVT_WG_READY);
      int installed = 0;
      for (int i = 0; i < ml->peer_count; i++) {
        if (!ml->peers[i].active) continue;
        /* Same allowlist gate as add_peer — the allowlist may have changed
                 * since these entries were cached. */
        if (!ml_config_peer_is_allowed(ml->config_httpd, ml->peers[i].vpn_ip)) continue;
        wg_install_iface_peer(ml, i);
        if (ml->peers[i].wg_peer_index >= 0) installed++;
      }
      ESP_LOGI(TAG, "WG preseeded from NVS: %d/%d cached peers installed (pre-registration)", installed, cached);
    }
  }

  /* Wait for registration OR shutdown before entering the full loop — but
     * when WG came up early, keep draining inbound WG packets while waiting:
     * a same-LAN operator remote can complete its handshake and run pstop
     * heartbeats against us during this window even with the coordination
     * plane unreachable (the exact post-reboot wedge scenario above). */
  for (;;) {
    EventBits_t wait_bits = xEventGroupWaitBits(
      ml->events,
      ML_EVT_COORD_REGISTERED | ML_EVT_SHUTDOWN_REQUEST,
      pdFALSE,
      pdFALSE,
      wg_up_early ? pdMS_TO_TICKS(20) : portMAX_DELAY);

    if (wait_bits & ML_EVT_SHUTDOWN_REQUEST) {
      ESP_LOGI(TAG, "Shutdown requested before registration, exiting");
      vTaskDelete(NULL);
      return; /* Not reached */
    }
    if (wait_bits & ML_EVT_COORD_REGISTERED) {
      break;
    }
    if (wg_up_early) {
      s_wg_hs_budget = ML_WG_HANDSHAKES_PER_PASS; /* own pass in this wait loop */
      wg_mgr_drain_wg_rx(ml);
    }
  }

  ESP_LOGI(TAG, "Coord registered, initializing WireGuard...");

  if (!wg_up_early) {
    /* Initialize WireGuard interface (magicsock mode) */
    if (wg_init_interface(ml) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init WireGuard, continuing without tunneling");
    } else {
      /* Update VPN IP if coord already set it */
      wg_update_vpn_ip(ml);
      xEventGroupSetBits(ml->events, ML_EVT_WG_READY);
    }
  } else {
    /* Registration may have (re)assigned our VPN IP — sync the netif so a
         * stale persisted address can't linger past this point. */
    wg_update_vpn_ip(ml);
  }

  ESP_LOGI(TAG, "Accepting peer updates");

  uint64_t last_disco_probe_ms = 0;
  uint64_t last_wg_periodic_ms = 0;
  uint64_t last_rehome_check_ms = 0;
  bool derp_was_connected = false;
  bool stun_cmm_sent = false; /* One-shot: send CMMs after first STUN result */

  while (!(xEventGroupGetBits(ml->events) & ML_EVT_SHUTDOWN_REQUEST)) {
    uint64_t wg_iter_t0 = ml_get_time_ms(); /* Stage-0b gauge */
    /* WG packets FIRST: this queue carries the pstop heartbeats. Budget and
     * per-pass ring counters reset here; every drain site this pass shares
     * the same handshake budget. */
    s_wg_hs_budget = ML_WG_HANDSHAKES_PER_PASS;
    s_pass_wg_pkts = 0;
    s_pass_wg_handshakes = 0;
    s_pass_peer_adds = 0;
    s_pass_disco_opens = 0;
    s_pass_periodic_ms = 0;
    s_pass_probe_ms = 0;
    s_pass_cmm_sends = 0;
    s_pass_nvs_flush_ms = 0;
    wg_mgr_drain_wg_rx(ml);

    /* Process peer updates from coord task (paced) */
    process_peer_updates(ml);

    /* Debounced peer-cache flash flush (see ml_peer_nvs.c). */
    {
      uint64_t f0 = ml_get_time_ms();
      (void)ml_peer_nvs_flush_if_due(f0, uxQueueMessagesWaiting(ml->peer_update_queue) > 0);
      uint64_t fd = ml_get_time_ms() - f0;
      s_pass_nvs_flush_ms = (fd > 0xFFFF) ? 0xFFFF : (uint16_t)fd;
    }

    /* Track DERP connection state for DISCO.
         * Note: We DON'T re-initiate WG handshakes on DERP connect because
         * Tailscale peers use lazy config and would drop our initiations.
         * WG sessions are established on-demand when peers initiate to us. */
    {
      EventBits_t bits = xEventGroupGetBits(ml->events);
      bool derp_connected_now = (bits & ML_EVT_DERP_CONNECTED) != 0;
      if (derp_connected_now && !derp_was_connected) {
        ESP_LOGI(TAG, "DERP connected, %d peers ready for incoming handshakes", ml->peer_count);
      }
      derp_was_connected = derp_connected_now;
    }

    /* After STUN completes, broadcast CallMeMaybe to all peers.
         * Peers need to know our public endpoint (from STUN) to send direct
         * probes. Without this, our initial CMMs during peer-add have 0
         * endpoints because STUN hasn't finished yet. */
    if (!stun_cmm_sent && ml->stun_public_ip != 0 && ml->peer_count > 0) {
      /* Paced: each CMM is a NaCl seal (~30 ms) and solicits a ping
             * back — broadcasting to 100 peers in one pass both stalled
             * this loop for seconds and triggered an inbound box-open
             * storm. Send a couple per 10 ms pass instead. */
      static int cmm_resume_idx;
      int sent_this_pass = 0;
      while ((cmm_resume_idx < ml->peer_count) && (sent_this_pass < ML_CMM_SENDS_PER_PASS)) {
        ml_peer_t * cp = &ml->peers[cmm_resume_idx];
        if (cp->active && !cp->has_direct_path) {
          disco_send_call_me_maybe(ml, cmm_resume_idx);
          sent_this_pass++;
        }
        cmm_resume_idx++;
      }
      if (cmm_resume_idx >= ml->peer_count) {
        stun_cmm_sent = true;
        ESP_LOGI(TAG, "STUN complete — CallMeMaybe broadcast finished (%d peers)", ml->peer_count);
      }
    }

    /* Process DISCO packets */
#ifdef CONFIG_ML_ZERO_COPY_WG
    /* Zero-copy mode: drain SPSC ring buffer (PCB callback → wg_mgr) */
    {
      uint8_t tail = __atomic_load_n(&ml->zc.rx_tail, __ATOMIC_RELAXED);
      uint8_t head = __atomic_load_n(&ml->zc.rx_head, __ATOMIC_ACQUIRE);
      while (tail != head) {
        ml_zc_disco_entry_t * entry = &ml->zc.rx_ring[tail];
        ml_rx_packet_t disco_pkt = {
          .data = entry->data,
          .len = entry->len,
          .src_ip = ntohl(entry->src_ip_nbo),
          .src_port = entry->src_port,
          .via_derp = false,
        };
        process_disco_packet(ml, &disco_pkt);
        /* Don't free — data is in the ring buffer, not heap-allocated */
        tail = (tail + 1) % ML_ZC_DISCO_RING_SIZE;
        head = __atomic_load_n(&ml->zc.rx_head, __ATOMIC_ACQUIRE);
      }
      __atomic_store_n(&ml->zc.rx_tail, tail, __ATOMIC_RELEASE);
    }
#endif
    /* Queue-based path: DISCO from DERP relay + fallback when zero-copy
         * disabled. Budgeted: each non-priority packet may cost a ~30 ms
         * NaCl box-open (the DH is recomputed every time), and the
         * 2026-07-20 rotation-rescue admits unknown keys to that cost —
         * on a 100-peer tailnet an unbounded drain is a multi-second
         * stall. Excess stays queued for the next 10 ms pass; sustained
         * overflow sheds at the net_io edge by design. */
    int disco_budget = ML_DISCO_OPENS_PER_PASS;
    ml_rx_packet_t disco_pkt;
    while ((disco_budget > 0) && (xQueueReceive(ml->disco_rx_queue, &disco_pkt, 0) == pdTRUE)) {
      /* Only UNKNOWN-key packets consume budget: they cost a full X25519.
             * Known-peer packets (cached DH, ~1 ms) flow freely, so pstop
             * counterparts' pings/pongs never wait behind tailnet chatter. */
      bool unknown_key = !disco_pkt_known_key(ml, &disco_pkt);
      if (unknown_key) {
        disco_budget--;
      }
      process_disco_packet(ml, &disco_pkt);
      free(disco_pkt.data);
      if (unknown_key) {
        /* That box-open was a full X25519 (~30 ms). Interleave any pending
                 * safety-heartbeat decrypt now so it isn't starved behind a
                 * burst of unknown-key DISCO opens. */
        wg_mgr_drain_wg_rx(ml);
      }
    }
    s_pass_disco_opens = (uint16_t)(ML_DISCO_OPENS_PER_PASS - disco_budget);

    /* WG packets again — a full disco budget may have taken ~120 ms;
         * don't make fresh heartbeats wait for the periodics below. */
    wg_mgr_drain_wg_rx(ml);

    /* Run WireGuard periodic processing (handshakes, keepalives, rekeys).
         * This runs on OUR task stack (8KB) instead of the lwIP TCPIP thread (3-8KB),
         * preventing heavy crypto (X25519, ChaCha20-Poly1305) from monopolizing
         * the TCPIP thread and blocking all socket operations system-wide. */
    uint64_t now = ml_get_time_ms();
    if (ml->wg_netif && now - last_wg_periodic_ms >= 400) {
      uint64_t t0 = now;
      wireguardif_periodic((struct netif *)ml->wg_netif);
      uint64_t dt = ml_get_time_ms() - t0;
      last_wg_periodic_ms = now;
      ESP_LOGI(TAG, "wireguardif_periodic: %llu ms", (unsigned long long)dt);
      s_pass_periodic_ms = (dt > 0xFFFF) ? 0xFFFF : (uint16_t)dt;
    }

    /* Periodic DISCO probes (every 1s check) */
    now = ml_get_time_ms();
    if (now - last_disco_probe_ms > 1000) {
      uint64_t t0 = now;
      disco_periodic_probes(ml);
      uint64_t dt = ml_get_time_ms() - t0;
      last_disco_probe_ms = now;
      ESP_LOGI(TAG, "disco_periodic_probes: %llu ms", (unsigned long long)dt);
      s_pass_probe_ms = (dt > 0xFFFF) ? 0xFFFF : (uint16_t)dt;
    }

    /* Self-heal the DERP home region every 3s: if a pinned peer's region is
         * known but we're homed elsewhere (ingestion-race), correct it. Cheap
         * (two peer-table lookups) and idempotent once homed. */
    now = ml_get_time_ms();
    if (now - last_rehome_check_ms > 3000) {
      self_heal_rehome(ml);
      last_rehome_check_ms = now;
    }

    /* Stage-0b gauge: worst single iteration (heartbeat-path stall detector),
     * plus a per-EVENT ring entry so a stall self-attributes to the work
     * that caused it (the high-water gauge is blind under its own mark). */
    {
      uint32_t wg_iter_ms = (uint32_t)(ml_get_time_ms() - wg_iter_t0);
      if (wg_iter_ms > s_diag_wg_max_iter_ms) s_diag_wg_max_iter_ms = wg_iter_ms;
      if (wg_iter_ms > ML_STALL_THRESH_MS) {
        uint32_t w = s_wg_stall_widx;
        ml_wg_stall_event_t * e = &s_wg_stall_ring[w % ML_STALL_RING_LEN];
        e->at_s = (uint32_t)(wg_iter_t0 / 1000);
        e->dur_ms = wg_iter_ms;
        e->wg_pkts = s_pass_wg_pkts;
        e->handshakes = s_pass_wg_handshakes;
        e->peer_adds = s_pass_peer_adds;
        e->disco_opens = s_pass_disco_opens;
        e->periodic_ms = s_pass_periodic_ms;
        e->disco_probe_ms = s_pass_probe_ms;
        e->cmm_sends = s_pass_cmm_sends;
        e->nvs_flush_ms = s_pass_nvs_flush_ms;
        __atomic_store_n(&s_wg_stall_widx, w + 1, __ATOMIC_RELEASE);
        ESP_LOGW(
          TAG,
          "WG loop stall %ums (hs=%u adds=%u opens=%u periodic=%ums)",
          (unsigned)wg_iter_ms,
          e->handshakes,
          e->peer_adds,
          e->disco_opens,
          e->periodic_ms);
      }
    }

    /* Yield - 10ms loop rate for minimum packet processing latency.
         * Each wake is cheap: queue check + event bits check, no crypto. */
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  /* Shutdown WireGuard interface (under TCPIP core lock per PR#14 audit) */
  if (ml->wg_netif) {
    struct netif * netif = (struct netif *)ml->wg_netif;
    bool sd_need_lock = ml_lwip_core_lock_needed();
    if (sd_need_lock) {
      LOCK_TCPIP_CORE();
    }
    wireguardif_shutdown(netif);
    netif_set_link_down(netif);
    netif_set_down(netif);
    netif_remove(netif);
    if (s_wg_output_pcb) {
      udp_remove(s_wg_output_pcb);
      s_wg_output_pcb = NULL;
    }
    if (sd_need_lock) {
      UNLOCK_TCPIP_CORE();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    free(netif);
    ml->wg_netif = NULL;
  }

  ESP_LOGI(TAG, "WG Manager task exiting");
  vTaskDelete(NULL);
}
