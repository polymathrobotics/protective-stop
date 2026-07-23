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
  int peer_index;
  bool active;
} disco_probe_t;

#define MAX_PENDING_PROBES 64
static disco_probe_t pending_probes[MAX_PENDING_PROBES];

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
  if (!ml || !ml->derp.connected) {
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

  /* On cellular AT socket bridge, force all WG output through DERP relay.
     * AT sockets are TCP-only, so direct UDP is impossible.
     * PPP mode has real lwIP sockets with UDP, so allow direct connections. */
#if CONFIG_ML_ENABLE_CELLULAR
  {
    bool at_ready = ml_at_socket_is_ready();
    ESP_LOGI(TAG, "Cellular mode: at_socket_ready=%d, force_derp=%d", at_ready, at_ready);
    if (at_ready) {
      wireguardif_force_derp_output(netif, true);
      ESP_LOGI(TAG, "Cellular AT socket: forcing DERP output (direct UDP disabled)");
    } else {
      ESP_LOGI(TAG, "Cellular PPP mode: direct UDP ENABLED");
    }
  }
#endif

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
 * (safety) peer — the machine — and the fleet coordination/OTA server. On a
 * large tailnet (100s of peers, trimmed to ML_MAX_PEERS) either could
 * otherwise fall out of the table, leaving the chip unable to reach the
 * machine (unsafe) or the fleet (no OTA/coordination). */
static bool is_pinned_peer(microlink_t * ml, uint32_t vpn_ip)
{
  if (vpn_ip == 0) return false;
  if (ml->config.priority_peer_ip != 0 && vpn_ip == ml->config.priority_peer_ip) {
    return true;
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

/* Diagnostic: report the DERP region the chip has LEARNED for the fleet and
 * priority peers (0 = not learned yet). Distinguishes "never learned the fleet's
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

/* Home our single DERP connection on a pinned peer's region.
 * microlink keeps ONE DERP connection, and a Tailscale DERP server only
 * delivers to peers connected to that same server — so if the chip homes on a
 * different region than a peer it must reach, its relayed DISCO/WG-init never
 * arrives and the link only forms after a `tailscale ping` FROM that peer.
 * Home on the priority (safety) peer's region when known; otherwise fall back
 * to the fleet server's region (the always-on anchor) so the chip can still be
 * managed/updated when no machine is online. The chosen region is reported as
 * PreferredDERP in the next MapRequest (ml_coord) so peers find us there too.
 * Called wherever a pinned peer's derp_region is (re)learned. */
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
  bool is_prio = (ml->config.priority_peer_ip != 0 && p->vpn_ip == ml->config.priority_peer_ip);
  uint16_t target = p->derp_region;
  if (!is_prio && ml->config.priority_peer_ip != 0) {
    /* Fleet peer: prefer the priority peer's region if we know it. */
    int pidx = find_peer_by_ip(ml, ml->config.priority_peer_ip);
    if (pidx >= 0 && ml->peers[pidx].derp_region != 0) {
      target = ml->peers[pidx].derp_region;
    }
  }
  ml->priority_peer_region = target; /* advertised as PreferredDERP */
  if (ml->derp_home_region != target) {
    s_diag_rehome_applied++;
    ESP_LOGW(
      TAG,
      "Re-homing DERP region %u -> %u (%s)",
      (unsigned)ml->derp_home_region,
      (unsigned)target,
      is_prio ? "priority peer" : "fleet server");
    ml->derp_home_region = target;
    xEventGroupSetBits(ml->events, ML_EVT_DERP_RECONNECT);
  }
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

static int add_peer(microlink_t * ml, const ml_peer_update_t * update)
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

    /* Find free slot */
    idx = -1;
    for (int i = 0; i < eff_max; i++) {
      if (!ml->peers[i].active) {
        idx = i;
        break;
      }
    }

    /* Peer table full — evict LRU non-pinned peer if the incoming peer is
         * pinned (priority/safety peer OR fleet coordination server). */
    if (idx < 0 && is_pinned_peer(ml, update->vpn_ip)) {
      uint64_t oldest_ms = UINT64_MAX;
      int evict_idx = -1;
      for (int i = 0; i < ML_MAX_PEERS; i++) {
        if (!ml->peers[i].active) continue;
        if (is_pinned_peer(ml, ml->peers[i].vpn_ip)) continue;
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
  p->vpn_ip = update->vpn_ip;
  memcpy(p->public_key, update->public_key, 32);
  memcpy(p->disco_key, update->disco_key, 32);
  strncpy(p->hostname, update->hostname, sizeof(p->hostname) - 1);
  p->hostname[sizeof(p->hostname) - 1] = '\0';
  /* Preserve a previously-learned DERP region. Full peer re-syncs frequently
     * re-add a peer with derp_region=0 (the region is carried separately, via
     * PeersChangedPatch), and unconditionally overwriting wiped the pinned
     * peer's known region back to 0 — so maybe_rehome() below early-returned and
     * the chip never stayed homed on the fleet's region. Behind a symmetric NAT
     * (USB tether) DERP relay is the only path, so this left the unit
     * permanently unreachable. Only overwrite when the update actually carries a
     * region. */
  if (update->derp_region != 0) {
    p->derp_region = update->derp_region;
  }
  p->active = true;
  maybe_rehome_to_priority(ml, p);

  /* Copy endpoints */
  p->endpoint_count = update->endpoint_count;
  for (int i = 0; i < update->endpoint_count && i < ML_MAX_ENDPOINTS; i++) {
    p->endpoints[i].ip = update->endpoints[i].ip;
    p->endpoints[i].port = update->endpoints[i].port;
    p->endpoints[i].is_ipv6 = update->endpoints[i].is_ipv6;
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
  if (ml->wg_netif && (!existing || p->wg_peer_index < 0)) {
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

    /* Set endpoint if available, otherwise leave blank for DERP-only */
    if (p->endpoint_count > 0 && p->endpoints[0].ip != 0) {
      uint8_t ea = (p->endpoints[0].ip >> 24) & 0xFF;
      uint8_t eb = (p->endpoints[0].ip >> 16) & 0xFF;
      uint8_t ec = (p->endpoints[0].ip >> 8) & 0xFF;
      uint8_t ed = p->endpoints[0].ip & 0xFF;
      IP4_ADDR(&wg_peer.endpoint_ip.u_addr.ip4, ea, eb, ec, ed);
      wg_peer.endport_port = p->endpoints[0].port;
    } else {
      ip_addr_set_any(false, &wg_peer.endpoint_ip);
      wg_peer.endport_port = 0;
    }

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
      if (ml->config.priority_peer_ip != 0 && update->vpn_ip == ml->config.priority_peer_ip) {
        wireguardif_connect_derp(netif, (u8_t)wg_peer_idx);
        /* Wake the remote's magicsock so it lazily adds us to
                 * wireguard-go. A bare WG-init relayed over DERP does NOT
                 * drive the peer's lazy reconfig — only an inbound DISCO
                 * ping does (that is why a normal peer otherwise has to
                 * `tailscale ping` us first). Fire one now, unconditionally
                 * (bypasses the enable_disco/cellular gates on the paths
                 * below); it is DERP-carried and embeds our node key, so a
                 * peer with a stale/rotated disco key can still map us. The
                 * sustained retry lives in disco_periodic_probes. */
        disco_send_ping_to_peer(ml, idx, true);
        /* Near-zero failover: mirror this peer's encrypted data over
                 * BOTH direct and DERP once a direct path forms. Set here and
                 * persisted in the WG peer struct (survives P7 re-ingests); it
                 * only takes effect while a direct endpoint exists, so it's a
                 * no-op until DISCO upgrades the path. Skip on cellular
                 * (direct paths impossible; DERP is already the only path). */
        if (!ml_at_socket_is_ready()) {
          wireguardif_set_dual_path(netif, (u8_t)wg_peer_idx, true);
        }
        ESP_LOGI(TAG, "WG: triggered DERP handshake to priority peer %s (dual-path)", update->hostname);
      } else {
        ESP_LOGI(TAG, "WG peer ready (passive), waiting for peer-initiated handshake");
      }
    } else {
      ESP_LOGW(TAG, "wireguardif_add_peer failed: %d", wg_err);
      p->wg_peer_index = -1;
    }
  }

  /* Persist to NVS for fast boot next time */
  ml_peer_nvs_save(p);

  /* Send CallMeMaybe to trigger peer-initiated handshake (NAT traversal).
     * Skip on cellular: our endpoints are behind carrier-grade NAT and
     * unreachable — all traffic goes through DERP relay. */
  if (!ml_at_socket_is_ready()) {
    disco_send_call_me_maybe(ml, idx);
  }

  /* Immediately probe known endpoints (throttled to avoid flooding with 100s of peers).
     * Only force-ping if fewer than 5 peers added in the last second;
     * the periodic probe (every 15s) will handle the rest.
     * Skip on cellular: direct probes fill DERP TX queue (~0.6s each on AT socket),
     * blocking time-critical WG handshake responses. */
  if (!ml_at_socket_is_ready()) {
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
    strncpy(info.hostname, p->hostname, sizeof(info.hostname) - 1);
    memcpy(info.public_key, p->public_key, 32);
    ml->peer_cb(ml, &info, ml->peer_cb_data);
  }

  return idx;
}

static void remove_peer(microlink_t * ml, const ml_peer_update_t * update)
{
  int idx = find_peer_by_key(ml, update->public_key);
  if (idx < 0) return;

  /* Remove from wireguard-lwip */
  if (ml->wg_netif && ml->peers[idx].wg_peer_index >= 0) {
    struct netif * netif = (struct netif *)ml->wg_netif;
    wireguardif_remove_peer(netif, (u8_t)ml->peers[idx].wg_peer_index);
  }

  char ip_str[16];
  microlink_ip_to_str(ml->peers[idx].vpn_ip, ip_str);
  ESP_LOGI(TAG, "Peer removed: %s (%s)", ml->peers[idx].hostname, ip_str);

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
      case ML_PEER_ADD:
        add_peer(ml, update);
        /* Each add costs an X25519 (~30 ms). Ingesting a 100-peer
             * MapResponse in one pass stalled the loop for ~10 s while the
             * pstop RX queue overflowed — pace it and let the 10 ms loop
             * come back for the rest. */
        adds_this_pass++;
        if (adds_this_pass >= ML_PEER_ADDS_PER_PASS) {
          paced_out = true;
        }
        break;
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
      pending_probes[i].active = true;
      registered = true;
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

  uint8_t pkt[256];
  size_t pkt_len = 0;
  disco_build_ping(ml, peer_idx, pkt, &pkt_len);

  if (pkt_len == 0) return;

  bool direct_sent = false;

  /* If we have a known working direct path, send there FIRST.
     * This is critical for heartbeat pings to renew trust_until_ms.
     * A DERP pong would arrive with via_derp=true and NOT renew trust. */
  if (p->has_direct_path && p->best_ip != 0 && p->best_port != 0) {
    disco_udp_sendto(ml, pkt, pkt_len, p->best_ip, p->best_port);
    direct_sent = true;
  }

  /* Also try direct UDP to all known endpoints from MapResponse */
  {
    bool has_udp = disco_has_udp_path(ml);
    if (has_udp) {
      for (int i = 0; i < p->endpoint_count; i++) {
        if (!p->endpoints[i].is_ipv6 && p->endpoints[i].ip != 0) {
          /* Skip if same as best_ip (already sent) */
          if (p->endpoints[i].ip == p->best_ip && p->endpoints[i].port == p->best_port) continue;
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

    /* If direct reply, update best path */
    if (!pkt->via_derp && pkt->src_ip != 0) {
      p->best_ip = pkt->src_ip;
      p->best_port = pkt->src_port;
      p->has_direct_path = true;
      p->trust_until_ms = now + ML_DISCO_TRUST_DURATION_MS;

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
    }

    pending_probes[i].active = false;
    matched = true;
    break;
  }

  if (!matched) {
    /* Find peer by disco key for logging */
    int peer_idx = find_peer_by_disco_key(ml, sender_disco_key);
    const char * name = peer_idx >= 0 ? ml->peers[peer_idx].hostname : "?";
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
        "CallMeMaybe from %s: %d endpoints (udp_path=%d, at_sock=%d)",
        ml->peers[peer_idx].hostname,
        ep_count,
        disco_has_udp_path(ml),
        ml_at_socket_is_ready());

      /* Reply with our own CallMeMaybe (bidirectional NAT traversal).
             * Skip on cellular: our endpoints are behind CGNAT and unreachable. */
      if (!ml_at_socket_is_ready()) {
        disco_send_call_me_maybe(ml, peer_idx);
      }

      /* Probe each endpoint with a DISCO ping.
             * Skip on cellular: direct UDP impossible, saves TX queue capacity. */
      for (int i = 0; !ml_at_socket_is_ready() && i < ep_count && i < ML_MAX_ENDPOINTS; i++) {
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

        ESP_LOGI(
          TAG,
          "  CMM ep[%d]: v4mapped=%d port=%d bytes=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
          i,
          is_v4_mapped,
          port,
          entry[0],
          entry[1],
          entry[2],
          entry[3],
          entry[4],
          entry[5],
          entry[6],
          entry[7],
          entry[8],
          entry[9],
          entry[10],
          entry[11],
          entry[12],
          entry[13],
          entry[14],
          entry[15]);

        if (!is_v4_mapped || port == 0) continue;

        uint32_t ip =
          ((uint32_t)entry[12] << 24) | ((uint32_t)entry[13] << 16) | ((uint32_t)entry[14] << 8) | (uint32_t)entry[15];

        /* Send DISCO ping to this endpoint */
        if (disco_has_udp_path(ml)) {
          uint8_t ping_pkt[256];
          size_t ping_len = 0;
          disco_build_ping(ml, peer_idx, ping_pkt, &ping_len);

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
        } else {
          ESP_LOGW(TAG, "CMM probe skipped: no UDP path (sock4=%d)", ml->disco_sock4);
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

  /* Build plaintext: [type(1)][version(1)][endpoints(N * 18)] */
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

/* Public wrapper for UDP API to trigger CallMeMaybe.
 * Skip on cellular: our endpoints are behind CGNAT and unreachable. */
void ml_wg_mgr_send_cmm(microlink_t * ml, uint32_t peer_vpn_ip)
{
  if (ml_at_socket_is_ready()) return; /* cellular: CMM useless */
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
#if CONFIG_ML_ENABLE_CELLULAR
  if (!ml || !ml->wg_netif) return;
  struct netif * netif = (struct netif *)ml->wg_netif;
  bool at_ready = ml_at_socket_is_ready();
  wireguardif_force_derp_output(netif, at_ready);
  ESP_LOGI(TAG, "WG transport updated: force_derp=%d (%s)", at_ready, at_ready ? "AT socket" : "PPP/WiFi");
#else
  (void)ml;
#endif
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
  /* Pinned-peer session wakes — run BEFORE the enable_disco/cellular gates so
     * the safety peer AND the fleet coordination/OTA server always (re)establish
     * their WG session disco-first, even on a busy tailnet where general disco is
     * throttled. Without this the fleet can be in the peer table yet have no live
     * session (check-in silently fails). */
  disco_wake_peer(ml, ml->config.priority_peer_ip, ml->priority_link_healthy);
  disco_wake_peer(ml, fleet_server_ip_cached(), ml->fleet_link_healthy);

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

    bool is_priority = (ml->config.priority_peer_ip != 0 && p->vpn_ip == ml->config.priority_peer_ip);

    /* Direct-path health. Two triggers:
         *  - trust-lease expiry (all peers, the original behaviour), and
         *  - pong-recency watchdog (priority peer only): liveness is what
         *    last_pong_recv_ms already tracks, but demotion used to rest on
         *    the fixed 60 s lease alone — a dead direct path went dark for
         *    10-60 s (measured) before failover. With the 3 s priority
         *    heartbeat below, two missed pongs demote in ~8-9 s. */
    bool lease_expired = p->has_direct_path && now > p->trust_until_ms;
    bool pong_dead = p->has_direct_path && is_priority && p->last_pong_recv_ms != 0 &&
                     now - p->last_pong_recv_ms > ML_DISCO_PRIORITY_PATH_DEAD_MS;
    if (lease_expired || pong_dead) {
      ESP_LOGI(
        TAG,
        "Direct path to %s %s, reverting to DERP",
        p->hostname,
        pong_dead ? "went silent (pong watchdog)" : "expired");
      p->has_direct_path = false;

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

        /* Force-ping to try re-establishing direct path (WiFi only) */
        if (!ml_at_socket_is_ready()) {
          disco_send_ping_to_peer(ml, i, true);
        }
      }
    }

    if (!peer_allowed) continue;

    /* Probe for direct path upgrade (every UPGRADE_INTERVAL when on DERP).
         * Skip on cellular: direct paths impossible through carrier-grade NAT.
         * Throttled to DISCO_PROBES_PER_TICK to spread load and reduce jitter. */
    uint64_t upgrade_interval = is_priority ? ML_DISCO_PRIORITY_REPROBE_MS : ML_DISCO_UPGRADE_INTERVAL_MS;
    if (!ml_at_socket_is_ready() && !p->has_direct_path && now - p->last_upgrade_ms > upgrade_interval) {
      if (upgrade_probes_sent < DISCO_PROBES_PER_TICK) {
        disco_send_ping_to_peer(ml, i, false);
        p->last_upgrade_ms = now;
        upgrade_probes_sent++;
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
    if (!is_priority && (ml->wg_netif != NULL) && (p->wg_peer_index >= 0)) {
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
    if (pending_probes[i].active && now - pending_probes[i].sent_ms > ML_DISCO_PING_TIMEOUT_MS) {
      pending_probes[i].active = false;
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
  if (ml->config.priority_peer_ip != 0) {
    int i = find_peer_by_ip(ml, ml->config.priority_peer_ip);
    if (i >= 0) maybe_rehome_to_priority(ml, &ml->peers[i]);
  }
  uint32_t fip = fleet_server_ip_cached();
  if (fip != 0) {
    int i = find_peer_by_ip(ml, fip);
    if (i >= 0) maybe_rehome_to_priority(ml, &ml->peers[i]);
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

  /* Wait for registration OR shutdown before proceeding */
  EventBits_t wait_bits =
    xEventGroupWaitBits(ml->events, ML_EVT_COORD_REGISTERED | ML_EVT_SHUTDOWN_REQUEST, pdFALSE, pdFALSE, portMAX_DELAY);

  if (wait_bits & ML_EVT_SHUTDOWN_REQUEST) {
    ESP_LOGI(TAG, "Shutdown requested before registration, exiting");
    vTaskDelete(NULL);
    return; /* Not reached */
  }

  ESP_LOGI(TAG, "Coord registered, initializing WireGuard...");

  /* Initialize WireGuard interface (magicsock mode) */
  if (wg_init_interface(ml) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init WireGuard, continuing without tunneling");
  } else {
    /* Update VPN IP if coord already set it */
    wg_update_vpn_ip(ml);
    xEventGroupSetBits(ml->events, ML_EVT_WG_READY);
  }

  ESP_LOGI(TAG, "Accepting peer updates");

  uint64_t last_disco_probe_ms = 0;
  uint64_t last_wg_periodic_ms = 0;
  uint64_t last_rehome_check_ms = 0;
  bool derp_was_connected = false;
  bool stun_cmm_sent = false; /* One-shot: send CMMs after first STUN result */

  while (!(xEventGroupGetBits(ml->events) & ML_EVT_SHUTDOWN_REQUEST)) {
    /* WG packets FIRST: this queue carries the 10 Hz pstop heartbeats.
         * It used to be drained after all disco crypto and peer ingestion,
         * so any burst of ~30 ms box-opens delayed (or overflowed) the
         * safety traffic. */
    {
      ml_rx_packet_t wg_pkt_first;
      while (xQueueReceive(ml->wg_rx_queue, &wg_pkt_first, 0) == pdTRUE) {
        process_wg_packet(ml, &wg_pkt_first);
      }
    }

    /* Process peer updates from coord task (paced) */
    process_peer_updates(ml);

    /* Debounced peer-cache flash flush (see ml_peer_nvs.c). */
    (void)ml_peer_nvs_flush_if_due(ml_get_time_ms());

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
    if (!stun_cmm_sent && !ml_at_socket_is_ready() && ml->stun_public_ip != 0 && ml->peer_count > 0) {
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
      if (!disco_pkt_from_priority(ml, &disco_pkt)) {
        disco_budget--;
      }
      process_disco_packet(ml, &disco_pkt);
      free(disco_pkt.data);
    }

    /* WG packets again — a full disco budget may have taken ~120 ms;
         * don't make fresh heartbeats wait for the periodics below. */
    ml_rx_packet_t wg_pkt;
    while (xQueueReceive(ml->wg_rx_queue, &wg_pkt, 0) == pdTRUE) {
      process_wg_packet(ml, &wg_pkt);
    }

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
    }

    /* Periodic DISCO probes (every 1s check) */
    now = ml_get_time_ms();
    if (now - last_disco_probe_ms > 1000) {
      uint64_t t0 = now;
      disco_periodic_probes(ml);
      uint64_t dt = ml_get_time_ms() - t0;
      last_disco_probe_ms = now;
      ESP_LOGI(TAG, "disco_periodic_probes: %llu ms", (unsigned long long)dt);
    }

    /* Self-heal the DERP home region every 3s: if a pinned peer's region is
         * known but we're homed elsewhere (ingestion-race), correct it. Cheap
         * (two peer-table lookups) and idempotent once homed. */
    now = ml_get_time_ms();
    if (now - last_rehome_check_ms > 3000) {
      self_heal_rehome(ml);
      last_rehome_check_ms = now;
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
