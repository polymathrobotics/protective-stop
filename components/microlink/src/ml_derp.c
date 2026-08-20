// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_derp.c
 * @brief DERP relay client: multi-region connection pool + single I/O task.
 *
 * Invariant: the DERP I/O task is the ONLY task that touches the pool's
 * sockets, TLS contexts and connect state — no mutex by construction.
 * Cross-task readers use word-sized volatile fields only.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mbedtls/error.h"
#include "mbedtls/net_sockets.h"
#include "microlink_internal.h"
#include "nacl_box.h"

/* Runtime-tunable inner-loop yield. 1 ms = original behaviour (high CPU,
 * lowest tail latency on TX queue drain). Bigger values cut DERP_TX CPU
 * roughly linearly at the cost of TX/RX responsiveness. Set via
 * microlink_set_derp_loop_delay_ms(). */
static atomic_int s_derp_loop_delay_ms = 1;

/* Runtime pause flag. When set, the DERP I/O task skips its loop body each
 * iteration (just a 100 ms yield + continue), so DERP idles but the task
 * remains alive and resumable. Safer than vTaskSuspend which can leave
 * mutexes / sockets in a held state. */
static atomic_int s_derp_paused = 0;

static const char * TAG = "ml_derp";

static uint32_t s_diag_wg_rx_drops; /* wg_rx_queue full at THIS producer (edge shed) */
static uint16_t s_diag_last_dns_ms; /* most recent blocking DNS resolve duration */
static uint16_t s_pass_rx_gap; /* rx-poll gap measured this pass (ring input) */

uint32_t ml_derp_get_wg_rx_drops(void)
{
  return s_diag_wg_rx_drops;
}

/* DERP-task stall-event ring (same pattern as the wg_mgr ring). */
static ml_derp_stall_event_t s_derp_stall_ring[ML_STALL_RING_LEN];
static volatile uint32_t s_derp_stall_widx;

int ml_derp_get_stall_events(ml_derp_stall_event_t * out, int max)
{
  uint32_t w = __atomic_load_n(&s_derp_stall_widx, __ATOMIC_ACQUIRE);
  int n = (w < (uint32_t)ML_STALL_RING_LEN) ? (int)w : ML_STALL_RING_LEN;
  if (n > max) n = max;
  for (int i = 0; i < n; i++) {
    out[i] = s_derp_stall_ring[(w - n + i) % ML_STALL_RING_LEN];
  }
  return n;
}

/* Timeout for DERP connection handshake operations */
#define DERP_CONNECT_TIMEOUT_MS 20000 /* bumped 10s -> 20s 2026-05-25: DERP servers occasionally take >10s under load */

/* ============================================================================
 * Custom BIO callbacks for non-blocking TLS I/O
 *
 * These wrap lwIP recv/send with guaranteed timeout behavior.
 * We don't trust mbedtls_net_recv_timeout on lwIP because lwIP's select()
 * can sometimes block indefinitely on ESP32.
 * ========================================================================== */

/**
 * Custom recv with timeout for mbedtls BIO.
 * Uses SO_RCVTIMEO on the socket as the timeout mechanism (simpler than select).
 * Returns bytes read, or MBEDTLS_ERR_SSL_TIMEOUT, MBEDTLS_ERR_SSL_WANT_READ.
 */
static int ml_derp_bio_recv_timeout(void * ctx, unsigned char * buf, size_t len, uint32_t timeout)
{
  int fd = *(int *)ctx;
  if (fd < 0) return MBEDTLS_ERR_NET_INVALID_CONTEXT;

  /* Set SO_RCVTIMEO to the requested timeout.
     * If timeout is 0 (mbedTLS default = "no timeout"), use 10s as a sane
     * default to avoid indefinite blocking on AT sockets. */
  uint32_t effective_timeout = (timeout > 0) ? timeout : DERP_CONNECT_TIMEOUT_MS;
  struct timeval tv;
  tv.tv_sec = effective_timeout / 1000;
  tv.tv_usec = (effective_timeout % 1000) * 1000;
  ml_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  int ret = (int)ml_read_sock(fd, buf, len);
  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return MBEDTLS_ERR_SSL_TIMEOUT;
    }
    if (errno == EPIPE || errno == ECONNRESET) {
      return MBEDTLS_ERR_NET_CONN_RESET;
    }
    if (errno == EINTR) {
      return MBEDTLS_ERR_SSL_WANT_READ;
    }
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }
  return ret;
}

/**
 * Custom send for mbedtls BIO.
 */
static int ml_derp_bio_send(void * ctx, const unsigned char * buf, size_t len)
{
  int fd = *(int *)ctx;
  if (fd < 0) return MBEDTLS_ERR_NET_INVALID_CONTEXT;

  int ret = (int)ml_write_sock(fd, buf, len);
  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    if (errno == EPIPE || errno == ECONNRESET) {
      return MBEDTLS_ERR_NET_CONN_RESET;
    }
    if (errno == EINTR) {
      return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }
  return ret;
}

/* DERP frame types */
#define DERP_FRAME_SERVER_KEY 0x01
#define DERP_FRAME_CLIENT_INFO 0x02
#define DERP_FRAME_SERVER_INFO 0x03
#define DERP_FRAME_SEND_PACKET 0x04
#define DERP_FRAME_RECV_PACKET 0x05
#define DERP_FRAME_KEEP_ALIVE 0x06
#define DERP_FRAME_NOTE_PREFERRED 0x07
#define DERP_FRAME_PEER_GONE 0x08
#define DERP_FRAME_PING 0x12
#define DERP_FRAME_PONG 0x13

/* DISCO magic bytes: "TS" + sparkles emoji UTF-8 */
static const uint8_t DISCO_MAGIC[6] = {'T', 'S', 0xf0, 0x9f, 0x92, 0xac};

/* ============================================================================
 * TLS Read/Write Helpers
 * ========================================================================== */

/**
 * Write exactly `len` bytes via TLS with WANT_WRITE retry.
 * Returns bytes written on success, -1 on error.
 * No mutex needed — called only from the DERP I/O task.
 */
static int derp_tls_write_all(microlink_t * ml, ml_derp_conn_t * c, const uint8_t * data, size_t len)
{
  (void)ml;
  size_t written = 0;
  int retries = 0;
  const int max_retries = 50; /* 50 * 10ms = 500ms max */

  while (written < len) {
    int ret = mbedtls_ssl_write(&c->ssl, data + written, len - written);
    if (ret < 0) {
      if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE || ret == MBEDTLS_ERR_SSL_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (++retries > max_retries) {
          ESP_LOGW(TAG, "TLS write timeout after %d retries", retries);
          return -1;
        }
        continue;
      }
      ESP_LOGE(TAG, "TLS write failed: -0x%04x", -ret);
      return -1;
    }
    written += ret;
    retries = 0;
  }
  return (int)written;
}

/* Write a complete DERP frame via TLS */
static int derp_write_frame(microlink_t * ml, ml_derp_conn_t * c, uint8_t type, const uint8_t * payload, uint32_t len)
{
  /* 5-byte header: 1 type + 4 length (big-endian) */
  uint8_t header[5];
  header[0] = type;
  header[1] = (len >> 24) & 0xFF;
  header[2] = (len >> 16) & 0xFF;
  header[3] = (len >> 8) & 0xFF;
  header[4] = len & 0xFF;

  if (derp_tls_write_all(ml, c, header, 5) < 0) return -1;

  if (len > 0 && payload) {
    if (derp_tls_write_all(ml, c, payload, len) < 0) return -1;
  }
  return 0;
}

/* Send a packet to a peer via DERP */
static int derp_send_packet(
  microlink_t * ml, ml_derp_conn_t * c, const uint8_t * dest_key, const uint8_t * data, size_t len)
{
  /* SendPacket frame: 32-byte dest key + payload */
  size_t frame_len = 32 + len;
  uint8_t * frame = malloc(frame_len);
  if (!frame) return -1;

  memcpy(frame, dest_key, 32);
  memcpy(frame + 32, data, len);

  int ret = derp_write_frame(ml, c, DERP_FRAME_SEND_PACKET, frame, frame_len);
  if (ret < 0) {
    ESP_LOGW(
      TAG,
      "derp_send_packet FAILED: dest=%02x%02x%02x%02x len=%d",
      dest_key[0],
      dest_key[1],
      dest_key[2],
      dest_key[3],
      (int)len);
  }
  free(frame);
  return ret;
}

/* ============================================================================
 * DERP Frame Reading and Dispatch (runs on DERP I/O task)
 * ========================================================================== */

/* Packet classification */
typedef enum
{
  PKT_DISCO,
  PKT_STUN,
  PKT_WIREGUARD,
  PKT_UNKNOWN,
} pkt_type_t;

static pkt_type_t classify_packet(const uint8_t * data, size_t len)
{
  if (len >= 20 && (data[0] == 0x00 || data[0] == 0x01) && data[1] == 0x01) {
    return PKT_STUN;
  }
  if (len >= 62 && memcmp(data, DISCO_MAGIC, 6) == 0) {
    return PKT_DISCO;
  }
  if (len >= 4) {
    return PKT_WIREGUARD;
  }
  return PKT_UNKNOWN;
}

static void route_derp_packet(microlink_t * ml, uint8_t * data, size_t len, const uint8_t * src_pubkey)
{
  pkt_type_t type = classify_packet(data, len);

  ml_rx_packet_t pkt = {
    .data = data,
    .len = len,
    .via_derp = true,
  };
  memcpy(pkt.src_pubkey, src_pubkey, 32);

  QueueHandle_t target = (type == PKT_DISCO) ? ml->disco_rx_queue : ml->wg_rx_queue;
  if (xQueueSend(target, &pkt, 0) != pdTRUE) {
    if (target == ml->wg_rx_queue) s_diag_wg_rx_drops++; /* edge shed, was silent */
    free(data);
  }
}

/* Dispatch a received DERP frame */
static void dispatch_derp_frame(
  microlink_t * ml, ml_derp_conn_t * c, uint8_t frame_type, uint8_t * src_key, uint8_t * payload, size_t payload_len)
{
  switch (frame_type) {
    case DERP_FRAME_RECV_PACKET:
      if (payload) {
        /* §7 PROVING strong-proof evidence: a peer's frame arrived THROUGH
         * this server. Only RecvPacket counts — keepalives/pongs prove the
         * server link, not end-to-end peer reachability. */
        c->rx_pkts++;
        c->last_relay_rx_ms = ml_get_time_ms();
        ESP_LOGI(
          TAG,
          "DERP RecvPacket: %d bytes from %02x%02x%02x%02x, hdr=%02x",
          (int)payload_len,
          src_key[0],
          src_key[1],
          src_key[2],
          src_key[3],
          payload_len > 0 ? payload[0] : 0xFF);
        route_derp_packet(ml, payload, payload_len, src_key);
        return; /* payload ownership transferred */
      }
      break;

    case DERP_FRAME_KEEP_ALIVE:
      ESP_LOGD(TAG, "DERP KeepAlive received");
      break;

    case DERP_FRAME_PING:
      /* Echo ping data back as PONG directly (single-threaded, safe to write) */
      if (payload && payload_len > 0) {
        ESP_LOGD(TAG, "DERP PING received, sending PONG");
        derp_write_frame(ml, c, DERP_FRAME_PONG, payload, payload_len);
      }
      break;

    case DERP_FRAME_PONG:
      /* §7 PROVING weak-proof evidence (Q3): the server answered our client
       * PING — a live application-level round-trip on THIS conn. */
      c->last_pong_ms = ml_get_time_ms();
      ESP_LOGD(TAG, "DERP PONG received");
      break;

    case DERP_FRAME_PEER_GONE:
      if (payload && payload_len >= 32) {
        ESP_LOGI(
          TAG,
          "DERP PeerGone: %02x%02x%02x%02x (len=%d)",
          payload[0],
          payload[1],
          payload[2],
          payload[3],
          (int)payload_len);
      }
      break;

    default:
      ESP_LOGD(TAG, "DERP frame type 0x%02x ignored (%d bytes)", frame_type, (int)payload_len);
      break;
  }

  if (payload) free(payload);
}

/**
 * Try to read one DERP frame.
 * Uses mbedtls recv_timeout (100ms) so ssl_read never blocks indefinitely.
 * Returns: 1 = frame read and dispatched, 0 = timeout (no data), <0 = error
 */
static int poll_derp_read(microlink_t * ml, ml_derp_conn_t * c)
{
  if (!c->connected || c->sockfd < 0) return -1;

  /* Read 5-byte frame header.
     * SO_RCVTIMEO=100ms ensures read() returns within 100ms if no data. */
  uint8_t header[5];
  int n = mbedtls_ssl_read(&c->ssl, header, 5);
  if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_TIMEOUT) {
    return 0; /* No data available / timeout */
  }
  if (n <= 0) {
    ESP_LOGW(TAG, "DERP header read returned %d (0x%04x)", n, n < 0 ? -n : 0);
    return n;
  }
  if (n < 5) {
    ESP_LOGW(TAG, "DERP partial header: got %d of 5 bytes", n);
    return -1;
  }

  uint8_t frame_type = header[0];
  uint32_t len = (header[1] << 24) | (header[2] << 16) | (header[3] << 8) | header[4];

  uint8_t src_key[32] = {0};
  uint8_t * payload = NULL;
  size_t payload_len = 0;

  if (len == 0) {
    dispatch_derp_frame(ml, c, frame_type, src_key, NULL, 0);
    return 1;
  }

  if (len > 65536) {
    ESP_LOGW(TAG, "DERP frame too large: %lu", (unsigned long)len);
    return -1;
  }

  /* Read frame payload - we already got the header so the payload should follow.
     * Bounded by the per-iteration safety timeout below (see rationale there). */
  uint8_t * buf = ml_psram_malloc(len);
  if (!buf) return -1;

  size_t total_read = 0;
  uint64_t payload_start = ml_get_time_ms();
  while (total_read < len) {
    /* Safety timeout for payload. Kept at 5 s (NOT lowered to 1 s): once a frame
     * header has arrived, the payload can straggle across several TCP retransmits
     * on a lossy NAT'd/cellular link (RTO >= 1 s, backing off). A 1 s cap would
     * truncate a legit in-flight heartbeat/STOP frame AND trip a home reconnect
     * that flushes the TX queue and gaps the bond — strictly worse than the brief
     * egress delay it was meant to avoid (adversarial review 2026-08-04). */
    if (ml_get_time_ms() - payload_start > 5000) {
      ESP_LOGW(TAG, "DERP payload timeout at %d/%lu bytes", (int)total_read, (unsigned long)len);
      free(buf);
      return -1;
    }
    n = mbedtls_ssl_read(&c->ssl, buf + total_read, len - total_read);
    if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_TIMEOUT) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (n <= 0) {
      ESP_LOGW(
        TAG,
        "DERP payload read error: %d (0x%04x) at %d/%lu bytes",
        n,
        n < 0 ? -n : 0,
        (int)total_read,
        (unsigned long)len);
      free(buf);
      return n;
    }
    total_read += n;
  }

  /* For RecvPacket (0x05): first 32 bytes are sender's public key */
  if (frame_type == DERP_FRAME_RECV_PACKET && len > 32) {
    memcpy(src_key, buf, 32);
    payload = malloc(len - 32);
    if (payload) {
      memcpy(payload, buf + 32, len - 32);
      payload_len = len - 32;
    }
    free(buf);
  } else {
    payload = buf;
    payload_len = len;
  }

  dispatch_derp_frame(ml, c, frame_type, src_key, payload, payload_len);
  return 1;
}

/* ============================================================================
 * DERP TX Queue Processing
 * ========================================================================== */

/* Queue a packet for DERP TX with backpressure */
esp_err_t ml_derp_queue_send(microlink_t * ml, const uint8_t * dest_key, const uint8_t * data, size_t len)
{
  if (!ml || !dest_key || !data || len == 0) return ESP_ERR_INVALID_ARG;

  uint8_t * pkt_data = malloc(len);
  if (!pkt_data) return ESP_ERR_NO_MEM;
  memcpy(pkt_data, data, len);

  ml_derp_tx_item_t item = {
    .data = pkt_data,
    .len = len,
    .frame_type = DERP_FRAME_SEND_PACKET,
    /* Resolve the destination peer's DERP home region NOW (this runs on the
     * wg_mgr task that owns the peer table). The DERP task then egresses this
     * frame on the pool connection homed on that region — the whole point of
     * the multi-region pool. 0 = peer/region unknown -> routed on the HOME
     * connection (effective home region). */
    .region_id = ml_wg_region_for_pubkey(ml, dest_key),
  };
  memcpy(item.dest_pubkey, dest_key, 32);

  /* Route to the correct FIFO queue: safety/handshake frames -> the PRIORITY
   * queue (the DERP task drains it first, so a 5 Hz heartbeat is never stuck
   * behind a disco burst); everything else -> the normal disco queue. Enqueue is
   * a single atomic op per queue, so it is safe against the concurrent producers
   * (wg_mgr disco + the TCPIP-thread heartbeat via wg_derp_output_cb) with NO
   * cross-thread drain, and per-peer frame order is preserved (FIFO within a
   * queue -> no OK-after-STOP reordering). */
  bool is_wg_handshake = ml_wg_is_handshake_frame(data, len);
  /* One classification walk: prio = pinned||health-tracked (fleet frames must
   * not sit behind a disco burst), mirror = heartbeat carriers only (priority
   * ||health-tracked — the fleet pin is reachability armor; doubling its bulk
   * frames buys no diversity and crowds the prio queue). */
  bool mirror = false;
  /* Classify FIRST: `||` short-circuiting on is_wg_handshake would skip the
   * call and cost a safety peer's rekey handshake its leg-2 mirror. */
  bool safety_prio = ml_wg_tx_class_pubkey(ml, dest_key, &mirror);
  bool is_priority = is_wg_handshake || safety_prio;
  QueueHandle_t txq = is_priority ? ml->derp_tx_prio_queue : ml->derp_tx_queue;

  /* Path diversity (docs/HEARTBEAT_PATH_DIVERSITY_DESIGN.md): heartbeat
   * carriers get a SECOND DERP leg on a distinct connected pool conn.
   * Enqueued AFTER the primary so a nearly-full queue can never let the
   * mirror displace or evict a real frame. Best-effort. */

  if (xQueueSend(txq, &item, 0) == pdTRUE) {
    if (mirror) {
      ml_derp_tx_item_t leg2 = item;
      leg2.leg2 = true;
      leg2.data = malloc(len);
      if (leg2.data) {
        memcpy(leg2.data, data, len);
        if (xQueueSend(txq, &leg2, 0) != pdTRUE) {
          free(leg2.data); /* mirror is best-effort; the primary already went */
        }
      }
    }
    return ESP_OK;
  }

  /* Queue full: drop the OLDEST item on THIS queue to admit the newer one, retry
   * up to 3x. On the PRIORITY queue the oldest is the STALEST heartbeat/handshake
   * — dropping it to keep the freshest is correct for liveness (a full priority
   * queue is already a multi-second backlog; the bond is gapping regardless, and
   * the receiver only cares about the newest heartbeat). A disco frame can NEVER
   * evict a heartbeat — they live on separate queues. All single-atomic ops, no
   * cross-thread drain/requeue. */
  for (int i = 0; i < 3; i++) {
    ml_derp_tx_item_t dropped;
    if (xQueueReceive(txq, &dropped, 0) == pdTRUE) {
      free(dropped.data);
    }
    if (xQueueSend(txq, &item, 0) == pdTRUE) {
      return ESP_OK;
    }
  }

  /* Still full after 3 attempts, drop the new packet. */
  free(pkt_data);
  return ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Multi-region pool routing + aux-connection management
 * ========================================================================== */

/* Pick the pool connection to egress a frame bound for `region_id`.
 * region_id 0 or == the effective home region -> the HOME conn
 * (ml->derp[derp_home_slot] — an index since the §7 MBB refactor; slot 0 at
 * boot, swapped by a committed home switch). Otherwise the connected aux conn
 * homed on that region; if none is up yet we fall back to the HOME conn so a
 * safety peer's frame is NEVER silently dropped (the caller logs the
 * fallback). Returns NULL only when the chosen conn is down (nothing can
 * carry it this pass). */
/* Frames stamped for a non-home region that had NO connected aux conn and
 * fell back to the HOME conn. A DERP server only delivers to clients
 * connected to IT — so unless the destination peer also sits on our home
 * region, every one of these frames is silently eaten server-side ("does not
 * know about peer"). This is the black-hole that starves WG rekeys after a
 * region switch when a peer-region record is stale (run-20/21 forensics). */
static uint32_t s_diag_route_home_fallbacks;

uint32_t ml_derp_get_route_fallbacks(void)
{
  return s_diag_route_home_fallbacks;
}

/* Home-DERP reconnect telemetry (2026-08-12 edge-flush fix). reconnects =
 * count of successful RST/EOF-triggered home reconnects; last/worst =
 * wall-clock ms from RST detection to reconnected. worst < ~1500 confirms the
 * relay is back inside the 2 s pstop timeout window (green rides it through a
 * flush); a worst climbing toward/over 2000 means recovery is still too slow. */
static uint32_t s_diag_derp_reconnects;
static uint32_t s_diag_derp_last_reconnect_ms;
static uint32_t s_diag_derp_worst_reconnect_ms;
/* HOME-reconnect cause attribution (run-46: machn churned ~50-64 home
 * reconnects/hr with no way to tell why). Indices: 0=tx write failed,
 * 1=rx read error (RST/EOF), 2=tx-active-rx-silent staleness reap,
 * 3=interface rebind (microlink.c network-change path). */
static uint32_t s_reconn_causes[4];

void ml_derp_note_reconnect_cause(int cause)
{
  if (cause >= 0 && cause < 4) s_reconn_causes[cause]++;
}

void ml_derp_get_reconnect_causes(uint32_t out[4])
{
  for (int i = 0; i < 4; i++) out[i] = s_reconn_causes[i];
}

void ml_derp_get_reconnect_diag(uint32_t out[3])
{
  out[0] = s_diag_derp_reconnects;
  out[1] = s_diag_derp_last_reconnect_ms;
  out[2] = s_diag_derp_worst_reconnect_ms;
}

static uint32_t s_diag_leg2_sent; /* path-diversity mirrors egressed */
static uint32_t s_diag_leg2_skipped; /* mirrors skipped (no distinct connected conn) */
static uint32_t s_diag_rescue_routes; /* primary frames rescued onto any-connected conn */
static uint32_t s_diag_rx_stale_reaps; /* tx-active conns reaped for rx-silence */

uint32_t ml_derp_get_rx_stale_reaps(void)
{
  return s_diag_rx_stale_reaps;
}

/* Single home-conn identity test (4 call sites). */
static inline bool derp_conn_is_home(const microlink_t * ml, const ml_derp_conn_t * c)
{
  return c == &ml->derp[ml->derp_home_slot];
}

/* Tear down a live conn and route its recovery: the HOME conn recovers via
 * the event-driven reconnect (the handler frees TLS then kicks), an aux is
 * freed NOW so derp_manage_aux can rebuild without re-initing over live
 * contexts. region_id is preserved for the retry. */
static void derp_reap_conn(microlink_t * ml, ml_derp_conn_t * c, int cause)
{
  c->connected = false;
  if (derp_conn_is_home(ml, c)) {
    ml_derp_note_reconnect_cause(cause);
    xEventGroupSetBits(ml->events, ML_EVT_DERP_RECONNECT);
  } else {
    ml_derp_disconnect(ml, c);
  }
}

void ml_derp_get_diversity_diag(uint32_t out[3])
{
  out[0] = s_diag_leg2_sent;
  out[1] = s_diag_leg2_skipped;
  out[2] = s_diag_rescue_routes;
}

/* Leg-2 route: a CONNECTED conn DISTINCT from the primary leg's choice.
 * Preference order: the "other" of {peer-region conn, home conn}, then any
 * other connected pool conn. NULL = no distinct conn (mirror skipped). */
static ml_derp_conn_t * derp_route_leg2(microlink_t * ml, ml_derp_conn_t * primary)
{
  int hs = ml->derp_home_slot;
  ml_derp_conn_t * home = &ml->derp[hs];
  if (primary != home && home->connected) return home;
  for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
    ml_derp_conn_t * c = &ml->derp[s];
    if (c == primary) continue;
    if (c->connected) return c;
  }
  return NULL;
}

/* Rescue route: ANY connected conn (replaces the silent drop when the
 * primary route is down — the peer may hold a pool conn on that region). */
static ml_derp_conn_t * derp_route_any(microlink_t * ml)
{
  int hs = ml->derp_home_slot;
  if (ml->derp[hs].connected) return &ml->derp[hs];
  for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
    if (ml->derp[s].connected) return &ml->derp[s];
  }
  return NULL;
}

/* Primary route: the conn LIVE on region_id (the home conn's CURRENT region,
 * not the effective-home override — during a pin/MBB prove window they differ
 * and comparing the override would black-hole frames for peers already homed
 * on the target). Falls back to the home conn (counted + warned, primaries
 * only) when no conn serves the region; NULL = pool dark, caller rescues. */
static ml_derp_conn_t * derp_route_conn(microlink_t * ml, uint16_t region_id, bool count_fb)
{
  int hs = ml->derp_home_slot;
  ml_derp_conn_t * home = &ml->derp[hs];
  uint16_t live_home = home->connected ? home->region_id : 0;
  if (region_id != 0 && region_id != live_home) {
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].connected && ml->derp[s].region_id == region_id) {
        return &ml->derp[s];
      }
    }
    if (home->connected && count_fb) {
      s_diag_route_home_fallbacks++;
      ESP_LOGW(TAG, "DERP TX: no conn for region %u, relaying via home (frame preserved)", (unsigned)region_id);
    }
  }
  return home->connected ? home : NULL;
}

/* Maintain the auxiliary slots against the CURRENT set of distinct
 * safety-peer regions (pinned/priority/health-tracked, queried from wg_mgr).
 * Opens a conn per needed region (excluding the home region, served by the
 * HOME slot), reaps aux conns whose region is no longer needed AND idle >
 * threshold, and applies a per-slot connect backoff. Runs at the TAIL of the
 * I/O loop so the home slot is always serviced first — a blocking aux TLS
 * handshake can't starve the home (safety) path. `aux_burst` is task-local
 * per-slot backoff state. NEVER reaps the HOME slot (any index since the §7
 * MBB refactor — a committed switch turns the OLD home slot into an ordinary
 * aux that IS managed here; that decay is the §7 DRAIN_OLD phase). */
static void derp_manage_aux(microlink_t * ml, uint16_t eff_home, int aux_burst[], uint64_t now)
{
  int hs = ml->derp_home_slot;
  uint16_t regs[ML_DERP_MAX_CONNS];
  int nregs = ml_wg_collect_safety_regions(ml, regs, ML_DERP_MAX_CONNS);

  /* Distinct wanted aux regions = safety regions minus the home region.
   * "Home region" here must cover BOTH the effective home (where we are
   * heading) and the LIVE home conn's region (where we are serving from):
   * during a Stage-1 pin MBB prove window they differ (eff == the pin, live
   * == the old region), and excluding only eff would open a DUPLICATE aux to
   * the region the home slot is actively serving (the served-check below
   * deliberately skips the home slot). Under autoneg the two are equal until
   * commit, so this is behavior-neutral there. */
  uint16_t live_home_region = ml->derp[hs].connected ? ml->derp[hs].region_id : 0;
  uint16_t want[ML_DERP_MAX_CONNS];
  int nwant = 0;
  for (int i = 0; i < nregs; i++) {
    if (regs[i] != 0 && regs[i] != eff_home && regs[i] != live_home_region) want[nwant++] = regs[i];
  }

  /* §7 AUX_OPENING: a pending MBB target region joins the want-set, so the
   * EXISTING open/backoff/one-blocking-connect-cap machinery below opens and
   * maintains its conn — MBB inherits every starvation protection for free
   * instead of duplicating connect logic. */
  {
    uint16_t mbb_tgt = ml->mbb_target_region;
    /* Exclude the target only when the LIVE home conn already serves it —
     * not when it merely equals eff_home. Under a pin (Stage-1),
     * eff_home == the override == the MBB target from the instant the pin is
     * set, while the actual home conn still sits on the OLD region; comparing
     * against eff_home would keep the target out of the want-set and the
     * pin's MBB could never open its conn. */
    bool tgt_is_live_home = ml->derp[hs].connected && ml->derp[hs].region_id == mbb_tgt;
    if (mbb_tgt != 0 && !tgt_is_live_home && nwant < ML_DERP_MAX_CONNS) {
      bool dup = false;
      for (int i = 0; i < nwant; i++) {
        if (want[i] == mbb_tgt) {
          dup = true;
          break;
        }
      }
      if (!dup) want[nwant++] = mbb_tgt;
    }
  }

  /* 1) Reap aux slots whose region is no longer a safety region. The clock is
   *    time-since-UNWANTED, not idle time. Incidental (non-safety) tailnet
   *    traffic to peers on that region routes through the aux and refreshes
   *    last_used_ms, so an idle-based reap never fired: a boot-transient aux
   *    region (one briefly in the safety set before the netmap/home settled, or
   *    a region == the not-yet-settled home) could persist for hours pinning a
   *    pool slot — the "phantom region-9 aux" seen in soak. Reaping on
   *    time-since-unwanted is immune to that incidental traffic. */
  for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
    if (s == hs) continue; /* NEVER reap the home slot */
    ml_derp_conn_t * c = &ml->derp[s];
    if (c->region_id == 0 && !c->tls_inited && !c->connected) {
      c->aux_unwanted_since_ms = 0;
      continue; /* free slot */
    }
    bool wanted = false;
    for (int w = 0; w < nwant; w++) {
      /* Same in-flight visibility as the served/reuse checks below: a
       * mid-connect slot (region_id still 0) toward a wanted region must
       * not start the unwanted-reap clock. */
      if (want[w] == c->region_id || (c->cstate != DERP_CS_IDLE && c->cs_region == want[w])) {
        wanted = true;
        break;
      }
    }
    if (wanted) {
      c->aux_unwanted_since_ms = 0; /* still a current safety region */
      continue;
    }
    if (c->aux_unwanted_since_ms == 0) c->aux_unwanted_since_ms = now;
    if (now - c->aux_unwanted_since_ms > ML_DERP_AUX_UNWANTED_REAP_MS) {
      ESP_LOGI(TAG, "Reaping unwanted aux DERP conn slot %d (region %u)", s, (unsigned)c->region_id);
      ml_derp_disconnect(ml, c);
      c->region_id = 0;
      c->aux_unwanted_since_ms = 0;
      aux_burst[s] = 0;
    }
  }

  /* 2) Ensure a conn for each wanted region. Connects are async: this only
   *    KICKS (the task-loop stepper advances ~one syscall per iteration), so
   *    aux work can never stall the home read / safety path. One new kick per
   *    call bounds concurrent handshake crypto. */
  for (int w = 0; w < nwant; w++) {
    uint16_t rid = want[w];
    bool served = false;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      /* An IN-FLIGHT connect toward rid counts as served: region_id is only
       * written on completion/failure, so without the cs_region check every
       * pass would kick ANOTHER slot at the same region until the pool
       * drained (and re-kick a mid-TLS slot, tearing down its handshake). */
      if (
        (ml->derp[s].connected && ml->derp[s].region_id == rid) ||
        (ml->derp[s].cstate != DERP_CS_IDLE && ml->derp[s].cs_region == rid))
      {
        served = true;
        break;
      }
    }
    if (served) continue;

    /* Reuse a slot already assigned this region (a prior drop/failed attempt),
     * else grab a free slot. Never a slot mid-connect toward another region. */
    int slot = -1;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].cstate == DERP_CS_IDLE && ml->derp[s].region_id == rid) {
        slot = s;
        break;
      }
    }
    if (slot < 0) {
      for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
        if (s == hs) continue;
        if (
          !ml->derp[s].connected && !ml->derp[s].tls_inited && ml->derp[s].region_id == 0 &&
          ml->derp[s].cstate == DERP_CS_IDLE) /* an in-flight TCP-phase connect is NOT free */
        {
          slot = s;
          break;
        }
      }
    }
    if (slot < 0) {
      ESP_LOGW(TAG, "No free DERP aux slot for region %u (pool full)", (unsigned)rid);
      continue;
    }

    ml_derp_conn_t * c = &ml->derp[slot];
    /* Per-slot backoff: fast at first (safety failover), then calm so a truly
     * down region server isn't hammered — mirrors the home retry cadence. */
    uint64_t gap = (aux_burst[slot] < 2) ? 5000u : (aux_burst[slot] < 3) ? 10000u : 60000u;
    if (c->last_connect_attempt_ms != 0 && (now - c->last_connect_attempt_ms) < gap) continue;
    /* Stage-2: KICK the async connect; the task loop's stepper advances it a
     * bounded action per iteration (aux_burst resets on completion there).
     * kick() failing == immediate DNS/socket error; the slot keeps the region
     * (derp_cs_fail assigns it) and the per-slot backoff retries. A spacing-
     * gate REFUSAL is not an attempt: stamp/bump only for real outcomes, or a
     * flush would burn the 5s→10s→60s ladder on kicks that never happened
     * (review finding). */
    if (ml_derp_connect_kick(ml, c, rid) == ESP_ERR_NOT_FINISHED) {
      return; /* window closed this pass for all non-home kicks anyway */
    }
    c->last_connect_attempt_ms = now;
    if (aux_burst[slot] < 100) aux_burst[slot]++;
    ESP_LOGI(TAG, "Opening aux DERP conn slot %d for region %u", slot, (unsigned)rid);
    /* One new kick per call keeps handshake crypto bursts bounded. */
    return;
  }
}

/* Telemetry: home-region-unreachable fallbacks (see design-review finding in
 * the periodic-retry block below). Owned by the DERP I/O task; read cross-task
 * by /admin/api/monitor via ml_derp_get_home_fallback_count(). */
static uint32_t s_derp_home_unreachable_fallbacks;

uint32_t ml_derp_get_home_fallback_count(void)
{
  return s_derp_home_unreachable_fallbacks;
}

/* Pick a DERP region that is PROVEN reachable right now, to fall back onto when
 * the home region's DERP server is TCP-unreachable. Order (task guidance):
 *   1. the region of any currently-CONNECTED aux slot — reachability just
 *      demonstrated by a live TLS session, so the fallback connect will almost
 *      certainly succeed on the first try (no retry storm);
 *   2. else the compiled default ML_DERP_REGION (Tailscale's most-available
 *      region), which also matches the PreferredDERP advertised when no
 *      priority region is learned;
 *   3. else any DISTINCT safety-peer want-set region (ml_wg_collect_safety_regions).
 * Never returns `avoid` (the dead region) or 0-unless-no-candidate. */
static uint16_t derp_pick_fallback_region(microlink_t * ml, uint16_t avoid)
{
  /* 1) a live aux conn — reachability is a fact, not a guess. */
  int hs = ml->derp_home_slot;
  for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
    if (s == hs) continue;
    if (ml->derp[s].connected && ml->derp[s].region_id != 0 && ml->derp[s].region_id != avoid) {
      return ml->derp[s].region_id;
    }
  }
  /* 2) compiled default. */
  if (ML_DERP_REGION != avoid) return ML_DERP_REGION;
  /* 3) any safety want-set region that isn't the dead home. */
  uint16_t regs[ML_DERP_MAX_CONNS];
  int nregs = ml_wg_collect_safety_regions(ml, regs, ML_DERP_MAX_CONNS);
  for (int i = 0; i < nregs; i++) {
    if (regs[i] != 0 && regs[i] != avoid) return regs[i];
  }
  return 0;
}

/* Find a free non-HOME aux slot (region 0, no TLS, not connected). -1 if none. */
static int derp_free_aux_slot(microlink_t * ml)
{
  int hs = ml->derp_home_slot;
  for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
    if (s == hs) continue;
    if (
      !ml->derp[s].connected && !ml->derp[s].tls_inited && ml->derp[s].region_id == 0 &&
      ml->derp[s].cstate == DERP_CS_IDLE) /* in-flight TCP-phase connect != free */
    {
      return s;
    }
  }
  return -1;
}

/* ============================================================================
 * §7 Make-before-break home-switch executor (design doc, Q3 = opportunistic
 * hybrid proving).
 *
 * Runs ONLY on this task (the pool owner). The wg_mgr negotiator requests a
 * switch by writing ml->mbb_target_region + bumping ml->mbb_generation; this
 * executor opens/proves the target and, only on proof, swaps
 * ml->derp_home_slot — the INDEX, never conn state, so sockets/TLS contexts
 * never move (§7 SWITCH_ADVERT). The old home decays into an ordinary aux and
 * is held exactly as long as any safety peer still homes there (the existing
 * want-set reap IS the §7 DRAIN_OLD grace period). The safety bond is never
 * touched: both regions hold live conns across the whole switch (I1/I5), and
 * rollback is a no-op on the bond because home was never moved.
 *
 * WHAT THIS REPLACES: for a bond-live region change, the old path was
 * ML_EVT_DERP_RECONNECT — tear down the home conn, then a 2-40 s TLS
 * handshake gap on the relay path (break-before-make). That path still exists
 * and is still used for the no-bond/lock cases (§6), unchanged.
 * ========================================================================== */

/* §16 telemetry (single-writer: this task; cross-task word reads via getter). */
static uint32_t s_mbb_commits;
static uint32_t s_mbb_rollbacks;
static uint32_t s_mbb_proofs_ok;
static uint32_t s_mbb_proofs_failed;

void ml_derp_get_mbb_diag(uint32_t out[4])
{
  out[0] = s_mbb_commits;
  out[1] = s_mbb_rollbacks;
  out[2] = s_mbb_proofs_ok;
  out[3] = s_mbb_proofs_failed;
}

/* Executor-local state (RAM-only by design §7: a reboot mid-MBB lands in the
 * proven boot path; NVS is written by nothing in this machine). */
typedef struct
{
  uint32_t gen; /* generation this run serves */
  uint16_t target;
  uint8_t state; /* ML_MBB_* */
  uint64_t entered_ms; /* AUX_OPENING deadline base */
  uint64_t prove_started_ms;
  uint64_t last_prove_ping_ms;
  int prove_slot;
  uint32_t prove_base_rx; /* rx_pkts snapshot at prove start */
  bool prove_peer_on_target; /* Q3: strong proof possible? */
} derp_mbb_t;

static derp_mbb_t s_mbb;

static void derp_mbb_post_outcome(microlink_t * ml, uint8_t outcome, uint16_t region)
{
  /* Region before outcome: the negotiator keys on outcome != NONE. */
  ml->mbb_outcome_region = region;
  ml->mbb_outcome = outcome;
  s_mbb.state = ML_MBB_IDLE;
  ml->mbb_state = ML_MBB_IDLE;
}

static void derp_mbb_rollback(microlink_t * ml, const char * why)
{
  s_mbb_rollbacks++;
  ESP_LOGW(
    TAG,
    "MBB ROLLBACK (region %u): %s — home untouched, bond unaffected; negotiator will ban the region",
    (unsigned)s_mbb.target,
    why);
  /* The target leaves the aux want-set once the negotiator clears the pending
   * request; the existing unwanted-reap then closes the conn (unless a safety
   * peer legitimately needs that region). No teardown here. */
  derp_mbb_post_outcome(ml, ML_MBB_OUTCOME_ROLLED_BACK, s_mbb.target);
}

static void derp_mbb_tick(microlink_t * ml, uint64_t now)
{
  uint16_t tgt = ml->mbb_target_region;
  uint32_t gen = ml->mbb_generation;

  /* New request, preemption or cancel: restart from IDLE (§7 — a newer target
   * preempts by restarting; a cleared target aborts). A torn (gen,target)
   * read costs one extra restart next tick, never a wrong commit. */
  if (gen != s_mbb.gen || tgt != s_mbb.target) {
    s_mbb.gen = gen;
    s_mbb.target = tgt;
    s_mbb.state = (tgt != 0) ? ML_MBB_AUX_OPENING : ML_MBB_IDLE;
    s_mbb.entered_ms = now;
    ml->mbb_state = s_mbb.state;
    if (tgt != 0) {
      ESP_LOGI(TAG, "MBB AUX_OPENING: target region %u (aux via existing want-set machinery)", (unsigned)tgt);
    }
  }
  if (s_mbb.state == ML_MBB_IDLE) {
    return;
  }

  /* I2' (Stage-1 pin->MBB, docs/STAGE1_PIN_MBB_DESIGN.md): an override aborts
   * an in-flight MBB only when it names a DIFFERENT region than the target —
   * operator preemption of an autoneg switch, the original I2 intent. An MBB
   * whose target EQUALS the override IS the override being applied (the
   * negotiator's pin tick issued it); it must proceed, or the pin could only
   * ever land via the gap-prone teardown path. */
  if (ml->derp_region_override != 0 && ml->derp_region_override != s_mbb.target) {
    ESP_LOGW(TAG, "MBB ABORT: region lock %u applied mid-switch", (unsigned)ml->derp_region_override);
    derp_mbb_post_outcome(ml, ML_MBB_OUTCOME_ABORTED, s_mbb.target);
    return;
  }
  /* Target is already the LIVE home (e.g. legacy rehome after the bond
   * dropped reconnected straight there): nothing left to do. Compare against
   * the actual connected home conn, NOT ml_effective_home_region() — under a
   * pin the eff alias equals the override from the instant it is set, long
   * before any conn has moved, and would falsely abort the pin's own MBB. */
  {
    ml_derp_conn_t * homec = &ml->derp[ml->derp_home_slot];
    if (homec->connected && homec->region_id == s_mbb.target) {
      derp_mbb_post_outcome(ml, ML_MBB_OUTCOME_ABORTED, s_mbb.target);
      return;
    }
  }

  if (s_mbb.state == ML_MBB_AUX_OPENING) {
    /* derp_manage_aux (want-set union) owns the actual connect + backoff +
     * one-blocking-connect-per-loop cap; we only watch for the conn. */
    int slot = -1;
    int hs = ml->derp_home_slot;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].connected && ml->derp[s].region_id == s_mbb.target) {
        slot = s;
        break;
      }
    }
    if (slot >= 0) {
      s_mbb.prove_slot = slot;
      s_mbb.prove_started_ms = now;
      s_mbb.last_prove_ping_ms = 0;
      s_mbb.prove_base_rx = ml->derp[slot].rx_pkts;
      /* Q3 opportunistic hybrid: if any SAFETY peer is homed on the target,
       * end-to-end proof is available for free — its dual-sent disco/heartbeat
       * DERP leg egresses via this conn (derp_route_conn routes by region) and
       * the reply comes back through the target server as a RecvPacket. Demand
       * that. Otherwise (fleet-advice on a machine/idle chip: no peer there to
       * echo) fall back to conn-stable + a DERP server PING/PONG round-trip —
       * weaker, acceptable because such switches are rare and damped, and
       * rollback is free. */
      uint16_t regs[ML_DERP_MAX_CONNS];
      int nregs = ml_wg_collect_safety_regions(ml, regs, ML_DERP_MAX_CONNS);
      s_mbb.prove_peer_on_target = false;
      for (int i = 0; i < nregs; i++) {
        if (regs[i] == s_mbb.target) {
          s_mbb.prove_peer_on_target = true;
          break;
        }
      }
      s_mbb.state = ML_MBB_PROVING;
      ml->mbb_state = ML_MBB_PROVING;
      ESP_LOGI(
        TAG,
        "MBB PROVING: region %u on slot %d (%s proof)",
        (unsigned)s_mbb.target,
        slot,
        s_mbb.prove_peer_on_target ? "end-to-end peer" : "conn-stable + server pong");
    } else if (now - s_mbb.entered_ms > ML_MBB_AUX_OPEN_TIMEOUT_MS) {
      derp_mbb_rollback(ml, "target aux failed to connect within the open window");
    }
    return;
  }

  /* ML_MBB_PROVING */
  {
    ml_derp_conn_t * c = &ml->derp[s_mbb.prove_slot];
    if (!c->connected || c->region_id != s_mbb.target) {
      /* The conn we were proving died — that IS a failed proof (§7). */
      s_mbb_proofs_failed++;
      derp_mbb_rollback(ml, "target conn dropped mid-prove");
      return;
    }
    bool stable = (now - c->connected_at_ms) >= ML_MBB_PROVE_STABLE_MS; /* gate (a) */
    bool proven;
    bool want_server_ping = false;
    if (s_mbb.prove_peer_on_target) {
      proven = (c->rx_pkts > s_mbb.prove_base_rx); /* gate (b), strong: peer echo */
      /* Prove-race fallback (§7): the peer is homed on the target region but may
       * have JUST switched there (an autoneg follow moves both ends together) and
       * not be echoing via this region yet — waiting the full 30 s window for its
       * echo spuriously rolls back and bans the region 15 min, stranding a split.
       * After a short grace favouring the strong peer echo, ALSO accept a DERP
       * server PONG (region reachable). Safe: MBB keeps the old home as a draining
       * aux and the safety heartbeat rides the region-INDEPENDENT direct path, so
       * committing on region-reachable never gaps the bond. */
      if (!proven && (now - s_mbb.prove_started_ms) >= ML_MBB_PROVE_PEER_GRACE_MS) {
        proven = (c->last_pong_ms >= s_mbb.prove_started_ms);
        want_server_ping = !proven;
      }
    } else {
      proven = (c->last_pong_ms >= s_mbb.prove_started_ms); /* gate (b), weak */
      want_server_ping = !proven;
    }
    if (
      want_server_ping &&
      (s_mbb.last_prove_ping_ms == 0 || (now - s_mbb.last_prove_ping_ms) >= ML_MBB_PROVE_PING_INTERVAL_MS))
    {
      /* Client PING (0x12, 8-byte payload) — server answers PONG on the same
       * conn. Direct write is safe: we ARE the single DERP writer. */
      uint8_t ping[8];
      esp_fill_random(ping, sizeof(ping));
      (void)derp_write_frame(ml, c, DERP_FRAME_PING, ping, sizeof(ping));
      s_mbb.last_prove_ping_ms = now;
    }

    if (stable && proven) {
      /* ---- SWITCH_ADVERT: the commit point (§7). Swap the home INDEX. ---- */
      int old_slot = ml->derp_home_slot;
      ml_derp_conn_t * oldc = &ml->derp[old_slot];
      uint8_t pref0 = 0x00;
      uint8_t pref1 = 0x01;
      /* Tell both servers which conn is our home now (mirrors magicsock's
       * NotePreferred). Best-effort: a failed frame is harmless — the coord
       * task's 60 s NotePreferred keepalive re-asserts it on the new home. */
      if (oldc->connected) {
        (void)derp_write_frame(ml, oldc, DERP_FRAME_NOTE_PREFERRED, &pref0, 1);
      }
      (void)derp_write_frame(ml, c, DERP_FRAME_NOTE_PREFERRED, &pref1, 1);
      ml->derp_home_slot = s_mbb.prove_slot; /* THE swap — index only, no conn state moves */
      ml->derp_home_region = s_mbb.target; /* routing + eff-home follow immediately */
      c->last_relay_rx_ms = ml_get_time_ms(); /* fresh reap grace: a weak-proof commit
                                               * (server PONG, no RecvPacket) would
                                               * otherwise carry the aux-connect stamp
                                               * into the home-only rx-staleness reap */
      oldc->aux_unwanted_since_ms = 0; /* old home starts life as a WANTED-until-proven-otherwise aux */
      s_mbb_proofs_ok++;
      s_mbb_commits++;
      ESP_LOGW(
        TAG,
        "MBB COMMIT: home slot %d -> %d, region %u (old region %u drains via want-set; no teardown, bond gapless)",
        old_slot,
        s_mbb.prove_slot,
        (unsigned)s_mbb.target,
        (unsigned)oldc->region_id);
      /* The advert side (priority_peer_region + PreferredDERP re-announce) is
       * finished by the wg_mgr negotiator on consuming this outcome — it owns
       * those fields today and stays their single writer. */
      derp_mbb_post_outcome(ml, ML_MBB_OUTCOME_COMMITTED, s_mbb.target);
      return;
    }

    if (now - s_mbb.prove_started_ms > ML_MBB_PROVE_TIMEOUT_MS) {
      s_mbb_proofs_failed++;
      derp_mbb_rollback(ml, "proof window expired (no end-to-end/pong evidence)");
    }
  }
}

/* ============================================================================
 * Unified DERP I/O Task
 * ========================================================================== */

/* Stage-2/3 driver state (DERP-task-owned). */
static uint64_t s_reconn_t0; /* RECONNECT-event wall-clock, for the reconnect diag */
static uint16_t s_home_fb_pending; /* home-fallback region awaiting async completion */
static int s_rescue_slot = -1; /* rescue-aux slot awaiting completion (counter gates on it) */
static int s_home_retry_burst; /* consecutive failed home attempts (was block-static) */
static uint32_t s_diag_max_iter_ms; /* Stage-0 gauge: worst single task iteration */
static uint32_t s_diag_rx_gap_worst_ms; /* Stage-0 gauge: worst gap between rx polls */

void ml_derp_get_iter_diag(uint32_t out[2])
{
  out[0] = s_diag_max_iter_ms;
  out[1] = s_diag_rx_gap_worst_ms;
}

/* Advance every in-progress connect one bounded step (home first; at most two
 * crypto-heavy TLS steps per pass). Completion bookkeeping for the home conn
 * (retry-burst reset, reconnect diag, fallback apply) lives here so the kick
 * sites can stay fire-and-forget. */
static void derp_drive_connects(microlink_t * ml, int aux_burst[])
{
  int crypto_steps = 0;
  int hs = ml->derp_home_slot;
  static int s_scan_rotor; /* rotate the non-home scan start so the crypto cap
                            * can't starve the same trailing slot */
  s_scan_rotor = (s_scan_rotor + 1) % ML_DERP_MAX_CONNS;
  /* home first (k==0), then ALL SIX rotated residues (k=1..6) — k-1+rotor for
   * k<6 only yields 5 consecutive residues, silently skipping one non-home
   * slot most passes. hs is deduped when it appears. */
  for (int k = 0; k <= ML_DERP_MAX_CONNS; k++) {
    int idx = (k == 0) ? hs : ((k - 1 + s_scan_rotor) % ML_DERP_MAX_CONNS);
    if (k != 0 && idx == hs) continue; /* home already served at k==0 */
    ml_derp_conn_t * c = &ml->derp[idx];
    if (c->cstate == DERP_CS_IDLE) continue;
    if (c->cstate == DERP_CS_TLS_HANDSHAKE) {
      if (crypto_steps >= 2) continue; /* bound concurrent handshake crypto */
      crypto_steps++;
    }
    int r = ml_derp_connect_step(ml, c);
    if (r == 1) {
      if (idx == hs) {
        s_home_retry_burst = 0;
        if (s_reconn_t0 != 0) {
          s_diag_derp_reconnects++;
          s_diag_derp_last_reconnect_ms = (uint32_t)(ml_get_time_ms() - s_reconn_t0);
          if (s_diag_derp_last_reconnect_ms > s_diag_derp_worst_reconnect_ms) {
            s_diag_derp_worst_reconnect_ms = s_diag_derp_last_reconnect_ms;
          }
          ESP_LOGW(TAG, "DERP home reconnected in %ums (async)", (unsigned)s_diag_derp_last_reconnect_ms);
          s_reconn_t0 = 0;
        }
        /* Re-validate the kick-time "no re-home owner" guard: wg_mgr/coord may
         * have learned a priority region (or an operator locked one) during
         * the async connect — their intent supersedes the fallback. */
        if (
          s_home_fb_pending != 0 && c->region_id == s_home_fb_pending && ml->derp_region_override == 0 &&
          ml->priority_peer_region == 0)
        {
          ml->derp_home_region = s_home_fb_pending;
          s_derp_home_unreachable_fallbacks++;
          ESP_LOGW(TAG, "DERP home fell back to reachable region %u", (unsigned)s_home_fb_pending);
        }
        s_home_fb_pending = 0;
      } else {
        aux_burst[idx] = 0; /* reset fast-retry on success */
        if (idx == s_rescue_slot) {
          s_derp_home_unreachable_fallbacks++; /* rescue aux CONFIRMED up */
          s_rescue_slot = -1;
        }
      }
    } else if (r == -1) {
      if (idx == hs) {
        s_home_fb_pending = 0; /* a failed fallback attempt must not apply later */
      }
      if (idx == s_rescue_slot) {
        s_rescue_slot = -1; /* rescue attempt died; a future pass may re-kick */
      }
    }
  }
}

void ml_derp_tx_task(void * arg)
{
  microlink_t * ml = (microlink_t *)arg;
  ESP_LOGI(TAG, "DERP I/O task started (Core %d)", xPortGetCoreID());

  uint32_t frames_rx = 0;
  uint32_t frames_tx = 0;
  uint64_t last_status_ms = 0;
  uint32_t loop_count = 0;
  uint64_t last_heartbeat_ms = 0;
  int aux_burst[ML_DERP_MAX_CONNS] = {0}; /* per-slot aux connect backoff state */

  while (!(xEventGroupGetBits(ml->events) & ML_EVT_SHUTDOWN_REQUEST)) {
    /* Runtime pause: skip the whole loop body, just yield. Mutexes /
         * sockets stay in their current state — caller can resume cleanly. */
    if (atomic_load(&s_derp_paused)) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    loop_count++;
    uint64_t loop_start = ml_get_time_ms();

    /* HOME connection = derp[derp_home_slot]. Re-resolved EVERY iteration
         * (it used to be a loop-invariant &derp[0]): a committed §7 MBB home
         * switch swaps the index mid-run, and every home-specific action below
         * (connect/reconnect/keepalive/fallback) must follow it immediately.
         * Only this task writes the index, so the read is race-free here. */
    ml_derp_conn_t * home = &ml->derp[ml->derp_home_slot];

    /* Unconditional heartbeat - proves task is alive */
    if (loop_start - last_heartbeat_ms > 5000) {
      int aux_up = 0;
      for (int s = 1; s < ML_DERP_MAX_CONNS; s++) {
        if (ml->derp[s].connected) aux_up++;
      }
      ESP_LOGI(
        TAG,
        "HEARTBEAT: loop=%lu home=%d aux=%d rx=%lu tx=%lu stack_free=%lu",
        (unsigned long)loop_count,
        home->connected,
        aux_up,
        (unsigned long)frames_rx,
        (unsigned long)frames_tx,
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));
      last_heartbeat_ms = loop_start;
    }

    /* ---- Periodic status logging (always, even when disconnected) ---- */
    {
      uint64_t now_ms = loop_start;
      if (now_ms - last_status_ms > 10000) {
        ESP_LOGI(
          TAG,
          "DERP status: home_connected=%d fd=%d region=%u rx=%lu tx=%lu loops=%lu",
          home->connected,
          home->sockfd,
          (unsigned)home->region_id,
          (unsigned long)frames_rx,
          (unsigned long)frames_tx,
          (unsigned long)loop_count);
        last_status_ms = now_ms;
      }
    }

    /* ---- Home connection (slot 0) connect/reconnect — serviced FIRST so the
         *      safety path is never starved by aux-region work. ---- */
    {
      EventBits_t bits = xEventGroupGetBits(ml->events);
      if ((bits & ML_EVT_DERP_CONNECT_REQ) && !home->connected) {
        xEventGroupClearBits(ml->events, ML_EVT_DERP_CONNECT_REQ);
        /* Stage-3: async kick — the stepper advances it every iteration and
         * the periodic-retry block below re-kicks on failure (5s/10s/60s),
         * so the old in-loop vTaskDelay retry ladder (which stalled rx for
         * seconds) is gone. Completion bookkeeping is in derp_drive_connects. */
        ESP_LOGI(TAG, "DERP connect requested, kicking async connect");
        (void)ml_derp_connect_kick(ml, home, ml_effective_home_region(ml));
      }

      /* ---- Periodic self-retry while disconnected ----
             * A failed 3-attempt burst used to leave DERP down until the
             * next coord reconnect re-fired CONNECT_REQ — with a stable
             * long-poll that can be never. Boot-time attempts can land in
             * a transiently hostile heap (TLS handshake stalls because
             * lwIP can't alloc RX pbufs); minutes later the heap has
             * settled and a retry succeeds. One attempt per minute. */
      {
        static uint64_t s_last_derp_retry_ms = 0;
        /* When a priority (safety) peer is configured, DERP is the
                 * failover path for its heartbeat — a 60 s reconnect wall is
                 * far too long. Retry fast for the first few attempts (5s,
                 * 5s, 10s) then fall back to the calm 60 s cadence so a
                 * genuinely-down DERP server isn't hammered. Non-priority
                 * builds keep the original 60 s. */
        bool has_prio = (ml->config.priority_peer_ip != 0);
        uint64_t retry_gap = 60000;
        if (has_prio) {
          retry_gap = (s_home_retry_burst < 2) ? 5000u : (s_home_retry_burst < 3) ? 10000u : 60000u;
        }
        if (
          !home->connected && home->cstate == DERP_CS_IDLE && ml->state == ML_STATE_CONNECTED &&
          loop_start - s_last_derp_retry_ms > retry_gap)
        {
          s_last_derp_retry_ms = loop_start;
          if (s_home_retry_burst < 100) {
            s_home_retry_burst++;
          }

          /* Home-region-unreachable escape: a chip with no re-home driver
           * (priority_peer_region==0) would otherwise retry a TCP-dead region
           * forever. After N consecutive failures, fall back to a PROVEN-
           * reachable region (a live aux conn is the reachability oracle).
           * A LOCK is never silently abandoned. */
          uint16_t home_region = ml_effective_home_region(ml);
          bool locked = (ml->derp_region_override != 0);
          bool try_fallback = (s_home_retry_burst >= ML_DERP_HOME_FALLBACK_AFTER);
          uint16_t fb = try_fallback ? derp_pick_fallback_region(ml, home_region) : 0;

          /* AUTO + no active re-home owner + a reachable alternative => actually
           * move the home. Writing derp_home_region is safe here precisely
           * BECAUSE priority_peer_region==0: nothing else (re-home) owns it, so
           * there is no cross-task ping-pong. eff_home + relay routing follow the
           * new region immediately; inbound PreferredDERP advert (default region)
           * is unaffected. */
          if (try_fallback && !locked && ml->priority_peer_region == 0 && fb != 0) {
            ESP_LOGW(
              TAG,
              "DERP home region %u unreachable after %d tries; falling back to reachable region %u",
              (unsigned)home_region,
              s_home_retry_burst,
              (unsigned)fb);
            s_home_fb_pending = fb; /* applied by derp_drive_connects on completion */
            if (ml_derp_connect_kick(ml, home, fb) != ESP_OK) {
              s_home_fb_pending = 0; /* sync failure: no step will ever clear it */
            }
          } else {
            /* Default path: (re)try the INTENDED home. This is what honours an
             * operator LOCK and an active re-home owner — we never abandon
             * either. */
            ESP_LOGI(TAG, "DERP periodic retry (disconnected, burst=%d)", s_home_retry_burst);
            (void)ml_derp_connect_kick(ml, home, home_region);
            if (try_fallback && fb != 0) {
              /* Home still down, and we deliberately did NOT move it (locked, or a
               * re-home owns the region). Keep retrying the intended home above,
               * but if the ENTIRE pool is dark the chip has no relay path at all —
               * open ONE rescue aux on the reachable region so it is not fully
               * offline. The lock/advert is untouched; derp_manage_aux reaps this
               * aux once home recovers or a real safety region needs the slot. */
              bool any_up = false;
              for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
                if (ml->derp[s].connected) {
                  any_up = true;
                  break;
                }
              }
              int slot = (any_up || s_rescue_slot >= 0) ? -1 : derp_free_aux_slot(ml);
              if (slot >= 0) { /* slot 0 is a legit aux once home migrated off it */
                ESP_LOGW(
                  TAG,
                  "DERP home %u unreachable + pool dark; kicking rescue aux slot %d region %u (home retry retained)",
                  (unsigned)home_region,
                  slot,
                  (unsigned)fb);
                if (ml_derp_connect_kick(ml, &ml->derp[slot], fb) == ESP_OK) {
                  s_rescue_slot = slot; /* counter increments on confirmed completion */
                }
              }
            }
          }
        }
      }
      if (bits & ML_EVT_DERP_RECONNECT) {
        xEventGroupClearBits(ml->events, ML_EVT_DERP_RECONNECT);
        ESP_LOGW(TAG, "DERP reconnect requested (home was %s)", home->connected ? "connected" : "disconnected");
        uint64_t reconn_t0 = ml_get_time_ms();
        ml_derp_disconnect(ml, home);
        /* An RST/EOF here is usually a middlebox conn-table flush, not a down
         * server, and the same flush kills the direct path — so kick an async
         * reconnect IMMEDIATELY (any pre-delay eats into the 2 s pstop
         * timeout). Failures re-kick via the periodic 5s/10s/60s cadence;
         * reconnect diag is recorded on completion via s_reconn_t0. */
        s_reconn_t0 = reconn_t0;
        s_home_fb_pending = 0; /* an external reconnect supersedes the fallback intent */
        (void)ml_derp_connect_kick(ml, home, ml_effective_home_region(ml));
      }
    }

    /* ---- Stage-2/3: advance every in-progress connect one bounded step. ---- */
    derp_drive_connects(ml, aux_burst);

    /* Pool dark => skip Phase 1/2 (frames wait in their queues); the shared
     * tail below still runs aux management and keeps the §7 MBB machine
     * abortable, and the delay at the bottom shortens to keep stepping. */
    bool any_connected = false;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (ml->derp[s].connected) {
        any_connected = true;
        break;
      }
    }

    /* ---- Phase 1: Drain TX queue FIRST (prioritize outgoing), routing each
         *      frame to the pool connection homed on the destination's region. ---- */
    if (any_connected) {
      /* Drain the PRIORITY (safety/handshake) queue FULLY first, then the disco
       * queue (bounded). Two separate FIFO queues => a 5 Hz safety heartbeat is
       * never stuck behind a disco burst, enqueue stays single-atomic per queue
       * (no cross-thread drain/requeue = the CRITICAL race is gone), and per-peer
       * frame order is preserved within a queue (no OK-after-STOP reordering). */
      for (int q = 0; q < 2; q++) {
        QueueHandle_t txq = (q == 0) ? ml->derp_tx_prio_queue : ml->derp_tx_queue;
        int budget = (q == 0) ? ML_DERP_TX_PRIO_DEPTH : 8;
        for (int tx_count = 0; tx_count < budget; tx_count++) {
          ml_derp_tx_item_t item;
          if (xQueueReceive(txq, &item, 0) != pdTRUE) {
            break;
          }
          ml_derp_conn_t * c = derp_route_conn(ml, item.region_id, !item.leg2);
          if (item.leg2) {
            /* Path-diversity mirror: needs a conn DISTINCT from the primary
             * leg's route. No distinct conn (or primary itself down) = skip
             * silently — the mirror is best-effort by design. */
            c = derp_route_leg2(ml, c);
            if (c == NULL) {
              s_diag_leg2_skipped++;
              free(item.data);
              continue;
            }
            s_diag_leg2_sent++;
          } else if (c == NULL) {
            /* Primary route down — rescue onto ANY connected pool conn
             * (the peer may hold a pool conn on that region too) instead of
             * the old silent drop; only drop when the whole pool is dark. */
            c = derp_route_any(ml);
            if (c == NULL) {
              ESP_LOGW(TAG, "DERP TX drop: pool dark for region %u", (unsigned)item.region_id);
              free(item.data);
              continue;
            }
            s_diag_rescue_routes++;
          }
          int ret;
          if (item.frame_type == DERP_FRAME_SEND_PACKET) {
            ESP_LOGI(
              TAG,
              "DERP TX: SendPacket %d bytes, dest=%02x%02x%02x%02x, region=%u, hdr=%02x",
              (int)item.len,
              item.dest_pubkey[0],
              item.dest_pubkey[1],
              item.dest_pubkey[2],
              item.dest_pubkey[3],
              (unsigned)item.region_id,
              item.data[0]);
            ret = derp_send_packet(ml, c, item.dest_pubkey, item.data, item.len);
          } else {
            ret = derp_write_frame(ml, c, item.frame_type, item.data, item.len);
          }
          if (ret < 0) {
            ESP_LOGW(
              TAG, "DERP write failed on %s conn (region %u)", (c == home) ? "home" : "aux", (unsigned)c->region_id);
            derp_reap_conn(ml, c, 0);
          } else {
            frames_tx++;
            c->last_used_ms = ml_get_time_ms();
          }
          free(item.data);
        }
      }

      /* Stage-0 gauge: gap between consecutive rx-poll passes (the quantity the
       * async engine exists to bound). Tracked only while home is connected. */
      {
        static uint64_t s_last_rx_poll_ms = 0;
        uint64_t rx_now = ml_get_time_ms();
        if (home->connected && s_last_rx_poll_ms != 0) {
          uint32_t gap = (uint32_t)(rx_now - s_last_rx_poll_ms);
          if (gap > s_diag_rx_gap_worst_ms) s_diag_rx_gap_worst_ms = gap;
          s_pass_rx_gap = (gap > 0xFFFF) ? 0xFFFF : (uint16_t)gap;
        } else {
          s_pass_rx_gap = 0; /* not measured this pass — a pool-dark stall
                              * event must not carry a pre-disconnect gap */
        }
        s_last_rx_poll_ms = rx_now;
      }

      /* ---- Phase 2: Poll for incoming DERP frames from ALL connected conns,
         *      HOME first/most so the safety path stays prompt. Iteration starts
         *      at the home INDEX (not slot 0) since the MBB refactor. Sockets
         *      are O_NONBLOCK so each idle poll returns immediately. ---- */
      for (int k = 0; k < ML_DERP_MAX_CONNS; k++) {
        int s = (ml->derp_home_slot + k) % ML_DERP_MAX_CONNS;
        ml_derp_conn_t * c = &ml->derp[s];
        if (!c->connected) continue;
        for (int burst = 0; burst < 4; burst++) {
          int ret = poll_derp_read(ml, c);
          if (ret > 0) {
            frames_rx++;
            c->last_recv_ms = ml_get_time_ms();
          } else if (ret == 0) {
            break; /* No more data / timeout */
          } else {
            ESP_LOGW(
              TAG,
              "DERP read error %d on %s conn (region %u)",
              ret,
              (c == home) ? "home" : "aux",
              (unsigned)c->region_id);
            derp_reap_conn(ml, c, 1);
            break;
          }
        }
      }
    } /* any_connected: Phase 1 + rx-gap gauge + Phase 2 */
    else
    {
      s_pass_rx_gap = 0; /* pool dark: a stall event recorded this pass must not
                          * carry the last connected pass's gap */
    }

    /* rx-staleness reap, HOME conn only: a conn we actively egress into that
     * has returned NOTHING for ML_DERP_RX_STALE_MS is server-side black-holed
     * or a half-dead TCP (a quiet registration loss TX'd replies into a
     * denying server for 25 min with every local gauge clean). Cycle it; the
     * reconnect re-auths against the (re-registered) control plane. Aux conns
     * are exempt — they are egress-only by construction (peers reply toward
     * OUR home region; dual_path mirrors keep an aux tx-fresh with no rx ever
     * owed), so this test would reap a healthy aux every ~150 s. Idle conns
     * are exempt — nothing is owed back on them. */
    {
      uint64_t rnow = ml_get_time_ms();
      if (
        home->connected && home->last_relay_rx_ms != 0 && rnow - home->last_relay_rx_ms > ML_DERP_RX_STALE_MS &&
        home->last_used_ms != 0 && rnow - home->last_used_ms <= ML_DERP_RX_STALE_TX_ACTIVE_MS)
      {
        s_diag_rx_stale_reaps++;
        ESP_LOGW(
          TAG,
          "DERP home conn (region %u) tx-active but relay-rx-silent %us — reaping",
          (unsigned)home->region_id,
          (unsigned)((rnow - home->last_relay_rx_ms) / 1000));
        derp_reap_conn(ml, home, 2);
      }
    }

    /* ---- Aux pool management at the TAIL: home is already serviced this
         *      iteration, so an aux connect kicked here can't starve the home
         *      slot / the safety path. Runs pool-dark too (it opens conns). ---- */
    derp_manage_aux(ml, ml_effective_home_region(ml), aux_burst, ml_get_time_ms());

    /* ---- §7 MBB home-switch machine, after manage_aux so a just-connected
         *      target aux is observed the same iteration. O(1) state checks —
         *      every blocking connect above is already capped, MBB adds none. ---- */
    derp_mbb_tick(ml, ml_get_time_ms());

    /* Stage-0 gauge: worst single iteration wall-clock + per-EVENT ring
     * entry (home conn phase separates blocking-DNS from rx backlog). */
    {
      uint32_t iter_ms = (uint32_t)(ml_get_time_ms() - loop_start);
      if (iter_ms > s_diag_max_iter_ms) s_diag_max_iter_ms = iter_ms;
      if (iter_ms > ML_STALL_THRESH_MS) {
        uint32_t w = s_derp_stall_widx;
        ml_derp_stall_event_t * e = &s_derp_stall_ring[w % ML_STALL_RING_LEN];
        e->at_s = (uint32_t)(loop_start / 1000);
        e->dur_ms = iter_ms;
        e->home_cstate = (uint8_t)ml->derp[ml->derp_home_slot].cstate;
        e->last_dns_ms = s_diag_last_dns_ms;
        e->rx_gap_ms = s_pass_rx_gap;
        __atomic_store_n(&s_derp_stall_widx, w + 1, __ATOMIC_RELEASE);
        ESP_LOGW(
          TAG,
          "DERP loop stall %ums (cstate=%u dns=%ums rx_gap=%ums)",
          (unsigned)iter_ms,
          e->home_cstate,
          e->last_dns_ms,
          e->rx_gap_ms);
      }
    }

    /* Yield briefly (runtime-tunable via microlink_set_derp_loop_delay_ms).
     * Pool dark: 20 ms while a connect is in flight (keep stepping briskly),
     * 100 ms otherwise. */
    uint32_t delay_ms = atomic_load(&s_derp_loop_delay_ms);
    if (!any_connected) {
      bool connecting = false;
      for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
        if (ml->derp[s].cstate != DERP_CS_IDLE) {
          connecting = true;
          break;
        }
      }
      delay_ms = connecting ? 20 : 100;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }

  ESP_LOGI(TAG, "DERP I/O task exiting");
  vTaskDelete(NULL);
}

esp_err_t microlink_set_derp_loop_delay_ms(int ms)
{
  if (ms < 1 || ms > 100) return ESP_ERR_INVALID_ARG;
  atomic_store(&s_derp_loop_delay_ms, ms);
  return ESP_OK;
}

int microlink_get_derp_loop_delay_ms(void)
{
  return atomic_load(&s_derp_loop_delay_ms);
}

void microlink_pause_derp(bool paused)
{
  atomic_store(&s_derp_paused, paused ? 1 : 0);
}

bool microlink_is_derp_paused(void)
{
  return atomic_load(&s_derp_paused) != 0;
}

/* ============================================================================
 * DERP Connection Management (called from coord task)
 * ========================================================================== */

/* ============================================================================
 * Stage-2/3 async connect engine (docs/NONBLOCKING_DERP_TLS_PLAN.md §2).
 *
 * The former monolithic ml_derp_connect() blocked the DERP I/O task for the
 * whole DNS→TCP→TLS→HTTP-upgrade→DERP-handshake sequence (1-2 s typical,
 * ~20-28 s against a NAT-churned half-open server — the wire-measured
 * 2026-08-13 delivery hole). It is replaced by:
 *
 *   ml_derp_connect_kick(ml, c, region)  — start a connect: teardown-if-live,
 *     resolve (per-region addr cache → DNS), create an O_NONBLOCK socket,
 *     begin connect(). Bounded: the only residual blocking is a DNS cache
 *     MISS (rare; lwIP resolver, normally ms).
 *   ml_derp_connect_step(ml, c)          — advance ONE bounded action:
 *     1 = became CONNECTED this call, 0 = in progress, -1 = failed (cleaned
 *     up; cstate back to IDLE; per-slot backoff owned by the callers).
 *
 * The task loop drives step() for every in-progress conn each iteration, so
 * a (re)connect — INCLUDING the home conn's own after an edge-flush RST —
 * can never starve the rx poll of connected conns or the direct-path
 * heartbeat. Single-task ownership is unchanged: kick/step run only on the
 * DERP I/O task; no locking (file-header invariant).
 * ========================================================================== */

/* Effective lookup region for a connect: 0 means "the compiled default"
 * (one definition, formerly inlined). */
static inline uint16_t derp_lookup_region(uint16_t region_id)
{
  return region_id ? region_id : ML_DERP_REGION;
}

/* Per-region resolved-address cache: skips blocking DNS on reconnect to the
 * same region (risk #7 in the plan). Invalidated when a connect that used the
 * cached address fails — the next kick does fresh DNS. DERP-task-owned. */
typedef struct
{
  uint16_t region;
  struct sockaddr_storage addr;
  socklen_t addrlen;
  int family;
  uint8_t post_tcp_fails; /* consecutive TLS/HTTP/DERP failures on this addr */
} derp_dns_cache_t;

static derp_dns_cache_t s_dns_cache[ML_DERP_MAX_CONNS + 2];

static derp_dns_cache_t * derp_dns_cache_find(uint16_t region)
{
  for (size_t i = 0; i < sizeof(s_dns_cache) / sizeof(s_dns_cache[0]); i++) {
    if (s_dns_cache[i].region == region && s_dns_cache[i].addrlen > 0) return &s_dns_cache[i];
  }
  return NULL;
}

static void derp_dns_cache_put(uint16_t region, const struct sockaddr * sa, socklen_t len, int family)
{
  derp_dns_cache_t * e = derp_dns_cache_find(region);
  if (!e) {
    for (size_t i = 0; i < sizeof(s_dns_cache) / sizeof(s_dns_cache[0]); i++) {
      if (s_dns_cache[i].addrlen == 0) {
        e = &s_dns_cache[i];
        break;
      }
    }
  }
  if (!e) e = &s_dns_cache[0]; /* full: overwrite deterministically */
  e->region = region;
  memcpy(&e->addr, sa, len);
  e->addrlen = len;
  e->family = family;
  e->post_tcp_fails = 0;
}

/* §16 stepper telemetry (DERP-task-owned; word-sized cross-task reads). */
static uint32_t s_diag_cs_steps; /* connect-engine steps executed */
static uint32_t s_diag_cs_fails; /* connects that ended in failure */

uint32_t ml_derp_get_connect_steps(void)
{
  return s_diag_cs_steps;
}

/* Host/port for a region from the parsed DERPMap; compiled default fallback. */
static void derp_resolve_region_host(microlink_t * ml, uint16_t lookup_region, const char ** host_out, int * port_out)
{
  const char * derp_host = ML_DERP_HOST;
  int derp_port = ML_DERP_PORT;
  if (ml->derp_region_count > 0 && lookup_region > 0) {
    for (int i = 0; i < ml->derp_region_count; i++) {
      if (ml->derp_regions[i].region_id == lookup_region) {
        for (int n = 0; n < ml->derp_regions[i].node_count; n++) {
          if (!ml->derp_regions[i].nodes[n].stun_only && ml->derp_regions[i].nodes[n].hostname[0]) {
            derp_host = ml->derp_regions[i].nodes[n].hostname;
            if (ml->derp_regions[i].nodes[n].derp_port > 0) {
              derp_port = ml->derp_regions[i].nodes[n].derp_port;
            }
            break;
          }
        }
        break;
      }
    }
  }
  *host_out = derp_host;
  *port_out = derp_port;
}

/* Single cleanup point for an in-progress connect (mirrors the old
 * DERP_CONNECT_FAIL_CLEANUP + the 2026-05-25 leak-audit rules): frees the 4
 * mbedTLS contexts exactly once, the socket, any pending tx buffer, and (if
 * this attempt rode the addr cache) invalidates the cache entry so the next
 * kick re-resolves. cstate returns to IDLE; last_connect_attempt_ms is left
 * as set by kick so the existing per-slot backoffs apply. */
static void derp_cs_fail(microlink_t * ml, ml_derp_conn_t * c, const char * why)
{
  ESP_LOGE(
    TAG,
    "DERP connect FAILED (%s) region=%u slot=%s state=%d",
    why,
    (unsigned)c->cs_region,
    derp_conn_is_home(ml, c) ? "home" : "aux",
    (int)c->cstate);
  if (c->tls_inited) {
    mbedtls_ssl_free(&c->ssl);
    mbedtls_ssl_config_free(&c->ssl_conf);
    mbedtls_ctr_drbg_free(&c->ctr_drbg);
    mbedtls_entropy_free(&c->entropy);
    c->tls_inited = false;
  }
  if (c->sockfd >= 0) {
    ml_close_sock(c->sockfd);
    c->sockfd = -1;
  }
  if (c->cs_tx_buf) {
    free(c->cs_tx_buf);
    c->cs_tx_buf = NULL;
  }
  if (c->cs_used_dns_cache) {
    /* At/before TCP connect: the address itself failed — invalidate now.
     * Post-TCP (TLS/HTTP/DERP): the address was reachable, so keep it —
     * re-resolving would put blocking DNS back into storm-time reconnects —
     * EXCEPT after 3 consecutive post-TCP failures: a re-IP'd server can
     * accept TCP and fail TLS forever, so force one fresh resolve. */
    derp_dns_cache_t * e = derp_dns_cache_find(derp_lookup_region(c->cs_region));
    if (e) {
      if (c->cstate <= DERP_CS_TCP_CONNECT || ++e->post_tcp_fails >= 3) e->addrlen = 0;
    }
  }
  c->connected = false;
  c->region_id = c->cs_region; /* keep the slot's target so retries find it (old aux semantics) */
  c->cstate = DERP_CS_IDLE;
  s_diag_cs_fails++;
}

static uint64_t s_last_connect_kick_ms;
static uint32_t s_diag_kicks_spaced;

uint32_t ml_derp_get_kicks_spaced(void)
{
  return s_diag_kicks_spaced;
}

esp_err_t ml_derp_connect_kick(microlink_t * ml, ml_derp_conn_t * c, uint16_t region_id)
{
  /* Flush-storm spacing (see ML_DERP_CONNECT_SPACING_MS): refuse BEFORE the
   * teardown so a live conn is never sacrificed for a kick we won't start.
   * The HOME slot is exempt from refusal — ML_EVT_DERP_RECONNECT re-kicks it
   * immediately to beat the 2 s heartbeat timeout, and a silent refusal there
   * would defer the safety path 5-60 s (review 🔴). Home kicks still stamp
   * the window so aux kicks cannot stack TLS work right behind home's. */
  uint64_t know = ml_get_time_ms();
  bool kick_is_home = derp_conn_is_home(ml, c);
  if (!kick_is_home && s_last_connect_kick_ms != 0 && know - s_last_connect_kick_ms < ML_DERP_CONNECT_SPACING_MS) {
    s_diag_kicks_spaced++;
    return ESP_ERR_NOT_FINISHED;
  }
  s_last_connect_kick_ms = know;

  /* Idempotency guard (2026-05-25 leak class): never re-init over live
   * contexts or a half-done connect. Tear down first. */
  if (c->tls_inited || c->cstate != DERP_CS_IDLE) {
    ml_derp_disconnect(ml, c); /* frees cs_tx_buf + resets cstate itself */
  }

  uint16_t lookup_region = derp_lookup_region(region_id);
  c->cs_region = region_id;
  c->cs_used_dns_cache = false;
  c->cstate_deadline_ms = ml_get_time_ms() + DERP_CONNECT_TIMEOUT_MS;
  c->cs_http_len = 0;
  c->cs_derp_step = 0;
  c->cs_hdr_got = 0;
  c->cs_payload_got = 0;
  c->cs_tx_sent = 0;

  struct sockaddr_storage addr;
  socklen_t addrlen = 0;
  int family = AF_INET;

  derp_dns_cache_t * cached = derp_dns_cache_find(lookup_region);
  if (cached) {
    memcpy(&addr, &cached->addr, cached->addrlen);
    addrlen = cached->addrlen;
    family = cached->family;
    c->cs_used_dns_cache = true;
  } else {
    const char * derp_host;
    int derp_port;
    derp_resolve_region_host(ml, lookup_region, &derp_host, &derp_port);
    struct addrinfo hints = {.ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM};
    struct addrinfo * res = NULL;
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", derp_port);
    int64_t t0 = esp_timer_get_time();
    if (ml_getaddrinfo(derp_host, port_str, &hints, &res) != 0 || !res) {
      ESP_LOGE(TAG, "DNS resolve failed for %s", derp_host);
      derp_cs_fail(ml, c, "dns");
      return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[TIMING-DERP] DNS: %lld ms (host=%s)", (esp_timer_get_time() - t0) / 1000, derp_host);
    {
      int64_t dns_ms = (esp_timer_get_time() - t0) / 1000;
      s_diag_last_dns_ms = (dns_ms > 0xFFFF) ? 0xFFFF : (uint16_t)dns_ms;
    }
    memcpy(&addr, res->ai_addr, res->ai_addrlen);
    addrlen = res->ai_addrlen;
    family = res->ai_family;
    ml_freeaddrinfo(res);
    derp_dns_cache_put(lookup_region, (struct sockaddr *)&addr, addrlen, family);
  }

  int sock = ml_socket(family, SOCK_STREAM, 0);
  if (sock < 0) {
    derp_cs_fail(ml, c, "socket");
    return ESP_FAIL;
  }
  /* O_NONBLOCK from birth: connect() returns EINPROGRESS and every later
   * read/write returns immediately — the stepper never waits in a syscall. */
  int flags = ml_fcntl(sock, F_GETFL, 0);
  if (flags >= 0) {
    ml_fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }
  c->sockfd = sock;
  if (ml_connect(sock, (struct sockaddr *)&addr, addrlen) < 0 && errno != EINPROGRESS && errno != EWOULDBLOCK) {
    derp_cs_fail(ml, c, "connect");
    return ESP_FAIL;
  }
  c->cstate = DERP_CS_TCP_CONNECT;
  ESP_LOGI(
    TAG,
    "DERP connect kicked: region %u slot=%s%s",
    (unsigned)lookup_region,
    derp_conn_is_home(ml, c) ? "home" : "aux",
    c->cs_used_dns_cache ? " (cached addr)" : "");
  return ESP_OK;
}

/* Resumable exact-read: one mbedtls_ssl_read of the REMAINING bytes.
 * Returns 1 complete, 0 yield (no data yet), -1 error/peer-close. */
static int derp_cs_read(ml_derp_conn_t * c, uint8_t * buf, uint32_t want, uint32_t * got)
{
  if (*got >= want) return 1;
  int n = mbedtls_ssl_read(&c->ssl, buf + *got, want - *got);
  if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_TIMEOUT) {
    return 0;
  }
  if (n <= 0) return -1;
  *got += (uint32_t)n;
  return (*got >= want) ? 1 : 0;
}

/* Advance one bounded action of an in-progress connect. See engine header. */
int ml_derp_connect_step(microlink_t * ml, ml_derp_conn_t * c)
{
  if (c->cstate == DERP_CS_IDLE) return c->connected ? 1 : -1;
  s_diag_cs_steps++;
  bool is_home = derp_conn_is_home(ml, c);
  uint64_t now = ml_get_time_ms();

  /* Whole-connect deadline. Past the ClientInfo send (cs_derp_step >= 3) the
   * session is functionally up and the old code tolerated a missing
   * ServerInfo ("continuing anyway") — complete instead of failing. */
  if (now > c->cstate_deadline_ms) {
    if (c->cstate == DERP_CS_DERP_HS && c->cs_derp_step >= 3) {
      ESP_LOGW(TAG, "No ServerInfo frame (continuing anyway)");
      goto complete;
    }
    derp_cs_fail(ml, c, "deadline");
    return -1;
  }

  switch (c->cstate) {
    case DERP_CS_TCP_CONNECT: {
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(c->sockfd, &wfds);
      struct timeval zero = {0, 0};
      int r = ml_select_fds(c->sockfd + 1, NULL, &wfds, NULL, &zero);
      if (r < 0) {
        derp_cs_fail(ml, c, "select");
        return -1;
      }
      if (r == 0 || !FD_ISSET(c->sockfd, &wfds)) return 0; /* still connecting */
      int soerr = 0;
      socklen_t slen = sizeof(soerr);
      getsockopt(c->sockfd, SOL_SOCKET, SO_ERROR, &soerr, &slen);
      if (soerr != 0) {
        ESP_LOGE(TAG, "TCP connect failed: %d", soerr);
        derp_cs_fail(ml, c, "tcp");
        return -1;
      }
      /* TLS setup (same contexts/config as the old blocking path; the leak
       * rules from the 2026-05-25 audit are enforced by derp_cs_fail). */
      mbedtls_ssl_init(&c->ssl);
      mbedtls_ssl_config_init(&c->ssl_conf);
      mbedtls_entropy_init(&c->entropy);
      mbedtls_ctr_drbg_init(&c->ctr_drbg);
      c->tls_inited = true;
      mbedtls_ctr_drbg_seed(&c->ctr_drbg, mbedtls_entropy_func, &c->entropy, NULL, 0);
      mbedtls_ssl_config_defaults(
        &c->ssl_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
      mbedtls_ssl_conf_authmode(&c->ssl_conf, MBEDTLS_SSL_VERIFY_NONE);
      mbedtls_ssl_conf_rng(&c->ssl_conf, mbedtls_ctr_drbg_random, &c->ctr_drbg);
      /* Socket is O_NONBLOCK: the BIO returns immediately with no data; this
       * timeout only shapes the SO_RCVTIMEO the BIO applies (moot). */
      mbedtls_ssl_conf_read_timeout(&c->ssl_conf, 100);
      mbedtls_ssl_setup(&c->ssl, &c->ssl_conf);
      {
        const char * host;
        int port;
        derp_resolve_region_host(ml, derp_lookup_region(c->cs_region), &host, &port);
        mbedtls_ssl_set_hostname(&c->ssl, host);
      }
      mbedtls_ssl_set_bio(&c->ssl, &c->sockfd, ml_derp_bio_send, NULL, ml_derp_bio_recv_timeout);
      c->cstate = DERP_CS_TLS_HANDSHAKE;
      return 0;
    }

    case DERP_CS_TLS_HANDSHAKE: {
      int ret = mbedtls_ssl_handshake(&c->ssl);
      if (ret == 0) {
        /* Build the HTTP upgrade request (sent incrementally next state). */
        const char * host;
        int port;
        derp_resolve_region_host(ml, derp_lookup_region(c->cs_region), &host, &port);
        c->cs_tx_buf = malloc(256);
        if (!c->cs_tx_buf) {
          derp_cs_fail(ml, c, "oom");
          return -1;
        }
        c->cs_tx_len = (uint32_t)snprintf(
          (char *)c->cs_tx_buf,
          256,
          "GET /derp HTTP/1.1\r\n"
          "Host: %s\r\n"
          "Connection: Upgrade\r\n"
          "Upgrade: DERP\r\n"
          "\r\n",
          host);
        c->cs_tx_sent = 0;
        c->cs_http_len = 0;
        c->cstate = DERP_CS_HTTP_UPGRADE;
        return 0;
      }
      if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE || ret == MBEDTLS_ERR_SSL_TIMEOUT) {
        return 0; /* mbedTLS resumes cleanly on a stream — next iteration */
      }
      {
        char err_buf[96];
        mbedtls_strerror(ret, err_buf, sizeof(err_buf));
        ESP_LOGE(TAG, "TLS handshake failed: %s", err_buf);
      }
      derp_cs_fail(ml, c, "tls");
      return -1;
    }

    case DERP_CS_HTTP_UPGRADE: {
      /* Send phase (resumable). */
      if (c->cs_tx_buf && c->cs_tx_sent < c->cs_tx_len) {
        int w = mbedtls_ssl_write(&c->ssl, c->cs_tx_buf + c->cs_tx_sent, c->cs_tx_len - c->cs_tx_sent);
        if (w == MBEDTLS_ERR_SSL_WANT_READ || w == MBEDTLS_ERR_SSL_WANT_WRITE) return 0;
        if (w < 0) {
          derp_cs_fail(ml, c, "http-send");
          return -1;
        }
        c->cs_tx_sent += (uint32_t)w;
        if (c->cs_tx_sent < c->cs_tx_len) return 0;
        free(c->cs_tx_buf);
        c->cs_tx_buf = NULL;
      }
      /* Receive phase: up to 64 bytes per step, byte-at-a-time so we never
       * over-read into the DERP binary stream (v1 rule preserved). */
      for (int k = 0; k < 64; k++) {
        if (c->cs_http_len >= sizeof(c->cs_buf) - 1) {
          derp_cs_fail(ml, c, "http-overflow");
          return -1;
        }
        int n = mbedtls_ssl_read(&c->ssl, c->cs_buf + c->cs_http_len, 1);
        if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_TIMEOUT) {
          return 0;
        }
        if (n <= 0) {
          derp_cs_fail(ml, c, "http-closed");
          return -1;
        }
        c->cs_http_len++;
        if (
          c->cs_http_len >= 4 && c->cs_buf[c->cs_http_len - 4] == '\r' && c->cs_buf[c->cs_http_len - 3] == '\n' &&
          c->cs_buf[c->cs_http_len - 2] == '\r' && c->cs_buf[c->cs_http_len - 1] == '\n')
        {
          c->cs_buf[c->cs_http_len] = '\0';
          if (strstr((char *)c->cs_buf, "101") == NULL) {
            ESP_LOGE(TAG, "DERP upgrade rejected: %.100s", (char *)c->cs_buf);
            derp_cs_fail(ml, c, "http-reject");
            return -1;
          }
          ESP_LOGI(TAG, "HTTP 101 Switching Protocols received");
          c->cs_derp_step = 0;
          c->cs_hdr_got = 0;
          c->cstate = DERP_CS_DERP_HS;
          return 0;
        }
      }
      return 0; /* budget spent; resume next iteration */
    }

    case DERP_CS_DERP_HS: {
      static const uint8_t DERP_MAGIC[8] = {0x44, 0x45, 0x52, 0x50, 0xf0, 0x9f, 0x94, 0x91};
      switch (c->cs_derp_step) {
        case 0: { /* ServerKey frame header */
          int r = derp_cs_read(c, c->cs_hdr, 5, &c->cs_hdr_got);
          if (r == 0) return 0;
          if (r < 0) {
            derp_cs_fail(ml, c, "skey-hdr");
            return -1;
          }
          c->cs_frame_type = c->cs_hdr[0];
          c->cs_frame_len = ((uint32_t)c->cs_hdr[1] << 24) | ((uint32_t)c->cs_hdr[2] << 16) |
                            ((uint32_t)c->cs_hdr[3] << 8) | c->cs_hdr[4];
          if (c->cs_frame_type != DERP_FRAME_SERVER_KEY || c->cs_frame_len < 40 || c->cs_frame_len > sizeof(c->cs_buf))
          {
            ESP_LOGE(
              TAG, "Expected ServerKey frame, got 0x%02x len=%lu", c->cs_frame_type, (unsigned long)c->cs_frame_len);
            derp_cs_fail(ml, c, "skey-frame");
            return -1;
          }
          c->cs_payload_got = 0;
          c->cs_derp_step = 1;
          return 0;
        }
        case 1: { /* ServerKey payload: magic(8) + server_key(32) [+ extra] */
          int r = derp_cs_read(c, c->cs_buf, c->cs_frame_len, &c->cs_payload_got);
          if (r == 0) return 0;
          if (r < 0) {
            derp_cs_fail(ml, c, "skey-payload");
            return -1;
          }
          if (memcmp(c->cs_buf, DERP_MAGIC, 8) != 0) {
            derp_cs_fail(ml, c, "magic");
            return -1;
          }
          /* Build ClientInfo: [nodekey(32)][nonce(24)][nacl_box(JSON)] as a
           * COMPLETE frame (5-byte header + payload) for resumable writes. */
          const uint8_t * derp_server_key = c->cs_buf + 8;
          const char * ci_json = "{\"Version\":2,\"CanAckPings\":true,\"IsProber\":false}";
          size_t json_len = strlen(ci_json);
          uint8_t nonce[NACL_BOX_NONCEBYTES];
          esp_fill_random(nonce, NACL_BOX_NONCEBYTES);
          size_t ct_len = json_len + NACL_BOX_MACBYTES;
          size_t payload_len = 32 + NACL_BOX_NONCEBYTES + ct_len;
          c->cs_tx_len = (uint32_t)(5 + payload_len);
          c->cs_tx_buf = malloc(c->cs_tx_len);
          if (!c->cs_tx_buf) {
            derp_cs_fail(ml, c, "oom");
            return -1;
          }
          uint8_t * p = c->cs_tx_buf;
          p[0] = DERP_FRAME_CLIENT_INFO;
          p[1] = (uint8_t)(payload_len >> 24);
          p[2] = (uint8_t)(payload_len >> 16);
          p[3] = (uint8_t)(payload_len >> 8);
          p[4] = (uint8_t)payload_len;
          memcpy(p + 5, ml->wg_public_key, 32);
          memcpy(p + 5 + 32, nonce, NACL_BOX_NONCEBYTES);
          if (
            nacl_box(
              p + 5 + 32 + NACL_BOX_NONCEBYTES,
              (const uint8_t *)ci_json,
              json_len,
              nonce,
              derp_server_key,
              ml->wg_private_key) != 0)
          {
            derp_cs_fail(ml, c, "naclbox");
            return -1;
          }
          c->cs_tx_sent = 0;
          c->cs_derp_step = 2;
          return 0;
        }
        case 2: { /* ClientInfo send (resumable) */
          int w = mbedtls_ssl_write(&c->ssl, c->cs_tx_buf + c->cs_tx_sent, c->cs_tx_len - c->cs_tx_sent);
          if (w == MBEDTLS_ERR_SSL_WANT_READ || w == MBEDTLS_ERR_SSL_WANT_WRITE) return 0;
          if (w < 0) {
            derp_cs_fail(ml, c, "ci-send");
            return -1;
          }
          c->cs_tx_sent += (uint32_t)w;
          if (c->cs_tx_sent < c->cs_tx_len) return 0;
          free(c->cs_tx_buf);
          c->cs_tx_buf = NULL;
          ESP_LOGI(TAG, "ClientInfo sent");
          c->cs_hdr_got = 0;
          c->cs_derp_step = 3;
          return 0;
        }
        case 3: { /* ServerInfo frame header (tolerated-missing via deadline) */
          int r = derp_cs_read(c, c->cs_hdr, 5, &c->cs_hdr_got);
          if (r == 0) return 0;
          if (r < 0) {
            ESP_LOGW(TAG, "No ServerInfo frame (continuing anyway)");
            goto complete;
          }
          c->cs_frame_type = c->cs_hdr[0];
          c->cs_frame_len = ((uint32_t)c->cs_hdr[1] << 24) | ((uint32_t)c->cs_hdr[2] << 16) |
                            ((uint32_t)c->cs_hdr[3] << 8) | c->cs_hdr[4];
          if (c->cs_frame_type != DERP_FRAME_SERVER_INFO || c->cs_frame_len == 0) {
            goto complete; /* something else already streaming — fine */
          }
          c->cs_payload_got = 0;
          c->cs_derp_step = 4;
          return 0;
        }
        case 4: { /* ServerInfo payload: read + discard in cs_buf chunks */
          if (c->cs_frame_len > 16384u) {
            ESP_LOGW(TAG, "ServerInfo oversized (%lu) — skipping via deadline path", (unsigned long)c->cs_frame_len);
            goto complete; /* session already usable; don't chew an absurd frame */
          }
          int chunks = 0;
          while (c->cs_payload_got < c->cs_frame_len) {
            if (++chunks > 8) return 0; /* bound per-call work */
            uint32_t remain = c->cs_frame_len - c->cs_payload_got;
            uint32_t chunk = remain > sizeof(c->cs_buf) ? (uint32_t)sizeof(c->cs_buf) : remain;
            int n = mbedtls_ssl_read(&c->ssl, c->cs_buf, chunk);
            if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE || n == MBEDTLS_ERR_SSL_TIMEOUT) {
              return 0;
            }
            if (n <= 0) goto complete; /* tolerate: session already usable */
            c->cs_payload_got += (uint32_t)n;
          }
          ESP_LOGI(TAG, "ServerInfo received (discarded)");
          goto complete;
        }
        default:
          derp_cs_fail(ml, c, "bad-step");
          return -1;
      }
    }

    default:
      derp_cs_fail(ml, c, "bad-state");
      return -1;
  }

complete:
  /* NotePreferred: preferred=1 only on the home conn (aux are egress-only). */
  {
    uint8_t preferred = is_home ? 0x01 : 0x00;
    (void)derp_write_frame(ml, c, DERP_FRAME_NOTE_PREFERRED, &preferred, 1);
  }
  /* Data-phase socket rhythm (matches the pre-refactor post-101 settings). */
  {
    struct timeval tv = {.tv_sec = 0, .tv_usec = 200000};
    ml_setsockopt(c->sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ml_setsockopt(c->sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    mbedtls_ssl_conf_read_timeout(&c->ssl_conf, 200);
  }
  c->connected = true;
  c->region_id = c->cs_region;
  c->last_relay_rx_ms = ml_get_time_ms(); /* staleness clock starts at connect */
  {
    derp_dns_cache_t * e = derp_dns_cache_find(derp_lookup_region(c->cs_region));
    if (e) e->post_tcp_fails = 0;
  }
  c->last_recv_ms = ml_get_time_ms();
  c->last_used_ms = c->last_recv_ms;
  /* §7 PROVING inputs: fresh stability clock + zeroed proof evidence. */
  c->connected_at_ms = c->last_recv_ms;
  c->last_pong_ms = 0;
  c->rx_pkts = 0;
  c->cstate = DERP_CS_IDLE;
  if (c->cs_tx_buf) {
    free(c->cs_tx_buf);
    c->cs_tx_buf = NULL;
  }
  if (is_home) {
    xEventGroupSetBits(ml->events, ML_EVT_DERP_CONNECTED);
  }
  ESP_LOGI(
    TAG,
    "DERP handshake complete, connected (region %u, slot=%s, %llums)",
    (unsigned)c->region_id,
    is_home ? "home" : "aux",
    /* elapsed = now - start; start = deadline - TIMEOUT. Signed-safe even
     * past the deadline (the tolerate-missing-ServerInfo path). */
    (unsigned long long)(ml_get_time_ms() + DERP_CONNECT_TIMEOUT_MS - c->cstate_deadline_ms));
  return 1;
}

void ml_derp_disconnect(microlink_t * ml, ml_derp_conn_t * c)
{
  bool is_home = derp_conn_is_home(ml, c);
  c->connected = false;
  if (is_home) {
    /* Only the home connection owns the global "DERP up" bit. */
    xEventGroupClearBits(ml->events, ML_EVT_DERP_CONNECTED);
  }

  /* Abort any in-progress ASYNC connect on this slot (Stage-2): free the
   * pending tx buffer and reset the state machine, so a reap/teardown mid-
   * handshake can neither leak nor leave a steppable corpse behind. */
  if (c->cs_tx_buf) {
    free(c->cs_tx_buf);
    c->cs_tx_buf = NULL;
  }
  c->cstate = DERP_CS_IDLE;

  /* Free the four mbedTLS contexts + socket exactly once. tls_inited guards
     * against a double-free when reaping a slot that was never (re)connected —
     * same leak-on-error hazard the connect macro protects (2026-05-25). */
  if (c->tls_inited) {
    if (c->sockfd >= 0) {
      mbedtls_ssl_close_notify(&c->ssl);
    }
    mbedtls_ssl_free(&c->ssl);
    mbedtls_ssl_config_free(&c->ssl_conf);
    mbedtls_ctr_drbg_free(&c->ctr_drbg);
    mbedtls_entropy_free(&c->entropy);
    c->tls_inited = false;
  }
  if (c->sockfd >= 0) {
    ml_close_sock(c->sockfd);
    c->sockfd = -1;
  }

  /* Only a HOME teardown drains the shared TX queue: those queued frames are
     * stale after a home reconnect. An aux teardown must NOT drop home traffic. */
  if (is_home) {
    ml_derp_tx_item_t item;
    while (xQueueReceive(ml->derp_tx_queue, &item, 0) == pdTRUE) {
      free(item.data);
    }
  }

  ESP_LOGI(TAG, "DERP disconnected (%s region %u)", is_home ? "home" : "aux", (unsigned)c->region_id);
}
