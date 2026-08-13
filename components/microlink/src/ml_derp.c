// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ml_derp.c
 * @brief Unified DERP I/O Task + Connection Management
 *
 * Single task handles BOTH reading and writing to DERP TLS connection.
 * This eliminates the need for a TLS mutex since only one task touches
 * the SSL context. Matches v1's single-threaded DERP model.
 *
 * Architecture:
 * - Poll for incoming DERP frames (TLS read) every iteration
 * - Drain TX queue between reads (TLS write)
 * - No mutex needed — single task owns the SSL context exclusively
 *
 * Backpressure strategy (from tailscaled):
 * When queue is full, dequeue oldest packet and retry up to 3 times.
 * If still full, drop the new packet.
 *
 * Reference: tailscale/wgengine/magicsock/derp.go (runDerpWriter)
 *            tailscale/derp/derphttp/derphttp_client.go
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
 * Read exactly `len` bytes via TLS with timeout and WANT_READ retry.
 * Returns number of bytes read on success, -1 on error, -2 on timeout.
 */
static void derp_pump_home_rx(microlink_t * ml); /* fwd: defined with the aux-connect machinery */

static int derp_tls_read_all(microlink_t * ml, ml_derp_conn_t * c, uint8_t * data, size_t len, int timeout_ms)
{
  size_t received = 0;
  uint64_t start_ms = ml_get_time_ms();

  while (received < len) {
    if (timeout_ms > 0 && (ml_get_time_ms() - start_ms) > (uint64_t)timeout_ms) {
      ESP_LOGW(TAG, "derp_tls_read_all timeout (%d/%d bytes in %dms)", (int)received, (int)len, timeout_ms);
      return -2;
    }

    int ret = mbedtls_ssl_read(&c->ssl, data + received, len - received);
    if (ret < 0) {
      if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE || ret == MBEDTLS_ERR_SSL_TIMEOUT) {
        /* An AUX conn mid-CONNECT waits here at 100 ms granularity (the short
         * read timeout is now kept for the WHOLE aux connect sequence, not
         * just the TLS handshake) — service the home conn's rx instead of
         * sleeping, so a silent post-TLS stall (NAT-churned half-open conn:
         * TCP+TLS complete, then blackhole — the 2026-08-13 storm's 28 s
         * machn->remote delivery hole) can never starve the safety path.
         * Operational reads (c->connected) and the home conn's own connect
         * keep the plain yield. */
        if (!c->connected && c != &ml->derp[ml->derp_home_slot]) {
          derp_pump_home_rx(ml);
        } else {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        continue;
      }
      if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) {
        ESP_LOGW(TAG, "DERP server closed connection");
        return -1;
      }
      ESP_LOGE(TAG, "TLS read failed: -0x%04x", -ret);
      return -1;
    }
    if (ret == 0) {
      ESP_LOGW(TAG, "TLS connection closed by peer (%d/%d bytes)", (int)received, (int)len);
      return -1;
    }
    received += ret;
  }
  return (int)received;
}

/**
 * Read a DERP frame header (5 bytes: type + 4-byte BE length) with timeout.
 */
static esp_err_t derp_recv_frame_header(
  microlink_t * ml, ml_derp_conn_t * c, uint8_t * type, uint32_t * len, int timeout_ms)
{
  uint8_t header[5];
  int ret = derp_tls_read_all(ml, c, header, 5, timeout_ms);
  if (ret < 0) {
    return (ret == -2) ? ESP_ERR_TIMEOUT : ESP_FAIL;
  }

  *type = header[0];
  *len = ((uint32_t)header[1] << 24) | ((uint32_t)header[2] << 16) | ((uint32_t)header[3] << 8) | (uint32_t)header[4];

  return ESP_OK;
}

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
  bool is_wg_handshake = (len >= 4 && (data[0] == 0x01 || data[0] == 0x02));
  bool is_priority = is_wg_handshake || ml_wg_is_safety_pubkey(ml, dest_key);
  QueueHandle_t txq = is_priority ? ml->derp_tx_prio_queue : ml->derp_tx_queue;

  if (xQueueSend(txq, &item, 0) == pdTRUE) {
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

/* Drain up to a few pending frames from the HOME conn — called from inside a
 * blocking AUX connect so the safety heartbeat relay riding home keeps flowing
 * during the ~1-2s aux TLS handshake (2026-08-12 edge-flush fix; replaces the
 * self-deadlocking defer-aux guard). Single-task, so re-entrant use of
 * poll_derp_read on a DIFFERENT conn is safe. Stops on no-data (bounded 100ms
 * per read) or error (the main loop handles a dead home conn after we return). */
static uint32_t s_diag_home_pumps; /* home-rx drains performed during aux connects */

static void derp_pump_home_rx(microlink_t * ml)
{
  ml_derp_conn_t * home = &ml->derp[ml->derp_home_slot];
  if (!home->connected || home->sockfd < 0) return;
  for (int i = 0; i < 4; i++) {
    int r = poll_derp_read(ml, home);
    if (r <= 0) break; /* 0 = no more data; <0 = home needs the main loop */
    home->last_recv_ms = ml_get_time_ms();
    s_diag_home_pumps++;
  }
}

uint32_t ml_derp_get_home_pumps(void)
{
  return s_diag_home_pumps;
}

/* Home-DERP reconnect telemetry (2026-08-12 edge-flush fix). reconnects =
 * count of successful RST/EOF-triggered home reconnects; last/worst =
 * wall-clock ms from RST detection to reconnected. worst < ~1500 confirms the
 * relay is back inside the 2 s pstop timeout window (green rides it through a
 * flush); a worst climbing toward/over 2000 means recovery is still too slow. */
static uint32_t s_diag_derp_reconnects;
static uint32_t s_diag_derp_last_reconnect_ms;
static uint32_t s_diag_derp_worst_reconnect_ms;

void ml_derp_get_reconnect_diag(uint32_t out[3])
{
  out[0] = s_diag_derp_reconnects;
  out[1] = s_diag_derp_last_reconnect_ms;
  out[2] = s_diag_derp_worst_reconnect_ms;
}

static ml_derp_conn_t * derp_route_conn(microlink_t * ml, uint16_t region_id, uint16_t eff_home)
{
  int hs = ml->derp_home_slot;
  ml_derp_conn_t * home = &ml->derp[hs];
  if (region_id != 0 && region_id != eff_home) {
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].connected && ml->derp[s].region_id == region_id) {
        return &ml->derp[s];
      }
    }
    s_diag_route_home_fallbacks++;
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
      if (want[w] == c->region_id) {
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

  /* 2) Ensure a conn for each wanted region. HARD CAP: at most ONE blocking
   *    ml_derp_connect() per call (each can block ~20-40 s on TLS handshake).
   *    Without this cap, N down aux regions would fire N back-to-back blocking
   *    connects at the loop tail, stalling the slot-0 home read (the safety
   *    inbound path) for tens of seconds → nuisance STOP risk. We attempt one
   *    region and return; the rest are picked up on the next task iteration,
   *    AFTER home connect/drain/read have run again. (Adversarial review 2026-08-04.) */

  /* Home-rx protection (2026-08-12): the one-connect-per-call cap above bounds
   * N back-to-back connects, but a single aux TLS handshake still takes ~1-2s.
   * Rather than DEFER the aux (the removed guard self-deadlocked a cross-region
   * relay-bound safety peer — @claude PR review), ml_derp_connect now services
   * the home conn's rx during an aux handshake (derp_pump_home_rx), so the
   * safety heartbeat relay keeps flowing while the standby (re)connects. */
  for (int w = 0; w < nwant; w++) {
    uint16_t rid = want[w];
    bool served = false;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].connected && ml->derp[s].region_id == rid) {
        served = true;
        break;
      }
    }
    if (served) continue;

    /* Reuse a slot already assigned this region (a prior drop/failed attempt),
     * else grab a free slot. */
    int slot = -1;
    for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
      if (s == hs) continue;
      if (ml->derp[s].region_id == rid) {
        slot = s;
        break;
      }
    }
    if (slot < 0) {
      for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
        if (s == hs) continue;
        if (!ml->derp[s].connected && !ml->derp[s].tls_inited && ml->derp[s].region_id == 0) {
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
    c->last_connect_attempt_ms = now;
    if (aux_burst[slot] < 100) aux_burst[slot]++;
    ESP_LOGI(TAG, "Opening aux DERP conn slot %d for region %u", slot, (unsigned)rid);
    if (ml_derp_connect(ml, c, rid) == ESP_OK) {
      aux_burst[slot] = 0; /* reset fast-retry on success */
    } else {
      /* Keep the region assigned to this slot so the next pass retries it. */
      c->region_id = rid;
    }
    /* HARD CAP (see loop header): exactly one blocking connect per call. Return
     * so the task loop services the home slot again before the next aux attempt. */
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
    if (!ml->derp[s].connected && !ml->derp[s].tls_inited && ml->derp[s].region_id == 0) {
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

void ml_derp_tx_task(void * arg)
{
  microlink_t * ml = (microlink_t *)arg;
  ESP_LOGI(TAG, "DERP I/O task started (Core %d)", xPortGetCoreID());

  uint32_t frames_rx = 0;
  uint32_t frames_tx = 0;
  uint64_t last_status_ms = 0;
  uint32_t loop_count = 0;
  uint64_t last_heartbeat_ms = 0;
  uint64_t connected_since_ms = 0;
  bool verbose_phase = false; /* verbose logging for first 15s after connect */
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
        /* Retry up to 3 times with EXPONENTIAL backoff (2s, 4s, 8s).
                 * Linear 2s backoff hammered the DERP server during the
                 * connect failure mode where the server was just slow —
                 * the chip kept asking and never giving the server time
                 * to settle, then leaked TLS contexts via the macro path
                 * audited 2026-05-25. */
        uint32_t backoff_ms = 2000;
        for (int attempt = 0; attempt < 3 && !home->connected; attempt++) {
          if (attempt > 0) {
            ESP_LOGW(TAG, "DERP connect retry %d/3 in %lums...", attempt + 1, (unsigned long)backoff_ms);
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms *= 2;
          } else {
            ESP_LOGI(TAG, "DERP connect requested, connecting from I/O task");
          }
          if (ml_derp_connect(ml, home, ml_effective_home_region(ml)) == ESP_OK) {
            connected_since_ms = ml_get_time_ms();
            verbose_phase = true;
            break;
          }
          ESP_LOGW(TAG, "DERP connect attempt %d failed", attempt + 1);
        }
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
        static int s_derp_retry_burst = 0;
        /* When a priority (safety) peer is configured, DERP is the
                 * failover path for its heartbeat — a 60 s reconnect wall is
                 * far too long. Retry fast for the first few attempts (5s,
                 * 5s, 10s) then fall back to the calm 60 s cadence so a
                 * genuinely-down DERP server isn't hammered. Non-priority
                 * builds keep the original 60 s. */
        bool has_prio = (ml->config.priority_peer_ip != 0);
        uint64_t retry_gap = 60000;
        if (has_prio) {
          retry_gap = (s_derp_retry_burst < 2) ? 5000u : (s_derp_retry_burst < 3) ? 10000u : 60000u;
        }
        if (!home->connected && ml->state == ML_STATE_CONNECTED && loop_start - s_last_derp_retry_ms > retry_gap) {
          s_last_derp_retry_ms = loop_start;
          if (s_derp_retry_burst < 100) {
            s_derp_retry_burst++;
          }

          /* ---- DERP design-review finding (2026-08): home-region unreachable ----
           * Historically this block retried ml_effective_home_region() FOREVER.
           * If that region's DERP server is genuinely TCP-unreachable (a transient
           * regional outage, or a bad advised/locked region) slot 0 — the chip's
           * inbound-reachability + relay conn — stayed down permanently. On a
           * remote it is masked (the safety peer's region drives an event-driven
           * re-home onto a reachable region), but a chip with no re-home driver
           * (priority_peer_region==0) had NO recovery path. Fix: after N
           * consecutive failed home attempts, fall back to a PROVEN-reachable
           * region. The multi-region aux pool already holds live conns to other
           * regions, so we lean on that as the reachability oracle. Bounded work,
           * no reconnect storm; a LOCK is never silently abandoned. */
          uint16_t home_region = ml_effective_home_region(ml);
          bool locked = (ml->derp_region_override != 0);
          bool try_fallback = (s_derp_retry_burst >= ML_DERP_HOME_FALLBACK_AFTER);
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
              s_derp_retry_burst,
              (unsigned)fb);
            if (ml_derp_connect(ml, home, fb) == ESP_OK) {
              ml->derp_home_region = fb; /* eff_home + routing follow (no re-home owner to fight) */
              connected_since_ms = ml_get_time_ms();
              verbose_phase = true;
              s_derp_retry_burst = 0;
              s_derp_home_unreachable_fallbacks++;
            }
          } else {
            /* Default path: (re)try the INTENDED home. This is what honours an
             * operator LOCK and an active re-home owner — we never abandon
             * either. */
            ESP_LOGI(TAG, "DERP periodic retry (disconnected, burst=%d)", s_derp_retry_burst);
            if (ml_derp_connect(ml, home, home_region) == ESP_OK) {
              connected_since_ms = ml_get_time_ms();
              verbose_phase = true;
              s_derp_retry_burst = 0; /* reset fast-retry on success */
            } else if (try_fallback && fb != 0) {
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
              int slot = any_up ? -1 : derp_free_aux_slot(ml);
              if (slot > 0 && ml_derp_connect(ml, &ml->derp[slot], fb) == ESP_OK) {
                ml->derp[slot].region_id = fb;
                s_derp_home_unreachable_fallbacks++;
                ESP_LOGW(
                  TAG,
                  "DERP home %u unreachable + pool dark; opened rescue aux slot %d region %u (home retry retained)",
                  (unsigned)home_region,
                  slot,
                  (unsigned)fb);
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
        verbose_phase = false;
        /* IMMEDIATE first reconnect (run-20/21/22 root cause, 2026-08-12
         * DUT-host wire+journal evidence): an RST/EOF here is almost always a
         * middlebox connection-table FLUSH (the office edge firewall RSTs
         * long-lived DERP TCP across regions and rebinds NAT in the same
         * instant), NOT a down server. The old unconditional 1000 ms pre-delay
         * + TLS handshake pushed relay recovery past the 2 s pstop timeout —
         * and because the same flush kills the direct hairpin simultaneously,
         * the heartbeat had NO path for those seconds -> green drop. tailscaled
         * recovers in <1 s (connGen++); match it: try NOW, back off (2 s) only
         * on CONSECUTIVE failures (a genuinely-down server). A short yield lets
         * ml_derp_disconnect's socket close settle without stalling recovery. */
        vTaskDelay(pdMS_TO_TICKS(20));
        for (int attempt = 0; attempt < 3 && !home->connected; attempt++) {
          if (attempt > 0) {
            ESP_LOGW(TAG, "DERP reconnect retry %d/3 in 2s...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
          }
          if (ml_derp_connect(ml, home, ml_effective_home_region(ml)) == ESP_OK) {
            connected_since_ms = ml_get_time_ms();
            verbose_phase = true;
            s_diag_derp_reconnects++;
            s_diag_derp_last_reconnect_ms = (uint32_t)(ml_get_time_ms() - reconn_t0);
            if (s_diag_derp_last_reconnect_ms > s_diag_derp_worst_reconnect_ms) {
              s_diag_derp_worst_reconnect_ms = s_diag_derp_last_reconnect_ms;
            }
            ESP_LOGW(
              TAG, "DERP home reconnected in %ums (attempt %d)", (unsigned)s_diag_derp_last_reconnect_ms, attempt + 1);
            break;
          }
          ESP_LOGW(TAG, "DERP reconnect attempt %d failed", attempt + 1);
        }
      }
    }

    /* Disable verbose logging after 15s */
    if (verbose_phase && ml_get_time_ms() - connected_since_ms > 15000) {
      verbose_phase = false;
    }

    /* If NOTHING in the pool is connected there's nothing to service. Home is
         * (re)tried above; aux is managed at the tail. Just wait a beat. */
    {
      bool any_connected = false;
      for (int s = 0; s < ML_DERP_MAX_CONNS; s++) {
        if (ml->derp[s].connected) {
          any_connected = true;
          break;
        }
      }
      if (!any_connected) {
        derp_manage_aux(ml, ml_effective_home_region(ml), aux_burst, loop_start);
        /* Keep the §7 MBB machine responsive to cancels/locks even with the
             * whole pool dark (it cannot progress, but it must stay abortable). */
        derp_mbb_tick(ml, loop_start);
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
    }

    /* ---- Phase 1: Drain TX queue FIRST (prioritize outgoing), routing each
         *      frame to the pool connection homed on the destination's region. ---- */
    {
      uint16_t eff_home = ml_effective_home_region(ml);
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
          ml_derp_conn_t * c = derp_route_conn(ml, item.region_id, eff_home);
          if (c == NULL) {
            /* Chosen conn (home fallback) is down — nothing can carry it now.
                     * Drop + log rather than block; wg_mgr re-fires safety inits. */
            ESP_LOGW(TAG, "DERP TX drop: no connected conn for region %u", (unsigned)item.region_id);
            free(item.data);
            continue;
          }
          bool fell_back = (item.region_id != 0 && item.region_id != eff_home && c == home);
          if (fell_back) {
            /* Never silently drop a safety peer's frame: aux for its region isn't
                     * up yet, so relay via home (may still reach if the peer also holds a
                     * home-region conn) and keep the aux opening in the background. */
            ESP_LOGW(
              TAG, "DERP TX: no aux conn for region %u, relaying via home (frame preserved)", (unsigned)item.region_id);
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
            c->connected = false;
            if (c == home) {
              /* Home reconnect is event-driven (handler frees TLS then reconnects). */
              xEventGroupSetBits(ml->events, ML_EVT_DERP_RECONNECT);
            } else {
              /* Aux has no event path: tear it down NOW (frees TLS) so the next
                         * derp_manage_aux reconnect doesn't leak by re-initing over live
                         * contexts. region_id is preserved for the retry. */
              ml_derp_disconnect(ml, c);
            }
          } else {
            frames_tx++;
            c->last_used_ms = ml_get_time_ms();
          }
          free(item.data);
        }
      }
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
          c->connected = false;
          if (c == home) {
            xEventGroupSetBits(ml->events, ML_EVT_DERP_RECONNECT);
          } else {
            ml_derp_disconnect(ml, c); /* free TLS now; manage_aux reconnects */
          }
          break;
        }
      }
    }

    /* ---- Aux pool management at the TAIL: home is already serviced this
         *      iteration, so a (potentially blocking) aux TLS handshake here can't
         *      starve the home slot / the safety path. ---- */
    derp_manage_aux(ml, ml_effective_home_region(ml), aux_burst, ml_get_time_ms());

    /* ---- §7 MBB home-switch machine, after manage_aux so a just-connected
         *      target aux is observed the same iteration. O(1) state checks —
         *      every blocking connect above is already capped, MBB adds none. ---- */
    derp_mbb_tick(ml, ml_get_time_ms());

    /* Yield briefly (runtime-tunable via microlink_set_derp_loop_delay_ms). */
    vTaskDelay(pdMS_TO_TICKS(atomic_load(&s_derp_loop_delay_ms)));
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

esp_err_t ml_derp_connect(microlink_t * ml, ml_derp_conn_t * c, uint16_t region_id)
{
  /* Idempotency guard: never re-init mbedTLS over live contexts. The HOME slot's
   * write/read-fail path sets connected=false + ML_EVT_DERP_RECONNECT but does NOT
   * disconnect, so tls_inited can still be true when the periodic-retry block calls
   * us — re-init here would leak the 4 mbedTLS contexts + the old socket (the exact
   * lwIP-wedge class fixed 2026-05-25). Tear down first. (Adversarial review 2026-08-04.) */
  if (c->tls_inited) {
    ml_derp_disconnect(ml, c);
  }

  /* Determine DERP host/port from DERPMap for the REQUESTED region (slot 0
     * passes the effective home region; aux slots pass a safety peer's region).
     * Always use the first non-stun-only node (the preferred node) so we land on
     * the same node most peers connect to. Falls back to the compiled default
     * host only when the region isn't in the parsed DERPMap. */
  const char * derp_host = ML_DERP_HOST;
  int derp_port = ML_DERP_PORT;

  uint16_t lookup_region = region_id ? region_id : ML_DERP_REGION;
  if (ml->derp_region_count > 0 && lookup_region > 0) {
    for (int i = 0; i < ml->derp_region_count; i++) {
      if (ml->derp_regions[i].region_id == lookup_region) {
        for (int attempt = 0; attempt < ml->derp_regions[i].node_count; attempt++) {
          if (!ml->derp_regions[i].nodes[attempt].stun_only && ml->derp_regions[i].nodes[attempt].hostname[0]) {
            derp_host = ml->derp_regions[i].nodes[attempt].hostname;
            if (ml->derp_regions[i].nodes[attempt].derp_port > 0) {
              derp_port = ml->derp_regions[i].nodes[attempt].derp_port;
            }
            break;
          }
        }
        break;
      }
    }
  }

  bool is_home = (c == &ml->derp[ml->derp_home_slot]);
  int64_t t_derp_start = esp_timer_get_time();

  ESP_LOGI(
    TAG,
    "Connecting to DERP %s:%d (region %d, slot=%s)",
    derp_host,
    derp_port,
    lookup_region,
    is_home ? "home" : "aux");

  /* DNS resolve — accept IPv4 or IPv6 (carrier may be IPv6-only) */
  struct addrinfo hints = {.ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM};
  struct addrinfo * res = NULL;
  char port_str[6];
  snprintf(port_str, sizeof(port_str), "%d", derp_port);

  if (ml_getaddrinfo(derp_host, port_str, &hints, &res) != 0 || !res) {
    ESP_LOGE(TAG, "DNS resolve failed for %s", derp_host);
    return ESP_FAIL;
  }

  int64_t t_derp_dns = esp_timer_get_time();
  ESP_LOGW(
    TAG,
    "[TIMING-DERP] DNS: %lld ms (host=%s ip-family=%d)",
    (t_derp_dns - t_derp_start) / 1000,
    derp_host,
    res->ai_family);

  /* TCP connect — use address family from DNS result */
  int sock = ml_socket(res->ai_family, SOCK_STREAM, 0);
  if (sock < 0) {
    ml_freeaddrinfo(res);
    return ESP_FAIL;
  }

  /* Set connect timeout */
  struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
  ml_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  ml_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (ml_connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
    ESP_LOGE(TAG, "TCP connect failed: %d", errno);
    ml_close_sock(sock);
    ml_freeaddrinfo(res);
    return ESP_FAIL;
  }
  ml_freeaddrinfo(res);

  int64_t t_derp_tcp = esp_timer_get_time();
  ESP_LOGW(TAG, "[TIMING-DERP] TCP connect: %lld ms", (t_derp_tcp - t_derp_dns) / 1000);

  /* TLS setup. Failure paths below MUST call derp_connect_fail() rather
     * than ml_close_sock+sockfd=-1 alone — otherwise the four mbedTLS
     * contexts initialised here leak every retry, eventually wedging lwip
     * and producing the silent network-deadlock symptom (2026-05-25 fix). */
  mbedtls_ssl_init(&c->ssl);
  mbedtls_ssl_config_init(&c->ssl_conf);
  mbedtls_entropy_init(&c->entropy);
  mbedtls_ctr_drbg_init(&c->ctr_drbg);
  bool tls_inited = true;

/* Inline helper macro: every failure path below has to release the four
     * mbedTLS contexts initialised above AND close the TCP socket. Wrapping
     * it as a macro keeps each early-return site visible/auditable. Operates on
     * THIS connection `c` — the leak-on-error hazard is per-slot (2026-05-25). */
#define DERP_CONNECT_FAIL_CLEANUP()          \
  do {                                       \
    if (tls_inited) {                        \
      mbedtls_ssl_free(&c->ssl);             \
      mbedtls_ssl_config_free(&c->ssl_conf); \
      mbedtls_ctr_drbg_free(&c->ctr_drbg);   \
      mbedtls_entropy_free(&c->entropy);     \
      tls_inited = false;                    \
    }                                        \
    if (sock >= 0) {                         \
      ml_close_sock(sock);                   \
    }                                        \
    c->sockfd = -1;                          \
    c->tls_inited = false;                   \
  } while (0)

  mbedtls_ctr_drbg_seed(&c->ctr_drbg, mbedtls_entropy_func, &c->entropy, NULL, 0);

  mbedtls_ssl_config_defaults(
    &c->ssl_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
  mbedtls_ssl_conf_authmode(&c->ssl_conf, MBEDTLS_SSL_VERIFY_NONE);
  mbedtls_ssl_conf_rng(&c->ssl_conf, mbedtls_ctr_drbg_random, &c->ctr_drbg);
  mbedtls_ssl_conf_read_timeout(&c->ssl_conf, DERP_CONNECT_TIMEOUT_MS);

  mbedtls_ssl_setup(&c->ssl, &c->ssl_conf);
  mbedtls_ssl_set_hostname(&c->ssl, derp_host);
  /* Store socket fd BEFORE setting bio.
     * Use custom BIO callbacks that route through ml_read_sock/ml_write_sock,
     * which transparently support both lwIP and AT socket backends.
     * Timeout is handled via SO_RCVTIMEO. BIO ctx is &c->sockfd (per-conn). */
  c->sockfd = sock;
  mbedtls_ssl_set_bio(&c->ssl, &c->sockfd, ml_derp_bio_send, NULL, ml_derp_bio_recv_timeout);

  /* TLS handshake. For an AUX conn, shorten the read timeout to 100 ms so the
   * BIO returns SSL_TIMEOUT between flights (mbedTLS resumes cleanly on a TLS
   * stream) — we service the home conn's rx there so a ~1-2s aux handshake
   * doesn't gap the safety heartbeat relay riding home (2026-08-12 edge-flush
   * fix; replaces defer-aux). Home keeps the long timeout. Overall deadline is
   * unchanged (DERP_CONNECT_TIMEOUT_MS). */
  if (!is_home) {
    mbedtls_ssl_conf_read_timeout(&c->ssl_conf, 100);
  }
  uint64_t hs_deadline = ml_get_time_ms() + DERP_CONNECT_TIMEOUT_MS;
  int ret;
  while ((ret = mbedtls_ssl_handshake(&c->ssl)) != 0) {
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE || ret == MBEDTLS_ERR_SSL_TIMEOUT) {
      if (!is_home) derp_pump_home_rx(ml); /* keep the safety relay flowing */
      if (ml_get_time_ms() > hs_deadline) {
        ESP_LOGE(TAG, "TLS handshake timeout (%s)", is_home ? "home" : "aux");
        DERP_CONNECT_FAIL_CLEANUP();
        return ESP_FAIL;
      }
      continue;
    }
    char err_buf[128];
    mbedtls_strerror(ret, err_buf, sizeof(err_buf));
    ESP_LOGE(TAG, "TLS handshake failed: %s", err_buf);
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }
  /* AUX conns KEEP the 100 ms read timeout through the HTTP-upgrade and DERP
   * handshake reads below (PR #94 review finding, initially deferred — then a
   * real storm produced its exact failure: a NAT-churned half-open conn that
   * completes TCP+TLS and silently blackholes stalled these "fast" phases for
   * the full 20 s budget with zero home-rx service — a wire-measured 28 s
   * machn->remote delivery hole, 2026-08-13). Every read loop below tolerates
   * SSL_TIMEOUT and now pumps home-rx while waiting; outer deadlines are
   * unchanged. The data phase drops to 200 ms at connect success as before. */

  int64_t t_derp_tls = esp_timer_get_time();
  ESP_LOGW(TAG, "[TIMING-DERP] TLS handshake: %lld ms", (t_derp_tls - t_derp_tcp) / 1000);
  ESP_LOGW(TAG, "[TIMING-DERP] TLS connected; sending HTTP upgrade now");

  /* HTTP Upgrade: GET /derp with Upgrade: DERP header */
  char upgrade_req[256];
  snprintf(
    upgrade_req,
    sizeof(upgrade_req),
    "GET /derp HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Connection: Upgrade\r\n"
    "Upgrade: DERP\r\n"
    "\r\n",
    derp_host);

  ret = mbedtls_ssl_write(&c->ssl, (const uint8_t *)upgrade_req, strlen(upgrade_req));
  if (ret < 0) {
    ESP_LOGE(TAG, "Failed to send HTTP upgrade");
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }

  /* Read HTTP response byte-by-byte until \r\n\r\n to avoid over-reading
     * into the DERP binary frame stream (matching v1 approach) */
  {
    uint8_t resp_buf[512];
    int resp_len = 0;
    bool found_end = false;
    uint64_t http_start = ml_get_time_ms();

    uint64_t last_diag_ms = http_start;
    while (resp_len < (int)sizeof(resp_buf) - 1) {
      uint64_t now_ms = ml_get_time_ms();
      if (now_ms - http_start > DERP_CONNECT_TIMEOUT_MS) {
        ESP_LOGE(TAG, "HTTP upgrade response timeout (resp_len=%d after %llums)", resp_len, now_ms - http_start);
        if (resp_len > 0) {
          resp_buf[resp_len] = 0;
          ESP_LOGE(TAG, "  partial body received: %.100s", (char *)resp_buf);
        }
        DERP_CONNECT_FAIL_CLEANUP();
        return ESP_FAIL;
      }
      if (now_ms - last_diag_ms > 2000) {
        ESP_LOGW(TAG, "[TIMING-DERP] HTTP-upgrade waiting: %llums, %d bytes so far", now_ms - http_start, resp_len);
        last_diag_ms = now_ms;
      }

      ret = mbedtls_ssl_read(&c->ssl, resp_buf + resp_len, 1);
      if (ret < 0) {
        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE || ret == MBEDTLS_ERR_SSL_TIMEOUT) {
          if (!is_home) {
            derp_pump_home_rx(ml); /* aux waits at 100 ms granularity: keep home rx flowing */
          } else {
            vTaskDelay(pdMS_TO_TICKS(10));
          }
          continue;
        }
        ESP_LOGE(TAG, "HTTP upgrade read failed: -0x%04x", -ret);
        DERP_CONNECT_FAIL_CLEANUP();
        return ESP_FAIL;
      }
      if (ret == 0) {
        ESP_LOGE(TAG, "Connection closed during HTTP upgrade");
        DERP_CONNECT_FAIL_CLEANUP();
        return ESP_FAIL;
      }
      resp_len++;

      /* Check for \r\n\r\n */
      if (
        resp_len >= 4 && resp_buf[resp_len - 4] == '\r' && resp_buf[resp_len - 3] == '\n' &&
        resp_buf[resp_len - 2] == '\r' && resp_buf[resp_len - 1] == '\n')
      {
        found_end = true;
        break;
      }
    }

    resp_buf[resp_len] = '\0';

    if (!found_end || strstr((char *)resp_buf, "101") == NULL) {
      ESP_LOGE(TAG, "DERP upgrade rejected: %.100s", resp_buf);
      DERP_CONNECT_FAIL_CLEANUP();
      return ESP_FAIL;
    }
    ESP_LOGI(TAG, "HTTP 101 Switching Protocols received");
  }

  /* Match v1 exactly: O_NONBLOCK + short SO_RCVTIMEO + SO_SNDTIMEO.
     * O_NONBLOCK ensures read()/write() never block indefinitely.
     * SO_RCVTIMEO provides 100ms polling rhythm for reads.
     * SO_SNDTIMEO prevents writes from blocking too long. */
  {
    int flags = ml_fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
      ml_fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }
    struct timeval io_tv = {.tv_sec = 0, .tv_usec = 100000}; /* 100ms */
    ml_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &io_tv, sizeof(io_tv));
    ml_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &io_tv, sizeof(io_tv));
  }

  /* ========================================================
     * DERP Handshake: ServerKey -> ClientInfo -> ServerInfo
     * ======================================================== */

  /* Step 1: Read ServerKey frame header using reliable read helper */
  uint8_t frame_type;
  uint32_t frame_len;
  esp_err_t err = derp_recv_frame_header(ml, c, &frame_type, &frame_len, DERP_CONNECT_TIMEOUT_MS);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read ServerKey frame header (err=%d)", err);
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }

  if (frame_type != DERP_FRAME_SERVER_KEY || frame_len < 40) {
    ESP_LOGE(TAG, "Expected ServerKey frame (0x01), got 0x%02x len=%lu", frame_type, (unsigned long)frame_len);
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }

  /* Read and verify 8-byte magic */
  uint8_t magic[8];
  static const uint8_t DERP_MAGIC[8] = {0x44, 0x45, 0x52, 0x50, 0xf0, 0x9f, 0x94, 0x91};
  if (derp_tls_read_all(ml, c, magic, 8, DERP_CONNECT_TIMEOUT_MS) < 0) {
    ESP_LOGE(TAG, "Failed to read ServerKey magic");
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }

  if (memcmp(magic, DERP_MAGIC, 8) != 0) {
    ESP_LOGE(
      TAG,
      "Invalid DERP magic: %02x%02x%02x%02x%02x%02x%02x%02x",
      magic[0],
      magic[1],
      magic[2],
      magic[3],
      magic[4],
      magic[5],
      magic[6],
      magic[7]);
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "DERP magic verified");

  /* Read 32-byte server public key */
  uint8_t derp_server_key[32];
  if (derp_tls_read_all(ml, c, derp_server_key, 32, DERP_CONNECT_TIMEOUT_MS) < 0) {
    ESP_LOGE(TAG, "Failed to read server key");
    DERP_CONNECT_FAIL_CLEANUP();
    return ESP_FAIL;
  }

  ESP_LOGI(
    TAG,
    "DERP server key received (first 8): %02x%02x%02x%02x%02x%02x%02x%02x",
    derp_server_key[0],
    derp_server_key[1],
    derp_server_key[2],
    derp_server_key[3],
    derp_server_key[4],
    derp_server_key[5],
    derp_server_key[6],
    derp_server_key[7]);

  /* Skip remaining bytes if frame_len > 40 */
  if (frame_len > 40) {
    uint8_t skip_buf[64];
    size_t remaining = frame_len - 40;
    while (remaining > 0) {
      size_t chunk = remaining > sizeof(skip_buf) ? sizeof(skip_buf) : remaining;
      if (derp_tls_read_all(ml, c, skip_buf, chunk, DERP_CONNECT_TIMEOUT_MS) < 0) break;
      remaining -= chunk;
    }
  }

  /* Step 2: Send ClientInfo frame (type 0x02)
     * Payload: [our_nodekey(32)][nonce(24)][nacl_box(JSON)] */
  {
    const char * client_info_json = "{\"Version\":2,\"CanAckPings\":true,\"IsProber\":false}";
    size_t json_len = strlen(client_info_json);

    /* Generate random nonce */
    uint8_t nonce[NACL_BOX_NONCEBYTES];
    esp_fill_random(nonce, NACL_BOX_NONCEBYTES);

    /* Encrypt JSON with NaCl box: our WG private key -> DERP server public key */
    size_t ciphertext_len = json_len + NACL_BOX_MACBYTES;
    uint8_t * ciphertext = malloc(ciphertext_len);
    if (!ciphertext) {
      DERP_CONNECT_FAIL_CLEANUP();
      return ESP_FAIL;
    }

    if (
      nacl_box(
        ciphertext,
        (const uint8_t *)client_info_json,
        json_len,
        nonce,
        derp_server_key, /* recipient: DERP server */
        ml->wg_private_key /* sender: our WG node key */
        ) != 0)
    {
      ESP_LOGE(TAG, "NaCl box encrypt failed");
      free(ciphertext);
      DERP_CONNECT_FAIL_CLEANUP();
      return ESP_FAIL;
    }

    /* Build ClientInfo frame payload: nodekey(32) + nonce(24) + ciphertext */
    size_t ci_payload_len = 32 + NACL_BOX_NONCEBYTES + ciphertext_len;
    uint8_t * ci_payload = malloc(ci_payload_len);
    if (!ci_payload) {
      free(ciphertext);
      DERP_CONNECT_FAIL_CLEANUP();
      return ESP_FAIL;
    }

    memcpy(ci_payload, ml->wg_public_key, 32);
    memcpy(ci_payload + 32, nonce, NACL_BOX_NONCEBYTES);
    memcpy(ci_payload + 32 + NACL_BOX_NONCEBYTES, ciphertext, ciphertext_len);
    free(ciphertext);

    ESP_LOGI(
      TAG,
      "DERP ClientInfo node_key=%02x%02x%02x%02x%02x%02x%02x%02x",
      ml->wg_public_key[0],
      ml->wg_public_key[1],
      ml->wg_public_key[2],
      ml->wg_public_key[3],
      ml->wg_public_key[4],
      ml->wg_public_key[5],
      ml->wg_public_key[6],
      ml->wg_public_key[7]);

    /* Send ClientInfo frame */
    if (derp_write_frame(ml, c, DERP_FRAME_CLIENT_INFO, ci_payload, ci_payload_len) < 0) {
      ESP_LOGE(TAG, "Failed to send ClientInfo");
      free(ci_payload);
      DERP_CONNECT_FAIL_CLEANUP();
      return ESP_FAIL;
    }
    free(ci_payload);

    ESP_LOGI(TAG, "ClientInfo sent");
  }

  /* Step 3: Read ServerInfo frame (type 0x03) */
  {
    uint8_t si_type;
    uint32_t si_len;
    err = derp_recv_frame_header(ml, c, &si_type, &si_len, DERP_CONNECT_TIMEOUT_MS);
    if (err == ESP_OK && si_type == DERP_FRAME_SERVER_INFO && si_len > 0) {
      /* Read and discard ServerInfo payload */
      uint8_t * si_buf = malloc(si_len);
      if (si_buf) {
        derp_tls_read_all(ml, c, si_buf, si_len, DERP_CONNECT_TIMEOUT_MS);
        free(si_buf);
      }
      ESP_LOGI(TAG, "ServerInfo received (discarded)");
    } else if (err != ESP_OK) {
      ESP_LOGW(TAG, "No ServerInfo frame (continuing anyway)");
    }
  }

  /* Send NotePreferred (type 0x07). preferred=1 ONLY on the home connection —
     * that is the region the control plane advertises as our PreferredDERP and
     * where peers expect to reach us. Aux connections are relay-only egress
     * paths, so they announce preferred=0 to avoid confusing the server about
     * which region is our home. */
  {
    uint8_t preferred = is_home ? 0x01 : 0x00;
    derp_write_frame(ml, c, DERP_FRAME_NOTE_PREFERRED, &preferred, 1);
  }

  /* Switch socket to short timeout for data phase.
     * Long timeout was needed for TLS handshake, but polling must be fast. */
  {
    struct timeval tv = {.tv_sec = 0, .tv_usec = 200000}; /* 200ms */
    ml_setsockopt(c->sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    mbedtls_ssl_conf_read_timeout(&c->ssl_conf, 200);
  }

  c->connected = true;
  c->tls_inited = true;
  c->region_id = region_id;
  c->last_recv_ms = ml_get_time_ms();
  c->last_used_ms = c->last_recv_ms;
  /* §7 PROVING inputs: fresh stability clock + zeroed proof evidence for this
   * connection instance (a reconnect must never inherit stale proof). */
  c->connected_at_ms = c->last_recv_ms;
  c->last_pong_ms = 0;
  c->rx_pkts = 0;
  /* ML_EVT_DERP_CONNECTED means "the chip has its HOME relay up" (inbound
     * reachable). Only slot 0 owns that bit; aux connections don't touch it. */
  if (is_home) {
    xEventGroupSetBits(ml->events, ML_EVT_DERP_CONNECTED);
  }

  int64_t t_derp_done = esp_timer_get_time();
  ESP_LOGI(
    TAG,
    "[TIMING] DERP total: %lld ms (DNS=%lld, TCP=%lld, TLS=%lld, proto=%lld)",
    (t_derp_done - t_derp_start) / 1000,
    (t_derp_dns - t_derp_start) / 1000,
    (t_derp_tcp - t_derp_dns) / 1000,
    (t_derp_tls - t_derp_tcp) / 1000,
    (t_derp_done - t_derp_tls) / 1000);
  ESP_LOGI(TAG, "DERP handshake complete, connected");
  return ESP_OK;
}

void ml_derp_disconnect(microlink_t * ml, ml_derp_conn_t * c)
{
  bool is_home = (c == &ml->derp[ml->derp_home_slot]);
  c->connected = false;
  if (is_home) {
    /* Only the home connection owns the global "DERP up" bit. */
    xEventGroupClearBits(ml->events, ML_EVT_DERP_CONNECTED);
  }

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
