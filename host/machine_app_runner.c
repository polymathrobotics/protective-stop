// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/*
 * machine_app_runner.c — laptop-side pstop machine for the pstop remote
 * example. Same logic as upstream pstop_c/examples/machine/machine_app.c
 * (we link the same library), but binds <bind>:<port> instead of 127.0.0.1
 * so the chip can reach it over Tailscale or USB-NCM, and reads all of its
 * settable parameters from a documented config file (machine.toml).
 *
 * Upstream pstop_c is left untouched — everything here is the transport +
 * config + logging shell around the unchanged, safety-validated library.
 *
 * Build:   make
 * Run:     ./machine_app_runner [config.toml] [port] [-v]
 *            - config.toml : path to config (default: ./machine.toml if present)
 *            - port        : overrides config [network].port
 *            - -v          : forces [logging].verbose = true
 *
 * Then point the chip at this host:
 *   HOST_TS_IP=$(tailscale ip -4)
 *   curl -X POST "http://<chip>/api/pstop_peer?ip=$HOST_TS_IP&port=8890"
 *
 * SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _GNU_SOURCE
  #define _GNU_SOURCE /* CLOCK_BOOTTIME (independent clock ref for the clock guard) */
#endif

#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "clock_guard.h"
#include "pstop/machine.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"
#include "transport/udp/udp_transport.h"

/* CLOCK_BOOTTIME is Linux-specific; fall back to MONOTONIC_RAW / REALTIME on
 * platforms that lack it so the build stays portable. Independence from the
 * primary CLOCK_MONOTONIC still holds against CLOCK_REALTIME + the call-count
 * proxy even when BOOTTIME degrades to a less-independent source. */
#ifndef CLOCK_BOOTTIME
  #ifdef CLOCK_MONOTONIC_RAW
    #define CLOCK_BOOTTIME CLOCK_MONOTONIC_RAW
  #else
    #define CLOCK_BOOTTIME CLOCK_REALTIME
  #endif
#endif

/* ============================================================================
 * Config — every setting the upstream machine library exposes, plus the
 * wrapper's transport/logging knobs. Loaded from a TOML-subset file. See
 * machine.toml (next to this binary) for the documented defaults.
 * ========================================================================== */

#define MAX_OPERATORS 32U

typedef struct
{
  uint32_t device_id; /* remote UUID (pstop_msg.id) this entry matches  */
  uint64_t heartbeat_ms; /* 0 => inherit default_heartbeat_ms              */
  int stop_only; /* true => may STOP but never transition to OK    */
  int allowed; /* false => machine rejects this remote's bonds   */
} op_cfg_t;

typedef struct
{
  /* [network] */
  int port;
  char bind_addr[64];
  /* [machine] */
  uint32_t machine_device_id;
  /* [limits] — upstream pstop_application_config_t + remotes array size.
     * Liveness is the LIBRARY's native check_heartbeats: timeout is governed by
     * each operator's heartbeat_ms and max_missed_heartbeats. There is no
     * wrapper watchdog and no clock option — the machine always uses its own
     * monotonic clock (see machine_now_ms). */
  uint16_t max_lost_messages;
  uint16_t max_missed_heartbeats;
  uint16_t max_remotes;
  /* [logging] */
  int verbose;
  /* [policy] — operator allowlist defaults for unlisted remotes */
  int allow_unlisted;
  uint64_t default_heartbeat_ms;
  int default_stop_only;
  /* Minimum duration (ms) between the STOP that opens an arming cycle and
     * the OK that completes it — the defence against EMC-induced loop blips
     * performing the arming gesture accidentally (observed 2026-07-21:
     * 100-200 ms both-channel blips in WiFi mode re-armed the machine).
     * Enforced NATIVELY by pstop_c since #59: wired straight into
     * app_config.delay_between_stop_ms; an early OK is refused by the
     * library (replies STOP, stays STOPPED) and the episode can still
     * complete by waiting out the minimum. 0 disables. */
  uint64_t min_stop_ms;
  /* [announce] — optional check-in to a central management console so an
     * operator dashboard can show this machine as RUNNING. One HTTP POST of
     * {"name":..., "port":...} every announce_interval_s; entirely OFF when
     * announce_url is empty. The URL and bearer-key FILE PATH can also come
     * from the environment (PSTOP_ANNOUNCE_URL / PSTOP_ANNOUNCE_KEY_FILE),
     * so deployment endpoints and secrets stay out of committed config. */
  char announce_url[256]; /* http://host[:port]/path — empty = disabled  */
  char announce_key_file[256]; /* file whose first line is the bearer token  */
  char announce_name[64]; /* display name; empty = gethostname()         */
  int announce_interval_s; /* default 60                                   */
  /* [[operator]] entries */
  op_cfg_t operators[MAX_OPERATORS];
  int n_operators;
} machine_cfg_t;

static machine_cfg_t g_cfg;

static void cfg_defaults(machine_cfg_t * c)
{
  memset(c, 0, sizeof(*c));
  c->port = 8890;
  snprintf(c->bind_addr, sizeof(c->bind_addr), "%s", "0.0.0.0");
  c->machine_device_id = 0x01020304U; /* matches DEVICE_ID_MACHINE in main.c */
  c->max_lost_messages = 10U;
  /* Library timeout = heartbeat_ms x max_missed_heartbeats (machine.c
   * check_heartbeats: missed = diff/hb, stop when missed >= max). Remotes
   * send at hb/2 and deliberately withhold a tick on a lockstep mismatch,
   * so max_missed=1 tolerates ZERO withheld ticks (a 1-tick withhold races
   * the deadline exactly — observed false STOPs on the bench). 3 = ~3 s
   * silence at hb=1000 and absorbs up to 5 withheld send slots. */
  c->max_missed_heartbeats = 3U;
  c->max_remotes = 3U;
  c->verbose = 0;
  c->allow_unlisted = 1;
  /* Advertised per-remote heartbeat window; remotes publish at half this
   * (2x margin), so 400 ms => a 5 Hz remote update rate. */
  c->default_heartbeat_ms = 400U;
  c->min_stop_ms = 500U;
  /* SAFETY default: an unlisted remote is accepted (allow_unlisted) and
   * heartbeat-monitored but STOP-ONLY — it may command STOP, never re-arm
   * (STOP->OK). Only a remote named in a [[operator]] entry (stop_only=false)
   * may re-arm. Empty operator list => every remote is stop-only = maximally
   * safe. This replaces the previous accept-any-as-full-operator default. */
  c->default_stop_only = 1;
  c->n_operators = 0;
  c->announce_url[0] = '\0';
  c->announce_key_file[0] = '\0';
  c->announce_name[0] = '\0';
  c->announce_interval_s = 60;
}

static char * trim(char * s)
{
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  if (*s == 0) return s;
  char * e = s + strlen(s) - 1;
  while (e > s && (*e == ' ' || *e == '\t' || *e == '\r' || *e == '\n')) *e-- = 0;
  return s;
}

static int parse_bool(const char * v)
{
  return (strcmp(v, "true") == 0 || strcmp(v, "1") == 0 || strcmp(v, "yes") == 0 || strcmp(v, "on") == 0);
}

/* strtoull with base 0 accepts decimal and 0x-hex. */
static uint64_t parse_uint(const char * v)
{
  return strtoull(v, NULL, 0);
}

/* Load a TOML-subset: '#'/';' comments, [section], [[operator]] array-tables,
 * and key = value (int / 0xhex / true|false / "string"). Unknown keys warn.
 * Returns 0 on success, -1 if the file can't be opened. */
static int cfg_load(machine_cfg_t * c, const char * path)
{
  FILE * f = fopen(path, "r");
  if (!f) return -1;

  char line[256], section[32] = "";
  op_cfg_t * op = NULL;

  while (fgets(line, sizeof(line), f)) {
    char * s = trim(line);
    if (*s == 0 || *s == '#' || *s == ';') continue;

    if (*s == '[') {
      if (s[1] == '[') { /* [[operator]] */
        snprintf(section, sizeof(section), "%s", "operator");
        op = NULL;
        if (c->n_operators < (int)MAX_OPERATORS) {
          op = &c->operators[c->n_operators++];
          op->device_id = 0;
          op->heartbeat_ms = 0; /* 0 => default */
          op->stop_only = 0;
          op->allowed = 1;
        } else {
          fprintf(stderr, "  config: too many [[operator]] entries (max %u)\n", MAX_OPERATORS);
        }
      } else { /* [section] */
        char * e = strchr(s, ']');
        if (e) *e = 0;
        snprintf(section, sizeof(section), "%s", s + 1);
        op = NULL;
      }
      continue;
    }

    char * eq = strchr(s, '=');
    if (!eq) continue;
    *eq = 0;
    char * key = trim(s);
    char * val = trim(eq + 1);
    if (*val == '"') {
      val++;
      char * q = strrchr(val, '"');
      if (q) *q = 0;
    }

    if (strcmp(section, "operator") == 0 && op) {
      if (!strcmp(key, "device_id"))
        op->device_id = (uint32_t)parse_uint(val);
      else if (!strcmp(key, "heartbeat_ms"))
        op->heartbeat_ms = parse_uint(val);
      else if (!strcmp(key, "stop_only"))
        op->stop_only = parse_bool(val);
      else if (!strcmp(key, "allowed"))
        op->allowed = parse_bool(val);
      else
        fprintf(stderr, "  config: unknown operator key '%s'\n", key);
      continue;
    }

    /* Flat keys matched by name (section is documentation only). */
    if (!strcmp(key, "port"))
      c->port = (int)parse_uint(val);
    else if (!strcmp(key, "bind"))
      snprintf(c->bind_addr, sizeof(c->bind_addr), "%s", val);
    else if (!strcmp(key, "machine_device_id"))
      c->machine_device_id = (uint32_t)parse_uint(val);
    else if (!strcmp(key, "max_lost_messages"))
      c->max_lost_messages = (uint16_t)parse_uint(val);
    else if (!strcmp(key, "max_missed_heartbeats"))
      c->max_missed_heartbeats = (uint16_t)parse_uint(val);
    else if (!strcmp(key, "max_remotes"))
      c->max_remotes = (uint16_t)parse_uint(val);
    else if (!strcmp(key, "verbose"))
      c->verbose = parse_bool(val);
    else if (!strcmp(key, "allow_unlisted"))
      c->allow_unlisted = parse_bool(val);
    else if (!strcmp(key, "default_heartbeat_ms"))
      c->default_heartbeat_ms = parse_uint(val);
    else if (!strcmp(key, "default_stop_only"))
      c->default_stop_only = parse_bool(val);
    else if (!strcmp(key, "min_stop_ms"))
      c->min_stop_ms = parse_uint(val);
    else if (!strcmp(key, "announce_url"))
      snprintf(c->announce_url, sizeof(c->announce_url), "%s", val);
    else if (!strcmp(key, "announce_key_file"))
      snprintf(c->announce_key_file, sizeof(c->announce_key_file), "%s", val);
    else if (!strcmp(key, "announce_name"))
      snprintf(c->announce_name, sizeof(c->announce_name), "%s", val);
    else if (!strcmp(key, "announce_interval_s"))
      c->announce_interval_s = (int)parse_uint(val);
    else
      fprintf(stderr, "  config: unknown key '%s' (section [%s])\n", key, section);
  }
  fclose(f);
  return 0;
}

static void cfg_dump(const machine_cfg_t * c)
{
  fprintf(
    stderr,
    "config: bind=%s:%d machine_id=0x%08X max_lost=%u max_missed=%u "
    "max_remotes=%u verbose=%d (liveness: library check_heartbeats on a "
    "monotonic clock)\n",
    c->bind_addr,
    c->port,
    c->machine_device_id,
    c->max_lost_messages,
    c->max_missed_heartbeats,
    c->max_remotes,
    c->verbose);
  fprintf(
    stderr,
    "config: policy allow_unlisted=%d default_heartbeat=%llums "
    "default_stop_only=%d; %d explicit operator(s):\n",
    c->allow_unlisted,
    (unsigned long long)c->default_heartbeat_ms,
    c->default_stop_only,
    c->n_operators);
  for (int i = 0; i < c->n_operators; i++)
    fprintf(
      stderr,
      "config:   operator 0x%08X heartbeat=%llums stop_only=%d allowed=%d\n",
      c->operators[i].device_id,
      (unsigned long long)(c->operators[i].heartbeat_ms ? c->operators[i].heartbeat_ms : c->default_heartbeat_ms),
      c->operators[i].stop_only,
      c->operators[i].allowed);
}

/* ============================================================================
 * Machine clock — the machine's OWN monotonic clock (CLOCK_MONOTONIC), wired in
 * as env.get_time_cb so pstop_c's native check_heartbeats (machine.c) advances
 * in real time and detects remote silence by itself.
 *
 * We deliberately do NOT follow the remote's stamp: the library never compares
 * the remote's stamp to anything — protocol.c only echoes it back in
 * received_stamp — so its uptime-vs-wall domain is irrelevant to the machine's
 * checks. Anchoring the machine clock to the remote would freeze it whenever
 * the remote went quiet, disabling the very heartbeat watchdog we rely on.
 * ========================================================================== */
static uint64_t clk_ms(clockid_t which)
{
  struct timespec ts;
  clock_gettime(which, &ts);
  return ((uint64_t)ts.tv_sec) * 1000ULL + ((uint64_t)ts.tv_nsec) / 1000000ULL;
}

/* ----------------------------------------------------------------------------
 * SR-H-04b / FMEA DU-2 — machine-side clock-freeze guard (the FMEDA's dominant
 * lambda_DU term). check_heartbeats trusts machine_now_ms blindly: a frozen or
 * backward CLOCK_MONOTONIC silently disables remote-silence detection, so a
 * dead remote keeps looking alive (dangerous). We cannot detect our own clock's
 * freeze from that same clock — every read of get_time_cb() therefore also
 * feeds an independent-reference guard (CLOCK_BOOTTIME + CLOCK_REALTIME + a
 * loop/call-count progress proxy). On a latched fault the main loop forces STOP
 * and refuses to emit OK. Detection logic is the pure, unit-tested clock_guard
 * module. -------------------------------------------------------------------- */
static clock_guard_t g_clock_guard;

/* Wired in as env.get_time_cb. Returns the machine's CLOCK_MONOTONIC (exactly
 * as before) AND cross-checks it against the independent references on every
 * call, so the guard is fed at the library's own time-read cadence in addition
 * to the explicit poll at the top of the main loop. */
static uint64_t machine_now_ms(void)
{
  uint64_t mono = clk_ms(CLOCK_MONOTONIC);
  uint64_t boot = clk_ms(CLOCK_BOOTTIME);
  uint64_t real = clk_ms(CLOCK_REALTIME);
  (void)clock_guard_update(&g_clock_guard, mono, boot, real);
  return mono;
}

static volatile int s_running = 1;

static void on_sig(int signo)
{
  (void)signo;
  s_running = 0;
}

/* Consults the configured allowlist. A remote listed in [[operator]] uses its
 * entry; an unlisted remote uses [policy] (allow_unlisted + defaults). */
static remote_details_t is_operator_allowed(const device_id_t * device_id)
{
  remote_details_t d;
  for (int i = 0; i < g_cfg.n_operators; i++) {
    if (g_cfg.operators[i].device_id == device_id->data) {
      d.allowed = g_cfg.operators[i].allowed;
      d.stop_only = g_cfg.operators[i].stop_only;
      d.heartbeat_ms = g_cfg.operators[i].heartbeat_ms ? g_cfg.operators[i].heartbeat_ms : g_cfg.default_heartbeat_ms;
      return d;
    }
  }
  d.allowed = g_cfg.allow_unlisted;
  d.stop_only = g_cfg.default_stop_only;
  d.heartbeat_ms = g_cfg.default_heartbeat_ms;
  return d;
}

/* Print transitions only — keeps the stderr stream readable when the chip
 * is hammering us at 10 Hz. */
static pstop_status_message_t s_last_status = PSTOP_STATUS_OK;
static int s_status_initialized = 0;

/* ARMING LATCH. The library emits status_cb(OK) from INSIDE
 * machine_process_message (machine.c:109) — before the wrapper's
 * min-stop-duration policy gets to veto the arming. OK pulses are
 * therefore never propagated from the raw callback; they are committed
 * by robot_status_commit_ok() only after the policy approves. STOPs
 * propagate immediately (the stop path must never be filtered).
 * ANY REAL ACTUATION MUST HANG OFF THE LATCHED PATH, NOT THE RAW
 * CALLBACK. s_policy_armed mirrors the approved state so the library's
 * per-poll re-notification (machine.c:296-302) stays consistent. */
static int s_policy_armed = 0;

/* Min-STOP-duration episode tracker (machine's own monotonic clock).
 * Valid from the accepted STOP that opens an arming cycle until the
 * cycle resolves (arm, veto, bond/unbond, or heartbeat failure). */
static uint64_t s_stop_episode_start = 0;
static int s_stop_episode_valid = 0;

static void status_emit(pstop_status_message_t status)
{
  if (!s_status_initialized || s_last_status != status) {
    fprintf(stderr, "*** ROBOT STATUS -> %s ***\n", status == PSTOP_STATUS_OK ? "OK (armed, cleared to run)" : "STOP");
    s_last_status = status;
    s_status_initialized = 1;
  }
}

static void robot_status(pstop_status_message_t status)
{
  if (status == PSTOP_STATUS_OK) {
    if (s_policy_armed) status_emit(PSTOP_STATUS_OK);
    return; /* unapproved OK: held back until the policy commits it */
  }
  s_policy_armed = 0;
  status_emit(status);
}

static void robot_status_commit_ok(void)
{
  s_policy_armed = 1;
  status_emit(PSTOP_STATUS_OK);
}

static const char * msg_name(uint8_t m)
{
  switch (m) {
    case PSTOP_MESSAGE_OK:
      return "OK";
    case PSTOP_MESSAGE_STOP:
      return "STOP";
    case PSTOP_MESSAGE_BOND:
      return "BOND";
    case PSTOP_MESSAGE_UNBOND:
      return "UNBOND";
    default:
      return "UNKNOWN";
  }
}

static const char * rstate_name(int s)
{
  return s == ROBOT_STATE_OK ? "OK" : "STOPPED";
}

static const char * restart_name(int s)
{
  switch (s) {
    case ROBOT_RESTART_STATE_OK:
      return "OK";
    case ROBOT_RESTART_STATE_NEED_STOP:
      return "NEED_STOP";
    case ROBOT_RESTART_STATE_STOP_RECEIVED:
      return "STOP_RECEIVED";
    default:
      return "?";
  }
}

static const char * err_name(pstop_error_t e)
{
  switch (e) {
    case PSTOP_OK:
      return "OK";
    case PSTOP_OPERATOR_NOT_ALLOWED:
      return "OPERATOR_NOT_ALLOWED";
    case PSTOP_OUT_OF_OPERATOR_SPACE:
      return "OUT_OF_OPERATOR_SPACE";
    case PSTOP_MSG_LOST:
      return "MSG_LOST";
    case PSTOP_MSG_REPETITION:
      return "MSG_REPETITION";
    case PSTOP_MSG_DELAYED:
      return "MSG_DELAYED";
    case PSTOP_MSG_OUT_OF_ORDER:
      return "MSG_OUT_OF_ORDER";
    case PSTOP_MSG_INVALID_CHECKSUM:
      return "INVALID_CHECKSUM";
    case PSTOP_ERROR_INVALID_ID:
      return "INVALID_ID";
    case PSTOP_NOT_BOND_MESSAGE:
      return "NOT_BOND_MESSAGE";
    case PSTOP_INVALID_BOND_REQUEST:
      return "INVALID_BOND_REQUEST";
    case PSTOP_HEARTBEAT_INVALID:
      return "HEARTBEAT_INVALID";
    case PSTOP_MESSAGE_TYPE_INVALID:
      return "MESSAGE_TYPE_INVALID";
    case PSTOP_MISSED_HEARTBEATS:
      return "MISSED_HEARTBEATS";
    case PSTOP_FATAL:
      return "FATAL";
    default:
      return "?";
  }
}

/* Map a local remote id (machine.robot_state.remote_stop_id) back to its wire
 * device id (0x01xxxxxx) for display. 0 = no owner / not found. */
static uint32_t device_id_for_local(const pstop_machine_t * m, uint32_t local_id)
{
  if (local_id == 0U) return 0U;
  for (uint16_t i = 0; i < m->remotes.max_remotes; i++) {
    const pstop_remote_data_t * c = &m->remotes.remotes[i];
    if (c->local_remote_id == local_id) return c->remote_data.remote_id.data;
  }
  return 0U;
}

/* Library log hook — fires when a remote's command TYPE changes (and on
 * rejections / heartbeat timeouts). Names the pstop by its device id so it is
 * always clear WHICH remote sent WHAT. */
static void log_message(uint64_t timestamp, const device_id_t * client, uint8_t message, pstop_error_t error)
{
  (void)timestamp;
  if (error == PSTOP_OK) {
    fprintf(stderr, "RX <- remote 0x%08X  now sending %s\n", client->data, msg_name(message));
  } else if ((error == PSTOP_MISSED_HEARTBEATS) && (message == PSTOP_MESSAGE_UNKNOWN)) {
    /* Not a received packet: this is the library's check_heartbeats verdict
     * (machine.c logs it with PSTOP_MESSAGE_UNKNOWN as a placeholder).
     * Printing it as "RX <-" cost real diagnosis time on the bench. */
    fprintf(stderr, "LIVENESS remote 0x%08X declared lost [MISSED_HEARTBEATS] — robot stopping\n", client->data);
  } else {
    fprintf(stderr, "RX <- remote 0x%08X  %s REJECTED [%s]\n", client->data, msg_name(message), err_name(error));
  }
}

/* ============================================================================
 * Console announce — optional periodic POST so a management console can show
 * this machine as RUNNING. Deliberately dependency-free (raw HTTP/1.1 over a
 * TCP socket, http:// only) and fully outside the safety loop: it runs on
 * its own thread, and any failure only logs. The safety protocol never
 * depends on it.
 * ========================================================================== */

static int announce_post_once(const char * url, const char * key, const char * name, int pstop_port)
{
  /* Parse http://host[:port]/path */
  const char * p = url;
  if (strncmp(p, "http://", 7) != 0) return -1;
  p += 7;
  char host[128];
  int port = 80;
  size_t h = 0;
  while (*p && *p != ':' && *p != '/' && h + 1 < sizeof(host)) host[h++] = *p++;
  host[h] = '\0';
  if (*p == ':') port = atoi(++p);
  while (*p && *p != '/') p++;
  const char * path = (*p != '\0') ? p : "/";
  if (host[0] == '\0' || port <= 0 || port > 65535) return -1;

  char portstr[8];
  snprintf(portstr, sizeof(portstr), "%d", port);
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo * ai = NULL;
  if (getaddrinfo(host, portstr, &hints, &ai) != 0 || ai == NULL) return -2;

  int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(ai);
    return -3;
  }
  struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
  (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  int rc = connect(fd, ai->ai_addr, ai->ai_addrlen);
  freeaddrinfo(ai);
  if (rc != 0) {
    close(fd);
    return -4;
  }

  char body[192];
  int blen = snprintf(body, sizeof(body), "{\"name\":\"%s\",\"port\":%d}", name, pstop_port);
  char req[768];
  int rlen = snprintf(
    req,
    sizeof(req),
    "POST %s HTTP/1.1\r\nHost: %s\r\nAuthorization: Bearer %s\r\n"
    "Content-Type: application/json\r\nContent-Length: %d\r\n"
    "Connection: close\r\n\r\n%s",
    path,
    host,
    key,
    blen,
    body);
  if (rlen <= 0 || rlen >= (int)sizeof(req)) {
    close(fd);
    return -5;
  }
  if (send(fd, req, (size_t)rlen, 0) != rlen) {
    close(fd);
    return -6;
  }
  char resp[64];
  ssize_t n = recv(fd, resp, sizeof(resp) - 1, 0);
  close(fd);
  if (n < 12) return -7;
  resp[n] = '\0';
  /* "HTTP/1.1 200 ..." */
  int status = atoi(resp + 9);
  return (status >= 200 && status < 300) ? 0 : status;
}

static void * announce_thread(void * arg)
{
  (void)arg;
  char key[128] = "";
  if (g_cfg.announce_key_file[0] != '\0') {
    FILE * f = fopen(g_cfg.announce_key_file, "r");
    if (f != NULL) {
      if (fgets(key, sizeof(key), f) != NULL) {
        key[strcspn(key, "\r\n")] = '\0';
      }
      fclose(f);
    } else {
      fprintf(stderr, "announce: cannot read key file %s — announce disabled\n", g_cfg.announce_key_file);
      return NULL;
    }
  }
  char name[64];
  if (g_cfg.announce_name[0] != '\0') {
    snprintf(name, sizeof(name), "%s", g_cfg.announce_name);
  } else if (gethostname(name, sizeof(name)) != 0) {
    snprintf(name, sizeof(name), "%s", "pstop-machine");
  }
  name[sizeof(name) - 1] = '\0';
  int interval = (g_cfg.announce_interval_s > 0) ? g_cfg.announce_interval_s : 60;
  int last_rc = -1000; /* log only on state change, not every minute */
  while (s_running) {
    int rc = announce_post_once(g_cfg.announce_url, key, name, g_cfg.port);
    if (rc != last_rc) {
      if (rc == 0)
        fprintf(stderr, "announce: OK -> %s (as \"%s\")\n", g_cfg.announce_url, name);
      else
        fprintf(stderr, "announce: FAILED rc=%d -> %s (will keep retrying)\n", rc, g_cfg.announce_url);
      last_rc = rc;
    }
    for (int i = 0; i < interval && s_running; i++) sleep(1);
  }
  return NULL;
}

static pstop_application_t pstop_app;
static pstop_machine_t machine;
static udp_transport_data_t udp_transport;

/* SR-H-03 / FMEA DU-4: refuse unsafe timing config at startup. The ROS2 node
 * validates these floors (machine_bridge_node.cpp validate_timing); the host
 * runner previously did not, so a machine.toml with min_stop_ms=0 defeated the
 * arming gesture (SF-3) and an over-large heartbeat window defeated the stop
 * latency (SF-1). Fail-safe: a machine that cannot enforce arming/latency must
 * refuse to run rather than silently accept the setting. Floors mirror
 * machine_bridge_node.cpp floor::. */
#define CFG_HB_MIN_MS 50U
#define CFG_HB_MAX_MS 1000U /* liveness window can't exceed 1 s        */
#define CFG_MAX_MISSED_MIN 1U
#define CFG_MAX_MISSED_MAX 5U /* can't tolerate more than 5 withheld ticks */
#define CFG_MIN_STOP_FLOOR_MS 100U /* anti-blip arming-delay floor             */

static int cfg_validate(const machine_cfg_t * c, char * reason, size_t rlen)
{
  if (c->default_heartbeat_ms < CFG_HB_MIN_MS || c->default_heartbeat_ms > CFG_HB_MAX_MS) {
    snprintf(
      reason,
      rlen,
      "default_heartbeat_ms %llu out of [%u,%u]",
      (unsigned long long)c->default_heartbeat_ms,
      CFG_HB_MIN_MS,
      CFG_HB_MAX_MS);
    return -1;
  }
  if (c->max_missed_heartbeats < CFG_MAX_MISSED_MIN || c->max_missed_heartbeats > CFG_MAX_MISSED_MAX) {
    snprintf(
      reason,
      rlen,
      "max_missed_heartbeats %u out of [%u,%u]",
      c->max_missed_heartbeats,
      CFG_MAX_MISSED_MIN,
      CFG_MAX_MISSED_MAX);
    return -1;
  }
  if (c->min_stop_ms < CFG_MIN_STOP_FLOOR_MS) {
    snprintf(
      reason,
      rlen,
      "min_stop_ms %llu below safety floor %u",
      (unsigned long long)c->min_stop_ms,
      CFG_MIN_STOP_FLOOR_MS);
    return -1;
  }
  for (int i = 0; i < c->n_operators; i++) {
    const uint64_t hb = c->operators[i].heartbeat_ms; /* 0 => inherit default */
    if (hb != 0U && (hb < CFG_HB_MIN_MS || hb > CFG_HB_MAX_MS)) {
      snprintf(
        reason,
        rlen,
        "operator[%d] heartbeat_ms %llu out of [%u,%u]",
        i,
        (unsigned long long)hb,
        CFG_HB_MIN_MS,
        CFG_HB_MAX_MS);
      return -1;
    }
  }
  return 0;
}

int main(int argc, char * argv[])
{
  cfg_defaults(&g_cfg);

  /* Args: [config.toml] [port] [-v]. A non-numeric, non-flag arg is the
     * config path; a numeric arg overrides the port; -v forces verbose. */
  const char * cfg_path = "machine.toml";
  int force_verbose = 0, port_override = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-v"))
      force_verbose = 1;
    else if (argv[i][0] >= '0' && argv[i][0] <= '9')
      port_override = atoi(argv[i]);
    else
      cfg_path = argv[i];
  }

  if (cfg_load(&g_cfg, cfg_path) == 0) {
    fprintf(stderr, "loaded config from %s\n", cfg_path);
  } else {
    fprintf(stderr, "no config at %s — using built-in defaults\n", cfg_path);
  }
  if (force_verbose) g_cfg.verbose = 1;
  if (port_override) g_cfg.port = port_override;
  /* Environment overrides keep deployment endpoints/secrets out of the
     * committed toml (see [announce] in machine.toml). */
  {
    const char * e = getenv("PSTOP_ANNOUNCE_URL");
    if (e != NULL && e[0] != '\0') snprintf(g_cfg.announce_url, sizeof(g_cfg.announce_url), "%s", e);
    e = getenv("PSTOP_ANNOUNCE_KEY_FILE");
    if (e != NULL && e[0] != '\0') snprintf(g_cfg.announce_key_file, sizeof(g_cfg.announce_key_file), "%s", e);
  }
  if (g_cfg.port <= 0 || g_cfg.port > 65535) {
    fprintf(stderr, "bad port: %d\n", g_cfg.port);
    return 1;
  }
  if (g_cfg.max_remotes == 0) g_cfg.max_remotes = 1;
  {
    char vreason[128];
    if (cfg_validate(&g_cfg, vreason, sizeof(vreason)) != 0) {
      fprintf(stderr, "*** UNSAFE CONFIG REFUSED (SR-H-03): %s ***\n", vreason);
      return 1;
    }
  }
  cfg_dump(&g_cfg);

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  /* Remotes array sized from config. */
  pstop_remote_data_t * pstop_clients = calloc(g_cfg.max_remotes, sizeof(pstop_remote_data_t));
  if (!pstop_clients) {
    fprintf(stderr, "OOM allocating %u remotes\n", g_cfg.max_remotes);
    return 1;
  }

  transport_udp_init(&udp_transport);
  pstop_application_init(&pstop_app);

  pstop_app.app_config.max_lost_messages = g_cfg.max_lost_messages;
  pstop_app.app_config.max_missed_heartbeats = g_cfg.max_missed_heartbeats;
  /* Native min-STOP-duration arming policy (pstop_c #59): an OK arriving
     * sooner than this after the STOP that opened the arming cycle is refused
     * by the LIBRARY (replies STOP, stays STOPPED; the episode may still
     * complete by waiting). Supersedes the wrapper-owned veto this runner
     * carried while the library lacked the feature. 0 disables. */
  pstop_app.app_config.delay_between_stop_ms = (uint32_t)g_cfg.min_stop_ms;
  pstop_app.remote_details_cb = is_operator_allowed;
  pstop_app.status_cb = robot_status;
  pstop_app.log_message_cb = log_message;
  /* Arm the clock-freeze guard (SR-H-04b / DU-2) BEFORE the library can call
   * get_time_cb. Defaults: 500 ms independent-reference window (well under the
   * ~1.2 s heartbeat timeout, so a freeze is caught before it can mask a dead
   * remote) + a call-count backstop for the all-clocks-wedged case. */
  clock_guard_init(&g_clock_guard, 0U, 0U);
  pstop_app.env.get_time_cb = machine_now_ms;

  machine_init(&machine, &pstop_app, pstop_clients, g_cfg.max_remotes);

  int result = transport_udp_listen(&udp_transport, g_cfg.bind_addr, g_cfg.port);
  if (result < 0) {
    fprintf(stderr, "transport_udp_listen %s:%d failed\n", g_cfg.bind_addr, g_cfg.port);
    return 1;
  }

  device_id_t machine_uuid = {.data = g_cfg.machine_device_id};
  device_id_copy(&(machine.application->machine_device_id), &machine_uuid);

  fprintf(
    stderr,
    "MACHINE node up on %s:%d (machine_id=0x%08X) — pstop remotes connect here.\n"
    "Log grammar: RX <- remote (what a pstop sent us) | TX -> remote (our reply) |\n"
    "*** ROBOT STATUS / ARMED *** (actuation-relevant) | STATE (library arming cycle).\n",
    g_cfg.bind_addr,
    g_cfg.port,
    g_cfg.machine_device_id);

  pthread_t announce_tid;
  bool announce_started = false;
  if (g_cfg.announce_url[0] != '\0') {
    if (pthread_create(&announce_tid, NULL, announce_thread, NULL) == 0) {
      announce_started = true;
      fprintf(stderr, "announce: every %ds to %s\n", g_cfg.announce_interval_s, g_cfg.announce_url);
    } else {
      fprintf(stderr, "announce: thread create failed — continuing without\n");
    }
  }

  uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
  uint8_t respbytes[PSTOP_MESSAGE_SIZE];
  pstop_msg_t req_msg;
  pstop_msg_t resp_msg;

  /* Sequence-anomaly watch (logged in BOTH modes): surfaces a missed /
     * duplicated / out-of-order message or a non-monotonic stamp even when
     * pstop_c tolerates it and the system does NOT stop. */
  uint32_t last_rx_counter = 0;
  uint64_t last_rx_stamp = 0;
  uint32_t last_rx_id = 0;
  int have_baseline = 0;
  int clock_fault_reported = 0;

  while (s_running) {
    /* 0. Clock-freeze guard (SR-H-04b / FMEA DU-2). Feed the guard explicitly
     *    every poll — independent of how often the library reads the clock —
     *    then fail safe on any latched fault. A frozen/backward CLOCK_MONOTONIC
     *    would silently disable check_heartbeats (a dead remote would look
     *    alive), so the ONLY safe response is to force STOP and never emit OK.
     *    The robot stays STOPPED for as long as the fault persists; in a full
     *    deployment an external watchdog would then restart the host. */
    (void)machine_now_ms(); /* samples all three clocks + updates the guard */
    if (clock_guard_fault(&g_clock_guard) != CLOCK_GUARD_OK) {
      if (!clock_fault_reported) {
        clock_fault_reported = 1;
        fprintf(
          stderr,
          "*** CLOCK GUARD FAULT (%s): machine monotonic clock froze/regressed — "
          "heartbeat watchdog is BLIND. Forcing STOP; refusing all OK. ***\n",
          clock_guard_fault_name(clock_guard_fault(&g_clock_guard)));
      }
      machine_stop_robot(&machine); /* latches ROBOT_STATE_STOPPED + status_cb(STOP) */
      s_stop_episode_valid = 0;
      /* Do NOT process inbound traffic (which could commit an OK) while the
       * clock is untrustworthy. Brief real-time sleep to avoid a busy spin;
       * nanosleep is a relative delay and safe even if the wall clock is off. */
      struct timespec nap = {.tv_sec = 0, .tv_nsec = 50 * 1000 * 1000};
      nanosleep(&nap, NULL);
      continue;
    }

    /* Native liveness watchdog: pstop_c's own check_heartbeats (machine.c),
         * driven by the machine's monotonic clock, issues the STOP (status_cb)
         * and clears the timed-out remote so a fresh BOND re-bonds. Called every
         * poll cycle (~10 ms) so detection is prompt; the timeout is each
         * operator's heartbeat_ms x max_missed_heartbeats (machine.c:290-298;
         * e.g. 400 ms x 3 ~= 1.2 s). [Reconciled 2026-08-02: was
         * "x (max_missed_heartbeats + 1)" — corrected to the library math.] */
    if (machine_validate_heartbeats(&machine) != PSTOP_OK) {
      /* A remote timed out: the library already stopped the robot and
             * reset the arming cycle — a stale episode timestamp must not
             * validate a later, unrelated release. */
      s_stop_episode_valid = 0;
    }

    struct sockaddr_storage client;
    result = transport_udp_read(&udp_transport, reqbytes, PSTOP_MESSAGE_SIZE, &client);

    if (result == PSTOP_MESSAGE_SIZE) {
      pstop_message_decode(&req_msg, reqbytes);

      if (g_cfg.verbose) {
        fprintf(
          stderr,
          "RX <- remote 0x%08X  %-6s cnt=%-5u rcnt=%-5u stamp=%llu crc=%s\n",
          req_msg.id.data,
          msg_name(req_msg.message),
          req_msg.counter,
          req_msg.received_counter,
          (unsigned long long)req_msg.stamp,
          req_msg.checksum == req_msg.calculated_checksum ? "ok" : "BAD");
      }

      /* Upstream protocol_handle_message dereferences the client before
             * its own NULL guard when a non-BOND request arrives from an unknown
             * sender; filter it at the wrapper. A non-BOND from an unbonded or
             * timed-out (marked UNKNOWN by check_heartbeats) remote is just
             * dropped; a fresh BOND re-bonds. */
      int known = (pstop_remote_get(&machine.remotes, &req_msg.id) != NULL);
      if (!known && req_msg.message != PSTOP_MESSAGE_BOND) {
        if (g_cfg.verbose)
          fprintf(stderr, "RX <- remote 0x%08X  %s DROPPED (not bonded)\n", req_msg.id.data, msg_name(req_msg.message));
        continue;
      }

      /* --- Sequence-anomaly watch (both modes) -----------------------
             * Two directions, both surfaced even when recoverable (no stop):
             *   req.counter / req.stamp     — the remote's own outgoing sequence
             *   err below (MSG_LOST/...)    — its echo of the machine's data.
             * NB the chip increments its counter every 10 Hz tick but only
             * SENDS on lockstep agreement, so a counter gap means N messages
             * were dropped OR withheld (mismatch / boot-priming hold) and the
             * link is recovering — not necessarily a wire loss. */
      if (req_msg.checksum != req_msg.calculated_checksum) {
        /* Corrupted on the wire. The library rejects it below
                 * (INVALID_CHECKSUM, no reply) — but it must not reach the
                 * anomaly tracker either: a single flipped bit in the stamp
                 * field once poisoned last_rx_stamp with a 2^56-off value,
                 * after which EVERY legit message logged a false
                 * "stamp not monotonic" anomaly (2026-07-20 chaos bench). */
        fprintf(stderr, "ANOMALY: bad checksum from wire (claimed 0x%08X) — dropped\n", req_msg.id.data);
      } else if (req_msg.message == PSTOP_MESSAGE_BOND) {
        last_rx_counter = req_msg.counter; /* (re)baseline on bond */
        last_rx_stamp = req_msg.stamp;
        last_rx_id = req_msg.id.data;
        have_baseline = 1;
      } else if (have_baseline && req_msg.id.data == last_rx_id) {
        if (req_msg.counter > last_rx_counter + 1U) {
          fprintf(
            stderr,
            "ANOMALY: counter gap from 0x%08X — expected %u, got %u "
            "(%u dropped/withheld; recovering)\n",
            req_msg.id.data,
            last_rx_counter + 1U,
            req_msg.counter,
            req_msg.counter - (last_rx_counter + 1U));
        } else if (req_msg.counter <= last_rx_counter) {
          fprintf(
            stderr,
            "ANOMALY: counter regress/dup from 0x%08X — last %u, got %u\n",
            req_msg.id.data,
            last_rx_counter,
            req_msg.counter);
        }
        if (req_msg.stamp <= last_rx_stamp) {
          fprintf(
            stderr,
            "ANOMALY: stamp not monotonic from 0x%08X — last %llu, got %llu\n",
            req_msg.id.data,
            (unsigned long long)last_rx_stamp,
            (unsigned long long)req_msg.stamp);
        }
        if (req_msg.counter > last_rx_counter) last_rx_counter = req_msg.counter;
        if (req_msg.stamp > last_rx_stamp) last_rx_stamp = req_msg.stamp;
      }

      int prev_robot = machine.robot_state.robot_state;
      int prev_restart = machine.robot_state.restart_state;
      uint32_t prev_sid = machine.robot_state.remote_stop_id;

      pstop_error_t err = machine_process_message(&machine, &req_msg, &resp_msg);

      /* --- Min-STOP-duration arming policy (LIBRARY-native, pstop_c #59) --
             * The library refuses an OK arriving sooner than
             * delay_between_stop_ms after the STOP that opened the arming
             * cycle (replies STOP, stays STOPPED; the episode can still
             * complete by waiting out the minimum). The wrapper no longer
             * enforces anything here — it only tracks the episode clock to
             * make the log lines self-explanatory. */
      if (err == PSTOP_OK) {
        if (req_msg.message == PSTOP_MESSAGE_BOND || req_msg.message == PSTOP_MESSAGE_UNBOND) {
          s_stop_episode_valid = 0;
        } else if (
          prev_restart != ROBOT_RESTART_STATE_STOP_RECEIVED &&
          machine.robot_state.restart_state == ROBOT_RESTART_STATE_STOP_RECEIVED)
        {
          /* Accepted STOP just opened an arming cycle. */
          s_stop_episode_start = machine_now_ms();
          s_stop_episode_valid = 1;
        }
        if (prev_robot == ROBOT_STATE_STOPPED && machine.robot_state.robot_state == ROBOT_STATE_OK) {
          uint64_t held = s_stop_episode_valid ? machine_now_ms() - s_stop_episode_start : 0U;
          fprintf(
            stderr,
            "*** ARMED by remote 0x%08X: STOP held %llu ms (library "
            "minimum %llu ms) ***\n",
            req_msg.id.data,
            (unsigned long long)held,
            (unsigned long long)g_cfg.min_stop_ms);
          robot_status_commit_ok();
          s_stop_episode_valid = 0;
        } else if (
          req_msg.message == PSTOP_MESSAGE_OK && resp_msg.message == PSTOP_MESSAGE_STOP &&
          prev_restart == ROBOT_RESTART_STATE_STOP_RECEIVED && machine.robot_state.robot_state == ROBOT_STATE_STOPPED)
        {
          /* The library refused an early OK (min-delay not yet met) —
                     * log it so a blip-driven release attempt is visible. */
          uint64_t held = s_stop_episode_valid ? machine_now_ms() - s_stop_episode_start : 0U;
          fprintf(
            stderr,
            "ANOMALY: arming DEFERRED for 0x%08X — OK refused, episode age "
            "%llu ms (library requires %llu ms of quiet after the LAST STOP)\n",
            req_msg.id.data,
            (unsigned long long)held,
            (unsigned long long)g_cfg.min_stop_ms);
        }
      }

      if (err == PSTOP_OK) {
        pstop_message_encode(&resp_msg, respbytes);
        transport_udp_write(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, (struct sockaddr_in *)&client);
        if (g_cfg.verbose)
          fprintf(
            stderr,
            "TX -> remote 0x%08X  %-6s cnt=%-5u hb=%ums\n",
            req_msg.id.data,
            msg_name(resp_msg.message),
            resp_msg.counter,
            resp_msg.heartbeat_timeout);
      } else {
        /* Rejected by pstop (recoverable: no response sent, and no STOP
                 * unless loss exceeds tolerance). MSG_LOST/OUT_OF_ORDER can come
                 * from EITHER direction, so disambiguate against the machine's
                 * per-remote counters (which are NOT advanced on the error path,
                 * so they still hold the values the check compared):
                 *   remote->machine : this message's counter vs the last remote
                 *                     counter the machine accepted.
                 *   machine->remote : the machine's own tx counter vs how far the
                 *                     remote has acknowledged it (received_counter).
                 * The larger gap is the culprit direction. */
        pstop_remote_data_t * c = pstop_remote_get(&machine.remotes, &req_msg.id);
        if (c) {
          long r2m = (long)req_msg.counter - (long)c->remote_data.last_received_counter;
          long lag = (long)c->remote_data.msg_counter - (long)req_msg.received_counter;
          fprintf(
            stderr,
            "ANOMALY: %s rejected by pstop — %s | "
            "remote->machine step=%ld (machine last accepted #%u, got #%u) ; "
            "machine->remote ack-lag=%ld (machine sent #%u, remote acked #%u)\n",
            msg_name(req_msg.message),
            err_name(err),
            r2m,
            c->remote_data.last_received_counter,
            req_msg.counter,
            lag,
            c->remote_data.msg_counter,
            req_msg.received_counter);
        } else {
          fprintf(
            stderr,
            "ANOMALY: %s rejected by pstop — %s (cnt=%u rcnt=%u; remote not bonded)\n",
            msg_name(req_msg.message),
            err_name(err),
            req_msg.counter,
            req_msg.received_counter);
        }
      }

      /* Trace the upstream machine's state transitions — read-only access
             * to the library's robot_state; the library itself is unchanged.
             * Surfaces the NEED_STOP -> STOP_RECEIVED -> OK arming cycle. */
      if (
        machine.robot_state.robot_state != prev_robot || machine.robot_state.restart_state != prev_restart ||
        machine.robot_state.remote_stop_id != prev_sid)
      {
        fprintf(
          stderr,
          "   STATE robot=%s restart=%s owner=0x%08X active=%u  (trigger: RX %s "
          "from remote 0x%08X; was robot=%s restart=%s)\n",
          rstate_name(machine.robot_state.robot_state),
          restart_name(machine.robot_state.restart_state),
          device_id_for_local(&machine, machine.robot_state.remote_stop_id),
          pstop_remote_num_active(&machine.remotes),
          msg_name(req_msg.message),
          req_msg.id.data,
          rstate_name(prev_robot),
          restart_name(prev_restart));
      }
    }
  }

  fprintf(stderr, "shutting down\n");
  if (announce_started) {
    (void)pthread_join(announce_tid, NULL); /* s_running=0 ends it within 1s */
  }
  transport_udp_close(&udp_transport);
  free(pstop_clients);
  return 0;
}
