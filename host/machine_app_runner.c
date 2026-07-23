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

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pstop/machine.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"
#include "transport/udp/udp_transport.h"

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
  /* Minimum duration (ms) a STOP episode must last for its OK release to
     * count as the arming gesture. Shorter STOP->OK cycles are VETOED (the
     * machine is returned to STOPPED/NEED_STOP via the library's own public
     * machine_stop_robot()) — this is the wrapper-owned defence against
     * EMC-induced loop blips on the remote performing the arming gesture
     * accidentally (observed 2026-07-21: 100-200 ms both-channel blips in
     * WiFi mode re-armed the machine). 0 disables the policy. The library
     * (pstop_c, certification track) is NOT modified: veto uses only public
     * API and lands in the same state as a heartbeat timeout. */
  uint64_t min_stop_ms;
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
  c->max_missed_heartbeats = 1U;
  c->max_remotes = 3U;
  c->verbose = 0;
  c->allow_unlisted = 1;
  c->default_heartbeat_ms = 1000U;
  c->min_stop_ms = 500U;
  c->default_stop_only = 0;
  c->n_operators = 0;
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
static uint64_t machine_now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec) * 1000ULL + ((uint64_t)ts.tv_nsec) / 1000000ULL;
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
    fprintf(stderr, "Robot Status = %s\n", status == PSTOP_STATUS_OK ? "OK" : "STOP");
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
    fprintf(stderr, "pstop 0x%08X -> %s\n", client->data, msg_name(message));
  } else {
    fprintf(stderr, "pstop 0x%08X -> %s  [%s]\n", client->data, msg_name(message), err_name(error));
  }
}

static pstop_application_t pstop_app;
static pstop_machine_t machine;
static udp_transport_data_t udp_transport;

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
  if (g_cfg.port <= 0 || g_cfg.port > 65535) {
    fprintf(stderr, "bad port: %d\n", g_cfg.port);
    return 1;
  }
  if (g_cfg.max_remotes == 0) g_cfg.max_remotes = 1;
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
  pstop_app.remote_details_cb = is_operator_allowed;
  pstop_app.status_cb = robot_status;
  pstop_app.log_message_cb = log_message;
  pstop_app.env.get_time_cb = machine_now_ms;

  machine_init(&machine, &pstop_app, pstop_clients, g_cfg.max_remotes);

  int result = transport_udp_listen(&udp_transport, g_cfg.bind_addr, g_cfg.port);
  if (result < 0) {
    fprintf(stderr, "transport_udp_listen %s:%d failed\n", g_cfg.bind_addr, g_cfg.port);
    return 1;
  }

  device_id_t machine_uuid = {.data = g_cfg.machine_device_id};
  device_id_copy(&(machine.application->machine_device_id), &machine_uuid);

  fprintf(stderr, "machine_app_runner listening on %s:%d\n", g_cfg.bind_addr, g_cfg.port);

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

  while (s_running) {
    /* Native liveness watchdog: pstop_c's own check_heartbeats (machine.c),
         * driven by the machine's monotonic clock, issues the STOP (status_cb)
         * and clears the timed-out remote so a fresh BOND re-bonds. Called every
         * poll cycle (~10 ms) so detection is prompt; the timeout is each
         * operator's heartbeat_ms x (max_missed_heartbeats + 1). */
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
          "  RX %-6s cnt=%-5u rcnt=%-5u from=0x%08X stamp=%llu crc=%s\n",
          msg_name(req_msg.message),
          req_msg.counter,
          req_msg.received_counter,
          req_msg.id.data,
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
          fprintf(stderr, "  (dropped %s from unbonded 0x%08X)\n", msg_name(req_msg.message), req_msg.id.data);
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

      /* --- Min-STOP-duration arming policy (wrapper-owned) ---------
             * A STOP episode must last >= min_stop_ms for its OK release to
             * count as the arming gesture; shorter STOP->OK cycles (e.g.
             * EMC-induced loop blips on the remote, observed 2026-07-21 in
             * WiFi mode) are vetoed. Enforcement uses ONLY public library
             * API: machine_stop_robot() lands in the exact state the
             * library's own heartbeat timeout produces, and the pending
             * reply is rewritten to STOP before it is encoded (the CRC is
             * computed at encode time). pstop_c itself is unchanged.
             * Fail-safe by construction: every path here moves the machine
             * toward STOPPED — a policy bug can cost availability, never
             * cause a spurious arm. */
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
          if (g_cfg.min_stop_ms > 0U && (!s_stop_episode_valid || held < g_cfg.min_stop_ms)) {
            machine_stop_robot(&machine);
            resp_msg.message = PSTOP_MESSAGE_STOP;
            fprintf(
              stderr,
              "ANOMALY: arming VETOED from 0x%08X — STOP held "
              "%llu ms < %llu ms policy minimum (blip?)\n",
              req_msg.id.data,
              (unsigned long long)held,
              (unsigned long long)g_cfg.min_stop_ms);
          } else {
            fprintf(
              stderr,
              "ARMED by 0x%08X: STOP held %llu ms (policy "
              "minimum %llu ms)\n",
              req_msg.id.data,
              (unsigned long long)held,
              (unsigned long long)g_cfg.min_stop_ms);
            robot_status_commit_ok();
          }
          s_stop_episode_valid = 0;
        }
      }

      if (err == PSTOP_OK) {
        pstop_message_encode(&resp_msg, respbytes);
        transport_udp_write(&udp_transport, respbytes, PSTOP_MESSAGE_SIZE, (struct sockaddr_in *)&client);
        if (g_cfg.verbose) fprintf(stderr, "  TX %-6s cnt=%u\n", msg_name(resp_msg.message), resp_msg.counter);
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
          long r2m = (long)req_msg.counter - (long)c->remote_data.last_sent_counter;
          long lag = (long)c->remote_data.msg_counter - (long)req_msg.received_counter;
          fprintf(
            stderr,
            "ANOMALY: %s rejected by pstop — %s | "
            "remote->machine step=%ld (machine last accepted #%u, got #%u) ; "
            "machine->remote ack-lag=%ld (machine sent #%u, remote acked #%u)\n",
            msg_name(req_msg.message),
            err_name(err),
            r2m,
            c->remote_data.last_sent_counter,
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
          "  STATE robot=%s restart=%s owner=0x%08X active=%u  (from pstop "
          "0x%08X %s; was robot=%s restart=%s)\n",
          rstate_name(machine.robot_state.robot_state),
          restart_name(machine.robot_state.restart_state),
          device_id_for_local(&machine, machine.robot_state.remote_stop_id),
          pstop_remote_num_active(&machine.remotes),
          req_msg.id.data,
          msg_name(req_msg.message),
          rstate_name(prev_robot),
          restart_name(prev_restart));
      }
    }
  }

  fprintf(stderr, "shutting down\n");
  transport_udp_close(&udp_transport);
  free(pstop_clients);
  return 0;
}
