// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file main.c — ESP32-S3 pstop MACHINE node ("machn").
 *
 * The machine role of pstop_c on the remote's platform, inverting the
 * remote's lockstep: the remote's two cores independently ENCODE and the
 * comparator transmits only on agreement; here the two cores independently
 * DECODE + DECIDE and the comparator actuates/replies only on agreement.
 *
 *   - Each core runs its OWN pstop_c machine instance (own state, own
 *     remotes table). Every received datagram is processed by BOTH.
 *   - The comparator compares (library verdict, reply bytes). Agreement →
 *     reply sent, relays driven to the agreed state. Disagreement →
 *     internal fault: NO reply (remotes see silence and stop), both relay
 *     outputs commanded open, mismatch counter incremented. Transient by
 *     design — agreement on a later tick resumes normally, like the remote.
 *   - Each core drives ITS OWN relay from ITS OWN instance's verdict; the
 *     two relay contacts are wired in series in the robot's stop circuit,
 *     so either core alone stops the robot. Energized = run; any reset,
 *     brownout, or hang fails to STOP.
 *   - Feedback: a resistor divider on each relay's switched output, read
 *     as a digital input by the comparator every tick. A persistent
 *     commanded-vs-observed contradiction raises a (self-clearing) relay
 *     fault indication; safety never depends on it — the series partner
 *     carries the stop.
 *
 * pstop_c (certification track) is UNMODIFIED — this file is the shell.
 * Design: docs/MACHINE_ESP32_DESIGN.md. Scaffold status: config is
 * compile-time constants (NVS/admin UI wiring is a follow-up), telemetry
 * reuses the dcs comparator/core atomics (machn-specific /state.json
 * fields are a follow-up).
 */

#include <stdatomic.h>
#include <string.h>

#include "dcs_support.h"
#include "esp_log.h"
#include "esp_timer.h"
// clang-format off
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// clang-format on
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "pstop/machine.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"

static const char * TAG = "machn";

/* === Role configuration (compile-time scaffold; NVS/admin UI later) ====== */
#define MACHN_LISTEN_PORT 8890
#define MACHN_MACHINE_ID 0x01020304u /* what remotes put in receiver_id */
#define MACHN_MAX_REMOTES 8
#define MACHN_MAX_LOST_MESSAGES 10u
#define MACHN_MAX_MISSED_HEARTBEATS 3u
#define MACHN_MIN_STOP_MS 500u /* -> library delay_between_stop_ms */
#define MACHN_DEFAULT_HEARTBEAT_MS 400u /* advertised: remotes publish at /2 */

/* === Relay I/O (reuses the remote's E-stop loop pins) ===================== */
#define RELAY_A_DRIVE 39 /* core 0 -> relay 1 coil driver */
#define RELAY_A_SENSE 40 /* divider on relay 1's switched output */
#define RELAY_B_DRIVE 41 /* core 1 -> relay 2 coil driver */
#define RELAY_B_SENSE 42 /* divider on relay 2's switched output */
#define RELAY_FEEDBACK_MS 100u /* commanded->observed settle allowance (TBD by part) */

#define TICK_MS 100u /* comparator/liveness cadence, matches the remote */
#define CORE_WINDOW_MS 80u /* per-core processing budget within a tick */
#define SAFETY_TASK_PRIO 8 /* above WG tasks, below TCPIP — remote rationale */

/* === Per-core machine instances =========================================== */
typedef struct
{
  pstop_application_t app;
  pstop_machine_t machine;
  pstop_remote_data_t remotes[MACHN_MAX_REMOTES];
  /* Outputs of the last processed op, read by the comparator after done. */
  pstop_error_t err;
  pstop_msg_t resp;
  uint8_t resp_bytes[PSTOP_MESSAGE_SIZE];
  int robot_state; /* ROBOT_STATE_OK / ROBOT_STATE_STOPPED after the op */
} machn_core_t;

static machn_core_t g_core[2];
static TaskHandle_t g_core_h[2];
static SemaphoreHandle_t g_done[2];

/* Datagram staged for both cores (comparator writes, cores read). */
static uint8_t g_rx_bytes[PSTOP_MESSAGE_SIZE];
static volatile int g_rx_pending; /* 1 = staged datagram this tick */

/* Relay command/observe state (comparator-owned). */
static volatile int g_relay_cmd[2]; /* last commanded level per channel */
static uint64_t g_relay_cmd_ms[2]; /* when it was commanded (settle window) */
static uint32_t g_relay_fault[2]; /* consecutive contradiction ticks */
static uint32_t g_relay_fault_total;

static uint64_t now_ms(void)
{
  return (uint64_t)(esp_timer_get_time() / 1000);
}

/* === pstop_c callbacks (per-core trampolines — the lib passes no ctx) ==== */

static remote_details_t details_default(const device_id_t * device_id)
{
  (void)device_id;
  remote_details_t d;
  /* Scaffold policy: accept any remote (bench allow_unlisted). The NVS
     * allowlist (allowed / stop_only / per-remote heartbeat) is a follow-up
     * and MUST land before any non-bench deployment. */
  remote_detail_set(&d, true, MACHN_DEFAULT_HEARTBEAT_MS, false);
  return d;
}

/* status_cb fires inside machine_process/validate on the calling core's
 * task; robot_state is snapshotted after the call instead, so these only
 * exist to satisfy the non-NULL callback contract. */
static void status_noop_core0(pstop_status_message_t status)
{
  (void)status;
}

static void status_noop_core1(pstop_status_message_t status)
{
  (void)status;
}

static void log_noop(uint64_t timestamp, const device_id_t * client, uint8_t message, pstop_error_t error)
{
  (void)timestamp;
  (void)client;
  (void)message;
  (void)error;
}

static uint64_t lib_now_core(void)
{
  return now_ms();
}

static void core_instance_init(int c)
{
  machn_core_t * mc = &g_core[c];
  memset(mc, 0, sizeof(*mc));
  pstop_application_init(&mc->app);
  mc->app.app_config.max_lost_messages = MACHN_MAX_LOST_MESSAGES;
  mc->app.app_config.max_missed_heartbeats = MACHN_MAX_MISSED_HEARTBEATS;
  mc->app.app_config.delay_between_stop_ms = MACHN_MIN_STOP_MS;
  mc->app.remote_details_cb = details_default;
  mc->app.status_cb = (c == 0) ? status_noop_core0 : status_noop_core1;
  mc->app.log_message_cb = log_noop;
  mc->app.env.get_time_cb = lib_now_core;
  machine_init(&mc->machine, &mc->app, mc->remotes, MACHN_MAX_REMOTES);
  device_id_t mid = {.data = MACHN_MACHINE_ID};
  device_id_copy(&mc->machine.application->machine_device_id, &mid);
}

/* === Per-core task: process staged op, run liveness, drive own relay ===== */

static void core_task(void * arg)
{
  const int c = (int)(intptr_t)arg;
  machn_core_t * mc = &g_core[c];
  const gpio_num_t drive = (c == 0) ? RELAY_A_DRIVE : RELAY_B_DRIVE;
  uint32_t tick = 0;

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tick++;

    mc->err = PSTOP_ERROR_INVALID_ID; /* placeholder; meaningful only when a datagram was staged */
    if (g_rx_pending != 0) {
      pstop_msg_t req;
      pstop_message_decode(&req, g_rx_bytes);
      memset(&mc->resp, 0, sizeof(mc->resp));
      mc->err = machine_process_message(&mc->machine, &req, &mc->resp);
      if (mc->err == PSTOP_OK) {
        pstop_message_encode(&mc->resp, mc->resp_bytes);
      } else {
        memset(mc->resp_bytes, 0, sizeof(mc->resp_bytes));
      }
    }

    /* Native liveness on this core's own instance/clock. */
    (void)machine_validate_heartbeats(&mc->machine);

    /* This core's verdict drives THIS core's relay, independent of the
         * other core: the series wiring makes any single-core STOP a real
         * stop, so a disagreement is never less safe than agreement. */
    mc->robot_state = mc->machine.robot_state.robot_state;
    (void)gpio_set_level(drive, (mc->robot_state == ROBOT_STATE_OK) ? 1 : 0);

    dcs_publish_core_tick(c, tick, (mc->robot_state == ROBOT_STATE_OK) ? 1u : 0u);
    (void)xSemaphoreGive(g_done[c]);
  }
}

/* === Relay feedback (comparator tick) ===================================== */

static void relay_feedback_check(void)
{
  const gpio_num_t sense[2] = {RELAY_A_SENSE, RELAY_B_SENSE};
  uint64_t t = now_ms();
  for (int ch = 0; ch < 2; ch++) {
    int observed = gpio_get_level(sense[ch]);
    if (observed == g_relay_cmd[ch]) {
      g_relay_fault[ch] = 0; /* matches — indication self-clears */
      continue;
    }
    if ((t - g_relay_cmd_ms[ch]) < RELAY_FEEDBACK_MS) {
      continue; /* still inside the settle window after a transition */
    }
    g_relay_fault[ch]++;
    g_relay_fault_total++;
    if (g_relay_fault[ch] == 1u || (g_relay_fault[ch] % 50u) == 0u) {
      ESP_LOGE(
        TAG,
        "RELAY FAULT ch%c: commanded %d, divider reads %d (x%u) — series partner carries the stop",
        'A' + ch,
        g_relay_cmd[ch],
        observed,
        (unsigned)g_relay_fault[ch]);
    }
  }
}

/* Track what the cores commanded this tick (for the feedback window). */
static void relay_note_commands(void)
{
  for (int c = 0; c < 2; c++) {
    int level = (g_core[c].robot_state == ROBOT_STATE_OK) ? 1 : 0;
    if (level != g_relay_cmd[c]) {
      g_relay_cmd[c] = level;
      g_relay_cmd_ms[c] = now_ms();
    }
  }
}

/* === Comparator: socket, staging, agreement, reply ======================= */

static void comparator_task(void * arg)
{
  (void)arg;

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "FATAL: socket() failed (%d) — clean reboot", errno);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }
  struct sockaddr_in bind_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(MACHN_LISTEN_PORT),
    .sin_addr.s_addr = htonl(INADDR_ANY),
  };
  if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
    ESP_LOGE(TAG, "FATAL: bind(%d) failed (%d) — clean reboot", MACHN_LISTEN_PORT, errno);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }
  ESP_LOGI(TAG, "MACHINE node listening on 0.0.0.0:%d (machine_id=0x%08x)", MACHN_LISTEN_PORT, MACHN_MACHINE_ID);

  uint32_t processed = 0;
  uint32_t mismatch = 0;
  uint64_t last_rx_ms = 0;

  for (;;) {
    /* Wait up to one tick for a datagram; a quiet tick still runs
         * liveness + relay supervision on both cores. */
    struct timeval tv = {.tv_sec = 0, .tv_usec = (int)(TICK_MS * 1000u)};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    int sel = select(sock + 1, &rfds, NULL, NULL, &tv);

    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    g_rx_pending = 0;
    if ((sel > 0) && FD_ISSET(sock, &rfds)) {
      int n = recvfrom(sock, g_rx_bytes, sizeof(g_rx_bytes), 0, (struct sockaddr *)&from, &from_len);
      if (n == PSTOP_MESSAGE_SIZE) {
        g_rx_pending = 1;
        last_rx_ms = now_ms();
      }
      /* Short/garbage datagrams are dropped silently (fail-safe: no
             * reply). Additional queued datagrams are handled on subsequent
             * ticks — remotes publish at >= 200 ms spacing per unit. */
    }

    /* Run both cores against the staged op (or a bare liveness tick). */
    (void)xTaskNotifyGive(g_core_h[0]);
    (void)xTaskNotifyGive(g_core_h[1]);
    bool ok0 = (xSemaphoreTake(g_done[0], pdMS_TO_TICKS(CORE_WINDOW_MS)) == pdTRUE);
    bool ok1 = (xSemaphoreTake(g_done[1], pdMS_TO_TICKS(CORE_WINDOW_MS)) == pdTRUE);

    if (!ok0 || !ok1) {
      /* A core missed its window — internal fault, treat as disagreement:
             * no reply this tick; the hung core's relay is unchanged but the
             * healthy core keeps enforcing its own verdict, and a truly hung
             * core eventually trips the task WDT into a (safe) reset. */
      mismatch++;
    } else if (g_rx_pending != 0) {
      bool agree =
        (g_core[0].err == g_core[1].err) &&
        ((g_core[0].err != PSTOP_OK) || (memcmp(g_core[0].resp_bytes, g_core[1].resp_bytes, PSTOP_MESSAGE_SIZE) == 0));
      if (agree && (g_core[0].err == PSTOP_OK)) {
        (void)sendto(sock, g_core[0].resp_bytes, PSTOP_MESSAGE_SIZE, 0, (struct sockaddr *)&from, from_len);
        processed++;
      } else if (!agree) {
        /* Cores diverged on the SAME input: withhold the reply (the
                 * remote sees silence and stops) and count it. Each core has
                 * already driven its own relay from its own verdict — the
                 * series chain resolves any split toward STOP. */
        mismatch++;
        ESP_LOGW(
          TAG, "LOCKSTEP MISMATCH: core0 err=%d core1 err=%d (reply withheld)", (int)g_core[0].err, (int)g_core[1].err);
      } else {
        /* Both cores rejected identically (bad CRC, unknown sender, ...):
                 * the library contract is no reply. */
      }
    } else {
      /* Quiet tick: liveness + relays already handled by the cores. */
    }

    relay_note_commands();
    relay_feedback_check();

    dcs_publish_comparator(processed, mismatch, g_relay_fault_total, last_rx_ms, 0);
  }
}

/* === Bring-up ============================================================== */

static void relay_gpio_init(void)
{
  gpio_config_t out = {
    .pin_bit_mask = (1ULL << RELAY_A_DRIVE) | (1ULL << RELAY_B_DRIVE),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE, /* default-low = de-energized = STOP */
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&out));
  (void)gpio_set_level(RELAY_A_DRIVE, 0);
  (void)gpio_set_level(RELAY_B_DRIVE, 0);

  gpio_config_t in = {
    .pin_bit_mask = (1ULL << RELAY_A_SENSE) | (1ULL << RELAY_B_SENSE),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&in));
}

void app_main(void);

void app_main(void)
{
  dcs_boot_state_t bs = dcs_support_init();
  (void)bs;

  relay_gpio_init(); /* relays open (STOP) before anything else runs */
  core_instance_init(0);
  core_instance_init(1);

  for (int i = 0; (i < 20) && ((g_done[0] == NULL) || (g_done[1] == NULL)); i++) {
    if (g_done[0] == NULL) {
      g_done[0] = xSemaphoreCreateBinary();
    }
    if (g_done[1] == NULL) {
      g_done[1] = xSemaphoreCreateBinary();
    }
    if ((g_done[0] != NULL) && (g_done[1] != NULL)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if ((g_done[0] == NULL) || (g_done[1] == NULL)) {
    ESP_LOGE(TAG, "FATAL: semaphore alloc failed — clean reboot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  BaseType_t r0 =
    xTaskCreatePinnedToCore(core_task, "core0", 4096, (void *)(intptr_t)0, SAFETY_TASK_PRIO, &g_core_h[0], 0);
  BaseType_t r1 =
    xTaskCreatePinnedToCore(core_task, "core1", 4096, (void *)(intptr_t)1, SAFETY_TASK_PRIO, &g_core_h[1], 1);
  BaseType_t rc = xTaskCreate(comparator_task, "comparator", 5120, NULL, SAFETY_TASK_PRIO, NULL);
  if ((r0 != pdPASS) || (r1 != pdPASS) || (rc != pdPASS) || (g_core_h[0] == NULL) || (g_core_h[1] == NULL)) {
    ESP_LOGE(TAG, "FATAL: task create failed — clean reboot");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
  }

  ESP_LOGI(
    TAG,
    "machn up: dual-core RX lockstep, relays A=%d/%d B=%d/%d",
    RELAY_A_DRIVE,
    RELAY_A_SENSE,
    RELAY_B_DRIVE,
    RELAY_B_SENSE);
}
