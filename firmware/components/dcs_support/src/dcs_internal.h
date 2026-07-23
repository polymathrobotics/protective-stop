// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_internal.h
 * @brief Shared state between the dcs_support .c files. Not part of the
 * public API; lives under src/ so consumers can't see it.
 */

#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "dcs_support.h" /* DCS_PSTOP_MAX_MACHINES + public telemetry API */
#include "esp_err.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microlink.h"
#include "ml_app.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DCS_NVS_NS "dcs_app"
#define DCS_NVS_KEY_USB_EN "usb_en"
#define DCS_NVS_KEY_BOOT_COUNT "boot_cnt"
#define DCS_NVS_KEY_PSTOP_IP "ps_ip"
#define DCS_NVS_KEY_PSTOP_PORT "ps_port"
#define DCS_NVS_KEY_TS_BOOT_EN "ts_boot"
#define DCS_NVS_KEY_RST_HIST "rst_hist" /* ring of recent esp_reset_reason() */
#define DCS_NVS_KEY_PSTOP_NUM "ps_num" /* USB "PSTOPxx" unit number (0=auto) */
#define DCS_NVS_KEY_RING_OFF "ring_off" /* ring rotation: physical index of LED 1 */
#define DCS_NVS_KEY_PSTOP_PEERS "ps_peers" /* multi-machine peer table (blob, see dcs_nvs.c) */

#define DCS_RST_HIST_LEN 16

#define DCS_PSTOP_PEER_DEFAULT_IP ((10u << 24) | (42u << 16) | (0u << 8) | 1u)
#define DCS_PSTOP_PEER_DEFAULT_PORT 8890

#define DCS_SAFETY_MAX_RAPID_BOOTS 3
#define DCS_SAFETY_CLEAR_AFTER_MS 120000

/* Network-interface route priorities. Higher wins the default route.
 * Ethernet must beat USB-NCM (bumped to 110 in ml_dev_tether) and WiFi STA
 * (100, IDF default). The dcs_net_supervisor enforces the same order at 1 Hz. */
#define DCS_ETH_ROUTE_PRIO 128

  /* Active-interface codes published by dcs_net_supervisor for /state.json. */
  typedef enum
  {
    DCS_IFACE_NONE = 0,
    DCS_IFACE_ETH = 1, /* W5500 wired Ethernet */
    DCS_IFACE_USB = 2, /* USB-CDC-NCM tether   */
    DCS_IFACE_WIFI = 3, /* WiFi STA             */
    DCS_IFACE_AP = 4, /* SoftAP setup mode    */
  } dcs_iface_t;

  /* esp_netif if_key for an interface code, or NULL for NONE. Single source of
 * truth shared by the supervisor, net-liveness, and the RGB LED so the keys
 * can't drift between modules. Keys must match how each netif is created:
 * dcs_eth.c ("DCS_ETH"), ml_dev_tether.c ("USB_NCM"), and IDF's
 * esp_netif_create_default_wifi_sta/ap ("WIFI_STA_DEF" / "WIFI_AP_DEF"). */
  static inline const char * dcs_iface_ifkey(dcs_iface_t iface)
  {
    switch (iface) {
      case DCS_IFACE_ETH:
        return "DCS_ETH";
      case DCS_IFACE_USB:
        return "USB_NCM";
      case DCS_IFACE_WIFI:
        return "WIFI_STA_DEF";
      case DCS_IFACE_AP:
        return "WIFI_AP_DEF";
      default:
        return NULL;
    }
  }

  /* Resolved boot state — exact same shape as dcs_boot_state_t but referenced
 * from inside the component to avoid a circular include. */
  typedef struct
  {
    bool usb_enabled;
    bool ts_boot_en;
    bool derp_only_mode;
    uint16_t boot_count;
    uint8_t reset_reason;
    microlink_t * ml_handle;
    ml_app_t * app;
  } dcs_state_t;

  /* The one process-wide instance. Populated by dcs_support_init(). */
  extern dcs_state_t g_dcs;

  /* Atomic telemetry sinks — published by main.c via dcs_publish_*, read by
 * dcs_admin_pages.c's /state.json handler. */
  extern atomic_uint_fast32_t g_dcs_core_tick[2];
  extern atomic_uint_fast32_t g_dcs_core_verdict[2];
  extern atomic_uint_fast32_t g_dcs_load_pct[2];

  /* E-stop loop diagnostics, published by main.c's core tasks (per channel):
 * high_ok=1 → last drive-high tick echoed high (loop continuity proven);
 * low_ok=1  → last drive-low tick echoed low (no stuck-high short).
 * Both 1 ⇒ that channel's loop is closed-and-healthy. Read by /state.json so a
 * single-leg fault (which the DPST button can't reproduce — it opens both
 * poles) is pinpointable: which channel, and continuity-loss vs stuck-high. */
  extern atomic_uint_fast32_t g_dcs_estop_high_ok[2];
  extern atomic_uint_fast32_t g_dcs_estop_low_ok[2];

  extern atomic_uint_fast32_t g_dcs_pstop_sent;
  extern atomic_uint_fast32_t g_dcs_pstop_replies; /* machine replies received */
  extern atomic_uint_fast32_t g_dcs_pstop_last_msg; /* last message TYPE from the machine (PSTOP_MESSAGE_*) */
  extern atomic_uint_fast32_t g_dcs_pstop_mismatch;
  extern atomic_uint_fast32_t g_dcs_pstop_send_fail;
  extern atomic_uint_fast32_t g_dcs_pstop_rebonds; /* link re-syncs after reply loss */
  extern atomic_uint_fast32_t g_dcs_pstop_rtt_ms;
  extern atomic_uint_fast64_t g_dcs_pstop_last_reply_ms;
  extern atomic_uint_fast32_t g_dcs_pstop_peer_ip;
  extern atomic_uint_fast32_t g_dcs_pstop_peer_port;

  /* === Multi-machine peer table + per-machine telemetry. ==================
 *
 * Up to DCS_PSTOP_MAX_MACHINES machine targets (slots). Slot 0 is mirrored
 * into the legacy g_dcs_pstop_peer_ip/port atomics and the legacy NVS keys
 * so old tooling (and a firmware rollback) keep working.
 *
 * Endpoint packing: one u64 atomic per slot =
 *      (configured?1:0) << 63 | ip(32) << 16 | port(16)
 * The machine_id lives in a SECOND atomic per slot, so a reader can see a
 * torn slot (new endpoint + old id) for one tick during an update. That is
 * benign by design: the wrong-id packet is rejected by the machine and the
 * session re-bonds on the next tick — fail-safe, never fail-wrong. */

  extern atomic_uint_fast64_t g_dcs_pstop_slot_ep[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_slot_id[DCS_PSTOP_MAX_MACHINES];

  /* Per-machine telemetry sinks, published by main.c's comparator via
 * dcs_publish_pstop_machine(); read by /state.json and the LED ring. */
  extern atomic_uint_fast32_t g_dcs_pstop_m_sent[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_replies[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_send_fail[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_rebonds[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_rtt_ms[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_hb_ms[DCS_PSTOP_MAX_MACHINES]; /* machine-requested heartbeat window */
  extern atomic_uint_fast32_t g_dcs_pstop_m_last_msg[DCS_PSTOP_MAX_MACHINES];
  extern atomic_uint_fast32_t g_dcs_pstop_m_state[DCS_PSTOP_MAX_MACHINES]; /* 0=idle 1=bonding 2=bonded */
  extern atomic_uint_fast64_t g_dcs_pstop_m_last_reply_ms[DCS_PSTOP_MAX_MACHINES];

  /* Set + persist one peer slot (0..DCS_PSTOP_MAX_MACHINES-1). configured=false
 * clears the slot. Slot 0 also updates the legacy atomics + NVS keys. Used by
 * the /api/pstop_peer(s) handlers. */
  esp_err_t dcs_pstop_set_peer_slot(int slot, bool configured, uint32_t ip, uint16_t port, uint32_t machine_id);

  /* WG-manager pause state (legacy /api/wg toggle). */
  extern TaskHandle_t g_dcs_wg_handle;
  extern atomic_int g_dcs_wg_paused;

  /* Heap-watermark snapshot maintained by load_sampler. */
  extern atomic_uint_fast32_t g_dcs_heap_min_internal;

  /* Currently-active network interface (dcs_iface_t), published by the
 * net supervisor every second, read by /state.json. */
  extern atomic_int g_dcs_active_iface;

  /* RGB status-LED loop counter — incremented once per blink cycle by dcs_rgb,
 * read by /state.json so the task's liveness is observable (a frozen counter
 * means the LED task is wedged). */
  extern atomic_uint_fast32_t g_dcs_rgb_cycles;

  /* === Task creation convention (dcs_support.c). ===
 *
 * Spawn a task whose TCB+stack live in PSRAM (external SPI RAM) rather than the
 * scarce internal SRAM, so the internal heap stays free for the networking/TLS
 * stack — the binding memory constraint on this device. Use this for every
 * NON-safety-critical task.
 *
 * The task body MUST obey two rules or it will crash:
 *   1. Never write flash/NVS or otherwise disable the flash cache — a
 *      PSRAM-resident stack is unreadable while the cache is down.
 *   2. Never be accessed from an ISR.
 * The lockstep cores, the comparator (which sends pstop), the watchdog feeder,
 * and any NVS-writing task therefore stay on INTERNAL stacks (plain
 * xTaskCreate*).
 *
 * stack is in BYTES (ESP-IDF convention). core is the core to pin to, or
 * tskNO_AFFINITY. Falls back to an internal-RAM stack if PSRAM is exhausted, so
 * a shortfall degrades rather than silently dropping the task. Returns the task
 * handle, or NULL on outright failure. */
  TaskHandle_t dcs_task_spawn_psram(
    TaskFunction_t fn, const char * name, uint32_t stack, void * arg, UBaseType_t prio, BaseType_t core);

  /* === Per-file bring-up hooks (internal). === */

  /* dcs_nvs.c */
  bool dcs_nvs_read_usb_enabled(void);
  esp_err_t dcs_nvs_write_usb_enabled(bool enable);
  bool dcs_nvs_read_ts_boot_en(void);
  esp_err_t dcs_nvs_write_ts_boot_en(bool enable);
  uint16_t dcs_nvs_read_boot_count(void);
  esp_err_t dcs_nvs_write_boot_count(uint16_t v);
  uint32_t dcs_nvs_read_pstop_peer_ip(void);
  uint16_t dcs_nvs_read_pstop_peer_port(void);
  esp_err_t dcs_nvs_write_pstop_peer(uint32_t ip, uint16_t port);
  /* Reset-reason history ring: push records this boot's esp_reset_reason() (oldest
 * shifted out); read fills out[] newest-last and returns the count. Lets a crash
 * pattern be seen remotely via /state.json even across later clean reboots. */
  void dcs_nvs_push_reset_reason(uint8_t reason);
  int dcs_nvs_read_reset_history(uint8_t * out, int max);
  uint8_t dcs_nvs_read_pstop_unit_num(void); /* 0 = auto (chip-ID derived) */
  esp_err_t dcs_nvs_write_pstop_unit_num(uint8_t n);
  /* Ring rotation offset: the PHYSICAL pixel index (0..15, clockwise from the
 * data-in pad) that the installed bezel makes "LED 1". Absent -> 0. */
  uint8_t dcs_nvs_read_ring_offset(void);
  esp_err_t dcs_nvs_write_ring_offset(uint8_t off);

  /* Multi-machine peer table (ps_peers blob). One record per slot. Read
 * falls back to migrating the legacy ps_ip/ps_port pair into slot 0 when
 * the blob is absent (first boot on this firmware). */
  typedef struct
  {
    bool configured;
    uint32_t ip; /* host byte order */
    uint16_t port;
    uint32_t machine_id; /* pstop_msg.receiver_id for this machine */
  } dcs_pstop_peer_rec_t;

  void dcs_nvs_read_pstop_peers(dcs_pstop_peer_rec_t out[DCS_PSTOP_MAX_MACHINES]);
  esp_err_t dcs_nvs_write_pstop_peers(const dcs_pstop_peer_rec_t recs[DCS_PSTOP_MAX_MACHINES]);

  /* dcs_safety.c */
  void dcs_safety_account_boot(void); /* increments crash counter + applies
                                            * rollback decision; may not return */
  void dcs_safety_start_heartbeat(void); /* TWDT-fed task */
  void dcs_safety_mark_ota_valid(void);
  void dcs_safety_start_bc_clear(void); /* Layer 4 age-out */
  /* Stamp an RTC_NOINIT flag marking the imminent abort() as a DELIBERATE
 * net_liveness wedge-recovery reboot, so the next boot's crash accounting does
 * NOT count it toward the firmware-rollback ladder (network/upstream conditions
 * must never downgrade the image — only a genuine code fault should). */
  void dcs_safety_mark_liveness_abort(void);

  /* dcs_boot.c */
  esp_err_t dcs_boot_try_tether_or_skip(uint32_t timeout_ms);
  void dcs_boot_wifi_idle_init(void);
  bool dcs_boot_alt_network_won(void); /* Ethernet/USB took the default route */
  esp_err_t dcs_usb_set_enabled(bool on); /* admin toggle: USB-CDC-NCM tether */
  bool dcs_usb_is_enabled(void);

  /* dcs_eth.c — W5500 SPI Ethernet (highest-priority netif). */
  esp_err_t dcs_eth_start(void); /* init + start; non-blocking */
  bool dcs_eth_wait_for_ip(uint32_t timeout_ms); /* block for boot gating */
  bool dcs_eth_link_up(void);
  bool dcs_eth_has_ip(void);
  esp_err_t dcs_eth_set_enabled(bool on); /* admin toggle: stop/start driver */
  bool dcs_eth_is_enabled(void);

  /* dcs_wifi.c — runtime WiFi-STA enable/disable (lazy init + failover). */
  esp_err_t dcs_wifi_set_enabled(bool on);
  bool dcs_wifi_is_enabled(void);
  void dcs_wifi_status(int * reason, int * connected, int * idx, int * count);

  /* dcs_net_supervisor.c */
  void dcs_net_supervisor_start(void); /* 1 Hz interface-priority enforcer */

  /* dcs_rgb.c — onboard WS2812 (GPIO21): colour=active iface, blink=IP last octet. */
  void dcs_rgb_start(void);

  /* dcs_pstop_ring.c — early power-on sign of life: init the ring's RMT channel
 * and paint all LEDs dim purple. Call FIRST in dcs_support_init(), before any
 * network bring-up; the ring task later reuses the channel and overwrites the
 * colour with the real link state. */
  void dcs_pstop_ring_bootsign(void);

  /* dcs_pstop_ring.c — 16-LED WS2812 ring (GPIO17): solid colour = PSTOP safety
 * state (green=RUN, red=STOP, amber=link-lost, magenta=lockstep fault). */
  void dcs_pstop_ring_start(void);

  /* Ring rotation offset (live value, mirrors NVS ring_off): rotates every frame
 * so logical LED 1 lands on the installed bezel's physical LED-1 position.
 * set clamps to 0..15, takes effect on the next repaint (<=250 ms); the caller
 * persists to NVS separately (dcs_nvs_write_ring_offset). */
  void dcs_pstop_ring_set_offset(uint8_t off);
  uint8_t dcs_pstop_ring_get_offset(void);

/* Locate mode: paint ONLY logical LED 1 solid white so an installer can see /
 * verify the rotation offset. Overrides the state colours; auto-expires after
 * DCS_RING_LOCATE_TIMEOUT_MS so a forgotten locate can't mask STOP/OK forever. */
#define DCS_RING_LOCATE_TIMEOUT_MS 300000u /* 5 min */
  void dcs_pstop_ring_locate(bool on);
  bool dcs_pstop_ring_locate_active(void);

  /* dcs_net_liveness.c */
  void dcs_net_liveness_start(void); /* esp_ping + watchdog task */
  /* Active-uplink gateway RTT telemetry (device-side latency, doesn't traverse the
 * monitoring host). rtt_ms: last; rtt_max_ms: peak since boot; replies/losses:
 * cumulative gateway-ping counts. Any pointer may be NULL. */
  void dcs_net_liveness_stats(uint32_t * rtt_ms, uint32_t * rtt_max_ms, uint32_t * replies, uint32_t * losses);
  /* ms the active uplink's gateway has been silent (0 = never armed / answering).
 * Used by the supervisor (with a pstop cross-check) to fail over off an uplink
 * whose upstream died but whose netif still holds a stale DHCP lease. */
  uint32_t dcs_net_liveness_gw_silent_ms(void);

  /* dcs_net_inet.c — internet-reachability probe (1.1.1.1 / 8.8.8.8), display
 * + telemetry only; NEVER reboots. The status LED blinks red while down. */
  void dcs_net_inet_start(void);
  bool dcs_net_inet_down(void); /* true once both targets silent ~20s */
  uint32_t dcs_net_inet_silent_ms(void); /* ms since either target last replied */

  /* dcs_telemetry.c */
  void dcs_telemetry_start_sampler(void);

  /* Snapshot of the most-recent task-breakdown buffer (called by /state.json) */
  typedef struct
  {
    char name[16];
    uint32_t pct;
  } dcs_task_share_t;

#define DCS_TOP_N 4

  typedef struct
  {
    dcs_task_share_t tasks[DCS_TOP_N];
    uint8_t n;
    uint32_t other_pct;
  } dcs_task_bucket_t;

  typedef struct
  {
    dcs_task_bucket_t b[3]; /* core0, core1, shared */
  } dcs_task_breakdown_t;

  void dcs_telemetry_snapshot(dcs_task_breakdown_t * out);

  /* dcs_admin_pages.c */
  void dcs_admin_pages_register(ml_app_t * app);

#ifdef __cplusplus
}
#endif
