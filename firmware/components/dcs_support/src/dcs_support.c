// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dcs_support.c
 * @brief Top-level bring-up orchestrator. Calls into the per-file modules
 * (dcs_nvs, dcs_safety, dcs_boot, dcs_net_liveness, dcs_telemetry,
 * dcs_admin_pages) in the right order.
 */

#include "dcs_support.h"

#include <stdatomic.h>
#include <string.h>

#include "dcs_identity.h"
#include "dcs_internal.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h" /* xTaskCreatePinnedToCoreWithCaps */
#include "freertos/task.h"
#include "microlink.h"
#include "ml_app.h"
#include "ml_dev_tether.h"
#include "nvs_flash.h"
#include "panic_log.h"

static const char * TAG = "dcs_support";

/* See the doc comment on the declaration in dcs_internal.h for the contract a
 * PSRAM-stack task must honour (no flash/NVS, no ISR access). */
TaskHandle_t dcs_task_spawn_psram(
  TaskFunction_t fn, const char * name, uint32_t stack, void * arg, UBaseType_t prio, BaseType_t core)
{
  TaskHandle_t h = NULL;
  if (
    (xTaskCreatePinnedToCoreWithCaps(fn, name, stack, arg, prio, &h, core, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) ==
     pdPASS) &&
    (h != NULL))
  {
    ESP_LOGI(TAG, "task '%s' (%u B stack) created in PSRAM", name, (unsigned)stack);
    return h;
  }
  ESP_LOGW(TAG, "PSRAM stack for '%s' unavailable — falling back to internal RAM", name);
  h = NULL;
  if (xTaskCreatePinnedToCore(fn, name, stack, arg, prio, &h, core) == pdPASS) {
    return h;
  }
  ESP_LOGE(TAG, "task '%s' create FAILED", name);
  return NULL;
}

/* v15.27 — graceful Tailscale quiesce on any esp_restart() path.
 *
 * Runs from esp_register_shutdown_handler(), which fires for *any*
 * call to esp_restart() (admin restart, /api/usb_enable, the v15.24
 * auto-recovery, the never-brick fallback, etc) but does NOT fire on
 * panic. So this hook only mitigates *graceful* restarts where some
 * WG packet processing might be mid-flight. Work is limited to what's
 * safe with interrupts about to be disabled: a flag flip + a
 * vTaskSuspend, both effectively instant.
 *
 * Motivation: a single panic seen during a WiFi→USB mode transition,
 * backtrace lost (no serial on the test rig). Suspected race between
 * in-flight WG decrypt on ml_wg_mgr and esp_restart_noos's task-kill
 * sequence. Pausing DERP + suspending wg_mgr means no new packets
 * enter the processing pipeline from this point; anything already
 * mid-flight finishes within microseconds of the hook running. */
static void dcs_tailscale_quiesce_on_restart(void)
{
  microlink_pause_derp(true);
  if (g_dcs_wg_handle != NULL) {
    vTaskSuspend(g_dcs_wg_handle);
  }
}

/* === Process-wide state (declared extern in dcs_internal.h). =============== */
dcs_state_t g_dcs;

atomic_uint_fast32_t g_dcs_core_tick[2];
atomic_uint_fast32_t g_dcs_core_verdict[2];
atomic_uint_fast32_t g_dcs_load_pct[2];
atomic_uint_fast32_t g_dcs_estop_high_ok[2];
atomic_uint_fast32_t g_dcs_estop_low_ok[2];

atomic_uint_fast32_t g_dcs_pstop_sent;
atomic_uint_fast32_t g_dcs_pstop_replies;
atomic_uint_fast32_t g_dcs_pstop_last_msg; /* last PSTOP_MESSAGE_* received from the machine */
atomic_uint_fast32_t g_dcs_pstop_mismatch;
atomic_uint_fast32_t g_dcs_pstop_send_fail;
atomic_uint_fast32_t g_dcs_pstop_rtt_ms;
atomic_uint_fast64_t g_dcs_pstop_last_reply_ms;
atomic_uint_fast32_t g_dcs_pstop_peer_ip;
atomic_uint_fast32_t g_dcs_pstop_peer_port;

atomic_uint_fast64_t g_dcs_pstop_slot_ep[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_slot_id[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_sent[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_replies[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_send_fail[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_rebonds[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_rtt_ms[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_hb_ms[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_last_msg[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast32_t g_dcs_pstop_m_state[DCS_PSTOP_MAX_MACHINES];
atomic_uint_fast64_t g_dcs_pstop_m_last_reply_ms[DCS_PSTOP_MAX_MACHINES];

/* Endpoint packing for g_dcs_pstop_slot_ep (see dcs_internal.h). */
#define PSTOP_EP_CONFIGURED (1ULL << 63)

static uint64_t pstop_ep_pack(bool configured, uint32_t ip, uint16_t port)
{
  return (configured ? PSTOP_EP_CONFIGURED : 0ULL) | ((uint64_t)ip << 16) | (uint64_t)port;
}

TaskHandle_t g_dcs_wg_handle;
atomic_int g_dcs_wg_paused;

atomic_uint_fast32_t g_dcs_heap_min_internal = UINT32_MAX;

atomic_int g_dcs_active_iface;
atomic_uint_fast32_t g_dcs_rgb_cycles;
atomic_uint_fast32_t g_dcs_pstop_rebonds;

/* === Public API ============================================================ */

dcs_boot_state_t dcs_support_init(void)
{
  /* RTC_NOINIT log capture: must be first so we don't miss early boot logs. */
  panic_log_init();

  /* Power-on sign of life: whole ring dim purple, before any network or
     * NVS work. Overwritten by the ring task once bring-up completes. */
  dcs_pstop_ring_bootsign();

  /* v15.27: register the Tailscale-quiesce shutdown hook before anything
     * else can call esp_restart(). Idempotent — runs once per process. */
  esp_err_t hook_err = esp_register_shutdown_handler(dcs_tailscale_quiesce_on_restart);
  if (hook_err != ESP_OK) {
    ESP_LOGW(TAG, "shutdown handler register: %s", esp_err_to_name(hook_err));
  }

  /* NVS up first — both our boot-counter and the usb_enabled flag live there. */
  esp_err_t nvs_err = nvs_flash_init();
  if ((nvs_err == ESP_ERR_NVS_NO_FREE_PAGES) || (nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
    esp_err_t erase_err = nvs_flash_erase();
    if (erase_err != ESP_OK) {
      ESP_LOGW(TAG, "nvs_flash_erase: %s", esp_err_to_name(erase_err));
    }
    nvs_err = nvs_flash_init();
  }
  if (nvs_err != ESP_OK) {
    ESP_LOGW(TAG, "nvs_flash_init early: %s", esp_err_to_name(nvs_err));
  }

  /* Boot-counter accounting + rollback decision. Does NOT return if it
     * decides to invoke esp_ota_mark_app_invalid_rollback_and_reboot(). */
  dcs_safety_account_boot();

  /* Resolve NVS-backed user toggles. */
  g_dcs.usb_enabled = dcs_nvs_read_usb_enabled();
  g_dcs.ts_boot_en = dcs_nvs_read_ts_boot_en();

  /* Safety ladder: boot_count==1 → drop direct-UDP path only;
     *                 boot_count>=2 → full Tailscale pause. */
  if (g_dcs.ts_boot_en && (g_dcs.boot_count == 1u)) {
    ESP_LOGW(TAG, "SAFETY: bc=1 — forcing DERP-only mode for this boot");
    g_dcs.derp_only_mode = true;
  } else if (g_dcs.ts_boot_en && (g_dcs.boot_count >= 2u)) {
    ESP_LOGW(
      TAG,
      "SAFETY: ts_boot=1 but boot_count=%u — forcing Tailscale "
      "paused for this boot (safe mode)",
      g_dcs.boot_count);
    g_dcs.ts_boot_en = false;
  } else {
    /* boot_count 0: no safety-ladder action */
  }
  ESP_LOGI(
    TAG, "Boot: usb_enabled=%d ts_boot_en=%d boot_count=%u", g_dcs.usb_enabled, g_dcs.ts_boot_en, g_dcs.boot_count);

  /* TWDT keep-alive. */
  dcs_safety_start_heartbeat();

  /* USB "PSTOPxx" unit number: admin-set value from NVS, or 0 = derive from
     * the chip ID. Must be set before ml_app_start (its try_alt_network hook can
     * bring up the USB tether, which fixes the descriptor at TinyUSB install). */
  ml_dev_tether_set_unit_number(dcs_nvs_read_pstop_unit_num());

  /* ml_app config — same defaults as the pre-refactor main.c. */
  ml_app_config_t cfg = ML_APP_CONFIG_DEFAULT();
  cfg.max_peers = 128; /* PSRAM-backed; load paths budgeted */
  /* Register on the tailnet as "pstop-01xxxxxx", matching the black-channel
     * device ID (dcs_identity_*). Stable per unit; static buffer, safe to hold. */
  cfg.device_name = dcs_identity_hostname();
  ESP_LOGI(
    TAG,
    "Identity: tailnet=%s  pstop device_id=0x%08x "
    "(machine must allow_unlisted or list this ID)",
    dcs_identity_hostname(),
    (unsigned)dcs_identity_device_id());
  cfg.enable_disco = !g_dcs.derp_only_mode;
  cfg.enable_stun = !g_dcs.derp_only_mode;
  cfg.try_alt_network = dcs_boot_try_tether_or_skip;
  cfg.alt_network_timeout_ms = 30000;
  /* httpd URI-handler budget. The server holds 16 + this many slots
     * (ml_app.c). microlink registers ~20 of its own (config panel + /admin
     * API + fleet-ota + verbose) and dcs_admin_pages registers 18; at 16 the
     * total overflowed 32 and the LAST-registered app routes silently failed
     * to register (observed: /api/pstop_num and /api/enter_download 404'd on
     * shipped firmware). 26 gives 42 total slots with headroom — re-check this
     * arithmetic whenever a route is added on either side. */
  cfg.max_user_uri_handlers = 26;
  g_dcs.app = ml_app_start(&cfg);
  g_dcs.ml_handle = ml_app_get_microlink(g_dcs.app);

  /* If an alt network took the default route, ml_app skipped wifi_init.
     * Bring the WiFi driver up in idle mode so admin endpoints touching
     * esp_wifi_* behave — EXCEPT on the wired-Ethernet path, where WiFi is
     * never used and esp_wifi_init() would burn ~30-50 KB of internal RAM we
     * can't spare (internal heap runs tight on this build; that pressure was
     * tipping the chip into intermittent allocation-failure panics). The
     * esp_wifi_* admin calls return ESP_ERR_WIFI_NOT_INIT rather than crash
     * when the driver is absent, so skipping it is safe. */
  if (dcs_boot_alt_network_won()) {
    if (dcs_eth_has_ip()) {
      /* Wired Ethernet: do NOT init the WiFi driver. It costs ~20-25 KB
             * internal RAM, and MEASURED (2026-06-23): always-initialising it
             * here on the eth path crushed heap_min to ~1.9 KB once the Tailscale
             * DERP-TLS handshake ran concurrently (vs ~46 KB skipped) and started
             * tripping early-boot reboots — i.e. it re-creates the very OOM
             * pressure the skip exists to avoid. The esp_wifi_* NULL-deref this
             * once caused in the admin status endpoints (an unguarded call on the
             * not-init driver crash-looped the device) is fixed at the call sites
             * instead — see the guards in dcs_admin_pages.c (/state.json) and
             * ml_config_httpd.c (/api/monitor). */
      ESP_LOGI(
        TAG,
        "wired Ethernet active — skipping WiFi driver init "
        "to preserve internal RAM (esp_wifi admin calls guarded)");
    } else {
      dcs_boot_wifi_idle_init(); /* USB-NCM path: keep WiFi driver */
    }
  }

  /* Onboard WS2812 RGB LED: colour = active interface (live), blink = IP
     * last octet. Replaces ml_ip_blink (which drove GPIO21 as a plain pin —
     * can't clock the board's addressable LED). */
  dcs_rgb_start();

  /* 16-LED WS2812 ring on GPIO17 — PSTOP safety state (separate from the
     * onboard single LED's network indicator). */
  dcs_pstop_ring_start();

  /* Initial peer targets (NVS-backed, legacy single peer migrated into
     * slot 0) — main.c reads the slots via dcs_get_pstop_peer_slot() before
     * and during the comparator's run. Slot 0 mirrors to the legacy atomics. */
  {
    dcs_pstop_peer_rec_t peers[DCS_PSTOP_MAX_MACHINES];
    dcs_nvs_read_pstop_peers(peers);
    for (int i = 0; i < DCS_PSTOP_MAX_MACHINES; i++) {
      atomic_store(&g_dcs_pstop_slot_ep[i], pstop_ep_pack(peers[i].configured, peers[i].ip, peers[i].port));
      atomic_store(&g_dcs_pstop_slot_id[i], peers[i].machine_id);
    }
    atomic_store(&g_dcs_pstop_peer_ip, peers[0].configured ? peers[0].ip : 0u);
    atomic_store(&g_dcs_pstop_peer_port, peers[0].port);
  }

  /* Register the admin pages now that ml_app is up. */
  dcs_admin_pages_register(g_dcs.app);

  /* Verbose ml_* logging — kept from pre-refactor; helps catch wedges. */
  esp_log_level_set("ml_wg_mgr", ESP_LOG_INFO);
  esp_log_level_set("ml_net_io", ESP_LOG_INFO);
  esp_log_level_set("ml_derp", ESP_LOG_INFO);
  esp_log_level_set("wireguard_lwip", ESP_LOG_INFO);
  esp_log_level_set("microlink", ESP_LOG_INFO);

  dcs_telemetry_start_sampler();
  dcs_net_supervisor_start(); /* 1 Hz Eth>USB>WiFi default-route enforcer */
  dcs_net_liveness_start();
  dcs_net_inet_start(); /* internet reachability -> red LED when down */

  /* Cache the ml_wg_mgr handle for the legacy /api/wg toggle. */
  g_dcs_wg_handle = xTaskGetHandle("ml_wg_mgr");

  /* Mirror into the public boot-state struct. */
  dcs_boot_state_t bs = {
    .usb_enabled = g_dcs.usb_enabled,
    .ts_boot_en = g_dcs.ts_boot_en,
    .derp_only_mode = g_dcs.derp_only_mode,
    .boot_count = g_dcs.boot_count,
    .reset_reason = g_dcs.reset_reason,
    .ml_handle = g_dcs.ml_handle,
    .app = g_dcs.app,
  };
  return bs;
}

void dcs_support_finalize(const dcs_boot_state_t * bs)
{
  (void)bs;
  /* Mark current OTA partition valid — does NOT clear boot_count, so the
     * crash-loop signal survives until the Layer-4 age-out fires. */
  dcs_safety_mark_ota_valid();

  /* Layer 4: age-out the crash counter after sustained healthy uptime,
     * and (v15.24) auto-restart out of derp-only mode on success. */
  dcs_safety_start_bc_clear();

  /* If Tailscale was supposed to start paused this boot, suspend
     * ml_wg_mgr now (after our NVS reads completed earlier). The 500 ms
     * delay lets wg_mgr finish its own NVS peer-cache load. */
  if ((!g_dcs.ts_boot_en) && (g_dcs_wg_handle != NULL)) {
    microlink_pause_derp(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskSuspend(g_dcs_wg_handle);
    atomic_store(&g_dcs_wg_paused, 1);
    ESP_LOGW(TAG, "ml_wg_mgr task suspended + DERP paused at boot");
  } else {
    ESP_LOGI(TAG, "Tailscale tasks active at boot (safety net: 4-crash rollback)");
  }
}

/* === Telemetry sinks ======================================================= */

void dcs_publish_core_tick(int core_id, uint32_t tick, uint8_t verdict)
{
  if ((core_id < 0) || (core_id > 1)) {
    return;
  }
  atomic_store(&g_dcs_core_tick[core_id], tick);
  atomic_store(&g_dcs_core_verdict[core_id], verdict);
}

void dcs_publish_estop(int core_id, bool high_ok, bool low_ok)
{
  if ((core_id < 0) || (core_id > 1)) {
    return;
  }
  atomic_store(&g_dcs_estop_high_ok[core_id], high_ok ? 1u : 0u);
  atomic_store(&g_dcs_estop_low_ok[core_id], low_ok ? 1u : 0u);
}

void dcs_publish_comparator(
  uint32_t sent, uint32_t mismatch, uint32_t send_fail, uint64_t last_reply_ms, uint32_t rtt_ms)
{
  atomic_store(&g_dcs_pstop_sent, sent);
  atomic_store(&g_dcs_pstop_mismatch, mismatch);
  atomic_store(&g_dcs_pstop_send_fail, send_fail);
  atomic_store(&g_dcs_pstop_last_reply_ms, last_reply_ms);
  atomic_store(&g_dcs_pstop_rtt_ms, rtt_ms);
}

void dcs_publish_pstop_peer(uint32_t peer_ip, uint16_t peer_port)
{
  /* Legacy single-peer setter (/api/pstop_peer, fleet tooling) = slot 0
   * with the default machine id. */
  esp_err_t werr = dcs_pstop_set_peer_slot(0, true, peer_ip, peer_port, DCS_PSTOP_DEFAULT_MACHINE_ID);
  if (werr != ESP_OK) {
    ESP_LOGW(TAG, "persist pstop peer: %s", esp_err_to_name(werr));
  }
}

esp_err_t dcs_pstop_set_peer_slot(int slot, bool configured, uint32_t ip, uint16_t port, uint32_t machine_id)
{
  if ((slot < 0) || (slot >= DCS_PSTOP_MAX_MACHINES)) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Live update first (comparator picks it up next tick)... */
  atomic_store(&g_dcs_pstop_slot_id[slot], machine_id);
  atomic_store(&g_dcs_pstop_slot_ep[slot], pstop_ep_pack(configured, ip, port));
  if (slot == 0) {
    atomic_store(&g_dcs_pstop_peer_ip, configured ? ip : 0u);
    atomic_store(&g_dcs_pstop_peer_port, port);
  }
  /* ...then persist the whole table (read-modify-write of the blob). */
  dcs_pstop_peer_rec_t peers[DCS_PSTOP_MAX_MACHINES];
  dcs_nvs_read_pstop_peers(peers);
  peers[slot].configured = configured;
  peers[slot].ip = ip;
  peers[slot].port = port;
  peers[slot].machine_id = machine_id;
  return dcs_nvs_write_pstop_peers(peers);
}

void dcs_get_pstop_peer_slot(
  int slot, bool * configured, uint32_t * peer_ip, uint16_t * peer_port, uint32_t * machine_id)
{
  uint64_t ep = 0;
  uint32_t id = 0;
  if ((slot >= 0) && (slot < DCS_PSTOP_MAX_MACHINES)) {
    ep = atomic_load(&g_dcs_pstop_slot_ep[slot]);
    id = (uint32_t)atomic_load(&g_dcs_pstop_slot_id[slot]);
  }
  if (configured != NULL) {
    *configured = (ep & PSTOP_EP_CONFIGURED) != 0ULL;
  }
  if (peer_ip != NULL) {
    *peer_ip = (uint32_t)((ep >> 16) & 0xFFFFFFFFu);
  }
  if (peer_port != NULL) {
    *peer_port = (uint16_t)(ep & 0xFFFFu);
  }
  if (machine_id != NULL) {
    *machine_id = id;
  }
}

void dcs_publish_pstop_machine(
  int slot,
  uint32_t sent,
  uint32_t replies,
  uint32_t send_fail,
  uint32_t rebonds,
  uint32_t hb_ms,
  uint64_t last_reply_ms,
  uint32_t rtt_ms,
  uint8_t last_msg,
  uint8_t sess_state)
{
  if ((slot < 0) || (slot >= DCS_PSTOP_MAX_MACHINES)) {
    return;
  }
  atomic_store(&g_dcs_pstop_m_sent[slot], sent);
  atomic_store(&g_dcs_pstop_m_replies[slot], replies);
  atomic_store(&g_dcs_pstop_m_send_fail[slot], send_fail);
  atomic_store(&g_dcs_pstop_m_rebonds[slot], rebonds);
  atomic_store(&g_dcs_pstop_m_last_reply_ms[slot], last_reply_ms);
  atomic_store(&g_dcs_pstop_m_rtt_ms[slot], rtt_ms);
  atomic_store(&g_dcs_pstop_m_hb_ms[slot], hb_ms);
  atomic_store(&g_dcs_pstop_m_last_msg[slot], last_msg);
  atomic_store(&g_dcs_pstop_m_state[slot], sess_state);
}

void dcs_get_initial_pstop_peer(uint32_t * peer_ip, uint16_t * peer_port)
{
  if (peer_ip != NULL) {
    *peer_ip = atomic_load(&g_dcs_pstop_peer_ip);
  }
  if (peer_port != NULL) {
    *peer_port = (uint16_t)atomic_load(&g_dcs_pstop_peer_port);
  }
}

uint32_t dcs_get_vpn_ip(void)
{
  return (g_dcs.ml_handle != NULL) ? microlink_get_vpn_ip(g_dcs.ml_handle) : 0u;
}

void dcs_notify_priority_health(bool healthy)
{
  if (g_dcs.ml_handle != NULL) {
    microlink_notify_priority_health(g_dcs.ml_handle, healthy);
  }
}
