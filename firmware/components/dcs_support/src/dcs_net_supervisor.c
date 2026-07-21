/**
 * @file dcs_net_supervisor.c
 * @brief 1 Hz network-priority supervisor.
 *
 * Enforces the network preference order for the Waveshare W5500 board:
 *
 *     Ethernet (W5500)  >  USB-Ethernet (CDC-NCM)  >  WiFi STA  >  SoftAP
 *
 * Each second it finds the highest-priority interface that is currently UP
 * with a valid (non-zero) IPv4 address and pins it as the system default
 * route via esp_netif_set_default_netif(). This both (a) makes a higher-
 * priority link reclaim the default route as soon as it re-establishes
 * (e.g. an Ethernet cable plugged in at runtime) and (b) fails traffic down
 * to the next interface when a higher one drops.
 *
 * esp_netif's own route_prio-based selection already does most of this on
 * GOT_IP / LOST_IP events (Ethernet=128, USB-NCM=110, WiFi=100); the
 * supervisor is the authoritative, polling backstop and the source of the
 * active-interface telemetry shown in /state.json.
 *
 * If no STA-class interface is usable, the active interface is reported as
 * SoftAP (when ml_app's AP fallback is hosting a setup network) or NONE.
 */

#include "dcs_internal.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ml_app.h"

static const char *TAG = "dcs_netsup";

#define SUPERVISOR_PERIOD_MS  1000

/* Failover-ladder timing (supervisor ticks ≈ seconds). The supervisor drives
 * the whole Eth > USB > WiFi ladder, not just the route: it brings a lower tier
 * UP once the tiers above it have been unusable for *_AFTER_S. Ethernet is never
 * toggled here — its netif is always left running + requesting DHCP, so a lease
 * that appears minutes after boot promotes it automatically. USB comes up before
 * WiFi so a USB host is preferred; WiFi waits longer to give USB a chance to pull
 * a lease.
 *
 * Tier-DOWN policy differs by tier: WiFi (the big RAM consumer, ~40 KB) IS
 * auto-dropped once a higher tier has been stable for DROP_HOLD_S, reclaiming
 * RAM. USB-NCM is NEVER auto-dropped — runtime TinyUSB teardown is fragile
 * (leak + RX-vs-destroy race -> panic under a flapping link), so once up it
 * stays resident as a dormant backup and route arbitration simply demotes it.
 * Only WiFi (and only when WE enabled it) is auto-disabled, so a manual admin
 * enable is never revoked. */
#define USB_FAILOVER_AFTER_S    5u   /* Eth unusable this long  -> bring up USB-NCM */
#define WIFI_FAILOVER_AFTER_S  20u   /* Eth+USB unusable this long -> bring up WiFi */
#define FAILOVER_DROP_HOLD_S   15u   /* higher tier stable this long -> drop lower  */

/* A netif is a usable uplink only with a real (non-zero, non-link-local) lease.
 * 169.254.0.0/16 (0xFEA9 in the low 16 bits of the network-order u32) is an
 * AUTOIP self-assignment with no upstream — never route to it. */
static bool netif_usable(esp_netif_t *n) {
    if ((n == NULL) || (!esp_netif_is_netif_up(n))) {
        return false;
    }
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(n, &ip) != ESP_OK) {
        return false;
    }
    if (ip.ip.addr == 0u) {
        return false;
    }
    if ((ip.ip.addr & 0xFFFFu) == 0xFEA9u) {
        return false;
    }
    return true;
}

static esp_netif_t *netif_for(dcs_iface_t code) {
    return esp_netif_get_handle_from_ifkey(dcs_iface_ifkey(code));
}

static void supervisor_task(void *arg) {
    (void)arg;
    dcs_iface_t last_logged = DCS_IFACE_NONE;
    bool     wifi_auto = false;                     /* WiFi tier is up because WE enabled it */
    uint32_t eth_bad_s = 0;                         /* Eth-unusable streak (gates USB bring-up) */
    uint32_t high_bad_s = 0, high_ok_s = 0;        /* (Eth&&USB) down / (Eth||USB) up  */

    for (;;) {
        esp_netif_t *eth  = netif_for(DCS_IFACE_ETH);
        esp_netif_t *usb  = netif_for(DCS_IFACE_USB);
        esp_netif_t *wifi = netif_for(DCS_IFACE_WIFI);
        bool eth_ok  = netif_usable(eth);
        bool usb_ok  = netif_usable(usb);
        bool wifi_ok = netif_usable(wifi);

        /* --- Default route: highest-priority usable uplink. --- */
        esp_netif_t *best      = NULL;
        dcs_iface_t  best_code = DCS_IFACE_NONE;
        const char  *best_lbl  = "none";
        if (eth_ok)       { best = eth;  best_code = DCS_IFACE_ETH;  best_lbl = "ethernet"; }
        else if (usb_ok)  { best = usb;  best_code = DCS_IFACE_USB;  best_lbl = "usb-ncm";  }
        else if (wifi_ok) { best = wifi; best_code = DCS_IFACE_WIFI; best_lbl = "wifi-sta"; }
        else              { /* no usable uplink; reported below */ }

        if (best != NULL) {
            if (esp_netif_get_default_netif() != best) {
                (void)esp_netif_set_default_netif(best);
                ESP_LOGI(TAG, "default route -> %s", best_lbl);
            }
        } else if ((g_dcs.app != NULL) && ml_app_in_ap_fallback(g_dcs.app)) {
            best_code = DCS_IFACE_AP;        /* ml_app hosting the setup SoftAP */
            best_lbl  = "softap-setup";
        } else {
            /* no uplink and no SoftAP: stays DCS_IFACE_NONE */
        }

        /* --- Streaks driving the ladder. --- */
        if (eth_ok) { eth_bad_s = 0u; } else { eth_bad_s++; }
        bool high_ok = eth_ok || usb_ok;
        if (high_ok) { high_ok_s++; high_bad_s = 0u; } else { high_bad_s++; high_ok_s = 0u; }

        /* --- USB-NCM tier: bring up when Ethernet is gone, then LEAVE IT UP. ---
         * We deliberately never auto-drop USB-NCM. Runtime TinyUSB teardown
         * (ml_dev_tether_stop -> destroy netif -> next bring-up re-runs
         * tinyusb_net_init) leaked internal heap and raced on_usb_rx against
         * esp_netif_destroy; under a flapping Ethernet link that churn drove
         * heap_min into the low-20 KB range and produced a reset_reason=PANIC.
         * Once NCM is installed it costs little to keep resident, and the route
         * arbitration above already demotes it to a dormant backup the instant
         * Ethernet returns (Eth route_prio 128 > USB-NCM 110). Robustness over a
         * few reclaimed KB on a safety device. */
        if ((!eth_ok) && (eth_bad_s >= USB_FAILOVER_AFTER_S) && (!dcs_usb_is_enabled())) {
            ESP_LOGW(TAG, "no Ethernet for %us — bringing up USB-NCM (stays resident)",
                     (unsigned)eth_bad_s);
            (void)dcs_usb_set_enabled(true);
        }

        /* --- WiFi tier — only when dcs_wifi owns the WiFi lifecycle (an
         * alt-network won at boot). When ml_app owns WiFi (boot fell through to
         * it) ml_app also drives the SoftAP fallback, so we never touch it. --- */
        if (dcs_boot_alt_network_won()) {
            if ((!high_ok) && (high_bad_s >= WIFI_FAILOVER_AFTER_S) && (!dcs_wifi_is_enabled())) {
                ESP_LOGW(TAG, "no Eth/USB for %us — bringing up WiFi", (unsigned)high_bad_s);
                if (dcs_wifi_set_enabled(true) == ESP_OK) { wifi_auto = true; }
            } else if (high_ok && (high_ok_s >= FAILOVER_DROP_HOLD_S)
                       && wifi_auto && dcs_wifi_is_enabled()) {
                ESP_LOGW(TAG, "Eth/USB stable %us — dropping WiFi", (unsigned)high_ok_s);
                if (dcs_wifi_set_enabled(false) == ESP_OK) { wifi_auto = false; }
            } else {
                /* ladder idle this tick */
            }
        }

        if (best_code != last_logged) {
            ESP_LOGI(TAG, "active interface: %s", best_lbl);
            last_logged = best_code;
        }
        atomic_store(&g_dcs_active_iface, (int)best_code);

        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
    }
}

void dcs_net_supervisor_start(void) {
    atomic_store(&g_dcs_active_iface, (int)DCS_IFACE_NONE);
    /* 4608 B: the task now calls dcs_wifi_set_enabled() (esp_wifi_init/deinit,
     * esp_netif create/destroy) on auto-failover, which needs more stack than
     * the bare route-arbitration loop. */
    /* PSRAM stack: route supervisor is non-safety, does no flash/NVS. */
    (void)dcs_task_spawn_psram(supervisor_task, "net_sup", 4608, NULL, 4, tskNO_AFFINITY);
}
