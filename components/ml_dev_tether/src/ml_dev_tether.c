/**
 * @file ml_dev_tether.c
 * @brief USB-CDC-NCM tether — see ml_dev_tether.h for usage.
 *
 * Architecture, end-to-end:
 *   1. tinyusb_driver_install + tinyusb_net_init  -> USB enumerates as NCM
 *   2. Create an esp_netif backed by tinyusb_net (Ethernet-class lwIP netif,
 *      flags = DHCP_CLIENT | FLAG_AUTOUP, route_prio higher than WiFi).
 *   3. Hook IP_EVENT_GOT_IP on a binary semaphore.
 *   4. Wait timeout_ms for the lease.
 *   5. If lease arrived  -> set as default netif, return ESP_OK
 *      otherwise         -> tear down and return ESP_ERR_TIMEOUT.
 *
 * The whole thing is one-shot: there's no expectation of plug/unplug while
 * running. If you need that, add an IP_EVENT_LOST_IP handler that flips
 * default route back to WiFi.
 */

#include "ml_dev_tether.h"

#include <stdio.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "lwip/esp_netif_net_stack.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_net.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"

#include <string.h>

static const char *TAG = "ml_dev_tether";

static esp_netif_t       *s_netif         = NULL;
static SemaphoreHandle_t  s_got_ip_sem    = NULL;
static SemaphoreHandle_t  s_rx_lock        = NULL; /* serialises on_usb_rx vs stop()'s destroy */
static bool               s_tusb_installed = false;
static bool               s_net_inited     = false; /* tinyusb_net_init done (once) */
static uint8_t            s_unit_number    = 0;   /* 0 = derive from chip ID (1..99) */

/* Set the "PSTOPxx" unit number used in the USB product string. 0 (default) =
 * derive xx from the chip ID. Must be called before the first try_start, since
 * the descriptor is fixed at TinyUSB install (first bring-up). */
void ml_dev_tether_set_unit_number(uint8_t n) { s_unit_number = n; }

/* === USB <-> netif glue =========================================== */

static esp_err_t netif_transmit(void *h, void *buffer, size_t len) {
    /* Synchronous send so lwIP can free the pbuf immediately. */
    return tinyusb_net_send_sync(buffer, len, NULL, pdMS_TO_TICKS(100));
}

static void netif_l2_free(void *h, void *buffer) {
    free(buffer);
}

static esp_err_t on_usb_rx(void *buffer, uint16_t len, void *ctx) {
    /* Serialise against ml_dev_tether_stop()'s esp_netif_destroy(). Without the
     * lock this task can read a non-NULL s_netif, get preempted while stop()
     * frees it, then pass the freed netif to esp_netif_receive() (use-after-
     * free). Holding s_rx_lock across the receive makes stop() wait for an
     * in-flight RX to finish, and any RX that starts after stop() sees a NULL
     * s_netif and bails. Runs on the TinyUSB task (not an ISR), so blocking is
     * legal. */
    if (s_rx_lock) xSemaphoreTake(s_rx_lock, portMAX_DELAY);
    esp_netif_t *n = s_netif;
    esp_err_t r = ESP_OK;
    if (n) {
        /* lwIP takes ownership; we hand it its own buffer so tinyusb can
         * recycle the original. */
        void *copy = malloc(len);
        if (!copy) {
            r = ESP_ERR_NO_MEM;
        } else {
            memcpy(copy, buffer, len);
            r = esp_netif_receive(n, copy, len, NULL);
        }
    }
    if (s_rx_lock) xSemaphoreGive(s_rx_lock);
    return r;
}

/* === DHCP lease signal ============================================ */

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
    if (evt && evt->esp_netif == s_netif) {
        ESP_LOGI(TAG, "USB netif got IP " IPSTR " gw " IPSTR,
                 IP2STR(&evt->ip_info.ip), IP2STR(&evt->ip_info.gw));
        if (s_got_ip_sem) xSemaphoreGive(s_got_ip_sem);
    }
}

/* === Public API =================================================== */

esp_err_t ml_dev_tether_try_start(uint32_t timeout_ms) {
    if (s_netif) {
        ESP_LOGW(TAG, "Already started");
        return ESP_OK;
    }

    /* We're called BEFORE wifi_init, so esp_netif_init() and the default
     * event loop may not exist yet. Both are idempotent — INVALID_STATE
     * means "already created", which is fine. Anything else is fatal. */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    s_got_ip_sem = xSemaphoreCreateBinary();
    if (!s_got_ip_sem) return ESP_ERR_NO_MEM;
    /* Resident mutex guarding on_usb_rx vs stop()'s destroy. Created once and
     * never deleted — it must exist before tinyusb_net_init() registers the RX
     * callback below, and must outlive every stop()/start() cycle. */
    if (!s_rx_lock) {
        s_rx_lock = xSemaphoreCreateMutex();
        if (!s_rx_lock) {
            vSemaphoreDelete(s_got_ip_sem);
            s_got_ip_sem = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_register: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_got_ip_sem);
        s_got_ip_sem = NULL;
        return ESP_FAIL;
    }

    /* --- TinyUSB up --- */
    if (!s_tusb_installed) {
        const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
        err = tinyusb_driver_install(&tusb_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "tinyusb_driver_install: %s", esp_err_to_name(err));
            goto fail;
        }
        s_tusb_installed = true;

        /* Per-unit USB product string "PSTOPxx" so several tethered units are
         * distinguishable on one host. xx is the admin-set unit number (1..99)
         * or, when unset, derived from the chip ID. Overrides the Kconfig product
         * string at STRID_PRODUCT (index 2) via the same runtime-string mechanism
         * tinyusb_net uses for the MAC string; the buffer is static so the
         * descriptor layer can keep the pointer. */
        extern void tinyusb_descriptors_set_string(const char *str, int str_idx);
        static char s_usb_product[12];
        unsigned xx;
        if (s_unit_number >= 1 && s_unit_number <= 99) {
            xx = s_unit_number;
        } else {
            uint8_t idmac[6];
            esp_read_mac(idmac, ESP_MAC_WIFI_STA);
            xx = (((unsigned)idmac[4] << 8 | idmac[5]) % 99u) + 1u;   /* 01..99 */
        }
        snprintf(s_usb_product, sizeof(s_usb_product), "PSTOP%02u", xx);
        tinyusb_descriptors_set_string(s_usb_product, 2);
        ESP_LOGW(TAG, "USB product string -> %s (unit_number=%u)", s_usb_product,
                 (unsigned)s_unit_number);

        /* Per-unit USB iSerial (descriptor string index 3) derived from the
         * eFuse MAC, replacing the shared Kconfig constant "123456". Lets a
         * host tell two connected units apart (udev ID_SERIAL_SHORT) and pin
         * per-unit interface names — see docs/USB_NCM_STABILITY.md. Stable
         * across reboots (eFuse MAC is fixed), unique per unit. */
        static char s_usb_serial[13];
        uint8_t idserial[6];
        esp_read_mac(idserial, ESP_MAC_WIFI_STA);
        snprintf(s_usb_serial, sizeof(s_usb_serial), "%02X%02X%02X%02X%02X%02X",
                 idserial[0], idserial[1], idserial[2],
                 idserial[3], idserial[4], idserial[5]);
        tinyusb_descriptors_set_string(s_usb_serial, 3);
        ESP_LOGW(TAG, "USB iSerial -> %s", s_usb_serial);

        /* CDC ACM init only — no tinyusb_console_init. Redirecting ESP_LOG
         * to USB CDC IS reproducible blocking: early-boot ESP_LOG writes
         * (microlink/nvs/etc.) happen before host opens the CDC line, the
         * TX path waits for host to drain, and main_task deadlocks before
         * DHCP can even start. We keep the CDC interface visible to host
         * for app-level uses; logs stay where IDF puts them by default. */
        const tinyusb_config_cdcacm_t cdc_cfg = {
            .cdc_port = TINYUSB_CDC_ACM_0,
        };
        err = tinyusb_cdcacm_init(&cdc_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "tinyusb_cdcacm_init: %s", esp_err_to_name(err));
        }
    }

    /* Use a locally-administered MAC (b0 of first octet = 1) derived from the
     * device's own. Host sees this as a distinct interface. */
    tinyusb_net_config_t net_cfg = {
        .on_recv_callback = on_usb_rx,
    };
    esp_read_mac(net_cfg.mac_addr, ESP_MAC_WIFI_STA);
    net_cfg.mac_addr[0] |= 0x02;   /* locally administered */
    net_cfg.mac_addr[5] ^= 0x01;   /* differ from WiFi MAC by 1 */

    /* tinyusb_net_init registers the NCM RX glue + descriptor MAC string and has
     * no public deinit; ml_dev_tether_stop() leaves TinyUSB installed. On a
     * stop->start cycle (e.g. a manual admin USB re-toggle) calling it again
     * re-registers the RX path and leaks — so run it exactly once per process.
     * The on_recv callback from the first init indirects through the file-static
     * s_netif, which we refresh just below, so RX keeps working after a restart. */
    if (!s_net_inited) {
        err = tinyusb_net_init(&net_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "tinyusb_net_init: %s", esp_err_to_name(err));
            goto fail;
        }
        s_net_inited = true;
    }

    /* --- esp_netif (Ethernet-class, DHCP client) --- */
    esp_netif_inherent_config_t base_cfg = {
        .flags      = (esp_netif_flags_t)(ESP_NETIF_DHCP_CLIENT | ESP_NETIF_FLAG_AUTOUP),
        .if_key     = "USB_NCM",
        .if_desc    = "usb_ncm",
        .route_prio = 110,  /* above WiFi STA (IDF default 100), below wired
                             * Ethernet (128) — so the order is Eth > USB > WiFi.
                             * (The old "64 > WiFi 50" comment was stale: WiFi
                             * STA's real default route_prio is 100.) */
        .get_ip_event = IP_EVENT_ETH_GOT_IP,
        .lost_ip_event = IP_EVENT_ETH_LOST_IP,
    };
    esp_netif_driver_ifconfig_t driver_cfg = {
        .handle = (void *)1,
        .transmit = netif_transmit,
        .driver_free_rx_buffer = netif_l2_free,
    };
    struct esp_netif_netstack_config lwip_cfg = {
        .lwip = {
            .init_fn  = ethernetif_init,
            .input_fn = ethernetif_input,
        }
    };
    esp_netif_config_t cfg = {
        .base = &base_cfg,
        .driver = &driver_cfg,
        .stack = &lwip_cfg,
    };
    s_netif = esp_netif_new(&cfg);
    if (!s_netif) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        goto fail;
    }

    /* Use the SAME MAC for the lwIP netif as the NCM endpoint. The IDF
     * sta2eth example uses different MACs because it bridges to a separate
     * DHCP server; in our DHCP-CLIENT topology there's a single logical
     * endpoint and ARP/DHCP must resolve to the same MAC the host sees. */
    esp_netif_set_mac(s_netif, net_cfg.mac_addr);

    /* Bring up the netif and also signal "link connected" — without this,
     * the Ethernet-class netif stays in admin-up/link-down state and the
     * DHCP client never sends DISCOVER (it's waiting for a phy link event
     * that's never coming from our USB driver). */
    esp_netif_action_start(s_netif, 0, 0, 0);
    esp_netif_action_connected(s_netif, 0, 0, 0);

    /* --- Wait for DHCP lease --- */
    ESP_LOGI(TAG, "Waiting up to %lu ms for DHCP from host...", (unsigned long)timeout_ms);
    if (xSemaphoreTake(s_got_ip_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "No DHCP lease in time; host not present or not configured. "
                       "Leaving USB stack live (CDC console + waiting NCM netif).");
        /* Intentionally DO NOT destroy the netif or stop tinyusb here.
         * Destroying mid-enumeration was observed to cause USB bus disconnects
         * that the host couldn't recover from until physical unplug. Better
         * to leave NCM up as a non-default interface — if the host configures
         * DHCP later it'll still get an IP, and the CDC console stays useful
         * regardless. ml_app falls back to WiFi as planned. */
        return ESP_ERR_TIMEOUT;
    }

    /* Pin as default netif so microlink's outbound TCP/UDP rides USB. */
    esp_err_t set_err = esp_netif_set_default_netif(s_netif);
    if (set_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_netif_set_default_netif: %s (continuing, microlink may "
                       "use WiFi as default)", esp_err_to_name(set_err));
    } else {
        ESP_LOGI(TAG, "USB tether is default route — microlink will use host's internet");
    }
    return ESP_OK;

fail:
    ml_dev_tether_stop();
    return ESP_FAIL;
}

void ml_dev_tether_stop(void) {
    /* NULL s_netif UNDER s_rx_lock, then destroy outside it. The lock makes this
     * mutually exclusive with on_usb_rx(): an RX already running holds the lock,
     * so we block until it finishes before freeing the netif; an RX starting
     * after this sees s_netif==NULL and bails. This closes the use-after-free
     * window that NULL-first ordering alone only narrowed. */
    if (s_rx_lock) xSemaphoreTake(s_rx_lock, portMAX_DELAY);
    esp_netif_t *n = s_netif;
    s_netif = NULL;
    if (s_rx_lock) xSemaphoreGive(s_rx_lock);
    if (n) {
        esp_netif_action_stop(n, 0, 0, 0);
        esp_netif_destroy(n);
    }
    if (s_got_ip_sem) {
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip);
        vSemaphoreDelete(s_got_ip_sem);
        s_got_ip_sem = NULL;
    }
    /* We intentionally leave tinyusb installed — uninstalling is brittle and
     * we may want to retry later without re-enumeration. */
}
