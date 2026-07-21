#include "dcs_identity.h"

#include <stdio.h>
#include "esp_mac.h"

/* Cached after first derivation. device_id is always non-zero (top byte 0x01),
 * so it doubles as the "computed yet?" flag. */
static uint32_t s_device_id = 0;
static char     s_hostname[16] = {0};   /* "pstop-" + 8 hex + NUL = 15 */

static void ensure_derived(void)
{
    if (s_device_id != 0) {
        return;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t unit24 = ((uint32_t)mac[3] << 16) |
                      ((uint32_t)mac[4] << 8)  |
                      (uint32_t)mac[5];
    s_device_id = 0x01000000u | unit24;   /* 0x01 = pstop remote type tag */
    snprintf(s_hostname, sizeof(s_hostname), "pstop-%08x", (unsigned)s_device_id);
}

uint32_t dcs_identity_device_id(void)
{
    ensure_derived();
    return s_device_id;
}

const char *dcs_identity_hostname(void)
{
    ensure_derived();
    return s_hostname;
}
