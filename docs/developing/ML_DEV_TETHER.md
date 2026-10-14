# ml_dev_tether — USB-CDC-NCM tether for dev iteration

Opt-in microlink module. When the ESP32-S3 boots, brings up its USB-OTG
controller as an NCM (Network Control Model) Ethernet device and waits for a
DHCP lease from the connected host. If a lease arrives, that netif becomes
the default route — DERP/Tailscale traffic rides over USB to the host,
through the host's existing internet, and out. If no lease arrives within
the configured timeout, falls back to WiFi.

## Usage on the device side

```c
#include "ml_dev_tether.h"

void app_main(void) {
    ml_app_config_t cfg = ML_APP_CONFIG_DEFAULT();
    cfg.try_alt_network        = ml_dev_tether_try_start;
    cfg.alt_network_timeout_ms = 5000;
    ml_app_start(&cfg);
}
```

Also enable the NCM class in `sdkconfig.defaults`:

```
CONFIG_TINYUSB_NET_MODE_NCM=y
```

## Host-side setup (Linux, NetworkManager)

Easiest approach — let NetworkManager handle DHCP server + NAT
automatically. Plug the ESP32 in once, then:

```sh
# Find the new interface (look for the locally-administered MAC)
ip link show

# Configure NetworkManager "shared" connection on it (does DHCP + NAT)
nmcli connection add type ethernet ifname <usb-iface-name> \
    con-name esp-tether ipv4.method shared ipv6.method ignore
nmcli connection up esp-tether
```

Verify:

```sh
ip addr show dev <usb-iface-name>     # should see 10.42.0.1/24 or similar
ping <esp-ip>                          # should see <1 ms
```

## Host-side setup (Linux, manual)

If NetworkManager isn't your friend, dnsmasq + iptables:

```sh
sudo ip addr add 192.168.7.1/24 dev <usb-iface-name>
sudo ip link set <usb-iface-name> up
sudo dnsmasq --interface=<usb-iface-name> --bind-interfaces \
    --dhcp-range=192.168.7.2,192.168.7.10,1h --no-daemon &
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o <wan-iface> -j MASQUERADE
sudo iptables -A FORWARD -i <usb-iface-name> -j ACCEPT
sudo iptables -A FORWARD -o <usb-iface-name> -j ACCEPT
```

## Caveats

* **Single USB-C connector on XIAO ESP32-S3.** USB-OTG and USB-Serial-JTAG
  share D+/D-. Enabling TinyUSB takes the JTAG console with it — the device
  re-enumerates as an NCM device (no `/dev/ttyACM*` console). Recovery:
  hold BOOT + tap RESET to enter ROM bootloader (USB-Serial-JTAG is provided
  by ROM), then `idf.py flash` works again. OTA over WiFi or over the USB
  netif also works.
* **First-boot fallback latency.** On boot, `ml_dev_tether_try_start` waits
  up to `alt_network_timeout_ms` for a DHCP lease before falling back to
  WiFi. Keep this short (~5 s) for fast cold-boot when no host is
  attached.
* **No hot-plug.** Tether is one-shot at boot. If you unplug USB while
  tethered, the netif loses its lease but won't auto-fall-back to WiFi.
  Reboot to retry.
* **No PSRAM-less boards.** Tested on ESP32-S3. ESP32 classic has no
  USB-OTG.
