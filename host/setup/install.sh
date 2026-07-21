#!/bin/bash
# Install the host-side USB-NCM stability config for the pstop tether.
# Pure host config — no device reflash. Idempotent.
set -e
D="$(cd "$(dirname "$0")" && pwd)"
sudo cp "$D/70-esp-pstop.link" /etc/systemd/network/70-esp-pstop.link
sudo cp "$D/90-esp-pstop-flush" /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
sudo chmod +x /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
echo "dhcp-authoritative" | sudo tee /etc/NetworkManager/dnsmasq-shared.d/esp-pstop.conf >/dev/null
sudo udevadm control --reload

# NM profile bound to the STABLE name, autoconnect, shared IPv4.
if ! nmcli -g NAME connection show | grep -qx esp-pstop; then
  sudo nmcli connection add type ethernet con-name esp-pstop ifname esp-pstop0 \
       ipv4.method shared ipv6.method ignore connection.autoconnect yes
else
  sudo nmcli connection modify esp-pstop connection.interface-name esp-pstop0 \
       ipv4.method shared ipv6.method ignore connection.autoconnect yes
fi
echo "Installed. Renaming applies on the tether's next re-enumeration"
echo "(power-cycle or replug the chip). The old enx<mac> binding is left"
echo "untouched so the current connection is not disrupted."
