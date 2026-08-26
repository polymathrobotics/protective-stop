#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Install the host-side USB-NCM stability config for the pstop tether.
# Pure host config — no device reflash. Idempotent. Auto-detects the host's
# network manager and installs the matching variant:
#   - NetworkManager   -> shared-mode nmcli profile + NM dispatcher flush hook
#   - systemd-networkd -> .network (DHCPServer+NAT) + networkd-dispatcher flush
# The 70-esp-pstop.link interface-naming rule is manager-agnostic (always).
set -e
D="$(cd "$(dirname "$0")" && pwd)"

# --- manager-agnostic: stable interface name -------------------------------
sudo cp "$D/70-esp-pstop.link" /etc/systemd/network/70-esp-pstop.link
# Keep ModemManager off the pstop's serial ports so esptool can flash it
# (MM grabs the ttyACM on each download-mode re-enumeration otherwise).
sudo cp "$D/71-esp-pstop-mm-ignore.rules" /etc/udev/rules.d/71-esp-pstop-mm-ignore.rules
sudo udevadm control --reload

nm_active() { command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; }
networkd_active() { systemctl is-active --quiet systemd-networkd 2>/dev/null; }

if nm_active; then
  echo "== Detected NetworkManager — installing the NM variant =="
  sudo cp "$D/90-esp-pstop-flush" /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
  sudo chmod +x /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
  sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
  echo "dhcp-authoritative" | sudo tee /etc/NetworkManager/dnsmasq-shared.d/esp-pstop.conf >/dev/null
  # NM profile bound to the STABLE name, autoconnect, shared IPv4.
  if ! nmcli -g NAME connection show | grep -qx esp-pstop; then
    sudo nmcli connection add type ethernet con-name esp-pstop ifname esp-pstop0 \
         ipv4.method shared ipv6.method ignore connection.autoconnect yes
  else
    sudo nmcli connection modify esp-pstop connection.interface-name esp-pstop0 \
         ipv4.method shared ipv6.method ignore connection.autoconnect yes
  fi
  VARIANT="NetworkManager (shared profile + dispatcher flush)"

elif networkd_active; then
  echo "== Detected systemd-networkd — installing the networkd variant =="
  sudo cp "$D/80-esp-pstop.network" /etc/systemd/network/80-esp-pstop.network
  # Neighbor-cache flush hook in BOTH state dirs (fires on up + re-enumerate).
  for st in routable degraded; do
    sudo mkdir -p "/etc/networkd-dispatcher/$st.d"
    sudo cp "$D/esp-pstop-flush-networkd" "/etc/networkd-dispatcher/$st.d/esp-pstop-flush"
    sudo chmod +x "/etc/networkd-dispatcher/$st.d/esp-pstop-flush"
  done
  # GOTCHA: the package can be present but the service DISABLED -> the hook
  # never fires. Enable it (best-effort; warn if the package is missing).
  if systemctl list-unit-files 2>/dev/null | grep -q '^networkd-dispatcher'; then
    sudo systemctl enable --now networkd-dispatcher || true
  else
    echo "  WARN: networkd-dispatcher not installed — the neighbor-cache flush"
    echo "        hook will NOT run (one half of the 'needs a power cycle' symptom)."
    echo "        Install it (e.g. apt-get install networkd-dispatcher) and re-run."
  fi
  sudo networkctl reload 2>/dev/null || sudo systemctl restart systemd-networkd || true
  VARIANT="systemd-networkd (.network DHCPServer+NAT + networkd-dispatcher flush)"

else
  echo "ERROR: neither NetworkManager nor systemd-networkd is active — cannot"
  echo "install the tether's shared-link config automatically. The 70-*.link"
  echo "naming rule was installed; configure a shared/DHCP link on esp-pstop0"
  echo "manually (static 10.42.0.1/24 + a DHCP server + NAT). See docs/USB_NCM_SETUP.md."
  exit 1
fi

echo "Installed variant: $VARIANT"
echo "Renaming/config applies on the tether's next re-enumeration"
echo "(power-cycle or replug the chip). The old enx<mac> binding is left"
echo "untouched so the current connection is not disrupted."
