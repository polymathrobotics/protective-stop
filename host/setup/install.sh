#!/bin/bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
# Install the host-side USB-NCM config for the pstop tether -- any number of
# units on one host (tested layout: up to 4). Pure host config -- no device
# reflash. Idempotent; safe to re-run to migrate from the old single-unit layout.
#
# Layout (both managers):
#   79-esp-pstop.rules  every 303a:4001 tether is named esp-pstop<N> (kernel
#                       usb<N> index, lowest free) -- distinct per unit.
#   pstop-br            one bridge = the host end of ALL tethers: 10.42.0.1/24,
#                       DHCP server, NAT. Every esp-pstop<N> is a bridge port.
# Because every unit lands in 10.42.0.0/24, the chip's factory-default machine
# peer (10.42.0.1:8890) is right for all of them with no per-unit config, and
# plug order does not matter. (Per-unit subnets were rejected for exactly that
# reason: the unit's peer IP would depend on which USB port came up first.)
#
#   NetworkManager   -> bridge profile (shared) + multi-connect port profile
#                       + NM dispatcher neighbor flush
#   systemd-networkd -> .netdev bridge + .network (DHCPServer+NAT) + port
#                       .network + networkd-dispatcher neighbor flush
set -e
D="$(cd "$(dirname "$0")" && pwd)"

# --- manager-agnostic: per-unit interface names ------------------------------
sudo cp "$D/79-esp-pstop.rules" /etc/udev/rules.d/79-esp-pstop.rules
# Superseded by the udev rule (it pinned ONE fixed name to the VID:PID).
sudo rm -f /etc/systemd/network/70-esp-pstop.link
sudo udevadm control --reload

nm_active() { command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; }
networkd_active() { systemctl is-active --quiet systemd-networkd 2>/dev/null; }
nm_has() { nmcli -g NAME connection show | grep -qx -- "$1"; }

if nm_active; then
  echo "== Detected NetworkManager — installing the NM variant =="
  sudo cp "$D/90-esp-pstop-flush" /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
  sudo chmod +x /etc/NetworkManager/dispatcher.d/90-esp-pstop-flush
  sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
  echo "dhcp-authoritative" | sudo tee /etc/NetworkManager/dnsmasq-shared.d/esp-pstop.conf >/dev/null

  # Old single-unit layout: a profile pinned to esp-pstop0. It would fight the
  # port profile for that device -- retire it.
  if nm_has esp-pstop; then
    echo "  retiring the old single-unit profile 'esp-pstop' (superseded by pstop-br/pstop-port)"
    sudo nmcli connection delete esp-pstop
  fi

  # Bridge master: the host end of every tether. Explicit address so it stays
  # 10.42.0.1 whatever other shared profiles exist (NM otherwise hands shared
  # connections 10.42.<n>.1 in activation order). STP off: a bridge with STP
  # holds each new port in listening/learning for 2x forward-delay (30 s) before
  # it forwards -- the unit would sit without DHCP for that long every replug.
  BR_OPTS=(bridge.stp no ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method ignore
    connection.autoconnect yes connection.autoconnect-slaves 1)
  if ! nm_has pstop-br; then
    sudo nmcli connection add type bridge con-name pstop-br ifname pstop-br "${BR_OPTS[@]}"
  else
    sudo nmcli connection modify pstop-br "${BR_OPTS[@]}"
  fi

  # Port profile: ONE profile, active on every esp-pstop<N> at the same time
  # (multi-connect), matched by name glob -- so a 2nd/3rd/4th unit needs nothing.
  PORT_OPTS=(match.interface-name 'esp-pstop*' master pstop-br slave-type bridge
    connection.multi-connect multiple connection.autoconnect yes)
  if ! nm_has pstop-port; then
    sudo nmcli connection add type ethernet con-name pstop-port ifname '*' "${PORT_OPTS[@]}"
  else
    sudo nmcli connection modify pstop-port "${PORT_OPTS[@]}"
  fi

  # Hand-made profiles from the single-unit days (esp-pstop-<x>, usb0, enx...)
  # would compete with pstop-port for a device. Not ours to delete -- list them.
  while IFS= read -r name; do
    case "$name" in pstop-br|pstop-port|"") continue ;; esac
    ifn="$(nmcli -g connection.interface-name connection show "$name" 2>/dev/null || true)"
    case "$ifn" in
      esp-pstop*|usb[0-9]*|enx*)
        echo "  NOTE: profile '$name' is bound to '$ifn' and may steal that unit from pstop-port;"
        echo "        remove it if it was a workaround: nmcli connection delete '$name'" ;;
    esac
  done < <(nmcli -g NAME connection show)
  VARIANT="NetworkManager (pstop-br shared bridge + multi-connect port profile + dispatcher flush)"

elif networkd_active; then
  echo "== Detected systemd-networkd — installing the networkd variant =="
  sudo cp "$D/pstop-br.netdev" /etc/systemd/network/pstop-br.netdev
  sudo cp "$D/79-pstop-br.network" /etc/systemd/network/79-pstop-br.network
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
  # reload creates new netdevs (the bridge) and re-reads .network files.
  sudo networkctl reload 2>/dev/null || sudo systemctl restart systemd-networkd || true
  VARIANT="systemd-networkd (pstop-br .netdev + DHCPServer/NAT .network + port .network + networkd-dispatcher flush)"

else
  echo "ERROR: neither NetworkManager nor systemd-networkd is active — cannot"
  echo "install the tether's shared-link config automatically. The 79-*.rules"
  echo "naming rule was installed (units appear as esp-pstop<N>); bridge them"
  echo "manually into one interface with 10.42.0.1/24 + a DHCP server + NAT."
  echo "See docs/USB_NCM_SETUP.md."
  exit 1
fi

echo "Installed variant: $VARIANT"
echo "Naming applies on each tether's next re-enumeration (power-cycle or"
echo "replug the units). Verify: ip addr show pstop-br   -> 10.42.0.1/24"
echo "                          bridge link              -> one esp-pstop<N> per unit"
