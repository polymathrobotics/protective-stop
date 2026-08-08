<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# USB-NCM tether — host setup

This is the one-time host setup for operating a pstop remote over its USB
cable (the "USB-NCM tether"). USB-NCM is the recommended transport for
**bench work and field service** — no LAN, no WiFi credentials, no Tailscale
round-trip — and it is the third automatic uplink after Ethernet and WiFi.
Firmware stability of the tether itself is characterized separately in
[`USB_NCM_STABILITY.md`](USB_NCM_STABILITY.md); this guide is only about
preparing the host.

## Why a host setup is needed at all

Plugging a pstop into a host does **not** give you a working link out of the
box, and the failure mode is misleading: the host's new "wired" connection
sits in *connecting…* forever while the unit silently falls back to its
provisioned WiFi.

By design the chip's USB-NCM interface runs **no DHCP server**. The chip
presents itself as a standard USB CDC-NCM network adapter and expects the
**host** to own the link — assign itself an address, serve DHCP to the chip,
and NAT the chip's traffic onward. Once the host does that:

- the host takes **`10.42.0.1`**,
- the chip DHCPs an address in **`10.42.0.0/24`** (typically `10.42.0.x`),
- the chip switches its active uplink to USB, and
- because the chip's factory-default machine peer is **`10.42.0.1:8890`**, it
  bonds to a `machine_app_runner` on this host with zero further config.

The chip enumerates with USB **VID:PID `303a:4001`** in application mode
(`303a:1001` is ROM download mode — a different device, ignored by the naming
rule below).

The subnet matters: the chip's baked-in default peer is `10.42.0.1`. Any host
setup that hands out a *different* subnet (notably Windows ICS, see below)
will bring the link up but the default machine bond will not form until you
either match the subnet or repoint the chip with
`POST /api/pstop_peer?ip=<host-ip>&port=8890`.

---

## Linux — verified

**Easiest: run the installer.** It auto-detects the host's network manager
(NetworkManager *or* systemd-networkd) and installs the matching variant,
plus the manager-agnostic interface-naming rule:

```sh
host/setup/install.sh
```

Replug the unit afterward so the `esp-pstop0` rename takes effect. The manual
steps for each manager, if you prefer them:

### NetworkManager

**1. Pin every pstop to one predictable interface name (`esp-pstop0`).**
Keyed on the USB VID:PID so it holds across units and across reboots, and
sorts before the default `73-usb-net-by-mac.link` (first match wins), so the
interface is never the per-unit `enx<mac>` name:

```sh
sudo cp host/setup/70-esp-pstop.link /etc/systemd/network/
sudo udevadm control --reload
```

**2. Bind a shared-mode NetworkManager profile to that name.** `shared` mode
is what makes NetworkManager assign `10.42.0.1`, start a dnsmasq DHCP server,
and NAT the link outward:

```sh
sudo nmcli con add type ethernet ifname esp-pstop0 con-name esp-pstop \
     ipv4.method shared connection.autoconnect yes
```

Replug the unit. Within a few seconds the host takes `10.42.0.1`, the chip
DHCPs `10.42.0.x`, and its active uplink switches to USB.

### systemd-networkd (no NetworkManager)

Hosts running systemd-networkd (common on HIL / fleet hosts) need the
`nmcli`-free equivalent — a NetworkManager "shared" profile has no effect
there. Install the `70-esp-pstop.link` naming rule as above, then:

```sh
sudo cp host/setup/80-esp-pstop.network /etc/systemd/network/
sudo cp host/setup/esp-pstop-flush-networkd /etc/networkd-dispatcher/routable.d/esp-pstop-flush
sudo cp host/setup/esp-pstop-flush-networkd /etc/networkd-dispatcher/degraded.d/esp-pstop-flush
sudo chmod +x /etc/networkd-dispatcher/routable.d/esp-pstop-flush \
              /etc/networkd-dispatcher/degraded.d/esp-pstop-flush
sudo systemctl enable --now networkd-dispatcher   # GOTCHA below
sudo networkctl reload
```

`80-esp-pstop.network` gives the host `10.42.0.1/24`, runs the built-in DHCP
server, and NATs the link out (`ConfigureWithoutCarrier=yes`, so it is ready
before the chip even enumerates). **Gotcha:** the `networkd-dispatcher`
package is often *installed but its service disabled* — then the neighbor-cache
flush hook never fires and the tether can need a manual `ip neigh flush` or a
power-cycle after a re-enumeration; the `systemctl enable --now` above fixes
it (or replace the dispatcher hook with a udev-triggered oneshot to drop the
dependency). Verified on a systemd-networkd HIL host 2026-08-08: the tether
self-heals unattended across the chip's re-enumerations (interface, neighbor
table, DHCP lease, ping, and HTTP all recover).

**Caveat (both managers):** only the *first* unit plugged in gets
`esp-pstop0`; simultaneous extra units fall back to `enx<mac>` names and each
needs its own profile/match.

Background on why the naming rule (not just a MAC-keyed profile) is required
is in [`USB_NCM_STABILITY.md`](USB_NCM_STABILITY.md).

---

## Windows — notes (similar idea, not bench-verified here)

The concept is identical — the host must own the link and serve DHCP — but
the mechanism differs.

- **Driver:** Windows 10 (1809+) and Windows 11 bind the chip to the built-in
  USB NCM class driver automatically; it appears as an ordinary Ethernet
  adapter. No vendor driver is needed. On older Windows a CDC-NCM/RNDIS
  driver may have to be supplied.
- **Serving the link:** enable **Internet Connection Sharing (ICS)** on an
  Internet-facing adapter and share it *to* the pstop adapter. ICS then runs
  a DHCP server and NATs the link.
- **Subnet gotcha:** ICS hands out **`192.168.137.0/24`** with the host at
  `192.168.137.1` — **not** `10.42.0.1`. The chip will get an address and the
  link will be up, but its factory-default machine peer (`10.42.0.1:8890`)
  will not match. Either repoint the chip
  (`POST /api/pstop_peer?ip=192.168.137.1&port=8890`) or change the ICS
  subnet to `10.42.0.0/24` via the registry
  (`HKLM\SYSTEM\CurrentControlSet\Services\SharedAccess\Parameters`,
  `ScopeAddress` / `ScopeAddressBackup`) and restart the service.
- **Static alternative:** instead of ICS, give the pstop adapter a static
  `10.42.0.1/24` and run any small DHCP server bound to it. The chip needs a
  DHCP lease; it will not self-assign.
- Reach the admin UI at the chip's leased address, or configure the machine
  runner to listen on the host's tether address.

## macOS — notes (similar idea, not bench-verified here)

- **Driver:** macOS binds CDC-NCM natively; the pstop shows up as a network
  service (often "USB 10/100/1000 LAN" or similar) in *System Settings →
  Network*.
- **Serving the link:** *System Settings → General → Sharing → Internet
  Sharing*, share your internet connection **to** the pstop's USB network
  service. macOS Internet Sharing runs `bootpd` (DHCP) and NATs the link.
- **Subnet gotcha:** macOS Internet Sharing also defaults to a
  `192.168.x.0/24` range, not `10.42.0.1`. As on Windows, either repoint the
  chip's peer with `POST /api/pstop_peer?...` or override the `bootpd` scope
  (`/etc/bootpd.plist`) to `10.42.0.0/24`.
- **Static alternative:** manually set the pstop network service to
  `10.42.0.1/255.255.255.0` and run a DHCP server (e.g. `dnsmasq` from
  Homebrew) bound to that interface. The chip still needs a lease.

---

## Verify (any host)

```sh
# host has the tether address
ip addr show esp-pstop0            # Linux: expect 10.42.0.1/24
# chip answers on its leased address (Linux shared mode → 10.42.0.x)
curl -s http://10.42.0.1/state.json | grep -o '"active_iface":[0-9]'   # from the chip's side use its IP
```

On the chip, `state.json` reports `active_iface` (`2` = USB-NCM) and
`usbncm_en`/`usb_ip`. A healthy tether shows the active uplink on USB and,
if a `machine_app_runner` is listening at the host's tether address, a bonded
machine session.

If the link comes up but never bonds, it is almost always the **subnet
gotcha** above: the host handed out a range other than `10.42.0.0/24` while
the chip is still pointed at its `10.42.0.1:8890` default.
