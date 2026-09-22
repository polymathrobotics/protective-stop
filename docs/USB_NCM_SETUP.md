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
- once pointed at the host (`POST /api/pstop_peer?ip=10.42.0.1&port=8890`),
  it bonds to a `machine_app_runner` there. A fresh unit has no peer.

The chip enumerates with USB **VID:PID `303a:4001`** in application mode
(`303a:1001` is ROM download mode — a different device, ignored by the naming
rule below).

The subnet matters only for the peer address you configure: a host setup that
hands out a *different* subnet (notably Windows ICS, see below) needs the
matching `POST /api/pstop_peer?ip=<host-ip>&port=8890`.

---

## Linux — verified

**Easiest: run the installer.** It auto-detects the host's network manager
(NetworkManager *or* systemd-networkd) and installs the matching variant,
plus the manager-agnostic interface-naming rule. It is idempotent and
migrates a host that ran the older single-unit layout:

```sh
host/setup/install.sh
```

Replug the unit(s) afterward so the `esp-pstop<N>` rename takes effect.

**Several units on one host work out of the box.** Each tether is named
`esp-pstop<N>` (N = the kernel's `usb<N>` index, lowest free), and all of
them are ports of one bridge, **`pstop-br` = `10.42.0.1/24`**, which is the
single host end of every tether (DHCP server + NAT). Because every unit
lands in `10.42.0.0/24`, the chip's factory-default machine peer
`10.42.0.1:8890` is right for all of them — no per-unit configuration, and
plug order does not matter. (Per-unit subnets were deliberately *not* used:
a unit's peer IP would then depend on which USB port enumerated first.)

The manual steps for each manager, if you prefer them:

### NetworkManager

**1. Give every pstop its own predictable interface name (`esp-pstop<N>`).**
Keyed on the USB VID:PID on the parent device, so it holds across units and
reboots, and sorts before `80-net-setup-link.rules`, whose `.link` naming
(Ubuntu's `73-usb-net-by-mac.link` → `enx<mac>`) only applies while `NAME`
is still empty:

```sh
sudo cp host/setup/79-esp-pstop.rules /etc/udev/rules.d/
sudo rm -f /etc/systemd/network/70-esp-pstop.link      # old single-name rule
sudo udevadm control --reload
```

**2. One shared-mode bridge, and one multi-connect port profile.** `shared`
mode is what makes NetworkManager assign `10.42.0.1`, start a dnsmasq DHCP
server, and NAT the segment outward; `multi-connect multiple` lets the port
profile be active on every `esp-pstop<N>` at once:

```sh
sudo nmcli con delete esp-pstop 2>/dev/null   # old single-unit profile, if any
sudo nmcli con add type bridge con-name pstop-br ifname pstop-br \
     bridge.stp no ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method ignore \
     connection.autoconnect yes connection.autoconnect-slaves 1
sudo nmcli con add type ethernet con-name pstop-port ifname '*' \
     match.interface-name 'esp-pstop*' master pstop-br slave-type bridge \
     connection.multi-connect multiple connection.autoconnect yes
```

`bridge.stp no` matters: with STP on, each newly attached port sits in
listening/learning for 30 s before it forwards, so a unit would wait that
long for DHCP on every replug. The explicit `ipv4.addresses` keeps the
bridge at `10.42.0.1` whatever other shared profiles exist (NetworkManager
otherwise hands shared connections `10.42.<n>.1` in activation order).

Replug the units. Within a few seconds each `esp-pstop<N>` joins the bridge,
the chip DHCPs `10.42.0.x`, and its active uplink switches to USB.

### systemd-networkd (no NetworkManager)

Hosts running systemd-networkd (common on HIL / fleet hosts) need the
`nmcli`-free equivalent — a NetworkManager "shared" profile has no effect
there. Install the `79-esp-pstop.rules` naming rule as above, then:

```sh
sudo cp host/setup/pstop-br.netdev host/setup/79-pstop-br.network \
        host/setup/80-esp-pstop.network /etc/systemd/network/
sudo cp host/setup/esp-pstop-flush-networkd /etc/networkd-dispatcher/routable.d/esp-pstop-flush
sudo cp host/setup/esp-pstop-flush-networkd /etc/networkd-dispatcher/degraded.d/esp-pstop-flush
sudo chmod +x /etc/networkd-dispatcher/routable.d/esp-pstop-flush \
              /etc/networkd-dispatcher/degraded.d/esp-pstop-flush
sudo systemctl enable --now networkd-dispatcher   # GOTCHA below
sudo networkctl reload                            # creates the bridge netdev
```

`pstop-br.netdev` creates the bridge (STP off), `79-pstop-br.network` gives
it `10.42.0.1/24`, runs the built-in DHCP server and NATs the segment out
(`ConfigureWithoutCarrier=yes`, so it is ready before any chip enumerates),
and `80-esp-pstop.network` enslaves every `esp-pstop*` to it. **Gotcha:** the
`networkd-dispatcher` package is often *installed but its service disabled* —
then the neighbor-cache flush hook never fires and a tether can need a manual
`ip neigh flush` or a power-cycle after a re-enumeration; the
`systemctl enable --now` above fixes it (or replace the dispatcher hook with a
udev-triggered oneshot to drop the dependency). The single-unit version of
this layout was verified on a systemd-networkd HIL host 2026-08-08: the
tether self-heals unattended across the chip's re-enumerations (interface,
neighbor table, DHCP lease, ping, and HTTP all recover).

**HIL hosts note:** the bench rig keeps two units on *separate* subnets on
purpose (`tools/hil/hil.toml`: soak unit `10.42.0.x`, test unit `10.43.0.x`)
so the rig can tell them apart by address. Do not run this installer there
without adapting the rig config — it would bridge both units into
`10.42.0.0/24`.

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
  `192.168.137.1` — **not** `10.42.0.1`. Either point the chip at that address
  (`POST /api/pstop_peer?ip=192.168.137.1&port=8890`) or change the ICS
  subnet to `10.42.0.0/24` via the registry
  (`HKLM\SYSTEM\CurrentControlSet\Services\SharedAccess\Parameters`,
  `ScopeAddress` / `ScopeAddressBackup`) and restart the service.
- **Static alternative:** instead of ICS, give the pstop adapter a static
  `10.42.0.1/24` and run any small DHCP server bound to it. The chip needs a
  DHCP lease; it will not self-assign.
- Reach the admin UI at the chip's leased address, or configure the machine
  runner to listen on the host's tether address.
- **Several units:** each appears as its own NCM adapter (per-unit USB serial
  and MAC), but ICS can share to only *one* adapter. Select all the pstop
  adapters in Network Connections → **Bridge Connections**, then share to the
  bridge — the same one-segment layout the Linux installer builds.

## macOS — notes (similar idea, not bench-verified here)

- **Driver:** macOS binds CDC-NCM natively; the pstop shows up as a network
  service (often "USB 10/100/1000 LAN" or similar) in *System Settings →
  Network*.
- **Serving the link:** *System Settings → General → Sharing → Internet
  Sharing*, share your internet connection **to** the pstop's USB network
  service. macOS Internet Sharing runs `bootpd` (DHCP) and NATs the link.
- **Subnet gotcha:** macOS Internet Sharing also defaults to a
  `192.168.x.0/24` range, not `10.42.0.1`. As on Windows, either point the
  chip's peer at the host's address with `POST /api/pstop_peer?...` or override the `bootpd` scope
  (`/etc/bootpd.plist`) to `10.42.0.0/24`.
- **Static alternative:** manually set the pstop network service to
  `10.42.0.1/255.255.255.0` and run a DHCP server (e.g. `dnsmasq` from
  Homebrew) bound to that interface. The chip still needs a lease.

---

## Verify (any host)

```sh
# host has the tether address (Linux: on the bridge, one port per unit)
ip addr show pstop-br              # expect 10.42.0.1/24
bridge link                        # expect one esp-pstop<N> per plugged unit
# each chip answers on its leased address (Linux shared mode → 10.42.0.x)
for CHIP in $(ip neigh show dev pstop-br | awk '/10\.42\.0\./{print $1}'); do
  echo "$CHIP: $(curl -s "http://$CHIP/state.json" | grep -o '"active_iface":[0-9]')"   # expect 2 = USB-NCM
done
```

On the chip, `state.json` reports `active_iface` (`2` = USB-NCM) and
`usbncm_en`/`usb_ip`. A healthy tether shows the active uplink on USB and,
if a `machine_app_runner` is listening at the host's tether address, a bonded
machine session.

If the link comes up but never bonds, check that the chip's configured peer
(`pstop_machines[0]` in `state.json`) matches the address the host actually
took on the tether, and that a machine process is listening there.
