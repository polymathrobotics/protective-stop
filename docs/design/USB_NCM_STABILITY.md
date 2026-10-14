# Stable, predictable USB-NCM tether on Ubuntu (2026-07-21)

## Symptom

After every chip reboot/OTA/power-cycle, the host's USB-NCM tether
interface came back and the NetworkManager profile did **not** auto-attach
— a human had to `nmcli connection modify … interface-name enx<new>` and
down/up it, and sometimes power-cycle before the chip answered HTTP.

## Root cause

The chip's host-facing NCM MAC is **already stable** (eFuse-derived —
`components/ml_dev_tether/src/ml_dev_tether.c:197`), proven by the kernel
log re-registering the same MAC across every reboot. The churn is the
**host naming policy**: Ubuntu's `/usr/lib/systemd/network/73-usb-net-by-mac.link`
names every USB net device `enx<mac>`, which differs per physical unit and
is what the NM profile brittlely bound to. So the profile broke on any
MAC-affecting change and never matched a *different* unit.

## Fix (host-only, no reflash) — installed & validated

`host/setup/` in this repo, applied by `host/setup/install.sh`:

1. **`70-esp-pstop.link`** → `/etc/systemd/network/`: renames the tether to
   the fixed `esp-pstop0` by matching VID:PID `303a:4001` only (ignores
   bootloader mode `303a:1001`; applies to any pstop unit regardless of
   MAC). Sorts before the default `73-` policy.
2. **NM profile `esp-pstop`** bound to `esp-pstop0`, `autoconnect yes`,
   `ipv4.method shared` — auto-attaches whenever `esp-pstop0` appears.
3. **`90-esp-pstop-flush`** dispatcher: `ip neigh flush` on link-up, clearing
   the stale-ARP half of the "needs a power cycle" symptom.
4. `dhcp-authoritative` in the shared dnsmasq for faster re-lease (the MAC
   is stable so the chip gets the same IP deterministically — DHCP is
   effectively static already).

**Validated 2026-07-21:** after a power-cycle with **zero** manual steps,
the tether came up as `esp-pstop0`, the `esp-pstop` profile auto-activated,
and the chip was reachable at `10.42.0.138`. The manual rebind is gone.

To reproduce on a fresh Ubuntu host: `./host/setup/install.sh` then
power-cycle/replug the chip once.

## Firmware follow-ups — DONE & validated (2026-07-21)

- **Per-unit iSerial** (`tether: per-unit USB iSerial`, commit 80a554a):
  descriptor string index 3 is now set at runtime from the eFuse MAC
  (`ml_dev_tether.c`), replacing the shared `123456`. Verified: sysfs
  `serial: 3C0F02D7791C` — unique per unit, stable across reboots. Hosts
  can now distinguish multiple units and pin per-unit names via
  `[Match] Property=ID_SERIAL_SHORT=…`.
- **NCM TX-stall hardening** (`tether: raise NCM NTB buffers`, commit
  a3132f6): `CONFIG_TINYUSB_NCM_OUT_NTB_BUFFS_COUNT` (host→device
  reception, the watchdog direction) and `_IN_` raised 2→4. The device→host
  `netif_transmit` drop-on-timeout was left as-is deliberately —
  fail-fast is correct there; blocking would stall the TCPIP thread that
  also carries the pstop RX. Validated: **3/3 power cycles recovered
  hands-free** (auto-reattach as `esp-pstop0`, no `nmcli` rebind) with
  **0 `NETDEV WATCHDOG` timeouts** and the chip back on the 128-peer
  tailnet with a clean bond each time. (The watchdog was intermittent
  before, so 3 clean cycles is strong evidence rather than absolute proof;
  the bigger NTB headroom is the correct mitigation regardless.)

Net result: chip reboot / OTA / power-cycle → host tether auto-reattaches
at `esp-pstop0` / `10.42.0.138` with no human step, no power-cycle-stuck
states. The manual recovery dance that ran through the whole prior
session is eliminated end to end (host `.link`+dispatcher and firmware
buffers together).
