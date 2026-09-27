<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# Security profiles

A remote keeps its Tailscale identity in flash. On an unprotected ESP32-S3,
anyone with the unit and a USB cable can copy it out in seconds and join your
tailnet as the remote. The build profile decides how hard that is:

| Profile | Turns on | Use for |
|---|---|---|
| **`secure-fe`** (default) | Secure Boot v2, flash encryption (release), NVS encryption, Secure Download Mode, JTAG off | Every deployed unit |
| `secure` | The same without flash encryption | Owners who cannot keep a flash-encryption secret |
| `dev` | Nothing; no eFuses burned | Development and the quickstart |

- **Secure Boot v2**: the chip runs only a bootloader and app signed with your
  key, so a thief cannot flash a program that dumps the keys.
- **NVS encryption**: the identity is encrypted with a key derived from an
  eFuse key that software cannot read.
- **Secure Download Mode**: USB can still write images, but reads nothing back.
- **Flash encryption**: the rest of the flash is encrypted with a per-unit key,
  so a USB write needs that unit's key (no downgrade, no edited partition table)
  and coredumps stay unreadable on the chip.

## Building

```sh
cd firmware                        # or machn
idf.py build                       # secure-fe
idf.py -DPSTOP_PROFILE=dev build
```

The profile is fixed when `sdkconfig` is generated. Build another profile in its
own directory: `idf.py -B build-dev -DSDKCONFIG=build-dev/sdkconfig -DPSTOP_PROFILE=dev build`.

The secure overlays in [`firmware/profiles/`](../firmware/profiles/):

- build **unsigned** images, so CI, releases and adopters need no keys and each
  owner signs with their own;
- set the bootloader's RTC watchdog to 60 s. Secure Download Mode stops esptool
  from disabling it, and a USB recovery after a power-cycle takes about 30 s for
  a 1.4 MB app (~20 s per MB). Raise it if the app grows past ~2.5 MB;
- use [`partitions-secure.csv`](../firmware/partitions-secure.csv): the secure
  bootloader needs the partition table at 0x10000, so `ota_0` gives up 64 KB.
  Units change layout only when provisioned by cable, never by OTA.

`idf.py flash` does not provision a secure build.

The image refuses to run on a chip whose Secure Boot state does not match it
(`dcs_support_init`, before NVS is touched), so a wrong-profile OTA rolls back.
