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

`idf.py flash` does not provision a secure build; [`tools/pstop_secure.py`](../tools/pstop_secure.py)
does, as below.

The image refuses to run on a chip whose Secure Boot state does not match it
(`dcs_support_init`, before NVS is touched), so a wrong-profile OTA rolls back.

## Keys

Three secrets, however many units:

| Secret | Used for | Variable |
|---|---|---|
| Primary signing key (RSA-3072) | Signing apps, provisioning | `PSTOP_SIGNING_KEY` |
| Backup signing key (RSA-3072) | Co-signing each bootloader; replaces a lost or leaked primary | `PSTOP_BACKUP_KEY` |
| Flash-encryption master | Deriving each unit's key from its MAC (HKDF-SHA256) | `PSTOP_FE_MASTER` |

Each variable holds a file path or a 1Password `op://` reference. Create them
with `openssl genrsa -out primary.pem 3072` (the same for `backup.pem`) and
`openssl rand -base64 48 > fe_master.txt`, and keep the backup key apart from the
other two. Losing both signing keys means the units can never be updated again;
losing the master leaves only OTA updates, no USB recovery.

The bootloader carries both signatures and cannot be updated over the air, so it
is signed once per bootloader version, and provisioning needs only the primary:

```sh
cd tools
uv run python pstop_secure.py sign-bootloader ../firmware/build -o bootloader-signed.bin
```

Each unit's NVS key is generated at provisioning, burned read-protected and never
stored, so your secrets do not reveal a unit's Tailscale identity. Provisioning
records (MAC, key digests, the last full eFuse summary; no secrets) go to
`$PSTOP_RECORDS_DIR` (default `~/.pstop-records`).

## Provisioning

Provisioning is one-way: afterwards the unit only runs firmware signed with your
key. Put it in download mode on USB-Serial-JTAG (`lsusb` shows `303a:1001`): a
blank board is already there, a `dev` unit gets there with
`/api/enter_download` or by holding BOOT at power-on. Then:

```sh
uv run python pstop_secure.py provision -p /dev/ttyACM0 --bootloader bootloader-signed.bin ../firmware/build
```

After you type the unit's MAC it follows Espressif's
["enable externally"](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/security/security-features-enablement-workflows.html)
workflow: keys, erase and write the images, the flash-encryption and JTAG
eFuses, a write-lock that keeps the USB recovery eFuses at 0, Secure Boot, and
Secure Download Mode last. It refuses to burn any eFuse that would end USB
recovery, resumes an interrupted run when repeated, and `--virt --mac MAC`
rehearses it on virtual eFuses. Then power-cycle the unit and enroll it with a
one-off Tailscale key. An existing unit moves to `secure-fe` the same way: its
flash is erased and it enrolls as a new node.

## Updating

Over the network:

```sh
uv run python pstop_secure.py sign-ota ../firmware/build -o pstop_remote-signed.bin
curl -u "admin:$ADMIN_PW" --data-binary @pstop_remote-signed.bin -X POST "http://$DEV/admin/api/ota"
```

Over USB, for recovery: `/api/enter_download` hands the port to USB-Serial-JTAG
for 60 s; within that time run

```sh
uv run python pstop_secure.py flash -p /dev/ttyACM0 ../firmware/build
```

(`--full --bootloader bootloader-signed.bin` also rewrites the bootloader and
partition table). If the app does not run, power-cycle the unit and start the
same command as soon as `303a:1001` appears. Secure Download Mode only writes
(esptool 5.3.1 or newer); `esptool --no-stub get-security-info` shows a unit's
state. Never enter download mode with a system reset: it re-arms the RTC
watchdog's ~9 s flash-boot protection, which esptool cannot disable in Secure
Download Mode.
