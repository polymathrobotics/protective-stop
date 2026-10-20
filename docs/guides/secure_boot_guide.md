---
title: Secure Boot Guide
---

<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# Secure Boot Guide

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

### 1. Set the admin password and Tailscale key

The admin password is compiled into the image, and the admin page cannot change it.
The release image uses the public password `microlink`, which the remote refuses over Tailscale.
To set your own, and optionally the Tailscale key, create a credentials file before the first build:

```sh
cd firmware
cp sdkconfig.credentials.example sdkconfig.credentials
$EDITOR sdkconfig.credentials            # CONFIG_ML_ADMIN_PASSWORD
rm -rf sdkconfig build                   # a stale sdkconfig keeps the old values
```

The machn role reads the same file from its own directory: `ln -s ../firmware/sdkconfig.credentials machn/sdkconfig.credentials`.

Every value in the credentials file is compiled into the image as plain text, so keep the image private;
`tools/release_guard.sh` refuses to release one.

### 2. Build

```sh
. ~/esp/esp-idf/export.sh          # ESP-IDF v5.5, installed as in the quickstart's Appendix E
cd firmware                        # or machn
idf.py build                       # secure-fe
```

The profile is fixed when `sdkconfig` is generated, so pick one per directory.
For `dev`, or to keep a second profile beside the first, use its own build directory:

```sh
idf.py -B build-dev -DSDKCONFIG=build-dev/sdkconfig -DPSTOP_PROFILE=dev build
```

Provision or update the unit from this build as described below.

The secure overlays in [`firmware/profiles/`](../../firmware/profiles/):

- build **unsigned** images, so CI, releases and adopters need no keys and each
  owner signs with their own;
- set the bootloader's RTC watchdog to 60 s. Secure Download Mode stops esptool
  from disabling it, and a USB recovery after a power-cycle takes about 30 s for
  a 1.4 MB app (~20 s per MB). Raise it if the app grows past ~2.5 MB;
- use [`partitions-secure.csv`](../../firmware/partitions-secure.csv): the secure
  bootloader needs the partition table at 0x10000, so `ota_0` gives up 64 KB.
  Units change layout only when provisioned by cable, never by OTA.

`idf.py flash` does not provision a secure build; [`tools/pstop_secure.py`](../../tools/pstop_secure.py)
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

Each variable holds a file path or a 1Password `op://` reference (assuming op cli is installed). Create them
outside the repository with `openssl genrsa -out primary.pem 3072` (the same for
`backup.pem`) and `openssl rand -base64 48 > fe_master.txt`, and keep the backup
key apart from the other two. Losing both signing keys means the units can never be updated again;
losing the master leaves only OTA updates, no USB recovery.

Export the variables in the shell that runs `pstop_secure.py`, either as file paths:

```sh
export PSTOP_SIGNING_KEY=~/pstop-keys/primary.pem
export PSTOP_BACKUP_KEY=~/pstop-keys/backup.pem
export PSTOP_FE_MASTER=~/pstop-keys/fe_master.txt
```

or as 1Password references, read with the [`op` CLI](https://developer.1password.com/docs/cli/get-started/) (signed in first, e.g. `op signin`):

```sh
export PSTOP_SIGNING_KEY='op://Vault/pstop primary key/private key'
export PSTOP_BACKUP_KEY='op://Vault/pstop backup key/private key'
export PSTOP_FE_MASTER='op://Vault/pstop fe master/password'
```

Each command reads only the secrets it needs:

| Command | Variables |
|---|---|
| `sign-bootloader` | `PSTOP_SIGNING_KEY`, `PSTOP_BACKUP_KEY` |
| `sign-ota` | `PSTOP_SIGNING_KEY` |
| `provision`, `flash` | `PSTOP_SIGNING_KEY`, and `PSTOP_FE_MASTER` on `secure-fe` |

Secrets are staged in `/dev/shm` for the duration of one command and never written to disk.
`uv run` passes the environment through, so the variables need no other setup.

The bootloader carries both signatures and cannot be updated over the air, so it
is signed once per bootloader version, and provisioning needs only the primary:

```sh
cd tools
uv sync
uv run python pstop_secure.py sign-bootloader ../firmware/build -o bootloader-signed.bin
```

Each unit's NVS key is generated at provisioning, burned read-protected and never
stored, so your secrets do not reveal a unit's Tailscale identity. Provisioning
records (MAC, profile, key digests, a flash-encryption key id, the last full
eFuse summary; no secrets) go to `$PSTOP_RECORDS_DIR` (default
`~/.pstop-records`); with a record present, `flash` refuses another build profile
or master.

## Provisioning and Flashing

Provisioning is one-way: afterwards the unit only runs firmware signed with your
key. Put it in download mode on USB-Serial-JTAG (`lsusb` shows `303a:1001`): a
blank board is already there, a `dev` unit gets there with
`/api/enter_download` or by holding BOOT at power-on. Then:

```sh
# enter download mode (or see below for a full reset)
curl -u "admin:$ADMIN_PW" -X POST "$DEV/api/enter_download?confirm=1"
uv run python pstop_secure.py provision -p /dev/ttyACM0 --bootloader bootloader-signed.bin ../firmware/build
```

After you type the unit's MAC it follows Espressif's
["enable externally"](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/security/security-features-enablement-workflows.html)
workflow: keys, erase and write the images, the flash-encryption and JTAG
eFuses, a write-lock that keeps the USB recovery eFuses at 0, Secure Boot, and
Secure Download Mode last. It refuses to burn any eFuse that would end USB
recovery, resumes an interrupted run when repeated, and `--virt --mac MAC`
rehearses it on virtual eFuses. Then power-cycle the unit and enroll it with a
one-off Tailscale key: create the key as in
[Enrollment keys](tailscale_isolation.md) and enter it as in
[QUICKSTART §4](quickstart.md#4-first-boot-find-the-remote-and-give-it-the-key). An existing unit moves to `secure-fe` the same way: its
flash is erased and it enrolls as a new node.

## Updating a Flashed Device

Two signed files come out of `pstop_secure.py`, and they go to different places:

| File | Made by | Contents | Delivered by |
|---|---|---|---|
| `bootloader-signed.bin` | `sign-bootloader` | Bootloader, signed with both keys | USB only (`provision`, `flash --full`) |
| `pstop_remote-signed.bin` | `sign-ota` | The app from `firmware/build`, signed with the primary key | Network (`/admin/api/ota`) |

An update that changes only the app needs `pstop_remote-signed.bin`.
A change to the bootloader or partition table needs a USB flash, as in [Resetting a Flashed Device](#resetting-a-flashed-device).
A unit's profile is fixed by the eFuses burned at provisioning, so a flash cannot change it.

After `idf.py build`, with `PSTOP_SIGNING_KEY` set to the key the unit was provisioned with:

```sh
cd tools
uv run python pstop_secure.py sign-ota ../firmware/build -o pstop_remote-signed.bin
curl -u "admin:$ADMIN_PW" --data-binary @pstop_remote-signed.bin -X POST "http://$DEV/admin/api/ota"
```

## Resetting a Flashed Device

Over USB, for recovery, put the unit in download mode (`lsusb` shows `303a:1001`):
`/api/enter_download` hands the port to USB-Serial-JTAG for 60 s, so run the flash within that time.

```sh
cd tools
uv run python pstop_secure.py flash -p /dev/ttyACM0 --full --bootloader bootloader-signed.bin ../firmware/build
uv run python pstop_secure.py provision -p /dev/ttyACM0 --bootloader bootloader-signed.bin ../firmware/build
```

`--full` also rewrites the bootloader and partition table, so the unit ends up on exactly this build.
Drop `--full --bootloader …` to write only the app.

If the unit does not boot afterwards, power-cycle it and start the flash as soon as `303a:1001` appears:

```sh
until lsusb | grep -q 303a:1001 && [ -e /dev/ttyACM0 ]; do sleep 0.05; done; \
  uv run python pstop_secure.py flash -p /dev/ttyACM0 --full --bootloader bootloader-signed.bin ../firmware/build
```

Secure Download Mode only writes
(esptool 5.3.1 or newer); `uv run esptool --no-stub get-security-info` shows a unit's
state. Never enter download mode with a system reset: it re-arms the RTC
watchdog's ~9 s flash-boot protection, which esptool cannot disable in Secure
Download Mode.

## Tailscale

The firmware sends the auth key only to enroll, so one-off keys work and a unit
deleted from the tailnet stays deleted. An image built with the public default
admin password refuses admin requests over Tailscale. What a stolen unit's
identity can reach is up to your tailnet policy; see
[`tailscale_isolation.md`](tailscale_isolation.md).

## Limits

The ESP32-S3 has no glitch detector. Voltage fault injection can recover keys
from its AES engine
([AR2026-005](https://documentation.espressif.com/AR2026-005_Security_Advisory_Concerning_AES_Key_Recovery_Using_Voltage_Fault_Injection_on%20ESP32-S3_EN.html)),
and combined side-channel and fault attacks defeated Secure Boot and flash
encryption on the related ESP32-C3 and C6
([AR2023-007](https://documentation.espressif.com/AR2023-007%20Security%20Advisory%20Concerning%20Bypassing%20Secure%20Boot%20and%20Flash%20Encryption%20using%20CPA%20and%20FI%20attack%20on%20ESP32-C3%20and%20ESP32-C6%20EN.html)).
A well-equipped lab can eventually extract one unit's secrets. `secure-fe` makes
that a per-unit lab job instead of a USB cable and a few seconds, and what it
yields opens no other unit. Limit what a remote can reach in your tailnet policy
and delete lost units promptly.

Sources: ESP-IDF v5.5 [security overview](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/security/security.html),
[Secure Boot v2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/security/secure-boot-v2.html),
[flash encryption](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/security/flash-encryption.html),
[NVS encryption](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/nvs_encryption.html);
esptool [Secure Download Mode](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/basic-commands.html),
[esptool#1173](https://github.com/espressif/esptool/issues/1173);
[AR2022-004](https://documentation.espressif.com/AR2022-004%20Security%20Advisory%20for%20USB_OTG%20&%20USB_Serial_JTAG%20Download%20Functions%20of%20ESP32-S3%20Series%20Products%20EN.html),
[esp-idf#13946](https://github.com/espressif/esp-idf/issues/13946).
