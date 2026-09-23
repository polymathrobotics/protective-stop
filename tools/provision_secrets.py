#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""provision_secrets.py — build and flash-prepare a unit's encrypted secrets partition.

The firmware reads its credentials (Tailscale auth key, admin password, WiFi,
fleet OTA backend) from the "secrets" NVS partition, XTS-AES encrypted with keys
derived from a per-device HMAC key in eFuse (see
components/microlink/include/ml_secrets.h). No firmware image carries them.

    uv run python provision_secrets.py keygen device_keys/new.bin
    uv run python provision_secrets.py build --hmac-key K.bin --credentials credentials.env secrets.bin
    uv run python provision_secrets.py provision --port /dev/ttyACM0 --credentials credentials.env secrets.bin

`provision` is what tools/flash_pstop.sh and flash_station.py run: it reads the
unit's MAC and eFuse state, burns a fresh HMAC key on first use (irreversible,
one eFuse key block per unit), keeps that key as <keys-dir>/<mac>.bin, and
builds the partition image. Flash the image at PARTITION_OFFSET.

A unit's key file decrypts that unit's secrets, and nothing else: keep
device_keys/ private. Losing it only means re-provisioning that unit by burning
a spare key block (--key-block, matching CONFIG_ML_SECRETS_HMAC_KEY_ID).
"""

import argparse
import hashlib
import hmac
import io
import json
import os
import secrets
import subprocess
import sys
from pathlib import Path

from esp_idf_nvs_partition_gen import nvs_partition_gen as nvs_gen

HERE = Path(__file__).resolve().parent
DEFAULT_CREDENTIALS = HERE / 'credentials.env'
DEFAULT_KEYS_DIR = HERE / 'device_keys'

# Contract with ml_secrets.h and partitions.csv; tools/test/test_provision_secrets.py checks it.
NAMESPACE = 'ml_secrets'
PARTITION_OFFSET = 0x1C000
PARTITION_SIZE = 0x3000
MAX_VALUE_BYTES = 128  # ML_SECRETS_MAX_LEN less the NUL
DEFAULT_KEY_BLOCK = 5  # CONFIG_ML_SECRETS_HMAC_KEY_ID default
CHIP = 'esp32s3'

# Credentials-file name -> NVS key.
CREDENTIAL_KEYS = {
    'TAILSCALE_AUTH_KEY': 'ts_auth_key',
    'ADMIN_PASSWORD': 'admin_pw',
    'WIFI_SSID': 'wifi_ssid',
    'WIFI_PASSWORD': 'wifi_pass',
    'WIFI_SSID_2': 'wifi_ssid_2',
    'WIFI_PASSWORD_2': 'wifi_pass_2',
    'FLEET_SERVER_IP': 'fleet_srv_ip',
    'OTA_BACKEND_URL': 'ota_url',
    'OTA_API_KEY': 'ota_api_key',
}
# An old sdkconfig.credentials parses as-is.
LEGACY_PREFIX = 'CONFIG_ML_'

# nvs_sec_provider.c derives the XTS-AES key pair as HMAC-SHA256 over these seeds.
_EKEY_SEED = bytes.fromhex('5a5abeae') * 8
_TKEY_SEED = bytes.fromhex('a5a5dece') * 8
HMAC_KEY_BYTES = 32


class ProvisionError(Exception):
    """A refusal to provision, with the reason for the operator."""


def parse_credentials(text):
    """`KEY=value` lines to {KEY: value}, empty values dropped.

    `#` starts a comment line. A value may be double-quoted, with `\\"` and
    `\\\\` escapes. Raises ValueError on an unknown or repeated key, or a value
    longer than MAX_VALUE_BYTES.
    """
    values = {}
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        key, sep, value = line.partition('=')
        key = key.strip().removeprefix(LEGACY_PREFIX)
        if not sep or key not in CREDENTIAL_KEYS:
            raise ValueError(f'line {n}: unknown key {key!r}; expected one of {", ".join(CREDENTIAL_KEYS)}')
        if key in values:
            raise ValueError(f'line {n}: {key} set twice')
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] == '"':
            value = value[1:-1].replace('\\"', '"').replace('\\\\', '\\')
        if len(value.encode()) > MAX_VALUE_BYTES:
            raise ValueError(f'line {n}: {key} is longer than {MAX_VALUE_BYTES} bytes')
        values[key] = value
    return {k: v for k, v in values.items() if v}


def derive_xts_keys(hmac_key):
    """The 64-byte NVS encryption key (eky || tky) the chip derives from `hmac_key`."""
    if len(hmac_key) != HMAC_KEY_BYTES:
        raise ValueError(f'HMAC key must be {HMAC_KEY_BYTES} bytes, got {len(hmac_key)}')
    return hmac.digest(hmac_key, _EKEY_SEED, hashlib.sha256) + hmac.digest(hmac_key, _TKEY_SEED, hashlib.sha256)


def build_partition(values, hmac_key):
    """The encrypted PARTITION_SIZE-byte NVS image holding `values` ({KEY: value})."""
    size, read_only = nvs_gen.check_size(hex(PARTITION_SIZE))
    out = io.BytesIO()
    with nvs_gen.nvs_open(
        out, size, nvs_gen.Page.VERSION2, is_encrypt=True, key=derive_xts_keys(hmac_key), read_only=read_only
    ) as nvs:
        nvs_gen.write_entry(nvs, NAMESPACE, 'namespace', '', '')
        for key, value in values.items():
            nvs_gen.write_entry(nvs, CREDENTIAL_KEYS[key], 'data', 'string', value)
    return out.getvalue()


def write_private(path, data):
    """Write `data` to a new file readable only by its owner, synced to disk.

    Raises FileExistsError rather than overwrite.
    """
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


# --- eFuse -----------------------------------------------------------------


def espefuse_base(port, virt_efuse=None):
    """espefuse argv prefix for `port`, or for a virtual eFuse file (no chip)."""
    base = [sys.executable, '-m', 'espefuse', '--chip', CHIP]
    if virt_efuse:
        return [*base, '--virt', '--path-efuse-file', str(virt_efuse)]
    return [*base, '-p', port, '--after', 'no-reset']


def _run(argv):
    r = subprocess.run(argv, capture_output=True, text=True)
    if r.returncode != 0:
        raise ProvisionError(f'{" ".join(argv[2:4])} failed ({r.returncode}): {(r.stderr or r.stdout).strip()}')
    return r.stdout


def read_efuse_summary(base):
    """espefuse's JSON summary as a dict."""
    out = _run([*base, 'summary', '--format', 'json'])
    return json.loads(out[out.index('{') :])


def key_block_state(summary, block):
    """'empty', 'hmac' (an HMAC_UP key is burned), or 'other' for BLOCK_KEY<block>."""
    purpose = summary[f'KEY_PURPOSE_{block}']['value']
    if purpose == 'HMAC_UP':
        return 'hmac'
    data = summary[f'BLOCK_KEY{block}']
    if purpose == 'USER' and data['readable'] and set(data['value'].split()) == {'00'}:
        return 'empty'
    return 'other'


def mac_of(summary):
    """The factory MAC as 12 lowercase hex digits."""
    return summary['MAC']['value'].split()[0].replace(':', '').lower()


def provision(base, credentials_text, keys_dir, block=DEFAULT_KEY_BLOCK, log=print):
    """The encrypted secrets image for the unit `base` reaches.

    Burns a new HMAC key into BLOCK_KEY<block> when that block is empty, keeping
    it as <keys_dir>/<mac>.bin first. Raises ProvisionError, before burning
    anything, on bad credentials or a block this host cannot use.
    """
    try:
        values = parse_credentials(credentials_text)
    except ValueError as e:
        raise ProvisionError(f'credentials: {e}') from e
    summary = read_efuse_summary(base)
    mac = mac_of(summary)
    key_path = Path(keys_dir) / f'{mac}.bin'
    state = key_block_state(summary, block)

    if state == 'other':
        raise ProvisionError(f'{mac}: eFuse BLOCK_KEY{block} holds a non-HMAC key or data; pick a free --key-block')
    if state == 'hmac' and not key_path.exists():
        raise ProvisionError(
            f'{mac}: BLOCK_KEY{block} already holds an HMAC key and {key_path} is missing. '
            f'Copy that file from the host that provisioned this unit, or burn a free block with --key-block'
        )
    if state == 'empty':
        if not key_path.exists():
            write_private(key_path, secrets.token_bytes(HMAC_KEY_BYTES))
            log(f'{mac}: new HMAC key saved to {key_path}')
        log(f'{mac}: burning HMAC key into eFuse BLOCK_KEY{block} (irreversible)')
        _run([*base, '--do-not-confirm', 'burn-key', f'BLOCK_KEY{block}', str(key_path), 'HMAC_UP'])

    image = build_partition(values, key_path.read_bytes())
    log(f'{mac}: secrets partition built ({", ".join(sorted(values)) or "no values"})')
    return image


# --- CLI -------------------------------------------------------------------


def _cmd_keygen(args):
    write_private(args.out, secrets.token_bytes(HMAC_KEY_BYTES))


def _cmd_build(args):
    values = parse_credentials(Path(args.credentials).read_text())
    Path(args.out).unlink(missing_ok=True)
    write_private(args.out, build_partition(values, Path(args.hmac_key).read_bytes()))


def _cmd_provision(args):
    base = espefuse_base(args.port, args.virt_efuse)
    image = provision(
        base, Path(args.credentials).read_text(), args.keys_dir, args.key_block, log=lambda m: print(m, file=sys.stderr)
    )
    Path(args.out).unlink(missing_ok=True)
    write_private(args.out, image)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('keygen', help='write a new random HMAC key (mode 0600)')
    p.add_argument('out')
    p.set_defaults(func=_cmd_keygen)

    p = sub.add_parser('build', help='encrypt a credentials file into a partition image')
    p.add_argument('--hmac-key', required=True)
    p.add_argument('--credentials', default=DEFAULT_CREDENTIALS)
    p.add_argument('out')
    p.set_defaults(func=_cmd_build)

    p = sub.add_parser('provision', help='ensure the unit on --port has a key, then build its image')
    target = p.add_mutually_exclusive_group(required=True)
    target.add_argument('--port')
    target.add_argument('--virt-efuse', help='virtual eFuse file instead of a chip (testing)')
    p.add_argument('--credentials', default=DEFAULT_CREDENTIALS)
    p.add_argument('--keys-dir', default=DEFAULT_KEYS_DIR)
    p.add_argument('--key-block', type=int, choices=range(6), default=DEFAULT_KEY_BLOCK)
    p.add_argument('out')
    p.set_defaults(func=_cmd_provision)

    args = ap.parse_args(argv)
    try:
        args.func(args)
    except (ProvisionError, ValueError, FileExistsError) as e:
        sys.exit(f'provision_secrets: {e}')


if __name__ == '__main__':
    main()
