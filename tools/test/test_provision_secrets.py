# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Tests for provision_secrets.py: credentials parsing, the encrypted image, the
eFuse provisioning flow against espefuse's virtual eFuses, and the contract
with the firmware (ml_secrets.h/.c, Kconfig, partitions.csv)."""

import argparse
import re
import stat
import subprocess
from pathlib import Path

import provision_secrets as ps
import pytest
from esp_idf_nvs_partition_gen import nvs_partition_gen as nvs_gen

REPO = Path(__file__).resolve().parents[2]

CREDENTIALS = """
# comment
TAILSCALE_AUTH_KEY="tskey-auth-kTEST1-abcdefghijklmnopqrstuvwxyz"
ADMIN_PASSWORD=hunter2-admin
WIFI_SSID="lab net"
WIFI_PASSWORD=""
"""


def decrypt(image, hmac_key, tmp_path):
    """The plaintext NVS image, via nvs_partition_gen's own decrypt."""
    (tmp_path / 'enc.bin').write_bytes(image)
    (tmp_path / 'xts.bin').write_bytes(ps.derive_xts_keys(hmac_key))
    nvs_gen.decrypt(
        argparse.Namespace(
            input=str(tmp_path / 'enc.bin'), key=str(tmp_path / 'xts.bin'), output='dec.bin', outdir=str(tmp_path)
        )
    )
    return (tmp_path / 'dec.bin').read_bytes()


# --- parse_credentials -------------------------------------------------------


def test_parse_drops_empty_values_and_unquotes():
    assert ps.parse_credentials(CREDENTIALS) == {
        'TAILSCALE_AUTH_KEY': 'tskey-auth-kTEST1-abcdefghijklmnopqrstuvwxyz',
        'ADMIN_PASSWORD': 'hunter2-admin',
        'WIFI_SSID': 'lab net',
    }


def test_parse_accepts_a_legacy_sdkconfig_credentials_file():
    """Kconfig string escapes and the CONFIG_ML_ prefix of the old file."""
    text = 'CONFIG_ML_ADMIN_PASSWORD="a\\"b\\\\c"\nCONFIG_ML_OTA_API_KEY=""\n'
    assert ps.parse_credentials(text) == {'ADMIN_PASSWORD': 'a"b\\c'}


@pytest.mark.parametrize(
    'text, match',
    [
        ('TAILSCALE_KEY=x', 'unknown key'),
        ('no equals sign', 'unknown key'),
        ('CONFIG_ML_DEVICE_NAME="x"', 'unknown key'),
        ('ADMIN_PASSWORD=a\nADMIN_PASSWORD=b', 'set twice'),
        ('OTA_API_KEY=' + 'x' * 129, 'longer than 128'),
    ],
)
def test_parse_rejects(text, match):
    with pytest.raises(ValueError, match=match):
        ps.parse_credentials(text)


def test_parse_accepts_a_value_at_the_limit():
    assert ps.parse_credentials('OTA_API_KEY=' + 'x' * 128) == {'OTA_API_KEY': 'x' * 128}


# --- key derivation and image ------------------------------------------------


def test_xts_keys_match_nvs_partition_gen_hmac_scheme(tmp_path):
    """Same 64 bytes nvs_partition_gen writes for --key_protect_hmac, i.e. what the chip derives."""
    hmac_key = bytes(range(32))
    (tmp_path / 'hmac.bin').write_bytes(hmac_key)
    nvs_gen.generate_key(
        argparse.Namespace(
            keyfile='keys.bin',
            outdir=str(tmp_path),
            key_protect_hmac=True,
            kp_hmac_keygen=False,
            kp_hmac_inputkey=str(tmp_path / 'hmac.bin'),
        )
    )
    assert ps.derive_xts_keys(hmac_key) == (tmp_path / 'keys' / 'keys.bin').read_bytes()[:64]


def test_xts_keys_reject_a_short_key():
    with pytest.raises(ValueError, match='32 bytes'):
        ps.derive_xts_keys(b'\x00' * 16)


def test_image_is_partition_sized_and_holds_no_plaintext(tmp_path):
    values = ps.parse_credentials(CREDENTIALS)
    hmac_key = bytes(range(32))
    image = ps.build_partition(values, hmac_key)
    assert len(image) == ps.PARTITION_SIZE
    for v in values.values():
        assert v.encode() not in image
    plain = decrypt(image, hmac_key, tmp_path)
    for v in values.values():
        assert v.encode() in plain
    assert ps.NAMESPACE.encode() in plain


def test_image_does_not_decrypt_under_another_key(tmp_path):
    image = ps.build_partition({'ADMIN_PASSWORD': 'hunter2-admin'}, bytes(range(32)))
    assert b'hunter2-admin' not in decrypt(image, bytes(32), tmp_path)


def test_write_private_is_owner_only_and_never_overwrites(tmp_path):
    path = tmp_path / 'keys' / 'k.bin'
    ps.write_private(path, b'x')
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
    with pytest.raises(FileExistsError):
        ps.write_private(path, b'y')
    assert path.read_bytes() == b'x'


# --- provision against virtual eFuses ------------------------------------------


@pytest.fixture
def virt(tmp_path):
    """espefuse argv for a fresh virtual ESP32-S3, and its eFuse file."""
    efuse = tmp_path / 'efuse.bin'
    return ps.espefuse_base(None, virt_efuse=efuse), efuse


def block_state(base, block=ps.DEFAULT_KEY_BLOCK):
    return ps.key_block_state(ps.read_efuse_summary(base), block)


def test_first_provision_burns_a_kept_key_and_reprovision_reuses_it(virt, tmp_path):
    base, _ = virt
    keys = tmp_path / 'keys'
    assert block_state(base) == 'empty'

    image = ps.provision(base, CREDENTIALS, keys, log=lambda m: None)

    [key_file] = keys.iterdir()
    assert re.fullmatch(r'[0-9a-f]{12}\.bin', key_file.name)
    assert stat.S_IMODE(key_file.stat().st_mode) == 0o600
    assert block_state(base) == 'hmac'
    summary = ps.read_efuse_summary(base)
    assert summary[f'BLOCK_KEY{ps.DEFAULT_KEY_BLOCK}']['readable'] is False
    assert b'hunter2-admin' in decrypt(image, key_file.read_bytes(), tmp_path)

    again = ps.provision(base, 'ADMIN_PASSWORD=rotated-pw', keys, log=lambda m: None)
    assert [key_file] == list(keys.iterdir())
    assert b'rotated-pw' in decrypt(again, key_file.read_bytes(), tmp_path)


def test_burned_block_without_the_local_key_is_refused(virt, tmp_path):
    base, _ = virt
    ps.provision(base, CREDENTIALS, tmp_path / 'host_a', log=lambda m: None)
    with pytest.raises(ps.ProvisionError, match='already holds an HMAC key'):
        ps.provision(base, CREDENTIALS, tmp_path / 'host_b', log=lambda m: None)


def test_block_used_for_another_purpose_is_refused(virt, tmp_path):
    base, _ = virt
    other = tmp_path / 'other.bin'
    other.write_bytes(bytes(32))
    subprocess.run(
        [*base, '--do-not-confirm', 'burn-key', 'BLOCK_KEY5', str(other), 'HMAC_DOWN_JTAG'],
        check=True,
        capture_output=True,
    )
    with pytest.raises(ps.ProvisionError, match='non-HMAC'):
        ps.provision(base, CREDENTIALS, tmp_path / 'keys', log=lambda m: None)
    assert not (tmp_path / 'keys').exists()


def test_bad_credentials_burn_nothing(virt, tmp_path):
    base, _ = virt
    with pytest.raises(ps.ProvisionError, match='credentials'):
        ps.provision(base, 'TYPO_KEY=1', tmp_path / 'keys', log=lambda m: None)
    assert block_state(base) == 'empty'
    assert not (tmp_path / 'keys').exists()


def test_key_block_selects_the_burned_block(virt, tmp_path):
    base, _ = virt
    ps.provision(base, CREDENTIALS, tmp_path / 'keys', block=2, log=lambda m: None)
    assert block_state(base, 2) == 'hmac'
    assert block_state(base, ps.DEFAULT_KEY_BLOCK) == 'empty'


def test_cli_provision_writes_an_owner_only_image(virt, tmp_path):
    _, efuse = virt
    creds = tmp_path / 'c.env'
    creds.write_text(CREDENTIALS)
    out = tmp_path / 'secrets.bin'
    ps.main(
        [
            'provision',
            '--virt-efuse', str(efuse),
            '--credentials', str(creds),
            '--keys-dir', str(tmp_path / 'keys'),
            str(out),
        ]
    )  # fmt: skip
    assert len(out.read_bytes()) == ps.PARTITION_SIZE
    assert stat.S_IMODE(out.stat().st_mode) == 0o600


# --- contract with the firmware -------------------------------------------------


def test_nvs_keys_match_ml_secrets_c():
    src = (REPO / 'components/microlink/src/ml_secrets.c').read_text()
    firmware_keys = set(re.findall(r'\[ML_SECRET_\w+\]\s*=\s*"(\w+)"', src))
    assert firmware_keys == set(ps.CREDENTIAL_KEYS.values())
    assert all(len(k) <= 15 for k in firmware_keys)  # NVS key limit


def test_constants_match_ml_secrets_h():
    hdr = (REPO / 'components/microlink/include/ml_secrets.h').read_text()
    assert re.search(r'#define ML_SECRETS_NAMESPACE "(\w+)"', hdr).group(1) == ps.NAMESPACE
    assert int(re.search(r'#define ML_SECRETS_MAX_LEN (\d+)', hdr).group(1)) == ps.MAX_VALUE_BYTES + 1


def test_default_key_block_matches_kconfig():
    kconfig = (REPO / 'components/microlink/Kconfig').read_text()
    m = re.search(r'config ML_SECRETS_HMAC_KEY_ID\n(?:.*\n)*?\s+default (\d+)', kconfig)
    assert int(m.group(1)) == ps.DEFAULT_KEY_BLOCK


def _partition_offsets(csv_text):
    """{name: (offset, size)} laid out the way gen_esp32part.py does for blank offsets."""
    offset = 0x9000
    table = {}
    for row in csv_text.splitlines():
        if not row.strip() or row.lstrip().startswith('#'):
            continue
        name, ptype, _, _, size = (c.strip() for c in row.split(',')[:5])
        if 'app' == ptype:
            offset = (offset + 0xFFFF) & ~0xFFFF
        table[name] = (offset, int(size, 0))
        offset += int(size, 0)
    return table


@pytest.mark.parametrize('role', ['firmware', 'machn'])
def test_partition_offset_and_size_match_partitions_csv(role):
    table = _partition_offsets((REPO / role / 'partitions.csv').read_text())
    assert table['secrets'] == (ps.PARTITION_OFFSET, ps.PARTITION_SIZE)
    assert table['ota_0'][0] == 0x20000  # OTA geometry unchanged
