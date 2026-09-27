# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""pstop_secure.py against espefuse's virtual eFuses and a synthetic build directory."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pstop_secure
import pytest
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

MAC = '3c:0f:02:00:00:01'
MASTER = b'test-master-secret-that-is-at-least-32-chars-long'


def espsecure(*args):
    return subprocess.run(
        [sys.executable, '-m', 'espsecure', *map(str, args)], capture_output=True, text=True, check=True
    ).stdout


def make_build(root: Path, fe: bool) -> Path:
    build = root / ('build-fe' if fe else 'build-secure')
    files = {
        'bootloader/bootloader.bin': 8192,
        'partition_table/partition-table.bin': 4096,
        'ota_data_initial.bin': 8192,
        'pstop_remote.bin': 8192,
    }
    for name, size in files.items():
        (build / name).parent.mkdir(parents=True, exist_ok=True)
        (build / name).write_bytes(os.urandom(size))
    (build / 'config').mkdir()
    (build / 'config' / 'sdkconfig.json').write_text(
        json.dumps({
            'SECURE_BOOT': True,
            'SECURE_FLASH_ENC_ENABLED': fe,
            'SECURE_BOOT_BUILD_SIGNED_BINARIES': False,
            'NVS_SEC_HMAC_EFUSE_KEY_ID': 5,
            'ESPTOOLPY_FLASHSIZE': '8MB',
        })
    )
    (build / 'flasher_args.json').write_text(
        json.dumps({
            'app': {'offset': '0x30000', 'file': 'pstop_remote.bin'},
            'partition-table': {'offset': '0x10000', 'file': 'partition_table/partition-table.bin'},
            'otadata': {'offset': '0x21000', 'file': 'ota_data_initial.bin'},
        })
    )
    return build


@pytest.fixture(scope='module')
def key_files(tmp_path_factory):
    d = tmp_path_factory.mktemp('keys')
    for name in ('primary', 'backup'):
        espsecure('generate-signing-key', '--version', '2', '--scheme', 'rsa3072', d / f'{name}.pem')
    (d / 'fe_master.txt').write_bytes(MASTER + b'\n')
    return d


@pytest.fixture
def env(key_files, tmp_path, monkeypatch):
    monkeypatch.setenv(pstop_secure.ENV_SIGNING_KEY, str(key_files / 'primary.pem'))
    monkeypatch.setenv(pstop_secure.ENV_BACKUP_KEY, str(key_files / 'backup.pem'))
    monkeypatch.setenv(pstop_secure.ENV_FE_MASTER, str(key_files / 'fe_master.txt'))
    monkeypatch.setenv('PSTOP_RECORDS_DIR', str(tmp_path / 'records'))
    return key_files


def signed_bootloader(build: Path) -> Path:
    out = build / 'bootloader-signed.bin'
    assert pstop_secure.main(['sign-bootloader', str(build), '-o', str(out)]) == 0
    return out


def virt_summary(tmp_path: Path) -> dict:
    virt = tmp_path / 'records' / 'virt' / MAC.replace(':', '') / 'virt-efuse.json'
    return pstop_secure.Efuse(virt_file=virt).summary()


def provision(build: Path, bootloader: Path) -> int:
    return pstop_secure.main(['provision', '--virt', '--mac', MAC, '--bootloader', str(bootloader), str(build)])


def assert_recovery_path_intact(s):
    for name in ('DIS_DOWNLOAD_MODE', 'DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE', 'DIS_USB_SERIAL_JTAG', 'DIS_FORCE_DOWNLOAD'):
        assert s[name]['value'] is False, name
    assert pstop_secure.efuse_raw(s, 'USB_PHY_SEL') == 0
    for name in ('DIS_USB_SERIAL_JTAG', 'USB_PHY_SEL', 'DIS_FORCE_DOWNLOAD'):
        assert s[name]['writeable'] is False, name


def test_fe_key_derivation_is_rfc5869_hkdf_and_per_unit():
    key = pstop_secure.derive_fe_key(MASTER + b'\n', MAC)
    reference = HKDF(
        algorithm=hashes.SHA256(),
        length=32,
        salt=pstop_secure.FE_KDF_SALT,
        info=pstop_secure.FE_KDF_INFO + bytes.fromhex(MAC.replace(':', '')),
    ).derive(MASTER)
    assert key == reference
    assert key != pstop_secure.derive_fe_key(MASTER, '3c:0f:02:00:00:02')
    with pytest.raises(pstop_secure.ProvisionError, match='too short'):
        pstop_secure.derive_fe_key(b'short', MAC)


def test_secure_download_write_is_forced_and_stubless(monkeypatch):
    calls = []
    monkeypatch.setattr(pstop_secure, 'run', lambda argv, capture=False: calls.append([str(a) for a in argv]))
    pstop_secure.write_flash('/dev/ttyACM0', [(0x30000, Path('app.bin'))], '8MB', secure_download=True)
    assert '--no-stub' in calls[0] and '--force' in calls[0]
    assert calls[0][calls[0].index('--after') + 1] == 'hard-reset'


@pytest.mark.parametrize('name', sorted(pstop_secure.NEVER_BURN))
def test_never_burn_is_refused(name):
    with pytest.raises(pstop_secure.ProvisionError, match=name):
        pstop_secure.check_never_burn(['burn-efuse', name, '1'])


def test_dev_build_is_rejected(tmp_path):
    build = make_build(tmp_path, fe=True)
    cfg = build / 'config' / 'sdkconfig.json'
    cfg.write_text(json.dumps({**json.loads(cfg.read_text()), 'SECURE_BOOT': False}))
    with pytest.raises(pstop_secure.ProvisionError, match='dev build'):
        pstop_secure.load_build(build)


def test_bootloader_from_another_build_is_rejected(tmp_path, env):
    signed = signed_bootloader(make_build(tmp_path / 'a', fe=True))
    assert provision(make_build(tmp_path / 'b', fe=True), signed) == 1


def test_images_are_signed_then_encrypted(tmp_path, env):
    build = make_build(tmp_path, fe=True)
    signed = signed_bootloader(build)
    b = pstop_secure.load_build(build)
    fe_key = tmp_path / 'fe.bin'
    fe_key.write_bytes(pstop_secure.derive_fe_key(MASTER, MAC))
    out = tmp_path / 'out'
    out.mkdir()
    images = dict(pstop_secure.prepare_images(b, env / 'primary.pem', fe_key, ('bootloader', 'app'), out, signed))
    assert set(images) == {0x0, 0x30000}
    plain = tmp_path / 'app-plain.bin'
    espsecure(
        'decrypt-flash-data', '--aes-xts', '--keyfile', fe_key, '--address', '0x30000', '-o', plain, images[0x30000]
    )
    espsecure('verify-signature', '--version', '2', '--keyfile', env / 'primary.pem', plain)


def test_virtual_provision_secure_fe(tmp_path, env):
    build = make_build(tmp_path, fe=True)
    signed = signed_bootloader(build)
    assert provision(build, signed) == 0
    s = virt_summary(tmp_path)
    assert s['SECURE_BOOT_EN']['value'] is True
    assert s['ENABLE_SECURITY_DOWNLOAD']['value'] is True
    assert pstop_secure.efuse_raw(s, 'SPI_BOOT_CRYPT_CNT') == 7
    purposes = [s[f'KEY_PURPOSE_{i}']['value'] for i in range(6)]
    assert purposes == ['XTS_AES_128_KEY', 'SECURE_BOOT_DIGEST0', 'SECURE_BOOT_DIGEST1', 'USER', 'USER', 'HMAC_UP']
    assert s['BLOCK_KEY0']['readable'] is False and s['BLOCK_KEY5']['readable'] is False
    assert s['SECURE_BOOT_KEY_REVOKE2']['value'] is True
    assert s['RD_DIS']['writeable'] is False
    for name in (
        'DIS_DOWNLOAD_MANUAL_ENCRYPT',
        'DIS_DOWNLOAD_ICACHE',
        'DIS_PAD_JTAG',
        'DIS_USB_JTAG',
        'DIS_DIRECT_BOOT',
    ):
        assert s[name]['value'] is True, name
    assert pstop_secure.efuse_raw(s, 'SOFT_DIS_JTAG') == 7
    assert_recovery_path_intact(s)

    for block, key in (('BLOCK_KEY1', 'primary.pem'), ('BLOCK_KEY2', 'backup.pem')):
        digest = tmp_path / f'{key}.digest'
        espsecure('digest-sbv2-public-key', '--keyfile', env / key, '--output', digest)
        assert s[block]['value'].replace(' ', '') == digest.read_bytes().hex()

    udir = tmp_path / 'records' / 'virt' / MAC.replace(':', '')
    assert sorted(p.name for p in udir.iterdir()) == ['efuse-summary.json', 'record.json', 'virt-efuse.json']
    assert json.loads((udir / 'record.json').read_text())['state'] == 'done'
    assert provision(build, signed) == 1  # already provisioned: Secure Download Mode is on


def test_virtual_provision_secure_has_no_flash_encryption(tmp_path, env):
    build = make_build(tmp_path, fe=False)
    assert provision(build, signed_bootloader(build)) == 0
    s = virt_summary(tmp_path)
    assert s['SECURE_BOOT_EN']['value'] is True
    assert pstop_secure.efuse_raw(s, 'SPI_BOOT_CRYPT_CNT') == 0
    assert s['KEY_PURPOSE_0']['value'] == 'USER'
    assert s['KEY_PURPOSE_5']['value'] == 'HMAC_UP'
    assert_recovery_path_intact(s)


def test_interrupted_provision_resumes(tmp_path, env, monkeypatch):
    build = make_build(tmp_path, fe=True)
    signed = signed_bootloader(build)
    real_burn = pstop_secure.Efuse.burn

    def failing_burn(self, *args):
        if 'SECURE_BOOT_EN' in args:
            raise pstop_secure.ProvisionError('simulated USB drop')
        return real_burn(self, *args)

    monkeypatch.setattr(pstop_secure.Efuse, 'burn', failing_burn)
    assert provision(build, signed) == 1
    assert virt_summary(tmp_path)['SECURE_BOOT_EN']['value'] is False

    monkeypatch.setattr(pstop_secure.Efuse, 'burn', real_burn)
    assert provision(build, signed) == 0
    s = virt_summary(tmp_path)
    assert s['SECURE_BOOT_EN']['value'] is True and s['ENABLE_SECURITY_DOWNLOAD']['value'] is True
