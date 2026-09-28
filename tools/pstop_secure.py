#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Sign, encrypt, provision and re-flash units of the secure profiles; see docs/SECURITY_PROFILES.md.

Secrets come from PSTOP_SIGNING_KEY, PSTOP_BACKUP_KEY (sign-bootloader only) and PSTOP_FE_MASTER,
each a file path or a 1Password op:// reference.
"""

import argparse
import contextlib
import hashlib
import hmac
import json
import os
import re
import secrets
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

CHIP = 'esp32s3'
BOOTLOADER_OFFSET = 0x0
FE_KEY_BLOCK = 'BLOCK_KEY0'
SB_DIGEST_BLOCKS = ('BLOCK_KEY1', 'BLOCK_KEY2')
# Must match CONFIG_NVS_SEC_HMAC_EFUSE_KEY_ID in firmware/profiles/secure*.defaults.
HMAC_KEY_BLOCK = 'BLOCK_KEY5'
HMAC_KEY_ID = 5
# Changing either value strands every unit provisioned with the old derivation.
FE_KDF_SALT = b'pstop-flash-encryption'
FE_KDF_INFO = b'pstop flash encryption key v1 '

ENV_SIGNING_KEY = 'PSTOP_SIGNING_KEY'
ENV_BACKUP_KEY = 'PSTOP_BACKUP_KEY'
ENV_FE_MASTER = 'PSTOP_FE_MASTER'

# Each ends USB recovery or bricks the chip (esp-idf#13946).
NEVER_BURN = frozenset({
    'DIS_DOWNLOAD_MODE',
    'DIS_USB_SERIAL_JTAG',
    'DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE',
    'DIS_FORCE_DOWNLOAD',
    'USB_PHY_SEL',
    'SECURE_BOOT_AGGRESSIVE_REVOKE',
    'DIS_USB_OTG',
})

# As burned by the ESP-IDF 5.5 bootloader (bootloader_support/src/esp32s3/*_secure_features.c).
FE_SECURITY_EFUSES = (
    ('DIS_DOWNLOAD_MANUAL_ENCRYPT', 1),
    ('DIS_DOWNLOAD_ICACHE', 1),
    ('DIS_DOWNLOAD_DCACHE', 1),
)
SB_SECURITY_EFUSES = (
    ('DIS_PAD_JTAG', 1),
    ('DIS_USB_JTAG', 1),
    ('SOFT_DIS_JTAG', 7),
    ('DIS_DIRECT_BOOT', 1),
)


class ProvisionError(Exception):
    pass


def read_secret(env):
    ref = os.environ.get(env, '')
    if not ref:
        raise ProvisionError(f'set {env} to a file path or an op:// reference')
    if ref.startswith('op://'):
        try:
            r = subprocess.run(['op', 'read', '--no-newline', ref], capture_output=True)
        except OSError as e:
            raise ProvisionError(f'op read for {env}: {e}') from e
        if r.returncode != 0:
            raise ProvisionError(f'op read for {env} failed: {r.stderr.decode(errors="replace").strip()[-300:]}')
        return r.stdout
    try:
        return Path(ref).expanduser().read_bytes()
    except OSError as e:
        raise ProvisionError(f'{env}: {e}') from e


@contextlib.contextmanager
def secret_dir():
    if not os.path.isdir('/dev/shm'):
        raise ProvisionError('no /dev/shm: refusing to write secrets to disk')
    with tempfile.TemporaryDirectory(dir='/dev/shm') as td:
        os.chmod(td, 0o700)
        yield Path(td)


def write_secret(path, data):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as f:
        f.write(data)
    return path


def derive_fe_key(master, mac):
    """HKDF-SHA256 (RFC 5869), 32 bytes: the XTS-AES-128 flash encryption key."""
    master = master.strip()
    if len(master) < 32:
        raise ProvisionError(f'{ENV_FE_MASTER} is too short (need at least 32 characters)')
    prk = hmac.new(FE_KDF_SALT, master, hashlib.sha256).digest()
    info = FE_KDF_INFO + bytes.fromhex(mac.replace(':', ''))
    return hmac.new(prk, info + b'\x01', hashlib.sha256).digest()


def records_dir():
    return Path(os.environ.get('PSTOP_RECORDS_DIR', Path.home() / '.pstop-records')).expanduser()


def load_record(path):
    return json.loads(path.read_text()) if path.exists() else None


def check_record(record, profile, fe_key):
    """Refuses a build or FE master other than the unit was provisioned with; returns the key's public id."""
    kid = hashlib.sha256(b'pstop fe key id ' + fe_key).hexdigest()[:16] if fe_key else None
    if record and record.get('profile', profile) != profile:
        raise ProvisionError(f'the unit was provisioned as {record["profile"]}; this build is {profile}')
    if record and record.get('fe_key_id', kid) != kid:
        raise ProvisionError(f'{ENV_FE_MASTER} gives another flash-encryption key than the unit was provisioned with')
    return kid


def run(argv, capture=False):
    print('  $ ' + ' '.join(str(a) for a in argv), flush=True)
    r = subprocess.run([str(a) for a in argv], capture_output=capture, text=True)
    if r.returncode != 0:
        detail = (r.stderr or r.stdout or '').strip()[-400:] if capture else ''
        raise ProvisionError(f'command failed with exit code {r.returncode}. {detail}')
    return r.stdout if capture else ''


def py_tool(name):
    return [sys.executable, '-m', name]


class Efuse:
    def __init__(self, port=None, virt_file=None):
        base = py_tool('espefuse') + ['--chip', CHIP]
        self.base = base + (['--virt', '--path-efuse-file', virt_file] if virt_file else ['-p', port])

    def summary(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / 'summary.json'
            run(self.base + ['summary', '--format', 'json', '--file', out], capture=True)
            return json.loads(out.read_text())

    def burn(self, *args):
        check_never_burn(args)
        run(self.base + ['--do-not-confirm'] + list(args), capture=True)


def check_never_burn(args):
    bad = sorted(NEVER_BURN.intersection(str(a) for a in args))
    if bad:
        raise ProvisionError(f'refusing to burn {", ".join(bad)}: that would end USB recovery')


def espsecure(*args):
    return run(py_tool('espsecure') + list(args), capture=True)


def load_build(build):
    build = Path(build)
    try:
        fa = json.loads((build / 'flasher_args.json').read_text())
        cfg = json.loads((build / 'config' / 'sdkconfig.json').read_text())
    except FileNotFoundError as e:
        raise ProvisionError(f'{build} is not an ESP-IDF build directory ({e.filename} missing)') from e
    if not cfg.get('SECURE_BOOT'):
        raise ProvisionError(f'{build} is a dev build; build a secure profile (-DPSTOP_PROFILE=secure-fe)')
    if cfg.get('SECURE_BOOT_BUILD_SIGNED_BINARIES'):
        raise ProvisionError(
            'build signs its own binaries; the profiles expect CONFIG_SECURE_BOOT_BUILD_SIGNED_BINARIES=n'
        )
    if cfg.get('NVS_SEC_HMAC_EFUSE_KEY_ID') != HMAC_KEY_ID:
        raise ProvisionError(f'CONFIG_NVS_SEC_HMAC_EFUSE_KEY_ID must be {HMAC_KEY_ID} (key block {HMAC_KEY_BLOCK})')

    def part(key):
        return int(fa[key]['offset'], 16), build / fa[key]['file']

    return {
        'profile': 'secure-fe' if cfg.get('SECURE_FLASH_ENC_ENABLED') else 'secure',
        'flash_size': cfg.get('ESPTOOLPY_FLASHSIZE', 'keep'),
        'bootloader': (BOOTLOADER_OFFSET, build / 'bootloader' / 'bootloader.bin'),
        'partition-table': part('partition-table'),
        'otadata': part('otadata'),
        'app': part('app'),
    }


def key_digest(pem, tmp, name):
    out = tmp / f'{name}.digest'
    espsecure('digest-sbv2-public-key', '--keyfile', pem, '--output', out)
    return out.read_bytes()


def check_signed_bootloader(b, signed, primary, tmp):
    """Returns the two key digests to burn, read from the bootloader's signature blocks."""
    signed = Path(signed)
    plain = b['bootloader'][1].read_bytes()
    data = signed.read_bytes()
    if data[: len(plain)] != plain:
        raise ProvisionError(f'{signed} was not signed from {b["bootloader"][1]}; run sign-bootloader for this build')
    info = espsecure('signature-info-v2', signed)
    digests = [
        bytes.fromhex(d.replace(' ', '')) for d in re.findall(r'Public key digest for block \d: ([0-9a-f ]+)', info)
    ]
    if len(digests) != 2:
        raise ProvisionError(f'{signed} must carry two signatures (primary and backup), found {len(digests)}')
    if digests[0] != key_digest(primary, tmp, 'primary'):
        raise ProvisionError(f'the first signature on {signed} is not from {ENV_SIGNING_KEY}')
    if digests[0] == digests[1]:
        raise ProvisionError(f'both signatures on {signed} are from the same key; the backup key must differ')
    return digests


def prepare_images(b, primary, fe_key, names, tmp, bootloader=None):
    """App signed with the primary key, bootloader as signed by sign-bootloader; fe_key encrypts per offset."""
    out = []
    for name in names:
        offset, src = b[name]
        if not src.is_file():
            raise ProvisionError(f'missing build output {src}')
        img = src
        if name == 'bootloader':
            img = Path(bootloader)
        elif name == 'app':
            img = tmp / 'app-signed.bin'
            espsecure('sign-data', '--version', '2', '--keyfile', primary, '--output', img, src)
        if fe_key is not None:
            enc = tmp / f'{name}-enc.bin'
            espsecure(
                'encrypt-flash-data', '--aes-xts', '--keyfile', fe_key, '--address', hex(offset), '--output', enc, img
            )
            img = enc
        out.append((offset, img))
    return out


def write_flash(port, images, flash_size, secure_download, force):
    # keep: rewriting the header of a signed or encrypted image corrupts it.
    after = 'hard-reset' if secure_download else 'no-reset'
    argv = py_tool('esptool') + ['--chip', CHIP, '-p', port, '--before', 'default-reset', '--after', after]
    if secure_download:
        argv.append('--no-stub')
    argv += ['write-flash', '--flash-mode', 'keep', '--flash-freq', 'keep', '--flash-size', flash_size]
    if force:
        # esptool cannot tell the images are already encrypted for this unit.
        argv.append('--force')
    for offset, img in images:
        argv += [hex(offset), img]
    run(argv)


def efuse_value(summary, name):
    return summary[name]['value']


def efuse_raw(summary, name):
    """SPI_BOOT_CRYPT_CNT reads back 'Enable' for both 1 and 7."""
    raw = summary[name]['raw_value']
    return int(raw, 0) if isinstance(raw, str) else int(raw)


def mac_from_usb(port):
    """USB-Serial-JTAG reports the MAC as its serial number."""
    try:
        props = subprocess.run(
            ['udevadm', 'info', '-q', 'property', '-n', port], capture_output=True, text=True, timeout=10
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    m = re.search(r'^ID_SERIAL_SHORT=([0-9A-Fa-f:]{12,17})$', props.stdout, re.M)
    return normalize_mac(m.group(1)) if m else None


def normalize_mac(s):
    h = re.sub(r'[^0-9a-fA-F]', '', s or '').lower()
    if len(h) != 12:
        raise ProvisionError(f'not a MAC address: {s!r}')
    return ':'.join(h[i : i + 2] for i in range(0, 12, 2))


def unit_mac(args):
    mac = normalize_mac(args.mac) if args.mac else mac_from_usb(args.port)
    if mac is None:
        raise ProvisionError('cannot read the MAC from the USB serial number; pass --mac')
    return mac


def cmd_sign_bootloader(args):
    b = load_build(args.build)
    with secret_dir() as sd:
        primary = write_secret(sd / 'primary.pem', read_secret(ENV_SIGNING_KEY))
        backup = write_secret(sd / 'backup.pem', read_secret(ENV_BACKUP_KEY))
        once = sd / 'bootloader-signed1.bin'
        espsecure('sign-data', '--version', '2', '--keyfile', primary, '--output', once, b['bootloader'][1])
        espsecure(
            'sign-data', '--version', '2', '--keyfile', backup, '--append-signatures', '--output', args.output, once
        )
        for key in (primary, backup):
            espsecure('verify-signature', '--version', '2', '--keyfile', key, args.output)
        digests = check_signed_bootloader(b, args.output, primary, sd)
    print(f'\nsigned: {args.output}')
    for name, d in zip(('primary', 'backup'), digests):
        print(f'  {name} key digest {d.hex()}')


def cmd_provision(args):
    b = load_build(args.build)
    fe = b['profile'] == 'secure-fe'
    mac = unit_mac(args)
    udir = records_dir() / ('virt' if args.virt else '') / mac.replace(':', '')
    udir.mkdir(parents=True, exist_ok=True, mode=0o700)
    for d in {records_dir(), udir.parent}:
        d.chmod(0o700)
    ef = Efuse(virt_file=udir / 'virt-efuse.json') if args.virt else Efuse(port=args.port)

    try:
        s = ef.summary()
    except ProvisionError as e:
        raise ProvisionError(f'{e}\nA provisioned unit (Secure Download Mode) blocks espefuse: use `flash`.') from e
    if efuse_value(s, 'ENABLE_SECURITY_DOWNLOAD'):
        raise ProvisionError('unit is already provisioned (Secure Download Mode is on); use `flash`')
    record_path = udir / 'record.json'
    record = load_record(record_path)
    burned = any(efuse_value(s, f'KEY_PURPOSE_{i}') != 'USER' for i in range(6))
    if burned and (record is None or record.get('state') != 'in-progress'):
        raise ProvisionError(f'eFuse keys are already set and there is no in-progress record for {mac}')
    if not burned:
        record = None  # nothing irreversible happened yet: start over

    with secret_dir() as sd:
        primary = write_secret(sd / 'primary.pem', read_secret(ENV_SIGNING_KEY))
        digests = check_signed_bootloader(b, args.bootloader, primary, sd)
        fe_raw = derive_fe_key(read_secret(ENV_FE_MASTER), mac) if fe else None
        fe_key_id = check_record(record, b['profile'], fe_raw)
        fe_key = write_secret(sd / 'fe.bin', fe_raw) if fe else None

        print(f'\nProvisioning {mac} as {b["profile"]} burns eFuses permanently (docs/SECURITY_PROFILES.md).')
        print(f'Key digests: primary {digests[0].hex()}\n             backup  {digests[1].hex()}')
        if not args.virt and not args.yes:
            if input(f'\nType the MAC ({mac}) to continue: ').strip().lower() != mac:
                raise ProvisionError('confirmation did not match; nothing was burned')

        record = record or {'mac': mac, 'profile': b['profile'], 'started': now()}
        record.update(state='in-progress', key_digests=[d.hex() for d in digests], fe_key_id=fe_key_id)
        save_record(record_path, record)

        if fe and efuse_value(s, 'KEY_PURPOSE_0') != 'XTS_AES_128_KEY':
            ef.burn('burn-key', FE_KEY_BLOCK, fe_key, 'XTS_AES_128_KEY')

        if efuse_value(s, f'KEY_PURPOSE_{HMAC_KEY_ID}') != 'HMAC_UP':
            ef.burn('burn-key', HMAC_KEY_BLOCK, write_secret(sd / 'hmac.bin', secrets.token_bytes(32)), 'HMAC_UP')
            (sd / 'hmac.bin').unlink()

        if efuse_value(s, 'KEY_PURPOSE_1') == 'SECURE_BOOT_DIGEST0':
            for block, digest in zip(SB_DIGEST_BLOCKS, digests):
                if str(efuse_value(s, block)).replace(' ', '') != digest.hex():
                    raise ProvisionError(f'{block} holds another Secure Boot key digest than {args.bootloader}')
        else:
            d0, d1 = write_secret(sd / 'd0.bin', digests[0]), write_secret(sd / 'd1.bin', digests[1])
            ef.burn(
                'burn-key',
                SB_DIGEST_BLOCKS[0],
                d0,
                'SECURE_BOOT_DIGEST0',
                SB_DIGEST_BLOCKS[1],
                d1,
                'SECURE_BOOT_DIGEST1',
            )
        if not efuse_value(s, 'SECURE_BOOT_KEY_REVOKE2'):
            ef.burn('burn-efuse', 'SECURE_BOOT_KEY_REVOKE2', '1')

        if not efuse_value(s, 'SECURE_BOOT_EN'):
            names = ('bootloader', 'partition-table', 'otadata', 'app')
            images = prepare_images(b, primary, fe_key, names, sd, bootloader=args.bootloader)
            if args.virt:
                print('  (virt: skipping erase + write of ' + ', '.join(hex(o) for o, _ in images) + ')')
            else:
                # --force: a resumed run may already have flash encryption enabled.
                run(
                    py_tool('esptool')
                    + ['--chip', CHIP, '-p', args.port, '--after', 'no-reset', 'erase-flash', '--force']
                )
                write_flash(args.port, images, b['flash_size'], secure_download=False, force=fe)

    s = ef.summary()
    todo = []
    wanted = ((('SPI_BOOT_CRYPT_CNT', 7),) + FE_SECURITY_EFUSES if fe else ()) + SB_SECURITY_EFUSES
    for name, val in wanted:
        if efuse_raw(s, name) != val:
            todo += [name, str(val)]
    if todo:
        ef.burn('burn-efuse', *todo)
    if ef.summary()['DIS_ICACHE']['writeable']:
        ef.burn('write-protect-efuse', 'DIS_ICACHE')
    if not efuse_value(s, 'SECURE_BOOT_EN'):
        ef.burn('burn-efuse', 'SECURE_BOOT_EN', '1')
    s = ef.summary()
    if s['RD_DIS']['writeable']:
        ef.burn('write-protect-efuse', 'RD_DIS')
        s = ef.summary()
    # Last possible full eFuse read: Secure Download Mode blocks espefuse.
    (udir / 'efuse-summary.json').write_text(json.dumps(s, indent=1))
    ef.burn('burn-efuse', 'ENABLE_SECURITY_DOWNLOAD', '1')

    record.update(state='done', finished=now(), app=str(b['app'][1]))
    save_record(record_path, record)
    print(f'\nProvisioned {mac} ({b["profile"]}). Record: {record_path}')
    if not args.virt:
        print('Power-cycle the unit, then enroll it with a one-off Tailscale key.')


def cmd_flash(args):
    b = load_build(args.build)
    fe = b['profile'] == 'secure-fe'
    mac = unit_mac(args)
    if args.full and not args.bootloader:
        raise ProvisionError('--full needs --bootloader (the output of sign-bootloader)')
    names = ('bootloader', 'partition-table', 'otadata', 'app') if args.full else ('otadata', 'app')
    record = load_record(records_dir() / mac.replace(':', '') / 'record.json')
    with secret_dir() as sd:
        primary = write_secret(sd / 'primary.pem', read_secret(ENV_SIGNING_KEY))
        if args.full:
            check_signed_bootloader(b, args.bootloader, primary, sd)
        fe_raw = derive_fe_key(read_secret(ENV_FE_MASTER), mac) if fe else None
        check_record(record, b['profile'], fe_raw)
        fe_key = write_secret(sd / 'fe.bin', fe_raw) if fe else None
        images = prepare_images(b, primary, fe_key, names, sd, bootloader=args.bootloader)
        write_flash(args.port, images, b['flash_size'], secure_download=True, force=True)
    print('\nDone. The unit reboots into the new image (ota_0).')


def cmd_sign_ota(args):
    b = load_build(args.build)
    with secret_dir() as sd:
        primary = write_secret(sd / 'primary.pem', read_secret(ENV_SIGNING_KEY))
        espsecure('sign-data', '--version', '2', '--keyfile', primary, '--output', args.output, b['app'][1])
    print(f'signed: {args.output}  (POST it to /admin/api/ota; the unit encrypts it as it writes)')


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def save_record(path, record):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(record, indent=1))
    tmp.replace(path)  # atomic: an interrupted write never leaves a truncated record


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('sign-bootloader', help='sign the bootloader with the primary and backup keys')
    p.add_argument('build')
    p.add_argument('-o', '--output', required=True)
    p = sub.add_parser('provision', help='one-way: burn eFuses and flash a blank unit')
    p.add_argument('build')
    p.add_argument('-p', '--port')
    p.add_argument('--bootloader', required=True, help='output of sign-bootloader for this build')
    p.add_argument('--mac', help='unit MAC (default: read from the USB serial number)')
    p.add_argument('--yes', action='store_true', help='skip the typed-MAC confirmation (unattended stations)')
    p.add_argument('--virt', action='store_true', help='rehearse on a virtual eFuse file; flashes nothing')
    p = sub.add_parser('flash', help='re-flash a provisioned unit over USB (Secure Download Mode)')
    p.add_argument('build')
    p.add_argument('-p', '--port', required=True)
    p.add_argument('--mac', help='unit MAC (default: read from the USB serial number)')
    p.add_argument('--full', action='store_true', help='also write the bootloader and partition table')
    p.add_argument('--bootloader', help='output of sign-bootloader, with --full')
    p = sub.add_parser('sign-ota', help='sign the app image for /admin/api/ota')
    p.add_argument('build')
    p.add_argument('-o', '--output', required=True)
    args = ap.parse_args(argv)
    if args.cmd == 'provision' and args.virt and not args.mac:
        ap.error('provision --virt needs --mac')
    if args.cmd == 'provision' and not args.virt and not args.port:
        ap.error('provision needs -p PORT (or --virt)')
    try:
        {
            'sign-bootloader': cmd_sign_bootloader,
            'provision': cmd_provision,
            'flash': cmd_flash,
            'sign-ota': cmd_sign_ota,
        }[args.cmd](args)
    except ProvisionError as e:
        print(f'\nERROR: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
