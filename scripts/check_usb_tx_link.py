#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Fail the build if NCM's final ELF calls bypass the owned-request callback."""

import argparse
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objdump', required=True)
    parser.add_argument('--elf', required=True)
    args = parser.parse_args()

    def disassemble(symbol):
        return subprocess.check_output([args.objdump, '-d', '--disassemble=' + symbol, args.elf], text=True)

    for caller, target in (
        ('tud_network_xmit', '__wrap_tud_network_xmit_cb'),
        ('__wrap_tud_network_xmit_cb', 'tud_network_xmit_cb'),
        ('ml_usb_tx_send', 'tud_defer_func_try'),
    ):
        output = disassemble(caller)
        if not re.search(r'<' + re.escape(target) + r'(?:\+0x[0-9a-f]+)?>', output):
            raise SystemExit(f'USB TX link check failed: {caller} does not reference {target}')
    print('USB TX link check: NCM wrapper, legacy forwarding and nonblocking defer verified')


if __name__ == '__main__':
    main()
