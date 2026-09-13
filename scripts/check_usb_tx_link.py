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
    ):
        output = disassemble(caller)
        if not re.search(r'<' + re.escape(target) + r'(?:\+0x[0-9a-f]+)?>', output):
            raise SystemExit(f'USB TX link check failed: {caller} does not reference {target}')

    def reaches_defer(symbol, visited):
        if symbol in visited:
            return False
        visited.add(symbol)
        output = disassemble(symbol)
        if re.search(r'<tud_defer_func_try(?:\+0x[0-9a-f]+)?>', output):
            return True
        # Producer and timer share usb_kick. GCC may inline it in the producer
        # or outline a .part/.constprop clone; follow only this helper family.
        helpers = re.findall(r'<(usb_kick(?:\.[A-Za-z0-9_.]+)?)(?:\+0x[0-9a-f]+)?>', output)
        return any(reaches_defer(helper, visited) for helper in helpers)

    for entry in ('ml_usb_tx_send', 'usb_kick'):
        if not reaches_defer(entry, set()):
            raise SystemExit(f'USB TX link check failed: {entry} has no verified nonblocking kick path')
    print('USB TX link check: NCM wrapper, legacy forwarding and nonblocking defer verified')


if __name__ == '__main__':
    main()
