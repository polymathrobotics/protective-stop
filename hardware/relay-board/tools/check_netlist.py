#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Verify the relay-board schematic netlist against the intended design.

Exports the netlist with kicad-cli (KiCad's own connectivity engine, independent
of whatever drew the schematic) and checks:

  1. every net contains exactly the intended pins and no other nets exist
     (nets are matched by pin membership; wired nets get auto-generated names);
  2. nets that carry a label keep the intended name;
  3. the safety rules in ../DESIGN.md section 4 hold.

Usage:
  check_netlist.py [--kicad-cli PATH]             exit status 0 = pass
  check_netlist.py [--kicad-cli PATH] --selftest  also prove the checks catch faults
"""

import argparse
import copy
import pathlib
import re
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
SCH = HERE.parent / 'relay-board.kicad_sch'

# One entry per channel; the two channels are identical by construction.
CHANNELS = {
    'A': dict(
        k='K1',
        q='Q1',
        d='D1',
        rg='R1',
        rp='R3',
        ru='R5',
        rs='R7',
        tp_gate='TP4',
        tp_coil='TP6',
        tp_mirror='TP8',
        jdrv='J1.3',
        jsns='J1.4',
    ),
    'B': dict(
        k='K2',
        q='Q2',
        d='D2',
        rg='R2',
        rp='R4',
        ru='R6',
        rs='R8',
        tp_gate='TP5',
        tp_coil='TP7',
        tp_mirror='TP9',
        jdrv='J1.5',
        jsns='J1.6',
    ),
}

# Nets that carry a schematic label, so their name is checked too.
LABELLED = {'+5V', '+3V3', 'GND', 'DRV_A', 'DRV_B', 'SNS_A', 'SNS_B', 'LOOP_IN', 'LOOP_MID', 'LOOP_OUT'}


def expected_nets():
    # Power flags (#FLG..) are schematic-only and not part of KiCad's netlist export.
    nets = {
        '+5V': {'J1.1', 'K1.A1', 'K2.A1', 'C1.1', 'TP1.1'},
        '+3V3': {'J1.2', 'R5.1', 'R6.1', 'TP2.1'},
        'GND': {'J1.7', 'R3.2', 'R4.2', 'Q1.2', 'Q2.2', 'D1.2', 'D2.2', 'K1.11', 'K2.11', 'C1.2', 'TP3.1'},
        'LOOP_IN': {'J2.1', 'K1.34'},
        'LOOP_MID': {'K1.44', 'K2.34', 'TP10.1'},
        'LOOP_OUT': {'K2.44', 'J3.1'},
    }
    for n, c in CHANNELS.items():
        nets[f'DRV_{n}'] = {c['jdrv'], f'{c["rg"]}.1'}
        nets[f'GATE_{n}'] = {f'{c["rg"]}.2', f'{c["rp"]}.1', f'{c["q"]}.1', f'{c["tp_gate"]}.1'}
        nets[f'COIL_{n}_N'] = {f'{c["q"]}.3', f'{c["d"]}.1', f'{c["k"]}.A2', f'{c["tp_coil"]}.1'}
        nets[f'SNS_{n}'] = {c['jsns'], f'{c["rs"]}.2'}
        nets[f'MIRROR_{n}'] = {f'{c["rs"]}.1', f'{c["ru"]}.2', f'{c["k"]}.21', f'{c["tp_mirror"]}.1'}
        nets[f'{c["k"]}_NC_LINK'] = {f'{c["k"]}.12', f'{c["k"]}.22'}
        nets[f'{c["k"]}_NO_LINK'] = {f'{c["k"]}.33', f'{c["k"]}.43'}
    return nets


def tokenize(text):
    return re.findall(r'"(?:[^"\\]|\\.)*"|\(|\)|[^\s()"]+', text)


def parse(text):
    stack, cur = [], []
    for tok in tokenize(text):
        if tok == '(':
            stack.append(cur)
            cur = []
        elif tok == ')':
            done, cur = cur, stack.pop()
            cur.append(done)
        else:
            cur.append(tok.strip('"') if tok.startswith('"') else tok)
    return cur


def find(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def read_netlist(kicad_cli):
    with tempfile.TemporaryDirectory() as tmp:
        out = pathlib.Path(tmp) / 'rb.net'
        subprocess.run(
            [kicad_cli, 'sch', 'export', 'netlist', '--format', 'kicadsexpr', '-o', str(out), str(SCH)],
            check=True,
            capture_output=True,
        )
        tree = parse(out.read_text())[0]
    nets = {}
    for net in find(find(tree, 'nets')[0], 'net'):
        name = find(net, 'name')[0][1]
        nodes = {f'{find(n, "ref")[0][1]}.{find(n, "pin")[0][1]}' for n in find(net, 'node')}
        nets[name] = nodes
    return nets


def check(got):
    """Return a list of problems (empty = pass). `got` maps net name -> set of 'REF.PIN'."""
    want = expected_nets()
    errors = []
    got = {k: set(v) for k, v in got.items() if not k.startswith('unconnected-')}
    got_by_pins = {frozenset(v): k for k, v in got.items()}

    resolved = {}
    for key, pins in want.items():
        name = got_by_pins.get(frozenset(pins))
        if name is None:
            near = [
                f'{n}: missing {sorted(pins - v)} extra {sorted(v - pins)}'
                for n, v in got.items()
                if len(pins & v) >= max(1, len(pins) // 2)
            ]
            errors.append(f'net {key} {sorted(pins)} not found as such' + (f'; closest: {near[0]}' if near else ''))
            resolved[key] = set()
            continue
        resolved[key] = got[name]
        # local labels are prefixed with the sheet path ("/") in KiCad netlists
        if key in LABELLED and name.lstrip('/') != key:
            errors.append(f'net {key} is named {name!r}')
    expected_sets = {frozenset(v) for v in want.values()}
    for name, pins in got.items():
        if frozenset(pins) not in expected_sets:
            errors.append(f'unexpected net {name}: {sorted(pins)}')

    # ---- safety rules (operate on the resolved nets, so they hold whatever the net is called)
    def refs(key):
        return {p.split('.')[0] for p in resolved.get(key, set())}

    loop = ('LOOP_IN', 'LOOP_MID', 'LOOP_OUT', 'K1_NO_LINK', 'K2_NO_LINK')
    allowed = {'K1', 'K2', 'J2', 'J3', 'TP10'}
    for key in loop:
        extra = refs(key) - allowed
        if extra:
            errors.append(f'RULE 1 (nothing bridges the loop contacts): {sorted(extra)} on {key}')
        for pin in resolved.get(key, set()):
            ref, num = pin.split('.')
            if ref in ('K1', 'K2') and num not in ('33', '34', '43', '44'):
                errors.append(f'RULE 2 (loop only on NO contacts): {pin} on {key}')
    for n, c in CHANNELS.items():
        if f'{c["rp"]}.2' not in resolved['GND'] or f'{c["rp"]}.1' not in resolved[f'GATE_{n}']:
            errors.append(f'RULE 3 (gate pull-down) violated on channel {n}')
        if f'{c["k"]}.A1' not in resolved['+5V']:
            errors.append(f'RULE 4 (coil high side on +5V) violated on channel {n}')
        mirror_k = {p for p in resolved[f'MIRROR_{n}'] if p.startswith(c['k'] + '.')}
        if mirror_k != {f'{c["k"]}.21'}:
            errors.append(f'RULE 4 (mirror uses NC contacts 11/12/21/22 only) violated on channel {n}: {mirror_k}')
    shared = {
        key
        for key, pins in resolved.items()
        if any(p.startswith('K1.') for p in pins) and any(p.startswith('K2.') for p in pins)
    }
    if shared - {'+5V', 'GND', 'LOOP_MID'}:
        errors.append(f'RULE 5 (channel independence): shared nets {sorted(shared - {"+5V", "GND", "LOOP_MID"})}')
    return errors


def selftest(base):
    """Inject the faults these checks exist to catch; every one must be reported."""

    def net_of(g, pin):
        return next(k for k, v in g.items() if pin in v)

    def move(g, pin, to_pin_net):
        g[net_of(g, pin)].discard(pin)
        g[net_of(g, to_pin_net)].add(pin)

    cases = {
        'TVS bridging LOOP_IN / LOOP_OUT': lambda g: (
            g[net_of(g, 'J2.1')].add('D9.1'),
            g[net_of(g, 'J3.1')].add('D9.2'),
        ),
        'capacitor from LOOP_MID': lambda g: g[net_of(g, 'TP10.1')].add('C9.1'),
        'loop wired to an NC contact': lambda g: move(g, 'K1.11', 'J2.1'),
        'gate pull-down removed': lambda g: g[net_of(g, 'R3.1')].discard('R3.1'),
        'mirror uses a NO contact': lambda g: move(g, 'K1.21', 'K1.33'),
        'channels share a drive net': lambda g: g[net_of(g, 'J1.5')].add('R1.1'),
        'coil A2 shorted to +5V': lambda g: move(g, 'K1.A2', 'J1.1'),
        'extra unexpected net': lambda g: g.__setitem__('FOO', {'R1.1'}),
        'LOOP_MID label renamed': lambda g: g.__setitem__('LOOP_X', g.pop(net_of(g, 'TP10.1'))),
    }
    bad = 0
    for name, fn in cases.items():
        g = copy.deepcopy(base)
        fn(g)
        errs = check(g)
        print(('  caught  ' if errs else '  MISSED  ') + name)
        bad += 0 if errs else 1
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kicad-cli', default='kicad-cli')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    got = read_netlist(args.kicad_cli)
    errors = check(got)
    if errors:
        print('FAIL')
        for e in errors:
            print('  -', e)
        return 1
    print(f'PASS: {len(got)} nets match the intended design; safety rules 1-5 hold')
    if args.selftest:
        print('self-test (injected faults):')
        if selftest(got):
            print('SELF-TEST FAILED: a fault was not detected')
            return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
