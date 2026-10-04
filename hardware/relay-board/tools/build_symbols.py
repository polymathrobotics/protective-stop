#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Build the project-local symbol libraries and the SR4D4005 footprint.

Copies the stock KiCad symbols this design uses into relay-board.kicad_sym and
power.kicad_sym (so the project is self-contained), adds the five-unit SR4D4005
relay symbol, and writes relay-board.pretty/SR4D4005.kicad_mod plus the
sym-lib-table / fp-lib-table.

KiCad 10 keeps stock symbols one-per-file in *.kicad_symdir folders. Point
KICAD_SYMBOL_LIBS at that "symbols" folder (see extract_kicad_libs.sh); default
~/Applications/kicad10-libs/symbols. Afterwards run
`kicad-cli sym upgrade --force relay-board.kicad_sym` to canonicalise formatting
(done automatically if KICAD_CLI is set).
"""

import os
import pathlib
import subprocess

LIBROOT = pathlib.Path(os.environ.get('KICAD_SYMBOL_LIBS', pathlib.Path.home() / 'Applications/kicad10-libs/symbols'))
OUT = pathlib.Path(__file__).resolve().parents[1]

WANT = [
    ('Device', 'R'),
    ('Device', 'C'),
    ('Device', 'D_Zener'),
    ('Transistor_FET', 'Q_NMOS_GSD'),
    ('Connector', 'TestPoint'),
    ('Connector_Generic', 'Conn_01x07'),
    ('Connector_Generic', 'Conn_01x01'),
    ('Mechanical', 'MountingHole'),
]

POWER = ['+5V', '+3V3', 'GND', 'PWR_FLAG']


def extract_symbol(text: str, name: str) -> str:
    start = text.index(f'(symbol "{name}"')
    depth, i = 0, start
    while True:
        c = text[i]
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return text[start : i + 1]
        i += 1


def relay_symbol() -> str:
    """SR4D4005 as five units (IEC split-symbol convention):
    1 coil A1/A2, 2 NC 11-12, 3 NC 21-22, 4 NO 33-34, 5 NO 43-44."""

    def pin(num, y, ang):
        return (
            f'\t\t\t(pin passive line (at 0 {y} {ang}) (length 2.54) '
            f'(name "{num}" (effects (font (size 1.27 1.27)))) '
            f'(number "{num}" (effects (font (size 1.27 1.27)))))'
        )

    def line(*pts):
        xy = ' '.join(f'(xy {x} {y})' for x, y in pts)
        return f'\t\t\t(polyline (pts {xy}) (stroke (width 0.254) (type default)) (fill (type none)))'

    coil = '\n'.join([
        '\t\t\t(rectangle (start -2.54 5.08) (end 2.54 -5.08) (stroke (width 0.254) (type default)) (fill (type background)))',
        pin('A1', 7.62, 270),
        pin('A2', -7.62, 90),
    ])

    def contact(top, bottom, nc):
        g = [line((0, 5.08), (0, 2.54)), line((0, 2.54), (-2.54, -2.54)), line((0, -5.08), (0, -2.54))]
        if nc:
            g += [line((0, -2.54), (-2.54, -2.54)), line((-2.075, 0.4), (-0.465, -0.4))]
        else:
            g += [line((-0.635, -2.54), (0.635, -2.54))]
        return '\n'.join(g + [pin(top, 7.62, 270), pin(bottom, -7.62, 90)])

    units = [
        (1, coil),
        (2, contact('12', '11', True)),
        (3, contact('21', '22', True)),
        (4, contact('34', '33', False)),
        (5, contact('43', '44', False)),
    ]
    body = '\n'.join(f'\t\t(symbol "SR4D4005_{u}_1"\n{g}\n\t\t)' for u, g in units)
    return f"""(symbol "SR4D4005"
\t\t(pin_names (offset 1.016) (hide yes))
\t\t(exclude_from_sim no)
\t\t(in_bom yes)
\t\t(on_board yes)
\t\t(property "Reference" "K" (at 0 10.16 0) (effects (font (size 1.27 1.27))))
\t\t(property "Value" "SR4D4005" (at 0 -10.16 0) (effects (font (size 1.27 1.27))))
\t\t(property "Footprint" "relay-board:SR4D4005" (at 0 -12.7 0) (effects (font (size 1.27 1.27)) hide))
\t\t(property "Datasheet" "https://www.te.com/en/product-7-1415054-1.html" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
\t\t(property "Description" "TE SCHRACK SR4 force-guided safety relay, 2 NO + 2 NC, 5 V coil, EN 61810-3 type A. Units: 1 coil, 2 NC 11-12, 3 NC 21-22, 4 NO 33-34, 5 NO 43-44" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
\t\t(property "MPN" "SR4D4005" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
\t\t(property "Manufacturer" "TE Connectivity" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
\t\t(property "LCSC" "C1525104" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
{body}
\t\t(embedded_fonts no)
\t)"""


def main():
    blocks = []
    for lib, name in WANT:
        p = LIBROOT / f'{lib}.kicad_symdir' / f'{name}.kicad_sym'
        blocks.append(extract_symbol(p.read_text(), name))
    blocks.append(relay_symbol())
    body = '\n\t'.join(blocks)
    lib = (
        '(kicad_symbol_lib\n\t(version 20251024)\n\t(generator "relay-board-lib")\n'
        f'\t(generator_version "10.0")\n\t{body}\n)\n'
    )
    (OUT / 'relay-board.kicad_sym').write_text(lib)
    pblocks = [extract_symbol((LIBROOT / 'power.kicad_symdir' / f'{n}.kicad_sym').read_text(), n) for n in POWER]
    pbody = '\n\t'.join(pblocks)
    (OUT / 'power.kicad_sym').write_text(
        '(kicad_symbol_lib\n\t(version 20251024)\n\t(generator "relay-board-lib")\n'
        f'\t(generator_version "10.0")\n\t{pbody}\n)\n'
    )

    # Footprint: THT, 40 x 13 body, pins per TE drawing S0413-BC (bottom view),
    # mirrored in X for the top view. Rows 7.6 mm apart; body centred on origin.
    pads = [
        ('A1', 16.6, -3.8),
        ('A2', 16.6, 3.8),
        ('21', -1.45, -3.8),
        ('11', -1.45, 3.8),
        ('22', -6.45, -3.8),
        ('12', -6.45, 3.8),
        ('44', -12.95, -3.8),
        ('34', -12.95, 3.8),
        ('43', -17.95, -3.8),
        ('33', -17.95, 3.8),
    ]
    pad_lines = '\n'.join(
        f'\t(pad "{n}" thru_hole circle (at {x} {y}) (size 2.4 2.4) (drill 1.4) (layers "*.Cu" "*.Mask"))'
        for n, x, y in pads
    )
    fp = f"""(footprint "SR4D4005"
\t(version 20260206)
\t(generator "relay-board-lib")
\t(generator_version "10.0")
\t(layer "F.Cu")
\t(descr "TE SCHRACK SR4 force-guided relay 2NO+2NC, 40x13x16.5 mm, THT. Pins per TE drawing S0413-BC (bottom view), mirrored for top view.")
\t(tags "relay force guided safety SR4")
\t(property "Reference" "REF**" (at 0 -8.5 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
\t(property "Value" "SR4D4005" (at 0 8.5 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))
\t(attr through_hole)
\t(fp_rect (start -20 -6.5) (end 20 6.5) (stroke (width 0.12) (type solid)) (fill no) (layer "F.SilkS"))
\t(fp_rect (start -20 -6.5) (end 20 6.5) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab"))
\t(fp_rect (start -20.5 -7) (end 20.5 7) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))
\t(fp_circle (center 16.6 -6.9) (end 17.1 -6.9) (stroke (width 0.12) (type solid)) (fill yes) (layer "F.SilkS"))
\t(fp_text user "A1" (at 16.6 -5.2 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))
{pad_lines}
)
"""
    pretty = OUT / 'relay-board.pretty'
    pretty.mkdir(exist_ok=True)
    (pretty / 'SR4D4005.kicad_mod').write_text(fp)

    (OUT / 'sym-lib-table').write_text(
        '(sym_lib_table\n\t(version 7)\n\t(lib (name "relay-board")(type "KiCad")'
        '(uri "${KIPRJMOD}/relay-board.kicad_sym")(options "")(descr "relay-board project symbols"))\n\t(lib (name "power")(type "KiCad")(uri "${KIPRJMOD}/power.kicad_sym")(options "")(descr "project power symbols"))\n)\n'
    )
    (OUT / 'fp-lib-table').write_text(
        '(fp_lib_table\n\t(version 7)\n\t(lib (name "relay-board")(type "KiCad")'
        '(uri "${KIPRJMOD}/relay-board.pretty")(options "")(descr "relay-board project footprints"))\n)\n'
    )
    print('ok', len(blocks), 'symbols')


main()
if os.environ.get('KICAD_CLI'):
    subprocess.run(
        [os.environ['KICAD_CLI'], 'sym', 'upgrade', '--force', str(OUT / 'relay-board.kicad_sym')], check=True
    )
