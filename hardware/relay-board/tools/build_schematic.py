#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Generate the relay-board schematic: explicit placement, real wires, deterministic UUIDs.

Provenance / reproducibility tool. Once the schematic exists, relay-board.kicad_sch
is the source of truth: edit it in KiCad, not here. Running this overwrites it.
Needs relay-board.kicad_sym and power.kicad_sym (see build_symbols.py). Re-running
reproduces the committed file exactly (after the pre-commit whitespace fixer).
"""

import math
import pathlib
import re
import uuid

PROJ = pathlib.Path(__file__).resolve().parents[1]
OUT = PROJ / 'relay-board.kicad_sch'
G = 1.27
NS = uuid.UUID('6c1f0a52-0000-4000-8000-00000000abcd')
ROOT = str(uuid.uuid5(NS, 'root'))


def g(n):
    return round(n * G, 3)


def uid(*k):
    return str(uuid.uuid5(NS, '/'.join(map(str, k))))


# ---------------------------------------------------------------- library
def block(text, start):
    d, i = 0, start
    while True:
        c = text[i]
        if c == '(':
            d += 1
        elif c == ')':
            d -= 1
            if d == 0:
                return text[start : i + 1]
        i += 1


def load_lib(path):
    t = path.read_text()
    syms = {}
    for m in re.finditer(r'\n\t\(symbol "([^"]+)"', t):
        syms[m.group(1)] = block(t, m.start() + 1)
    return syms


LIBS = {'relay-board': load_lib(PROJ / 'relay-board.kicad_sym'), 'power': load_lib(PROJ / 'power.kicad_sym')}


def unit_pins(symtext, unit):
    pins = {}
    for m in re.finditer(r'\(symbol "[^"]+_(\d+)_\d+"', symtext):
        u = int(m.group(1))
        if u not in (0, unit):
            continue
        sub = block(symtext, m.start())
        for p in re.finditer(r'\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\).*?\(number "([^"]+)"', sub, re.S):
            pins[p.group(4)] = (float(p.group(1)), float(p.group(2)))
    return pins


def xform(px, py, x0, y0, rot, mirror):
    a = math.radians(-rot)
    x, y = px, -py
    rx = x * math.cos(a) - y * math.sin(a)
    ry = x * math.sin(a) + y * math.cos(a)
    if mirror == 'x':
        ry = -ry
    elif mirror == 'y':
        rx = -rx
    return round(x0 + rx, 3), round(y0 + ry, 3)


# ---------------------------------------------------------------- output
parts, wires, juncs, labels, texts, rects, used_libs = [], [], [], [], [], [], {}
pwr_n = [0]
stroke_loop = 0.35


def place(
    lib,
    name,
    ref,
    x,
    y,
    rot=0,
    mirror='',
    unit=1,
    value=None,
    props=None,
    ref_at=None,
    val_at=None,
    show_val=True,
    footprint=None,
    fang=0,
):
    key = f'{lib}:{name}'
    used_libs[key] = LIBS[lib][name]
    pins = {n: xform(px, py, x, y, rot, mirror) for n, (px, py) in unit_pins(LIBS[lib][name], unit).items()}
    parts.append(
        dict(
            lib=key,
            ref=ref,
            x=x,
            y=y,
            rot=rot,
            mirror=mirror,
            unit=unit,
            value=value if value is not None else name,
            props=props or {},
            ref_at=ref_at,
            val_at=val_at,
            show_val=show_val,
            footprint=footprint,
            fang=fang,
        )
    )
    return pins


def power(name, x, y):
    pwr_n[0] += 1
    used_libs[f'power:{name}'] = LIBS['power'][name]
    parts.append(
        dict(
            lib=f'power:{name}',
            ref=f'#PWR{pwr_n[0]:03d}',
            x=x,
            y=y,
            rot=0,
            mirror='',
            unit=1,
            value=name,
            props={},
            ref_at=None,
            val_at=('power',),
            show_val=True,
            footprint=None,
            power=True,
        )
    )


def flag(x, y):
    pwr_n[0] += 1
    used_libs['power:PWR_FLAG'] = LIBS['power']['PWR_FLAG']
    parts.append(
        dict(
            lib='power:PWR_FLAG',
            ref=f'#FLG{pwr_n[0]:03d}',
            x=x,
            y=y,
            rot=0,
            mirror='',
            unit=1,
            value='PWR_FLAG',
            props={},
            ref_at=None,
            val_at=('power',),
            show_val=True,
            footprint=None,
            power=True,
        )
    )


def wire(*pts, w=0):
    for a, b in zip(pts, pts[1:]):
        assert abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6, f'non-manhattan wire {a}->{b}'
        wires.append((a, b, w))


def junction(x, y):
    juncs.append((x, y))


def label(name, x, y, ang=0):
    labels.append((name, x, y, ang))


def text(s, x, y, size=1.27, bold=False, ital=False, just='left top'):
    texts.append((s, x, y, size, bold, ital, just))


def frame(x1, y1, x2, y2, title, dash=True):
    rects.append((x1, y1, x2, y2, dash))
    text(title, x1 + 2.54, y1 + 2.54, size=2.0, bold=True)


# ================================================================= LAYOUT
# ---------------- J1: ESP32-S3-ETH interface (left)
frame(g(8), g(14), g(64), g(112), 'J1  ESP32-S3-ETH wire pads')
J1x, J1y = g(44), g(38)
j1 = place(
    'relay-board',
    'Conn_01x07',
    'J1',
    J1x,
    J1y,
    mirror='y',
    value='ESP32-S3-ETH',
    ref_at=(J1x + 2 * G, J1y - 11 * G),
    val_at=(J1x + 2 * G, J1y + 11 * G),
    props={'Note': 'Solder wire pads, no header fitted. Wire to the ESP32-S3-ETH pins listed on the sheet.'},
)
# pin numbers: 1 +5V, 2 +3V3, 3 DRV_A, 4 SNS_A, 5 DRV_B, 6 SNS_B, 7 GND
x1 = j1['1'][0]
wire(j1['1'], (g(52), j1['1'][1]), (g(52), g(22)))
power('+5V', g(52), g(22))
junction(g(52), g(27))
wire((g(52), g(27)), (g(49), g(27)))
flag(g(49), g(27))
wire(j1['2'], (g(58), j1['2'][1]), (g(58), g(22)))
power('+3V3', g(58), g(22))
junction(g(58), g(26))
wire((g(58), g(26)), (g(62), g(26)))
flag(g(62), g(26))
for n, nm in (('3', 'DRV_A'), ('4', 'SNS_A'), ('5', 'DRV_B'), ('6', 'SNS_B')):
    px, py = j1[n]
    wire((px, py), (px + 3 * G, py))
    label(nm, px + 3 * G, py, 0)
wire(j1['7'], (g(52), j1['7'][1]), (g(52), g(52)))
power('GND', g(52), g(52))
junction(g(52), g(48))
wire((g(52), g(48)), (g(56), g(48)))
flag(g(56), g(48))
text(
    'PIN  ESP32-S3-ETH\n1   VBUS   (left hdr pin 40)\n2   3V3    (left hdr pin 36)\n3   IO39   (right hdr pin 12)\n4   IO40   (pin 11)\n5   IO41   (pin 10)\n6   IO42   (pin 9)\n7   GND    (pin 8 or 13)',
    g(11),
    g(58),
    size=1.3,
)
# decoupling
c1 = place(
    'relay-board',
    'C',
    'C1',
    g(30),
    g(96),
    value='22uF',
    ref_at=(g(32.5), g(94.5)),
    val_at=(g(32.5), g(97.5)),
    props={'LCSC': 'C45783', 'MPN': 'CL21A226MAQNNNE', 'Manufacturer': 'Samsung'},
)
power('+5V', g(30), g(88))
wire(c1['1'], (g(30), g(88)))
power('GND', g(30), g(107))
wire(c1['2'], (g(30), g(107)))
text('+5V bulk\n(both coils)', g(33), g(101), size=1.3)

# rail test points
text('TEST POINTS', g(38), g(73), size=1.3, bold=True)
for i, (rf, nm) in enumerate((('TP1', '+5V'), ('TP2', '+3V3'), ('TP3', 'GND'))):
    tx = g(40 + 8 * i)
    if nm == 'GND':
        t_ = place('relay-board', 'TestPoint', rf, tx, g(80), value=nm, ref_at=(tx, g(77)), show_val=False)
        wire((tx, g(80)), (tx, g(86)))
        power('GND', tx, g(86))
    else:
        power(nm, tx, g(78))
        t_ = place('relay-board', 'TestPoint', rf, tx, g(84), rot=180, value=nm, ref_at=(tx, g(88)), show_val=False)
        wire((tx, g(78)), (tx, g(84)))


# ---------------- channel lanes
def lane(ch, ly):
    """ch = 'A' or 'B'; ly = top of lane frame (mm). Returns nothing; adds everything."""
    k = 'K1' if ch == 'A' else 'K2'
    q, d, rg, rp, ru, rs = ('Q1', 'D1', 'R1', 'R3', 'R5', 'R7') if ch == 'A' else ('Q2', 'D2', 'R2', 'R4', 'R6', 'R8')
    tpg, tpc, tpm = ('TP4', 'TP6', 'TP8') if ch == 'A' else ('TP5', 'TP7', 'TP9')
    core = 'core 0' if ch == 'A' else 'core 1'
    frame(g(70), ly, g(224), ly + g(78), f'CHANNEL {ch}  ({core}):  coil drive  +  contact mirror')
    y0 = ly + g(10)
    text('COIL DRIVE  (fail-off: gate pulled down)', g(72), ly + g(6), size=1.4, bold=True)
    # gate path
    gy = y0 + g(30)
    label(f'DRV_{ch}', g(72), gy, 0)
    r_g = place(
        'relay-board',
        'R',
        rg,
        g(82),
        gy,
        rot=90,
        value='100',
        ref_at=(g(82), gy - 3 * G),
        val_at=(g(82), gy + 3.4 * G),
        props={'LCSC': 'C22775', 'MPN': '0603WAF1000T5E', 'Manufacturer': 'UNI-ROYAL'},
    )
    wire((g(72), gy), r_g['1'])
    gate_x = g(91)
    wire(r_g['2'], (gate_x, gy))
    qx = g(99)
    qp = place(
        'relay-board',
        'Q_NMOS_GSD',
        q,
        qx,
        gy,
        value='AO3400A',
        ref_at=(qx + 4 * G, gy + 3.5 * G),
        val_at=(qx + 4 * G, gy + 6 * G),
        props={'LCSC': 'C20917', 'MPN': 'AO3400A', 'Manufacturer': 'Alpha & Omega'},
    )
    wire((gate_x, gy), qp['G'] if 'G' in qp else qp['1'])
    # gate pull-down
    r_p = place(
        'relay-board',
        'R',
        rp,
        gate_x,
        gy + g(8),
        value='10k',
        ref_at=(gate_x + 2 * G, gy + g(8) - 1.2 * G),
        val_at=(gate_x + 2 * G, gy + g(8) + 1.4 * G),
        props={'LCSC': 'C25804', 'MPN': '0603WAF1002T5E', 'Manufacturer': 'UNI-ROYAL'},
    )
    junction(gate_x, gy)
    wire((gate_x, gy), r_p['1'])
    power('GND', gate_x, gy + g(17))
    wire(r_p['2'], (gate_x, gy + g(17)))
    # gate test point
    place(
        'relay-board',
        'TestPoint',
        tpg,
        gate_x,
        gy - g(9),
        value='GATE',
        ref_at=(gate_x - 4 * G, gy - g(9) - 1.5 * G),
        show_val=False,
    )
    wire((gate_x, gy), (gate_x, gy - g(9)))
    # drain / coil
    dx = qp['3'][0]
    dy = qp['3'][1]
    sy = qp['2'][1]
    coil_y = dy - g(6) - 7.62 - 0  # coil centre so that A2 sits g(6) above drain
    kc = place(
        'relay-board',
        'SR4D4005',
        k,
        dx,
        coil_y,
        unit=1,
        value='SR4D4005',
        ref_at=(dx + 4 * G, coil_y - 1.2 * G),
        show_val=False,
    )
    node_y = round((dy + kc['A2'][1]) / 2 / G) * G
    node_y = round(node_y, 3)
    wire(kc['A2'], (dx, node_y))
    junction(dx, node_y)
    wire((dx, node_y), (dx, dy))
    power('+5V', dx, kc['A1'][1] - g(5))
    wire(kc['A1'], (dx, kc['A1'][1] - g(5)))
    power('GND', dx, sy + g(6))
    wire((dx, sy), (dx, sy + g(6)))
    tcx = dx + g(5)
    # clamp diode: cathode up
    ddx = dx + g(15)
    dd = place(
        'relay-board',
        'D_Zener',
        d,
        ddx,
        node_y + 3 * G,
        rot=270,
        value='SMAJ12A',
        ref_at=(ddx + 2 * G, node_y + 1.6 * G),
        val_at=(ddx + 2 * G, node_y + 4.4 * G),
        props={'LCSC': 'C148213', 'MPN': 'SMAJ12A', 'Manufacturer': 'Littelfuse', 'Note': 'coil clamp'},
        fang=90,
    )
    wire((dx, node_y), (tcx, node_y), dd['1'])
    power('GND', ddx, dd['2'][1] + g(5))
    wire(dd['2'], (ddx, dd['2'][1] + g(5)))
    # coil test point (left of node)
    junction(tcx, node_y)
    place(
        'relay-board',
        'TestPoint',
        tpc,
        tcx,
        node_y + g(5),
        rot=180,
        value='COIL',
        ref_at=(tcx + 4 * G, node_y + g(5) + 1.5 * G),
        show_val=False,
    )
    wire((tcx, node_y), (tcx, node_y + g(5)))
    text(
        'SMAJ12A clamps the coil kick at ~20 V\n(fast release; a plain diode is slow)', dx - g(2), gy + g(18), size=1.3
    )
    text(f'{k} coil A1-A2  (SR4D4005, 5 V, 161 mA)', dx + g(4), coil_y + g(2.5), size=1.3)

    # ---- contact mirror block
    mx = g(160)
    text('CONTACT MIRROR  (feedback to MCU)', g(140), ly + g(6), size=1.4, bold=True)
    ry = y0 + g(12)
    ru_p = place(
        'relay-board',
        'R',
        ru,
        mx,
        ry,
        value='330',
        ref_at=(mx + 2 * G, ry - 1.2 * G),
        val_at=(mx + 2 * G, ry + 1.4 * G),
        props={'LCSC': 'C23138', 'MPN': '0603WAF3300T5E', 'Manufacturer': 'UNI-ROYAL'},
    )
    power('+3V3', mx, ry - g(7))
    wire(ru_p['1'], (mx, ry - g(7)))
    nody = ry + g(9)
    wire(ru_p['2'], (mx, nody))
    nc21y = nody + g(6) + 7.62
    n21 = place(
        'relay-board', 'SR4D4005', k, mx, nc21y, unit=3, value='SR4D4005', show_val=False, ref_at=(mx - 5 * G, nc21y)
    )
    wire((mx, nody), n21['21'])
    nc11y = n21['22'][1] + g(5) + 7.62
    n11 = place(
        'relay-board', 'SR4D4005', k, mx, nc11y, unit=2, value='SR4D4005', show_val=False, ref_at=(mx - 5 * G, nc11y)
    )
    wire(n21['22'], n11['12'])
    power('GND', mx, n11['11'][1] + g(5))
    wire(n11['11'], (mx, n11['11'][1] + g(5)))
    # sense output
    rsx = mx + g(11)
    tmx = mx + g(5)
    r_s = place(
        'relay-board',
        'R',
        rs,
        rsx,
        nody,
        rot=90,
        value='100',
        ref_at=(rsx, nody - 3 * G),
        val_at=(rsx, nody + 3.4 * G),
        props={'LCSC': 'C22775', 'MPN': '0603WAF1000T5E', 'Manufacturer': 'UNI-ROYAL'},
    )
    junction(mx, nody)
    wire((mx, nody), (tmx, nody), r_s['1'])
    wire(r_s['2'], (rsx + g(7), nody))
    label(f'SNS_{ch}', rsx + g(7), nody, 0)
    # mirror test point
    junction(tmx, nody)
    place(
        'relay-board',
        'TestPoint',
        tpm,
        tmx,
        nody - g(5),
        value='MIRROR',
        ref_at=(tmx + 4 * G, nody - g(5) - 1.5 * G),
        show_val=False,
    )
    wire((tmx, nody), (tmx, nody - g(5)))
    text(
        f'{k}: NC 21-22 and NC 11-12 in series.\nClosed only while released:\n  SNS = 0  released (safe)\n  SNS = 1  energized / weld',
        mx + g(9),
        nc21y - g(1),
        size=1.3,
    )


lane('A', g(14))
lane('B', g(95))

# ---------------- stop loop (dry contacts) band
ly = g(178)
frame(g(8), ly, g(224), ly + g(44), 'STOP LOOP  (dry contacts, <= 24 V DC, <= 500 mA, inductive)  -  drawn released')
cy = ly + g(20)
j2 = place(
    'relay-board',
    'Conn_01x01',
    'J2',
    g(20),
    cy,
    mirror='y',
    value='LOOP IN',
    ref_at=(g(20), cy - 4 * G),
    val_at=(g(20), cy + 4 * G),
    props={'Note': 'Solder wire to Phoenix DFK-MSTB header pin 1'},
)
chain = []
xs = [g(44), g(78), g(120), g(154)]
names = [('K1', 4), ('K1', 5), ('K2', 4), ('K2', 5)]
prev = j2['1']
wire_pts = []
for (kk, u), xx in zip(names, xs):
    p = place(
        'relay-board',
        'SR4D4005',
        kk,
        xx,
        cy,
        rot=90,
        unit=u,
        value='SR4D4005',
        show_val=False,
        ref_at=(xx, cy - 5 * G),
        fang=90,
    )
    chain.append(p)


# unit 4: pins 34 (left) / 33 (right); unit 5: 43 (left) / 44 (right)
def lw(*pts):
    wire(*pts, w=stroke_loop)


lw(j2['1'], chain[0]['34'])
label('LOOP_IN', g(26), cy, 0)
lw(chain[0]['33'], chain[1]['43'])
lw(chain[1]['44'], chain[2]['34'])
lw(chain[2]['33'], chain[3]['43'])
mid_x = (chain[1]['44'][0] + chain[2]['34'][0]) / 2
mid_x = round(mid_x / G) * G
junction(mid_x, cy)
label('LOOP_MID', mid_x - g(3), cy, 0)
tpm10 = place(
    'relay-board',
    'TestPoint',
    'TP10',
    mid_x,
    cy - g(8),
    value='LOOP_MID',
    ref_at=(mid_x + 4 * G, cy - g(8) - 1.5 * G),
    show_val=False,
)
lw((mid_x, cy), (mid_x, cy - g(8)))
j3 = place(
    'relay-board',
    'Conn_01x01',
    'J3',
    g(190),
    cy,
    value='LOOP OUT',
    ref_at=(g(190), cy - 4 * G),
    val_at=(g(190), cy + 4 * G),
    props={'Note': 'Solder wire to Phoenix DFK-MSTB header pin 2'},
)
lw(chain[3]['44'], j3['1'])
label('LOOP_OUT', g(170), cy, 0)
text(
    'NO 33-34 + NO 43-44 in series inside each relay, then K1 in series with K2:\n4 gaps split the DC arc.  Nothing may bridge these contacts (no TVS, snubber, LED).\nFit the flyback diode / TVS at the LOAD (contactor or relay coil).',
    g(12),
    ly + g(31),
    size=1.3,
)
for (kk, u), xx, pr in zip(names, xs, ('33-34', '43-44', '33-34', '43-44')):
    text(f'NO {pr}', xx - g(3), cy + g(4), size=1.3)

# ---------------- right column: notes, mounting holes
frame(g(230), g(14), g(324), g(84), 'HOW IT WORKS')
text(
    'Fail-safe: de-energized = loop open.\n'
    'Power loss, reset, floating GPIO or a broken\nwire all leave both relays released.\n\n'
    'Each relay is force-guided (EN 61810-3 type A):\n'
    'if a NO contact welds, every NC contact of that\nrelay stays >= 0.5 mm open.  The NC mirror\n'
    'therefore proves the loop contacts released.\n\n'
    'Commanded 0, SNS reads 0:  released, healthy.\n'
    'Commanded 1, SNS reads 1:  pulled in, healthy.\n'
    'Commanded 0, SNS reads 1:  WELD or coil stuck on.\n'
    '   The partner relay stops the robot; raise fault.\n'
    'Commanded 1, SNS reads 0:  did not pull in (safe).\n\n'
    'Re-arm only after BOTH SNS read 0.\n\n'
    'K1 = channel A = core 0 (IO39 drive, IO40 sense)\n'
    'K2 = channel B = core 1 (IO41 drive, IO42 sense)\n'
    'K unit letters: A coil, B/C NC mirror, D/E NO loop.',
    g(233),
    g(22),
    size=1.4,
)
frame(g(230), g(90), g(324), g(124), 'MECHANICAL')
for i, (rf, xx) in enumerate((('H1', 240), ('H2', 258), ('H3', 276), ('H4', 294))):
    place('relay-board', 'MountingHole', rf, g(xx), g(113), value='M3', ref_at=(g(xx), g(108)), val_at=(g(xx), g(118)))
text('M3 holes match the 44.86 x 33.57 mm standoff\npattern in machine-casing.FCStd.', g(233), g(98), size=1.3)


# ================================================================= EMIT
def fmt(v):
    return f'{v:.3f}'.rstrip('0').rstrip('.')


def eff(size=1.4, bold=False, ital=False, just=None, hide=False):
    s = f'(effects (font (size {size} {size})' + (' bold' if bold else '') + (' italic' if ital else '') + ')'
    if just:
        s += f' (justify {just})'
    if hide:
        s += ' hide'
    return s + ')'


out = []
out.append('(kicad_sch\n\t(version 20260306)\n\t(generator "eeschema")\n\t(generator_version "10.0")')
out.append(f'\t(uuid "{ROOT}")\n\t(paper "A3")')
out.append(
    '\t(title_block\n\t\t(title "Machine relay board: dual force-guided safety relays")\n\t\t(date "2026-09-29")\n\t\t(rev "A")\n\t\t(company "Polymath Robotics")\n'
    '\t\t(comment 1 "Two TE SR4D4005 force-guided relays, NO contacts in series in the stop loop")\n'
    '\t\t(comment 2 "NC contacts read back by the ESP32 (contact mirror) on IO40 / IO42")\n'
    '\t\t(comment 3 "Schematic only. Not a functional-safety certification. See DESIGN.md")\n\t)'
)
# lib_symbols
libs = []
for key, txt in used_libs.items():
    lib, name = key.split(':')
    t = txt.replace(f'(symbol "{name}"', f'(symbol "{key}"', 1)
    libs.append('\n'.join('\t\t' + ln for ln in t.splitlines()))
out.append('\t(lib_symbols\n' + '\n'.join(libs) + '\n\t)')
for x, y in juncs:
    out.append(
        f'\t(junction\n\t\t(at {fmt(x)} {fmt(y)})\n\t\t(diameter 0)\n\t\t(color 0 0 0 0)\n\t\t(uuid "{uid("j", x, y)}")\n\t)'
    )
for i, (a, b, w) in enumerate(wires):
    out.append(
        f'\t(wire\n\t\t(pts\n\t\t\t(xy {fmt(a[0])} {fmt(a[1])}) (xy {fmt(b[0])} {fmt(b[1])})\n\t\t)\n\t\t(stroke\n\t\t\t(width {w})\n\t\t\t(type default)\n\t\t)\n\t\t(uuid "{uid("w", i, a, b)}")\n\t)'
    )
for i, (n, x, y, ang) in enumerate(labels):
    just = 'left bottom' if ang == 0 else 'right bottom'
    out.append(
        f'\t(label "{n}"\n\t\t(at {fmt(x)} {fmt(y)} {ang})\n\t\t{eff(just=just)}\n\t\t(uuid "{uid("l", i, n)}")\n\t)'
    )
for i, (x1, y1, x2, y2, dash) in enumerate(rects):
    out.append(
        f'\t(rectangle\n\t\t(start {fmt(x1)} {fmt(y1)})\n\t\t(end {fmt(x2)} {fmt(y2)})\n\t\t(stroke\n\t\t\t(width 0.2)\n\t\t\t(type {"dash" if dash else "default"})\n\t\t)\n\t\t(fill\n\t\t\t(type none)\n\t\t)\n\t\t(uuid "{uid("r", i)}")\n\t)'
    )
for i, (s, x, y, size, bold, ital, just) in enumerate(texts):
    s2 = s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
    out.append(
        f'\t(text "{s2}"\n\t\t(exclude_from_sim no)\n\t\t(at {fmt(x)} {fmt(y)} 0)\n\t\t{eff(size, bold, ital, just)}\n\t\t(uuid "{uid("t", i)}")\n\t)'
    )
for i, p in enumerate(parts):
    lib = p['lib']
    ref_at = p['ref_at'] or (p['x'] + 2.54, p['y'] - 2.54)
    if p.get('power'):
        ref_hide, val_at = True, None
        vx, vy = p['x'], p['y'] + (3.5 if p['value'] == 'GND' else -3.5)
        if p['value'] == 'PWR_FLAG':
            vy = p['y'] - 4.5
        val_pos = (vx, vy)
    else:
        ref_hide = False
        val_pos = p['val_at'] or (p['x'] + 2.54, p['y'] + 2.54)
    mir = f'\n\t\t(mirror {p["mirror"]})' if p['mirror'] else ''
    props = []

    def jf(pt, ax):
        return 'left' if pt[0] > ax + 0.1 else ('right' if pt[0] < ax - 0.1 else None)

    props.append(
        f'\t\t(property "Reference" "{p["ref"]}"\n\t\t\t(at {fmt(ref_at[0])} {fmt(ref_at[1])} {p.get("fang", 0)})\n\t\t\t{"(hide yes)" if ref_hide else ""}\n\t\t\t{eff(just=jf(ref_at, p["x"]))}\n\t\t)'
    )
    hide_val = (not p['show_val']) or p['value'] == 'PWR_FLAG'
    props.append(
        f'\t\t(property "Value" "{p["value"]}"\n\t\t\t(at {fmt(val_pos[0])} {fmt(val_pos[1])} {p.get("fang", 0)})\n\t\t\t{"(hide yes)" if hide_val else ""}\n\t\t\t{eff(just=jf(val_pos, p["x"]))}\n\t\t)'
    )
    fp = p['footprint'] or {
        'R': 'Resistor_SMD:R_0603_1608Metric',
        'C': 'Capacitor_SMD:C_0805_2012Metric',
        'Q_NMOS_GSD': 'Package_TO_SOT_SMD:SOT-23',
        'D_Zener': 'Diode_SMD:D_SMA',
        'SR4D4005': 'relay-board:SR4D4005',
        'TestPoint': 'TestPoint:TestPoint_Pad_D1.5mm',
        'MountingHole': 'MountingHole:MountingHole_3.2mm_M3',
        'Conn_01x07': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
        'Conn_01x01': 'Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical',
    }.get(lib.split(':')[1], '')
    props.append(
        f'\t\t(property "Footprint" "{fp}"\n\t\t\t(at {fmt(p["x"])} {fmt(p["y"])} 0)\n\t\t\t(hide yes)\n\t\t\t{eff()}\n\t\t)'
    )
    props.append(
        f'\t\t(property "Datasheet" ""\n\t\t\t(at {fmt(p["x"])} {fmt(p["y"])} 0)\n\t\t\t(hide yes)\n\t\t\t{eff()}\n\t\t)'
    )
    for k2, v2 in p['props'].items():
        props.append(
            f'\t\t(property "{k2}" "{v2}"\n\t\t\t(at {fmt(p["x"])} {fmt(p["y"])} 0)\n\t\t\t(hide yes)\n\t\t\t{eff()}\n\t\t)'
        )
    is_pwr = p.get('power')
    out.append(
        f'\t(symbol\n\t\t(lib_id "{lib}")\n\t\t(at {fmt(p["x"])} {fmt(p["y"])} {p["rot"]}){mir}\n\t\t(unit {p["unit"]})\n\t\t(body_style 1)\n\t\t(exclude_from_sim no)\n\t\t(in_bom {"no" if is_pwr else "yes"})\n\t\t(on_board {"no" if is_pwr else "yes"})\n\t\t(in_pos_files {"no" if is_pwr else "yes"})\n\t\t(dnp no)\n\t\t(uuid "{uid("s", i, p["ref"], p["unit"])}")\n'
        + '\n'.join(props)
        + f'\n\t\t(instances\n\t\t\t(project "relay-board"\n\t\t\t\t(path "/{ROOT}"\n\t\t\t\t\t(reference "{p["ref"]}")\n\t\t\t\t\t(unit {p["unit"]})\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)'
    )
out.append('\t(sheet_instances\n\t\t(path "/"\n\t\t\t(page "1")\n\t\t)\n\t)\n\t(embedded_fonts no)\n)')
OUT.write_text('\n'.join(out) + '\n')
print('wrote', OUT, len(parts), 'symbols', len(wires), 'wires', len(labels), 'labels')
