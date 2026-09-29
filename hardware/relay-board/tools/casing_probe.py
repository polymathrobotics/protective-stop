# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Print the machine-box geometry the relay board must fit (run inside FreeCAD).

    CASING_FCSTD=hardware/machine-casing.FCStd \\
        FreeCAD-AppImage freecadcmd hardware/relay-board/tools/casing_probe.py

Reports, in case coordinates (mm): the relay-module envelope, the M3 standoff
positions, how far the Phoenix header intrudes, and the lid ceiling over the
relay area. These are the numbers in DESIGN.md section 5.
"""

import os

import FreeCAD
import Part

path = os.environ.get('CASING_FCSTD', 'hardware/machine-casing.FCStd')
doc = FreeCAD.openDocument(path)
base = doc.getObject('MC_Base').Shape
lid = doc.getObject('MC_Lid').Shape
env = doc.getObject('Extrude').Shape  # relay_module_2ch_envelope
header = doc.getObject('MC_PhoenixHeader').Shape

b = env.BoundBox
print(f'RELAY ENVELOPE  X[{b.XMin:.2f},{b.XMax:.2f}] Y[{b.YMin:.2f},{b.YMax:.2f}] Z[{b.ZMin:.2f},{b.ZMax:.2f}]')

seen = set()
for f in base.Faces:
    s = f.Surface
    if s.__class__.__name__ == 'Cylinder' and s.Radius < 4.0:
        c = s.Center
        if -45 < c.y < 0 and -5 < c.x < 60:
            seen.add((round(c.x, 2), round(c.y, 2), round(s.Radius, 2)))
for x, y, r in sorted(seen):
    print(f'STANDOFF/HOLE   x={x} y={y} r={r}')

for z in (6.0, 8.0, 10.0, 14.0):
    boxes = []
    for wire in header.slice(FreeCAD.Vector(0, 0, 1), z):
        w = Part.Wire(wire).BoundBox
        boxes.append((round(w.XMin, 1), round(w.XMax, 1), round(w.YMin, 1), round(w.YMax, 1)))
    print(f'HEADER z={z:>4}  (xmin,xmax,ymin,ymax) {boxes[:6]}')


def z_hits(shape, x, y):
    line = Part.makeLine(FreeCAD.Vector(x, y, -10), FreeCAD.Vector(x, y, 60))
    return sorted(round(v.Z, 2) for e in shape.common(line).Edges for v in e.Vertexes)


print('LID CEILING (lowest lid Z over the relay area):')
for x in (5, 27, 50):
    for y in (-5, -22, -38):
        print(f'  x={x:>3} y={y:>4}  lid Z {z_hits(lid, x, y)}')
