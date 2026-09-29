#!/usr/bin/env python3
"""Verify the relay-board schematic netlist against the intended design.

Exports the netlist with kicad-cli (KiCad's own connectivity engine, independent
of whatever generated the schematic) and checks:

  1. every net contains exactly the intended pins, and no other nets exist;
  2. the safety rules in ../DESIGN.md section 4 hold.

Usage: check_netlist.py [--kicad-cli PATH]     exit status 0 = pass
"""
import argparse
import pathlib
import re
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
SCH = HERE.parent / "relay-board.kicad_sch"

# One entry per channel; the two channels are identical by construction.
CHANNELS = {
    "A": dict(k="K1", q="Q1", d="D1", rg="R1", rp="R3", ru="R5", rs="R7", tp_gate="TP4", tp_coil="TP6", tp_mirror="TP8", jdrv="J1.3", jsns="J1.4"),
    "B": dict(k="K2", q="Q2", d="D2", rg="R2", rp="R4", ru="R6", rs="R8", tp_gate="TP5", tp_coil="TP7", tp_mirror="TP9", jdrv="J1.5", jsns="J1.6"),
}


def expected_nets():
    # Power flags (#FLG..) are schematic-only and not part of KiCad's netlist export.
    nets = {
        "+5V": {"J1.1", "K1.A1", "K2.A1", "C1.1", "TP1.1"},
        "+3V3": {"J1.7", "R5.1", "R6.1", "TP2.1"},
        "GND": {"J1.2", "R3.2", "R4.2", "Q1.2", "Q2.2", "D1.2", "D2.2", "K1.11", "K2.11", "C1.2", "TP3.1"},
        "LOOP_IN": {"J2.1", "K1.34"},
        "LOOP_MID": {"K1.44", "K2.34", "TP10.1"},
        "LOOP_OUT": {"K2.44", "J2.2"},
    }
    for n, c in CHANNELS.items():
        nets[f"DRV_{n}"] = {c["jdrv"], f"{c['rg']}.1"}
        nets[f"GATE_{n}"] = {f"{c['rg']}.2", f"{c['rp']}.1", f"{c['q']}.1", f"{c['tp_gate']}.1"}
        nets[f"COIL_{n}_N"] = {f"{c['q']}.3", f"{c['d']}.1", f"{c['k']}.A2", f"{c['tp_coil']}.1"}
        nets[f"SNS_{n}"] = {c["jsns"], f"{c['rs']}.2"}
        nets[f"MIRROR_{n}"] = {f"{c['rs']}.1", f"{c['ru']}.2", f"{c['k']}.21", f"{c['tp_mirror']}.1"}
        nets[f"{c['k']}_NC_LINK"] = {f"{c['k']}.12", f"{c['k']}.22"}
        nets[f"{c['k']}_NO_LINK"] = {f"{c['k']}.33", f"{c['k']}.43"}
    return nets


def tokenize(text):
    return re.findall(r'"(?:[^"\\]|\\.)*"|\(|\)|[^\s()"]+', text)


def parse(text):
    stack, cur = [], []
    for tok in tokenize(text):
        if tok == "(":
            stack.append(cur)
            cur = []
        elif tok == ")":
            done, cur = cur, stack.pop()
            cur.append(done)
        else:
            cur.append(tok.strip('"') if tok.startswith('"') else tok)
    return cur


def find(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def read_netlist(kicad_cli):
    with tempfile.TemporaryDirectory() as tmp:
        out = pathlib.Path(tmp) / "rb.net"
        subprocess.run(
            [kicad_cli, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(out), str(SCH)],
            check=True, capture_output=True,
        )
        tree = parse(out.read_text())[0]
    nets = {}
    for net in find(find(tree, "nets")[0], "net"):
        name = find(net, "name")[0][1]
        nodes = {f"{find(n, 'ref')[0][1]}.{find(n, 'pin')[0][1]}" for n in find(net, "node")}
        nets[name] = nodes
    return nets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kicad-cli", default="kicad-cli")
    args = ap.parse_args()
    got = read_netlist(args.kicad_cli)
    errors = check(got)
    if errors:
        print("FAIL")
        for e in errors:
            print("  -", e)
        return 1
    print(f"PASS: {len(got)} nets match the intended design; safety rules 1-5 hold")
    return 0


def check(got):
    want = expected_nets()
    errors = []

    # 1. exact net-by-net comparison (ignore unconnected-* placeholder nets)
    got = {k: v for k, v in got.items() if not k.startswith("unconnected-")}
    for name in sorted(set(want) | set(got)):
        w, g = want.get(name), got.get(name)
        if w is None:
            errors.append(f"unexpected net {name}: {sorted(g)}")
        elif g is None:
            errors.append(f"missing net {name}")
        elif w != g:
            errors.append(f"net {name}: missing {sorted(w - g)} extra {sorted(g - w)}")

    # 2. safety rules
    def refs(net):
        return {p.split(".")[0] for p in got.get(net, set())}

    loop_nets = ("LOOP_IN", "LOOP_MID", "LOOP_OUT", "K1_NO_LINK", "K2_NO_LINK")
    allowed_loop_refs = {"K1", "K2", "J2", "TP10"}
    for net in loop_nets:
        extra = refs(net) - allowed_loop_refs
        if extra:
            errors.append(f"RULE 1 (nothing bridges the loop contacts): {sorted(extra)} on {net}")
    for net in loop_nets:
        for pin in got.get(net, set()):
            ref, num = pin.split(".")
            if ref in ("K1", "K2") and num not in ("33", "34", "43", "44"):
                errors.append(f"RULE 2 (loop only on NO contacts): {pin} on {net}")
    for n, c in CHANNELS.items():
        # RULE 3: gate has a pull-down to GND (fail-off when the drive is floating)
        if f"{c['rp']}.2" not in got.get("GND", set()) or f"{c['rp']}.1" not in got.get(f"GATE_{n}", set()):
            errors.append(f"RULE 3 (gate pull-down) violated on channel {n}")
        # RULE 4: coil is low-side driven and the sense chain uses NC contacts only
        if f"{c['k']}.A1" not in got.get("+5V", set()):
            errors.append(f"RULE 4 (coil high side on +5V) violated on channel {n}")
        mirror_pins = {p for p in got.get(f"MIRROR_{n}", set()) if p.startswith(c["k"] + ".")}
        if mirror_pins != {f"{c['k']}.21"}:
            errors.append(f"RULE 4 (mirror uses NC contacts 11/12/21/22 only) violated on channel {n}: {mirror_pins}")
        # RULE 5: the two channels share no net other than power, GND and the loop chain
    shared = {n for n in got if any(p.startswith("K1.") for p in got[n]) and any(p.startswith("K2.") for p in got[n])}
    if shared - {"+5V", "GND", "LOOP_MID"}:
        errors.append(f"RULE 5 (channel independence): shared nets {sorted(shared - {'+5V', 'GND', 'LOOP_MID'})}")

    return errors


if __name__ == "__main__":
    sys.exit(main())
