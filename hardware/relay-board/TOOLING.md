# Relay board: tools used, and how to reproduce on another computer

This board was designed in an AI-assisted session (OpenCode). Everything the
session produced is in this directory; this note records **which tools and MCP
servers were used**, what was learned about them, and how to set the same
environment up on a second machine.

## 1. What was used

| Tool | Version | Used for |
|---|---|---|
| KiCad (AppImage) | 10.0.0 | Schematic and library format, `kicad-cli` (ERC, netlist, PDF, symbol upgrade) |
| **kicad-mcp-pro** (MCP server) | 3.35.2, MIT | Schematic project setup, ERC, readability scoring, project checks |
| OpenCode | current | Agent host that runs the MCP servers |
| FreeCAD (AppImage) | 1.1.1 | `freecadcmd` scripts to read the casing geometry (`tools/casing_probe.py`) |
| uv | 0.10.0 | Installs the MCP server in an isolated Python 3.13 environment |
| Python | 3.10 system, 3.13 via uv | Checker and generator scripts (stdlib only) |
| poppler-utils | 22.02 | `pdftotext` / `pdftoppm` to read datasheets and render pages |
| Inkscape, ImageMagick | 1.1.2 | SVG to PNG, to eyeball symbols, footprints and the schematic |
| pre-commit | 4.6.2 | The repo's Polymath code standard (CI runs it) |
| gh | 2.4.0 | Opening the PR |

Web sources for parts (TE, Panasonic, Omron, Finder datasheets; jlcpcb.com and
lcsc.com part pages) are listed in `DESIGN.md`. Prices and stock were read on
2026-09-29 and will have moved.

The FreeCAD, Blender and Graphify MCP servers were also configured in the
session but were **not** used for this work: the FreeCAD MCP needs a running GUI,
so the casing was read with `freecadcmd` directly.

## 2. The KiCad MCP server: kicad-mcp-pro

Chosen because the project needs a permissive licence for commercial use, and
because it authors KiCad 10 schematics without a running GUI. Not chosen:
Konnect (AGPL-3.0), the original KiCAD-MCP-Server (needs SWIG `pcbnew`, which the
AppImage does not provide to a system Python).

**Vetting done before installing** (2026-09-29; targeted, not a line-by-line read
of its 90k lines):

- The PyPI wheel matches the signed attestation (published by GitHub Actions from
  `oaslananka/kicad-mcp-pro`) and its `src/` is byte-identical to release commit
  `1a117ee`.
- Scanned all 314 Python files for `shell=True`, dynamic code, obfuscation, install
  hooks, and network use. Subprocess calls use fixed argument lists. The only
  outbound calls are the part-search tools (JLCPCB/jlcsearch and optional Nexar,
  DigiKey, Mouser), and only when invoked. HTTP transport binds to loopback by default.
  Telemetry is off unless an OTLP endpoint is configured.
- All 93 installed dependencies are permissive (MIT, Apache-2.0, BSD, ISC), plus
  `certifi` under MPL-2.0. Dependency code was **not** audited.
- Risk to keep in mind: the project is about four months old, has one main
  maintainer, and ships several releases a week. Pin the version.

Pinned artifact:

```text
kicad_mcp_pro-3.35.2-py3-none-any.whl
sha256 a7cf9e13f1924d5b1360984b8ff12ac774f76f6529d1fb6328026dd12e215fee
```

**Install (verify the hash first):**

```sh
pip download kicad-mcp-pro==3.35.2 --no-deps --python-version 3.13 \
    --only-binary=:all: -d /tmp/kmp
sha256sum /tmp/kmp/*.whl        # must match the value above
uv tool install --python 3.13 /tmp/kmp/kicad_mcp_pro-3.35.2-py3-none-any.whl
```

**Wire it to KiCad 10.** `kicad-cli` must be a real file; a wrapper around the
AppImage works:

```sh
cat > ~/.local/bin/kicad-cli <<'EOF'
#!/usr/bin/env bash
exec "$HOME/Applications/kicad-10.0.0-x86_64.AppImage" kicad-cli "$@"
EOF
chmod +x ~/.local/bin/kicad-cli
```

Then add the server to OpenCode; see `tools/opencode-kicad-mcp.example.jsonc`.

**Profiles.** Servers expose different tool sets per `KICAD_MCP_PROFILE`:

| Profile | Tools | Use |
|---|---|---|
| `schematic_authoring` | 191 | What this work used: schematic edits, ERC, library search, quality gates |
| `pcb_layout` | 97 | Inspection only; placing and routing need a live KiCad (IPC), see next steps |
| `build` | 17 | Too narrow for authoring; only net-label plans |

## 3. Reproducing the environment

```sh
# 1. KiCad 10 AppImage and the kicad-cli wrapper (section 2)
# 2. kicad-mcp-pro 3.35.2, hash-checked (section 2)
# 3. Stock KiCad libraries out of the AppImage (only needed to rebuild the libraries)
hardware/relay-board/tools/extract_kicad_libs.sh ~/Applications/kicad-10.0.0-x86_64.AppImage
# 4. Point OpenCode at the project (tools/opencode-kicad-mcp.example.jsonc), restart it
```

To confirm the environment matches, from the repo root:

```sh
kicad-cli sch erc hardware/relay-board/relay-board.kicad_sch          # 0 violations
python3 hardware/relay-board/tools/check_netlist.py --selftest         # PASS, all faults caught
kicad-cli sch export pdf -o /tmp/relay-board.pdf hardware/relay-board/relay-board.kicad_sch
```

In an OpenCode session with the MCP running, `run_erc` should report PASS and
`sch_cosmetic_score` should report 100.

Rebuild the generated files (optional; the committed files are the source of truth):

```sh
export KICAD_SYMBOL_LIBS=~/Applications/kicad10-libs/symbols KICAD_CLI=$(command -v kicad-cli)
python3 hardware/relay-board/tools/build_symbols.py     # libraries + SR4D4005 footprint
python3 hardware/relay-board/tools/build_schematic.py   # overwrites relay-board.kicad_sch
pre-commit run --files hardware/relay-board/*             # whitespace fixer, then no diff
CASING_FCSTD=hardware/machine-casing.FCStd \
  ~/Applications/FreeCAD_1.1.1-Linux-x86_64-py311.AppImage freecadcmd \
  hardware/relay-board/tools/casing_probe.py             # casing envelope numbers
```

Re-running both build scripts reproduces the committed files exactly. Once you start
editing the schematic in KiCad, stop using `build_schematic.py`: it would overwrite
your edits.

## 4. Things that went wrong or surprised us

- **The MCP part search is not reliable for value matching.** It matched `10k 0603`
  to a 510 kΩ part, and its index missed the TE SR4 relays that jlcpcb.com and
  lcsc.com list. Every part number in `DESIGN.md` was checked on the vendor page.
- **A label-only schematic is unreadable.** `sch_build_circuit` connects nets with
  labels by design ("collision-safe"), which scored 32/100 on `sch_cosmetic_score`.
  The redraw uses real wires and a multi-unit relay symbol (score 100).
- **`sch_cosmetic_score` measures multi-unit parts as the union of all their units**,
  so it reports a symbol overlapping itself. The units here are spaced to keep it quiet.
- **`kicad_create_new_project` nests the project one folder deeper** than the path
  given; the files were moved up afterwards.
- **KiCad 10 stores stock symbols one per file** in `*.kicad_symdir` folders, which this
  MCP does not read. The project therefore carries its own symbol libraries.
- **KiCad prefixes local-label net names with the sheet path** (`/LOOP_IN`); the
  checker normalises this.
- **Netlist export omits power flags** (`#FLG..`), so they are not in the expected nets.
- The pre-commit hooks strip whitespace-only lines from the `.kicad_sch`; harmless.
