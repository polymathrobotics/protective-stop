# Contributing

Thanks for your interest in the Polymath Protective Stop. This is a
safety-relevant device, so contributions are held to a high bar: clear
design intent, tested changes, and clean CI. Please read this document
before opening a pull request.

## Repository layout

| Path          | What it is                                                                 |
| ------------- | ------------------------------------------------------------------------- |
| `pstop_c/`    | Certification-track C protocol library (Apache-2.0). **Do not modify here** - see below. |
| `firmware/`   | ESP-IDF v5.5 project; builds `pstop_remote.bin` for the ESP32-S3 remote.   |
| `components/` | Shared ESP-IDF components (`microlink`, `ml_dev_tether`, `pstop`).         |
| `host/`       | Plain-C `machine_app_runner` (robot-side pstop machine; no ESP-IDF).       |
| `tools/`      | Python test tools and `misra_check.sh` (cppcheck MISRA pre-check). Run them with `uv` — see `tools/README.md`. |
| `test/`       | Bash test ladders (chaos, netem, soak, recovery).                         |
| `docs/`       | Design and test documentation.                                            |
| `hardware/`   | Certified enclosure CAD, schematic, STEP files, BOM, and assembly guide.  |
| `archive/`    | Deprecated ROS2 packages. Not built; do not extend.                        |

## The certification-track library: `pstop_c/`

`pstop_c/` is the certification-track protocol library. It is vendored
into this repo unmodified and is **excluded from the linters/formatters**
on purpose.

**Do not change `pstop_c/` in this repository.** If you find a bug or
need a feature in the protocol library, contribute it upstream so the
change goes through the library's own review, test, and certification
process. It then flows back here as a version bump. PRs that edit
`pstop_c/` will be asked to move the change upstream.

## Building

### Firmware (ESP32-S3 remote)

Requires ESP-IDF v5.5.

```sh
cd firmware
cp sdkconfig.credentials.example sdkconfig.credentials   # then edit with real values
idf.py build            # produces build/pstop_remote.bin
idf.py flash            # to a connected board
```

`idf.py monitor` only shows the first seconds of boot: the USB port becomes the
network tether once TinyUSB starts and the serial console goes quiet. Use
`http://<remote>/state.json` and `/api/last_log` instead, or wire a USB-UART
adapter to the UART0 pins. Full first-time walkthrough: `docs/QUICKSTART.md`.

`sdkconfig.credentials` holds secrets (Wi-Fi, Tailscale auth key, admin
password) and is gitignored — never commit it. Every value in it is compiled
into the `.bin` **and** the `.elf` as a plain string, so a build made with a
credentials file present is private and must never be attached to a GitHub
release or shared outside the organisation.

### Publishing release binaries

Only builds made **without** a credentials file go on a release. Build them in a
clean checkout with `idf.py -DPROJECT_VER=<tag>-public build merge-bin` (the
`-public` suffix keeps `fw_ver` distinguishable from private builds of the same
commit), then run the guard on every artifact before `gh release upload`:

```sh
tools/release_guard.sh firmware/build/pstop_remote.bin firmware/build/pstop_remote.elf ...
```

It refuses (exit 1) any file that carries the private-build brand (every
image compiled while a credentials file was present is branded
`ML-BUILD-WITH-CREDENTIALS`, so the verdict does not depend on which
credentials file the checking machine has), any value of a local credentials
file, or a secret-shaped string (`tskey-…`, PEM keys, literal auth tokens).

### Host runner (robot-side machine)

No ESP-IDF needed, just a C compiler.

```sh
cd host
make                    # produces ./machine_app_runner
```

## Testing

- **Remote protocol / arming policy:** `tools/pstop_test_remote.py` bonds
  over the real wire protocol and runs timed STOP/OK sequences against a
  runner instance. Run it as `cd tools && uv run python
  pstop_test_remote.py`; see `docs/TESTING.md` and `tools/README.md`.
- **Test ladders:** the scripts in `test/` (`chaos_ladder.sh`,
  `netem_ladder.sh`, `longsoak.sh`, `test_suite.sh`, recovery scripts)
  exercise the system under packet loss, latency, and fault injection.
- **MISRA pre-check:** run the free cppcheck MISRA C:2012 check over the
  code we own:

  ```sh
  ./tools/misra_check.sh          # main + dcs_support (default: all)
  ```

  Residual findings are the documented deviation register in
  `docs/MISRA_COMPLIANCE_2026-07-21.md`. This is an engineering pre-check;
  formal certification evidence needs a licensed MISRA checker.

## Pre-commit

This repo uses the Polymath code standard via pre-commit. Install the
hook once and let it format/lint on every commit:

```sh
pip install pre-commit
pre-commit install
pre-commit run --all-files       # optional: check the whole tree
```

Note `pstop_c/` is intentionally excluded from the C/C++ hooks.

## Pull request expectations

- **CI green.** Firmware build, host build, `pstop_c` build + tests, and
  pre-commit must all pass.
- **Pre-commit clean.** Run it locally before pushing; do not disable
  hooks to get around findings.
- **Tests for behavior changes.** Anything touching the safety chain
  (arming policy, heartbeat/comparator, failover) needs a corresponding
  test or a clear explanation of how it was verified on hardware.
- **No changes to `pstop_c/`** (contribute upstream instead).
- **Clear commits.** Explain the design intent, not just the diff.
  Reference the relevant `docs/` design note where one applies.
- **Comments carry constraints, not stories.** A code comment states what the
  reader cannot infer from the code — an ordering, a unit, an ownership rule, a
  "must not" with its consequence — in a few lines. The incident, the analysis
  and the alternatives go in the commit message, PR or issue, and the comment
  points there (`#158`). Write the rationale once, at the code that owns the
  decision; every other site gets one line and a pointer (`see dcs_rgb_start()`,
  `see ml_peer_nvs.c`). If the same paragraph would be right in two places, it
  belongs in neither.
- **Keep safety traceability lint clean.** The linter checks requirement and
  function mappings, statuses, evidence citations, and ownership of numeric
  coverage claims. Generated coverage becomes stale whenever document
  citations or statuses change; refresh it with
  `cd tools && uv run python -m safety_lint --write` (see `tools/README.md`
  for the environment). This command recounts
  citations and statuses from the documents; it does not execute tests or
  establish that cited tests pass. Write mode refuses to modify the document
  while unbaselined errors exist. Automatic pre-commit rewriting is
  deliberately not configured because coverage drops require human review.

Contributions are licensed according to where they land: software and firmware
under Apache-2.0, hardware design files under CERN-OHL-P-2.0, and documentation
under CC-BY-4.0. The full texts are in `LICENSE` and `LICENSES/`.
