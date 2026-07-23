# Contributing

Thanks for your interest in the Polymath Protective Stop. This is a
safety-relevant device, so contributions are held to a high bar: clear
design intent, tested changes, and clean CI. Please read this document
before opening a pull request.

## Repository layout

| Path          | What it is                                                                 |
| ------------- | ------------------------------------------------------------------------- |
| `pstop_c/`    | Certified C protocol library (Apache-2.0). **Do not modify here** — see below. |
| `firmware/`   | ESP-IDF v5.5 project; builds `pstop_remote.bin` for the ESP32-S3 remote.   |
| `components/` | Shared ESP-IDF components (`microlink`, `ml_dev_tether`, `pstop`).         |
| `host/`       | Plain-C `machine_app_runner` (robot-side pstop machine; no ESP-IDF).       |
| `tools/`      | Python test tools and `misra_check.sh` (cppcheck MISRA pre-check).         |
| `test/`       | Bash test ladders (chaos, netem, soak, recovery).                         |
| `docs/`       | Design and test documentation.                                            |
| `hardware/`   | Enclosure CAD, schematic, STEP files (WIP — see `hardware/README.md`).     |
| `archive/`    | Deprecated ROS2 packages. Not built; do not extend.                        |

## The certified library: `pstop_c/`

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
idf.py flash monitor    # to a connected board
```

`sdkconfig.credentials` holds secrets (Wi-Fi, Tailscale auth key, admin
password) and is gitignored — never commit it.

### Host runner (robot-side machine)

No ESP-IDF needed, just a C compiler.

```sh
cd host
make                    # produces ./machine_app_runner
```

## Testing

- **Remote protocol / arming policy:** `tools/pstop_test_remote.py` bonds
  over the real wire protocol and runs timed STOP/OK sequences against a
  runner instance. See `docs/TESTING.md`.
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

By contributing you agree that your contributions are licensed under the
repository's Apache-2.0 license (see `LICENSE`).
