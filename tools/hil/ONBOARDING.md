# pstop HIL bench — onboarding & handoff

Audience: an engineer (or Claude instance) bringing this hardware-in-the-loop
rig up on a **new machine**, and the roadmap for wiring it into GitHub CI/CD.
Read `README.md` first for what the suite covers; this document is everything
that is NOT in the code.

Status at handoff (2026-07-28): **12/12 tests green on real hardware, ~86 s
full run** (~33 s with `-m 'not power'`), on branch `pstop`.

## 1. Hardware inventory

| Item | Identity | Notes |
|------|----------|-------|
| Test DUT | PSTOP06, MAC 3C:0F:02:D7:F3:44, pstop id `0x01D7F344` | USB-powered THROUGH relay 3 |
| Relay board | 4-ch "A0 protocol", STM32 VCP `0483:5740` | driver: `tools/usb_relay4.py` |
| Rig wiring | relays 1/2 → E-stop loops A/B, relay 3 → DUT VBUS | NO contacts; see README table |

The DUT's E-stop loop pins: A = GPIO39→GPIO40, B = GPIO41→GPIO42
(`firmware/main/main.c`, "Dual-channel hardware E-stop loops" block — read it
before changing any loop-related test; the rolling-drive/debounce semantics
are load-bearing).

## 2. Bring-up on a new host

1. **Clone + branch**: `git clone <repo>`, `git checkout pstop`.
2. **Build the machine runner**: `cd host && make` (plain C, no deps). The
   suite spawns `host/machine_app_runner` itself.
3. **Permissions**: user in `dialout` group (serial). No udev rules needed.
4. **USB-NCM network**: plug the DUT in; `dmesg | tail` shows a `cdc_ncm`
   interface (name varies: `usb0`, `enx…`, or `esp-pstop0` if a .link rule
   matched). Give the host side a shared address so the chip can DHCP:
   `nmcli con add type ethernet ifname <iface> con-name pstop-hil ipv4.method shared ipv4.addresses 10.43.0.1/24`
   (any private /24; the chip takes a lease from it).
5. **Configure**: edit `tools/hil/hil.toml` — set `ncm_iface`, `host_ip`,
   and the relay polarity if the rig was rewired.
6. **Validate the rig**: `cd tools/hil && ./run.sh test_00_rig.py` — this
   creates the venv on first run and proves the wiring channel-by-channel.
7. **Full suite**: `./run.sh` (power cycles included) or
   `./run.sh -m 'not power'` for the quick loop.

## 3. Gotchas that already cost time (don't rediscover these)

- **`/api/pstop_peers` id needs the `0x` prefix.** Firmware parses it with
  `strtoul(base 0)`; a bare `01020390` is octal ⇒ silent wrong id ⇒ every
  bond rejected INVALID_ID. `rig.py` handles it; don't hand-roll curls
  without the prefix.
- **Never toggle the power relay casually.** Any blip reboots the DUT
  (~15 s until HTTP is back). The rig self-test deliberately skips it.
- **Stale `machine_app_runner` holding the UDP port** makes every bond land
  on a process with the wrong machine id. `MachineRunner.start()` now fails
  fast on EADDRINUSE; if you see it: `pgrep -af machine_app_runner`.
- **ROS on the host poisons pytest**: sourced ROS setup puts broken plugins
  on PYTHONPATH. `run.sh` strips PYTHONPATH — always run via `run.sh`.
- **The relay board occasionally swallows a command** sent immediately after
  its previous reply; the driver retries once (all commands idempotent).
- **Two pstops on one bench**: chip discovery is per-interface
  (`ncm_iface`); wrong iface = you're testing the wrong robot's chip.
- **After a power-cycle run the chip may still be booting** when the next
  session starts; the chip fixture waits up to 30 s before skipping.
- **Nextcloud-synced checkouts** can race incremental builds with stale
  mtimes; if a firmware build behaves impossibly, clean-build and verify a
  known string in the ELF before flashing.

## 4. Architecture (30-second version)

`conftest.py` wires three actors from `rig.py`:

- `RelayRig` — polarity-aware press/release (gang write: both loops within
  ~2 ms, like a real DPST) and DUT power control.
- `Chip` — the DUT's HTTP API: `state.json` (loop health `e_hi*/e_lo*`,
  lockstep `pstop_mismatch`, `uptime_ms` for reboot proof), peer-slot table.
- `MachineRunner` — dedicated per-test machine instance (UDP 8894,
  id 0x01020390, config generated from `hil.toml [timing]`), stderr parsed
  into a timestamped event stream (`ROBOT STATUS`, `now sending`,
  `MISSED_HEARTBEATS`, `ANOMALY: arming DEFERRED`).

Tests assert **at the machine**, which is where safety semantics live:
STOP/OK transitions, arming refusals, and liveness timeouts — with the chip's
own `state.json` as the second witness.

## 5. CI/CD roadmap (the goal this bench exists for)

Target: on every firmware change, GitHub CI flashes the latest build to the
test DUT, runs this suite, and gates the merge. Design constraints and the
path there:

1. **Self-hosted runner on the bench host.** GitHub-hosted runners can't
   touch USB. Register the bench machine as a self-hosted runner with labels
   like `pstop-hil`; the workflow job runs `tools/hil/run.sh` directly.
2. **Getting firmware onto the DUT** — two options, in order of preference:
   - **OTA via the fleet server** (chips already pull standing assignments):
     CI posts/updates the test unit's assignment to the new build, then
     power-cycles the DUT via relay 3 and waits for it to come back on the
     new firmware. Zero USB fiddling, exercises the real update path.
   - **USB flash**: `tools/flash_pstop.sh --from-ip <chip>` kicks a running
     unit into download mode over HTTP and flashes over USB. Needs the
     (git-ignored) `production_image/` secrets baked locally on the runner.
   Fleet-server URL and API key are PROPRIETARY: provide them to the runner
   as environment/secret (`PSTOP_FLEET_URL`, key file) — they must never
   appear in committed files (same rule as `sdkconfig.credentials`).
3. **Flash verification — firmware gap to close first**: `state.json` does
   not yet expose a firmware version/sha. Add a `fw_sha` (or app-desc
   version) field so CI can assert "the DUT is actually running the build
   under test" before trusting the results. Until then, verification is
   indirect (fleet console's reported sha).
4. **Suite shape for CI**: `run.sh -m 'not power'` (~33 s) as the
   per-commit gate; the full run with power cycles (~86 s) on merge to
   `pstop`/`main` or nightly. Add `--junitxml=report.xml` for annotations.
5. **Serialization**: one rig, one job at a time — GitHub `concurrency`
   group on the runner label. The suite already cleans up after itself
   (peer slot cleared, rig back to bench-normal) even on failure.
6. **Recovery**: if a run dies mid-power-cycle, the DUT may be dark. First
   step of every CI job: `tools/usb_relay4.py on 1,2,3` (bench-normal),
   then wait for the chip before testing.

## 6. Known limitations / next work

- Machine-runner binary is x86 Linux, built from `host/` — rebuild on the
  runner, don't commit binaries.
- The `wired` fixture proves loops A/B but power wiring is only proven by
  the power tests themselves (a wiring check would reboot the DUT).
- Multi-machine HIL (`tools/pstop_multi_machine_test.py`) is a separate,
  older harness — one remote to 4 machines — not yet folded into pytest.
- No stuck-high short-circuit fault injection (needs a relay to 3V3, spare
  relay 4 is a candidate).
