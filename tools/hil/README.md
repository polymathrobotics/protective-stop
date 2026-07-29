# pstop hardware-in-the-loop test suite

Drives a real pstop remote through its **actual E-stop loops and power
feed** using a 4-channel USB relay board (`tools/usb_relay4.py`), and
asserts the safety behavior at a **dedicated real machine instance**
(`host/machine_app_runner`) over the real transport. Nothing is mocked:
the loop GPIOs, the lockstep comparator, the wire protocol, and the
machine-side arming policy are all the shipping code paths.

## Rig wiring (as built, verified 2026-07-28)

Relay board: STM32 VCP (`0483:5740`), auto-detected. All three channels
use the relays' **NO contacts**, so the fully de-energized rig is the
fail-safe state: loops open ("button pressed"), DUT unpowered.
Bench-normal requires relays 1–3 energized — `rig.idle()` and every
fixture teardown put the rig there.

| Relay | NO contact in series with            | De-energized means    |
|-------|--------------------------------------|-----------------------|
| 1     | E-stop loop A: GPIO39 → GPIO40       | loop A open ("press") |
| 2     | E-stop loop B: GPIO41 → GPIO42       | loop B open ("press") |
| 3     | DUT USB power feed (VBUS)            | DUT power cut         |
| 4     | spare                                |                       |

If a future rig wires NC contacts instead, flip `press_energizes` /
`power_cut_energizes` in `hil.toml` — the suite is polarity-agnostic.

The DUT is tethered on USB-NCM; `hil.toml` names the host interface
(`ncm_iface`) and the suite finds the chip's IP from the kernel
neighbor table, then talks to `state.json` + `/api/pstop_peers`. This
bench has TWO pstops attached (soak unit on `esp-pstop0`/10.42.0.x,
test unit on `usb0`/10.43.0.x) — the iface setting is what keeps the
suite pointed at the right one. The suite claims **peer slot 3** on the
chip for its own machine instance (port 8894, id 0x01020390) and clears
it on teardown; slot 0 belongs to whatever real bench machine exists.

## Running

```sh
cd tools/hil
python3 -m pytest                 # everything (incl. power cycles, slow)
python3 -m pytest -m 'not power'  # skip the power-cycle tests
python3 -m pytest test_00_rig.py  # rig self-check after (re)wiring
```

Requires: pyserial, pytest (system python3 is fine — nothing is
installed). If the loops aren't wired yet, everything that needs them
**skips** with a message rather than failing; `test_00_rig.py` tells
you whether the wiring is live.

## What is covered

- `test_00_rig.py` — relay board round-trip, chip API shape, and that
  each relay opens exactly its own loop (catches miswiring).
- `test_10_button.py` — press ⇒ STOP on the wire; release ⇒ armed, never
  sooner than `min_stop_ms`; held button pins the robot at STOP; a
  sub-`min_stop` blip defers arming (pstop_c #59 semantics); re-arm
  after a stop while armed.
- `test_20_discordance.py` — one loop open, the other closed (per
  channel): comparator must go silent (no STOP message on the wire),
  machine stops via MISSED_HEARTBEATS, chip counts lockstep mismatches,
  and healing the fault must NOT re-arm without a fresh gesture.
- `test_30_power_cycle.py` (`-m power`) — power loss stops the machine;
  a cold boot re-bonds (NVS peer slot) but never self-arms; boot with
  the button held flows STOP and then completes a gesture on release.

Timing knobs live in `hil.toml [timing]` and are fed into the generated
machine config, so the asserted budgets and the machine under test
always agree.
