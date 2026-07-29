# pstop hardware-in-the-loop test suite

Drives a real pstop remote through its **actual E-stop loops and power
feed** using a 4-channel USB relay board (`tools/usb_relay4.py`), and
asserts the safety behavior at a **dedicated real machine instance**
(`host/machine_app_runner`) over the real transport. Nothing is mocked:
the loop GPIOs, the lockstep comparator, the wire protocol, and the
machine-side arming policy are all the shipping code paths.

## Rig wiring

Relay board: STM32 VCP (`0483:5740`), auto-detected. Use each relay's
**NC contact** so the de-energized rig leaves the bench completely
normal (button works, DUT powered):

| Relay | NC contact in series with            | Energized means      |
|-------|--------------------------------------|----------------------|
| 1     | E-stop loop A: GPIO39 → GPIO40       | loop A open ("press")|
| 2     | E-stop loop B: GPIO41 → GPIO42       | loop B open ("press")|
| 3     | DUT USB power feed (VBUS)            | DUT power cut        |
| 4     | spare                                |                      |

Keep the physical DPST button in the loops — the relay contacts sit in
series with its poles, so both the button and the rig can open a loop.
If you wire NO contacts instead, flip `press_energizes` /
`power_cut_energizes` in `hil.toml`.

The DUT stays on USB-NCM (`esp-pstop0`, host = 10.42.0.1); the suite
finds its IP from the neighbor table and talks to `state.json` +
`/api/pstop_peers`. The suite claims **peer slot 3** on the chip for its
own machine instance (port 8894, id 0x01020390) and clears it on
teardown — the bench machine in slot 0 (port 8890) is never touched.
Bench side effect: while a test holds a "press" or cuts power, the
bench machine will also see STOPs. That is real behavior, not damage.

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
