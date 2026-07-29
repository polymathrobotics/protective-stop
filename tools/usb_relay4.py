#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Driver + CLI for the 4-channel "A0" USB relay board used by the HIL rig.

Board: STM32 virtual COM port (USB CDC, `0483:5740`, baud-agnostic).
Commands are 4-byte binary frames; replies are ASCII lines.

    command:  [0xA0, addr, op, checksum]   checksum = (0xA0+addr+op) & 0xFF
    addr:     0x01..0x04 = relay channel, 0x0F = all channels
    op:       0x00 = off, 0x01 = on, 0x02 = query status
    reply:    one "CHn:ON\\r\\n" / "CHn:OFF\\r\\n" line per affected channel
              (query-all reports CH1..CH8 — the firmware is 8-channel,
              only 1..4 are populated on this board)

Every set is verified against the board's ack; a missing or wrong ack
raises. Verified live against the bench unit 2026-07-28.

Library use:

    from usb_relay4 import UsbRelay4
    with UsbRelay4() as board:          # auto-detects the STM32 VCP
        board.on(1)
        assert board.status()[1]
        board.all_off()

CLI use:

    usb_relay4.py on 1|2|3|4|all
    usb_relay4.py off 1|2|3|4|all
    usb_relay4.py on 1,2                  # gang: both in one write (~2 ms apart)
    usb_relay4.py pulse 2 --seconds 0.5   # on, hold, off (power-blip a DUT)
    usb_relay4.py status
"""

from __future__ import annotations

import argparse
import glob
import re
import sys
import time

import serial

START = 0xA0
ADDR_ALL = 0x0F
OP_OFF = 0x00
OP_ON = 0x01
OP_QUERY = 0x02
CHANNELS = (1, 2, 3, 4)

# STM32 virtual COM port as enumerated on this bench; --port overrides.
PORT_GLOB = '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort*'

_ACK_RE = re.compile(rb'CH(\d+):(ON|OFF)')


def _frame(addr: int, op: int) -> bytes:
    return bytes([START, addr, op, (START + addr + op) & 0xFF])


def find_port() -> str:
    matches = sorted(glob.glob(PORT_GLOB))
    if not matches:
        raise RuntimeError(f'no relay board found (looked for {PORT_GLOB})')
    if len(matches) > 1:
        raise RuntimeError(f'multiple STM32 VCP devices, pass --port: {matches}')
    return matches[0]


class UsbRelay4:
    """4-channel A0-protocol USB relay board (ack-verified)."""

    def __init__(self, port: str | None = None, timeout: float = 0.5):
        self._ser = serial.Serial(port or find_port(), 115200, timeout=timeout)
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> 'UsbRelay4':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _xfer(self, addr: int, op: int, expect_lines: int) -> dict[int, bool]:
        """Send one frame, collect `expect_lines` CHn:ON/OFF ack lines.

        The board occasionally swallows a command that lands right after it
        finished transmitting a reply, so an incomplete ack is retried once
        (every command is idempotent)."""
        for attempt in (1, 2):
            time.sleep(0.02)  # breathing room after the previous reply
            self._ser.reset_input_buffer()
            self._ser.write(_frame(addr, op))
            self._ser.flush()
            states: dict[int, bool] = {}
            deadline = time.monotonic() + 1.0
            while len(states) < expect_lines and time.monotonic() < deadline:
                line = self._ser.read_until(b'\n')
                m = _ACK_RE.search(line)
                if m:
                    states[int(m.group(1))] = m.group(2) == b'ON'
            if len(states) >= expect_lines:
                return states
        raise RuntimeError(
            f'relay board acked {len(states)}/{expect_lines} lines '
            f'for cmd addr=0x{addr:02X} op=0x{op:02X} (after retry)'
        )

    def set(self, channel: int, on: bool) -> None:
        if channel not in CHANNELS:
            raise ValueError(f'channel must be 1..4, got {channel}')
        ack = self._xfer(channel, OP_ON if on else OP_OFF, expect_lines=1)
        if ack.get(channel) != on:
            raise RuntimeError(f'relay {channel}: commanded {on}, board acked {ack}')

    def set_many(self, states: dict[int, bool]) -> None:
        """Switch several channels as near-simultaneously as the board
        allows: all command frames go out in ONE serial write, so the MCU
        actuates them back-to-back (~1 ms apart; relay mechanics dominate).
        Use for DPST-style gestures where both E-stop loops must move
        together. Acks for every channel are collected and verified."""
        bad = [ch for ch in states if ch not in CHANNELS]
        if bad:
            raise ValueError(f'channels must be 1..4, got {bad}')
        if not states:
            return
        time.sleep(0.02)
        self._ser.reset_input_buffer()
        self._ser.write(b''.join(_frame(ch, OP_ON if on else OP_OFF) for ch, on in states.items()))
        self._ser.flush()
        acked: dict[int, bool] = {}
        deadline = time.monotonic() + 1.0
        while len(acked) < len(states) and time.monotonic() < deadline:
            line = self._ser.read_until(b'\n')
            m = _ACK_RE.search(line)
            if m:
                acked[int(m.group(1))] = m.group(2) == b'ON'
        wrong = {ch: on for ch, on in states.items() if acked.get(ch) != on}
        if wrong:
            raise RuntimeError(f'set_many: bad/missing acks for {wrong} (got {acked})')

    def on(self, channel: int) -> None:
        self.set(channel, True)

    def off(self, channel: int) -> None:
        self.set(channel, False)

    def all_on(self) -> None:
        self._set_all(True)

    def all_off(self) -> None:
        self._set_all(False)

    def _set_all(self, on: bool) -> None:
        # The board acks all 8 firmware channels on an ALL command.
        ack = self._xfer(ADDR_ALL, OP_ON if on else OP_OFF, expect_lines=8)
        wrong = [ch for ch in CHANNELS if ack.get(ch) != on]
        if wrong:
            raise RuntimeError(f'all-{"on" if on else "off"}: bad ack for {wrong}: {ack}')

    def status(self) -> dict[int, bool]:
        """Query the board; returns {channel: is_on} for channels 1..4."""
        reply = self._xfer(ADDR_ALL, OP_QUERY, expect_lines=8)
        return {ch: reply[ch] for ch in CHANNELS if ch in reply}

    def pulse(self, channel: int, seconds: float) -> None:
        """On, hold, off — e.g. blip power to a device under test."""
        self.on(channel)
        try:
            time.sleep(seconds)
        finally:
            self.off(channel)


def _parse_channel(text: str) -> list[int] | None:
    """'all' -> None; '2' -> [2]; '1,2' -> [1, 2] (gang-switched)."""
    if text.lower() == 'all':
        return None
    try:
        chans = [int(part) for part in text.split(',')]
    except ValueError:
        raise argparse.ArgumentTypeError("channel must be 1..4, a list like 1,2, or 'all'")
    if len(set(chans)) != len(chans) or any(ch not in CHANNELS for ch in chans):
        raise argparse.ArgumentTypeError('channels must be unique and each 1..4')
    return chans


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', help='serial port (default: auto-detect STM32 VCP)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    for verb in ('on', 'off'):
        p = sub.add_parser(verb)
        p.add_argument('channel', type=_parse_channel, help="1..4, a list like 1,2, or 'all'")
    p = sub.add_parser('pulse')
    p.add_argument('channel', type=_parse_channel, help='1..4')
    p.add_argument('--seconds', type=float, default=1.0)
    sub.add_parser('status')
    args = ap.parse_args(argv)

    with UsbRelay4(args.port) as board:
        if args.cmd in ('on', 'off'):
            want_on = args.cmd == 'on'
            if args.channel is None:
                board.all_on() if want_on else board.all_off()
            else:
                board.set_many({ch: want_on for ch in args.channel})
        elif args.cmd == 'pulse':
            if args.channel is None or len(args.channel) != 1:
                ap.error('pulse needs a single channel')
            board.pulse(args.channel[0], args.seconds)
        for ch, on in sorted(board.status().items()):
            print(f'relay {ch}: {"ON" if on else "off"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
