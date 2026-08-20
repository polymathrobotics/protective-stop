#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""UDP NAT canary: wire-level truth about the network edge, independent of
any pstop firmware.

Sends STUN Binding Requests at a fixed cadence from ONE fixed local port to
two public STUN servers and logs three event classes with millisecond stamps:

  CANARY_REBIND    the reflexive (public) address CHANGED — a NAT rebind or a
                   multi-WAN failover. This is the event that killed run-46:
                   the office edge's public IP itself flipped, so every NAT
                   mapping and DERP TCP conn died at once. Firmware-side
                   gauges could not distinguish that from their own bugs; the
                   canary settled it in one hour.
  CANARY_GAP_OPEN  no STUN response from EITHER server for > gap_s — the
                   edge's UDP state (or the uplink) is actually down.
  CANARY_GAP_CLOSE responses resumed; the outage duration is logged.

Run it wherever a truth-reference is useful (a host on the same LAN as the
bench, plus ideally one off-site box). Example systemd unit:

  [Unit]
  Description=pstop UDP NAT canary
  After=network-online.target
  [Service]
  ExecStart=/usr/bin/python3 /opt/pstop/udp_canary.py --log /var/log/udp-canary.log
  Restart=always
  [Install]
  WantedBy=multi-user.target

Stdlib only; no third-party deps.
"""

import argparse
import os
import secrets
import socket
import struct
import time

STUN_MAGIC = 0x2112A442
DEFAULT_SERVERS = ['stun.l.google.com:19302', 'stun.cloudflare.com:3478']


def build_binding_request():
    txid = secrets.token_bytes(12)
    return struct.pack('!HHI12s', 0x0001, 0, STUN_MAGIC, txid), txid


def parse_mapped_address(data):
    """Return (txid, 'ip:port') from a Binding Response, or (None, None)."""
    if len(data) < 20 or struct.unpack('!H', data[0:2])[0] != 0x0101:
        return None, None
    txid = data[8:20]
    pos = 20
    while pos + 4 <= len(data):
        atype, alen = struct.unpack('!HH', data[pos : pos + 4])
        aval = data[pos + 4 : pos + 4 + alen]
        if atype == 0x0020 and len(aval) >= 8:  # XOR-MAPPED-ADDRESS
            port = struct.unpack('!H', aval[2:4])[0] ^ (STUN_MAGIC >> 16)
            raw = struct.unpack('!I', aval[4:8])[0] ^ STUN_MAGIC
            ip = socket.inet_ntoa(struct.pack('!I', raw))
            return txid, '%s:%d' % (ip, port)
        if atype == 0x0001 and len(aval) >= 8:  # MAPPED-ADDRESS (fallback)
            port = struct.unpack('!H', aval[2:4])[0]
            return txid, '%s:%d' % (socket.inet_ntoa(aval[4:8]), port)
        pos += 4 + alen + ((4 - alen % 4) % 4)
    return txid, None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=int, default=3479, help='fixed local UDP port (the mapping under test)')
    ap.add_argument('--interval', type=float, default=0.5, help='seconds between probes (per server, alternating)')
    ap.add_argument('--gap-s', type=float, default=3.0, help='silence threshold for CANARY_GAP_OPEN')
    ap.add_argument('--servers', nargs='*', default=DEFAULT_SERVERS)
    ap.add_argument('--log', default='-', help='log file path, - for stdout')
    args = ap.parse_args()

    out = open(args.log, 'a', buffering=1) if args.log != '-' else None

    def log(msg):
        line = '[%s] %s' % (time.strftime('%Y-%m-%dT%H:%M:%S') + ('.%03dZ' % (time.time() % 1 * 1000)), msg)
        print(line, flush=True)
        if out:
            out.write(line + os.linesep)

    resolved = []
    for server in args.servers:
        host, port = server.rsplit(':', 1)
        resolved.append((server, (socket.gethostbyname(host), int(port))))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(args.interval)

    mapped = {}  # per-server reflexive address: a symmetric NAT maps each
    # destination differently, so diffing across servers would flap REBIND
    # forever (review red). A real rebind/failover changes the SAME server's view.
    last_rx = time.monotonic()
    gap_open_at = None
    idx = 0
    log('CANARY_START port=%d servers=%s' % (args.port, [server for server, _ in resolved]))

    # Outstanding probes by txid: a reply arriving later than its own
    # iteration's recv window must still match (review red: a single late
    # reply otherwise desynced the lockstep recv forever — every iteration
    # consumed the PREVIOUS reply, txids never matched, and a permanent
    # false CANARY_GAP_OPEN resulted on a healthy network).
    outstanding = {}

    while True:
        iter_start = time.monotonic()
        req, txid = build_binding_request()
        name, addr = resolved[idx % len(resolved)]
        idx += 1
        try:
            sock.sendto(req, addr)
            outstanding[txid] = (name, time.monotonic())
            # Drain EVERY queued reply, matching each by its own txid. The
            # FIRST recv waits the full interval; subsequent drain checks use
            # a near-zero timeout so a consumed reply doesn't cost another
            # full interval and halve the probe cadence (review 🟡).
            first_recv = True
            while True:
                try:
                    sock.settimeout(args.interval if first_recv else 0.02)
                    first_recv = False
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    break
                rx_txid, got = parse_mapped_address(data)
                sent = outstanding.pop(rx_txid, None)
                if sent is None or got is None:
                    continue  # stale/foreign packet — ignore, do not desync
                src_name = sent[0]
                now = time.monotonic()
                if gap_open_at is not None:
                    log('CANARY_GAP_CLOSE after %.3fs' % (now - gap_open_at))
                    gap_open_at = None
                last_rx = now
                if src_name not in mapped:
                    mapped[src_name] = got
                    log('CANARY_BASELINE %s (via %s)' % (got, src_name))
                elif got != mapped[src_name]:
                    log('CANARY_REBIND %s -> %s (via %s)' % (mapped[src_name], got, src_name))
                    mapped[src_name] = got
        except OSError as exc:
            log('CANARY_SOCKET_ERR %s' % exc)
            time.sleep(1)
        stale = time.monotonic() - 5.0
        for key in [key for key, (_, at) in outstanding.items() if at < stale]:
            del outstanding[key]
        if gap_open_at is None and time.monotonic() - last_rx > args.gap_s:
            gap_open_at = last_rx
            log('CANARY_GAP_OPEN silence > %.1fs' % args.gap_s)
        # Pace by REMAINDER: a no-reply iteration already spent the full recv
        # timeout, so an unconditional extra sleep would halve the cadence in
        # exactly the outage windows the GAP events exist to time (review 🟡).
        time.sleep(max(0.0, args.interval - (time.monotonic() - iter_start)))


if __name__ == '__main__':
    main()
