#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
r"""Read-only, ONE-PHASE DUT + peer armed-soak collector (stdlib only).

Example (run a separate phase/output directory for each interface):
  ML_ADMIN_USER=admin ML_ADMIN_PASSWORD=... python3 tools/soak_stability.py \
    --dut http://10.43.0.122 --peer http://100.110.35.58:8895 \
    --machine-id 0x01020393 --remote-id 0x01D7F344 --slot 3 --iface usb \
    --dut-vpn-ip 100.75.70.74 --clean-seconds 14400 --poll-sec 1 \
    --out /path/to/usb-phase

Only GETs; no rearming, transport changes, settings dump, or coredump downloads.
Credentials come ONLY from ML_ADMIN_USER/ML_ADMIN_PASSWORD. An optional executable
--failure-hook receives one event.json path (no shell); its actions belong to the
outer controller. Exit 0 means the actual consecutive target passed; 2 means
incomplete/error, 130/143 mean interrupted. Restarts retain evidence but start a
new clean window, with five healthy seconds of recovery before any credit.

The nested peer schema was inspected live on 2026-09-10; adapter acknowledgment
does not establish that the real DUT is armed. --dut-vpn-ip pins its wire source;
if omitted, the pin is learned from DUT state.vpn_ip (never from peer traffic).
Wire-format adaptation belongs in the explicit normalize_peer_* helpers. Peer
/events.jsonl?after=N&limit=1000 must return complete newline-terminated records
after the initial status sequence baseline. Historical context is saved but never
credited or sent to repair hooks. /config.json machine/provenance/generation are
frozen; live.observed_epoch proves revalidation freshness. Clocks must be synced.
400 ms * 4 = 1600 ms is enforced as the packet/reply budget, below the 2 s machine
stop deadline. Sampled telemetry alone does not measure physical stop latency.
"""

import argparse
import base64
import copy
import fcntl
import hashlib
import ipaddress
import json
import math
import os
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

HTTP_TIMEOUT = 1.8
FRESH_SECONDS = 1.6
RECOVERY_SECONDS = 5.0
INITIALIZATION_SECONDS = 45.0
CLOCK_UNCERTAINTY = 0.1  # Maximum acknowledged host clock offset, seconds.
BOOT_DRIFT_PPM = 100
DIAGNOSTIC_COOLDOWN = 30.0
PEER_SCHEMA_CANDIDATE = 'nested-soak-2026-09-10'
LIVE_PEER_SCHEMA_ACK = PEER_SCHEMA_CANDIDATE
MAX_BODY = 1024 * 1024
MIN_FREE_BYTES = 256 * 1024 * 1024
HOOK_TIMEOUT = 10.0
HOOK_COOLDOWN = 60.0
FAULT_BITS = ('xcheck_fault', 'gpio_cfg_fault')
WARNING_COUNTERS = ('ml_reconnects', 'pstop_sf_txdrv', 'pstop_sf_txdrv_recovered')
FAILURE_COUNTERS = ('pstop_mismatch', 'eth_recoveries', 'relay_fault_a', 'relay_fault_b')
HEALTH_COUNTERS = ('presses', 'mismatch_events', 'nvs_flush_fails', 'dropped', 'boots', 'flashes', 'otas')


def timestamp(mono=None, epoch=None):
    epoch = time.time() if epoch is None else epoch
    return {
        'mono': time.monotonic() if mono is None else mono,
        'epoch': epoch,
        'utc': datetime.fromtimestamp(epoch, timezone.utc).isoformat(),
    }


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def load_json(text):
    def unique_object(pairs):
        obj = {}
        for key, value in pairs:
            require(key not in obj, 'duplicate JSON key: ' + key)
            obj[key] = value
        return obj

    return json.loads(text, object_pairs_hook=unique_object)


def number(value, label, integer=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
        or (integer and not isinstance(value, int))
    ):
        raise ValueError(label + ': expected nonnegative ' + ('integer' if integer else 'number'))
    return value


def identifier(value):
    if isinstance(value, str):
        return int(value, 16 if value.lower().startswith('0x') else 10)
    return number(value, 'id', integer=True)


def remote_identifier(value):
    """ROS dictionary/event IDs are eight HEX digits, including digits-only IDs."""
    if isinstance(value, str):
        require(re.fullmatch(r'(?:0[xX])?[0-9a-fA-F]{8}', value), 'peer: malformed remote id')
        return int(value, 16)
    return identifier(value)


def decimal_counter(value, label):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    return number(value, label, True)


def selected_remote(remotes, remote_id, label):
    require(isinstance(remotes, dict), label + ': dictionary required')
    matches = [value for key, value in remotes.items() if remote_identifier(key) == remote_id]
    require(len(matches) == 1 and isinstance(matches[0], dict), label + ': selected remote missing/duplicate')
    return matches[0]


def source_ip(source):
    require(isinstance(source, str), 'peer.wire: last_src missing')
    parsed = urllib.parse.urlsplit('//' + source)
    require(
        parsed.username is None and parsed.hostname and parsed.port and 0 < parsed.port <= 65535,
        'peer.wire: invalid last_src endpoint',
    )
    address = ipaddress.ip_address(parsed.hostname)
    require(
        address in ipaddress.ip_network('100.64.0.0/10') or address in ipaddress.ip_network('fd7a:115c:a1e0::/48'),
        'peer.wire: non-VPN/synthetic source',
    )
    return str(address)


def require(condition, label):
    if not condition:
        raise ValueError(label)


class Redactor:
    """Preserve complete payload structure, except secret-bearing values/text."""

    secret_key = re.compile(
        r'password|passwd|secret|token|authorization|cookie|private.?key|preshared|credential', re.I
    )
    text_secret = re.compile(
        r'(?i)(authorization\s*[:=]\s*(?:basic|bearer)\s+)[^\s,;"\']+|'
        r'((?:password|passwd|token|secret|private_key)\s*[:=]\s*)[^\s,;]+'
    )

    def __init__(self, user='', password=''):
        self.secrets = [
            s for s in (password, user, base64.b64encode(f'{user}:{password}'.encode()).decode() if user else '') if s
        ]

    def __call__(self, value):
        if isinstance(value, dict):
            return {k: '[REDACTED]' if self.secret_key.search(str(k)) else self(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self(v) for v in value]
        if isinstance(value, str):
            for secret in sorted(self.secrets, key=len, reverse=True):
                value = value.replace(secret, '[REDACTED]')
            return self.text_secret.sub(lambda m: (m.group(1) or m.group(2)) + '[REDACTED]', value)
        return value


class SchemaPending(ValueError):
    """The wire contract needs an explicit adapter, not a guessed fallback."""


def receipt_snapshot_age(epoch, received_epoch):
    delta = received_epoch - number(epoch, 'snapshot.epoch')
    require(delta >= -CLOCK_UNCERTAINTY, 'snapshot: future epoch beyond clock uncertainty')
    return max(0.0, delta + CLOCK_UNCERTAINTY)


def normalize_packet_message(msg):
    """The inspected observer exports decoded OK/STOP/BOND/UNBOND strings."""
    require(
        isinstance(msg, str) and msg.upper() in {'OK', 'STOP', 'BOND', 'UNBOND'},
        'peer.packet: unacknowledged message encoding',
    )
    return msg.upper()


def normalize_peer_watermarks(raw, received_epoch):
    """Parse the two single-threaded input/committed-journal publication fences.

    Source event_seq is global at its last emission, not a per-source counter.
    It is normal for one source's latest event to be newer than the OTHER input
    watermark. Reconcile all captured journal sequences, never compare that
    event's emission timestamp to the minimum input watermark.
    """
    require(isinstance(raw, dict), 'peer.status: object required')
    mono = number(raw.get('mono'), 'peer.mono')
    epoch = number(raw.get('epoch'), 'peer.epoch')
    snapshot_age = receipt_snapshot_age(epoch, received_epoch)
    require(snapshot_age <= FRESH_SECONDS, 'peer.status: stale epoch')
    events, marks = raw.get('events'), raw.get('watermarks')
    require(isinstance(events, dict) and isinstance(marks, dict), 'peer: events/watermarks missing')
    seqs = [
        number(events.get('seq'), 'peer.events.seq', True),
        number(marks.get('journal_seq_committed'), 'peer.journal_seq_committed', True),
    ]
    sources, ages = {}, {'snapshot': snapshot_age}
    for name in ('observer', 'ros_journal'):
        mark = marks.get(name)
        require(isinstance(mark, dict), 'peer.watermark.' + name + ': missing')
        m = number(mark.get('input_mono'), 'peer.watermark.input_mono')
        e = number(mark.get('input_epoch'), 'peer.watermark.input_epoch')
        require(m <= mono + max(0, received_epoch - epoch) + 2 * CLOCK_UNCERTAINTY, 'peer: future processing watermark')
        age = max(snapshot_age + max(0, mono - m), receipt_snapshot_age(e, received_epoch))
        require(age <= FRESH_SECONDS, 'peer: processing watermark stale')
        emitted = number(mark.get('event_seq'), 'peer.watermark.event_seq', True)
        journal = number(mark.get('journal_seq'), 'peer.watermark.journal_seq', True)
        require(emitted <= journal, 'peer: uncommitted source event')
        seqs.extend((emitted, journal))
        sources[name] = {
            'mono': m,
            'epoch': e,
            'event_seq': emitted,
            'journal_seq': journal,
            'events_emitted': number(mark.get('events_emitted'), 'peer.events_emitted', True),
        }
        ages['watermark.' + name] = age
    ages['watermark'] = max(ages['watermark.observer'], ages['watermark.ros_journal'])
    return {
        'mono': mono,
        'epoch': epoch,
        'event_seq': max(seqs),
        'status_seq': events['seq'],
        'processed_mono': min(s['mono'] for s in sources.values()),
        'watermarks': sources,
        'ages': ages,
    }


def normalize_peer_status(raw, remote_id, received_epoch, dut_vpn_ip=None, machine_id=None):
    """Adapter for actual /status.json inspected 2026-09-10 (nested schema).

    Node ages are relative to ros_snapshot_epoch; collector.node.age_s is also
    that cached topic age. Observer/ROS publisher ages and their two committed
    input watermarks are checked independently of the fresh HTTP envelope.
    All wire timing/counts are from the selected remote, NEVER wire totals or
    aggregate last-packet times. ROS age/rtt/rebond placeholders are not used.
    Sparse gap histogram bins represent zero until first occurrence.
    """
    result = normalize_peer_watermarks(raw, received_epoch)
    mono, ages = result['mono'], result['ages']
    node, wire, collectors = raw.get('node'), raw.get('wire'), raw.get('collector')
    require(all(isinstance(value, dict) for value in (node, wire, collectors)), 'peer: nested status missing')
    machine = raw.get('machine')
    require(
        isinstance(machine, dict) and isinstance(machine.get('bound'), str) and machine['bound'],
        'peer.machine: UDP binding missing',
    )
    port = number(machine.get('udp_port'), 'peer.machine.udp_port', True)
    require(0 < port <= 65535, 'peer.machine: invalid port')
    if machine_id is not None:
        require(remote_identifier(machine.get('machine_id')) == machine_id, 'peer.machine: wrong machine id')
    require(wire.get('silence_budget_ms') == 1600, 'peer: wrong silence budget')
    require(
        node.get('armed') is True and node.get('state') == 'ACTIVE' and node.get('hb_stop') is False,
        'peer: not positively armed ACTIVE / heartbeat stop',
    )
    require(
        node.get('topic_stalled') is False and node.get('hb_stamp_stalled') is False,
        'peer.node: topic/heartbeat stamp stalled',
    )
    require(isinstance(node.get('status_reason'), str), 'peer.node.status_reason missing')
    ros_remote = selected_remote(node.get('remotes'), remote_id, 'peer.node.remotes')
    require(
        ros_remote.get('stop_only') is False
        and ros_remote.get('bond_state') == 'bonded'
        and ros_remote.get('in_use') is True,
        'peer.node: selected remote not active bonded operator',
    )
    selected = selected_remote(wire.get('remotes'), remote_id, 'peer.wire.remotes')
    require(remote_identifier(selected.get('remote_id')) == remote_id, 'peer.wire: selected identity mismatch')
    physical = source_ip(selected.get('last_src'))
    if dut_vpn_ip is not None:
        require(physical == str(ipaddress.ip_address(dut_vpn_ip)), 'peer.wire: selected DUT VPN source mismatch')
    require(selected.get('last_role') == 'operator', 'peer.wire: selected remote not operator')
    require(selected.get('silent') is False and selected.get('stalled') is False, 'peer.wire: silent/stalled')
    require(
        number(selected.get('consecutive_no_reply'), 'peer.wire.consecutive_no_reply', True) < 3,
        'peer.wire: three consecutive unanswered packets',
    )
    failures, progress = [], {}

    def check_age(label, age):
        require(age <= FRESH_SECONDS, label + ': stale observation')
        ages[label] = age

    def snapshot_age(obj, epoch_key, age_key):
        return max(
            receipt_snapshot_age(obj.get(epoch_key), received_epoch),
            ages['snapshot'] + number(obj.get(age_key), 'peer.' + age_key),
        )

    ros_age = snapshot_age(node, 'ros_snapshot_epoch', 'ros_snapshot_age_s')
    wire_age = snapshot_age(wire, 'snapshot_epoch', 'snapshot_age_s')
    check_age('ros_snapshot', ros_age)
    check_age('wire_snapshot', wire_age)
    counts = node.get('topic_counts')
    require(isinstance(counts, dict), 'peer.node.topic_counts missing')
    for name, age_key in (('state', 'topic_age_s'), ('remotes', 'remotes_age_s'), ('hb', 'hb_age_s')):
        check_age('topic.' + name, ros_age + number(node.get(age_key), 'peer.node.' + age_key))
        progress['topic.' + name] = number(counts.get(name), 'peer.topic_count.' + name, True)
    stamp = number(node.get('hb_last_stamp'), 'peer.node.hb_last_stamp')
    check_age('hb_stamp', receipt_snapshot_age(stamp, received_epoch))
    progress['hb_stamp'] = stamp
    for direction, time_key, message_key in (
        ('rx', 'last_rx_epoch', 'last_msg'),
        ('tx', 'machine_last_tx_epoch', 'machine_last_msg'),
    ):
        when = number(selected.get(time_key), 'peer.wire.' + time_key)
        require(when <= wire['snapshot_epoch'] + 0.001, 'peer.wire: packet newer than observer snapshot')
        check_age('packet.' + direction, receipt_snapshot_age(when, received_epoch))
        progress['packet.' + direction] = when
        msg = normalize_packet_message(selected.get(message_key))
        if msg != 'OK':
            failures.append('peer.packet.' + direction + ': ' + msg)
    rx_mono = number(selected.get('last_rx_mono'), 'peer.wire.last_rx_mono')
    require(
        rx_mono <= result['watermarks']['observer']['mono'] + 0.001, 'peer.wire: RX beyond processed observer input'
    )
    check_age('packet.rx', max(ages['packet.rx'], ages['snapshot'] + mono - rx_mono))
    number(selected.get('last_counter'), 'peer.wire.last_counter', True)  # Protocol counter may wrap.
    counters = {
        'rx': number(selected.get('rx'), 'peer.wire.rx', True),
        'tx': number(selected.get('machine_tx'), 'peer.wire.machine_tx', True),
    }
    require(counters['rx'] > 0 and counters['tx'] > 0, 'peer.wire: no selected traffic')
    for name in ('bonds', 'crc_fail', 'counter_gaps', 'no_reply', 'stale_echo'):
        counters[name] = number(selected.get(name), 'peer.wire.' + name, True)
    histogram = selected.get('gap_hist_ms')
    require(isinstance(histogram, dict), 'peer.wire.gap_hist_ms missing')
    for key, count in histogram.items():
        require(key in {'<=250', '<=400', '<=800', '<=1600', '>1600'}, 'peer.wire: unknown gap bin')
        number(count, 'peer.wire.gap_hist_ms.' + key, True)
    counters['gap_over_budget'] = histogram.get('>1600', 0)
    maximum = number(selected.get('max_gap_ms'), 'peer.wire.max_gap_ms')
    identities, passive = {}, {}
    for name in ('node', 'observer', 'ros_journal', 'pcap', 'status_server'):
        item = collectors.get(name)
        require(
            isinstance(item, dict) and item.get('alive') is True and item.get('unit_state') == 'active',
            'peer.collector.' + name + ': not alive/active',
        )
        pid = decimal_counter(item.get('pid'), 'peer.collector.' + name + '.pid')
        require(pid > 0, 'peer.collector: invalid PID')
        identities[name] = pid
        counters['collector.' + name + '.restarts'] = decimal_counter(item.get('restarts'), 'peer.collector.restarts')
        if name == 'pcap' and 'age_s' in item and item['age_s'] is None:
            raise SchemaPending('peer.pcap: awaiting first capture timestamp')
        age = ages['snapshot'] + number(item.get('age_s'), 'peer.collector.' + name + '.age_s')
        if name == 'node':
            age = max(age, ages['topic.state'])
        elif name == 'observer':
            age = max(age, wire_age)
        elif name == 'ros_journal':
            age = max(age, ros_age)
        check_age('collector.' + name, age)
    # dut_poll is a 5-second ancillary reader, not a critical producer.
    unit = node.get('unit')
    require(isinstance(unit, dict), 'peer.node.unit missing')
    require(
        decimal_counter(unit.get('MainPID'), 'peer.node.MainPID') == identities['node']
        and decimal_counter(unit.get('NRestarts'), 'peer.node.NRestarts') == counters['collector.node.restarts'],
        'peer.node: inconsistent unit identity',
    )
    for name, watermark in result['watermarks'].items():
        passive[name + '.events_emitted'] = watermark['events_emitted']
    result.update(
        pid=identities['node'],
        identities=identities,
        progress=progress,
        counters=counters,
        failures=failures,
        passive=passive,
        max_gap_ms=maximum,
        physical_source=physical,
        machine_port=port,
    )
    return result


def normalize_peer_config(raw, machine_id, remote_id=None, slot=3):
    """Live immutable generation/machine/provenance plus fresh hash revalidation.

    Top-level observed_* is the original capture, served_* just the HTTP reply.
    Only live.observed_epoch/mono establishes when hashes/drift were rechecked.
    """
    require(isinstance(raw, dict), 'peer.config: object required')
    machine, provenance, live = raw.get('machine'), raw.get('provenance'), raw.get('live')
    require(all(isinstance(v, dict) for v in (machine, provenance, live)), 'peer.config: nested sections missing')
    require(
        remote_identifier(machine.get('machine_id')) == machine_id and machine.get('machine_id_dec') == machine_id,
        'peer.config: wrong machine id',
    )
    require(machine.get('dut_slot') == slot, 'peer.config: wrong DUT slot')
    operators = machine.get('operators')
    require(
        isinstance(operators, list) and machine.get('default_stop_only') is True, 'peer.config: operator policy missing'
    )
    if remote_id is not None:
        require(
            sum(remote_identifier(value) == remote_id for value in operators) == 1,
            'peer.config: DUT operator absent/duplicate',
        )
    timing = machine.get('timing')
    require(
        isinstance(timing, dict)
        and timing.get('heartbeat_ms') == 400
        and timing.get('max_missed') == 4
        and timing.get('min_stop_ms') == 500
        and machine.get('silence_budget_ms') == 1600,
        'peer.config: expected 400 ms * 4, min_stop_ms=500',
    )
    require(0 < number(machine.get('udp_port'), 'peer.config.udp_port', True) <= 65535, 'peer.config: bad UDP port')
    protocol = provenance.get('pstop_c_protocol')
    require(
        isinstance(protocol, dict)
        and identifier(protocol.get('PSTOP_VERSION')) == 2
        and protocol.get('PSTOP_MESSAGE_SIZE') == 48,
        'peer.config: wrong wire protocol',
    )
    generation = raw.get('config_generation')
    require(
        isinstance(generation, str) and re.fullmatch(r'[0-9a-f]{64}', generation),
        'peer.config: generation hash missing',
    )
    require(
        live.get('config_generation') == generation and live.get('drift') is False,
        'peer.config: live drift/generation mismatch',
    )
    for key, size in (('commit', 40), ('binary_sha256', 64), ('params_sha256', 64)):
        value = provenance.get(key)
        require(
            isinstance(value, str) and re.fullmatch(r'[0-9a-fA-F]{' + str(size) + '}', value),
            'peer.config: invalid ' + key,
        )
        require(live.get(key) == value, 'peer.config: live ' + key + ' mismatch')
    for name in ('units_sha256', 'scripts_sha256'):
        hashes = provenance.get(name)
        require(
            isinstance(hashes, dict)
            and hashes
            and all(isinstance(v, str) and re.fullmatch(r'[0-9a-fA-F]{64}', v) for v in hashes.values()),
            'peer.config: missing ' + name,
        )
    for key in ('observed_epoch', 'observed_mono'):
        number(live.get(key), 'peer.config.live.' + key)
    # These are the exact immutable sections, including nested params text and
    # all binary/unit/script hashes. Observation times never affect the digest.
    config = {key: copy.deepcopy(raw[key]) for key in ('config_generation', 'machine', 'provenance')}
    return {'sha256': digest(config), 'snapshot': config, 'stop_budget_ms': 1600}


class BootClock:
    """Fixed whole-epoch anchor with request uncertainty and bounded clock drift.

    Samples cannot walk the anchor forward by a small error on every request.
    precision=1 handles the lifetime health counter's integer-second rounding.
    """

    def __init__(self, precision=0.001):
        self.precision = precision
        self.anchor = None
        self.previous = None
        self.certified = False

    def observe(self, uptime, record):
        start, end = record['start']['mono'], record['end']['mono']
        if self.anchor is None:
            self.anchor = (start - uptime - self.precision, end - uptime, start)
        low, high, origin = self.anchor
        drift = 0.05 + max(0, end - origin) * BOOT_DRIFT_PPM / 1e6
        require(
            start - uptime - self.precision <= high + drift and end - uptime >= low - drift,
            'whole-epoch uptime drift exceeds request uncertainty',
        )
        if self.previous is not None:
            require(uptime > self.previous, 'uptime reset/stale snapshot')
            self.certified = True
        self.previous = uptime
        return low + uptime - drift


class CounterBank:
    """Keep valid fields independently: disappearance never erases a baseline."""

    def __init__(self):
        self.values = {}

    def observe(self, prefix, raw, keys, warnings=()):
        failures, notices = [], []
        for key in keys:
            label = prefix + '.' + key
            try:
                value = number(raw.get(key), label, True)
            except ValueError:
                failures.append(label + ': missing/invalid counter coverage')
                continue
            before = self.values.get(label)
            if before is not None:
                if value < before:
                    failures.append(label + ': counter reset')
                elif value > before:
                    (notices if key in warnings else failures).append(label + ': increment')
            self.values[label] = value
        return failures, notices


class Progress:
    def __init__(self):
        self.values = {}

    def observe(self, key, value, now):
        old = self.values.get(key)
        changed = old is not None and value > old[0]
        if old is None or value != old[0]:
            self.values[key] = (value, now, changed)
        if old is not None and value < old[0]:
            return key + ': counter/time reset'
        current = self.values[key]
        if now - current[1] > FRESH_SECONDS:
            return key + ': no progress'
        if not current[2]:
            return key + ': progress not yet demonstrated'
        return None


class CleanGate:
    """Credit only jointly observed monotonic time; never extrapolate to now."""

    def __init__(self, target, recovery=RECOVERY_SECONDS):
        self.target = target
        self.recovery = recovery
        self.healthy_since = None
        self.clean_start = None
        self.clean_s = 0.0
        self.state = 'WAITING'
        self.last_frontier = None

    def update(self, frontier, reasons, ready=True):
        if reasons:
            self.healthy_since = self.clean_start = None
            self.last_frontier = None
            self.clean_s = 0.0
            self.state = 'WAITING'
        elif ready:
            if self.last_frontier is not None and frontier < self.last_frontier:
                return self.update(frontier, ['collector.monotonic_reset'])
            if self.healthy_since is None:
                self.healthy_since = frontier
            if self.clean_start is None and frontier - self.healthy_since >= self.recovery:
                # Recovery is deliberately excluded from credited clean time.
                self.clean_start = frontier
                self.state = 'RUNNING'
            if self.clean_start is not None:
                self.clean_s = max(0.0, frontier - self.clean_start)
                self.state = 'RUNNING'  # Only Evidence's closing checks may pass.
            self.last_frontier = frontier
        return self.state


def normalize_peer_event(event, remote_id=None, credit_epoch=None, negative_evidence=True):
    """Trust general severity; contrary wire/stop and over-budget gaps override.

    Known aliases are explicit. A new informational kind is not a schema error;
    absent/ambiguous severity, kind or timestamp still is. Numeric wire message
    encodings and nested duration formats await actual live JSON adaptation.
    """
    severity = event.get('severity', event.get('sev'))
    levels = {
        'info': 'info',
        'warning': 'warning',
        'warn': 'warning',
        'failure': 'failure',
        'error': 'failure',
        'critical': 'failure',
        'fatal': 'failure',
    }
    require(isinstance(severity, str) and severity.lower() in levels, 'peer.event: severity missing/unknown')
    level = levels[severity.lower()]
    if 'severity' in event and 'sev' in event:
        require(levels.get(str(event['sev']).lower()) == level, 'peer.event: conflicting severity')
    kind = event.get('kind', event.get('type'))
    require(isinstance(kind, str) and bool(kind), 'peer.event: kind missing')
    if 'kind' in event and 'type' in event:
        require(event['kind'] == event['type'], 'peer.event: conflicting kind')
    number(event.get('epoch'), 'peer.event.epoch')
    number(event.get('mono'), 'peer.event.mono')
    data = event.get('data', {})
    require(isinstance(data, dict), 'peer.event: data must be an object')
    upper = kind.upper()
    global_types = {
        'MACHINE_UNEXPECTED_STOP',
        'MACHINE_UNSTABLE',
        'ROS_TOPIC_STALL',
        'HB_STAMP_STALL',
        'BACKEND_DISAGREE',
        'DUT_REBOOT',
        'DUT_UNREACHABLE',
        'DUT_LOCKSTEP_MISMATCH',
        'EXIT',
        'EVIDENCE_COLLECTED',
    }
    if remote_id is not None and 'remote_id' in data and upper not in global_types:
        if remote_identifier(data['remote_id']) != remote_id:
            return 'info', 'peer.other_remote.' + kind
    tokens = set(re.split(r'[^a-z0-9]+', kind.lower()))
    negative_kinds = {
        'stop',
        'disconnect',
        'disarmed',
        'bond',
        'unbond',
        'rebond',
        'restart',
        'packet_stop',
        'packet_bond',
        'packet_unbond',
        'wire_stop',
        'wire_bond',
        'wire_unbond',
        'machine_stop',
        'machine_disconnect',
        'node_restart',
    }
    if kind.lower() in negative_kinds:
        level = 'failure'
    failure_types = {
        'REMOTE_SILENT',
        'REMOTE_DROPPED',
        'MACHINE_UNEXPECTED_STOP',
        'MACHINE_UNSTABLE',
        'MACHINE_STALL',
        'MACHINE_STALE_ECHO',
        'ROS_TOPIC_STALL',
        'HB_STAMP_STALL',
        'BACKEND_DISAGREE',
        'TX_CRC_FAIL',
        'DUT_REBOOT',
        'DUT_UNREACHABLE',
        'DUT_LOCKSTEP_MISMATCH',
        'EXIT',
    }
    if upper in failure_types or (
        upper == 'EVIDENCE_COLLECTED' and 'EXIT' in re.split(r'[^A-Z0-9]+', str(data.get('reason', '')).upper())
    ):
        level = 'failure'
    within_window = negative_evidence and (credit_epoch is None or event['epoch'] >= credit_epoch - CLOCK_UNCERTAINTY)
    if within_window:
        for obj in (event, data):
            for key in ('msg', 'last_msg', 'machine_last_msg', 'new_msg'):
                if key in obj and normalize_packet_message(obj[key]) != 'OK':
                    level = 'failure'
            if any(word in upper for word in ('WIRE', 'MESSAGE', 'MSG')) and obj.get('to') in (
                'STOP',
                'BOND',
                'UNBOND',
            ):
                level = 'failure'
    if 'gap' in tokens:
        for obj in (event, data):
            for key, scale in (('gap_s', 1), ('gap_ms', 0.001), ('duration_s', 1), ('duration_ms', 0.001)):
                if key in obj and number(obj[key], 'peer.event.' + key) * scale > FRESH_SECONDS:
                    level = 'failure'
    return level, 'peer.' + kind


class PeerEvents:
    def __init__(self, baseline_required=False):
        self.cursor = 0
        self.hashes = {}
        self.last_mono = None
        self.initialized = not baseline_required
        self.initial_cursor = None

    def baseline(self, seq):
        require(not self.initialized, 'peer.events: baseline already established')
        self.cursor = number(seq, 'peer.events.baseline', True)
        self.initial_cursor = self.cursor
        self.initialized = True

    def consume(self, events, remote_id=None, credit_epoch=None, negative_evidence=True, received_epoch=None):
        """Return one retained local event per new peer event, plus gap events."""
        require(isinstance(events, list), 'peer.events: list required')
        if not self.initialized:
            return []  # Full raw response is retained; no pre-baseline hooks/counts.
        outcomes = []
        previous = 0
        for event in events:
            try:
                require(isinstance(event, dict), 'peer.event: object required')
                seq = number(event.get('seq'), 'peer.event.seq', True)
                require(seq > previous, 'peer.events: unordered/duplicate sequence')
            except (ValueError, TypeError, OverflowError):
                # Preserve earlier outcomes and the malformed item. An unknown
                # sequence cannot advance the cursor and cannot silently skip
                # the rest of this batch. The next request retries from here.
                outcomes.append(('failure', 'peer.event_schema', event))
                break
            previous = seq
            fingerprint = digest(event)
            if seq <= self.cursor:
                if seq in self.hashes and self.hashes[seq] != fingerprint:
                    outcomes.append(('failure', 'peer.event_mutated', event))
                continue
            if seq != self.cursor + 1:
                outcomes.append(('failure', 'peer.event_gap', {'expected': self.cursor + 1, 'observed': seq}))
            try:
                level, reason = normalize_peer_event(event, remote_id, credit_epoch, negative_evidence)
                if received_epoch is not None:
                    require(event['epoch'] <= received_epoch + CLOCK_UNCERTAINTY, 'peer.event: future epoch')
            except (ValueError, TypeError, OverflowError):
                level, reason = 'failure', 'peer.event_schema'
            outcomes.append((level, reason, event))
            if isinstance(event.get('mono'), (int, float)) and not isinstance(event.get('mono'), bool):
                if self.last_mono is not None and event['mono'] < self.last_mono:
                    outcomes.append(('failure', 'peer.event_clock_reset', event))
                self.last_mono = event['mono']
            self.cursor = seq
            self.hashes[seq] = fingerprint
            if len(self.hashes) > 2048:
                del self.hashes[min(self.hashes)]
        return outcomes


class Evidence:
    """Mutable sample validation plus immutable phase provenance."""

    def __init__(self, args, provenance=None):
        self.args = args
        self.provenance = copy.deepcopy(provenance or {})
        self.latest = {}
        self.reasons = {}
        self.waiting = {}
        self.started = None
        self.initialized = False
        self.wait_reasons = []
        self.receipt_holds = []
        self.generated = {}
        self.counters = CounterBank()
        self.dut_clock = BootClock()
        self.health_clock = BootClock(precision=1.0)
        self.health_barrier = -math.inf
        self.health_boot_min = None
        self.boot_health_baseline = None
        self.progress = Progress()
        self.previous_dut = None
        self.previous_peer = None
        self.previous_health = None
        self.last_warnings = {}
        self.journal = PeerEvents(baseline_required=True)
        self.expected_vpn = getattr(args, 'dut_vpn_ip', None)
        self.gate = CleanGate(args.clean_seconds)
        self.last_sync = None
        self.closing_cutoff = None
        self.closing_verified = {}
        self.closing_fence = None
        self.closing_witness = None

    def invalidate(self, now, reasons):
        self.gate.update(now, reasons)
        self.closing_cutoff = self.closing_fence = None
        self.closing_verified = {}
        self.closing_witness = None

    def capabilities(self, source, raw, candidates, required=()):
        observed = set(raw) & set(candidates)
        caps = self.provenance.setdefault('capabilities', {})
        expected = set(caps.setdefault(source, sorted(observed | set(required))))
        errors = []
        if observed != expected:
            errors.append(source + ': capability coverage changed/missing')
        return expected, errors

    def freeze(self, name, value):
        if name not in self.provenance:
            self.provenance[name] = copy.deepcopy(value)
        require(self.provenance[name] == value, name + ': phase provenance changed')

    def dut(self, raw, record):
        require(isinstance(raw, dict), 'dut.state: object required')
        now = record['end']['mono']
        failures, warnings = [], []
        if 'vpn_ip' in raw:
            vpn = str(ipaddress.ip_address(raw['vpn_ip']))
            if self.expected_vpn is None:
                try:
                    self.expected_vpn = vpn_ip(vpn)
                except argparse.ArgumentTypeError as exc:
                    raise ValueError('dut: VPN identity not available') from exc
            require(vpn == self.expected_vpn, 'dut: VPN identity changed/mismatch')
        if self.expected_vpn is not None:
            self.freeze('dut_vpn_ip', self.expected_vpn)
        capabilities, missing = self.capabilities(
            'dut', raw, FAILURE_COUNTERS + WARNING_COUNTERS + ('health', 'role', 'relay_stop'), ('pstop_mismatch',)
        )
        failures.extend(missing)
        counter_keys = capabilities & set(FAILURE_COUNTERS + WARNING_COUNTERS)
        bad, notices = self.counters.observe('dut', raw, counter_keys, WARNING_COUNTERS)
        failures.extend(bad)
        warnings.extend(notices)
        slots = raw.get('pstop_machines')
        require(isinstance(slots, list) and len(slots) > self.args.slot, 'dut: selected slot missing')
        slot = slots[self.args.slot]
        require(
            isinstance(slot, dict) and slot.get('cfg') == 1 and identifier(slot.get('id')) == self.args.machine_id,
            'dut: selected slot id/config mismatch',
        )
        require(
            sum(
                isinstance(s, dict) and s.get('cfg') == 1 and identifier(s.get('id')) == self.args.machine_id
                for s in slots
            )
            == 1,
            'dut: duplicate selected machine',
        )
        for key in ('fw_ver', 'fw_sha'):
            require(isinstance(raw.get(key), str) and bool(raw[key].strip()), 'dut: missing ' + key)
        require(re.fullmatch(r'[0-9a-fA-F]{7,64}', raw['fw_sha']), 'dut: firmware hash missing/unknown')
        uptime = number(raw.get('uptime_ms'), 'dut.uptime_ms', True)
        number(raw.get('boot_count'), 'dut.boot_count', True)
        number(raw.get('reset_reason'), 'dut.reset_reason', True)
        self.freeze('firmware', {k: raw[k] for k in ('fw_ver', 'fw_sha')})
        number(slot.get('ip'), 'dut.slot.ip', True)
        require(0 < number(slot.get('port'), 'dut.slot.port', True) <= 65535, 'dut.slot: bad port')
        require(slot.get('hb_ms') == 400, 'dut.slot: expected heartbeat 400 ms')
        self.freeze('dut_slot', {k: slot[k] for k in ('cfg', 'id', 'ip', 'port', 'hb_ms')})
        expected = {
            'active_iface': 2 if self.args.iface == 'usb' else 1,
            'eth_en': 0 if self.args.iface == 'usb' else 1,
            'wifi_en': 0,
            'l0': 85,
            'l1': 85,
            'e_hi0': 1,
            'e_lo0': 1,
            'e_hi1': 1,
            'e_lo1': 1,
        }
        if self.args.iface == 'ethernet':
            expected['eth_link'] = 1
            # USB may remain enabled for read-only management during Ethernet;
            # active_iface, eth_en/link, and WiFi exclusion identify the path.
            require(raw.get('usbncm_en') in (0, 1), 'dut.usbncm_en missing')
        else:
            expected['usbncm_en'] = 1
            require(raw.get('eth_link') in (0, 1), 'dut.eth_link missing')
        for key, value in expected.items():
            if raw.get(key) != value:
                failures.append('dut.' + key + ': unexpected')
        if raw.get('wifi_ip') not in (0, '0.0.0.0'):
            failures.append('dut: WiFi association/unknown IP')
        if 'role' in raw and raw['role'] != 'operator':
            failures.append('dut: not operator')
        if slot.get('state') != 2:
            failures.append('dut.slot: not bonded')
        if 'relay_stop' in raw and raw['relay_stop'] != 0:
            failures.append('dut.relay_stop: active')
        for key in FAULT_BITS:
            if raw.get(key) != 0:
                failures.append('dut.' + key + ': fault/missing')
        if 'health' in raw:
            level = number(raw['health'], 'dut.health', True)
            if level >= 2:
                failures.append('dut.health: critical')
            elif level == 1:
                warnings.append('dut.health: warning')
        reply = number(slot.get('last_reply_ms'), 'dut.slot.last_reply_ms', True)
        if reply <= 0 or not 0 <= uptime - reply <= 1600:
            failures.append('dut.slot: stale/future reply')
        old = self.previous_dut
        reboot = old and (
            raw['boot_count'] != old['boot_count']
            or raw['reset_reason'] != old['reset_reason']
            or uptime < old['uptime_ms']
        )
        if reboot:
            failures.append('dut: reboot/reset reason changed')
            self.dut_clock = BootClock()
            self.health_clock = BootClock(precision=1.0)
            self.health_barrier = now
            if self.boot_health_baseline is not None:
                self.health_boot_min = self.boot_health_baseline + 1
            elif self.health_boot_min is not None:
                self.health_boot_min += 1
            self.boot_health_baseline = None
            self.generated.pop('health', None)
        try:
            self.generated['dut_state'] = self.dut_clock.observe(uptime / 1000, record)
        except ValueError as exc:
            failures.append('dut: ' + str(exc))
        if old:
            if not reboot and uptime <= old['uptime_ms']:
                failures.append('dut: uptime reset/stale JSON')
        bad, _ = self.counters.observe('dut.slot', slot, ('send_fail', 'rebonds'))
        failures.extend(bad)
        for key in ('sent', 'replies', 'send_fail', 'rebonds'):
            try:
                value = number(slot.get(key), 'dut.slot.' + key, True)
            except ValueError as exc:
                failures.append(str(exc))
                continue
            if key in ('sent', 'replies'):
                problem = self.progress.observe('dut.' + key, value, now)
                if problem:
                    failures.append(problem)
        self.previous_dut = dict(raw, _slot=dict(slot), _mono=now)
        return failures, warnings

    def peer(self, raw, record):
        now = record['end']['mono']
        if self.expected_vpn is None:
            raise SchemaPending('peer: awaiting DUT VPN identity or --dut-vpn-ip')
        normalized = normalize_peer_status(
            raw, self.args.remote_id, record['end']['epoch'], self.expected_vpn, self.args.machine_id
        )
        failures, warnings = list(normalized['failures']), []
        caps, missing = self.capabilities('peer_counters', normalized['counters'], normalized['counters'])
        failures.extend(missing)
        old = self.previous_peer
        if old:
            if normalized['pid'] != old['pid']:
                failures.append('peer: node restart')
            if normalized['identities'] != old['identities']:
                failures.append('peer: collector process identity changed')
            if normalized['mono'] <= old['mono'] or normalized['epoch'] <= old['epoch']:
                failures.append('peer: stale JSON/clock reset')
            if normalized['processed_mono'] < old['processed_mono']:
                failures.append('peer: processing watermark regressed')
            if normalized['event_seq'] < old['event_seq']:
                failures.append('peer: event sequence reset')
                self.journal = PeerEvents()
            if abs(normalized['mono'] - old['mono'] - (now - old['_mono'])) > FRESH_SECONDS:
                failures.append('peer: clock discontinuity')
        for name, value in dict(normalized['progress'], **{k: normalized['counters'][k] for k in ('rx', 'tx')}).items():
            problem = self.progress.observe('peer.' + name, value, now)
            if problem:
                failures.append(problem)
        if old and normalized['max_gap_ms'] > max(1600, old['max_gap_ms']):
            failures.append('peer: new over-budget RX gap')
        for key, value in normalized['passive'].items():
            previous = self.counters.values.get('peer.passive.' + key)
            if previous is not None and value < previous:
                failures.append('peer.' + key + ': counter reset')
            self.counters.values['peer.passive.' + key] = value
        bad, notices = self.counters.observe(
            'peer', normalized['counters'], caps - {'rx', 'tx'}, ('bonds', 'counter_gaps', 'no_reply')
        )
        failures.extend(bad)
        warnings.extend(notices)
        self.generated['peer_status'] = now - normalized['ages']['snapshot']
        self.previous_peer = dict(normalized, _mono=now)
        return failures, warnings

    def health(self, raw, record):
        require(isinstance(raw, dict) and raw.get('ok') is True, 'dut.health: invalid snapshot')
        require(record['start']['mono'] > self.health_barrier, 'health: snapshot predates DUT boot barrier')
        if self.health_boot_min is not None:
            require(
                number(raw.get('boots'), 'health.boots', True) >= self.health_boot_min,
                'health: stale pre-reboot boots; cannot establish baseline',
            )
        keys, failures = self.capabilities(
            'health', raw, HEALTH_COUNTERS + ('uptime_s',), ('presses', 'boots', 'uptime_s')
        )
        bad, warnings = self.counters.observe('health', raw, keys - {'uptime_s'})
        failures.extend(bad)
        level = number(raw.get('level'), 'dut.health.level', True)
        if level >= 2:
            failures.append('dut.health: critical')
        elif level == 1:
            warnings.append('dut.health: warning')
        uptime = number(raw.get('uptime_s'), 'health.uptime_s', True)
        try:
            self.generated['health'] = self.health_clock.observe(uptime, record)
        except ValueError as exc:
            failures.append('health: ' + str(exc))
        if self.boot_health_baseline is None:
            self.boot_health_baseline = raw.get('boots')
        self.previous_health = raw
        return failures, warnings

    def accept(self, record):
        source, now = record['source'], record['end']['mono']
        if self.started is None:
            self.started = record['start']['mono']
        self.latest[source] = record
        failures, warnings, events = [], [], []
        if record.get('error'):
            failures.append(source + ': HTTP ' + record['error'])
        else:
            raw = record['data']
            try:
                if source == 'dut_state':
                    failures, warnings = self.dut(raw, record)
                elif source == 'peer_status':
                    if not self.journal.initialized:
                        header = normalize_peer_watermarks(raw, record['end']['epoch'])
                        self.journal.baseline(header['status_seq'])
                        events.append((
                            'info',
                            'peer.journal_baseline',
                            {'excluded_through_seq': self.journal.cursor, 'status_epoch': header['epoch']},
                        ))
                    failures, warnings = self.peer(raw, record)
                elif source == 'peer_config':
                    config = normalize_peer_config(raw, self.args.machine_id, self.args.remote_id, self.args.slot)
                    # Compare the original hash; stored snapshot may be redacted.
                    if 'peer_config' in self.provenance:
                        require(
                            self.provenance['peer_config']['sha256'] == config['sha256'],
                            'peer_config: phase provenance changed',
                        )
                    else:
                        self.provenance['peer_config'] = config
                    age = receipt_snapshot_age(raw['live']['observed_epoch'], record['end']['epoch'])
                    require(age <= HTTP_TIMEOUT, 'peer.config: stale live revalidation epoch')
                    observed_mono = raw['live']['observed_mono']
                    served_mono = number(raw.get('served_mono'), 'peer.config.served_mono')
                    require(0 <= served_mono - observed_mono <= HTTP_TIMEOUT, 'peer.config: stale live monotonic proof')
                    self.generated['peer_config'] = now - age
                elif source == 'dut_role':
                    require(
                        isinstance(raw, dict) and raw.get('ok') is True and raw.get('role') == 'operator',
                        'dut.role: not positively operator',
                    )
                    self.freeze('role', 'operator')
                elif source == 'peer_events':
                    start = self.gate.clean_start if self.gate.clean_start is not None else self.gate.healthy_since
                    window_epoch = None if start is None else record['end']['epoch'] - (now - start)
                    events = self.journal.consume(
                        raw,
                        self.args.remote_id,
                        window_epoch,
                        negative_evidence=start is not None,
                        received_epoch=record['end']['epoch'],
                    )
                    failures.extend(reason for severity, reason, _ in events if severity == 'failure')
                elif source == 'health':
                    failures, warnings = self.health(raw, record)
                elif source == 'monitor':
                    require(
                        isinstance(raw, dict) and raw and raw.get('ok') is not False, 'dut.monitor: invalid snapshot'
                    )
            except SchemaPending as exc:
                if self.initialized:
                    failures.append(source + ': required evidence lost: ' + str(exc))
                else:
                    self.waiting[source] = [str(exc)]
                    self.reasons[source] = []
                    return [], events
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                failures.append(source + ': schema/validation: ' + str(exc))
        self.waiting[source] = [r for r in failures if r.endswith(': progress not yet demonstrated')]
        failures = [r for r in failures if r not in self.waiting[source]]
        self.reasons[source] = failures
        if failures:
            self.invalidate(now, failures)
        if source in ('dut_state', 'health'):
            changes = [r for r in failures if r.endswith((': increment', ': counter reset', ': increment/reset'))]
            if changes:
                # Distinct counter changes are events even when the preceding
                # sample had the same failure reason and the hook is cooling down.
                events.append(('failure', 'counter_change', {'source': source, 'reasons': changes}))
        new_warnings = [w for w in warnings if w.endswith(': increment') or w not in self.last_warnings.get(source, [])]
        self.last_warnings[source] = warnings
        return new_warnings, events

    def evaluate(self, now, periods, external=()):
        if self.started is None:
            self.started = now
        failures = list(external)
        waiting = []
        self.receipt_holds = []
        for source, period in periods.items():
            rec = self.latest.get(source)
            if rec is None:
                waiting.append(source + ': awaiting sample')
            else:
                budget = (
                    FRESH_SECONDS if source in ('dut_state', 'peer_status', 'peer_events') else period + HTTP_TIMEOUT
                )
                if now - rec['end']['mono'] > budget:
                    failures.append(source + ': coverage gap')
            failures.extend(self.reasons.get(source, []))
            waiting.extend(self.waiting.get(source, []))
        # Account conservatively for transport and time since the snapshot;
        # a fresh HTTP response must not extend an already old reply/heartbeat.
        dut_record = self.latest.get('dut_state')
        if dut_record and not self.reasons.get('dut_state') and self.previous_dut:
            dut = self.previous_dut
            age = (dut['uptime_ms'] - dut['_slot']['last_reply_ms']) / 1000
            receipt = dut_record['end']['mono']
            generated = max(dut_record['start']['mono'], self.generated.get('dut_state', -math.inf))
            age_at_receipt = age + max(0, receipt - generated)
            if age_at_receipt + now - receipt > FRESH_SECONDS:
                failures.append('dut.slot: reply evidence aged out')
        peer_record = self.latest.get('peer_status')
        if peer_record and not self.reasons.get('peer_status') and self.previous_peer:
            peer = self.previous_peer
            elapsed = now - peer['_mono']
            for key, age_at_receipt in peer['ages'].items():
                if age_at_receipt + elapsed > FRESH_SECONDS:
                    self.receipt_holds.append('peer.' + key + ': awaiting fresh receipt')
        if not {'firmware', 'dut_slot', 'peer_config', 'role'} <= self.provenance.keys():
            waiting.append('phase: incomplete provenance')
        if (
            not self.health_clock.certified
            or self.latest.get('health', {}).get('start', {}).get('mono', -math.inf) <= self.health_barrier
        ):
            waiting.append('health: awaiting advancing post-boot snapshot')
        peer = self.previous_peer
        reconciled = peer is not None and self.journal.initialized and self.journal.cursor >= peer['event_seq']
        if reconciled:
            self.last_sync = now
        elif self.last_sync is not None and now - self.last_sync > FRESH_SECONDS:
            failures.append('peer: event/status coverage gap')
        elif self.last_sync is None:
            waiting.append('peer: awaiting event/status reconciliation')
        if waiting and not self.initialized and now - self.started >= INITIALIZATION_SECONDS:
            # Acquisition timeout is observable, but is not a repair-hook cause.
            failures.append('collector: initialization deadline expired')
        if not waiting and not failures:
            self.initialized = True
        config = self.provenance.get('peer_config', {}).get('snapshot', {}).get('machine')
        if config and self.provenance.get('dut_slot') and self.provenance['dut_slot']['port'] != config['udp_port']:
            failures.append('phase: selected DUT slot / peer UDP port mismatch')
        if config and peer and peer['machine_port'] != config['udp_port']:
            failures.append('phase: peer status / config UDP port mismatch')
        if LIVE_PEER_SCHEMA_ACK != PEER_SCHEMA_CANDIDATE:
            waiting.append('peer: live schema not acknowledged')
        self.wait_reasons = sorted(set(waiting))
        critical = [self.latest[s]['start']['mono'] for s in ('dut_state', 'peer_events') if s in self.latest]
        if peer:
            critical.append(peer['_mono'] - peer['ages']['watermark'])
        frontier = min(critical) if len(critical) == 3 else now
        # A 1-Hz publisher plus 1-Hz HTTP has a bounded phase offset: already
        # certified historical input must not become a synthetic fault between
        # reads. HOLD credit/PASS while that receipt ages out. A stale NEW
        # snapshot is rejected by normalize_peer_status; missing HTTP coverage
        # still resets at 1.6 s. A later committed input watermark and consumed
        # journal prove the intervening interval before any further credit.
        ready = reconciled and not self.receipt_holds
        if self.gate.last_frontier is not None and frontier < self.gate.last_frontier:
            failures.append('collector: certified frontier regressed')
        if failures or waiting:
            self.invalidate(now, failures or waiting)
        else:
            self.gate.update(frontier, [], ready=ready)
            if ready and self.gate.clean_s >= self.gate.target:
                self.close_window(now)
        return sorted(set(failures)), ready

    def close_window(self, now):
        """Two fences: fresh slow snapshots, then processed/consumed event evidence.

        Public state remains RUNNING while closing. Counter changes, faults or
        any coverage gap invalidate both fences and the entire clean window.
        """
        if self.closing_cutoff is None:
            self.closing_cutoff = now
        for source in ('health', 'peer_config', 'dut_role', 'monitor'):
            if source in self.closing_verified:
                continue
            rec = self.latest.get(source)
            if not rec or rec['start']['mono'] <= self.closing_cutoff or self.reasons.get(source):
                continue
            if source in ('health', 'peer_config') and self.generated.get(source, -math.inf) <= self.closing_cutoff:
                continue
            self.closing_verified[source] = rec['end']['mono']
        if len(self.closing_verified) != 4:
            return
        self.closing_fence = max(self.closing_verified.values())
        peer = self.previous_peer
        if self.closing_witness is None and peer['_mono'] - peer['ages']['watermark'] >= self.closing_fence:
            self.closing_witness = {'receipt_mono': peer['_mono'], 'event_seq': peer['event_seq']}
        witness = self.closing_witness
        if (
            witness
            and self.latest['peer_events']['start']['mono'] >= witness['receipt_mono']
            and self.journal.cursor >= witness['event_seq']
        ):
            self.gate.state = 'PASSED'

    def refresh_sources(self):
        sources = {}
        if not self.health_clock.certified:
            sources['health'] = self.health_barrier
        if self.closing_cutoff is not None:
            for source in ('health', 'peer_config', 'dut_role', 'monitor'):
                if source not in self.closing_verified:
                    sources[source] = self.closing_cutoff
        return sources


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError('HTTP redirect refused')


def fetch(url, auth=None, jsonl=False, text=False):
    """Bounded GET, no proxy/redirect/credential forwarding, full payload or error."""
    headers = {'Cache-Control': 'no-cache, no-store', 'Accept-Encoding': 'identity'}
    if auth:
        headers['Authorization'] = 'Basic ' + base64.b64encode(f'{auth[0]}:{auth[1]}'.encode()).decode()
    request = urllib.request.Request(url, headers=headers, method='GET')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    try:
        response = opener.open(request, timeout=HTTP_TIMEOUT)
    except urllib.error.HTTPError as exc:
        response = exc  # Preserve the error response body as well as status.
    with response:
        body = response.read(MAX_BODY + 1)
        status = response.status
        response_headers = {
            k: response.headers[k] for k in ('Content-Type', 'Content-Length', 'Date', 'ETag') if k in response.headers
        }
    require(len(body) <= MAX_BODY, 'response exceeds bounded body limit (incremental journal support required)')
    raw = body.decode('utf-8', errors='replace')
    result = {'http_status': status, 'http_headers': response_headers, 'bytes': len(body), 'raw': raw}
    if not 200 <= status < 300:
        result['error'] = 'HTTP status ' + str(status)
    if raw.encode('utf-8') != body:
        result['error'] = 'invalid UTF-8 response'
        return result
    try:
        if text:
            result['data'] = raw
        elif jsonl:
            require(not raw or raw.endswith('\n'), 'peer.events: partial trailing record')
            result['data'] = [load_json(line) for line in raw.splitlines() if line.strip()]
        else:
            result['data'] = load_json(raw)
        # Python json accepts NaN/Infinity by default. They are never evidence.
        canonical(result['data'])
        del result['raw']  # Complete parsed snapshot, not selected summary fields.
    except (ValueError, TypeError, RecursionError) as exc:
        result.pop('data', None)
        result['error'] = 'invalid response: ' + str(exc)
    return result


class HTTPWorker:
    """One daemon per endpoint, one request/result slot; never unbounded retries.

    urllib's socket timeout is not a DNS/whole-body deadline. The supervisor
    invalidates evidence at HTTP_TIMEOUT even if DNS or a trickling response
    leaves this worker stuck. It never launches replacement threads for a stuck
    endpoint; late results are retained but cannot qualify. Other workers and
    the gate keep running, and shutdown never waits for stuck network threads.
    """

    def __init__(self, source, url, period, auth=None, jsonl=False, text=False):
        self.source, self.url, self.period = source, url, period
        self.auth, self.jsonl, self.text = auth, jsonl, text
        self.tasks, self.results = queue.Queue(1), queue.Queue(1)
        self.closed = threading.Event()
        self.pending = None
        self.expired = False
        self.next_due = 0.0
        threading.Thread(target=self._run, daemon=True, name='soak-' + source).start()

    def _run(self):
        while True:
            request = self.tasks.get()
            if request is None or self.closed.is_set():
                return
            try:
                payload = fetch(request['url'], self.auth, self.jsonl, self.text)
            except Exception as exc:  # Transport/parser failure remains evidence.
                payload = {'error': type(exc).__name__ + ': ' + str(exc), 'http_status': getattr(exc, 'code', None)}
            end = timestamp()
            self.results.put(dict(request, **payload, end=end, latency_s=end['mono'] - request['start']['mono']))

    def submit(self, now, suffix='', context=None):
        if self.pending is not None or self.closed.is_set():
            return False
        scheduled = self.next_due or now
        started = timestamp()
        self.pending = {
            'source': self.source,
            'url': self.url + suffix,
            'start': started,
            'scheduled_mono': scheduled,
            'collector_lag_s': max(0.0, started['mono'] - scheduled),
            'context': context,
        }
        self.expired = False
        self.tasks.put_nowait(self.pending)
        if self.period and started['mono'] >= scheduled:
            # Fixed cadence, skip missed ticks rather than creating backlog.
            # An early forced closing/recovery read is EXTRA evidence: it must
            # not consume a future regular tick and create a two-period gap.
            self.next_due = (
                scheduled + (math.floor(max(0, started['mono'] - scheduled) / self.period) + 1) * self.period
            )
        return True

    def drain(self, now):
        records = []
        try:
            record = self.results.get_nowait()
        except queue.Empty:
            record = None
        if record is not None:
            if self.expired or record['latency_s'] > HTTP_TIMEOUT:
                record['late'] = True
                record['error'] = 'HTTP whole-request deadline exceeded'
            records.append(record)
            self.pending = None
        if self.pending is not None and not self.expired and now - self.pending['start']['mono'] >= HTTP_TIMEOUT:
            records.append(
                dict(
                    self.pending,
                    end=timestamp(),
                    latency_s=now - self.pending['start']['mono'],
                    error='HTTP whole-request deadline exceeded',
                    deadline=True,
                )
            )
            self.expired = True
        return records

    def close(self):
        self.closed.set()
        try:
            self.tasks.put_nowait(None)
        except queue.Full:
            pass


class RequestScheduler:
    """Reserve one DUT lane for state; serialize/stagger every auxiliary GET.

    At most two DUT requests can be active, including expired but still blocked
    workers. Diagnostics never occupy the critical lane and run after due slow
    acceptance polls. Peer requests have their own independent workers.
    """

    def __init__(self, workers, diagnostics, now):
        self.workers, self.diagnostics = workers, diagnostics
        self.last_start = {}
        self.batch = None
        self.diagnostic_queue = deque()
        self.next_aux = now + 0.2
        offsets = {'monitor': 0.2, 'dut_role': 0.8, 'health': 1.4, 'peer_config': 0.5}
        for name, worker in workers.items():
            worker.next_due = now + offsets.get(name, 0)

    def schedule(self, now, evidence, batch=None):
        if batch is not None and batch != self.batch:
            self.batch = batch
            self.diagnostic_queue = deque(self.diagnostics)
        refresh = evidence.refresh_sources()

        def due(name):
            worker = self.workers[name]
            if (
                name in ('monitor', 'dut_role', 'health', 'peer_config')
                and now - self.last_start.get(name, -math.inf) < 2.0
            ):
                return False  # Forced closing reads must not collide with a regular tick.
            forced = name in refresh and name in self.last_start and now - self.last_start[name] >= 2.0
            return worker.pending is None and (now >= worker.next_due or forced)

        def submit(name):
            suffix = f'?after={evidence.journal.cursor}&limit=1000' if name == 'peer_events' else ''
            if self.workers[name].submit(now, suffix):
                self.last_start[name] = now

        for name in ('dut_state', 'peer_status', 'peer_events', 'peer_config'):
            if name == 'peer_events' and not evidence.journal.initialized:
                continue
            if due(name):
                submit(name)
        aux = ('monitor', 'dut_role', 'health')
        if now < self.next_aux or any(self.workers[name].pending is not None for name in aux):
            return
        if any(worker.pending is not None for worker in self.diagnostics.values()):
            return
        eligible = [name for name in aux if due(name)]
        if eligible:
            name = min(eligible, key=lambda n: (n not in refresh, self.workers[n].next_due))
            submit(name)
            self.next_aux = now + 0.2
        elif self.diagnostic_queue:
            name = self.diagnostic_queue[0]
            if self.diagnostics[name].submit(now, context=str(self.batch)):
                self.diagnostic_queue.popleft()
                self.next_aux = now + 0.2


class Artifacts:
    def __init__(self, out, redactor):
        self.out, self.redact = Path(out).resolve(), redactor
        self.out.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock = open(self.out / '.collector.lock', 'a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise ValueError('another collector owns this output directory')
        self.restart = (self.out / 'events.jsonl').exists()
        self.run_id = uuid.uuid4().hex
        self.prehistory = deque()
        self.prehistory_bytes = 0
        self.event_index = 0
        self.diagnostic_batch = None
        self.last_batch_time = -math.inf

    def disk_check(self):
        usage = shutil.disk_usage(self.out)
        if usage.free < MIN_FREE_BYTES or usage.free < usage.total * 0.01:
            raise OSError('low disk: less than 256 MiB or 1% free')

    def append(self, path, value):
        self.append_many(path, [value])

    def append_many(self, path, values):
        path = self.out / path
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        with os.fdopen(fd, 'ab') as stream:
            for value in values:
                stream.write((canonical(self.redact(value)) + '\n').encode())
            stream.flush()
            os.fsync(stream.fileno())

    def atomic(self, path, value):
        path = self.out / path
        temp = path.with_name(path.name + '.' + self.run_id + '.tmp')
        fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(canonical(self.redact(value)) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
        fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def sample(self, record):
        record = dict(record, run_id=self.run_id)
        self.append(record['source'] + '.jsonl', record)
        safe = self.redact(record)
        size = len(canonical(safe))
        self.prehistory.append((safe, size))
        self.prehistory_bytes += size
        while self.prehistory and (
            len(self.prehistory) > 512
            or self.prehistory_bytes > 16 * MAX_BODY
            or record['end']['mono'] - self.prehistory[0][0]['end']['mono'] > 60
        ):
            self.prehistory_bytes -= self.prehistory.popleft()[1]

    def event(self, severity, kind, reasons, latest, observation=None, hook='disabled'):
        self.event_index += 1
        event_id = f'{self.run_id}-{self.event_index:06d}'
        directory = self.out / 'events' / event_id
        directory.mkdir(parents=True, mode=0o700)
        now = time.monotonic()
        if severity in ('failure', 'warning') and now - self.last_batch_time >= DIAGNOSTIC_COOLDOWN:
            self.diagnostic_batch = self.out / 'evidence' / event_id
            self.diagnostic_batch.mkdir(parents=True, mode=0o700)
            self.atomic(self.diagnostic_batch / 'latest.json', latest)
            self.append_many(self.diagnostic_batch / 'prehistory.jsonl', (rec for rec, _ in self.prehistory))
            self.last_batch_time = now
        # Each event is small and points to complete raw source records plus a
        # shared evidence batch. Warning storms do not copy/fsync 60 s per event.
        refs = {
            name: {'file': name + '.jsonl', 'start': rec.get('start'), 'end': rec.get('end'), 'run_id': self.run_id}
            for name, rec in latest.items()
        }
        event = dict(
            timestamp(),
            schema_version=1,
            run_id=self.run_id,
            event_id=event_id,
            event_index=self.event_index,
            severity=severity,
            type=kind,
            reasons=reasons,
            root_cause='unknown',
            supporting_observations=observation,
            latest=refs,
            evidence_batch=str(self.diagnostic_batch) if self.diagnostic_batch else None,
            hook=hook,
        )
        self.atomic(directory / 'event.json', event)
        self.append('events.jsonl', event)
        return directory / 'event.json'

    def close(self):
        self.lock.close()


class FailureHook:
    def __init__(self, path):
        self.path = str(Path(path).resolve()) if path else None
        self.process = None
        self.last_start = -math.inf
        self.event_path = None

    def disposition(self, now):
        if not self.path:
            return 'disabled'
        if self.process:
            return 'skipped_busy'
        if now - self.last_start < HOOK_COOLDOWN:
            return 'skipped_cooldown'
        return 'scheduled'

    def start(self, event_path, now):
        self.last_start, self.event_path = now, str(event_path)
        env = {k: v for k, v in os.environ.items() if k not in ('ML_ADMIN_USER', 'ML_ADMIN_PASSWORD')}
        self.process = subprocess.Popen(
            [self.path, str(event_path)],
            shell=False,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=env,
        )

    def poll(self, now, stop=False):
        if self.process is None:
            return None
        result = self.process.poll()
        expired = now - self.last_start >= HOOK_TIMEOUT
        if result is None and not stop and not expired:
            return None
        # Bound the entire hook process group, including surviving descendants.
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            result = self.process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            result = None
        self.process = None
        return dict(timestamp(), event_path=self.event_path, returncode=result, timed_out=expired, shutdown=stop)


def status_document(evidence, store, failures, warnings, reasons, state=None):
    """Atomic schema v1; counts are per run_id, full history is events.jsonl."""
    latest = {name: rec['end'] for name, rec in evidence.latest.items()}
    closing_reasons = []
    if evidence.closing_cutoff is not None and evidence.gate.state != 'PASSED':
        missing = sorted({'health', 'peer_config', 'dut_role', 'monitor'} - evidence.closing_verified.keys())
        if missing:
            closing_reasons = ['closing: awaiting post-cutoff generation/read proof for ' + ', '.join(missing)]
        elif evidence.closing_witness is None:
            closing_reasons = ['closing: awaiting event processing watermark beyond slow-check fence']
        else:
            closing_reasons = ['closing: awaiting subsequent journal reconciliation']
    return dict(
        timestamp(),
        schema_version=1,
        run_id=store.run_id,
        state=state or evidence.gate.state,
        clean_s=evidence.gate.clean_s,
        target_s=evidence.gate.target,
        recovery_s=RECOVERY_SECONDS,
        failures=failures,
        warnings=warnings,
        phase=evidence.args.iface,
        selection={
            'machine_id': evidence.args.machine_id,
            'remote_id': evidence.args.remote_id,
            'slot': evidence.args.slot,
            'dut_vpn_ip': evidence.expected_vpn,
            'expected_iface': 2 if evidence.args.iface == 'usb' else 1,
        },
        provenance=evidence.provenance,
        latest_sample_time=latest,
        last_reasons=reasons + evidence.wait_reasons + evidence.receipt_holds + closing_reasons,
        event_seq=evidence.journal.cursor,
        stage='CLOSING'
        if evidence.closing_cutoff is not None
        else (
            'INITIALIZING'
            if not evidence.initialized
            else ('RECOVERING' if evidence.gate.state == 'WAITING' else 'SOAKING')
        ),
        schema_ack=LIVE_PEER_SCHEMA_ACK,
        event_baseline_seq=evidence.journal.initial_cursor,
        initialization_deadline_s=INITIALIZATION_SECONDS,
        closing={
            'cutoff_mono': evidence.closing_cutoff,
            'verified': evidence.closing_verified,
            'fence_mono': evidence.closing_fence,
            'witness': evidence.closing_witness,
        },
        root_cause='unknown',
    )


def run(args, stop=None):
    stop = stop or threading.Event()
    user, password = os.environ.get('ML_ADMIN_USER'), os.environ.get('ML_ADMIN_PASSWORD')
    require(bool(user) and bool(password), 'ML_ADMIN_USER and ML_ADMIN_PASSWORD are required')
    store = Artifacts(args.out, Redactor(user, password))
    hook = FailureHook(args.failure_hook)
    intent = {
        'dut': args.dut,
        'peer': args.peer,
        'machine_id': args.machine_id,
        'remote_id': args.remote_id,
        'slot': args.slot,
        'iface': args.iface,
        'clean_seconds': args.clean_seconds,
        'poll_sec': args.poll_sec,
        'dut_vpn_ip': getattr(args, 'dut_vpn_ip', None),
        'peer_schema': LIVE_PEER_SCHEMA_ACK,
    }
    evidence = Evidence(args)
    periods = {
        'dut_state': args.poll_sec,
        'peer_status': args.poll_sec,
        'peer_events': args.poll_sec,
        'monitor': 5.0,
        'dut_role': 5.0,
        'health': 30.0,
        'peer_config': 30.0,
    }
    failures = warnings = 0
    active_reasons = set()
    reasons = ['collector: starting']
    workers, diagnostics = {}, {}
    start = time.monotonic()
    previous_loop, previous_epoch = start, time.time()
    next_status = start
    final_state = 'INCOMPLETE'
    result = 2

    def emit(severity, kind, why, observation=None):
        nonlocal failures, warnings
        failures += severity == 'failure'
        warnings += severity == 'warning'
        actionable = severity == 'failure' and any('initialization deadline' not in r for r in why)
        disposition = hook.disposition(time.monotonic()) if actionable else 'not_repairable'
        path = store.event(severity, kind, why, evidence.latest, observation, disposition)
        if disposition == 'scheduled':
            try:
                hook.start(path, time.monotonic())
            except OSError as exc:
                store.append('hooks.jsonl', dict(timestamp(), event_path=str(path), error=str(exc)))
        return path

    try:
        store.disk_check()
        phase_path = store.out / 'phase.json'
        if phase_path.exists():
            phase = json.loads(phase_path.read_text())
            require(phase['intent'] == intent, 'output directory belongs to a different phase/target')
            evidence.provenance = phase['provenance']
        frozen_digest = None
        emit(
            'info',
            'collector_restart' if store.restart else 'collector_start',
            ['new monotonic window; no credit for prior process or blind time'],
        )
        store.atomic('status.json', status_document(evidence, store, failures, warnings, reasons))
        specs = {
            'dut_state': (args.dut + '/state.json', None),
            'monitor': (args.dut + '/admin/api/monitor', (user, password)),
            'dut_role': (args.dut + '/api/role', (user, password)),
            'health': (args.dut + '/api/health', None),
            'peer_status': (args.peer + '/status.json', None),
            'peer_events': (args.peer + '/events.jsonl', None),
            'peer_config': (args.peer + '/config.json', None),
        }
        workers = {
            name: HTTPWorker(name, url, periods[name], auth, jsonl=name == 'peer_events')
            for name, (url, auth) in specs.items()
        }
        diagnostics = {
            name: HTTPWorker(
                'diagnostic_' + name,
                args.dut + ('/api/last_log' if name == 'last_log' else '/admin/api/' + name),
                None,
                (user, password),
                text=name == 'last_log',
            )
            for name in ('peers', 'monitor', 'last_log')
        }
        scheduler = RequestScheduler(workers, diagnostics, time.monotonic())
        while not stop.is_set():
            now, epoch = time.monotonic(), time.time()
            external = []
            lag = now - previous_loop
            if lag > 0.5:
                external.append('collector: scheduling/storage coverage gap')
            if abs((epoch - previous_epoch) - lag) > 0.5:
                external.append('collector: wall clock discontinuity')
            previous_loop, previous_epoch = now, epoch
            if args.max_duration is not None and now - start >= args.max_duration:
                reasons = ['collector: max duration reached before clean target']
                break
            for name, worker in workers.items():
                for record in worker.drain(now):
                    store.sample(record)
                    new_warnings, observed_events = evidence.accept(record)
                    # Invalidate immediately, before another result can replace
                    # a transient bad sample or a peer failure batch with [] .
                    if evidence.reasons[name]:
                        evidence.invalidate(now, evidence.reasons[name])
                        external.extend(evidence.reasons[name])
                    for warning in new_warnings:
                        emit('warning', 'observation', [warning])
                    for severity, kind, observation in observed_events:
                        emit(severity, kind, [kind], observation)
            for worker in diagnostics.values():
                for record in worker.drain(now):
                    store.sample(record)
                    store.append(Path(record['context']) / 'diagnostics.jsonl', record)
            hook_result = hook.poll(now)
            if hook_result:
                store.append('hooks.jsonl', hook_result)
            # Include time spent persisting evidence in freshness decisions.
            now = time.monotonic()
            if now - previous_loop > 0.5:
                external.append('collector: scheduling/storage coverage gap')
            reasons, ready = evidence.evaluate(now, periods, external)
            new_reasons = set(reasons) - active_reasons
            if new_reasons:
                emit('failure', 'acceptance_fault', sorted(new_reasons))
            active_reasons = set(reasons)
            scheduler.schedule(time.monotonic(), evidence, store.diagnostic_batch)
            provenance_digest = digest(evidence.provenance)
            if provenance_digest != frozen_digest:
                store.atomic('phase.json', {'schema_version': 1, 'intent': intent, 'provenance': evidence.provenance})
                frozen_digest = provenance_digest
            if now >= next_status or evidence.gate.state == 'PASSED':
                store.disk_check()
                store.append(
                    'collector.jsonl',
                    dict(timestamp(), run_id=store.run_id, loop_lag_s=lag, event_caught_up=ready, reasons=reasons),
                )
                # Recheck after storage/diagnostics; a delayed write cannot buy time.
                reasons, ready = evidence.evaluate(time.monotonic(), periods, external)
                document = status_document(
                    evidence,
                    store,
                    failures,
                    warnings,
                    reasons,
                    state='RUNNING' if evidence.gate.state == 'PASSED' else None,
                )
                store.atomic('status.json', document)
                next_status = now + 1
            if evidence.gate.state == 'PASSED' and ready and not stop.is_set():
                # This status is committed only after all evidence is durable.
                final_state, result = 'PASSED', 0
                reasons = []
                break
            stop.wait(0.05)
        if stop.is_set():
            reasons = ['collector: interrupted; incomplete']
            final_state, result = 'INCOMPLETE', 2
        emit('info', 'collector_end', reasons or ['clean target observed'])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        reasons = ['collector: storage/configuration error: ' + str(exc)]
        evidence.gate.update(time.monotonic(), reasons)
        final_state, result = 'INCOMPLETE', 2
        print(store.redact(reasons[0]), file=sys.stderr)
        try:
            emit('failure', 'collector_error', reasons)
        except OSError:
            pass
    finally:
        for worker in list(workers.values()) + list(diagnostics.values()):
            worker.close()
        try:
            hook_result = hook.poll(time.monotonic(), stop=True)
            if hook_result:
                store.append('hooks.jsonl', hook_result)
            if stop.is_set():
                final_state, result = 'INCOMPLETE', 2
                reasons = ['collector: interrupted; incomplete']
            if result == 0:
                reasons, ready = evidence.evaluate(time.monotonic(), periods)
                if reasons or not ready or evidence.gate.state != 'PASSED':
                    final_state, result = 'INCOMPLETE', 2
            store.atomic('status.json', status_document(evidence, store, failures, warnings, reasons, final_state))
            if stop.is_set() and result == 0:
                result = 2
                store.atomic(
                    'status.json',
                    status_document(
                        evidence, store, failures, warnings, ['collector: interrupted; incomplete'], 'INCOMPLETE'
                    ),
                )
        except OSError as exc:
            # A failed disk cannot promise a final status update. Exit nonzero;
            # outer supervision must also enforce status timestamp/process liveness.
            print(store.redact('collector: cannot persist final status: ' + str(exc)), file=sys.stderr)
            result = 2
        store.close()
    return result


def base_url(value, peer=False):
    parsed = urllib.parse.urlsplit(value)
    if (
        parsed.scheme not in ('http', 'https')
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.query
        or parsed.fragment
        or parsed.path not in ('', '/')
    ):
        raise argparse.ArgumentTypeError('expected bare http(s) URL without credentials, query, or path')
    try:
        parsed.port
        address = ipaddress.ip_address(parsed.hostname)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('use an explicit numeric IP address') from exc
    if (
        peer
        and address not in ipaddress.ip_network('100.64.0.0/10')
        and address not in ipaddress.ip_network('fd7a:115c:a1e0::/48')
    ):
        raise argparse.ArgumentTypeError('--peer must be a Tailscale IP (100.64.0.0/10 or fd7a:115c:a1e0::/48)')
    return value.rstrip('/')


def positive(value):
    try:
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError()
        return value
    except ValueError as exc:
        raise argparse.ArgumentTypeError('expected a finite positive number') from exc


def vpn_ip(value):
    try:
        address = ipaddress.ip_address(value)
        if address not in ipaddress.ip_network('100.64.0.0/10') and address not in ipaddress.ip_network(
            'fd7a:115c:a1e0::/48'
        ):
            raise ValueError('not a Tailscale VPN address')
        return str(address)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('expected a numeric DUT Tailscale VPN IP') from exc


def device_id(value):
    try:
        value = int(value, 0)
        if not 0 < value <= 0xFFFFFFFF:
            raise ValueError()
        return value
    except ValueError as exc:
        raise argparse.ArgumentTypeError('expected nonzero uint32 device ID (decimal or 0x...)') from exc


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dut', type=base_url, required=True)
    parser.add_argument('--peer', type=lambda s: base_url(s, peer=True), required=True)
    parser.add_argument(
        '--dut-vpn-ip', type=vpn_ip, help='pin selected wire.last_src IP; otherwise derive from DUT state.vpn_ip'
    )
    parser.add_argument('--machine-id', type=device_id, required=True)
    parser.add_argument('--remote-id', type=device_id, required=True)
    parser.add_argument(
        '--slot', type=int, choices=(3,), required=True, help='zero-based slot; this gate requires slot 3'
    )
    parser.add_argument('--iface', choices=('usb', 'ethernet'), required=True)
    parser.add_argument('--clean-seconds', type=positive, default=14400)
    parser.add_argument('--max-duration', type=positive)
    parser.add_argument('--poll-sec', type=positive, default=1.0, help='critical cadence in seconds, at most 1')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument(
        '--failure-hook', type=Path, help='executable called asynchronously with event.json; 10s limit, 60s cooldown'
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0.1 <= args.poll_sec <= 1.0:
        parser.error('--poll-sec must be between 0.1 and 1 seconds')
    if args.failure_hook and (not args.failure_hook.is_file() or not os.access(args.failure_hook, os.X_OK)):
        parser.error('--failure-hook must be an executable file')
    stop = threading.Event()
    interrupted = []

    def handle(signum, frame):
        interrupted.append(signum)
        stop.set()

    old_handlers = {sig: signal.signal(sig, handle) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        try:
            result = run(args, stop)
        except (OSError, ValueError) as exc:
            print(
                Redactor(os.environ.get('ML_ADMIN_USER', ''), os.environ.get('ML_ADMIN_PASSWORD', ''))(str(exc)),
                file=sys.stderr,
            )
            result = 2
        return 128 + interrupted[0] if interrupted else result
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
