#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Offline acceptance/failure tests; no DUT, peer, ROS, relays, or real hooks.

Run: python3 -B -m unittest discover -s tools -p test_soak_stability.py -v
"""

import argparse
import copy
import io
import json
import os
import signal
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

import soak_stability as soak

MACHINE = 0x01020393
REMOTE = 0x01D7F344
VPN = '100.75.70.74'
WIRE_ID = '0x01D7F344'
ROS_ID = '01D7F344'
EPOCH = 1800000000.0
PERIODS = {
    'dut_state': 1,
    'peer_status': 1,
    'peer_events': 1,
    'dut_role': 5,
    'monitor': 5,
    'health': 30,
    'peer_config': 30,
}


def arguments(out='unused', **kwargs):
    values = dict(
        dut='http://10.43.0.122',
        peer='http://100.110.35.58:8895',
        machine_id=MACHINE,
        remote_id=REMOTE,
        slot=3,
        iface='usb',
        clean_seconds=3.0,
        poll_sec=1.0,
        max_duration=None,
        out=Path(out),
        failure_hook=None,
        dut_vpn_ip=VPN,
    )
    values.update(kwargs)
    return argparse.Namespace(**values)


def dut(t, iface='usb'):
    slots = [{'cfg': 0, 'id': 0} for _ in range(4)]
    slots[3] = dict(
        cfg=1,
        id=MACHINE,
        ip=0x646E233A,
        port=8893,
        state=2,
        sent=100 + int(t * 3),
        replies=100 + int(t * 3),
        send_fail=0,
        rebonds=1,
        hb_ms=400,
        last_reply_ms=100000 + int(t * 1000) - 100,
    )
    return dict(
        fw_ver='soak-test',
        fw_sha='a' * 16,
        uptime_ms=100000 + int(t * 1000),
        boot_count=3,
        reset_reason=1,
        pstop_machines=slots,
        role='operator',
        l0=85,
        l1=85,
        e_hi0=1,
        e_lo0=1,
        e_hi1=1,
        e_lo1=1,
        pstop_mismatch=0,
        active_iface=2 if iface == 'usb' else 1,
        eth_en=0 if iface == 'usb' else 1,
        eth_link=1,
        usbncm_en=1,
        wifi_en=0,
        wifi_ip=0,
        vpn_ip=1682654794,
        xcheck_fault=0,
        gpio_cfg_fault=0,
        health=0,
        ml_reconnects=1,
        pstop_sf_txdrv=0,
        pstop_sf_txdrv_recovered=0,
        eth_recoveries=0,
        relay_fault_a=0,
        relay_fault_b=0,
    )


def peer(t, seq=0, producer_lag=0.01):
    """Actual nested schema; timestamps shifted for deterministic offline tests."""
    source_t = t - producer_lag
    selected = dict(
        remote_id=WIRE_ID,
        first_seen_epoch=EPOCH - 100,
        last_rx_epoch=EPOCH + source_t - 0.1,
        last_rx_mono=10000 + source_t - 0.1,
        last_msg='OK',
        last_role='operator',
        last_counter=int(t) % 256,
        last_src=VPN + ':44719',
        rx=100 + int(source_t * 3),
        bonds=1,
        crc_fail=0,
        counter_gaps=0,
        silent=False,
        silent_since_epoch=None,
        gap_hist_ms={'<=250': 90, '<=800': 3},
        max_gap_ms=597.4,
        machine_tx=100 + int(source_t * 3),
        machine_last_msg='OK',
        machine_last_tx_epoch=EPOCH + source_t - 0.099,
        no_reply=0,
        consecutive_no_reply=0,
        stalled=False,
        stale_echo=0,
        last_reply_latency_ms=0.71,
        max_reply_latency_ms=13.4,
    )
    node = dict(
        state='ACTIVE',
        armed=True,
        hb_stop=False,
        status_reason='running',
        remotes={
            ROS_ID: {
                'stop_only': False,
                'bond_state': 'bonded',
                'in_use': True,
                'reply_age_ms': 0,
                'loop_rtt_ms': 0,
                'rebonds': 0,
            }
        },
        topic_age_s=0.031,
        remotes_age_s=0.031,
        hb_age_s=0.031,
        hb_last_stamp=EPOCH + source_t - 0.031,
        topic_counts={key: 1000 + int(source_t * 10) for key in ('state', 'remotes', 'hb')},
        topic_stalled=False,
        hb_stamp_stalled=False,
        ros_snapshot_epoch=EPOCH + source_t,
        ros_snapshot_age_s=producer_lag,
        unit={'MainPID': '42', 'NRestarts': '0'},
    )
    wire = dict(
        snapshot_epoch=EPOCH + source_t,
        snapshot_age_s=producer_lag,
        packet_rx_epoch=EPOCH + t,
        packet_rx_age_s=0,
        packet_tx_epoch=EPOCH + t,
        packet_tx_age_s=0,
        totals={'rx': 10000, 'tx': 10000},
        silence_budget_ms=1600,
        remotes={WIRE_ID: selected},
    )
    collectors = {
        name: {'alive': True, 'age_s': producer_lag, 'pid': 50 + i, 'restarts': 0, 'unit_state': 'active'}
        for i, name in enumerate(('node', 'observer', 'ros_journal', 'pcap', 'dut_poll', 'status_server'))
    }
    collectors['node'].update(pid='42', age_s=0.031)
    collectors['pcap'].update(age_s=producer_lag + 0.1)
    collectors['dut_poll'].update(age_s=4.9)
    collectors['status_server'].update(age_s=0)
    watermarks = {
        name: {
            'input_epoch': EPOCH + source_t,
            'input_mono': 10000 + source_t,
            'event_seq': seq,
            'journal_seq': seq,
            'events_emitted': seq,
        }
        for name in ('observer', 'ros_journal')
    }
    watermarks['journal_seq_committed'] = seq
    return dict(
        epoch=EPOCH + t,
        mono=10000.0 + t,
        machine={'udp_port': 8893, 'machine_id': '0x01020393', 'bound': 'UNCONN 0 0 0.0.0.0:8893'},
        node=node,
        wire=wire,
        collector=collectors,
        watermarks=watermarks,
        events={'seq': seq, 'counts': {'INFO': seq, 'WARN': 0, 'ERROR': 0}, 'last': None, 'last_error': None},
    )


def config(t=None):
    generation = '058954ef5e27c0547d86f550804494f95358286af625431991c287c442920ee2'
    provenance = {
        'commit': 'e67b5c506c52ce5f1089174ee3ebaf12e6ab8624',
        'branch': 'main',
        'binary_sha256': 'acac264e2b40633a7f4de8e60742ced643f3f07aa519a5977f772d58f92a6f84',
        'params_sha256': 'fb9c1e7e6323f25785715561c00b4af61490cdefe55d0c1a241620d0943b9b58',
        'pstop_c_protocol': {'PSTOP_VERSION': '0x02', 'PSTOP_MESSAGE_SIZE': 48},
        'units_sha256': {'node': '7ebc98ebb5503b6c30e80206cd45f0ae491b63e74b26a1d74f08b786596679ca'},
        'scripts_sha256': {'status_server.py': '221ade95b20788749d4f12da28187f21bc6d5e5b27cdbdfee421b8be53c12743'},
    }
    t = 0 if t is None else t
    return {
        'observed_epoch': EPOCH - 200,
        'observed_mono': 9800,
        'config_generation': generation,
        'machine': {
            'udp_port': 8893,
            'bind_addr': '0.0.0.0',
            'machine_id': '0x01020393',
            'machine_id_dec': MACHINE,
            'dut_slot': 3,
            'operators': [WIRE_ID],
            'default_stop_only': True,
            'timing': {'heartbeat_ms': 400, 'max_missed': 4, 'min_stop_ms': 500},
            'rates': {'publish_rate_hz': 10.0, 'diagnostics_rate_hz': 1.0},
            'silence_budget_ms': 1600,
        },
        'provenance': provenance,
        'live': dict(
            observed_epoch=EPOCH + t,
            observed_mono=10000 + t,
            config_generation=generation,
            drift=False,
            **{key: provenance[key] for key in ('commit', 'binary_sha256', 'params_sha256')},
        ),
        'served_epoch': EPOCH + t + 0.005,
        'served_mono': 10000 + t + 0.005,
    }


def health(t):
    return {
        'ok': True,
        'level': 0,
        'uptime_s': 1000 + int(t),
        'boots': 3,
        'presses': 0,
        'mismatch_events': 0,
        'nvs_flush_fails': 0,
        'dropped': 0,
    }


def peer_event(seq, kind='control_reconnect', severity='warning'):
    return dict(
        seq=seq,
        type=kind,
        severity=severity,
        src='observer',
        data={},
        mono=10000 + seq - 0.05,
        epoch=EPOCH + seq - 0.05,
    )


def record(source, data, t, **kwargs):
    return dict(
        source=source,
        data=data,
        start=soak.timestamp(1000.0 + t, EPOCH + t),
        end=soak.timestamp(1000.0 + t + 0.01, EPOCH + t + 0.01),
        latency_s=0.01,
        collector_lag_s=0.0,
        **kwargs,
    )


class Scenario:
    def __init__(self, **kwargs):
        self.evidence = soak.Evidence(arguments(**kwargs))
        self.warnings = []
        self.events = []
        self.seq = 0

    def step(self, t, state=None, status=None, events=None, overrides=None, omit=(), external=()):
        payloads = dict(
            dut_state=dut(t, self.evidence.args.iface) if state is None else state,
            peer_status=peer(t, self.seq) if status is None else status,
            peer_events=[] if events is None else events,
            dut_role={'ok': True, 'role': 'operator'},
            monitor={'tasks': []},
            health=health(t),
            peer_config=config(t),
        )
        payloads.update(overrides or {})
        immediate = list(external)
        for source, data in payloads.items():
            if source in omit:
                continue
            warnings, outcomes = self.evidence.accept(record(source, data, t))
            self.warnings.extend(warnings)
            self.events.extend(event for event in outcomes if event[1] != 'peer.journal_baseline')
            immediate.extend(self.evidence.reasons[source])
        reasons, _ = self.evidence.evaluate(1000.0 + t + 0.01, PERIODS, immediate)
        return reasons

    def prime(self):
        for t in range(8):
            self.step(t)
        assert self.evidence.gate.state == 'RUNNING'
        assert abs(self.evidence.gate.clean_s - 1) < 0.001
        return self


class AcceptanceTests(unittest.TestCase):
    def test_closing_rejects_stale_live_revalidation_despite_fresh_served_time(self):
        s = Scenario().prime()
        for t in range(8, 21):
            raw = config(t)
            raw['live']['observed_epoch'] = EPOCH
            raw['live']['observed_mono'] = 10000
            s.step(t, overrides={'peer_config': raw})
        self.assertNotIn('peer_config', s.evidence.closing_verified)
        self.assertNotEqual(s.evidence.gate.state, 'PASSED')
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_closing_slow_health_catches_a_hidden_press_after_target(self):
        s = Scenario().prime()
        s.step(8)
        s.step(9)
        cutoff = s.evidence.closing_cutoff
        self.assertIsNotNone(cutoff)
        changed = health(10)
        changed['presses'] = 1
        s.step(10, overrides={'health': changed})
        self.assertIsNone(s.evidence.closing_cutoff)
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_closing_requires_processed_watermark_then_subsequent_event_read(self):
        s = Scenario().prime()
        for t in range(8, 12):
            s.step(t)
        self.assertIsNotNone(s.evidence.closing_fence)
        lagging = peer(12)
        lagging['watermarks']['ros_journal'].update(input_mono=10011.0, input_epoch=EPOCH + 11)
        self.assertFalse(s.step(12, status=lagging))
        self.assertIsNone(s.evidence.closing_witness)
        s.step(13)
        self.assertIsNotNone(s.evidence.closing_witness)
        self.assertNotEqual(s.evidence.gate.state, 'PASSED')
        s.seq = 1
        s.step(14, events=[peer_event(1, 'stop', 'failure')])
        self.assertEqual(s.evidence.gate.clean_s, 0)
        self.assertIsNone(s.evidence.closing_witness)

    def test_both_interfaces_need_full_target_after_positive_recovery(self):
        for iface in ('usb', 'ethernet'):
            with self.subTest(iface=iface):
                scenario = Scenario(iface=iface).prime()
                scenario.step(8)
                self.assertEqual(scenario.evidence.gate.state, 'RUNNING')
                scenario.step(9)
                self.assertIsNotNone(scenario.evidence.closing_cutoff)
                self.assertNotEqual(scenario.evidence.gate.state, 'PASSED')
                for t in range(10, 14):
                    scenario.step(t)
                self.assertEqual(scenario.evidence.gate.state, 'PASSED')
                self.assertGreaterEqual(scenario.evidence.gate.clean_s, 3)

    def test_four_hour_target_is_not_credited_for_blind_time(self):
        s = Scenario(clean_seconds=14400).prime()
        reasons, _ = s.evidence.evaluate(1000 + 14407, PERIODS)
        self.assertTrue(any('coverage gap' in r for r in reasons))
        self.assertEqual(s.evidence.gate.clean_s, 0)
        s.step(14408)
        self.assertEqual(s.evidence.gate.state, 'WAITING')

    def test_four_hours_of_continuous_evidence_passes_only_at_target(self):
        # Advance a simulated clock, not the bench or real time.
        s = Scenario(clean_seconds=14400)
        for t in range(14406):
            s.step(t)
        self.assertEqual(s.evidence.gate.state, 'RUNNING')
        self.assertAlmostEqual(s.evidence.gate.clean_s, 14399)
        s.step(14406)
        self.assertNotEqual(s.evidence.gate.state, 'PASSED')
        for t in range(14407, 14416):
            s.step(t)
        self.assertEqual(s.evidence.gate.state, 'PASSED')

    def test_fault_requires_rearmed_fresh_five_seconds_then_new_target(self):
        s = Scenario().prime()
        stopped = peer(8)
        stopped['node']['armed'] = False
        self.assertTrue(s.step(8, status=stopped))
        self.assertEqual(s.evidence.gate.clean_s, 0)
        for t in range(9, 14):
            s.step(t)
            self.assertEqual(s.evidence.gate.state, 'WAITING')
        s.step(14)
        self.assertEqual(s.evidence.gate.clean_s, 0)
        self.assertEqual(s.evidence.gate.state, 'RUNNING')
        s.step(15)
        s.step(16)
        self.assertNotEqual(s.evidence.gate.state, 'PASSED')
        s.step(17)
        self.assertNotEqual(s.evidence.gate.state, 'PASSED')
        for t in range(18, 22):
            s.step(t)
        self.assertEqual(s.evidence.gate.state, 'PASSED')

    def test_warning_reconnect_and_recovered_tx_are_not_disconnects(self):
        s = Scenario().prime()
        state = dut(8)
        state.update(ml_reconnects=2, pstop_sf_txdrv=3, pstop_sf_txdrv_recovered=3)
        s.seq = 1
        reasons = s.step(8, state=state, events=[peer_event(1)])
        self.assertFalse(reasons)
        self.assertAlmostEqual(s.evidence.gate.clean_s, 2)
        self.assertEqual(len(s.warnings), 3)
        self.assertEqual(s.events[0][0], 'warning')

    def test_warning_event_in_flight_pauses_credit_without_false_disconnect(self):
        s = Scenario().prime()
        s.seq = 1
        self.assertFalse(s.step(8))
        self.assertAlmostEqual(s.evidence.gate.clean_s, 1)
        s.step(9, events=[peer_event(1)])
        for t in range(10, 14):
            s.step(t)
        self.assertEqual(s.evidence.gate.state, 'PASSED')

    def test_true_peer_failure_between_healthy_statuses_resets(self):
        for kind, severity in (('stop', 'failure'), ('disconnect', 'warning'), ('opaque_failure', 'error')):
            with self.subTest(kind=kind):
                s = Scenario().prime()
                s.seq = 1
                self.assertTrue(s.step(8, events=[peer_event(1, kind, severity)]))
                self.assertEqual(s.evidence.gate.clean_s, 0)
                self.assertEqual(s.events[0][0], 'failure')

    def test_no_endpoint_can_be_replaced_by_other_healthy_samples(self):
        for missing in PERIODS:
            with self.subTest(missing=missing):
                s = Scenario()
                for t in range(12):
                    s.step(t, omit=(missing,))
                self.assertEqual(s.evidence.gate.state, 'WAITING')
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_collector_lag_resets_even_when_all_latest_samples_look_good(self):
        s = Scenario().prime()
        s.step(8, external=['collector: scheduling/storage coverage gap'])
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_slow_transport_does_not_refresh_old_packet_or_reply(self):
        s = Scenario().prime()
        raw = dut(8)
        raw['pstop_machines'][3]['last_reply_ms'] = raw['uptime_ms'] - 1500
        self.assertFalse(s.step(8, state=raw))
        reasons, _ = s.evidence.evaluate(1008.2, PERIODS)
        self.assertTrue(any('aged out' in r for r in reasons))
        self.assertEqual(s.evidence.gate.clean_s, 0)


class DUTFailureTests(unittest.TestCase):
    def test_deferred_usb_drops_reset_clean_time_even_when_peer_is_healthy(self):
        fields = (
            'usb_tx_cancelled',
            'usb_tx_errors',
            'usb_tx_exhausted',
            'usb_tx_defer_full',
            'usb_tx_can_xmit_fail',
            'usb_tx_integrity_errors',
            'usb_tx_defer_cap',
        )
        for iface in ('usb', 'ethernet'):
            for field in fields:
                with self.subTest(iface=iface, field=field):
                    s = Scenario(iface=iface)
                    for t in range(8):
                        raw = dut(t, iface)
                        raw[field] = 0
                        s.step(t, state=raw)
                    self.assertGreater(s.evidence.gate.clean_s, 0)
                    raw = dut(8, iface)
                    raw[field] = 1
                    reasons = s.step(8, state=raw)
                    self.assertIn('dut.' + field + ': increment', reasons)
                    self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_usb_availability_losses_fail_usb_window_and_only_warn_in_ethernet(self):
        for iface in ('usb', 'ethernet'):
            for field in ('usb_tx_rejected_unavailable', 'usb_tx_unmounted_drop'):
                with self.subTest(iface=iface, field=field):
                    s = Scenario(iface=iface)
                    for t in range(8):
                        raw = dut(t, iface)
                        raw[field] = 0
                        s.step(t, state=raw)
                    raw = dut(8, iface)
                    raw[field] = 1
                    reasons = s.step(8, state=raw)
                    if iface == 'usb':
                        self.assertIn('dut.' + field + ': increment', reasons)
                        self.assertEqual(s.evidence.gate.clean_s, 0)
                    else:
                        self.assertFalse(reasons)
                        self.assertGreater(s.evidence.gate.clean_s, 0)
                        self.assertEqual(len(s.warnings), 1)

    def test_missing_optional_counter_cannot_erase_its_previous_value(self):
        for source, key in (('dut_state', 'eth_recoveries'), ('health', 'nvs_flush_fails')):
            with self.subTest(source=source):
                s = Scenario().prime()
                raw = dut(8) if source == 'dut_state' else health(8)
                del raw[key]
                reasons = s.step(8, overrides={source: raw})
                self.assertTrue(any('coverage' in reason for reason in reasons))
                raw = dut(9) if source == 'dut_state' else health(9)
                raw[key] = 1
                reasons = s.step(9, overrides={source: raw})
                self.assertTrue(any(key + ': increment' in reason for reason in reasons))
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_lifetime_presses_are_faults_even_if_armed_has_recovered(self):
        s = Scenario().prime()
        raw = health(8)
        raw['presses'] = 1
        reasons = s.step(8, overrides={'health': raw})
        self.assertIn('health.presses: increment', reasons)
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_whole_boot_anchor_rejects_slowly_replayed_uptime(self):
        s = Scenario()
        found = []
        for t in range(15):
            raw = dut(t)
            raw['uptime_ms'] = 100000 + t * 100  # Positive on every 1-Hz read.
            raw['pstop_machines'][3]['last_reply_ms'] = raw['uptime_ms'] - 20
            found.extend(s.step(t, state=raw))
        self.assertTrue(any('whole-epoch uptime drift' in reason for reason in found))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_reboot_cannot_baseline_a_cached_preboot_health_snapshot(self):
        s = Scenario().prime()
        for t in range(8, 18):
            raw = dut(t)
            raw['boot_count'] = 4
            raw['uptime_ms'] = (t - 7) * 1000
            raw['pstop_machines'][3]['last_reply_ms'] = raw['uptime_ms'] - 100
            reasons = s.step(t, state=raw, overrides={'health': health(7)})
            self.assertTrue(any('pre-reboot' in r or 'boot barrier' in r for r in reasons))
            self.assertFalse(s.evidence.health_clock.certified)
            self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_slot_three_and_machine_id_are_both_mandatory_no_fallback(self):
        for variant in ('wrong_id', 'moved', 'unconfigured', 'duplicate', 'lost'):
            with self.subTest(variant=variant):
                s = Scenario().prime()
                raw = dut(8)
                slots = raw['pstop_machines']
                if variant == 'wrong_id':
                    slots[3]['id'] += 1
                elif variant == 'moved':
                    slots[0], slots[3] = slots[3], slots[0]
                elif variant == 'unconfigured':
                    slots[3]['cfg'] = 0
                elif variant == 'duplicate':
                    slots[0] = dict(slots[3])
                else:
                    slots.pop()
                self.assertTrue(s.step(8, state=raw))
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_interface_safety_bits_reboots_and_firmware_are_gating(self):
        bad = {
            'active_iface': 1,
            'eth_en': 1,
            'usbncm_en': 0,
            'wifi_en': 1,
            'wifi_ip': '192.168.1.2',
            'l0': 0,
            'l1': 0,
            'e_hi0': 0,
            'e_lo0': 0,
            'e_hi1': 0,
            'e_lo1': 0,
            'xcheck_fault': 1,
            'gpio_cfg_fault': 1,
            'boot_count': 4,
            'reset_reason': 2,
            'uptime_ms': 1,
            'fw_ver': 'changed',
            'fw_sha': 'f' * 16,
            'pstop_mismatch': 1,
            'eth_recoveries': 1,
            'health': 2,
            'role': 'stop_only',
            'relay_stop': 1,
        }
        for key, value in bad.items():
            with self.subTest(field=key):
                s = Scenario().prime()
                raw = dut(8)
                raw[key] = value
                self.assertTrue(s.step(8, state=raw))
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_bond_loss_rebond_send_failure_counter_reset_and_stale_reply(self):
        bad = {
            'state': 1,
            'rebonds': 2,
            'send_fail': 1,
            'sent': 0,
            'replies': 0,
            'last_reply_ms': 100000,
            'hb_ms': 1000,
            'ip': 7,
            'port': 9000,
        }
        for key, value in bad.items():
            with self.subTest(field=key):
                s = Scenario().prime()
                raw = dut(8)
                raw['pstop_machines'][3][key] = value
                self.assertTrue(s.step(8, state=raw))
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_no_sends_or_no_replies_each_independently_fail(self):
        for field in ('sent', 'replies'):
            with self.subTest(field=field):
                s = Scenario().prime()
                for t in (8, 9):
                    raw = dut(t)
                    raw['pstop_machines'][3][field] = 121  # frozen at t=7
                    reasons = s.step(t, state=raw)
                self.assertTrue(any('no progress' in r for r in reasons))
                self.assertNotEqual(s.evidence.gate.state, 'PASSED')

    def test_firmware_change_is_latched_for_the_entire_phase(self):
        s = Scenario().prime()
        for t in range(8, 21):
            raw = dut(t)
            raw['fw_sha'] = 'f' * 16
            s.step(t, state=raw)
        self.assertEqual(s.evidence.gate.state, 'WAITING')
        self.assertEqual(s.evidence.provenance['firmware']['fw_sha'], 'a' * 16)

    def test_replayed_state_is_never_accepted(self):
        s = Scenario().prime()
        self.assertTrue(s.step(8, state=dut(7)))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_repeated_failure_increments_remain_distinct_events(self):
        s = Scenario().prime()
        for t in (8, 9, 10):
            raw = dut(t)
            raw['pstop_machines'][3]['send_fail'] = t - 7
            s.step(t, state=raw)
        changes = [event for event in s.events if event[1] == 'counter_change']
        self.assertEqual(len(changes), 3)
        self.assertTrue(all(event[0] == 'failure' for event in changes))
        self.assertEqual(s.evidence.gate.clean_s, 0)


class PeerContractTests(unittest.TestCase):
    def test_cached_snapshot_age_is_carried_forward_from_receipt(self):
        s = Scenario().prime()
        cached = peer(7.1)
        self.assertFalse(s.step(8, status=cached))
        credited = s.evidence.gate.clean_s
        reasons, ready = s.evidence.evaluate(1008.55, PERIODS)
        self.assertFalse(ready)
        self.assertTrue(s.evidence.receipt_holds)
        self.assertEqual(s.evidence.gate.clean_s, credited)
        reasons, _ = s.evidence.evaluate(1009.7, PERIODS)
        self.assertTrue(any('coverage gap' in reason for reason in reasons))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_unknown_info_and_subbudget_gap_warnings_are_not_faults(self):
        for event in (
            {'kind': 'new_observer_note', 'sev': 'INFO'},
            {'kind': 'stop_budget_configuration', 'sev': 'INFO'},
            {'kind': 'packet_gap', 'severity': 'warning', 'gap_ms': 400},
            {'kind': 'packet_gap', 'sev': 'WARN', 'gap_s': 0.8},
        ):
            with self.subTest(event=event):
                event.update(seq=1, epoch=EPOCH, mono=10000)
                result = soak.PeerEvents().consume([event])
                self.assertIn(result[0][0], ('info', 'warning'))

    def test_errors_stops_and_over_budget_gaps_override_armed_or_warning(self):
        cases = [
            {'kind': 'packet_gap', 'severity': 'warning', 'gap_ms': 1601},
            {'kind': 'packet_gap', 'severity': 'info', 'gap_s': 2.0},
            {'kind': 'packet_rx', 'severity': 'info', 'msg': 'STOP'},
            {'kind': 'disconnect', 'sev': 'WARN'},
            {'kind': 'new_condition', 'sev': 'ERROR'},
        ]
        for event in cases:
            event.update(seq=1, epoch=EPOCH, mono=10000)
            self.assertEqual(soak.PeerEvents().consume([event])[0][0], 'failure')
        for msg in ('STOP', 'BOND', 'UNBOND'):
            s = Scenario().prime()
            raw = peer(8)
            raw['wire']['remotes'][WIRE_ID]['last_msg'] = msg
            self.assertTrue(s.step(8, status=raw))
            self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_negative_counts_and_overbudget_gap_counts_catch_between_poll_faults(self):
        for mutation in (
            lambda p: p['wire']['remotes'][WIRE_ID].update(crc_fail=1),
            lambda p: p['wire']['remotes'][WIRE_ID]['gap_hist_ms'].update({'>1600': 1}),
            lambda p: p['wire']['remotes'][WIRE_ID].update(max_gap_ms=1601),
        ):
            s = Scenario().prime()
            raw = peer(8)
            mutation(raw)
            self.assertTrue(s.step(8, status=raw))
            self.assertEqual(s.evidence.gate.clean_s, 0)
        s = Scenario().prime()
        raw = peer(8)
        raw['wire']['remotes'][WIRE_ID].update(counter_gaps=1, no_reply=1, consecutive_no_reply=1)
        self.assertFalse(s.step(8, status=raw))
        self.assertTrue(s.warnings)

    def test_historical_gap_high_water_does_not_upgrade_new_subbudget_warning(self):
        s = Scenario()
        for t in range(9):
            raw = peer(t)
            raw['wire']['remotes'][WIRE_ID].update(max_gap_ms=2000, counter_gaps=0 if t < 8 else 1)
            raw['wire']['remotes'][WIRE_ID]['gap_hist_ms'].update({'>1600': 1})
            reasons = s.step(t, status=raw)
        self.assertFalse(reasons)
        self.assertGreater(s.evidence.gate.clean_s, 0)
        self.assertTrue(s.warnings)

    def test_event_sequence_equality_without_processing_watermark_is_insufficient(self):
        s = Scenario().prime()
        raw = peer(8)
        raw['watermarks']['observer'].update(input_mono=10006, input_epoch=EPOCH + 6)
        self.assertTrue(s.step(8, status=raw))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_nested_contract_cannot_fall_back_to_healthy_legacy_fields(self):
        raw = peer(8)
        raw['node'].update(armed=False, state='DEACTIVATED')
        raw.update(armed=True, node_state='armed', event_seq=0)
        with self.assertRaises(ValueError):
            soak.normalize_peer_status(raw, REMOTE, EPOCH + 8)

    def test_missing_ages_never_use_ros_zero_placeholders(self):
        for missing in ('node', 'wire', 'collector', 'watermarks'):
            with self.subTest(missing=missing):
                raw = peer(1)
                del raw[missing]
                with self.assertRaises(ValueError):
                    soak.normalize_peer_status(raw, REMOTE, EPOCH + 1)
        raw = peer(1)
        raw['node'].pop('remotes_age_s')
        with self.assertRaises(ValueError):
            soak.normalize_peer_status(raw, REMOTE, EPOCH + 1)

    def test_arm_freshness_remote_identity_and_collector_liveness(self):
        mutations = [
            lambda p: p['node'].update(armed=False),
            lambda p: p['node'].update(armed='true'),
            lambda p: p['node'].update(state='UNSTABLE'),
            lambda p: p.update(epoch=EPOCH - 20),
            lambda p: p.update(epoch=EPOCH + 20),
            lambda p: p['node']['unit'].update(MainPID='0'),
            lambda p: p['node']['remotes'].pop(ROS_ID),
            lambda p: p['wire']['remotes'][WIRE_ID].update(remote_id='0x01020381'),
            lambda p: p['wire']['remotes'][WIRE_ID].update(machine_last_tx_epoch=EPOCH - 5),
            lambda p: p['node'].update(ros_snapshot_epoch=EPOCH - 20),
            lambda p: p['collector']['ros_journal'].update(alive=False),
            lambda p: p['collector']['pcap'].pop('age_s'),
            lambda p: p['collector'].pop('node'),
            lambda p: p['node'].update(hb_stamp_stalled=True),
            lambda p: p.update(mono=float('nan')),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                raw = peer(8)
                mutate(raw)
                s = Scenario().prime()
                self.assertTrue(s.step(8, status=raw))
                self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_regenerated_status_with_stalled_underlying_progress_fails(self):
        s = Scenario().prime()
        frozen = peer(7)
        for t in (8, 9):
            raw = peer(t)
            raw['wire']['remotes'][WIRE_ID]['last_rx_epoch'] = frozen['wire']['remotes'][WIRE_ID]['last_rx_epoch']
            raw['wire']['remotes'][WIRE_ID]['last_rx_mono'] = frozen['wire']['remotes'][WIRE_ID]['last_rx_mono']
            reasons = s.step(t, status=raw)
        self.assertTrue(reasons)
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_replayed_status_counters_and_process_reset(self):
        for mutate in (
            lambda p: p['collector']['observer'].update(pid=900),
            lambda p: p.update(mono=1),
            lambda p: p['wire']['remotes'][WIRE_ID].update(rx=1),
        ):
            raw = peer(8)
            mutate(raw)
            s = Scenario().prime()
            self.assertTrue(s.step(8, status=raw))
            self.assertEqual(s.evidence.gate.clean_s, 0)
        s = Scenario().prime()
        self.assertTrue(s.step(8, status=peer(7)))

    def test_timing_and_configuration_hashes_are_required_and_frozen(self):
        for mutation in (
            lambda c: c['machine']['timing'].update(max_missed=5),
            lambda c: c['machine']['timing'].update(heartbeat_ms=500),
            lambda c: c['machine'].update(machine_id_dec=7),
            lambda c: c['provenance'].pop('units_sha256'),
            lambda c: c['provenance'].update(commit='unknown'),
        ):
            raw = config()
            mutation(raw)
            with self.assertRaises(ValueError):
                soak.normalize_peer_config(raw, MACHINE)
        s = Scenario().prime()
        raw = config()
        raw['machine']['other_parameter'] = True
        for t in range(8, 20):
            self.assertTrue(s.step(t, overrides={'peer_config': raw}))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_event_gaps_unknown_schema_and_mutation_are_failures(self):
        journal = soak.PeerEvents()
        events = journal.consume([peer_event(2)])
        self.assertIn('peer.event_gap', [e[1] for e in events])
        changed = peer_event(2, 'stop', 'failure')
        self.assertEqual(journal.consume([changed])[0][1], 'peer.event_mutated')
        for event in (dict(seq=3), peer_event(3, severity='debug')):
            journal = soak.PeerEvents()
            journal.cursor = 2
            self.assertEqual(journal.consume([event])[0][0], 'failure')
        outcomes = soak.PeerEvents().consume([peer_event(1), peer_event(1)])
        self.assertEqual(outcomes[-1][1], 'peer.event_schema')

    def test_malformed_tail_cannot_hide_an_earlier_stop_event(self):
        journal = soak.PeerEvents()
        outcomes = journal.consume([peer_event(1, 'stop', 'failure'), {'unrecognized': True}])
        self.assertEqual([item[1] for item in outcomes], ['peer.stop', 'peer.event_schema'])
        self.assertEqual(journal.cursor, 1)

    def test_old_bounded_journal_replays_are_idempotent(self):
        journal = soak.PeerEvents()
        self.assertEqual(len(journal.consume([peer_event(1)])), 1)
        self.assertEqual(journal.consume([peer_event(1)]), [])
        self.assertEqual(len(journal.consume([peer_event(1), peer_event(2)])), 1)
        self.assertEqual(journal.cursor, 2)


class LiveAdapterTests(unittest.TestCase):
    def test_live_schema_is_acknowledged_and_config_matches_inspected_hashes(self):
        self.assertEqual(soak.LIVE_PEER_SCHEMA_ACK, 'nested-soak-2026-09-10')
        normalized = soak.normalize_peer_config(config(10), MACHINE, REMOTE, 3)
        self.assertEqual(normalized['stop_budget_ms'], 1600)
        self.assertEqual(normalized['snapshot']['provenance']['commit'], 'e67b5c506c52ce5f1089174ee3ebaf12e6ab8624')

    def test_inspected_unarmed_selftest_state_cannot_qualify(self):
        # Values/layout observed in the allowed 04:43:02Z GET, with shifted clocks.
        raw = peer(0, 222, producer_lag=0.658)
        raw['node'].update(
            state='DEACTIVATED',
            armed=False,
            hb_stop=True,
            remotes={},
            status_reason='need_stop (awaiting arming gesture)',
        )
        selected = raw['wire']['remotes'][WIRE_ID]
        selected.update(
            last_src='127.0.0.1:44719',
            last_role='stop_only',
            silent=True,
            last_rx_epoch=EPOCH - 174.8,
            last_rx_mono=10000 - 174.8,
            machine_last_msg='STOP',
            no_reply=1,
        )
        raw['collector']['pcap']['age_s'] = 162.6
        evidence = soak.Evidence(arguments())
        _, events = evidence.accept(record('peer_status', raw, 0))
        self.assertEqual(evidence.journal.initial_cursor, 222)
        self.assertEqual(events[0][1], 'peer.journal_baseline')
        self.assertTrue(evidence.reasons['peer_status'])
        self.assertEqual(evidence.gate.clean_s, 0)
        self.assertEqual(evidence.gate.state, 'WAITING')

    def test_source_pin_rejects_loopback_and_wrong_vpn_even_for_correct_id(self):
        for source in ('127.0.0.1:44719', '100.75.70.75:44719', '192.168.1.2:44719'):
            raw = peer(1)
            raw['wire']['remotes'][WIRE_ID]['last_src'] = source
            with self.subTest(source=source), self.assertRaises(ValueError):
                soak.normalize_peer_status(raw, REMOTE, EPOCH + 1.01, VPN, MACHINE)
        valid = soak.normalize_peer_status(peer(1), REMOTE, EPOCH + 1.01, VPN, MACHINE)
        self.assertEqual(valid['physical_source'], VPN)

    def test_optional_pin_is_derived_from_dut_not_peer_claim(self):
        evidence = soak.Evidence(arguments(dut_vpn_ip=None))
        evidence.accept(record('dut_state', dut(0), 0))
        self.assertEqual(evidence.expected_vpn, VPN)
        forged = peer(1)
        forged['wire']['remotes'][WIRE_ID]['last_src'] = '100.75.70.75:1234'
        evidence.accept(record('peer_status', forged, 1))
        self.assertTrue(any('VPN source mismatch' in r for r in evidence.reasons['peer_status']))

    def test_other_remote_and_aggregate_packets_do_not_substitute_for_dut(self):
        raw = peer(8)
        other = copy.deepcopy(raw['wire']['remotes'][WIRE_ID])
        other.update(
            remote_id='0x01020381',
            last_src='127.0.0.1:55656',
            silent=True,
            last_rx_epoch=EPOCH - 160,
            machine_last_msg='STOP',
        )
        raw['wire']['remotes']['0x01020381'] = other
        self.assertFalse(soak.normalize_peer_status(raw, REMOTE, EPOCH + 8.01, VPN, MACHINE)['failures'])
        # Aggregate times are fresh, but selected DUT is stale: it must fail.
        raw['wire']['remotes'][WIRE_ID].update(last_rx_epoch=EPOCH + 4, last_rx_mono=10004)
        with self.assertRaises(ValueError):
            soak.normalize_peer_status(raw, REMOTE, EPOCH + 8.01, VPN, MACHINE)

    def test_digits_only_peer_ids_are_hex_and_no_placeholder_ages_used(self):
        self.assertEqual(soak.remote_identifier('01020381'), 0x01020381)
        raw = peer(1)
        item = raw['node']['remotes'].pop(ROS_ID)
        raw['node']['remotes']['01020381'] = item
        with self.assertRaises(ValueError):
            soak.normalize_peer_status(raw, REMOTE, EPOCH + 1.01, VPN, MACHINE)

    def test_isolated_no_reply_gaps_and_rebond_are_warnings_but_three_losses_fail(self):
        s = Scenario().prime()
        raw = peer(8)
        raw['wire']['remotes'][WIRE_ID].update(no_reply=1, consecutive_no_reply=1, counter_gaps=1, bonds=2)
        self.assertFalse(s.step(8, status=raw))
        self.assertEqual(len(s.warnings), 3)
        raw = peer(9)
        raw['wire']['remotes'][WIRE_ID].update(no_reply=3, consecutive_no_reply=3, counter_gaps=1, bonds=2)
        self.assertTrue(s.step(9, status=raw))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_pcap_mtime_must_advance_with_traffic_but_dut_poll_is_not_critical(self):
        raw = peer(1)
        raw['collector']['dut_poll'].update(alive=False, age_s=100, pid=0)
        self.assertFalse(soak.normalize_peer_status(raw, REMOTE, EPOCH + 1.01, VPN, MACHINE)['failures'])
        for age in (None, 2.0):
            raw['collector']['pcap']['age_s'] = age
            with self.assertRaises(ValueError):
                soak.normalize_peer_status(raw, REMOTE, EPOCH + 1.01, VPN, MACHINE)

    def test_initial_pcap_no_data_waits_but_loss_after_initialization_is_a_fault(self):
        evidence = soak.Evidence(arguments())
        evidence.accept(record('dut_state', dut(0), 0))
        raw = peer(0)
        raw['collector']['pcap']['age_s'] = None
        evidence.accept(record('peer_status', raw, 0))
        self.assertFalse(evidence.reasons['peer_status'])
        self.assertTrue(evidence.waiting['peer_status'])
        s = Scenario().prime()
        raw = peer(8)
        raw['collector']['pcap']['age_s'] = None
        self.assertTrue(s.step(8, status=raw))
        self.assertEqual(s.evidence.gate.clean_s, 0)

    def test_status_sequence_baseline_excludes_all_selftest_events_without_hooks(self):
        evidence = soak.Evidence(arguments())
        evidence.accept(record('dut_state', dut(0), 0))
        evidence.accept(record('peer_status', peer(0, 220), 0))
        old = [
            dict(peer_event(seq, 'REMOTE_DROPPED', 'failure'), data={'remote_id': '01020381'}) for seq in range(40, 220)
        ]
        warnings, events = evidence.accept(record('peer_events', old, 0))
        self.assertEqual(events, [])
        self.assertEqual(warnings, [])
        self.assertFalse(evidence.reasons['peer_events'])
        self.assertEqual(evidence.journal.cursor, 220)
        self.assertEqual(evidence.gate.clean_s, 0)
        fresh = peer_event(221, 'DUT_UNREACHABLE', 'failure')
        fresh.update(epoch=EPOCH + 1, mono=10001)
        _, events = evidence.accept(record('peer_events', [fresh], 1))
        self.assertEqual(events[0][0], 'failure')

    def test_real_event_shapes_severity_scope_and_observer_exit(self):
        # Actual event 224: underscore-delimited OBSERVER_EXIT, WARN wrapper.
        exited = dict(
            peer_event(224, 'EVIDENCE_COLLECTED'),
            src='collector',
            sev='WARN',
            data={'reason': 'OBSERVER_EXIT', 'detail': 'result=success code=killed'},
        )
        self.assertEqual(soak.normalize_peer_event(exited, REMOTE)[0], 'failure')
        for kind in ('REMOTE_REBOND', 'ROLE_CHANGE', 'MACHINE_NO_REPLY', 'RX_GAP', 'NEW_DIAGNOSTIC'):
            event = dict(peer_event(1, kind), sev='WARN', data={'remote_id': WIRE_ID, 'gap_ms': 1600})
            self.assertEqual(soak.normalize_peer_event(event, REMOTE)[0], 'warning')
        event = dict(peer_event(2, 'RX_GAP'), data={'remote_id': WIRE_ID, 'gap_ms': 1601})
        self.assertEqual(soak.normalize_peer_event(event, REMOTE)[0], 'failure')
        event = dict(peer_event(3, 'REMOTE_DROPPED', 'failure'), data={'remote_id': '01020381'})
        self.assertEqual(soak.normalize_peer_event(event, REMOTE)[0], 'info')

    def test_info_gesture_is_negative_inside_window_but_not_initial_arming(self):
        event = dict(peer_event(1, 'WIRE_MSG', 'info'), data={'remote_id': WIRE_ID, 'msg': 'STOP'})
        self.assertEqual(soak.normalize_peer_event(event, REMOTE, negative_evidence=False)[0], 'info')
        self.assertEqual(soak.normalize_peer_event(event, REMOTE, credit_epoch=EPOCH)[0], 'failure')
        self.assertEqual(soak.normalize_peer_event(event, REMOTE, credit_epoch=EPOCH + 5)[0], 'info')

    def test_review_annotation_does_not_reset_armed_window(self):
        s = Scenario().prime()
        review = dict(
            peer_event(1, 'INDEPENDENT_RESULT', 'failure'),
            src='machine_agent',
            epoch=EPOCH + 8,
            mono=10008,
            data={'bench_event': 'previous outage', 'msg': 'STOP'},
        )
        self.assertFalse(s.step(8, events=[review]))
        self.assertGreater(s.evidence.gate.clean_s, 1)
        self.assertEqual(s.evidence.journal.cursor, 1)
        self.assertEqual(s.events[-1][0], 'warning')

    def test_actual_review_sequence_and_its_evidence_bundle_are_annotations(self):
        # Sanitized shapes of real peer seq806/807/808 at08:33:01Z, reviewing
        # already-recorded08:29/08:30 outages. They are not new wire failures.
        records = [
            dict(peer_event(seq, 'INDEPENDENT_RESULT', 'failure'), src='machine_agent', data={'bench_event': ref})
            for seq, ref in ((806, '000003'), (807, '000057'))
        ]
        records.append(
            dict(
                peer_event(808, 'EVIDENCE_COLLECTED', 'failure'),
                src='collector',
                data={'reason': 'FAIL_INDEPENDENT_RESULT', 'detail': 'seq=806 src=machine_agent'},
            )
        )
        journal = soak.PeerEvents(baseline_required=True)
        journal.baseline(805)
        outcomes = journal.consume(records, REMOTE)
        self.assertEqual([row[0] for row in outcomes], ['warning'] * 3)
        self.assertEqual(journal.cursor, 808)
        actual = dict(peer_event(809, 'REMOTE_SILENT', 'failure'), src='pkt_observer')
        self.assertEqual(journal.consume([actual], REMOTE)[0][0], 'failure')

    def test_annotation_filter_cannot_hide_observer_or_recorder_faults(self):
        for source, kind, data in (
            ('pkt_observer', 'INDEPENDENT_RESULT', {}),
            ('machine_agent', 'REMOTE_DROPPED', {}),
            ('collector', 'EVIDENCE_COLLECTED', {'reason': 'OBSERVER_EXIT', 'detail': 'src=machine_agent'}),
            ('collector', 'EVIDENCE_COLLECTED', {'reason': 'FAIL_MACHINE_STALL', 'detail': 'src=pkt_observer'}),
            ('collector', 'EVIDENCE_COLLECTED', {'reason': 'FAIL_INDEPENDENT_RESULT', 'detail': 'src=pkt_observer'}),
        ):
            with self.subTest(source=source, kind=kind, data=data):
                event = dict(peer_event(1, kind, 'failure'), src=source, data=data)
                self.assertEqual(soak.normalize_peer_event(event, REMOTE)[0], 'failure')

    def test_all_captured_watermark_sequences_are_reconciled_not_emission_times(self):
        s = Scenario().prime()
        raw = peer(8, seq=1)
        raw['watermarks']['observer'].update(input_mono=10007.9, input_epoch=EPOCH + 7.9, event_seq=0, journal_seq=1)
        raw['watermarks']['ros_journal'].update(event_seq=2, journal_seq=2)
        event1 = dict(peer_event(1, 'ROS_JOURNAL_ALIVE', 'info'), epoch=EPOCH + 7.95, mono=10007.95)
        self.assertFalse(s.step(8, status=raw, events=[event1]))
        self.assertLess(s.evidence.gate.clean_s, 2)  # Captured global/source seq=2 still missing.
        event2 = dict(peer_event(2, 'ROS_JOURNAL_ALIVE', 'info'), epoch=EPOCH + 8.01, mono=10008.01)
        s.evidence.accept(record('peer_events', [event2], 8.1))
        failures, ready = s.evidence.evaluate(1008.11, PERIODS)
        self.assertFalse(failures)
        self.assertTrue(ready)  # Event emission is legitimately newer than min input.

    def test_static_observation_times_do_not_change_frozen_config(self):
        before, after = config(1), config(100)
        after['observed_epoch'] += 900
        after['observed_mono'] += 900
        self.assertEqual(
            soak.normalize_peer_config(before, MACHINE)['sha256'], soak.normalize_peer_config(after, MACHINE)['sha256']
        )
        for mutate in (
            lambda c: c['live'].update(drift=True),
            lambda c: c['live'].update(binary_sha256='0' * 64),
            lambda c: c['live'].update(params_sha256='0' * 64),
            lambda c: c['live'].update(commit='0' * 40),
            lambda c: c['live'].update(config_generation='0' * 64),
        ):
            raw = config(2)
            mutate(raw)
            with self.assertRaises(ValueError):
                soak.normalize_peer_config(raw, MACHINE, REMOTE, 3)


class TransportTests(unittest.TestCase):
    def fake_opener(self, body):
        response = mock.MagicMock()
        response.__enter__.return_value = response
        response.read.return_value = body
        response.status = 200
        response.headers = {'Content-Type': 'application/json'}
        opener = mock.Mock()
        opener.open.return_value = response
        return opener

    def test_http_is_get_bounded_and_auth_only_explicit(self):
        opener = self.fake_opener(b'{"complete": {"unfamiliar": 17}}')
        with mock.patch.object(soak.urllib.request, 'build_opener', return_value=opener):
            result = soak.fetch('http://10.43.0.122/state.json')
            req = opener.open.call_args.args[0]
            self.assertEqual(req.method, 'GET')
            self.assertNotIn('Authorization', req.headers)
            self.assertLessEqual(opener.open.call_args.kwargs['timeout'], 2)
            self.assertEqual(result['data']['complete']['unfamiliar'], 17)
            soak.fetch('http://10.43.0.122/admin/api/monitor', ('test-user', 'test-password'))
            self.assertIn('Authorization', opener.open.call_args.args[0].headers)

    def test_partial_jsonl_invalid_json_and_oversized_responses_block(self):
        for body, jsonl in (
            (b'{"seq": 1}', True),
            (b'{incomplete', False),
            (b'{"x": NaN}', False),
            (b'{"armed":false,"armed":true}', False),
        ):
            with self.subTest(body=body):
                with mock.patch.object(soak.urllib.request, 'build_opener', return_value=self.fake_opener(body)):
                    result = soak.fetch('http://100.110.35.58:8895/events.jsonl', jsonl=jsonl)
                self.assertIn('error', result)
                self.assertEqual(result['raw'], body.decode())
                self.assertNotIn('data', result)
                soak.canonical(result)  # Invalid JSON must remain persistable evidence.
        with mock.patch.object(
            soak.urllib.request, 'build_opener', return_value=self.fake_opener(b'x' * (soak.MAX_BODY + 1))
        ):
            with self.assertRaises(ValueError):
                soak.fetch('http://100.110.35.58:8895/events.jsonl', jsonl=True)

    def test_error_bodies_and_bad_encoding_are_retained_without_qualifying(self):
        error = soak.urllib.error.HTTPError('http://unused', 503, 'busy', {}, io.BytesIO(b'{"message":"busy"}'))
        opener = mock.Mock()
        opener.open.side_effect = error
        with mock.patch.object(soak.urllib.request, 'build_opener', return_value=opener):
            result = soak.fetch('http://unused')
        self.assertEqual(result['http_status'], 503)
        self.assertEqual(result['data'], {'message': 'busy'})
        self.assertIn('error', result)
        with mock.patch.object(soak.urllib.request, 'build_opener', return_value=self.fake_opener(b'bad\xfftext')):
            result = soak.fetch('http://unused')
        self.assertIn('raw', result)
        self.assertIn('error', result)
        self.assertNotIn('data', result)

    def test_timeout_does_not_spawn_more_workers_or_accept_late_result(self):
        with mock.patch.object(soak.threading, 'Thread') as thread:
            worker = soak.HTTPWorker('dut_state', 'http://10.43.0.122/state.json', 1)
            with mock.patch.object(soak, 'timestamp', return_value=soak.timestamp(1000, EPOCH)):
                worker.submit(1000)
            self.assertFalse(worker.submit(1001))
            deadline = worker.drain(1002)
            self.assertEqual(len(deadline), 1)
            self.assertTrue(deadline[0]['deadline'])
            self.assertFalse(worker.submit(1003))
            self.assertFalse(worker.drain(1004))
            late = record('dut_state', dut(0), 0)
            late['latency_s'] = 4
            worker.results.put(late)
            self.assertTrue(worker.drain(1004)[0]['late'])
            self.assertEqual(thread.call_count, 1)

    def test_forced_health_read_preserves_the_next_regular_poll(self):
        # Real hardware: a recovery read at t=2 consumed the t=30 tick, leaving
        # no health observation until t=60 and falsely breaking the clean run.
        with mock.patch.object(soak.threading, 'Thread'):
            worker = soak.HTTPWorker('health', 'http://10.43.0.122/api/health', 30)
            worker.next_due = 1030
            with mock.patch.object(soak, 'timestamp', return_value=soak.timestamp(1002, EPOCH)):
                self.assertTrue(worker.submit(1002))
            self.assertEqual(worker.next_due, 1030)
            queued = worker.tasks.get_nowait()
            worker.results.put(dict(queued, end=soak.timestamp(1002.01, EPOCH + 0.01), latency_s=0.01))
            worker.drain(1002.01)
            with mock.patch.object(soak, 'timestamp', return_value=soak.timestamp(1030.1, EPOCH + 28.1)):
                self.assertTrue(worker.submit(1030.1))
            self.assertEqual(worker.next_due, 1060)

    def test_independent_worker_finishes_while_other_http_is_blocked(self):
        entered, release, healthy_done = threading.Event(), threading.Event(), threading.Event()

        def fake_fetch(url, *unused):
            if url.endswith('/blocked'):
                entered.set()
                release.wait(2)
            else:
                healthy_done.set()
            return {'data': {'ok': True}}

        with mock.patch.object(soak, 'fetch', fake_fetch):
            blocked = soak.HTTPWorker('blocked', 'http://unused/blocked', 1)
            healthy = soak.HTTPWorker('healthy', 'http://unused/healthy', 1)
            try:
                blocked.submit(soak.time.monotonic())
                self.assertTrue(entered.wait(1))
                healthy.submit(soak.time.monotonic())
                self.assertTrue(healthy_done.wait(1))
                self.assertFalse(release.is_set())
            finally:
                blocked.close()
                healthy.close()
                release.set()

    def test_peer_must_be_tailscale_and_urls_cannot_carry_credentials(self):
        for url in (
            'http://127.0.0.1:8895',
            'http://example.com',
            'http://user:password@100.110.35.58',
            'http://100.110.35.58/settings',
        ):
            with self.assertRaises(argparse.ArgumentTypeError):
                soak.base_url(url, peer=True)
        self.assertEqual(soak.base_url('http://100.110.35.58:8895/', peer=True), 'http://100.110.35.58:8895')


class ArtifactAndHookTests(unittest.TestCase):
    def test_warning_storm_retains_every_event_but_shares_prehistory(self):
        with tempfile.TemporaryDirectory() as out:
            store = soak.Artifacts(out, soak.Redactor())
            try:
                store.sample(record('dut_state', dut(0), 0))
                paths = []
                for i in range(40):
                    with mock.patch.object(soak.time, 'monotonic', return_value=1000 + i / 10):
                        paths.append(store.event('warning', 'notice', ['notice'], {}))
                self.assertEqual(len({json.loads(p.read_text())['evidence_batch'] for p in paths}), 1)
                self.assertEqual(len(list(Path(out).rglob('prehistory.jsonl'))), 1)
                self.assertEqual(len((Path(out) / 'events.jsonl').read_text().splitlines()), 40)
            finally:
                store.close()

    def test_all_events_survive_cooldown_and_restart_without_credit_or_secrets(self):
        with tempfile.TemporaryDirectory() as out:
            store = soak.Artifacts(out, soak.Redactor('private-user', 'password-value'))
            try:
                store.sample(
                    record(
                        'dut_state',
                        {
                            'secret': 'hidden',
                            'nested': {'password': 'password-value'},
                            'text': 'password-value',
                            'untouched': [1, 2],
                        },
                        1,
                    )
                )
                first = store.event('failure', 'fault', ['unknown'], {}, hook='scheduled')
                second = store.event('failure', 'fault', ['again'], {}, hook='skipped_cooldown')
                self.assertTrue(first.exists() and second.exists())
                batch = Path(json.loads(first.read_text())['evidence_batch'])
                saved = json.loads((batch / 'prehistory.jsonl').read_text())
                self.assertEqual(saved['data']['untouched'], [1, 2])
                self.assertNotIn('password-value', first.read_text())
                for path in Path(out).rglob('*.jsonl'):
                    self.assertNotIn('password-value', path.read_text())
                    self.assertNotIn('hidden', path.read_text())
                store.atomic('status.json', {'state': 'PASSED'})
            finally:
                store.close()
            resumed = soak.Artifacts(out, soak.Redactor())
            try:
                self.assertTrue(resumed.restart)
                resumed.event('info', 'collector_restart', ['no blind-time credit'], {})
                lines = (Path(out) / 'events.jsonl').read_text().splitlines()
                self.assertEqual(len(lines), 3)
                self.assertEqual(json.loads(lines[-1])['type'], 'collector_restart')
                self.assertEqual(soak.Evidence(arguments(out)).gate.clean_s, 0)
            finally:
                resumed.close()

    def test_low_disk_and_exclusive_output_lock(self):
        with tempfile.TemporaryDirectory() as out:
            store = soak.Artifacts(out, soak.Redactor())
            try:
                with self.assertRaises(ValueError):
                    soak.Artifacts(out, soak.Redactor())
                with mock.patch.object(
                    soak.shutil, 'disk_usage', return_value=argparse.Namespace(total=10**10, free=20)
                ):
                    with self.assertRaisesRegex(OSError, 'low disk'):
                        store.disk_check()
            finally:
                store.close()

    def test_hook_argv_no_shell_credential_leak_timeout_and_cooldown(self):
        hook = soak.FailureHook('/tmp/a hook; literal')
        process = mock.Mock(pid=12345)
        process.poll.return_value = None
        process.wait.return_value = -9
        with (
            mock.patch.object(soak.subprocess, 'Popen', return_value=process) as popen,
            mock.patch.object(soak.os, 'killpg') as killpg,
            mock.patch.dict(os.environ, {'ML_ADMIN_USER': 'user', 'ML_ADMIN_PASSWORD': 'secret'}),
        ):
            self.assertEqual(hook.disposition(100), 'scheduled')
            hook.start(Path('/tmp/event file.json'), 100)
            self.assertEqual(popen.call_args.args[0], ['/tmp/a hook; literal', '/tmp/event file.json'])
            self.assertFalse(popen.call_args.kwargs['shell'])
            self.assertNotIn('ML_ADMIN_PASSWORD', popen.call_args.kwargs['env'])
            self.assertNotIn('ML_ADMIN_USER', popen.call_args.kwargs['env'])
            self.assertEqual(hook.disposition(101), 'skipped_busy')
            self.assertIsNone(hook.poll(109))
            self.assertTrue(hook.poll(110)['timed_out'])
            killpg.assert_called_once()
            self.assertEqual(hook.disposition(111), 'skipped_cooldown')
            self.assertEqual(hook.disposition(160), 'scheduled')


class FakeClock:
    def __init__(self):
        self.mono = 1000.0

    def time(self):
        return EPOCH + self.mono - 1000

    def monotonic(self):
        return self.mono


class FakeStop:
    def __init__(self, clock, interrupt_at=None):
        self.clock, self.interrupt_at = clock, interrupt_at

    def is_set(self):
        return self.interrupt_at is not None and self.clock.mono >= self.interrupt_at

    def wait(self, duration):
        self.clock.mono = round(self.clock.mono + duration, 5)


class RunTests(unittest.TestCase):
    def test_actual_publisher_phase_lag_at_one_hz_credits_only_committed_inputs(self):
        def mutate(source, data, t):
            if source == 'peer_status':
                data.clear()
                data.update(peer(t, producer_lag=0.658))

        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(
                self.execute(out, mutate=mutate, max_duration=50, clean_seconds=20, delays={'peer_status': 0.65}), 0
            )
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['failures'], 0)
            self.assertEqual(status['state'], 'PASSED')
            self.assertGreaterEqual(status['clean_s'], 20)
            self.assertGreater(status['closing']['witness']['receipt_mono'], status['closing']['fence_mono'])

    def test_stale_processed_snapshots_cannot_pass_even_with_fresh_http_and_armed(self):
        def mutate(source, data, t):
            if source == 'peer_status':
                data.clear()
                data.update(peer(t, producer_lag=1.8))

        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, mutate=mutate, max_duration=15), 2)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['clean_s'], 0)
            self.assertGreater(status['failures'], 0)

    def test_initial_status_baseline_drives_first_incremental_get(self):
        history = []

        def mutate(source, data, t):
            if source == 'peer_status':
                data.clear()
                data.update(peer(t, seq=237))

        with tempfile.TemporaryDirectory() as out, mock.patch.object(soak.FailureHook, 'start') as hook:
            self.assertEqual(self.execute(out, mutate=mutate, history=history), 0)
            event_requests = [r for r in history if r['source'] == 'peer_events']
            self.assertTrue(event_requests)
            self.assertEqual(event_requests[0]['suffix'], '?after=237&limit=1000')
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['event_baseline_seq'], 237)
            self.assertEqual(status['failures'], 0)
            hook.assert_not_called()

    def test_realistic_multicadence_startup_has_no_false_failures_or_hooks(self):
        history = []
        with tempfile.TemporaryDirectory() as out, mock.patch.object(soak.FailureHook, 'start') as hook:
            self.assertEqual(self.execute(out, history=history, clean_seconds=35, max_duration=60), 0)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['failures'], 0)
            hook.assert_not_called()
            by_source = {name: [r['start'] for r in history if r['source'] == name] for name in PERIODS}
            self.assertTrue(
                all(abs(b - a - 1) < 0.051 for a, b in zip(by_source['dut_state'], by_source['dut_state'][1:]))
            )
            self.assertTrue(
                any(29 <= b - a <= 31 for a, b in zip(by_source['peer_config'], by_source['peer_config'][1:]))
            )
            self.assertTrue(any(4.9 <= b - a <= 5.1 for a, b in zip(by_source['monitor'], by_source['monitor'][1:])))
            cutoff = status['closing']['cutoff_mono']
            self.assertTrue(all(t > cutoff for t in status['closing']['verified'].values()))
            self.assertGreater(status['closing']['witness']['receipt_mono'], status['closing']['fence_mono'])

    def test_650ms_peer_gets_at_one_hz_do_not_double_count_transport_age(self):
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, delays={'peer_status': 0.65}), 0)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['failures'], 0)

    def test_schema_ack_absent_keeps_core_waiting_without_repair(self):
        with (
            tempfile.TemporaryDirectory() as out,
            mock.patch.object(soak, 'LIVE_PEER_SCHEMA_ACK', None),
            mock.patch.object(soak.FailureHook, 'start') as hook,
        ):
            self.assertEqual(self.execute(out, max_duration=10), 2)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['clean_s'], 0)
            self.assertIn('peer: live schema not acknowledged', status['last_reasons'])
            self.assertEqual(status['failures'], 0)
            hook.assert_not_called()

    def test_initial_missing_samples_wait_then_expire_without_repair_hook(self):
        clock = FakeClock()
        worker_type = self.fake_worker(clock)
        worker_type.drain = lambda worker, now: []  # Pending acquisition, no observed fault.
        with (
            tempfile.TemporaryDirectory() as out,
            mock.patch.object(soak.time, 'monotonic', clock.monotonic),
            mock.patch.object(soak.time, 'time', clock.time),
            mock.patch.object(soak, 'HTTPWorker', worker_type),
            mock.patch.object(soak.FailureHook, 'start') as hook,
            mock.patch.dict(os.environ, {'ML_ADMIN_USER': 'test-user', 'ML_ADMIN_PASSWORD': 'test-password'}),
            mock.patch.object(soak.Artifacts, 'disk_check'),
        ):
            self.assertEqual(
                soak.run(arguments(out, max_duration=47, failure_hook=Path('/mock/failure-hook')), FakeStop(clock)), 2
            )
            events = [json.loads(line) for line in (Path(out) / 'events.jsonl').read_text().splitlines()]
            failures = [event for event in events if event['severity'] == 'failure']
            self.assertEqual(len(failures), 1)
            self.assertGreaterEqual(failures[0]['mono'], 1045)
            hook.assert_not_called()

    def test_closing_refresh_finds_health_fault_before_next_thirty_second_poll(self):
        history = []

        def mutate(source, data, t):
            if source == 'health' and t >= 12:
                data['presses'] = 1

        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, mutate=mutate, history=history, max_duration=20), 2)
            text = (Path(out) / 'events.jsonl').read_text()
            self.assertIn('health.presses: increment', text)
            self.assertTrue(any(r['source'] == 'health' and 1012 <= r['start'] < 1020 for r in history))

    def test_warning_storm_respects_dut_lanes_and_diagnostic_cooldown(self):
        history = []

        def mutate(source, data, t):
            if source == 'dut_state':
                data['ml_reconnects'] = 1 + int(t)

        delays = {
            name: 0.35
            for name in (
                'monitor',
                'health',
                'dut_role',
                'diagnostic_peers',
                'diagnostic_monitor',
                'diagnostic_last_log',
            )
        }
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(
                self.execute(out, mutate=mutate, history=history, delays=delays, max_duration=65, clean_seconds=10000),
                2,
            )
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['failures'], 0)
            diagnostics = [r for r in history if r['source'].startswith('diagnostic_')]
            self.assertGreater(len(diagnostics), 0)
            self.assertLessEqual(len(diagnostics), 9)  # At most three batches in 65 s.
            self.assertGreater(status['warnings'], 50)
            requests = [
                r
                for r in history
                if r['source'] in ('dut_state', 'monitor', 'health', 'dut_role')
                or r['source'].startswith('diagnostic_')
            ]
            for instant in [r['start'] for r in requests]:
                active = [r for r in requests if r['start'] <= instant < r['ready'] - 0.00001]
                self.assertLessEqual(len(active), 2)
                self.assertLessEqual(sum(r['source'] != 'dut_state' for r in active), 1)
            batches = sorted({r['context'] for r in diagnostics})
            self.assertLessEqual(len(batches), 3)

    def fake_worker(self, clock, unarmed=False, delays=None, mutate=None, history=None):
        delays = delays or {}
        history = history if history is not None else []

        class Worker:
            def __init__(self, source, url, period, auth=None, jsonl=False, text=False):
                self.source, self.period, self.pending, self.next_due = source, period, None, 0

            def submit(self, now, suffix='', context=None):
                if self.pending is not None:
                    return False
                delay = delays.get(self.source, 0.05)
                if callable(delay):
                    delay = delay(now, context)
                self.pending = {
                    'start': now,
                    'context': context,
                    'suffix': suffix,
                    'ready': now + delay,
                }
                history.append(dict(self.pending, source=self.source))
                if self.period:
                    while self.next_due <= now:
                        self.next_due += self.period
                return True

            def drain(self, now):
                if self.pending is None or now + 0.00001 < self.pending['ready']:
                    return []
                start, context = self.pending['start'], self.pending['context']
                end = self.pending['ready']
                t = end - 1000 - 0.02  # Fresh server snapshot near response generation.
                status = peer(t)
                if unarmed:
                    status['node']['armed'] = False
                data = {
                    'dut_state': dut(t),
                    'peer_status': status,
                    'peer_events': [],
                    'dut_role': {'ok': True, 'role': 'operator'},
                    'monitor': {'tasks': []},
                    'health': health(t),
                    'peer_config': config(t),
                }.get(self.source, {'ok': True})
                if mutate:
                    mutate(self.source, data, t)
                self.pending = None
                rec = record(self.source, data, t, context=context)
                rec['start'] = soak.timestamp(start, EPOCH + start - 1000)
                rec['end'] = soak.timestamp(end, EPOCH + end - 1000)
                rec['latency_s'] = end - start
                return [rec]

            def close(self):
                pass

        return Worker

    def execute(
        self,
        out,
        max_duration=25,
        interrupt_at=None,
        unarmed=False,
        atomic=None,
        interrupt_on_pass=False,
        delays=None,
        mutate=None,
        history=None,
        clean_seconds=3,
        peer_poll_sec=1.0,
        adaptive_dut_poll=False,
    ):
        clock = FakeClock()
        stop = FakeStop(clock, interrupt_at)
        if interrupt_on_pass:
            original = soak.Artifacts.atomic

            def atomic(store, path, value):
                original(store, path, value)
                if str(path) == 'status.json' and value.get('state') == 'PASSED':
                    stop.interrupt_at = clock.mono

        with (
            mock.patch.object(soak.time, 'monotonic', clock.monotonic),
            mock.patch.object(soak.time, 'time', clock.time),
            mock.patch.object(soak, 'HTTPWorker', self.fake_worker(clock, unarmed, delays, mutate, history)),
            mock.patch.dict(os.environ, {'ML_ADMIN_USER': 'test-user', 'ML_ADMIN_PASSWORD': 'test-password'}),
            mock.patch.object(soak.Artifacts, 'disk_check'),
        ):
            args = arguments(
                out,
                max_duration=max_duration,
                clean_seconds=clean_seconds,
                peer_poll_sec=peer_poll_sec,
                adaptive_dut_poll=adaptive_dut_poll,
            )
            if atomic:
                with mock.patch.object(soak.Artifacts, 'atomic', atomic):
                    return soak.run(args, stop)
            return soak.run(args, stop)

    def test_faster_peer_observation_does_not_increase_dut_http_load(self):
        history = []
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, peer_poll_sec=0.5, history=history), 0)
        dut_reads = [row['start'] for row in history if row['source'] == 'dut_state']
        peer_reads = [row['start'] for row in history if row['source'] == 'peer_status']
        self.assertGreater(len(peer_reads), 1.5 * len(dut_reads))
        self.assertTrue(all(b - a >= 0.95 for a, b in zip(dut_reads, dut_reads[1:])))

    def test_adaptive_healthy_path_keeps_nominal_dut_cadence(self):
        history = []
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, adaptive_dut_poll=True, history=history), 0)
            phase = json.loads((Path(out) / 'phase.json').read_text())
            self.assertTrue(phase['intent']['adaptive_dut_poll'])
            self.assertEqual(phase['intent']['dut_refresh_limits']['per_hour'], 60)
        reads = [row for row in history if row['source'] == 'dut_state']
        self.assertTrue(all(row['context'] is None for row in reads))
        self.assertTrue(all(b['start'] - a['start'] >= 0.95 for a, b in zip(reads, reads[1:])))

    def test_adaptive_refresh_samples_recovery_before_existing_certificate_expires(self):
        # Recorded 482/786/794 age/HTTP pairs. Responses are simulated, not
        # invented intermediate hardware observations: this tests scheduling.
        for age_ms, latency in ((670, 0.131), (590, 0.264), (686, 0.147)):

            def mutate(source, data, t):
                if source == 'dut_state' and 10 <= t < 10.4:
                    data['pstop_machines'][3]['last_reply_ms'] = data['uptime_ms'] - age_ms

            for adaptive in (False, True):
                with self.subTest(age_ms=age_ms, adaptive=adaptive), tempfile.TemporaryDirectory() as out:
                    history = []
                    self.assertEqual(
                        self.execute(
                            out,
                            mutate=mutate,
                            delays={'dut_state': latency},
                            history=history,
                            max_duration=22,
                            clean_seconds=10000,
                            adaptive_dut_poll=adaptive,
                        ),
                        2,
                    )
                    status = json.loads((Path(out) / 'status.json').read_text())
                    text = (Path(out) / 'events.jsonl').read_text()
                    if not adaptive:
                        self.assertGreater(status['failures'], 0)
                        self.assertIn('dut.slot: reply evidence aged out', text)
                        continue
                    self.assertEqual(status['failures'], 0)
                    reads = [row for row in history if row['source'] == 'dut_state']
                    early = [row for row in reads if row['context'] is not None]
                    self.assertEqual(len(early), 1)
                    self.assertLess(early[0]['ready'], early[0]['context']['evidence_expires_mono'])
                    following = reads[reads.index(early[0]) + 1]
                    self.assertGreaterEqual(following['start'] - early[0]['start'], 0.99)
                    self.assertTrue(all(b['start'] - a['start'] >= 0.4999 for a, b in zip(reads, reads[1:])))
                    schedule = [json.loads(line) for line in (Path(out) / 'scheduler.jsonl').read_text().splitlines()]
                    self.assertEqual(sum(row['type'] == 'dut_early_refresh' for row in schedule), 1)

    def test_adaptive_caps_fail_closed_without_starving_auxiliary_polls(self):
        def mutate(source, data, t):
            if source == 'dut_state' and t >= 8:
                data['pstop_machines'][3]['last_reply_ms'] = data['uptime_ms'] - 900

        history = []
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(
                self.execute(
                    out,
                    mutate=mutate,
                    history=history,
                    max_duration=75,
                    clean_seconds=10000,
                    adaptive_dut_poll=True,
                ),
                2,
            )
            schedule = [json.loads(line) for line in (Path(out) / 'scheduler.jsonl').read_text().splitlines()]
            self.assertTrue(any(row['type'] == 'dut_refresh_capped' for row in schedule))
            text = (Path(out) / 'events.jsonl').read_text()
            self.assertIn('dut.slot: reply evidence aged out', text)
            self.assertGreater(json.loads((Path(out) / 'status.json').read_text())['failures'], 0)
        early = [row['start'] for row in history if row['source'] == 'dut_state' and row['context']]
        self.assertGreater(len(early), 6)  # Minute budget eventually recovers.
        for when in early:
            self.assertLessEqual(sum(when - 60 + 0.00001 < t <= when for t in early), 6)
        for source in ('monitor', 'dut_role'):
            starts = [row['start'] for row in history if row['source'] == source]
            self.assertGreater(len(starts), 12)
            self.assertLess(max(b - a for a, b in zip(starts, starts[1:])), 5.2)
        self.assertGreaterEqual(sum(row['source'] == 'health' for row in history), 3)

    def test_adaptive_hour_budget_and_expiration(self):
        clock = FakeClock()
        worker_type = self.fake_worker(clock)
        workers = {name: worker_type(name, 'http://unused', period) for name, period in PERIODS.items()}
        scheduler = soak.RequestScheduler(workers, {}, 5000, adaptive_dut_poll=True)
        state = dut(4000)
        state['pstop_machines'][3]['last_reply_ms'] = state['uptime_ms'] - 700
        scenario = Scenario()
        scenario.step(4000, state=state)
        workers['dut_state'].next_due = 5001
        now = 5000.75
        scheduler.dut_refresh_times.extend(sorted(now - 100 - 50 * i for i in range(60)))
        self.assertIsNone(scheduler.early_dut_context(now, scenario.evidence))
        self.assertEqual(scheduler.dut_refresh_cap_hits, 1)
        self.assertIsNone(scheduler.early_dut_context(now + 0.01, scenario.evidence))
        self.assertEqual(scheduler.dut_refresh_cap_hits, 1)  # Same sample, no log storm.
        scheduler.dut_refresh_times.clear()
        scheduler.dut_refresh_times.extend([now - 3601] * 60)
        context = scheduler.early_dut_context(now, scenario.evidence)
        self.assertIsNotNone(context)
        self.assertEqual(context['early_last_hour'], 0)

    def test_adaptive_does_not_rescue_future_or_stale_reply_values(self):
        for age_ms in (-1, 1700):

            def mutate(source, data, t):
                if source == 'dut_state' and 10 <= t < 10.4:
                    data['pstop_machines'][3]['last_reply_ms'] = data['uptime_ms'] - age_ms

            with self.subTest(age_ms=age_ms), tempfile.TemporaryDirectory() as out:
                self.assertEqual(
                    self.execute(
                        out,
                        mutate=mutate,
                        max_duration=15,
                        clean_seconds=10000,
                        adaptive_dut_poll=True,
                    ),
                    2,
                )
                self.assertGreater(json.loads((Path(out) / 'status.json').read_text())['failures'], 0)
                self.assertIn('dut.slot: stale/future reply', (Path(out) / 'events.jsonl').read_text())

    def test_blocked_early_request_keeps_single_worker_and_expiry_failure(self):
        delays = {'dut_state': 0.05}

        def mutate(source, data, t):
            if source == 'dut_state' and 10 <= t < 10.4:
                data['pstop_machines'][3]['last_reply_ms'] = data['uptime_ms'] - 700
                delays['dut_state'] = 3.0  # The next, early request stays blocked.

        history = []
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(
                self.execute(
                    out,
                    mutate=mutate,
                    delays=delays,
                    history=history,
                    max_duration=12,
                    clean_seconds=10000,
                    adaptive_dut_poll=True,
                ),
                2,
            )
            self.assertIn('dut.slot: reply evidence aged out', (Path(out) / 'events.jsonl').read_text())
            self.assertGreater(json.loads((Path(out) / 'status.json').read_text())['failures'], 0)
        reads = [row for row in history if row['source'] == 'dut_state']
        early = [row for row in reads if row['context']]
        self.assertEqual(len(early), 1)
        self.assertEqual(reads[-1], early[0])  # No replacement while it is pending.

    def test_early_result_just_after_expiry_cannot_hide_gap_between_loop_ticks(self):
        def mutate(source, data, t):
            if source == 'dut_state' and 10 <= t < 10.4:
                data['pstop_machines'][3]['last_reply_ms'] = data['uptime_ms'] - 686

        for offset in (-0.001, 0, 0.001):

            def delay(now, context):
                return context['evidence_expires_mono'] - now + offset if context else 0.05

            # All three responses are processed on the same next supervisor
            # tick. Capture time, not arrival at that tick, decides lateness.
            with self.subTest(offset=offset), tempfile.TemporaryDirectory() as out:
                self.assertEqual(
                    self.execute(
                        out,
                        mutate=mutate,
                        delays={'dut_state': delay},
                        max_duration=12,
                        clean_seconds=10000,
                        adaptive_dut_poll=True,
                    ),
                    2,
                )
                records = [json.loads(line) for line in (Path(out) / 'scheduler.jsonl').read_text().splitlines()]
                result = next(r for r in records if r['type'] == 'dut_refresh_result')
                self.assertAlmostEqual(result['response_end']['mono'] - result['evidence_expires_mono'], offset)
                status = json.loads((Path(out) / 'status.json').read_text())
                if offset > 0:
                    self.assertGreater(status['failures'], 0)
                    self.assertIn('dut.slot: reply evidence aged out', (Path(out) / 'events.jsonl').read_text())
                else:
                    self.assertEqual(status['failures'], 0)

    def test_full_run_only_exits_zero_on_clean_target_and_retains_restart(self):
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out), 0)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['state'], 'PASSED')
            self.assertGreaterEqual(status['clean_s'], 3)
            old_events = (Path(out) / 'events.jsonl').read_text()
            self.assertEqual(self.execute(out, max_duration=2), 2)
            status = json.loads((Path(out) / 'status.json').read_text())
            self.assertEqual(status['state'], 'INCOMPLETE')
            self.assertEqual(status['clean_s'], 0)
            text = (Path(out) / 'events.jsonl').read_text()
            self.assertTrue(text.startswith(old_events))
            self.assertIn('collector_restart', text)

    def test_unarmed_finite_run_and_signal_are_incomplete(self):
        for kwargs in ({'unarmed': True}, {'interrupt_at': 1008.5}):
            with self.subTest(kwargs=kwargs), tempfile.TemporaryDirectory() as out:
                self.assertEqual(self.execute(out, **kwargs), 2)
                status = json.loads((Path(out) / 'status.json').read_text())
                self.assertEqual(status['state'], 'INCOMPLETE')
                self.assertLess(status['clean_s'], status['target_s'])

    def test_storage_failure_never_returns_success(self):
        original = soak.Artifacts.atomic

        def broken(store, path, value):
            if str(path) == 'status.json' and value.get('state') == 'PASSED':
                raise OSError('simulated full disk')
            return original(store, path, value)

        with tempfile.TemporaryDirectory() as out, mock.patch('sys.stderr', new_callable=io.StringIO):
            self.assertEqual(self.execute(out, atomic=broken), 2)
            self.assertNotEqual(json.loads((Path(out) / 'status.json').read_text())['state'], 'PASSED')

    def test_signal_during_final_status_publication_overrides_pass(self):
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(self.execute(out, interrupt_on_pass=True), 2)
            self.assertEqual(json.loads((Path(out) / 'status.json').read_text())['state'], 'INCOMPLETE')

    def test_main_sigterm_never_returns_zero(self):
        def interrupt(args, stop):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            self.assertTrue(stop.is_set())
            return 0

        with mock.patch.object(soak, 'run', interrupt):
            self.assertEqual(
                soak.main([
                    '--dut',
                    'http://10.43.0.122',
                    '--peer',
                    'http://100.110.35.58:8895',
                    '--machine-id',
                    hex(MACHINE),
                    '--remote-id',
                    hex(REMOTE),
                    '--slot',
                    '3',
                    '--iface',
                    'usb',
                    '--out',
                    '/tmp/unused',
                ]),
                143,
            )


class SnapshotEnvelopeRaceTests(unittest.TestCase):
    def raced_snapshot(self, delta=0.003):
        raw = peer(10)
        raw['wire']['snapshot_epoch'] = raw['epoch'] + delta
        raw['wire']['snapshot_age_s'] = -delta
        raw['collector']['observer']['age_s'] = -delta
        raw['watermarks']['observer']['input_epoch'] = raw['epoch'] + delta
        raw['watermarks']['observer']['input_mono'] = raw['mono'] + delta
        return raw

    def test_atomic_snapshot_replacement_after_envelope_uses_receipt_age(self):
        # Real events 58/105: the file was replaced 2-3ms after the server took
        # its envelope timestamp, but before our HTTP response was received.
        for delta in (0.002, 0.003):
            with self.subTest(delta=delta):
                raw = self.raced_snapshot(delta)
                received = raw['epoch'] + 0.020
                result = soak.normalize_peer_status(raw, REMOTE, received, VPN, MACHINE)
                actual = soak.receipt_snapshot_age(raw['wire']['snapshot_epoch'], received)
                self.assertAlmostEqual(result['ages']['wire_snapshot'], actual)
                self.assertAlmostEqual(result['ages']['collector.observer'], actual)
                self.assertGreater(actual, 0)

    def test_negative_age_cannot_hide_inconsistent_or_future_evidence(self):
        cases = []
        bad = self.raced_snapshot()
        bad['wire']['snapshot_age_s'] = -0.5
        cases.append(bad)
        cases.append(self.raced_snapshot(0.5))
        bad = peer(10)
        bad['wire']['snapshot_age_s'] = -0.003
        cases.append(bad)
        for raw in cases:
            with self.subTest(snapshot=raw['wire']['snapshot_epoch']):
                with self.assertRaises(ValueError):
                    soak.normalize_peer_status(raw, REMOTE, raw['epoch'] + 0.020, VPN, MACHINE)


if __name__ == '__main__':
    unittest.main()
