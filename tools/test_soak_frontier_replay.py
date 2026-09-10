#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""Offline real-timebase replay; no network, DUT, peer, or hooks.

Compact allowlisted extraction from campaign-20260910T053721Z/usb/evidence/
ccf680289b6746a5912773efd0949471-{000006,000012,000019,000025,000034}/
prehistory.jsonl: last two peer headers, processing fences and fast request starts.
All floats are original (rebasing would erase the rounding bug). No payloads,
addresses, credentials, configuration, or full logs are retained. Other healthy
fields use the existing synthetic schema fixture. 000034 is a real receipt gap,
not a frontier regression; its event.json supplies the final evaluation time.

Row: request start mono/epoch, end mono/epoch, peer mono/epoch,
     observer and ros_journal (input_mono, input_epoch, event_seq, journal_seq,
     events_emitted), status_seq, journal_seq_committed, DUT/events request starts.
"""

import copy
import json
import tempfile
import unittest

import soak_stability as soak
from test_soak_stability import EPOCH, PERIODS, arguments, peer

# fmt: off
CASES = {
    6: (
        (98455.193128351, 1789018778.8684595, 98455.400309203, 1789018779.0756397, 816996.010792, 1789018778.903673,
         ((816995.864647279, 1789018778.7575283, 551, 560, 15), (816995.831091588, 1789018778.723972, 560, 560, 125)),
         560, 560, (98455.19308052, 98455.193152527)),
        (98456.157596213, 1789018779.8329275, 98456.369329786, 1789018780.0446608, 816996.978612, 1789018779.871493,
         ((816996.879030097, 1789018779.771911, 551, 560, 15), (816995.831091588, 1789018778.723972, 560, 560, 125)),
         560, 560, (98456.157573009, 98456.157604949)),
    ),
    12: (
        (98645.187049493, 1789018968.8623807, 98645.397453319, 1789018969.0727844, 817186.011558, 1789018968.904439,
         ((817185.892733771, 1789018968.7856147, 551, 563, 15), (817185.83107683, 1789018968.7239573, 563, 563, 128)),
         563, 563, (98645.186980172, 98645.187075832)),
        (98646.203709468, 1789018969.8790407, 98646.412445935, 1789018970.0877771, 817187.015252, 1789018969.908133,
         ((817186.892768637, 1789018969.78565, 551, 563, 15), (817185.83107683, 1789018968.7239573, 563, 563, 128)),
         563, 563, (98646.203659253, 98646.203733403)),
    ),
    19: (
        (98830.179694153, 1789019153.8550253, 98830.375134112, 1789019154.050465, 817370.990388, 1789019153.883269,
         ((817370.862897314, 1789019153.7557788, 551, 568, 15), (817370.831165258, 1789019153.7240453, 568, 568, 131)),
         568, 568, (98830.179653226, 98830.179711496)),
        (98831.196680361, 1789019154.8720114, 98831.401664549, 1789019155.0769956, 817372.006764, 1789019154.899645,
         ((817371.863082796, 1789019154.7559638, 551, 568, 15), (817370.831165258, 1789019153.7240453, 568, 568, 131)),
         568, 568, (98831.196633303, 98831.196703064)),
    ),
    25: (
        (99036.185949228, 1789019359.8612804, 99036.389712405, 1789019360.0650434, 817576.9945, 1789019359.887381,
         ((817576.888444351, 1789019359.781325, 551, 572, 15), (817576.831072641, 1789019359.723953, 572, 572, 134)),
         572, 572, (99036.185913981, 99036.185965648)),
        (99037.200760482, 1789019360.8760917, 99037.414816032, 1789019361.0901473, 817578.012751, 1789019360.905632,
         ((817577.888458823, 1789019360.7813394, 551, 572, 15), (817576.831072641, 1789019359.723953, 572, 572, 134)),
         572, 572, (99037.200709556, 99037.200787072)),
    ),
    34: (
        (99379.163043345, 1789019702.8383746, 99379.377544044, 1789019703.0528753, 817919.974369, 1789019702.867249,
         ((817919.584552387, 1789019702.4774337, 551, 579, 15), (817919.08077373, 1789019701.9736547, 579, 579, 140)),
         579, 579, (99379.16297703, 99379.16306751)),
        (99380.179328481, 1789019703.8546598, 99380.384175878, 1789019704.0595064, 817920.990147, 1789019703.883027,
         ((817920.584558289, 1789019703.4774394, 551, 579, 15), (817920.330673621, 1789019703.2235548, 579, 579, 140)),
         579, 579, (99380.179289247, 99380.179346395)),
    ),
}
# fmt: on


def replay_record(row, index):
    sm, se, em, ee, pm, pe, marks, seq, committed, _ = row
    t = 100 + index

    def shift(value, key=''):
        if isinstance(value, dict):
            return {k: shift(v, k) for k, v in value.items()}
        if isinstance(value, (int, float)):
            if key.endswith('epoch') or key == 'hb_last_stamp':
                return pe + (value - (EPOCH + t))
            if key.endswith('mono'):
                return pm + (value - (10000 + t))
        return value

    raw = shift(peer(t, seq, producer_lag=0.5))
    raw['mono'], raw['epoch'] = pm, pe
    raw['watermarks'] = {
        name: dict(zip(('input_mono', 'input_epoch', 'event_seq', 'journal_seq', 'events_emitted'), mark))
        for name, mark in zip(('observer', 'ros_journal'), marks)
    }
    raw['watermarks']['journal_seq_committed'] = committed
    return dict(source='peer_status', data=raw, start=dict(mono=sm, epoch=se), end=dict(mono=em, epoch=ee))


def install_fast(evidence, row):
    for source, start in zip(('dut_state', 'peer_events'), row[-1]):
        evidence.latest[source] = dict(source=source, start=dict(mono=start), end=dict(mono=start + 0.05))


def established(row):
    """Seed the already-running window; replay real source validation thereafter."""
    evidence = soak.Evidence(arguments(clean_seconds=14400))
    evidence.initialized = True
    evidence.health_clock.certified = True
    evidence.provenance = dict(
        firmware={},
        dut_slot={'port': 8893},
        role='operator',
        peer_config={'snapshot': {'machine': {'udp_port': 8893}}},
    )
    rec = replay_record(row, 0)
    normalized = soak.normalize_peer_status(
        rec['data'], evidence.args.remote_id, row[3], evidence.expected_vpn, evidence.args.machine_id
    )
    for key, value in dict(normalized['progress'], **{k: normalized['counters'][k] for k in ('rx', 'tx')}).items():
        evidence.progress.observe('peer.' + key, value - 1, row[2] - 1)
    for source in PERIODS:
        evidence.latest[source] = dict(source=source, start=dict(mono=row[0]), end=dict(mono=row[2]))
    install_fast(evidence, row)
    evidence.accept(rec)
    evidence.gate.healthy_since = row[0] - 20
    evidence.gate.clean_start = row[0] - 10
    return evidence


class FrontierReplayTests(unittest.TestCase):
    def start_case(self, index=6):
        first, second = CASES[index]
        evidence = established(first)
        reasons, ready = evidence.evaluate(first[2], PERIODS)
        self.assertEqual(reasons, [])
        self.assertTrue(ready)
        self.assertGreater(evidence.gate.clean_s, 0)
        return evidence, second

    def test_real_repeated_watermarks_preserve_exact_credit(self):
        decreases_us = {
            6: 0.4989269655197859,
            12: 0.09793438948690891,
            19: 0.06721529643982649,
            25: 0.18044374883174896,
        }
        for index, decrease in decreases_us.items():
            with self.subTest(event=f'{index:06d}'):
                evidence, second = self.start_case(index)
                previous, credit = evidence.gate.last_frontier, evidence.gate.clean_s
                before = copy.deepcopy(evidence.previous_peer['watermarks'])
                install_fast(evidence, second)
                evidence.accept(replay_record(second, 1))
                p = evidence.previous_peer
                for name, mark in p['watermarks'].items():
                    for key, value in mark.items():
                        self.assertGreaterEqual(value, before[name][key])
                self.assertEqual(p['watermarks']['ros_journal'], before['ros_journal'])
                candidate = p['_mono'] - p['ages']['watermark']
                self.assertAlmostEqual((previous - candidate) * 1e6, decrease, places=6)
                reasons, ready = evidence.evaluate(second[2], PERIODS)
                self.assertEqual(reasons, [])
                self.assertTrue(ready)
                self.assertEqual(evidence.gate.last_frontier, previous)
                self.assertEqual(evidence.gate.clean_s, credit)
                self.assertEqual(evidence.gate.target, 14400)

    def test_000034_real_coverage_gap_still_resets(self):
        evidence, second = self.start_case(34)
        install_fast(evidence, second)
        evidence.accept(replay_record(second, 1))
        self.assertEqual(evidence.evaluate(second[2], PERIODS)[0], [])
        event_mono = 99382.056554735
        # Real latest DUT/events starts from this event.json, still covered.
        for source, start in (('dut_state', 99381.195091567), ('peer_events', 99381.195165988)):
            evidence.latest[source] = dict(start=dict(mono=start), end=dict(mono=start + 0.05))
        reasons, _ = evidence.evaluate(event_mono, PERIODS)
        self.assertEqual(reasons, ['peer_status: coverage gap'])
        self.assertAlmostEqual(event_mono - second[2], 1.672378856994328)
        self.assertEqual(evidence.gate.clean_s, 0)

    def assert_reset(self, evidence, now, reason):
        reasons, _ = evidence.evaluate(now, PERIODS)
        self.assertIn(reason, reasons)
        self.assertEqual(evidence.gate.clean_s, 0)
        self.assertIsNone(evidence.gate.last_frontier)
        self.assertFalse(evidence.frontier_certificates)
        self.assertIsNone(evidence.closing_cutoff)
        self.assertIsNone(evidence.closing_witness)

    def test_native_fence_regressions_cannot_hide_behind_other_source(self):
        for source in ('observer', 'ros_journal'):
            for field in ('input_mono', 'input_epoch', 'event_seq', 'journal_seq', 'events_emitted'):
                with self.subTest(source=source, field=field):
                    evidence, second = self.start_case()
                    rec = replay_record(second, 1)
                    old = evidence.latest['peer_status']['data']['watermarks'][source]
                    rec['data']['watermarks'][source][field] = old[field] - (0.00001 if 'input_' in field else 1)
                    if field == 'journal_seq':
                        mark = rec['data']['watermarks'][source]
                        mark['event_seq'] = min(mark['event_seq'], mark['journal_seq'])
                    if source == 'observer' and field == 'input_mono':
                        # Keep RX within the regressed observer fence so the
                        # independent per-source check, not schema rejection, fires.
                        remote = next(iter(rec['data']['wire']['remotes'].values()))
                        remote['last_rx_mono'] = old[field] - 0.01
                    evidence.closing_cutoff = CASES[6][0][2]
                    evidence.closing_witness = {'receipt_mono': CASES[6][0][2], 'event_seq': old['journal_seq']}
                    install_fast(evidence, second)
                    evidence.accept(rec)
                    label = field.removeprefix('input_')
                    self.assert_reset(evidence, second[2], f'peer.watermark.{source}.{label}: regressed')

    def test_native_global_sequence_loss_is_not_masked_by_maximum(self):
        for field in ('status_seq', 'journal_seq_committed'):
            with self.subTest(field=field):
                evidence, second = self.start_case()
                rec = replay_record(second, 1)
                if field == 'status_seq':
                    rec['data']['events']['seq'] -= 1
                else:
                    rec['data']['watermarks'][field] -= 1
                install_fast(evidence, second)
                evidence.accept(rec)
                self.assert_reset(evidence, second[2], f'peer.{field}: regressed')

    def test_fast_request_time_regression_still_resets(self):
        for source in ('dut_state', 'peer_events'):
            with self.subTest(source=source):
                evidence, second = self.start_case()
                previous_start = evidence.latest[source]['start']['mono']
                install_fast(evidence, second)
                evidence.accept(replay_record(second, 1))
                evidence.latest[source]['start']['mono'] = previous_start - 0.00001
                self.assert_reset(evidence, second[2], source + '.start.mono: raw frontier regressed')

    def test_clock_discontinuity_with_advancing_native_clocks_still_fails(self):
        evidence, second = self.start_case()
        rec = replay_record(second, 1)
        rec['data']['mono'] -= 0.3
        install_fast(evidence, second)
        evidence.accept(rec)
        self.assert_reset(evidence, second[2], 'peer: epoch/monotonic clock discontinuity')

    def test_native_snapshot_reset_still_fails(self):
        evidence, second = self.start_case()
        rec = replay_record(second, 1)
        rec['data']['epoch'] = CASES[6][0][5] - 0.00001
        install_fast(evidence, second)
        evidence.accept(rec)
        self.assert_reset(evidence, second[2], 'peer_status: schema/validation: peer: processing watermark stale')

    def test_traffic_counter_loss_still_fails(self):
        evidence, second = self.start_case()
        rec = replay_record(second, 1)
        next(iter(rec['data']['wire']['remotes'].values()))['rx'] = evidence.previous_peer['counters']['rx'] - 1
        install_fast(evidence, second)
        evidence.accept(rec)
        self.assert_reset(evidence, second[2], 'peer.rx: counter/time reset')

    def test_upward_reconversion_of_unchanged_watermark_cannot_buy_credit(self):
        evidence, second = self.start_case()
        previous, credit = evidence.gate.last_frontier, evidence.gate.clean_s
        rec = replay_record(second, 1)
        # Change just the local receipt conversion by +10 us, preserving raw inputs.
        rec['end']['mono'] += 0.00001
        install_fast(evidence, second)
        evidence.accept(rec)
        self.assertEqual(evidence.evaluate(rec['end']['mono'], PERIODS)[0], [])
        self.assertEqual(evidence.gate.last_frontier, previous)
        self.assertEqual(evidence.gate.clean_s, credit)
        self.assertGreater(evidence.frontier_observation['candidate_mono'], previous)

    def test_receipt_hold_and_unreconciled_journal_cannot_certify_new_credit(self):
        for blocked_by in ('receipt', 'journal'):
            with self.subTest(blocked_by=blocked_by):
                evidence, second = self.start_case(34)  # Both native fences advance.
                credit = evidence.gate.clean_s
                certs = copy.deepcopy(evidence.frontier_certificates)
                install_fast(evidence, second)
                evidence.accept(replay_record(second, 1))
                if blocked_by == 'receipt':
                    evidence.previous_peer['ages']['collector.pcap'] = soak.FRESH_SECONDS + 0.01
                else:
                    evidence.journal.cursor -= 1
                reasons, ready = evidence.evaluate(second[2], PERIODS)
                self.assertEqual(reasons, [])
                self.assertFalse(ready)
                self.assertEqual(evidence.gate.clean_s, credit)
                self.assertEqual(evidence.frontier_certificates, certs)

    def test_real_gap_clears_certificates_and_requires_new_recovery(self):
        evidence, second = self.start_case()
        evidence.evaluate(second[2], PERIODS, ['collector: scheduling/storage coverage gap'])
        self.assertFalse(evidence.frontier_certificates)
        install_fast(evidence, second)
        evidence.accept(replay_record(second, 1))
        self.assertEqual(evidence.evaluate(second[2], PERIODS)[0], [])
        self.assertEqual(evidence.gate.clean_s, 0)
        self.assertIsNone(evidence.gate.clean_start)
        self.assertEqual(evidence.gate.state, 'WAITING')

    def test_slow_refresh_does_not_change_fast_frontier_or_bypass_closing(self):
        evidence, second = self.start_case()
        credit, frontier = evidence.gate.clean_s, evidence.gate.last_frontier
        evidence.gate.target = credit
        # Force closing, with all slow checks still pre-cutoff.
        evidence.evaluate(CASES[6][0][2], PERIODS)
        self.assertIsNotNone(evidence.closing_cutoff)
        self.assertEqual(evidence.gate.state, 'RUNNING')
        install_fast(evidence, second)
        evidence.accept(replay_record(second, 1))
        for source in ('monitor', 'dut_role', 'health', 'peer_config'):
            evidence.latest[source] = dict(start=dict(mono=second[0]), end=dict(mono=second[2]))
        self.assertEqual(evidence.evaluate(second[2], PERIODS)[0], [])
        self.assertEqual(evidence.gate.last_frontier, frontier)
        self.assertEqual(evidence.gate.clean_s, credit)
        self.assertEqual(evidence.gate.state, 'RUNNING')
        self.assertIsNone(evidence.closing_witness)

    def test_status_and_event_observations_attribute_previous_and_new_bounds(self):
        evidence, second = self.start_case()
        previous = evidence.gate.last_frontier
        install_fast(evidence, second)
        evidence.accept(replay_record(second, 1))
        reasons, _ = evidence.evaluate(second[2], PERIODS, ['collector: scheduling/storage coverage gap'])
        observation = evidence.frontier_observation
        self.assertEqual(observation['previous_mono'], previous)
        self.assertLess(observation['candidate_mono'], previous)
        self.assertEqual(observation['new_mono'], previous)
        self.assertIsNone(observation['certified_mono'])
        for key in ('previous_argmin', 'candidate_argmin', 'argmin'):
            argmin = observation[key]
            self.assertEqual(argmin['source'], 'peer.watermark.ros_journal')
            self.assertIn(argmin['field'], ('input_mono', 'input_epoch'))
            self.assertIn('raw_value', argmin)
            self.assertIn('converted_mono', argmin)
            self.assertIn('start', argmin['request_timebase'])
            self.assertIn('end', argmin['request_timebase'])
            self.assertIn('peer_epoch', argmin['request_timebase'])
        with tempfile.TemporaryDirectory() as out:
            store = soak.Artifacts(out, soak.Redactor())
            try:
                document = soak.status_document(evidence, store, 1, 0, reasons)
                self.assertEqual(document['frontier'], observation)
                path = store.event('failure', 'acceptance_fault', reasons, evidence.latest, {'frontier': observation})
                self.assertEqual(json.loads(path.read_text())['supporting_observations']['frontier'], observation)
            finally:
                store.close()

    def test_rejected_http_record_cannot_retime_last_valid_peer_evidence(self):
        evidence, second = self.start_case()
        rec = replay_record(second, 1)
        rec['end']['epoch'] -= 100
        evidence.accept(rec)
        # Diagnostic derivation must remain total even for a failed clock/schema
        # sample; it must use the timebase paired with the last valid peer data.
        reasons, _ = evidence.evaluate(second[2], PERIODS)
        self.assertTrue(reasons)
        self.assertEqual(evidence.gate.clean_s, 0)
        soak.canonical(evidence.frontier_observation)


if __name__ == '__main__':
    unittest.main()
