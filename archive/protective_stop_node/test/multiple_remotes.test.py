# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
# !/usr/bin/env python3
import os
import time
import uuid

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from protective_stop_msg.srv import ProtectiveStop as ProtectiveStopSrv
from protective_stop_node.models import PStopRemoteStatusEnum
from protective_stop_node.test_utils.base_test import BaseTestProtectiveStopNode
from protective_stop_node.test_utils.helpers import build_pstop_message

"""
Tests with more than one remote paired at the same time, e.g. the operator's and a safety person's pstop.
"""

# Isolate on a dedicated ROS_DOMAIN_ID (distinct from launch.test.py and unmonitored_mode.test.py).
os.environ['ROS_DOMAIN_ID'] = '79'

machine_uuid = 'machine-uuid'
TEST_HEARTBEAT_TIMEOUT_S = 0.1
TEST_DEACTIVATION_TIMEOUT_S = 3.0
MAX_PSTOP_COUNT = 3
# How long a state is held before checking every heartbeat published in the last part of the hold
HOLD_S = 1.0
SETTLE_S = 0.4


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('protective_stop_node')) / 'launch' / 'protective_stop_node.launch.yaml',
            launch_arguments={
                'machine_uuid': machine_uuid,
                'heartbeat_timeout': str(TEST_HEARTBEAT_TIMEOUT_S),
                'deactivation_timeout': str(TEST_DEACTIVATION_TIMEOUT_S),
                'max_pstop_count': str(MAX_PSTOP_COUNT),
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMultipleRemotes(BaseTestProtectiveStopNode):
    def _call_activate(self, remote_uuid):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = self.machine_uuid
        request.sender_uuid = remote_uuid

        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def _send(self, pressed_by_uuid):
        """Sends one message per remote within half a heartbeat period, spaced out as the node's queue depth is 1."""
        for remote_uuid, pressed in pressed_by_uuid.items():
            self.pstop_publisher.publish(build_pstop_message(self.machine_uuid, remote_uuid, pressed))
            end = time.time() + TEST_HEARTBEAT_TIMEOUT_S / 2 / len(pressed_by_uuid)
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.005)

    def _spin_until_stop_is(self, pressed_by_uuid, stop, timeout=5.0):
        start = time.time()
        while not self.pstop_hb_msgs or self.pstop_hb_msgs[-1].stop != stop:
            if time.time() - start > timeout:
                self.fail(f'/pstop_hb stop never became {stop}')
            self._send(pressed_by_uuid)

    def _hold(self, pressed_by_uuid):
        """Sends heartbeats from the given remotes only and returns the /pstop_hb stop values after SETTLE_S."""
        settled = self.node.get_clock().now() + rclpy.duration.Duration(seconds=SETTLE_S)
        start = time.time()
        while time.time() - start < HOLD_S:
            self._send(pressed_by_uuid)
        stops = [
            msg.stop for msg in self.pstop_hb_msgs if rclpy.time.Time.from_msg(msg.stamp) > settled
        ]
        self.assertTrue(stops, 'No /pstop_hb received while holding')
        return stops

    def _registered_remotes(self, pressed_by_uuid):
        """Keeps the given remotes sending and returns {uuid: status} from the next debug message."""
        after = self.node.get_clock().now()
        start = time.time()
        while True:
            if time.time() - start > 5.0:
                self.fail('No debug message received')
            self._send(pressed_by_uuid)
            if self.pstop_debug_msgs and rclpy.time.Time.from_msg(self.pstop_debug_msgs[-1].timestamp) > after:
                return {
                    remote.uuid: remote.connection_status.status
                    for remote in self.pstop_debug_msgs[-1].connected_remote_pstops
                }

    def _pair_two_released(self):
        remote_a = str(uuid.uuid4())
        remote_b = str(uuid.uuid4())
        self._activate(remote_a)
        self._activate(remote_b)
        self._spin_until_stop_is({remote_a: False, remote_b: False}, False)
        return remote_a, remote_b

    def test_both_released_runs(self):
        remote_a, remote_b = self._pair_two_released()

        self.assertFalse(any(self._hold({remote_a: False, remote_b: False})))

        self._deactivate(remote_a)
        self._deactivate(remote_b)

    def test_any_pressed_remote_stops(self):
        remote_a, remote_b = self._pair_two_released()

        for pressed_uuid in (remote_a, remote_b):
            pressed_by_uuid = {remote_a: False, remote_b: False}
            pressed_by_uuid[pressed_uuid] = True
            self.assertTrue(all(self._hold(pressed_by_uuid)), f'Pressing {pressed_uuid} did not stop')

            self._spin_until_stop_is({remote_a: False, remote_b: False}, False)

        self._deactivate(remote_a)
        self._deactivate(remote_b)

    def test_any_silent_remote_stops(self):
        remote_a, remote_b = self._pair_two_released()

        for silent_uuid, other_uuid in ((remote_a, remote_b), (remote_b, remote_a)):
            self.assertTrue(all(self._hold({other_uuid: False})), f'Losing {silent_uuid} did not stop')

            self._spin_until_stop_is({remote_a: False, remote_b: False}, False)

        self._deactivate(remote_a)
        self._deactivate(remote_b)

    def test_full_registry_never_drops_an_active_remote(self):
        remotes = [str(uuid.uuid4()) for _ in range(MAX_PSTOP_COUNT)]
        for remote_uuid in remotes:
            self._activate(remote_uuid)
        released = dict.fromkeys(remotes, False)
        self._spin_until_stop_is(released, False)

        response = self._call_activate(str(uuid.uuid4()))
        self.assertFalse(response.success)
        self.assertIn('Max pstop count', response.message)

        # A repeated activation from a registered remote is refused without pruning anyone
        response = self._call_activate(remotes[-1])
        self.assertFalse(response.success)
        self.assertIn('Already activated', response.message)

        registered = self._registered_remotes(released)
        for remote_uuid in remotes:
            self.assertIn(remote_uuid, registered)
            self.assertNotEqual(registered[remote_uuid], PStopRemoteStatusEnum.DEACTIVATED.value)

        # The oldest remote still stops the machine
        self.assertTrue(all(self._hold({**released, remotes[0]: True})))
        self._spin_until_stop_is(released, False)

        # Once a remote is deactivated, a new one takes its place and the active ones stay
        self._deactivate(remotes[-1])
        new_uuid = str(uuid.uuid4())
        self._activate(new_uuid)

        registered = self._registered_remotes(dict.fromkeys(remotes[:-1], False))
        self.assertNotIn(remotes[-1], registered)
        for remote_uuid in remotes[:-1]:
            self.assertIn(remote_uuid, registered)
            self.assertNotEqual(registered[remote_uuid], PStopRemoteStatusEnum.DEACTIVATED.value)
        self.assertIn(new_uuid, registered)

        for remote_uuid in [*remotes[:-1], new_uuid]:
            self._deactivate(remote_uuid)
