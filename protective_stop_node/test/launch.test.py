# !/usr/bin/env python3
import pytest
import launch_testing.util
import launch_testing.actions
import launch_testing
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from protective_stop_msg.srv import ProtectiveStop as ProtectiveStopSrv

from protective_stop_node.models import PStopRemoteStatusEnum
from protective_stop_node.test_utils.helpers import build_pstop_message
from protective_stop_node.test_utils.base_test import BaseTestProtectiveStopNode
import rclpy
import time
import uuid


machine_uuid = "machine-uuid"
TEST_HEARTBEAT_TIMEOUT_S = 0.1
TEST_DEACTIVATION_TIMEOUT_S = 3.0
TIMEOUT_PADDING = 0.05
MAX_PSTOP_COUNT = 3


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("protective_stop_node"))
                    / "launch"
                    / "protective_stop_node.launch.yaml",
                launch_arguments={
                    "machine_uuid": machine_uuid,
                    "heartbeat_timeout": str(TEST_HEARTBEAT_TIMEOUT_S),
                    "deactivation_timeout": str(TEST_DEACTIVATION_TIMEOUT_S),
                    "max_pstop_count": str(MAX_PSTOP_COUNT),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


"""
TODO(troy):
- put launch test on different domain_id to maintain purity
"""


class TestProtectiveStopNode(BaseTestProtectiveStopNode):
    # Set class variables for this specific test
    machine_uuid = machine_uuid
    TEST_HEARTBEAT_TIMEOUT_S = TEST_HEARTBEAT_TIMEOUT_S
    TEST_DEACTIVATION_TIMEOUT_S = TEST_DEACTIVATION_TIMEOUT_S
    TIMEOUT_PADDING = TIMEOUT_PADDING
    MAX_PSTOP_COUNT = MAX_PSTOP_COUNT

    def test_activation_lifecycle(self):
        self._activate()

        self._deactivate()

    def test_err_for_multiple_activations(self):
        self._activate()

        request = ProtectiveStopSrv.Request()
        request.recv_uuid = machine_uuid
        request.sender_uuid = self.uuid

        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertFalse(response.success)

    def test_err_uuid_mismatch_state_transition_service(self):
        request = ProtectiveStopSrv.Request()
        # Set to unknown machine uuid
        request.recv_uuid = str(uuid.uuid4())
        request.sender_uuid = self.uuid

        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertIsNotNone(response, "No response received from /state_transition_service")
        self.assertFalse(response.success)
        self.assertEqual(response.message, "Unknown receiver uuid")
        self.assertEqual(response.params.machine_uuid, machine_uuid)

    def test_err_deactivating_before_activating(self):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = machine_uuid
        request.sender_uuid = self.uuid

        future = self.deactivate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertIsNotNone(response, "No response received from /state_transition_service")
        self.assertFalse(response.success)
        self.assertEqual(response.message, "Deactivation failed. UUID not in connected list")

    def test_multiple_activations_deactivations(self):
        for _ in range(MAX_PSTOP_COUNT):
            self._activate()
            self._spin_till_ok()
            self._deactivate()

    def test_max_connections(self):
        for _ in range(MAX_PSTOP_COUNT + 1):
            self._activate(str(uuid.uuid4()))
            self._deactivate(str(uuid.uuid4()))

    def test_receives_ok_messages(self):
        self._activate()
        self._spin_till_ok()
        self._deactivate()

    def test_presses_pstop(self):
        self._activate()
        self._spin_till_ok()

        self.assertFalse(self.pstop_hb_msgs[-1].stop, "Protective stop bool not set to true")

        PSTOP_PRESSED = True

        while not self.pstop_hb_msgs[-1].stop:
            self.pstop_publisher.publish(
                build_pstop_message(machine_uuid, self.uuid, PSTOP_PRESSED)
            )
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S - TIMEOUT_PADDING)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.pstop_hb_msgs[-1].stop, "Protective stop bool not set to false")

        PSTOP_PRESSED = False

        while self.pstop_hb_msgs[-1].stop:
            self.pstop_publisher.publish(
                build_pstop_message(machine_uuid, self.uuid, PSTOP_PRESSED)
            )
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S - TIMEOUT_PADDING)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertFalse(self.pstop_hb_msgs[-1].stop, "Protective stop bool not set to false")

        self._deactivate()

    def test_exceeds_heartbeats(self):
        self._activate()
        self._spin_till_ok()

        while not self.pstop_hb_msgs[-1].stop:
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S + 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        while self.pstop_msgs[-1].connection_status.status != PStopRemoteStatusEnum.UNSTABLE.value:
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S + 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self._deactivate()

    def test_recovers_from_unstable(self):
        self._activate()
        self._spin_till_ok()

        while not self.pstop_hb_msgs[-1].stop:
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S + 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        while self.pstop_msgs[-1].connection_status.status != PStopRemoteStatusEnum.UNSTABLE.value:
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S + 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self._spin_till_ok()
        self._deactivate()

    def test_deactivates_after_set_timeout(self):
        self._activate()
        self._spin_till_ok()

        # Spin till unstable
        while self.pstop_msgs[-1].connection_status.status != PStopRemoteStatusEnum.UNSTABLE.value:
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S + 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)


        time.sleep(TEST_DEACTIVATION_TIMEOUT_S)

        # Should be able to re-activate since it has been deactivated internally
        self._activate()
        self._deactivate()
