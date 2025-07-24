# !/usr/bin/env python3
import pytest
import launch_testing.actions
import launch_testing
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from rcl_interfaces.srv import SetParameters

from protective_stop_node.test_utils.base_test import BaseTestProtectiveStopNode
import rclpy
import time

"""
Separating out the unsafe mode tests to a separate file to keep the test file clean and unlikely to suffer any kind of cross-test pollution.
"""


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
                    "is_user_monitored": "False",
                    "machine_uuid": machine_uuid,
                    "heartbeat_timeout": str(TEST_HEARTBEAT_TIMEOUT_S),
                    "deactivation_timeout": str(TEST_DEACTIVATION_TIMEOUT_S),
                    "max_pstop_count": str(MAX_PSTOP_COUNT),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestProtectiveStopNode(BaseTestProtectiveStopNode):
    def test_setting_unmonitored_mode(self):
       # Wait for a heartbeat with stop == False
        timeout = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        self.assertFalse(self.pstop_hb_msgs[-1].stop)
