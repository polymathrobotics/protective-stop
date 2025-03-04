# !/usr/bin/env python3
import pytest
from protective_stop_msg.msg import (
    ProtectiveStopHeartbeat,
)
import threading
import launch_testing.util
import launch_testing.actions
import launch_testing
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
from std_msgs.msg import Bool
from protective_stop_msg.srv import (
    ProtectiveStop as ProtectiveStopSrv,
)

from protective_stop_node.protective_stop_node import (
    PROTECTIVE_STOP_HB_TOPIC,
)
import rclpy
import unittest
import uuid

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
    # Start in debug mode log level
    protective_stop_node = LaunchNode(
        package="protective_stop_node",
        executable="protective_stop_node",
        name="protective_stop_node",
        output="screen",
        parameters=[
            {"machine_uuid": machine_uuid},
            {"heartbeat_timeout": TEST_HEARTBEAT_TIMEOUT_S},
            {"deactivation_timeout": TEST_DEACTIVATION_TIMEOUT_S},
            {"max_pstop_count": MAX_PSTOP_COUNT},
        ],
    )

    # Lifecycle Manager
    nav2_lifecycle_manager = LaunchNode(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {"autostart": True},
            {"bond_timeout": 0.0},
            {"node_names": ["protective_stop_node"]},
        ],
    )

    context = {"protective_stop_node": protective_stop_node}
    return (
        LaunchDescription(
            [protective_stop_node, nav2_lifecycle_manager, launch_testing.actions.ReadyToTest()]
        ),
        context,
    )


class TestProtectiveStopNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.done_event = threading.Event()
        self.uuid = str(uuid.uuid4())

        self.node = rclpy.create_node(
            "test_node",
        )
        self.pstop_hb_msgs = []

        self.unsafe_mode_client = self.node.create_client(
            ProtectiveStopSrv, "/protective_stop_node/toggle_unsafe_mode"
        )
        if not self.unsafe_mode_client.wait_for_service(timeout_sec=5.0):
            self.fail("Service /protective_stop_node/toggle_unsafe_mode not available")

        self.pstop_bool_subscriber = self.node.create_subscription(
            ProtectiveStopHeartbeat,
            PROTECTIVE_STOP_HB_TOPIC,
            self.protective_stop_hb_callback,
            1,
        )

    def tearDown(self):
        self.node.destroy_node()

    def protective_stop_hb_callback(self, msg):
        self.pstop_hb_msgs.append(msg)

    def test_unsafe_mode(self):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = machine_uuid
        request.sender_uuid = self.uuid
        self.unsafe_mode_client.call_async(request)

        future = self.unsafe_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertTrue(response.success)

        while not self.pstop_hb_msgs:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        while not self.pstop_hb_msgs[-1].stop:
            rclpy.spin_once(self.node, timeout_sec=0.1)


"""
todo:
- unsafe mode!!
- multiple p-stops test


- publish heartbeat with foreign sender_uuid
- test for two pstops that disagree on whether connected or not
- what if two pstops with same uuid publish?

"""
