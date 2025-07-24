#!/usr/bin/env python3
import threading
import time
import unittest
import uuid

import rclpy
from protective_stop_msg.msg import (
    ProtectiveStop,
    ProtectiveStopDebug,
    ProtectiveStopHeartbeat,
)
from protective_stop_msg.srv import ProtectiveStop as ProtectiveStopSrv

from protective_stop_node.protective_stop_node import (
    PROTECTIVE_STOP_TOPIC,
    PROTECTIVE_STOP_DEBUG_TOPIC,
    PROTECTIVE_STOP_HB_TOPIC,
)
from protective_stop_node.test_utils.helpers import build_pstop_message


class BaseTestProtectiveStopNode(unittest.TestCase):
    """Base test class for protective stop node tests."""

    # These should be set by the child class or test module
    machine_uuid = "machine-uuid"
    TEST_HEARTBEAT_TIMEOUT_S = 0.1
    TEST_DEACTIVATION_TIMEOUT_S = 3.0
    TIMEOUT_PADDING = 0.05
    MAX_PSTOP_COUNT = 3

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
        params = self.node.get_parameters(["use_sim_time"])
        if params:
            use_sim_time_value = params[0].value
            print("use_sim_time:", use_sim_time_value)

        self.activating_msgs = []
        self.pstop_msgs = []
        self.pstop_hb_msgs = []
        self.pstop_debug_msgs = []

        self.activate_client = self.node.create_client(
            ProtectiveStopSrv, "/protective_stop_node/activate"
        )
        if not self.activate_client.wait_for_service(timeout_sec=5.0):
            self.fail("Service /protective_stop_node/activate not available")

        self.deactivate_client = self.node.create_client(
            ProtectiveStopSrv, "/protective_stop_node/deactivate"
        )
        if not self.deactivate_client.wait_for_service(timeout_sec=5.0):
            self.fail("Service /protective_stop_node/deactivate not available")

        self.pstop_publisher = self.node.create_publisher(
            ProtectiveStop,
            PROTECTIVE_STOP_TOPIC,
            1,
        )

        # Subscribers
        self.pstop_subscriber = self.node.create_subscription(
            ProtectiveStop,
            PROTECTIVE_STOP_TOPIC,
            self.protective_stop_callback,
            1,
        )

        self.pstop_bool_subscriber = self.node.create_subscription(
            ProtectiveStopHeartbeat,
            PROTECTIVE_STOP_HB_TOPIC,
            self.protective_stop_hb_callback,
            1,
        )

        self.pstop_debug_subscriber = self.node.create_subscription(
            ProtectiveStopDebug,
            PROTECTIVE_STOP_DEBUG_TOPIC,
            self.protective_stop_debug_callback,
            1,
        )

    def tearDown(self):
        self.node.destroy_node()

    def protective_stop_debug_callback(self, msg):
        self.pstop_debug_msgs.append(msg)

    def protective_stop_hb_callback(self, msg):
        self.pstop_hb_msgs.append(msg)

    def protective_stop_callback(self, msg):
        # Ignore messages sent by this node
        if msg.sender_uuid == self.uuid:
            return

        self.pstop_msgs.append(msg)

    def _activate(self, uuid=None):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = self.machine_uuid
        request.sender_uuid = self.uuid if not uuid else uuid

        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertIsNotNone(response, "No response received from /state_transition_service")
        self.assertTrue(response.success)

    def _deactivate(self, uuid=None):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = self.machine_uuid
        request.sender_uuid = self.uuid if not uuid else uuid

        future = self.deactivate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertIsNotNone(response, "No response received from /state_transition_service")

    def _spin_till_ok(self, timeout=5.0):
        now = time.time()
        while self.pstop_hb_msgs[-1].stop if self.pstop_hb_msgs else True:
            if time.time() - now > timeout:
                self.fail("Timeout exceeded")
            self.pstop_publisher.publish(build_pstop_message(self.machine_uuid, self.uuid, False))
            time.sleep(self.TEST_HEARTBEAT_TIMEOUT_S - self.TIMEOUT_PADDING)
            rclpy.spin_once(self.node, timeout_sec=0.1)
