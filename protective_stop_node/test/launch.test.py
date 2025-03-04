# !/usr/bin/env python3
import pytest
from builtin_interfaces.msg import Time
from protective_stop_msg.msg import (
    ProtectiveStop,
    ProtectiveStopDebug,
    ProtectiveStopHeartbeat,
)
import threading
import launch_testing.util
import launch_testing.actions
import launch_testing
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
from protective_stop_msg.srv import (
    ProtectiveStop as ProtectiveStopSrv,
)

from protective_stop_node.protective_stop_node import (
    PROTECTIVE_STOP_TOPIC,
    PROTECTIVE_STOP_DEBUG_TOPIC,
    PROTECTIVE_STOP_HB_TOPIC,
)
from protective_stop_node.models import  PStopRemoteStatusEnum
import rclpy
import unittest
import time
import uuid


# Define the PStopMsg message structure as a dictionary based on the .msg file
# Note: This assumes `roslibpy.Message` can be used in this format, otherwise, adjustments will be needed.
PSTOP_MESSAGE_VERSION = 0

# Create a function that builds the PStopMsg dictionary
def build_pstop_message(receiver_uuid, sender_uuid, pstop_pressed, counter=0):
    timestamp = Time(sec=int(time.time()), nanosec=int((time.time() % 1) * 1e9))

    # Build the payload to compute the checksum
    msg = ProtectiveStop(
        version=PSTOP_MESSAGE_VERSION,
        pstop_pressed=pstop_pressed,
        stamp=timestamp,
        sender_uuid=sender_uuid,
        receiver_uuid=receiver_uuid,
        counter=counter,
        checksum_type="CRC-16",
    )

    return msg


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
        arguments=[
            "--ros-args",
            "--log-level",
            "DEBUG",
            "--log-level",
            "rcl:=WARN",
            "--log-level",
            "rclcpp:=WARN",
            "--log-level",
            "rclpy:=WARN",
            "--log-level",
            "rmw:=WARN",
            "--log-level",
            "rmw_cyclonedds_cpp:=WARN",
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


"""
TODO(troy):
- put launch test on different domain_id to maintain purity

"""


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
        # self.node.declare_parameter("use_sim_time", True)
        params = self.node.get_parameters(["use_sim_time"])
        if params:
            use_sim_time_value = params[0].value
            print("use_sim_time:", use_sim_time_value)

        # self.executor = SingleThreadedExecutor()
        # self.executor.add_node(self.node)

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
            # qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        self.pstop_debug_subscriber = self.node.create_subscription(
            ProtectiveStopDebug,
            PROTECTIVE_STOP_DEBUG_TOPIC,
            self.protective_stop_debug_callback,
            1,
            # qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
        )

        # self.spin_thread = threading.Thread(target=self._spin_fn, args=(self.executor,))
        # self.spin_thread.start()

        # self.node.get_logger().info("SETUP")
        # while rclpy.ok():
        # sub_count = self.pstop_publisher.get_subscription_count()
        # if sub_count >= 2:
        # break
        # else:
        # print(f"Waiting for subscriptions... {sub_count}")
        # time.sleep(0.01)

    def tearDown(self):
        # self.spin_thread.join()
        self.node.destroy_node()
        # self.executor.shutdown()

    # def _spin_fn(self, executor):
    # rclpy.spin(self.node, executor=executor)

    def protective_stop_debug_callback(self, msg):
        # self.node.get_logger().info("Received message")
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
        request.recv_uuid = machine_uuid
        request.sender_uuid = self.uuid if not uuid else uuid

        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        self.assertIsNotNone(response, "No response received from /state_transition_service")
        self.assertTrue(response.success)

    def _deactivate(self, uuid=None):
        request = ProtectiveStopSrv.Request()
        request.recv_uuid = machine_uuid
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
            self.pstop_publisher.publish(build_pstop_message(machine_uuid, self.uuid, False))
            time.sleep(TEST_HEARTBEAT_TIMEOUT_S - TIMEOUT_PADDING)
            rclpy.spin_once(self.node, timeout_sec=0.1)

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
