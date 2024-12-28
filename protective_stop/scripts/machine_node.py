#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.executors import SingleThreadedExecutor
from pstop_msg.msg import PStopMsg
from rclpy.duration import Duration
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.event_handler import QoSSubscriptionEventType

from rclpy.lifecycle import State, TransitionCallbackReturn, LifecycleNode
from lifecycle_msgs.msg import State


class MachineNode(LifecycleNode):
    def __init__(self, node_name):    
        super().__init__(node_name)
        self.get_logger().debug(f'Initializing {node_name}')

        # Declaring ROS2 params
        self.declare_parameter('machine_UUID', [0x00] * 16)
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('pstop_auto_accept', True)
        self.declare_parameter('pstops_connected', [""])

        self.timer = None  # Timer will be initialized during configuration
        self.publisher_pstop = None  # To be initialized in on_activate
        self.publisher_status = None  # To be initialized in on_activate
        self.reply = PStopMsg()  # Primary message, tracks global state, with changes per PSTOP sent
        self.reply.state = State(id=State.PRIMARY_STATE_UNCONFIGURED, label="unconfigured")
        self.pstop_data = {}  # Dictionary to store all connected PStops
        self.param_event_handler = None  # To be initialized in on_configure
        self.get_logger().info('pstop_machine_node node has been initialized')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring node '{self.get_name()}' in state '{state.label}'.")

        # Initialize timer
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        # Initialize parameters
        self.machine_UUID = list(map(lambda x: x & 0xFF, self.get_parameter('machine_UUID').get_parameter_value().integer_array_value))
        self.reply.id.uuid = self.machine_UUID
        self.pstop_auto_accept = self.get_parameter('pstop_auto_accept').get_parameter_value().bool_value
        self.pstop_array = self.get_parameter('pstops_connected').get_parameter_value().string_array_value

        # Set up a ParameterEventHandler to listen for all parameter changes
        self.param_event_handler = ParameterEventHandler(self)
        self.param_event_handler.add_parameter_callback(
            parameter_name='heartbeat_timeout',
            node_name=self.get_name(),
            callback=self.parameter_callback
        )
        self.param_event_handler.add_parameter_callback(
            parameter_name='machine_UUID',
            node_name=self.get_name(),
            callback=self.parameter_callback
        )
        self.param_event_handler.add_parameter_callback(
            parameter_name='pstops_connected',
            node_name=self.get_name(),
            callback=self.parameter_callback
        )
        self.param_event_handler.add_parameter_callback(
            parameter_name='pstop_auto_accept',
            node_name=self.get_name(),
            callback=self.parameter_callback
        )

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating node '{self.get_name()}' in state '{state.label}'.")
        
        # Start Timer
        self.timer = self.create_timer(self.heartbeat_timeout / 3.0, self.timer_callback)

        # Pubs and Subs
        self.subscriber_pstop = self.create_subscription(PStopMsg, 'protective_stop', self.pstop_callback, 1)
        self.publisher_pstop = self.create_publisher(PStopMsg, 'protective_stop', 1)
        self.publisher_status = self.create_publisher(Bool, 'pstop', 1)  # Communicates to HAL about overall status
        
        self.get_logger().info("Node activated successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating node '{self.get_name()}' in state '{state.label}'.")
        if self.timer is not None:
            self.timer.cancel()
        if self.subscriber_pstop:
            self.destroy_subscription(self.subscriber_pstop)
            self.subscriber_pstop = None
        if self.publisher_pstop:
            self.destroy_publisher(self.publisher_pstop)
            self.publisher_pstop = None
        if self.publisher_status:
            self.destroy_publisher(self.publisher_status)
            self.publisher_status = None
        self.get_logger().info("Node deactivated successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down node '{self.get_name()}' in state '{state.label}'.")
        # Clean up resources, close connections, etc.
        return TransitionCallbackReturn.SUCCESS
        
    def on_finalize(self, state: State):
        self.destroy_node()
        rclpy.shutdown()
        
    def pstop_callback(self, msg):
        """
        Main subscriber callback, most machine node logic happens here
        """
        uuid = tuple(msg.id.uuid.tolist())  # just for clarity

        if uuid == tuple(self.machine_UUID):
            self.get_logger().debug('Ignoring self-generated message')
        else:  # Not self generated
            self.get_logger().debug(f'❗ Got message from: {uuid}')

            self.pstop_data.setdefault(uuid, {"last_timestamp": 0, "counter": 0, "state": 0, "message": 0})
            self.pstop_data[uuid].update({
                "last_timestamp": msg.stamp,
                "counter": self.pstop_data[uuid]["counter"] + 1,
                "state": msg.state,
                "message": msg.message
            })

            self.reply.stamp = self.get_clock().now().to_msg()
            self.reply.counter = self.pstop_data[uuid]["counter"]
            self.reply.receiver_id.uuid = uuid
            self.reply.heartbeat_timeout.sec = int(self.heartbeat_timeout)
            self.reply.heartbeat_timeout.nanosec = int((self.heartbeat_timeout - self.reply.heartbeat_timeout.sec) * 1e9)
            self.reply.received_stamp = msg.stamp
            self.reply.received_counter = msg.counter
            self.get_logger().debug(f'❗ Sending: {self.reply}')
            self.publisher_pstop.publish(self.reply)

    def parameter_callback(self, event):
        """
        Callback for any parameter change.
        """
        self.get_logger().info(f'Parameter changed: {event.name}')
        if event.name == "heartbeat_timeout":
            self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value
        if event.name == "machine_UUID":
            self.machine_UUID = list(map(lambda x: x & 0xFF, self.get_parameter('machine_UUID').get_parameter_value().integer_array_value))
            self.reply.id.uuid = self.machine_UUID
        if event.name == "pstops_connected":
            pass  # TODO fill in logic here to convert to/from string array
        if event.name == "pstop_auto_accept":
            self.pstop_auto_accept = self.get_parameter('pstop_auto_accept').get_parameter_value().bool_value

    def format_pstop_msg(self, msg):  # Just helps with formatting for log readability
        return (
            f"Message: {msg.message}\n"
            f"State:\n  ID: {msg.state.id}\n  Label: {msg.state.label}\n"
            f"Stamp:\n  Seconds: {msg.stamp.sec}\n  Nanoseconds: {msg.stamp.nanosec}\n"
            f"ID: {list(msg.id.uuid)}\n"
            f"Receiver ID: {list(msg.receiver_id.uuid)}\n"
            f"Counter: {msg.counter}\n"
            f"Heartbeat Timeout:\n  Seconds: {msg.heartbeat_timeout.sec}\n  "
            f"Nanoseconds: {msg.heartbeat_timeout.nanosec}\n"
            f"Checksum Type: {msg.checksum_type}\n"
            f"Checksum Value: {msg.checksum_value}\n"
            f"Received Stamp:\n  Seconds: {msg.received_stamp.sec}\n  "
            f"Nanoseconds: {msg.received_stamp.nanosec}\n"
            f"Received Counter: {msg.received_counter}\n\n"
        )

    def timer_callback(self):
        """
        Handles loss of comms or timeouts
        """
        self.get_logger().info(f"State:\n{self.format_pstop_msg(self.reply)}")
        val = Bool()
        val.data = bool(self.reply.message)
        self.publisher_status.publish(val)


def main(args=None):
    rclpy.init(args=args)
    machine_node = MachineNode("pstop_machine_node")
    try:
        rclpy.spin(machine_node)
    except KeyboardInterrupt:
        machine_node.get_logger().info('KeyboardInterrupt received, shutting down node')
    except Exception as e:
        machine_node.get_logger().error('An error occurred: %s' % str(e))
    finally:
        machine_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

