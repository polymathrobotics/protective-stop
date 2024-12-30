#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.executors import SingleThreadedExecutor
from pstop_msg.msg import PStopMsg
from rclpy.duration import Duration
from rclpy.time import Time
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.event_handler import QoSSubscriptionEventType

from rclpy.lifecycle import State, TransitionCallbackReturn, LifecycleNode
from lifecycle_msgs.msg import State

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter



class MachineNode(LifecycleNode):
    def __init__(self, node_name):    
        super().__init__(node_name)
        self.get_logger().debug(f'Initializing {node_name}')

        # Declaring ROS2 params
        self.declare_parameter('machine_UUID', [0x00] * 16,ParameterDescriptor(description='uint8[16] UUID from unique_identifier_msgs'))
        self.declare_parameter('heartbeat_timeout', 1.0,ParameterDescriptor(description='timeout float value in seconds before pstop is triggered'))
        self.declare_parameter('pstop_auto_accept', True,ParameterDescriptor(description='If set to false, need to explicitly create an allow list in pstops_connected'))
        self.declare_parameter('pstops_connected', [""],ParameterDescriptor(description='array of UUID values for all connected pstops'))

        self.timer = None  # Timer will be initialized during configuration
        self.publisher_pstop = None  # To be initialized in on_activate
        self.publisher_status = None  # To be initialized in on_activate
        self.param_event_handler = None  # To be initialized in on_configure
        self.get_logger().info('pstop_machine_node node has been initialized')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring node '{self.get_name()}' in state '{state.label}'.")

        self.reply = PStopMsg()  # Primary message, tracks global state, with changes per PSTOP sent
        self.pstop_data = {}  # Dictionary to store all connected PStops

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
        self.timer = self.create_timer(self.heartbeat_timeout / 10.0, self.timer_callback)

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
        # TODO: Review what actually needs to happen here.
        #self.destroy_publisher(self.publisher_pstop)
        #self.destroy_publisher(self.publisher_status)
        #self.destroy_subscription(self.subscriber_pstop)
        #self.destroy_timer(self.timer)
        #self.destroy_node()
        return TransitionCallbackReturn.SUCCESS
        
    def pstop_callback(self, msg):
        """
        Main subscriber callback, most machine node logic happens here
        For the pstop output topic to be set true (IE all is well), all of the following must be met:
        - version matches
        - message value is set to 1 (OK)
        - timestamp delta between messages must not exceed heartbeat_timeout
        - id must be on pstop connected list (if not set to auto accept)
        - receiver_id must match this machine node's uuid
        - counter must not skip more than 3 messages
        - checksum type and value must be valid
        - received_stamp and received_counter must be valid for the machine node
            Valid meaning not in the future, not further behind than heartbeat_timeout, not skipping more than 3 messages
        On top of the above, the pstop node must go through a connection process:
        - Send a connection PStopMsg with state field set to "TRANSITION_STATE_ACTIVATING"
        - Receives back same from this machine node
            If it does not receive same back from machine node, it loops until it does
        - Sets to PRIMARY_STATE_ACTIVE
        - Receives same from machine node
        To disconnect, same process is done, however sending TRANSITION_STATE_DEACTIVATING and then PRIMARY_STATE_INACTIVE
        """
        uuid = tuple(msg.id.uuid.tolist())
        recv_uuid = tuple(msg.receiver_id.uuid.tolist())

        if uuid == tuple(self.machine_UUID):
            self.get_logger().debug('Ignoring self-generated message')
        elif (recv_uuid == tuple(self.machine_UUID)) and (msg.VERSION == 0):  # Not self generated and intended for this machine node
            self.get_logger().debug(f'❗ Got message from: {uuid}')

            # Add Pstop publisher to array of publishers, if auto accepting is True
            if (self.pstop_auto_accept): # and (msg.state.id != State.TRANSITION_STATE_DEACTIVATING):
                if uuid not in self.pstop_data: #this is a new pstop
                    self.pstop_data.setdefault(uuid, {"last_timestamp": 0, "counter": 0, "state": 0, "last_state":0, "message": 0})        
                    self.set_parameters([
                        Parameter(
                            name='pstops_connected',
                            value=[str(key) for key in self.pstop_data.keys()]
                        )
                    ])
                
                self.pstop_data[uuid].update({
                    "received_time" : self.get_clock().now(),
                    "last_timestamp": msg.stamp,
                    "counter": self.pstop_data[uuid]["counter"] + 1,
                    "last_state": self.pstop_data[uuid]["state"],
                    "state": msg.state.id,
                    "message": msg.message
                })
            

            if uuid in list(self.pstop_data.keys()):# Check that pstop message is in our allow list of pstops
                self.get_logger().debug(f"This is the droid {uuid} we are looking for")
                
                self.reply.state = State(id=State.PRIMARY_STATE_UNCONFIGURED, label="unconfigured")
                
                self.reply.stamp = self.get_clock().now().to_msg()
                self.reply.counter = self.pstop_data[uuid]["counter"]
                self.reply.receiver_id.uuid = uuid
                self.reply.heartbeat_timeout.sec = int(self.heartbeat_timeout)
                self.reply.heartbeat_timeout.nanosec = int((self.heartbeat_timeout - self.reply.heartbeat_timeout.sec) * 1e9)
                self.reply.received_stamp = msg.stamp
                self.reply.received_counter = msg.counter
                
                if self.pstop_data[uuid]["state"] == State.TRANSITION_STATE_ACTIVATING:
                    self.reply.state = State(id=State.TRANSITION_STATE_ACTIVATING, label="activating")
                    
                # The pstop has followed process, we are now successfully bonded
                if (self.pstop_data[uuid]["state"] == State.PRIMARY_STATE_ACTIVE) and (self.pstop_data[uuid]["last_state"]== State.TRANSITION_STATE_ACTIVATING):
                    self.reply.state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
                    
                if self.pstop_data[uuid]["state"] == State.TRANSITION_STATE_DEACTIVATING:
                    self.reply.state = State(id=State.TRANSITION_STATE_DEACTIVATING, label="deactivating")
                    if uuid in self.pstop_data:
                        self.pstop_data.pop(uuid)
                        self.set_parameters([ #TODO: Don't update this repeatedly, check if the pstop has already been removed
                            Parameter(
                                name='pstops_connected',
                                value=[str(key) for key in self.pstop_data.keys()]
                            )
                        ])

                self.get_logger().info(f"State:\n{self.format_pstop_msg(self.reply)}")
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
        if a pstop is connected, and is in active mode, it must report in at least every heartbeat_timeout seconds
        """
        #self.get_logger().info(f"State:\n{self.format_pstop_msg(self.reply)}")
        
        for uuid, data in self.pstop_data.items():
            if self.get_clock().now() > (data["received_time"] + Duration(seconds=self.heartbeat_timeout)):
                self.get_logger().warn(f"missed update from {uuid}")

        val = Bool()
        val.data = bool(self.reply.message)
        self.publisher_status.publish(val)
        
        
        # CRC-16 Calculation Function
    def compute_crc16(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
        """
        Compute CRC-16-CCITT for the given data.

        Args:
            data (bytes): The input data.
            poly (int): Polynomial used for CRC calculation (default 0x1021).
            init (int): Initial value for the CRC (default 0xFFFF).

        Returns:
            int: Computed CRC-16 checksum.
        """
        crc = init
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFFFF  # Ensure CRC remains a 16-bit value
        return crc


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

