#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pstop_msg.msg import PStopMsg
from builtin_interfaces.msg import Time, Duration
from lifecycle_msgs.msg import State
from unique_identifier_msgs.msg import UUID

class MachineNode(Node):
    def __init__(self):
        super().__init__('pstop_machine_node')
        self.get_logger().info('Initializing pstop_machine_node')
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('machine_UUID',[0x00] * 16)
        self.publisher_ = self.create_publisher(PStopMsg, 'protective_stop_topic', 10)
        
        self.machine_UUID = list(map(lambda x: x & 0xFF, self.get_parameter('machine_UUID').get_parameter_value().integer_array_value))
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        self.timer = self.create_timer(self.heartbeat_timeout/3.0, self.timer_callback) #run the loop 3x faster than timeout
        self.get_logger().info('pstop_machine_node node has been started')

    def timer_callback(self):
    
        # Checks for updated rosparams on every publication, in case we need to dynamically change UUID or timeout for whatever reason
        self.machine_UUID = list(map(lambda x: x & 0xFF, self.get_parameter('machine_UUID').get_parameter_value().integer_array_value))
        if self.heartbeat_timeout != self.get_parameter('heartbeat_timeout').get_parameter_value().double_value:
           self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value
           # Cancel the old timer and create a new one
           self.timer.cancel()
           self.timer = self.create_timer(self.heartbeat_timeout / 3.0, self.timer_callback)
           self.get_logger().info(f'Timer rate updated to: {self.heartbeat_timeout / 3.0} seconds')
    
        msg = PStopMsg()
        msg.message = PStopMsg.STOP  # Set message to STOP (1)
        msg.state = State(id=3)  # Example state value, set appropriately
        msg.stamp = Time()  # Set timestamp, add appropriate values
        msg.id = UUID(uuid=self.machine_UUID)
        msg.receiver_id = UUID(uuid=[1]*16)  # Set receiver ID, modify as needed
        msg.counter = 1  # Set counter value
        msg.heartbeat_timeout = Duration(sec=int(self.heartbeat_timeout))
        msg.checksum_type = "CRC-16"
        msg.checksum_value = "abcd1234"
        msg.received_stamp = Time()  # Set appropriate timestamp
        msg.received_counter = 0  # Example counter value
        
        self.get_logger().debug('Creating message: message=%s, state=%s' % (msg.message, msg.state.id))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    machine_node = MachineNode()
    try:
        rclpy.spin(machine_node)
    except KeyboardInterrupt:
        machine_node.get_logger().info('KeyboardInterrupt received, shutting down node')
    except Exception as e:
        machine_node.get_logger().error('An error occurred: %s' % str(e))
    finally:
        machine_node.destroy_node()
        rclpy.shutdown()
        machine_node.get_logger().info('pstop_machine_node has been shut down')

if __name__ == '__main__':
    main()

