#!/usr/bin/env python3
import rclpy
from protective_stop_msg.msg import (
    ProtectiveStop,
    ProtectiveStopParams,
    ProtectiveStopHeartbeat,
    ProtectiveStopDebug,
)
from protective_stop_msg.srv import (
    ProtectiveStop as ProtectiveStopSrv,
)
from termcolor import colored
import threading

from protective_stop_node.models import (
    PStopModel,
    PStopRemoteStatusEnum,
    Ros2Time,
    ConnectionStatus,
)

from rclpy.duration import Duration
import traceback
from functools import partial


from rclpy.lifecycle import State, TransitionCallbackReturn, LifecycleNode

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


PROTECTIVE_STOP_TOPIC = "/protective_stop"
PROTECTIVE_STOP_HB_TOPIC = "/pstop_hb"
PROTECTIVE_STOP_DEBUG_TOPIC = "/protective_stop/debug"


class ProtectiveStopNode(LifecycleNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().debug(f"Initializing {node_name}")

        # Declaring ROS2 params
        self.declare_parameter(
            "machine_uuid", "", ParameterDescriptor(description="Machine uuid of robot")
        )
        self.declare_parameter(
            "heartbeat_timeout",
            1.0,
            ParameterDescriptor(
                description="timeout float value in seconds before pstop is triggered"
            ),
        )

        self.declare_parameter(
            "deactivation_timeout",
            5.0,
            ParameterDescriptor(
                description="Wait time in seconds before deactivating a remote pstop and deleting it from internal register"
            ),
        )

        self.declare_parameter(
            "max_pstop_count",
            10,
            ParameterDescriptor(
                description="Maximum number of pstops that can be registered to the protective stop node"
            ),
        )

        self.declare_parameter(
            "is_user_monitored",
            True,
            ParameterDescriptor(
                description="If True, the node will monitor connected pstops and enforce their state. If False, it will not enforce that a pstop remote is connected to the node in order for it to send healthy heartbeats."
            ),
        )

        # Dictionary to store all connected PStops
        self.connected_remote_pstop_state = {}
        self.connected_remote_pstop_state_lock = threading.Lock()

        """
        Initialize the node's state and parameters
        """
        self.hb_timer = None
        self.debug_timer = None
        self.publisher_pstop = None
        self.machine_uuid = None
        self.publisher_hb = None
        self.param_event_handler = None
        self.is_user_monitored = None


        self.activate_service = self.create_service(
            ProtectiveStopSrv,
            f"{self.get_name()}/activate",
            partial(self.state_transition_callback, True),
        )
        self.deactivate_service = self.create_service(
            ProtectiveStopSrv,
            f"{self.get_name()}/deactivate",
            partial(self.state_transition_callback, False),
        )

        self.add_on_set_parameters_callback(self.parameters_callback)


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring node '{self.get_name()}'  in state '{state.label}'.")

        self.heartbeat_timeout = (
            self.get_parameter("heartbeat_timeout").get_parameter_value().double_value
        )
        self.deactivation_timeout = (
            self.get_parameter("deactivation_timeout").get_parameter_value().double_value
        )

        self.machine_uuid = self.get_parameter("machine_uuid").get_parameter_value().string_value

        self.max_pstop_count = (
            self.get_parameter("max_pstop_count").get_parameter_value().integer_value
        )

        self.is_user_monitored = (
            self.get_parameter("is_user_monitored").get_parameter_value().bool_value
        )
        self.set_is_user_monitored(self.is_user_monitored)

        self.get_logger().info(
            colored(
                f"""
Node configured successfully with:
    - machine_uuid: '{self.machine_uuid}'.
    - heartbeat timeout: {self.heartbeat_timeout}
    - max pstop count: {self.max_pstop_count}
    - deactivation timeout: {self.deactivation_timeout}
    - user monitor mode: {self.is_user_monitored}
    """,
                "cyan",
            )
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating node '{self.get_name()}' in state '{state.label}'.")

        self.subscriber_pstop = self.create_subscription(
            ProtectiveStop, PROTECTIVE_STOP_TOPIC, self.pstop_callback, 1
        )
        self.publisher_pstop = self.create_publisher(ProtectiveStop, PROTECTIVE_STOP_TOPIC, 1)
        self.publisher_pstop_debug = self.create_publisher(
            ProtectiveStopDebug, PROTECTIVE_STOP_DEBUG_TOPIC, 1
        )
        self.publisher_hb = self.create_publisher(
            ProtectiveStopHeartbeat, PROTECTIVE_STOP_HB_TOPIC, 1
        )

        self.hb_timer = self.create_timer(self.heartbeat_timeout, self.process_heartbeat)
        self.debug_timer = self.create_timer(1.0, self._publish_debug_msg)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            colored(f"Deactivating node '{self.get_name()}' in state '{state.label}'.", "cyan")
        )
        if self.hb_timer is not None:
            self.hb_timer.cancel()
        if self.debug_timer is not None:
            self.debug_timer.cancel()
        if self.subscriber_pstop:
            self.destroy_subscription(self.subscriber_pstop)
            self.subscriber_pstop = None
        if self.publisher_pstop:
            self.destroy_publisher(self.publisher_pstop)
            self.publisher_pstop = None
        if self.publisher_hb:
            self.destroy_publisher(self.publisher_hb)
            self.publisher_hb = None
        if self.publisher_pstop_debug:
            self.destroy_publisher(self.publisher_pstop_debug)
            self
        self.get_logger().info("Node deactivated successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down node '{self.get_name()}' in state '{state.label}'.")
        return TransitionCallbackReturn.SUCCESS

    def parameters_callback(self, params):
        """
        Callback for parameter updates
        """
        for param in params:
            if param.name == "is_user_monitored":
                self.set_is_user_monitored(param.value)
        return SetParametersResult(
            successful=True,
            reason="Parameter update processed successfully."
        )

    def set_is_user_monitored(self, is_user_monitored: bool):
        """
        USE WITH CAUTION

        Sets the protective_stop_node in a mode where it will not enforce
        connected protective stop remotes in order to be considered healthy. Any
        connected pstop remotes will continue to function, but if they fall out
        of connection with the protective_stop_node, the protective_stop_node
        will send healthy signals.
        """
        self.is_user_monitored = is_user_monitored

        warning_str = 'User Monitor Mode DISABLED. If you lose connection to your protective stop on machine, it will continue to produce a heartbeat.' if not self.is_user_monitored else 'User Monitor Mode ENABLED.'
        self.get_logger().warn(
            colored(
                f"Received request to set monitoring mode. Current state: {warning_str}",
                "yellow",
            )
        )

    def state_transition_callback(self, activate: bool, request, response):
        """
        Callback for the state transition service
        """
        uuid = request.sender_uuid
        recv_uuid = request.recv_uuid
        state = "ACTIVE" if activate else "DEACTIVATED"
        self.get_logger().info(
            colored(
                f"Received state transition request from {uuid} to {recv_uuid} to state '{state}'",
                "cyan",
            )
        )

        params = ProtectiveStopParams()
        params.heartbeat_timeout = self.heartbeat_timeout
        params.machine_uuid = self.machine_uuid
        response.params = params

        if uuid == self.machine_uuid:
            response.success = False
            response.message = "Cannot send a state transition to self"
            return response
        elif recv_uuid != self.machine_uuid:
            response.success = False
            response.message = "Unknown receiver uuid"
            return response
        # TODO(troy): Update version here to constant
        elif request.version != ProtectiveStop.VERSION:
            response.success = False
            response.message = "Invalid version"
            return response

        if activate:
            while len(self.connected_remote_pstop_state) >= self.max_pstop_count:
                oldest_pstop = sorted(
                    self._safe_remote_iterator(),
                    key=lambda x: x[1].init_timestamp.to_ros_time(self.get_clock()),
                )
                self.get_logger().warn(
                    colored(
                        f"Max pstop count of {self.max_pstop_count} reached. Pruning oldest pstop: {oldest_pstop[0][0]}",
                        "yellow",
                    ),
                    throttle_duration_sec=1.0,
                )

                with self.connected_remote_pstop_state_lock:
                    del self.connected_remote_pstop_state[oldest_pstop[0][0]]

            if (
                uuid in self.connected_remote_pstop_state
                and self.connected_remote_pstop_state[uuid].connection_status.status
                != PStopRemoteStatusEnum.DEACTIVATED
            ):
                self.get_logger().warn(
                    colored(
                        f"Received activation request from {uuid} while already activated",
                        "yellow",
                    ),
                    throttle_duration_sec=1.0,
                )
                response.success = False
                response.message = f"Already activated. Current state of remote is {self.connected_remote_pstop_state[uuid].connection_status.status.name}"
                return response

            pstop_state = PStopModel(
                connection_status=ConnectionStatus(
                    status=PStopRemoteStatusEnum.ACTIVE, message="Activated by remote"
                ),
                init_timestamp=Ros2Time.from_ros_time(self.get_clock().now()),
                receive_timestamp=Ros2Time.from_ros_time(self.get_clock().now()),
                remote_timestamp=None,
            )
            self.connected_remote_pstop_state[uuid] = pstop_state

            response.success = True
            response.message = "Successfully activated"
            return response
        else:
            if uuid in self.connected_remote_pstop_state:
                self.connected_remote_pstop_state[uuid].connection_status = ConnectionStatus(
                    status=PStopRemoteStatusEnum.DEACTIVATED, message="Deactivated by remote"
                )
            else:
                response.success = False
                response.message = "Deactivation failed. UUID not in connected list"
                return response

            response.success = True
            response.message = "Successfully deactivated"
            return response

    def _get_status_string(self, connection_status: ConnectionStatus):
        return connection_status.status.name

    def pstop_callback(self, msg):
        uuid = msg.sender_uuid
        recv_uuid = msg.receiver_uuid

        if uuid == self.machine_uuid:
            return
        elif (
            (recv_uuid == self.machine_uuid)
            and (msg.version == 0)
            and (uuid in [uuid for uuid, _ in self._safe_remote_iterator()])
        ):
            if (
                self.connected_remote_pstop_state[uuid].connection_status.status
                == PStopRemoteStatusEnum.DEACTIVATED
            ):
                self.get_logger().warn(
                    f"Received message from {uuid} while not in active state",
                    throttle_duration_sec=1.0,
                )
                return
            self.connected_remote_pstop_state[uuid].pstop_pressed = msg.pstop_pressed
            self.connected_remote_pstop_state[uuid].receive_timestamp = Ros2Time.from_ros_time(
                self.get_clock().now()
            )
            self.connected_remote_pstop_state[uuid].remote_timestamp = Ros2Time.from_time_msg(
                msg.stamp
            )
            self.connected_remote_pstop_state[uuid].counter = (
                self.connected_remote_pstop_state[uuid].counter + 1
            )

            reply_msg = ProtectiveStop()
            reply_msg.version = 0
            reply_msg.pstop_pressed = self.connected_remote_pstop_state[uuid].pstop_pressed
            reply_msg.connection_status = self.connected_remote_pstop_state[
                uuid
            ].connection_status.to_ros_message()
            reply_msg.sender_uuid = self.machine_uuid
            reply_msg.receiver_uuid = uuid
            reply_msg.counter = self.connected_remote_pstop_state[uuid].counter
            reply_msg.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(
                f"Sending reply to {uuid}. P-Stop Pressed: {reply_msg.pstop_pressed}. Counter: {reply_msg.counter}",
                throttle_duration_sec=1.0,
            )
            self.publisher_pstop.publish(reply_msg)
        else:
            self.get_logger().warn(
                f"Received message from unknown source: {uuid}", throttle_duration_sec=1.0
            )

    def _has_exceeded_heartbeat_timeout(self, receive_timestamp: Ros2Time):
        timeout_adjusted_time = receive_timestamp.to_ros_time(self.get_clock()) + Duration(
            seconds=self.heartbeat_timeout
        )
        now = self.get_clock().now()
        has_exceeded = now > timeout_adjusted_time
        if has_exceeded:
            diff = now - timeout_adjusted_time
            self.get_logger().warn(
                f"Exceeded heartbeat of {self.heartbeat_timeout}s. Diff: {diff.nanoseconds / 1e9}s",
                throttle_duration_sec=1.0,
            )
        return has_exceeded

    def _has_exceeded_deactivation_timeout(self, receive_timestamp: Ros2Time):
        timeout_adjusted_time = receive_timestamp.to_ros_time(self.get_clock()) + Duration(
            seconds=self.deactivation_timeout
        )
        now = self.get_clock().now()
        has_exceeded = now > timeout_adjusted_time
        if has_exceeded:
            diff = now - timeout_adjusted_time
            self.get_logger().warn(
                f"Exceeded deactivation timeout of {self.deactivation_timeout}s. Diff: {diff.nanoseconds / 1e9}s",
                throttle_duration_sec=1.0,
            )
        return has_exceeded

    def _safe_remote_iterator(self):
        """
        Safely iterate over all connected remote pstops
        """
        with self.connected_remote_pstop_state_lock:
            for uuid, pstop_remote in self.connected_remote_pstop_state.items():
                yield uuid, pstop_remote

    def process_heartbeat(self):
        """
        Handles loss of comms or timeouts
        if a pstop is connected, and is in active mode, it must report in at least every heartbeat_timeout seconds
        """
        stop = None if self.is_user_monitored else False

        # Find all non-deactivated pstops
        if not len(
            [
                pstop
                for _, pstop in self._safe_remote_iterator()
                if pstop.connection_status.status != PStopRemoteStatusEnum.DEACTIVATED
            ]
        ):
            self.get_logger().info("No connected remote pstops...", throttle_duration_sec=1.0)

        for uuid, pstop_remote in self._safe_remote_iterator():
            if pstop_remote.connection_status.status == PStopRemoteStatusEnum.DEACTIVATED:
                continue
            self.get_logger().debug(
                f"Checking heartbeat for {uuid}. Status: {self._get_status_string(pstop_remote.connection_status)}. P-Stop Pressed: {pstop_remote.pstop_pressed}",
                throttle_duration_sec=1.0,
            )
            if self._has_exceeded_deactivation_timeout(pstop_remote.receive_timestamp):
                self.get_logger().info(
                    colored(
                        f"Deactivating remote '{uuid}'",
                        "cyan",
                    ),
                    throttle_duration_sec=1.0,
                )
                self.connected_remote_pstop_state[uuid].connection_status = ConnectionStatus(
                    status=PStopRemoteStatusEnum.DEACTIVATED, message="Deactivated due to timeout"
                )

                continue
            elif self._has_exceeded_heartbeat_timeout(pstop_remote.receive_timestamp):
                stop = True
                self.get_logger().warn(
                    colored(
                        f"Missed heartbeat from remote '{uuid}'. Status: {PStopRemoteStatusEnum.UNSTABLE.name}...",
                        "yellow",
                    ),
                    throttle_duration_sec=1.0,
                )
                self.connected_remote_pstop_state[uuid].connection_status = ConnectionStatus(
                    status=PStopRemoteStatusEnum.UNSTABLE, message="Missed heartbeat"
                )
                continue
            elif (
                self.connected_remote_pstop_state[uuid].connection_status.status
                == PStopRemoteStatusEnum.UNSTABLE
            ):
                self.get_logger().info(
                    colored(
                        f"Recovered from unstable state for remote '{uuid}'",
                        "cyan",
                    ),
                )

            self.connected_remote_pstop_state[uuid].connection_status = ConnectionStatus(
                status=PStopRemoteStatusEnum.ACTIVE, message=""
            )
            stop = self.connected_remote_pstop_state[uuid].pstop_pressed

        hb_msg = ProtectiveStopHeartbeat()
        hb_msg.stop = stop if stop is not None else True
        hb_msg.stamp = self.get_clock().now().to_msg()

        self.get_logger().debug(
            f"Sending heartbeat. {colored(f'Stop: {hb_msg.stop}', 'cyan')}",
        )

        self.publisher_hb.publish(hb_msg)

    def _publish_debug_msg(self):
        debug_msg = ProtectiveStopDebug()
        debug_msg.timestamp = self.get_clock().now().to_msg()

        for uuid, pstop_remote in self._safe_remote_iterator():
            remote_pstop_msg = pstop_remote.to_ros_message(uuid)
            debug_msg.connected_remote_pstops.append(remote_pstop_msg)

        self.publisher_pstop_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    protective_stop_node = ProtectiveStopNode("protective_stop_node")
    try:
        rclpy.spin(protective_stop_node)
    except KeyboardInterrupt:
        protective_stop_node.get_logger().info("KeyboardInterrupt received, shutting down node")
    except SystemExit:
        protective_stop_node.get_logger().info("SystemExit received, shutting down node")
    except Exception as e:
        protective_stop_node.get_logger().error("An error occurred: %s" % str(e))
        protective_stop_node.get_logger().error(traceback.format_exc())
    finally:
        protective_stop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
