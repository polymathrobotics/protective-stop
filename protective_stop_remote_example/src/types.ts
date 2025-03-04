// Manually define ROS 2 types
export interface Time {
  sec: number;
  nanosec: number;
}
export interface State {
  id: number;
  label: string;
}

export interface Duration {
  sec: number;
  nanosec: number;
}

export enum ConnectionStatus {
  ACTIVE = 0,
  DEACTIVATED = 1,
  UNSTABLE = 2,
}

export interface PStopMessage {
  /** PSTOP message version. Expected to be the constant value 0. */
  version: number;

  /** The state of the PSTOP (STOP or OK). */
  pstop_pressed: boolean;

  connection_status: {
    status: ConnectionStatus;
    message: string;
  };

  /** Local timestamp of the sender. */
  stamp: Time;

  /** Unique ID for the machine or PSTOP sending this message. */
  sender_uuid: string;

  /** Unique ID for the machine or PSTOP this message is sent to. */
  receiver_uuid: string;

  /**
   * Counter increments by 1 for every sent message.
   * This is a uint64 value; note that JavaScript’s number type can only safely
   * represent integers up to 2^53 - 1. For larger values consider using a BigInt.
   */
  counter: bigint;
}

export interface PStopServiceRequest {
  /** Version of the message (uint8). */
  version: number;

  /** UUID of the receiver. */
  recv_uuid: string;

  /** UUID of the sender. */
  sender_uuid: string;
}

export interface PStopServiceResponse {
  success: boolean;
  message: string;
  params: ProtectiveStopParams;
}

/**
 * Parameters for the protective stop.
 */
export interface ProtectiveStopParams {
  /** Heartbeat timeout value (float32). */
  heartbeat_timeout: number;

  /** Unique identifier of the machine (UUID). */
  machine_uuid: string;

  /** If true, protective stop auto-accepts the stop command. */
  pstop_auto_accept: boolean;
}

/**
 * Response message for the Protective Stop service.
 */
export interface ProtectiveStopResponse {
  /** Indicates if the request was successful. */
  success: boolean;

  /** Additional message or information. */
  message: string;

  /** Parameters related to the protective stop. */
  params: ProtectiveStopParams;
}

export enum ProtectiveStopUIState {
  DISCONNECTED = "DISCONNECTED",
  CONNECTING = "CONNECTING",
  UNSTABLE_ROBOT_CONNECTION = "UNSTABLE_ROBOT_CONNECTION",
  UNSTABLE_CLIENT_CONNECTION = "UNSTABLE_CLIENT_CONNECTION",
  FAILED_CONNECTION = "FAILED_CONNECTION",
  ACTIVE_PRESSED = "ACTIVE_PRESSED",
  ACTIVE_UNPRESSED = "ACTIVE_UNPRESSED",
}
