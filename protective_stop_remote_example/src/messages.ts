import { ConnectionStatus, PStopMessage } from "./types";

export const buildPstopMessage = (
  receiverId: string,
  senderId: string,
  pStopPressed: boolean
): PStopMessage => {
  return {
    version: 0,
    pstop_pressed: pStopPressed,
    connection_status: {
      status: ConnectionStatus.ACTIVE,
      message: "",
    },
    stamp: {
      sec: 0,
      nanosec: 0,
    },
    sender_uuid: senderId,
    receiver_uuid: receiverId,
    counter: BigInt(0),
  };
};
