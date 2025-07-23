#!/usr/bin/env python3
import time
from builtin_interfaces.msg import Time
from protective_stop_msg.msg import ProtectiveStop


PSTOP_MESSAGE_VERSION = 0


def build_pstop_message(receiver_uuid, sender_uuid, pstop_pressed, counter=0):
    """
    Build a ProtectiveStop message for testing purposes.
    
    Args:
        receiver_uuid: UUID of the receiver
        sender_uuid: UUID of the sender
        pstop_pressed: Boolean indicating if pstop is pressed
        counter: Message counter (default: 0)
    
    Returns:
        ProtectiveStop message
    """
    timestamp = Time(sec=int(time.time()), nanosec=int((time.time() % 1) * 1e9))

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