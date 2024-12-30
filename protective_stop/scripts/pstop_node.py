import roslibpy
import yaml
import argparse
import time

# Define the PStopMsg message structure as a dictionary based on the .msg file
# Note: This assumes `roslibpy.Message` can be used in this format, otherwise, adjustments will be needed.
PSTOP_MESSAGE_VERSION = 0
PSTOP_STOP = 0
PSTOP_OK = 1

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

# Create a function that builds the PStopMsg dictionary
def build_pstop_message(message_id, state, timestamp, id, receiver_id, counter, heartbeat_timeout, received_stamp, received_counter):
    """
    Build the PStopMsg message to be sent to the ROS 2 bridge.

    Args:
        message_id (int): Value from OK or STOP.
        state (dict): State message (following lifecycle_msgs/State structure).
        timestamp (dict): Time message (following builtin_interfaces/Time structure).
        id (dict): UUID for the machine or PSTOP (following unique_identifier_msgs/UUID structure).
        receiver_id (dict): UUID of the receiver (following unique_identifier_msgs/UUID structure).
        counter (int): Counter for the message.
        heartbeat_timeout (dict): Duration message (following builtin_interfaces/Duration structure).
        received_stamp (dict): Timestamp of the last received message (following builtin_interfaces/Time structure).
        received_counter (int): Counter for the last received message.

    Returns:
        dict: PStopMsg dictionary.
    """
    # Build the payload to compute the checksum
    payload = (
        f"{message_id}{state['id']}{state['label']}"
        f"{timestamp['sec']}{timestamp['nanosec']}"
        f"{id['uuid']}{receiver_id['uuid']}"
        f"{counter}{heartbeat_timeout['sec']}{heartbeat_timeout['nanosec']}"
        f"{received_stamp['sec']}{received_stamp['nanosec']}{received_counter}"
    ).encode()

    # Compute the checksum
    checksum_value = f"{compute_crc16(payload):04X}"

    return {
        'message': message_id,
        'state': state,
        'stamp': timestamp,
        'id': id,
        'receiver_id': receiver_id,
        'counter': counter,
        'heartbeat_timeout': heartbeat_timeout,
        'checksum_type': 'CRC-16',
        'checksum_value': checksum_value,
        'received_stamp': received_stamp,
        'received_counter': received_counter
    }

# ROS 2 roslibpy node that publishes the PStopMsg to the rosbridge_server
def main():
    """
    Main function to create the ROS 2 roslibpy publisher node.
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS 2 PStopMsg Publisher')
    parser.add_argument('--config', type=str, required=True, help='Path to the YAML configuration file')
    args = parser.parse_args()

    # Load configuration from YAML file
    with open(args.config, 'r') as file:
        config = yaml.safe_load(file)

    host = config.get('host', 'localhost')
    port = config.get('port', 9190)

    # Create the ROS bridge client
    client = roslibpy.Ros(host=host, port=port)

    # Define the topic to which messages will be published
    publisher = roslibpy.Topic(client, '/protective_stop', 'pstop_msg/msg/PStopMsg')

    client.run()

    try:
        publisher.advertise()
        counter = 0
        while client.is_connected:
            # Example data for the PStopMsg
            message = build_pstop_message(
                message_id=PSTOP_OK,
                state={
                    'id': 14,
                    'label': 'activating'
                },
                timestamp={'sec': int(time.time()), 'nanosec': int((time.time() % 1) * 1e9)},
                id={
                    'uuid': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
                },
                receiver_id={
                    'uuid': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
                },
                counter=counter,
                heartbeat_timeout={
                    'sec': 1,
                    'nanosec': 0
                },
                received_stamp={
                    'sec': 1234567890,
                    'nanosec': 123456789
                },
                received_counter=counter - 1
            )

            # Publish the message to the topic
            publisher.publish(roslibpy.Message(message))

            print(f'Published PStopMsg: {message}')
            counter += 1

            # Wait for a few seconds before sending the next message
            time.sleep(1)

    except KeyboardInterrupt:
        print('Shutting down publisher...')

    finally:
        publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    main()

