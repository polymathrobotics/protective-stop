import roslibpy
import yaml
import argparse
import time

# Define the PStopMsg message structure as a dictionary based on the .msg file
# Note: This assumes `roslibpy.Message` can be used in this format, otherwise, adjustments will be needed.
PSTOP_MESSAGE_VERSION = 0
PSTOP_OK = 0
PSTOP_STOP = 1

# Create a function that builds the PStopMsg dictionary

def build_pstop_message(message_id, state, timestamp, id, receiver_id, counter, heartbeat_timeout, checksum_type, checksum_value, received_stamp, received_counter):
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
        checksum_type (str): Checksum type (e.g., CRC-16, SHA-256, MD5, etc.).
        checksum_value (str): Hexadecimal string of the checksum value.
        received_stamp (dict): Timestamp of the last received message (following builtin_interfaces/Time structure).
        received_counter (int): Counter for the last received message.
    
    Returns:
        dict: PStopMsg dictionary.
    """
    return {
        'message': message_id,
        'state': state,
        'stamp': timestamp,
        'id': id,
        'receiver_id': receiver_id,
        'counter': counter,
        'heartbeat_timeout': heartbeat_timeout,
        'checksum_type': checksum_type,
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
    publisher = roslibpy.Topic(client, '/protective_stop', 'pstop_msg/msg/PStopMsg',queue_size=1,queue_length=1)
    
    client.run()
    
    try:
        publisher.advertise()
        counter = 0
        while client.is_connected:
            # Example data for the PStopMsg
            message = build_pstop_message(
                message_id=PSTOP_OK,
                state={
                    'id': 1,
                    'label': 'active'
                },
                timestamp={
                    'sec': int(time.time()),
                    'nanosec': int((time.time() - int(time.time())) * 1e9)
                },
                id={
                    'uuid': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
                },
                receiver_id={
                    'uuid': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                },
                counter=counter,
                heartbeat_timeout={ # Not used by the PSTOP, only Machine Node
                    'sec': 0,
                    'nanosec': 0
                },
                checksum_type='CRC-16',
                checksum_value='ABCD1234',
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

