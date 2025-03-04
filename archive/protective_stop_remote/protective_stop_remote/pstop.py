import time
import logging
import roslibpy
import threading
import uuid
import base64


# TODO:
# - Handle connect/disconnect/connect more reliably
# - Generally handle multiple clients better
# - LOTS of testing, especially with multiple clients (not checked!)

# Define the PStopMsg message structure as a dictionary based on the .msg file
# Note: This assumes `roslibpy.Message` can be used in this format, otherwise, adjustments will be needed.
PSTOP_MESSAGE_VERSION = 0
PSTOP_STOP = 0
PSTOP_OK = 1
rosbridge_port = 9190

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


def build_pstop_message(
    message_id, state_id, timestamp, sender_uuid, receiver_uuid,
    counter, heartbeat_timeout, received_stamp, received_counter
):
    """
    Constructs a PStopMsg dictionary using the correct lifecycle state definitions.

    Args:
        message_id (int): OK (1) or STOP (0)
        state_id (int): Lifecycle state (e.g., 13 for ACTIVATING)
        timestamp (dict): Current timestamp {'sec': int, 'nanosec': int}
        sender_uuid (dict): Sender UUID {'uuid': str}
        receiver_uuid (dict): Receiver UUID {'uuid': str}
        counter (int): Message counter
        heartbeat_timeout (dict): Heartbeat timeout {'sec': int, 'nanosec': int}
        received_stamp (dict): Timestamp of the last received message
        received_counter (int): Counter for the last received message

    Returns:
        dict: Formatted PStopMsg dictionary
    """

    # Lifecycle state definitions, from https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/State.html
    lifecycle_states = {
        0: "UNKNOWN",
        1: "UNCONFIGURED",
        2: "INACTIVE",
        3: "ACTIVE",
        4: "FINALIZED",
        10: "CONFIGURING",
        11: "CLEANINGUP",
        12: "SHUTTINGDOWN",
        13: "ACTIVATING",
        14: "DEACTIVATING",
        15: "ERRORPROCESSING"
    }

    # Function to get state label dynamically
    def get_state_label(state_id):
        return lifecycle_states.get(state_id, "UNKNOWN")

    return {
        'message': message_id,
        'state': {'id': int(state_id), 'label': get_state_label(int(state_id))},
        'stamp': timestamp,
        'id': sender_uuid,
        'receiver_uuid': receiver_uuid,
        'counter': counter,
        'heartbeat_timeout': heartbeat_timeout,
        'checksum_type': 'CRC-16',
        'checksum_value': f"{compute_crc16(str(counter).encode()):04X}",
        'received_stamp': received_stamp,
        'received_counter': received_counter
    }


def start_client(ip, port, shared):
    client = roslibpy.Ros(host=ip, port=port)
    attempt_count = 0
    max_attempts = 3

    while not client.is_connected and attempt_count < max_attempts:
        try:
            logging.info(
                f"Attempting to connect to {ip}:{port} (Attempt {attempt_count + 1}/{max_attempts})")
            client.run()
            client.connect()
            time.sleep(0.5)
            if client.is_connected:
                logging.info(f"Connected to {ip}:{port}")
                machine = next(
                    (m for m in shared["target_pairs"] if m.get("ip") == ip), None)
                if machine:
                    machine['status'] = "Connected"
            else:
                logging.error(f"Connection to {ip}:{port} failed")
                client.close()
        except Exception as e:
            logging.error(f"Connection to {ip}:{port} failed: {e}")
            attempt_count += 1
            machine = next(
                (m for m in shared["target_pairs"] if m.get("ip") == ip), None)
            if machine:
                machine['status'] = f"Retrying ({attempt_count}/{max_attempts})"
            if attempt_count < max_attempts:
                logging.info("Retrying in 5 seconds...")
                time.sleep(5)

    if not client.is_connected:
        logging.error(
            f"Failed to connect to {ip}:{port} after {max_attempts} attempts.")
        machine = next(
            (m for m in shared["target_pairs"] if m.get("ip") == ip), None)
        if machine:
            machine['status'] = "Failed"

    return client


def setup_client(client, ip, port, shared):
    publisher = roslibpy.Topic(
        client, '/protective_stop', 'pstop_msg/msg/PStopMsg')
    publisher.advertise()
    subscriber = roslibpy.Topic(
        client, '/protective_stop', 'pstop_msg/msg/PStopMsg')

    bonding_status = {}  # Track bonding state per machine UUID

    try:
        self_uuid = uuid.UUID(shared.get("uuid"))
    except ValueError:
        logging.error(f"Invalid self UUID: {shared.get('uuid')}")
        return

    def callback(message):
        sender_uuid_raw = message.get("id", {}).get("uuid")
        receiver_uuid_raw = message.get("receiver_uuid", {}).get("uuid")

        sender_uuid = None
        receiver_uuid = None

        logging.info(f"⚠️  Received message: {message}")

        # ✅ Ensure sender UUID is extracted correctly
        if isinstance(sender_uuid_raw, list) and len(sender_uuid_raw) == 16:
            sender_uuid = uuid.UUID(bytes=bytes(sender_uuid_raw))
        elif isinstance(sender_uuid_raw, str):
            try:
                sender_uuid = uuid.UUID(
                    bytes=base64.b64decode(sender_uuid_raw))
            except Exception:
                logging.warning(
                    f"⚠️  Failed to decode sender UUID from Base64: {sender_uuid_raw}")

        # ✅ Ensure receiver UUID is extracted correctly
        if isinstance(receiver_uuid_raw, list) and len(receiver_uuid_raw) == 16:
            receiver_uuid = uuid.UUID(bytes=bytes(receiver_uuid_raw))
        elif isinstance(receiver_uuid_raw, str):
            try:
                receiver_uuid = uuid.UUID(
                    bytes=base64.b64decode(receiver_uuid_raw))
            except Exception:
                logging.warning(
                    f"⚠️  Failed to decode receiver UUID from Base64: {receiver_uuid_raw}")

        # ✅ Convert self UUID for consistent comparison
        try:
            self_uuid = uuid.UUID(shared.get("uuid"))
        except ValueError:
            logging.error(f"❌ Invalid self UUID: {shared.get('uuid')}")
            return

        logging.info(
            f"Sender UUID: {sender_uuid}, Receiver UUID: {receiver_uuid}, Self UUID: {self_uuid}")

        if sender_uuid and sender_uuid == self_uuid:
            logging.info(
                f"❌ Ignoring self-generated message from UUID {self_uuid}")
            return

        sender_str = str(sender_uuid)
        state_id = message["state"]["id"]

        if state_id == 13:  # TRANSITION_STATE_ACTIVATING
            logging.info(f"🔄 Bonding started with {sender_uuid}")
            bonding_status[sender_str] = "ACTIVATING"

            response = build_pstop_message(
                PSTOP_OK, 13, message["stamp"], message["id"], message["receiver_uuid"],
                message["counter"], message["heartbeat_timeout"],
                message["received_stamp"], message["received_counter"]
            )
            publisher.publish(roslibpy.Message(response))

        elif state_id == 3:  # PRIMARY_STATE_ACTIVE
            if bonding_status.get(sender_str) == "ACTIVATING":
                logging.info(f"✅ Bonded successfully with {sender_uuid}")
                bonding_status[sender_str] = "ACTIVE"

        elif state_id == 14:  # TRANSITION_STATE_DEACTIVATING
            logging.info(f"❌ Unbonding from {sender_uuid}")
            bonding_status[sender_str] = "DEACTIVATING"

            response = build_pstop_message(
                PSTOP_OK, 14, message["stamp"], message["id"], message["receiver_uuid"],
                message["counter"], message["heartbeat_timeout"],
                message["received_stamp"], message["received_counter"]
            )
            publisher.publish(roslibpy.Message(response))

        elif state_id == 2:  # PRIMARY_STATE_INACTIVE
            if bonding_status.get(sender_str) == "DEACTIVATING":
                logging.info(f"✅ Successfully unbonded from {sender_uuid}")
                bonding_status.pop(sender_str, None)

    subscriber.subscribe(callback)
    logging.info(f"Publisher and subscriber set up for {ip}:{port}")

    def bonding_thread():
        while True:
            for target in shared.get("target_pairs", []):
                target_uuid = target.get("uuid")
                if target_uuid and target["ip"] == ip and target_uuid not in bonding_status:
                    logging.info(f"🔄 Initiating bonding with {target_uuid}")

                    bonding_msg = build_pstop_message(
                        PSTOP_OK, 13,  # ✅ Uses TRANSITION_STATE_ACTIVATING from build_pstop_message()
                        {"sec": int(time.time()), "nanosec": int(
                            (time.time() % 1) * 1e9)},
                        {"uuid": str(self_uuid)},
                        {"uuid": target_uuid},
                        0, {"sec": 1, "nanosec": 500000000},
                        {"sec": 0, "nanosec": 0}, 0
                    )

                    publisher.publish(roslibpy.Message(bonding_msg))
                    bonding_status[target_uuid] = "ACTIVATING"

            time.sleep(1)

    threading.Thread(target=bonding_thread, daemon=True).start()
    return publisher, subscriber


def client_thread(ip, rosbridge_port, shared, connected_clients):
    client = start_client(ip, rosbridge_port, shared)
    publisher, subscriber = setup_client(client, ip, rosbridge_port, shared)
    connected_clients[ip] = {
        "client": client,
        "publisher": publisher,
        "subscriber": subscriber,
        "thread": threading.current_thread()
    }

    counter = 0

    # ✅ Ensure receiver UUID is extracted correctly
    receiver_uuid_str = next(
        (m["uuid"] for m in shared["target_pairs"] if m["ip"] == ip), None)
    receiver_uuid = uuid.UUID(receiver_uuid_str.strip(
    )) if receiver_uuid_str else uuid.UUID(int=0)

    while ip in connected_clients:
        if not client.is_connected:
            logging.info(f"Client {ip} disconnected, attempting reconnection.")
            client = start_client(ip, rosbridge_port, shared)
            publisher, subscriber = setup_client(
                client, ip, rosbridge_port, shared)
            connected_clients[ip] = {
                "client": client,
                "publisher": publisher,
                "subscriber": subscriber,
                "thread": threading.current_thread()
            }

        # ✅ Convert self UUID properly
        self_uuid = uuid.UUID(shared["uuid"].strip())

        # ✅ Convert both UUIDs to `uint8[16]` lists before sending
        id = {"uuid": list(self_uuid.bytes)}
        receiver_uuid = {"uuid": list(receiver_uuid.bytes)}

        # ✅ Build and send a protective stop message
        timestamp = {'sec': int(time.time()), 'nanosec': int(
            (time.time() % 1) * 1e9)}
        state = {'id': 1, 'label': 'ACTIVE'}
        heartbeat_timeout = {'sec': 1, 'nanosec': 500000000}
        received_stamp = {
            'sec': timestamp['sec'], 'nanosec': timestamp['nanosec']}
        received_counter = counter
        button = PSTOP_STOP
        if shared.get("button_pressed") is False:
            button = PSTOP_OK  # ✅ Only set to OK if the button is NOT pressed

        message = build_pstop_message(
            button, state["id"], timestamp, id, receiver_uuid, counter,
            heartbeat_timeout, received_stamp, received_counter
        )
        publisher.publish(roslibpy.Message(message))
        counter += 1

        time.sleep(1)


def pstop_node(shared):
    logging.basicConfig(level=logging.DEBUG)
    logging.info(f"PStop Node started at {time.time()}")

    while "target_pairs" not in shared:
        time.sleep(0.1)

    connected_clients = {}

    try:
        while True:
            current_machines = {machine['ip'] for machine in shared.get(
                "target_pairs", {}) if 'ip' in machine}
            current_ips = set(connected_clients.keys())
            ips_to_add = current_machines - current_ips
            ips_to_remove = current_ips - current_machines

            # Add new connections
            for ip in ips_to_add:
                logging.info(f"Starting client thread for {ip}")
                thread = threading.Thread(target=client_thread, args=(
                    ip, rosbridge_port, shared, connected_clients))
                thread.daemon = True
                thread.start()

            # Remove old connections
            for ip in ips_to_remove:
                logging.info(f"Stopping client thread for {ip}")
                connection = connected_clients.pop(ip, None)
                if connection:
                    connection["publisher"].unadvertise()
                    connection["subscriber"].unsubscribe()
                    connection["client"].close()

            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("PStop Node exiting.")

    finally:
        for ip, connection in connected_clients.items():
            logging.info(f"Cleaning up connection for {ip}")
            connection["publisher"].unadvertise()
            connection["subscriber"].unsubscribe()
            connection["client"].terminate()


# Running standalone for testing, with some default values
if __name__ == '__main__':
    shared = {
        "target_pairs": [
            {
                "ip": "100.125.44.33",
                "uuid": "00000000-0000-0000-0000-000000000000",
                "status": "Not Connected"
            }
        ],
        "button_pressed": False,
        "uuid": "00000000-0000-0000-0000-000000000001",
    }
    pstop_node(shared)
