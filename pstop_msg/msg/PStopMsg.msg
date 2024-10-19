# PSTOP message type, used to safely control local and remote robots

# We need to have access control, be able to configure the expected behavior, report state
# and be able to have many to many interactions



builtin_interfaces/msg/Time sender_stamp
builtin_interfaces/msg/Time receiver_stamp
builtin_interfaces/msg/Duration heartbeat_timeout

# Used to notify if the PStop wants to become active, or not
Desired connection state

int64 checksum
string sender_timestamp
string sender_id
uint32 rolling_counter
float32 expected_heartbeat_rate
uint32 expected_heartbeat_timeout
string state
string target
