# PSTOP message type, used to safely control local and remote robots

# We need to have access control, be able to configure the expected behavior, report state
# and be able to have many to many interactions

# The PSTOP message version
uint8 VERSION = 0

# PSTOP message constants
uint8 OK = 0
uint8 STOP = 1

# The PSTOP message id value from the above definitions.
uint8 self_message

# Used to notify if the PStop wants to become active, or not
lifecycle_msgs/State self_state

# Local timestamp of the robot or PSTOP sending this message
builtin_interfaces/Time self_stamp

# Unique ID for the robot or PSTOP
unique_identifier_msgs/UUID self_id

# Unique ID for the robot or PSTOP this message is sent to
unique_identifier_msgs/UUID receiver_id

# counter increments by 1 for every sent message, will not roll over in any realistic use case
uint64 self_counter

# robot expected heartbeat_timeout, defaults to 0 for PSTOP messages
builtin_interfaces/Duration heartbeat_timeout

# Checksum type (ex: CRC-16, SHA-256, MD5, etc)
string checksum_type

# Checksum value, should be a hexadecimal string
string checksum_value

# All items below are received from the other end of the bond (robot or PSTOP) and returned in the following message

# Timestamp of the latest received message, defaults to 0 on new bonding request
builtin_interfaces/Time receiver_stamp

# rolling counter of the latest received message, defaults to 0 on new bonding request
# counter increments by 1 for every sent message, will not roll over in any realistic use case
uint64 receiver_counter
