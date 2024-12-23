# PSTOP message type, used to safely control local and remote machines

# We need to have access control, be able to configure the expected behavior, report state
# and be able to have many to many interactions

# The PSTOP message version
uint8 VERSION = 0

# PSTOP message constants
uint8 STOP = 0
uint8 OK = 1

# The PSTOP message id value from the above definitions.
uint8 message

# Used to notify if the PStop wants to become active, or not
lifecycle_msgs/State state

# Local timestamp of the machine or PSTOP sending this message
builtin_interfaces/Time stamp

# Unique ID for the machine or PSTOP
unique_identifier_msgs/UUID id

# Unique ID for the machine or PSTOP this message is sent to
unique_identifier_msgs/UUID receiver_id

# counter increments bgy 1 for every sent message, will not roll over in any realistic use case
uint64 counter

# machine expected heartbeat_timeout, defaults to 0 for PSTOP messages
builtin_interfaces/Duration heartbeat_timeout

# Checksum type (ex: CRC-16, SHA-256, MD5, etc)
string checksum_type

# Checksum value, should be a hexadecimal string
string checksum_value

# All items below are received from the other end of the bond (machine or PSTOP) and returned in the following message

# Timestamp of the latest received message, defaults to 0 on new bonding request
builtin_interfaces/Time received_stamp

# rolling counter of the latest received message, defaults to 0 on new bonding request
# counter increments by 1 for every sent message, will not roll over in any realistic use case
uint64 received_counter
