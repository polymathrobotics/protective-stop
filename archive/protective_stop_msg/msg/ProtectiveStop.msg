# PSTOP message type, used to safely control local and remote machines

# We need to have access control, be able to configure the expected behavior, report state
# and be able to have many to many interactions

# The PSTOP message version
uint8 VERSION = 0
uint8 version

# PSTOP state - true indicates that the PSTOP is indicating STOP
bool pstop_pressed

ProtectiveStopStatus connection_status

# Local timestamp of the machine or PSTOP sending this message
builtin_interfaces/Time stamp

# Unique ID for the machine or PSTOP
string sender_uuid

# Unique ID for the machine or PSTOP this message is sent to
string receiver_uuid

# counter increments bgy 1 for every sent message, will not roll over in any realistic use case
uint32 counter

# Checksum type (ex: CRC-16, SHA-256, MD5, etc)
string checksum_type

# Checksum value, should be a hexadecimal string
string checksum_value
