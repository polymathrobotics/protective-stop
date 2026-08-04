# This message definition is based on the PStopModel Pydantic model.
# It contains a state string, a remote status, a non-negative counter,
# three timestamp fields, and two additional state strings.

string uuid

# PSTOP state - true indicates that the PSTOP is indicating STOP
bool pstop_pressed

ProtectiveStopStatus connection_status

# A non-negative counter (default: 0)
uint32 counter

builtin_interfaces/Time init_timestamp    # Initialization timestamp (required)
builtin_interfaces/Time receive_timestamp # Receive timestamp (if not available, use zero values)
builtin_interfaces/Time remote_timestamp  # Remote timestamp (if not available, use zero values)
