# Physical stop-circuit state of a pstop machine. Meaningful for the hardware
# (ESP32) backend; the software backend has no relays and sets applicable=false.

builtin_interfaces/Time stamp

bool applicable          # false on the software backend (no physical relays)
bool run                 # true = circuit closed / robot cleared to move
bool relay_stop          # true = stop asserted (circuit open)
bool relay_fault_a       # per-core relay feedback fault (channel A)
bool relay_fault_b       # per-core relay feedback fault (channel B)
uint32 mismatch          # dual-core lockstep mismatch count
