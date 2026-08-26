# One remote currently bonded to the machine.

string device_id         # hex UUID, e.g. "01d7eed0"
uint8 bond_state         # 0 empty, 1 connecting, 2 bonded
bool in_use              # actively arming/holding this cycle
bool stop_only           # may STOP but never arm (effective policy: role AND allowlist)
uint8 claimed_role       # remote-announced role: 0 unspecified, 1 stop-only, 2 operator
uint32 reply_age_ms      # ms since last valid message from this remote
uint32 loop_rtt_ms       # round-trip on the pstop loop
uint32 disco_rtt_ms      # disco-layer path RTT (0 if unknown)
uint32 rebonds           # cumulative rebond count for this slot
