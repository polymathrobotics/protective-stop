
Requires: https://github.com/RobotWebTools/rosbridge_suite


Quickstart:

On the machine:
- ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9190
- ros2 run protective_stop machine_node.py
- if needed, randomly generate UUID with:  
ros2 param set /pstop_machine_node machine_UUID [$(uuidgen | sed 's/-//g' | fold -w2 | head -n16 | awk '{printf "%d,", "0x"$1}' | sed 's/,$//')]
- if needed, dynamically change the heartbeat timeout to 2 seconds (for example) with: 
ros2 param set /pstop_machine_node heartbeat_timeout 2.0

