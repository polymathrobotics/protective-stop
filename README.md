# Polymath Protective Stop 🛑🛡

This initiative is focused on developing a low-cost, reliable protective stop mechanism for robotics and automation systems. Our protective stop system is designed to be a preventative measure that initiates a controlled shutdown with no immediate danger.

**Safety disclaimer:** This system is not designed to be an emergency stop; as such, it should not be used when there is an immediate threat to human safety or significant risk to equipment. It is not designed for immediate, uuncontrolled shutdowns.

## Quick Start

Start the Foxglove bridge:

```bash
 ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Start the node:

```bash
ros2 launch protective_stop_node protective_stop_node.launch.py
```

If you want to run the example remote, `cd` into that directory, and run:

```bash
npm install
npm run dev
```

## Components

### Protective Stop Node

ROS2 node that runs on your robot. This node publishes:

- `/protective_stop`: A heartbeat message back to the remote controlled by the user.
- `/pstop_hb`: A heartbeat message into your robot, which is used to actually stop the robot if `stop` is set to `True`.
- `/protective_stop/debug`: Debug message showing which remotes are actively connected.

To register a remote protective stop, the node also exposes two services:

- `/activate`
- `/deactivate`

In order to integrate with the node, you have to activate it first before starting to send heartbeat messages.


#### Unsafe Mode

**USE WITH EXTREME CAUTION**

In the case where your robot's internet connection is not reliable, there is an unsafe mode that you can trigger via the `/toggle_unsafe_mode` service, which will relax the requirement in the pstop node that it needs to be connected to a remote in order to send safe heartbeat messages to your controls system.

It is intentionally written as a service (instead of param) so users don't get lock in this param by default. It should only be used if you are aware of the risks.


### Protective Stop Remote

The remote is run next to the operator of the P-Stop. It exchanges heartbeat messages with the node on the robot, the latter of which will signal if it detects that the heartbeats are coming in at the expected rate.

In this repo currently, we've implemented it as a web client, but it can also be instantiated as a hardware interface.


### Foxglove Websocket Bridge

The bridge acts as the proxy between the remote and the node, so the two can talk to one-another. If it is not online, the other components will fail gracefully.
