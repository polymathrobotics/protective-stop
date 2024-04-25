# Robot Quick-Start
Have a protective stop and want to set up your robot to work with it? Follow these instructions.

## Install Tailscale
1. On your robot, install Tailscale on it with the following commands. You will be provided a link to log in to your Tailscale account after the last step.
```bash
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list
sudo apt-get update
sudo apt-get install tailscale
sudo tailscale up
```
2. We can then check the Tailscale address of the robot with `tailscale ip -4`.


## roslibpy Custom Message Set-up
On your local machine your local machine or (robot that you want the p-stop to control):
1. Install rosbridge-server with `sudo apt-get install ros-$ROS-DISTRO-rosbridge-server`.
2. Create a colcon workspace if you don't already have one:
```
mkdir -p colcon_ws/src
cd colcon_ws/src
```
3. Put the [`pstop_msg`](../pstop_msg) folder in this repo inside the `src` directory. It contains a custom message definition for our protective stop.
4. Go back to the `colcon_ws` directory and run:
```
colcon build
source install/setup.bash
```
5. To verify that our custom message set-up was successful, you can run `ros2 interface show pstop_msg/msg/EStopMsg`, which should print out the message definition.

## roslibpy Client Set-up
In a terminal on your local machine where you have built and sourced the custom message type above:
1. Launch a rosbridge_server with `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`.
- If you are running this in a Docker container, make sure that port 9090 is exposed by doing `docker run -p 9090:9090 ...`
2. To run the roslibpy client, run `python roslibpy_client.py [ip address]`. 
- You can set a default ip address by altering line 12 of [`roslibpy_client.py`](../roslibpy_client.py):
```
parser.add_argument('target', type=str, help='Target IP address', default='')
```
3. To use the flask interface, connect to the Raspberry Pi through SSH with portforwarding:
```
ssh -L 8000:localhost:5000 [username]@[tailscale ip]
```
- Now, you should be able to see the flask interface by going to `http://localhost:8000/config`.