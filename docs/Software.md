# Software/Firmware

## Imaging Raspberry Pi
1. Basic Set-up
- Take a microSD card and insert it into a computer with [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Use a microSD adapter if necessary.
- In the imager:
    - For device, elect “Raspberry Pi Zero 2W”.
    - For operating system, use “Custom” and use `2024-03-15-raspios-bookworm-arm64-lite.img`. This file is included in the repo but you will have to unzip.
    - For storage, choose the microSD card you just inserted.
<p align="center">
  <img width="30%" src="/docs/img/rpi-imager.png"> <br><i> Raspberry Pi Imager </i>
</p>

2. Custom settings — “General”
- After clicking next, there is a pop-up for OS customization. Click “Edit Settings”.
- Set a hostname, as well as a username and password
- Input WiFi credentials - note that you likely can't use a 5G network.
<p align="center">
  <img width="30%" src="/docs/img/rpi-custom-1.png"> <br><i> Custom Settings - General </i>
</p>

3. Enable SSH
- Go to the “Services” tab and click Enable SSH.
- Use password authentication so other people can communicate with the Raspberry Pi other than you.
<p align="center">
  <img width="30%" src="/docs/img/rpi-custom-ssh.png"> <br><i> Custom Settings - General </i>
</p>

4. Proceed with flashing the Pi — there will be a “write” and “verify” step.

5. Remove microSD from your computer and insert it in the Raspberry Pi. Turn on power and wait a couple minutes for the Raspberry Pi to boot.  

6. You can use find the IP address of the Raspberry Pi by going on your computer and running `nmap -p 22 --open [your network range]`.
- Your network range will be something like `192.168.93.0/24`, and can be found by running `ifconfig` or `ip addr`.
- Alternatively, you can use the more aggressive `nmap -sV -p 22 192.168.93.0/24`, and identify the Raspberry Pi from its operating system.

7. Once you’ve found the IP address of the Raspberry Pi, connect to it over SSH by using: `ssh [username]@[ip address]`
- Here `[username]` is the username you set during the configuration step before, and the `[ip address]` is the IP address of the Raspberry Pi you just found.
- You will be prompted for the password — this is the password set earlier during the OS customization step.

## Raspberry Pi Software Installation
1. After connecting to the Raspberry Pi through SSH, install Tailscale on it with the following commands. You will be provided a link to log in to your Tailscale account after the last step.
```bash
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list
sudo apt-get update
sudo apt-get install tailscale
sudo tailscale up
```
2. We can then check the Tailscale address of the Rasperry Pi with `tailscale ip -4`. Now, we can access the Raspberry Pi over VPN anywhere through Tailscale by using `ssh [username]@[tailscale ip]`

3. Install the following packages on the Raspberry Pi:
- Install chrony with `sudo apt install chrony`
- Install pip with `sudo apt install python3-pip -y` 
- Install PySerial with `sudo pip install pyserial --break-system-packages`
- Install the roslibpy with: 
```
sudo apt install git-all
git clone https://github.com/gramaziokohler/roslibpy.git
cd roslibpy
sudo pip install -r requirements-dev.txt --break-system-packages
```

## ESP32 Set-up
On the Raspberry Pi (over SSH):
1. Install esptool with `pip install esptool --break-system-packages`
2. Make sure these four files on Raspberry Pi, which are in the [pstop_MCU build folder](../pstop_MCU/build/esp32.esp32.esp32/) folder:
- `boot_app0.bin`
- `pstop_MCU.ino.bin`
- `pstop_MCU.ino.bootloader.bin`
- `pstop_MCU.ino.partitions.bin`
- If you are moving these files from your local computer, this can be done by going into the directory with these files and doing:
```
scp esp32_new.ino.bin esp32_new.ino.bootloader.bin esp32_new.ino.partitions.bin [username]@[tailscale ip]:/path/to/home/dir
```
- The original code [`pstop_MCU.ino`](../pstop_MCU/pstop_MCU.ino) file is also included, so you can make changes and re-compile if desired.
3. Now, run the [esp32_flash.sh](../esp32_flash.sh) bash script with `bash esp32_flash.sh`. This should output something like this:
```
polymath@polymath-estop-001:~ $ bash esp32_flash.sh
esptool.py v4.7.0
Serial port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
Connecting.....
Chip is ESP32-D0WD-V3 (revision v3.1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: e4:65:b8:0f:49:f4
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 921600
Changed.
Configuring flash size...
Flash will be erased from 0x00001000 to 0x00005fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x0000e000 to 0x0000ffff...
Flash will be erased from 0x00010000 to 0x000e4fff...
Compressed 18992 bytes to 13112...
Wrote 18992 bytes (13112 compressed) at 0x00001000 in 0.3 seconds (effective 583.1 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 146...
Wrote 3072 bytes (146 compressed) at 0x00008000 in 0.0 seconds (effective 1368.9 kbit/s)...
Hash of data verified.
Compressed 8192 bytes to 47...
Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.0 seconds (effective 2240.4 kbit/s)...
Hash of data verified.
Compressed 870384 bytes to 563721...
Wrote 870384 bytes (563721 compressed) at 0x00010000 in 7.2 seconds (effective 968.5 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Flashing complete.
Resetting USB-to-UART bridge...
Bridge reset complete.
Configuring serial port settings...
Reading from serial port...
```
4. After seeing "Reading from serial port", you can stop the script with Ctrl + C.
5. At this point, the LED ring should be lighting up and be responsive to the red stop button.

## SIM Module Set-up
On the Raspberry Pi:
1. Install minicom with `sudo apt install minicom`
2. Start minimcom with `sudo systemctl stop ModemManager.service && minicom -D /dev/ttyUSB2`
- This may not work properly if `/dev/ttyUSB2` is not the right address.
- You can tell whether it's working properly by typing `ATE` and then Enter in the minicom terminal and seeing if there is an `OK` response.
- You may have to try some different addresses, which can be found with:
```
cd /dev/serial/by-id
ls
```
3. In the minicom terminal, issue the following commands:
```
ATE
AT&F
ATI
AT&V
AT+CGDCONT?
AT+CUSBPIDSWITCH?
AT+CGPSAUTO?
AT+CUSBPIDSWITCH=9001,1,1
AT+CGDCONT=1,"IPV4V6","h2g2"
AT+CGDCONT=6,"IPV4V6","h2g2"
AT+CGPSAUTO=1
```
4. Exit minicom (`Ctrl+A, Z, X`), and then power cycle the system.
5. Now, configure the Raspian Bookworm Network Manager with: `sudo nmcli connection add type gsm ifname '*' con-name '1-gsm' apn 'h2g2' connection.autoconnect yes`
6. If you need more detailed instructions, refer to https://wimsworld.wordpress.com/2023/12/

## E-ink Paper Display
On the Raspberry Pi terminal:
1. Install the following:
```
sudo pip3 install RPi.GPIO
sudo pip3 install spidev
```
2. Git clone the e-Paper repo with `git clone https://github.com/waveshare/e-Paper.git`
3. Go in the Raspberry Pi directory with: `cd e-Paper/RaspberryPi_JetsonNano/`
4. Set up the libraries with `sudo python3 [setup.py](http://setup.py/) install`
5. Install flask with `pip install flask --break-system-packages`
6. Configure the Raspberry Pi by typing `sudo raspi-config`. 
- Go to `Choose Interfacing Options -> SPI -> Yes Enable SPI interface`
<p align="center">
  <img width="30%" src="/docs/img/rpi-config.png"> <br><i> Raspberry Pi Config Menu </i>
</p>

7. Reboot your Raspberry Pi with either `sudo reboot` or power cycle.

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
5. To verify that our custom message set-up was successful, you can run `ros2 interface show estop_interface/msg/EStopMsg`, which should print out the message definition.

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