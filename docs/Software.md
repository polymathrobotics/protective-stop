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
- Install `rosbridge-server` on your local machine (robot that you want the p-stop to control) with `sudo apt-get install ros-humble-rosbridge-server`.

## ESP32 Set-up

## SIM Module Set-up

## E-ink Paper Display

## roslibpy Client Set-up