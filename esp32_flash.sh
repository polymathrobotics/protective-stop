#!/bin/bash
# Flashing ESP32 with specific firmware
python -m esptool --chip esp32 --port "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0" --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 "estop_V2_MCU.ino.bootloader.bin" 0x8000 "estop_V2_MCU.ino.partitions.bin" 0xe000 "boot_app0.bin" 0x10000 "estop_V2_MCU.ino.bin"
echo "Flashing complete."
sleep 1
# Unload and reload cp210x kernel module to reset USB-to-UART bridge
echo "Resetting USB-to-UART bridge..."
sudo modprobe -r cp210x
sudo modprobe cp210x
sleep 1
echo "Bridge reset complete."
# Configure serial port settings and read from the serial device
echo "Configuring serial port settings..."
sudo stty -F /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 115200
sleep 1
echo "Reading from serial port..."
sudo cat /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
# Note: The 'cat' command will keep reading the serial port indefinitely.
# Use Ctrl+C to stop it, or adapt this part of the script as needed for your application