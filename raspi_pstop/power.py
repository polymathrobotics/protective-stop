#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Handles power button, UPS management, shutdown command

import time
import smbus
import subprocess
import logging
import traceback

# ========== I²C Setup (PiSugar) ==========
I2C_BUS = 1
DEVICE_ADDR = 0x57
REG_BUTTON = 0x02       # Register to read button state (LSB) & modify bits
REG_ACTION = 0x09       # Register to write an action/delay if needed

PRESS_THRESHOLD = 0.2   # Number of seconds to hold button

bus = smbus.SMBus(I2C_BUS)
button_pressed_start = None

def shutdown_process(shared):
    logging.info("==> Shut Down Sequence Initiated...")
    shared['shutdown'] = True

    # Sets shutdown delay to 20 seconds
    bus.write_byte_data(DEVICE_ADDR, REG_ACTION, 20)

    value = None
    while value is None:
        try:
            value = bus.read_byte_data(DEVICE_ADDR, REG_BUTTON)
        except OSError as e:
            logging.warning(f"I2C Read Error: {e}. Retrying...")
            continue

    # Clear the 5th bit (bit index 5) of 0x02
    # bit 5 => (1 << 5) = 0x20
    new_value = value & ~(1 << 5)  # set bit 5 to 0

    try:
        bus.write_byte_data(DEVICE_ADDR, REG_BUTTON, new_value)
        verify_val = bus.read_byte_data(DEVICE_ADDR, REG_BUTTON)
        logging.info(f"Verified 0x02 after write: 0x{verify_val:02X}")
    except OSError as e:
        logging.warning(f"I2C Write/Verify Error (REG_BUTTON): {e}")


    # ========== Issue System Shutdown ==========
    logging.info("Issuing system shutdown command...")
    subprocess.run(["sudo", "shutdown", "-h", "now"])

def power_node(shared):

    logging.basicConfig(level=logging.DEBUG)

    # Optional: Make certain registers writable (per PiSugar docs, if needed)
    try:
        bus.write_byte_data(DEVICE_ADDR, 0x0B, 0x29)
        logging.info("Setting 0x0B = 0x29 to enable writable registers.")
    except OSError as e:
        logging.warning(f"I2C Write Error (0x0B): {e}")
        
    shared['shutdown'] = False

    # ========== Main Loop ==========
    while True:
        # ========== Battery Check ==========
        try:
            # Use shell=True so we can pipe directly in one command
            battery_output = subprocess.check_output(
                "echo 'get battery' | nc -q 0 127.0.0.1 8423",
                shell=True
            ).decode('utf-8').strip()

            # If battery_output is something like "37%" or "37", parse it
            # Remove any trailing '%' if present:
            battery_value = battery_output.replace('%', '').strip()[9:]
            battery_percentage = round(float(battery_value),1)
            shared['battery_level'] = battery_percentage

            #logging.warning(f"Battery level is ({battery_percentage}%).")
            if battery_percentage < 4:
                logging.warning(f"Battery level is below 4% ({battery_percentage}%).")
                shutdown_process(shared)

        except Exception as e:
            logging.warning(f"Battery check failed: {e}")

        # ========== Button Handling ==========
        try:
            value = bus.read_byte_data(DEVICE_ADDR, REG_BUTTON)
        except OSError as e:
            logging.warning(f"I2C Read Error: {e}. Retrying...")
            time.sleep(0.1)
            continue

        # Bit 0 is the button state (1=pressed, 0=released)
        button_state = value & 0x01

        if button_state == 1:
            # Button is pressed
            if button_pressed_start is None:
                # Record the time when press began
                button_pressed_start = time.time()
            else:
                # Check how long it's been held
                if (time.time() - button_pressed_start) >= PRESS_THRESHOLD:

                    shutdown_process(shared)
                    # Reset press timer so we only do this once per press
                    button_pressed_start = None
        else:
            # Button is released
            button_pressed_start = None

        time.sleep(0.1)
    
    
if __name__ == '__main__':
    power_node(shared={})  # Normal dict here for a simple test
