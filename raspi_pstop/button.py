#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Handles pstop button physical interaction

import RPi.GPIO as GPIO
import time
import logging

# GPIO setup
# 15 connected to 23 when button pressed
# 14 connected to 23 when button released

def button_node(shared):
    logging.basicConfig(level=logging.DEBUG)
    OUTPUT_PIN = 23
    INPUT_PINS = [14, 15]
    
    if "button_pressed" not in shared:
        shared["button_pressed"] = None
    
    try:
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(OUTPUT_PIN, GPIO.OUT)
    
        logging.info(f"Button Node starting at {time.time()}")
        # The following code toggles output high and low, and checks for correct return
        # If the value is not correct, one or both of the return lines is broken    
        while True:
            GPIO.output(OUTPUT_PIN, GPIO.HIGH)
            for pin in INPUT_PINS:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            new_state = None
            # Released check on high
            if GPIO.input(INPUT_PINS[0]) and not GPIO.input(INPUT_PINS[1]):
                new_state = False
            # Pressed check on high
            elif not GPIO.input(INPUT_PINS[0]) and GPIO.input(INPUT_PINS[1]):
                new_state = True
            else:  # Wire failure
                new_state = None

            GPIO.output(OUTPUT_PIN, GPIO.LOW)
            for pin in INPUT_PINS:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Released check on low
            if not GPIO.input(INPUT_PINS[0]) and GPIO.input(INPUT_PINS[1]):
                new_state = False
            # Pressed check on low
            elif GPIO.input(INPUT_PINS[0]) and not GPIO.input(INPUT_PINS[1]):
                new_state = True
            else:  # Wire failure
                new_state = None

            # Log only when the state changes
            if new_state != shared["button_pressed"]:
                shared["button_pressed"] = new_state
                if new_state is True:
                    logging.info(f"Button pressed at {time.time()}")
                elif new_state is False:
                    logging.info(f"Button released at {time.time()}")
                else:
                    logging.info(f"Button state unknown (failure) at {time.time()}")

            time.sleep(0.05)  # 20 Hz
        
    except KeyboardInterrupt:
        logging.info(f"Button Node exiting at {time.time()}")    
        
    finally:   
        GPIO.cleanup()
            
# Running standalone if needed
if __name__ == '__main__':
    button_node(shared={})
