#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Controls the LED ring, E paper display
# Due to control of LED ring, must be run as root on raspi

# On the e-ink display, we want to show:
# Polymath Logo
# Tailscale DNS Name
# Battery Level, wifi and cell signal
# UUID
# Target UUID and IP
# High Level Status (Not Connected, Connecting, OK, ESTOP, Lost Connection)

# The LED ring will mirror high level status:
# Not Connected = Solid Blue
# Connecting = Pulsing Blue
# OK = Solid Green
# ESTOP = Spinning Red
# Lost Connection = Flashing Yellow

from waveshare_epd import epd2in13_V4
from PIL import Image, ImageDraw, ImageFont

import time
import board
import neopixel
import logging


pixel_pin = board.D18
# The number of NeoPixels
num_pixels = 16
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=50, auto_write=False, pixel_order=ORDER
)

# NeoPixel ring configuration
PIXEL_PIN = board.D18  # GPIO pin connected to the NeoPixels
NUM_PIXELS = 16        # Number of pixels in the ring
BRIGHTNESS = 1       # Brightness of the LEDs (0.0 to 1.0)
GAMMA = 1.8            # Gamma correction value for human eyesight
FADE_FACTOR = 0.6      # Base fading factor for trail (stronger fade)
TRAIL_LENGTH = 7       # Number of pixels in the fading trail
FRAME_DELAY = 0.05     # Delay between frames in seconds

font24 = ImageFont.truetype('/home/administrator/static/Font.ttc', 12)
logo = Image.open('/home/administrator/static/images/polymath_logo_small.png')

def gamma_correct(value, gamma):
    """Apply gamma correction to a value."""
    return int((value / 255) ** gamma * 255)

def fade_pixel(color, factor):
    """Fade a color by a given factor with gamma correction."""
    return tuple(gamma_correct(int(c * factor), GAMMA) for c in color)
    
def fading_trail():
    """Create a fading trail effect with two red segments chasing each other."""
    segment1 = 0
    segment2 = NUM_PIXELS // 2  # Second segment starts opposite to the first

    RED = (255, 0, 0)  # Red color

    while True:
        # Clear the ring
        pixels.fill((0, 0, 0))

        # Update segment positions
        segment1 = (segment1 - 1) % NUM_PIXELS
        segment2 = (segment2 - 1) % NUM_PIXELS

        # Draw the main segments and their trails
        for i in range(TRAIL_LENGTH):
            factor = FADE_FACTOR ** (TRAIL_LENGTH - i - 1)
            pixels[(segment1 - i) % NUM_PIXELS] = fade_pixel(RED, factor)
            pixels[(segment2 - i) % NUM_PIXELS] = fade_pixel(RED, factor)

        # Show the updated pixel colors
        pixels.show()

        # Wait a bit before the next frame
        time.sleep(FRAME_DELAY)

def solid_color(color):
    """Set all pixels to a solid color."""
    pixels.fill(color)
    pixels.show()

def pulsing_color(color, duration=2):
    """Pulse the LEDs with the given color over a specified duration."""
    steps = 50
    for step in range(steps):
        brightness = (1 + math.sin(2 * math.pi * step / steps)) / 2
        adjusted_color = tuple(int(c * brightness) for c in color)
        pixels.fill(adjusted_color)
        pixels.show()
        time.sleep(duration / steps)

def flashing_color(color, interval=0.5):
    """Flash the LEDs on and off with the given color at a specified interval."""
    for _ in range(int(5 / interval)):
        pixels.fill(color)
        pixels.show()
        time.sleep(interval)
        pixels.fill((0, 0, 0))
        pixels.show()
        time.sleep(interval)

def ui_node(shared):

    # ========== E-Paper Setup ==========
    try:
        # Instantiate the display
        epd = epd2in13_V4.EPD()
        logging.info("Initializing e-paper...")
        epd.init_fast()
        #epd.init()
        #epd.Clear(0xFF)
        
        # Display startup logo
        image1 = Image.new('1', (epd.height, epd.width), 255)  # 255: clear the frame
        image1.paste(logo, (2,2))    
        epd.display(epd.getbuffer(image1))


    except IOError as e:
        logging.error(f"E-Paper Initialization Error: {e}")
        epd = None  # If e-paper fails, we skip display updates

    # Update function for the E-Paper display layout
    def update_display(shared, epd, font):
        """Updates the e-paper display with system information."""
        try:
            image = Image.new('1', (epd.height, epd.width), 255)  # Create blank image
            draw = ImageDraw.Draw(image)

            # Polymath Logo
            image.paste(logo, (2, 2))

            # Layout configuration
            start_x, start_y = 5, 5  # Initial drawing position
            line_spacing = 20  # Spacing between lines

            # Extract shared memory values
            tailscale_dns = shared.get('DNSName', 'N/A')[:-1]
            battery_level = shared.get('battery_level', 'Unknown')
            wifi_signal = shared.get('WifiSignal', 'Unknown')
            cell_signal = shared.get('CellularSignal', 'Unknown')
            uuid = shared.get('uuid', 'N/A').split('-')[-1]
            target_uuid = shared.get('target_uuid', 'N/A')
            target_ip = shared.get('target_ip', 'N/A')
            high_level_status = shared.get('status_summary', 'Unknown')

            # Draw details
            draw.text((start_x+34, start_y-2), f"{tailscale_dns}", font=font, fill=0)
            draw.text((start_x+180, start_y+ 1 * line_spacing), f"Bat: {battery_level}%", font=font, fill=0)
            draw.text((start_x+34, start_y + 1 * line_spacing), f"WiFi: {wifi_signal}%", font=font, fill=0)
            draw.text((start_x+110, start_y + 1 * line_spacing), f"Cell: {cell_signal}%", font=font, fill=0)
            draw.text((start_x, start_y + 2 * line_spacing), f"Status: {high_level_status}", font=font, fill=0)
            draw.text((start_x, start_y + 3 * line_spacing), f"UUID: {uuid}", font=font, fill=0)
            draw.text((start_x, start_y + 4 * line_spacing), f"Target UUID: {target_uuid}", font=font, fill=0)
            draw.text((start_x, start_y + 5 * line_spacing), f"Target IP: {target_ip}", font=font, fill=0)


            # Update display
            epd.displayPartial(epd.getbuffer(image))
            #epd.display_fast(epd.getbuffer(image))

        except IOError as e:
            logging.error(f"Display Update Error: {e}")

    def display_shutdown_message():
        if epd:
            try:
                # Create a new blank image
                image = Image.new('1', (epd.height, epd.width), 255)
                draw = ImageDraw.Draw(image)
                image.paste(logo, (2, 2))

                # Write "PSTOP Shutting DOWN" near the center
                uuid = shared.get('uuid', 'N/A').split('-')[-1]
                tailscale_dns = shared.get('DNSName', 'N/A')[:-1]
                battery_level = shared.get('battery_level', 'Unknown')
                
                text = "PSTOP Powered Off"
                text_x, text_y = 10, 40  # Adjust positioning as needed
                draw.text((text_x, text_y), text, font=font24, fill=0)
                draw.text((text_x, text_y+ 1*20), f"{tailscale_dns}", font=font24, fill=0)
                draw.text((text_x, text_y + 2 * 20), f"UUID: {uuid}", font=font24, fill=0)
                draw.text((text_x, text_y + 3 * 20), f"Battery: {battery_level}%", font=font24, fill=0)

                # Display in partial update (for speed) or full update
                epd.displayPartBaseImage(epd.getbuffer(image))

                # Sleep the display if you want to finalize
                time.sleep(0.1)
                epd.sleep()

            except IOError as e:
                logging.error(f"E-Paper Display Error: {e}")

    try:
        while True:
            if 'shutdown' in shared and shared.get('shutdown', True):
                # Turn off LEDs and show shutdown message
                pixels.fill((0, 0, 0))
                pixels.show()
                display_shutdown_message()
                break
                
            if epd:
                # Update e-paper display
                update_display(shared, epd, font24)

            if 'status_summary' in shared:
                status = shared['status_summary']

                if status == "Not Connected":
                    solid_color((0, 0, 255))  # Solid Blue

                elif status == "Connecting":
                    pulsing_color((0, 0, 255))  # Pulsing Blue

                elif status == "OK":
                    solid_color((0, 255, 0))  # Solid Green

                elif status == "ESTOP":
                    fading_trail()  # Spinning Red (already implemented)

                elif status == "Lost Connection":
                    flashing_color((255, 255, 0))  # Flashing Yellow

                else:
                    pixels.fill((0, 0, 0))  # Turn off LEDs for unknown status
                    pixels.show()

            time.sleep(0.1)  # Adjust polling interval as needed
    except KeyboardInterrupt:
        print("[UI Node] Shutting down")
        pixels.fill((0, 0, 0))
        pixels.show()

# Running standalone if needed
if __name__ == '__main__':
    ui_node(shared={'status_summary': "Not Connected"})

