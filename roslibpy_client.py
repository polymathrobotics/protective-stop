import time
import socket
import serial
import roslibpy
import sys
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont
from waveshare_epd import epd2in13_V4
from flask import Flask, request, jsonify, render_template
import argparse

parser = argparse.ArgumentParser(description='roslibpy client')
parser.add_argument('target', type=str, help='Target IP address', default='')
args = parser.parse_args()

# Flask app for live configuration
app = Flask(__name__)

# Initialize global variables with default values
sender_id = socket.gethostname()
heartbeat_rate = 1
heartbeat_timeout = 5
target = args.target
battery_status = "Good"
state = ""

@app.route('/config', methods=['GET'])
def config():
    return render_template('config.html')

@app.route('/config', methods=['POST'])
def update_config():
    global sender_id, target, heartbeat_rate, heartbeat_timeout, client
    # Store old target to compare if it changes
    old_target = target

    sender_id = request.form.get('sender_id', sender_id)
    target = request.form.get('target', target)
    heartbeat_rate = int(request.form.get('heartbeat_rate', heartbeat_rate))
    heartbeat_timeout = int(request.form.get('heartbeat_timeout', heartbeat_timeout))

    # Update the display with the new settings
    update_display(sender_id, target, battery_status)

    # Check if target has changed, and reconnect if necessary
    if old_target != target:
        client.terminate()  # Properly terminate the existing ROS connection
        client = roslibpy.Ros(host=target, port=9090)  # Reinitialize with new target
        client.run()

    return jsonify({'sender_id': sender_id, 'target': target,                   
                    'heartbeat_rate': heartbeat_rate, 'heartbeat_timeout': heartbeat_timeout})


# Setup the e-Paper display
epd = epd2in13_V4.EPD()
epd.init()
epd.Clear(0xFF)

# Prepare for drawing
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 14)
image = Image.new('1', (epd.height, epd.width), 255)  # 255: clear the frame
draw = ImageDraw.Draw(image)

def update_display(sender_id, target, battery_status, message=None):
    draw.rectangle((0, 0, epd.height, epd.width), fill=255)
    if message:
        draw.text((10, 10), message, font=font, fill=0)
    else:
        draw.text((10, 10), 'Sender: ' + sender_id, font=font, fill=0)
        draw.text((10, 40), 'Target: ' + target, font=font, fill=0)
        draw.text((10, 70), 'Battery: ' + battery_status, font=font, fill=0)
    epd.display(epd.getbuffer(image.rotate(0)))

# Initialize ROS client
client = roslibpy.Ros(host=target, port=9090)
client.run()

talker = roslibpy.Topic(client, '/estop', 'estop_interface/msg/EStopMsg')
talker.advertise()

arduino_port = '/dev/ttyUSB0'
ser = serial.Serial(arduino_port, 115200, timeout=1)

# Update display initially
update_display(sender_id, target, battery_status)

last_connection_status = None
counter = 0

# Separate thread to run Flask app
from threading import Thread
flask_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False))
flask_thread.start()

try:
    while True:
        current_connection_status = 'CONNECTED' if client.is_connected else 'DISCONNECTED'
        if current_connection_status != last_connection_status:
            ser.write(f'{current_connection_status}\n'.encode())
            last_connection_status = current_connection_status

        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if "SHUTDOWN" in line:
                update_display(sender_id, target, battery_status, message="Shutdown")
                epd.sleep()  # Put the display to sleep after showing "Shutdown"
                client.terminate()  # Properly terminate the roslibpy client
                sys.exit()  # Exit the script safely
            if line in ["Normal", "Stopped", "Disconnected"]:
                state = line
            if "Battery Low" in line:
                if battery_status != "Low":
                    battery_status = "Low"
                    update_display(sender_id, target, battery_status)
            elif battery_status != "Good":
                battery_status = "Good"
                update_display(sender_id, target, battery_status)

        timestamp = datetime.now().isoformat()
        checksum = hash(f"{timestamp}{sender_id}{counter}")
        message_data = {
            "checksum": checksum,
            "sender_timestamp": timestamp,
            "sender_id": sender_id,
            "rolling_counter": counter,
            "expected_heartbeat_timeout": heartbeat_timeout,
            "state": state,
            "target": target
        }
        talker.publish(roslibpy.Message(message_data))
        print(f"Sending: {message_data}")
        counter += 1

        # Adjust sleep time according to the heartbeat rate
        sleep_time = 1.0 / heartbeat_rate if heartbeat_rate > 0 else 1  # Prevent division by zero
        time.sleep(sleep_time)

except KeyboardInterrupt:
    print("Shutdown requested by user")

finally:
    # Cleanup resources properly
    client.terminate()  # Properly terminate the roslibpy client
    epd.sleep()  # Put the e-paper display to sleep before exiting