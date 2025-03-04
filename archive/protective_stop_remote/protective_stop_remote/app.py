#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Handles configuration via webpage + saving config to json file

import json
import subprocess
import re
from flask import Flask, request, redirect, render_template, jsonify
import logging

app = Flask(__name__)
CONFIG_FILE = 'config.json'

# Functions for managing Wi-Fi connections


def list_wifi_connections():
    try:
        result = subprocess.run(['nmcli', '-t', '-f', 'NAME,DEVICE,STATE',
                                'connection', 'show'], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"nmcli error: {result.stderr}")

        connections = []
        for line in result.stdout.strip().split("\n"):
            name, device, state = line.split(':')
            connections.append(
                {"name": name, "device": device, "state": state})
        return connections
    except Exception as e:
        print(f"Error listing Wi-Fi connections: {e}")
        return []


def add_wifi_connection(ssid, password):
    try:
        result = subprocess.run(['nmcli', 'dev', 'wifi', 'connect',
                                ssid, 'password', password], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"nmcli error: {result.stderr}")
        return True
    except Exception as e:
        print(f"Error adding Wi-Fi connection: {e}")
        return False


def remove_wifi_connection(name):
    try:
        result = subprocess.run(
            ['nmcli', 'connection', 'delete', name], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"nmcli error: {result.stderr}")
        return True
    except Exception as e:
        print(f"Error removing Wi-Fi connection: {e}")
        return False


def activate_wifi_connection(name):
    """Activate a specific Wi-Fi connection."""
    try:
        result = subprocess.run(
            ['nmcli', 'connection', 'up', name], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"nmcli error: {result.stderr}")
        return True
    except Exception as e:
        print(f"Error activating Wi-Fi connection: {e}")
        return False

# Tailscale and signal strength functions


def get_wifi_signal_strength():
    try:
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'IN-USE,SIGNAL', 'dev', 'wifi'], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"nmcli error: {result.stderr}")

        wifi_lines = result.stdout.strip().split("\n")
        for line in wifi_lines:
            if line.startswith('*'):
                _, signal = line.split(':')
                return int(signal)
    except Exception as e:
        print(f"Error getting Wi-Fi signal strength: {e}")
        return None


def get_cellular_signal_strength():
    try:
        result = subprocess.run(
            ['mmcli', '-L'], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"mmcli error: {result.stderr}")

        output = result.stdout.strip()
        match = re.search(r'/Modem/(\d+)', output)
        if not match:
            raise Exception("Could not find a modem index in mmcli output.")

        modem_index = match.group(1)

        result = subprocess.run(
            ['mmcli', '-m', modem_index], capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"mmcli error: {result.stderr}")

        output = result.stdout
        signal_match = re.search(r'signal quality: (\d+)', output)
        if signal_match:
            return int(signal_match.group(1))
        else:
            raise Exception("Signal quality not found in modem details.")
    except Exception as e:
        print(f"Error getting cellular signal strength: {e}")
        return None


def fetch_tailscale_info(shared):
    try:
        result = subprocess.run(
            ["tailscale", "status", "--json"], capture_output=True, text=True, check=True)
        status = json.loads(result.stdout)
        tailscale_ips = status.get("Self", {}).get("TailscaleIPs", [])
        dns_name = status.get("Self", {}).get("DNSName", "")
        shared["TailscaleIP"] = tailscale_ips[0] if tailscale_ips else "N/A"
        shared["DNSName"] = dns_name if dns_name else "N/A"
    except Exception as e:
        print(f"Error fetching Tailscale info: {e}")
        shared["TailscaleIP"] = "N/A"
        shared["DNSName"] = "N/A"

# Load and save configuration


def load_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_config(cfg):
    """Write the shared dictionary to the config.json file."""
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print(f"Error saving config: {e}")

# Shared dictionary initialization


def initialize_shared_dict(shared):
    """Ensure all required keys exist in the shared dictionary with default values."""
    if "status_summary" not in shared:
        shared["status_summary"] = "Not Connected"
    if "uuid" not in shared:
        shared["uuid"] = "Unassigned!"
    if "TailscaleIP" not in shared:
        shared["TailscaleIP"] = "N/A"
    if "DNSName" not in shared:
        shared["DNSName"] = "N/A"
    if "battery_level" not in shared:
        shared["battery_level"] = "Unknown"
    if "WifiSignal" not in shared:
        shared["WifiSignal"] = "N/A"
    if "CellularSignal" not in shared:
        shared["CellularSignal"] = "N/A"
    if "target_pairs" not in shared:
        shared["target_pairs"] = []
    if "WifiConnections" not in shared:
        shared["WifiConnections"] = []


def load_shared_from_config(shared):
    """Load saved configuration from config.json into the shared dictionary."""
    config = load_config()
    for key, value in config.items():
        shared[key] = value


# Flask endpoints
def config_app(shared={}):
    # Initialize shared dictionary with default keys
    load_shared_from_config(shared)
    initialize_shared_dict(shared)

    @app.route('/data', methods=['GET'])
    def get_data():
        """API endpoint for live data updates."""
        fetch_tailscale_info(shared)
        shared["WifiSignal"] = get_wifi_signal_strength() or "N/A"
        shared["CellularSignal"] = get_cellular_signal_strength() or "N/A"
        shared["WifiConnections"] = list_wifi_connections()
        return jsonify({
            "status_summary": shared.get("status_summary", "Not Connected"),
            "uuid": shared.get("uuid", "Unassigned"),
            "TailscaleIP": shared.get("TailscaleIP", "N/A"),
            "DNSName": shared.get("DNSName", "N/A"),
            "BatteryLevel": shared.get("battery_level", "Unknown"),
            "WifiSignal": shared.get("WifiSignal", "N/A"),
            "CellularSignal": shared.get("CellularSignal", "N/A"),
            "TargetPairs": shared.get("target_pairs", []),
            "WifiConnections": shared["WifiConnections"]
        })

    @app.route('/wifi', methods=['POST'])
    def manage_wifi():
        action = request.form.get('action', '')
        if action == 'add':
            ssid = request.form.get('ssid', '')
            password = request.form.get('password', '')
            if ssid and password:
                if add_wifi_connection(ssid, password):
                    return redirect('/')
        elif action.startswith('remove_'):
            name = action.split('_', 1)[1]
            if remove_wifi_connection(name):
                return redirect('/')
        elif action.startswith('activate_'):
            name = action.split('_', 1)[1]
            if activate_wifi_connection(name):
                return redirect('/')
        return redirect('/')

    @app.route('/', methods=['GET', 'POST'])
    def index():
        if request.method == 'POST':
            action = request.form.get('action', '')
            if action == 'add':
                new_ip = request.form.get('new_ip', '')
                new_uuid = request.form.get('new_uuid', '')
                if new_ip and new_uuid:
                    pairs = list(shared['target_pairs'])
                    pairs.append({'ip': new_ip, 'uuid': new_uuid,
                                 'status': "Not Connected"})
                    shared['target_pairs'] = pairs
                    save_config(dict(shared))
            elif action.startswith('remove_'):
                idx_str = action.split('_', 1)[1]
                try:
                    idx = int(idx_str)
                    pairs = list(shared['target_pairs'])
                    if 0 <= idx < len(pairs):
                        pairs.pop(idx)
                        shared['target_pairs'] = pairs
                        save_config(dict(shared))
                except ValueError:
                    pass
            return redirect('/')

        return render_template('index.html')

    return app


def run_app(shared):
    app = config_app(shared)
    app.run(host='0.0.0.0', port=80)


if __name__ == '__main__':
    run_app(shared={})
