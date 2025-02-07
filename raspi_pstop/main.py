#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Automatically started on boot, combines data from other nodes

from multiprocessing import Process, Manager

from power import power_node # Handles power button, UPS management, shutdown command
from ui import ui_node # Controls the LED ring, E paper display
from config import config_node # Handles configuration via webpage + saving config to json file
from pstop import pstop_node # Main pstop node, communicates with machine node on robot
from button import button_node # Handles pstop button

import logging

if __name__ == '__main__':
    with Manager() as manager:
        # This manager dict will be visible to all processes
        shared_data = manager.dict()

        # Create two processes, one for ROS node and one for UI node
        power_process = Process(target=power_node, args=(shared_data,))
        config_process = Process(target=config_node,  args=(shared_data,))
        ui_process = Process(target=ui_node, args=(shared_data,))
        pstop_process = Process(target=pstop_node, args=(shared_data,))
        button_process = Process(target=button_node, args=(shared_data,))

        ui_process.start()
        power_process.start()
        config_process.start()
        button_process.start()
        pstop_process.start()

        ui_process.join()
        power_process.join()
        config_process.join()
        button_process.join()
        pstop_process.join()
