#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lux Logger for UT383BT LUX Meter via Bluetooth on macOS
=======================================================

This script connects to the Uni-T UT383BT Bluetooth LUX meter and logs
real-time LUX values with timestamps (in epoch time) to a CSV file for later analysis.

Features:
---------
- Connects to the UT383BT LUX meter via BLE.
- Sends a command to enable notifications for LUX values.
- Subscribes to the Data Out characteristic to receive LUX measurements.
- Logs retrieved LUX values and timestamps (epoch time) to a CSV file.

Usage:
------
- Run the script to start logging LUX values.
- The data is saved in the 'logs' directory as 'lux_data.csv'.

Requirements:
-------------
- Python 3.8 or higher.
- bleak library (for BLE communication).
- Ensure that the device UUID and characteristic UUIDs match your device.

License:
--------
This script is released under the "Do What The F*ck You Want To Public License" (Version 2).
"""

import asyncio
import re
import os
import csv
import time  # Changed from datetime to time for epoch timestamps
from bleak import BleakClient

# UUIDs for the UT383BT Device
DEVICE_UUID = "5D769E40-0CCD-8741-F6C7-0A5D76800EEB"  # Replace with your device's UUID if different
DATA_IN_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # UUID for the Data In characteristic
DATA_OUT_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"  # UUID for the Data Out characteristic

LOG_DIR = "logs"
LOG_FILE_NAME = "lux_data.csv"
LOG_FILE_PATH = os.path.join(LOG_DIR, LOG_FILE_NAME)

class LuxLogger:
    """
    A class to handle connection to the UT383BT device and logging of LUX values.
    """
    def __init__(self, device_uuid, data_in_uuid, data_out_uuid, log_file_path):
        self.device_uuid = device_uuid
        self.data_in_uuid = data_in_uuid
        self.data_out_uuid = data_out_uuid
        self.log_file_path = log_file_path
        self.client = None
        self.csv_file = None
        self.csv_writer = None

    async def connect(self):
        """
        Connects to the BLE device and sets up notifications.
        """
        self.client = BleakClient(self.device_uuid)
        await self.client.connect()
        if not self.client.is_connected:
            print("Failed to connect to the device.")
            return False

        print(f"Connected to device: {self.device_uuid}")

        # Ensure the logs directory exists
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)

        # Open the CSV file for appending data
        file_exists = os.path.isfile(self.log_file_path)
        self.csv_file = open(self.log_file_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header if file is new
        if not file_exists:
            self.csv_writer.writerow(['timestamp', 'lux_value'])

        # Start notifications
        await self.client.start_notify(self.data_out_uuid, self.notification_handler)
        print(f"Subscribed to notifications on {self.data_out_uuid}.")

        return True

    async def disconnect(self):
        """
        Stops notifications, disconnects from the device, and closes the log file.
        """
        if self.client and self.client.is_connected:
            await self.client.stop_notify(self.data_out_uuid)
            await self.client.disconnect()
            print(f"Disconnected from device: {self.device_uuid}")

        if self.csv_file:
            self.csv_file.close()
            print(f"Log file {self.log_file_path} closed.")

    async def send_command_periodically(self, command, interval):
        """
        Sends a command to the device periodically.

        Args:
            command (bytes): The command to send.
            interval (float): The interval in seconds between commands.

        Returns:
            None
        """
        try:
            while True:
                await self.client.write_gatt_char(self.data_in_uuid, command)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            print(f"Error sending command: {e}")

    def notification_handler(self, sender, data):
        """
        Handles incoming notifications from the Data Out characteristic.
        Parses and logs the LUX values with timestamps.

        Args:
            sender (str): The UUID of the characteristic that sent the notification.
            data (bytes): The raw data received from the characteristic.

        Returns:
            None
        """
        try:
            # Print raw data in hex format for debugging
            print(f"Notification from {sender}: {data.hex()}")

            # Attempt to extract the LUX value from the ASCII data
            # Find the start (':') and end (';') of the ASCII string
            colon_index = data.find(b':')
            semicolon_index = data.find(b';', colon_index)
            if colon_index != -1 and semicolon_index != -1:
                ascii_data = data[colon_index+1:semicolon_index].decode('ascii', errors='ignore').strip()
                # Use regex to find the LUX value
                match = re.search(r'(\d+)\s*LUX', ascii_data)
                if match:
                    lux_value = int(match.group(1))
                else:
                    # If 'LUX' is not present, extract digits directly
                    digits = ''.join(filter(str.isdigit, ascii_data))
                    if digits:
                        lux_value = int(digits)
                    else:
                        print("No LUX value found in data.")
                        return
                # Get current timestamp in epoch time
                timestamp = time.time()
                # Write to CSV
                self.csv_writer.writerow([timestamp, lux_value])
                self.csv_file.flush()  # Ensure data is written to file
                print(f"Logged LUX Data: {lux_value} LUX at {timestamp}")
            else:
                print("Colon ':' or semicolon ';' not found in data.")
        except Exception as e:
            print(f"Error parsing notification data: {e}")

    async def start_logging(self, duration=30, command_interval=1):
        """
        Starts the logging process.

        Args:
            duration (float): Total duration in seconds to log data.
            command_interval (float): Interval in seconds to send the command.

        Returns:
            None
        """
        try:
            # Start sending command periodically
            command = bytes([0x5E])
            send_command_task = asyncio.create_task(
                self.send_command_periodically(command, command_interval)
            )
            print(f"Started sending {command.hex()} to {self.data_in_uuid} every {command_interval} seconds.")

            # Keep the script running for the specified duration
            await asyncio.sleep(duration)

        finally:
            send_command_task.cancel()
            await self.disconnect()

if __name__ == "__main__":
    async def main():
        lux_logger = LuxLogger(DEVICE_UUID, DATA_IN_UUID, DATA_OUT_UUID, LOG_FILE_PATH)
        connected = await lux_logger.connect()
        if connected:
            await lux_logger.start_logging(duration=60, command_interval=1)

    asyncio.run(main())
