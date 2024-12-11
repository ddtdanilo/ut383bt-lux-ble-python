#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synchronized Lux Logger for BLE and Serial Sensors
==================================================

This script connects to the Uni-T UT383BT Bluetooth LUX meter and a serial-connected LUX sensor,
logs synchronized LUX values with timestamps (in epoch time) to a CSV file for later analysis.

Features:
---------
- Connects to the UT383BT LUX meter via BLE.
- Connects to a LUX sensor via a serial port.
- Reads and logs LUX values from both sensors.
- Logs synchronized LUX values and timestamps to a single CSV file with separate columns for each sensor.

Usage:
------
- Run the script to start logging synchronized LUX values.
- The data is saved in the 'logs' directory as 'synchronized_lux_data.csv'.

Requirements:
-------------
- Python 3.8 or higher.
- bleak library for BLE communication.
- pySerial library for serial communication.
- Ensure that the device UUIDs and serial port match your devices.

License:
--------
This script is released under the "Do What The F*ck You Want To Public License" (Version 2).
"""

import asyncio
import threading
import time
import os
import csv
import re
import serial
from bleak import BleakClient

# BLE configuration
BLE_DEVICE_UUID = "F35544C1-2CF9-1C06-307B-3F9D1F8B5FBC"  # Replace with your BLE device's UUID
DATA_IN_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"     # BLE Data In characteristic UUID
DATA_OUT_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"    # BLE Data Out characteristic UUID

# Serial port configuration
SERIAL_PORT = "/dev/tty.usbserial-A50285BI"  # Replace with your serial port
BAUD_RATE = 115200  # Adjust based on your device's baud rate
SERIAL_TIMEOUT = 1       # Timeout for serial read in seconds

# Log file configuration
LOG_DIR = "logs"
LOG_FILE_NAME = "synchronized_lux_data.csv"
LOG_FILE_PATH = os.path.join(LOG_DIR, LOG_FILE_NAME)

# Polynomial coefficients (a, b, c) for second-order model
POLYNOMIAL_COEFFICIENTS = (0.002, 1.3, -5.0)  # Example coefficients

# Shared variables for sensor readings
latest_ble_lux = None
latest_serial_lux = None
data_lock = threading.Lock()  # Lock to synchronize access to shared variables

class LuxLogger:
    """
    Base class for logging LUX data to a CSV file.
    """
    def __init__(self, log_file_path, polynomial_coefficients):
        self.log_file_path = log_file_path
        self.csv_file = None
        self.csv_writer = None
        self.coefficients = polynomial_coefficients

    def setup_csv(self):
        # Ensure the logs directory exists
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)

        # Open the CSV file for writing data
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        self.csv_writer.writerow(['timestamp', 'ble_lux_value', 'serial_lux_value', 'processed_serial_lux_value'])

    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            print(f"Log file {self.log_file_path} closed.")

    # def process_serial_lux(self, serial_lux):
    #     """
    #     Processes the Serial LUX value using a second-order polynomial model.
    #     """
    #     if serial_lux is None:
    #         return None
    #     a, b, c = self.coefficients
    #     return a * serial_lux**2 + b * serial_lux + c
    def process_serial_lux(self, x):
        """
        Predice el valor de BLE Lux usando un modelo lineal basado en rangos definidos por clusters.

        Parámetros:
            x (float): Valor de Serial Lux.

        Retorna:
            float: Valor predicho de BLE Lux.
        """
        # Coeficientes e interceptos por rango
        if 0.0 <= x <= 217.51:  # Cluster 1
            coef = 2.2223
            intercept = 16.0177
        elif 217.51 <= x <= 671.25:  # Cluster 0
            coef = 2.5976
            intercept = -44.3843
        elif 671.25 <= x <= 1155.8:  # Cluster 2
            coef = -0.1316
            intercept = 1686.7994
        else:
            raise ValueError("El valor de Serial Lux está fuera de los rangos definidos por los clusters.")

        return coef * x + intercept

    def log_data(self, timestamp, ble_lux, serial_lux):
        processed_serial_lux = self.process_serial_lux(serial_lux)
        with data_lock:
            self.csv_writer.writerow([timestamp, ble_lux, serial_lux, processed_serial_lux])
            self.csv_file.flush()  # Ensure data is written to file

class SerialLuxReader(threading.Thread):
    """
    Thread to handle reading from the serial port and updating the latest LUX value.
    """
    def __init__(self, port, baud_rate, timeout):
        threading.Thread.__init__(self)
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self.stop_event = threading.Event()

    def connect(self):
        """
        Connects to the serial port.
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"Connected to serial port: {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to serial port {self.port}: {e}")
            return False

    def disconnect(self):
        """
        Closes the serial connection.
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"Disconnected from serial port: {self.port}")

    def run(self):
        """
        Reads lines from the serial port and updates the latest LUX value.
        """
        try:
            while not self.stop_event.is_set():
                raw_data = self.serial_conn.readline()
                # print(f"[Serial] Raw data: {raw_data}")  # Debug: Print raw bytes
                line = raw_data.decode('utf-8', errors='ignore').strip()
                if line:
                    # print(f"[Serial] Received line: {line}")
                    self.parse_and_update(line)
        except Exception as e:
            print(f"Error reading from serial port: {e}")

    def parse_and_update(self, line):
        """
        Parses a line from the serial output and updates the latest LUX value.
        """
        try:
            # Remove ANSI escape sequences
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            line_clean = ansi_escape.sub('', line)
            # Use regex to match lines with LUX values
            match = re.match(r'^(\d+\.\d+)\s+lux$', line_clean)
            if match:
                lux_value = float(match.group(1))
                with data_lock:
                    global latest_serial_lux
                    latest_serial_lux = lux_value
                # print(f"Updated latest serial LUX value: {lux_value}")
            else:
                # Ignore lines that do not match
                pass
        except Exception as e:
            print(f"Error parsing line '{line}': {e}")

    def stop(self):
        self.stop_event.set()

class BLELuxReader:
    """
    Class to handle connecting to the BLE device and updating the latest LUX value.
    """
    def __init__(self, device_uuid, data_in_uuid, data_out_uuid):
        self.device_uuid = device_uuid
        self.data_in_uuid = data_in_uuid
        self.data_out_uuid = data_out_uuid
        self.client = None

    async def connect(self):
        """
        Connects to the BLE device and sets up notifications.
        """
        self.client = BleakClient(self.device_uuid)
        await self.client.connect()
        if not self.client.is_connected:
            print("Failed to connect to the BLE device.")
            return False

        print(f"Connected to BLE device: {self.device_uuid}")

        # Start notifications
        await self.client.start_notify(self.data_out_uuid, self.notification_handler)
        print(f"Subscribed to BLE notifications on {self.data_out_uuid}.")

        return True

    async def disconnect(self):
        """
        Stops notifications and disconnects from the BLE device.
        """
        if self.client and self.client.is_connected:
            await self.client.stop_notify(self.data_out_uuid)
            await self.client.disconnect()
            print(f"Disconnected from BLE device: {self.device_uuid}")

    async def send_command_periodically(self, command, interval):
        """
        Sends a command to the BLE device periodically.
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
        Handles incoming notifications from the BLE Data Out characteristic.
        Parses and updates the latest LUX value.
        """
        try:
            # Remove ANSI escape sequences if any
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            data_clean = ansi_escape.sub('', data.decode('utf-8', errors='ignore'))
            # Attempt to extract the LUX value from the ASCII data
            # Find the start (':') and end (';') of the ASCII string
            colon_index = data_clean.find(':')
            semicolon_index = data_clean.find(';', colon_index)
            if colon_index != -1 and semicolon_index != -1:
                ascii_data = data_clean[colon_index+1:semicolon_index].strip()
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
                with data_lock:
                    global latest_ble_lux
                    latest_ble_lux = lux_value
                # print(f"Updated latest BLE LUX value: {lux_value}")
            else:
                print("Colon ':' or semicolon ';' not found in BLE data.")
        except Exception as e:
            print(f"Error parsing BLE notification data: {e}")

    async def start_reading(self, command_interval=1):
        """
        Starts the BLE reading process.
        """
        # Start sending command periodically
        command = bytes([0x5E])
        self.send_command_task = asyncio.create_task(
            self.send_command_periodically(command, command_interval)
        )
        print(f"Started sending {command.hex()} to {self.data_in_uuid} every {command_interval} seconds.")

    async def stop(self):
        self.send_command_task.cancel()

async def main():
    # Create instances of the loggers and readers
    logger = LuxLogger(log_file_path=LOG_FILE_PATH, polynomial_coefficients=POLYNOMIAL_COEFFICIENTS)
    logger.setup_csv()

    serial_reader = SerialLuxReader(
        port=SERIAL_PORT,
        baud_rate=BAUD_RATE,
        timeout=SERIAL_TIMEOUT
    )

    ble_reader = BLELuxReader(
        device_uuid=BLE_DEVICE_UUID,
        data_in_uuid=DATA_IN_UUID,
        data_out_uuid=DATA_OUT_UUID
    )

    # Start serial reader thread
    serial_connected = serial_reader.connect()
    if serial_connected:
        serial_reader.start()
    else:
        print("Serial reader failed to connect.")
        return

    # Start BLE reader
    ble_connected = await ble_reader.connect()
    if not ble_connected:
        print("BLE reader failed to connect.")
        serial_reader.stop()
        serial_reader.join()
        return
    await ble_reader.start_reading(command_interval=1)

    # Periodically log the data
    try:
        while True:
            await asyncio.sleep(1)  # Adjust the interval as needed
            with data_lock:
                timestamp = time.time()
                ble_lux = latest_ble_lux
                serial_lux = latest_serial_lux
            processed_serial_lux = logger.process_serial_lux(serial_lux)
            logger.log_data(timestamp, ble_lux, serial_lux)
            print(f"Logged data at {timestamp}: BLE={ble_lux}, Serial={serial_lux}, Processed Serial={processed_serial_lux}")
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Stop both readers and close the logger
        serial_reader.stop()
        serial_reader.join()
        await ble_reader.stop()
        await ble_reader.disconnect()
        logger.close_csv()
        serial_reader.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
