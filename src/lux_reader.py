#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for Connecting to UT383BT LUX Meter via Bluetooth on macOS
=================================================================

This script is designed to interface with the Uni-T UT383BT Bluetooth LUX meter
on macOS using the `bleak` library. It reads real-time LUX values by periodically
sending a specific command (`0x5E`) to the `Data In` characteristic and then
subscribing to the `Data Out` characteristic to receive data.

Context:
--------
- Developed on macOS.
- Device used: Uni-T UT383BT Bluetooth LUX meter.
- BLE characteristics identified using the LightBlue app on macOS.
- Writing `0x5E` to the `Data In` characteristic enables notifications on the
  `Data Out` characteristic, which sends LUX values.
- The device requires the `0x5E` command to be sent periodically (every 0.5 seconds)
  to continue sending LUX data.

License:
---------
This script is released under the "Do What The F*ck You Want To Public License"
(Version 2). This license allows you to freely use, modify, and distribute this
script for any purpose. Attribution is appreciated but not required.

DO WHAT THE F*CK YOU WANT TO PUBLIC LICENSE
Version 2, December 2004

Everyone is permitted to copy and distribute verbatim or modified copies of this
license document, and changing it is allowed as long as the name is changed.

TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION, AND MODIFICATION:
0. You just DO WHAT THE F*CK YOU WANT TO.

"""

import asyncio
import re
from bleak import BleakClient

# UUIDs for the UT383BT Device
DEVICE_UUID = "5D769E40-0CCD-8741-F6C7-0A5D76800EEB"  # Replace with your device's UUID if different
DATA_IN_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # UUID for the Data In characteristic
DATA_OUT_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"  # UUID for the Data Out characteristic

def notification_handler(sender, data):
    """
    Handles incoming notifications from the Data Out characteristic.
    Parses and prints the LUX values received from the UT383BT device.

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
                print(f"Received LUX Data: {lux_value} LUX")
            else:
                # If 'LUX' is not present, extract digits directly
                digits = ''.join(filter(str.isdigit, ascii_data))
                if digits:
                    lux_value = int(digits)
                    print(f"Received LUX Data: {lux_value} LUX")
                else:
                    print("No LUX value found in data.")
        else:
            print("Colon ':' or semicolon ';' not found in data.")
    except Exception as e:
        print(f"Error parsing notification data: {e}")

async def send_command_periodically(client, uuid, command, interval):
    """
    Sends a command to the device periodically.

    Args:
        client (BleakClient): The BLE client connected to the device.
        uuid (str): The UUID of the characteristic to write to.
        command (bytes): The command to send.
        interval (float): The interval in seconds between commands.

    Returns:
        None
    """
    while True:
        try:
            await client.write_gatt_char(uuid, command)
            await asyncio.sleep(interval)
        except Exception as e:
            print(f"Error sending command: {e}")
            break

async def main():
    """
    Main function to connect to the UT383BT device, enable notifications,
    and print LUX values received via the Data Out characteristic.

    Returns:
        None
    """
    async with BleakClient(DEVICE_UUID) as client:
        if not client.is_connected:
            print("Failed to connect to the device.")
            return

        print(f"Connected to device: {DEVICE_UUID}")

        # Subscribe to notifications on the Data Out characteristic
        await client.start_notify(DATA_OUT_UUID, notification_handler)
        print(f"Subscribed to notifications on {DATA_OUT_UUID}.")

        # Start sending 0x5E to Data In every X seconds
        command = bytes([0x5E])
        interval = 1  # Interval in seconds
        send_command_task = asyncio.create_task(
            send_command_periodically(client, DATA_IN_UUID, command, interval)
        )
        print(f"Started sending {command.hex()} to {DATA_IN_UUID} every {interval} seconds.")

        # Keep the script running to receive notifications
        try:
            await asyncio.sleep(30)  # Adjust duration as needed
        finally:
            send_command_task.cancel()
            await client.stop_notify(DATA_OUT_UUID)
            print(f"Unsubscribed from {DATA_OUT_UUID}.")

# Run the asyncio event loop
if __name__ == "__main__":
    asyncio.run(main())
