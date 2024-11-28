#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for Connecting to UT383BT LUX Meter via Bluetooth on macOS
=================================================================

This script is designed to interface with the Uni-T UT383BT Bluetooth LUX meter 
on macOS using the `bleak` library. It allows you to read real-time LUX values by 
writing a specific command (0x5E) to the `Data In` characteristic to enable 
notifications and then subscribing to the `Data Out` characteristic to receive 
data.

Context:
--------
- This script was developed on macOS.
- The device used is the Uni-T UT383BT Bluetooth LUX meter.
- The initial exploration of the device's BLE characteristics was performed using 
  the LightBlue app on macOS, which helped identify the `Data In` and `Data Out` 
  characteristics under the primary service `0xFF12`.
- Writing `0x5E` to the `Data In` characteristic enables notifications on the 
  `Data Out` characteristic, which sends LUX values.

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
from bleak import BleakClient

# UUIDs for the UT383BT Device
DEVICE_UUID = "5D769E40-0CCD-8741-F6C7-0A5D76800EEB"  # UUID of the UT383BT device
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
        # Assuming the LUX data is sent as a UTF-8 encoded string (e.g., "75LUX")
        lux_data = data.decode("utf-8").strip()
        print(f"Received LUX Data: {lux_data}")
    except Exception as e:
        print(f"Error decoding notification data: {e}")

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

        # Write 0x5E to Data In to enable notifications
        await client.write_gatt_char(DATA_IN_UUID, bytes([0x5E]))
        print(f"Written 0x5E to {DATA_IN_UUID} to enable notifications.")

        # Subscribe to notifications on the Data Out characteristic
        await client.start_notify(DATA_OUT_UUID, notification_handler)
        print(f"Subscribed to notifications on {DATA_OUT_UUID}.")

        # Keep the script running to receive notifications
        try:
            await asyncio.sleep(30)  # Adjust duration as needed
        finally:
            await client.stop_notify(DATA_OUT_UUID)
            print(f"Unsubscribed from {DATA_OUT_UUID}.")

# Run the asyncio event loop
if __name__ == "__main__":
    asyncio.run(main())
