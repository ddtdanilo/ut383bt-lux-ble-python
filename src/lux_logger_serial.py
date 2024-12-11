#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synchronized Lux Logger for BLE and Serial Sensors
==================================================

Este script:
- Lee datos LUX desde un dispositivo BLE.
- Lee datos LUX desde un sensor serial (que imprime línea con MUESTRA, LUX, FULL, IR, ITIME, GAIN, ATIME, AGAIN, CPL).
- Las líneas seriales pueden indicar saturación con SATURADO y campos N/A.
- Procesa el valor serial_lux con un modelo lineal por rangos.
- Registra todo en un CSV.
- El número de muestra (MUESTRA) ahora se extrae directamente de la línea serial, no se genera internamente.

Requisitos previos:
- bleak
- pySerial

Asegúrate de reemplazar los UUID, el puerto serial y otros ajustes según tu hardware.
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
BLE_DEVICE_UUID = "F35544C1-2CF9-1C06-307B-3F9D1F8B5FBC"  # Reemplaza con el UUID de tu dispositivo BLE
DATA_IN_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"    # Característica BLE Data In
DATA_OUT_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"   # Característica BLE Data Out

# Serial port configuration
SERIAL_PORT = "/dev/tty.usbserial-A50285BI"  # Reemplaza con tu puerto serial
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1

# Log file configuration
LOG_DIR = "logs"
LOG_FILE_NAME = "synchronized_lux_data.csv"
LOG_FILE_PATH = os.path.join(LOG_DIR, LOG_FILE_NAME)

# Coeficientes para procesar el lux serial (modelo ya definido)
POLYNOMIAL_COEFFICIENTS = (0.002, 1.3, -5.0)

# Variables compartidas
latest_ble_lux = None
latest_muestra_id = None
latest_serial_lux = None
latest_full = None
latest_ir = None
latest_itime = None
latest_gain = None
latest_atime = None
latest_again = None
latest_cpl = None

data_lock = threading.Lock()

class LuxLogger:
    """
    Clase para registrar datos en CSV con columnas adicionales.
    """
    def __init__(self, log_file_path, polynomial_coefficients):
        self.log_file_path = log_file_path
        self.csv_file = None
        self.csv_writer = None
        self.coefficients = polynomial_coefficients

    def setup_csv(self):
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'muestra_id',
            'timestamp',
            'ble_lux_value',
            'serial_lux_value',
            'processed_serial_lux_value',
            'full',
            'ir',
            'itime',
            'gain',
            'atime',
            'again',
            'cpl'
        ])

    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            print(f"Log file {self.log_file_path} closed.")

    def process_serial_lux(self, x):
        """
        Procesa el lux serial con un modelo lineal por rangos.
        Si x es None o fuera de rango, retorna None.
        """
        if x is None:
            return None
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
            return None
        return coef * x + intercept

    def log_data(self, muestra_id, timestamp, ble_lux, serial_lux, full, ir, itime, gain, atime, again, cpl):
        processed_serial_lux = self.process_serial_lux(serial_lux)
        with data_lock:
            self.csv_writer.writerow([
                muestra_id,
                timestamp,
                ble_lux,
                serial_lux,
                processed_serial_lux,
                full,
                ir,
                itime,
                gain,
                atime,
                again,
                cpl
            ])
            self.csv_file.flush()

class SerialLuxReader(threading.Thread):
    """
    Hilo para leer del puerto serial y parsear tanto líneas normales como saturadas, incluyendo MUESTRA.
    """
    def __init__(self, port, baud_rate, timeout):
        threading.Thread.__init__(self)
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self.stop_event = threading.Event()

    def connect(self):
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
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"Disconnected from serial port: {self.port}")

    def run(self):
        try:
            while not self.stop_event.is_set():
                raw_data = self.serial_conn.readline()
                line = raw_data.decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_and_update(line)
        except Exception as e:
            print(f"Error reading from serial port: {e}")

    def parse_float_or_na(self, val):
        if val == 'N/A':
            return None
        try:
            return float(val)
        except:
            return None

    def parse_and_update(self, line):
        try:
            # Remover secuencias ANSI
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            line_clean = ansi_escape.sub('', line)

            # Patrón para saturado con MUESTRA
            saturado_pattern = re.compile(
                r'^MUESTRA=(\d+),\s*SATURADO:\s*Lux=(-?\d+\.\d+),\s*FULL=(\d+),\s*IR=(\d+),\s*ITIME=(\d+),\s*GAIN=(\d+),\s*ATIME=([N/A\d\.]+),\s*AGAIN=([N/A\d\.]+),\s*CPL=([N/A\d\.]+)'
            )
            # Patrón para línea normal con MUESTRA
            normal_pattern = re.compile(
                r'^MUESTRA=(\d+),\s*LUX=([\d\.]+),\s*FULL=(\d+),\s*IR=(\d+),\s*ITIME=(\d+),\s*GAIN=(\d+),\s*ATIME=([N/A\d\.]+),\s*AGAIN=([N/A\d\.]+),\s*CPL=([N/A\d\.]+)'
            )

            match_saturado = saturado_pattern.search(line_clean)
            match_normal = normal_pattern.search(line_clean)

            muestra_value = None
            lux_value = None
            full = None
            ir = None
            itime = None
            gain = None
            atime = None
            again = None
            cpl = None

            if match_saturado:
                # Línea saturada
                muestra_value = int(match_saturado.group(1))
                lux_value = float(match_saturado.group(2))
                full = int(match_saturado.group(3))
                ir = int(match_saturado.group(4))
                itime = int(match_saturado.group(5))
                gain = int(match_saturado.group(6))
                atime = self.parse_float_or_na(match_saturado.group(7))
                again = self.parse_float_or_na(match_saturado.group(8))
                cpl = self.parse_float_or_na(match_saturado.group(9))

            elif match_normal:
                # Línea normal
                muestra_value = int(match_normal.group(1))
                lux_value = float(match_normal.group(2))
                full = int(match_normal.group(3))
                ir = int(match_normal.group(4))
                itime = int(match_normal.group(5))
                gain = int(match_normal.group(6))
                atime = self.parse_float_or_na(match_normal.group(7))
                again = self.parse_float_or_na(match_normal.group(8))
                cpl = self.parse_float_or_na(match_normal.group(9))

            else:
                # Línea no reconocida, ignorar
                return

            with data_lock:
                global latest_muestra_id, latest_serial_lux, latest_full, latest_ir, latest_itime, latest_gain, latest_atime, latest_again, latest_cpl
                latest_muestra_id = muestra_value
                latest_serial_lux = lux_value
                latest_full = full
                latest_ir = ir
                latest_itime = itime
                latest_gain = gain
                latest_atime = atime
                latest_again = again
                latest_cpl = cpl

        except Exception as e:
            print(f"Error parsing line '{line}': {e}")

    def stop(self):
        self.stop_event.set()

class BLELuxReader:
    """
    Lector BLE que obtiene datos LUX del dispositivo BLE.
    """
    def __init__(self, device_uuid, data_in_uuid, data_out_uuid):
        self.device_uuid = device_uuid
        self.data_in_uuid = data_in_uuid
        self.data_out_uuid = data_out_uuid
        self.client = None

    async def connect(self):
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
        if self.client and self.client.is_connected:
            await self.client.stop_notify(self.data_out_uuid)
            await self.client.disconnect()
            print(f"Disconnected from BLE device: {self.device_uuid}")

    async def send_command_periodically(self, command, interval):
        try:
            while True:
                await self.client.write_gatt_char(self.data_in_uuid, command)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error sending command: {e}")

    def notification_handler(self, sender, data):
        try:
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            data_clean = ansi_escape.sub('', data.decode('utf-8', errors='ignore'))
            colon_index = data_clean.find(':')
            semicolon_index = data_clean.find(';', colon_index)
            if colon_index != -1 and semicolon_index != -1:
                ascii_data = data_clean[colon_index+1:semicolon_index].strip()
                match = re.search(r'(\d+)\s*LUX', ascii_data)
                if match:
                    lux_value = int(match.group(1))
                else:
                    digits = ''.join(filter(str.isdigit, ascii_data))
                    if digits:
                        lux_value = int(digits)
                    else:
                        print("No LUX value found in BLE data.")
                        return
                with data_lock:
                    global latest_ble_lux
                    latest_ble_lux = lux_value
            else:
                print("Colon ':' or semicolon ';' not found in BLE data.")
        except Exception as e:
            print(f"Error parsing BLE notification data: {e}")

    async def start_reading(self, command_interval=1):
        command = bytes([0x5E])
        self.send_command_task = asyncio.create_task(
            self.send_command_periodically(command, command_interval)
        )
        print(f"Started sending {command.hex()} to {self.data_in_uuid} every {command_interval} seconds.")

    async def stop(self):
        self.send_command_task.cancel()

async def main():
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

    # Iniciar el lector serial
    serial_connected = serial_reader.connect()
    if serial_connected:
        serial_reader.start()
    else:
        print("Serial reader failed to connect.")
        return

    # Iniciar el lector BLE
    ble_connected = await ble_reader.connect()
    if not ble_connected:
        print("BLE reader failed to connect.")
        serial_reader.stop()
        serial_reader.join()
        return
    await ble_reader.start_reading(command_interval=1)

    # Loguear datos periódicamente
    try:
        while True:
            await asyncio.sleep(1)
            with data_lock:
                timestamp = time.time()
                muestra_id = latest_muestra_id
                ble_lux = latest_ble_lux
                serial_lux = latest_serial_lux
                full = latest_full
                ir = latest_ir
                itime = latest_itime
                gain = latest_gain
                atime = latest_atime
                again = latest_again
                cpl = latest_cpl

            # Sólo loggeamos si ya recibimos al menos una línea con MUESTRA
            if muestra_id is not None:
                logger.log_data(muestra_id, timestamp, ble_lux, serial_lux, full, ir, itime, gain, atime, again, cpl)
                print(f"MUESTRA={muestra_id}, Logged data at {timestamp}: BLE={ble_lux}, Serial={serial_lux}, FULL={full}, IR={ir}, ITIME={itime}, GAIN={gain}, ATIME={atime}, AGAIN={again}, CPL={cpl}")

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Detener ambos lectores y cerrar el logger
        serial_reader.stop()
        serial_reader.join()
        await ble_reader.stop()
        await ble_reader.disconnect()
        logger.close_csv()
        serial_reader.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
