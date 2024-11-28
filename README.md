
# UT383BT-LUX-BLE-PYTHON

A Python project for interacting with the Uni-T UT383BT Bluetooth LUX meter using BLE (Bluetooth Low Energy). This project enables real-time retrieval and optional logging of light intensity values (in LUX) from the UT383BT device. It uses the `bleak` library for cross-platform BLE communication, specifically tested on macOS.

## Features

- Connects to the Uni-T UT383BT LUX meter via BLE.
- Sends a command to enable notifications for LUX values.
- Subscribes to the `Data Out` characteristic to receive real-time measurements.
- Logs retrieved LUX values to a file for later analysis (optional).
- Modular structure for extensibility and testing.

## Getting Started

### Prerequisites

1. Python 3.8 or higher.
2. Install the required Python packages listed in `requirements.txt`.

### Setting Up a Virtual Environment

It is recommended to use a Python virtual environment to manage dependencies for this project.

#### Steps to Create and Activate a Virtual Environment:

1. **Create the Virtual Environment:**
   ```bash
   python3 -m venv venv
   ```

2. **Activate the Virtual Environment:**
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```
   - On Windows:
     ```bash
     .\venv\Scripts\activate
     ```

3. **Install Dependencies:**
   Once the virtual environment is activated, install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

4. **Deactivate the Virtual Environment:**
   When you're done, deactivate the environment:
   ```bash
   deactivate
   ```

### Tested Environment

- **OS**: macOS (tested with BLE using LightBlue for initial device exploration).
- **Device**: Uni-T UT383BT Bluetooth LUX meter.
- **Python Library**: `bleak` (for BLE communication).

### Folder Structure

```plaintext
ut383bt-lux-ble-python/
├── README.md              # Project documentation
├── LICENSE                # License for the project
├── requirements.txt       # Python dependencies
├── src/                   # Source code
│   ├── __init__.py        # Package initialization
│   ├── lux_reader.py      # Reads LUX values via BLE
│   ├── lux_logger.py      # Logs LUX values to a file
│   └── utils.py           # Utility functions (optional)
├── logs/                  # Log files
│   └── lux_data_example.log # Example of logged LUX data
└── tests/                 # Unit tests
    ├── __init__.py        # Package initialization
    └── test_lux_reader.py # Unit tests for lux_reader.py
```

## Usage

### 1. Reading LUX Values
Run the `lux_reader.py` script to connect to the UT383BT device and print LUX values to the console:
```bash
python src/lux_reader.py
```

### 2. Logging LUX Values
Run the `lux_logger.py` script to log LUX values to a file in the `logs` directory:
```bash
python src/lux_logger.py
```

### 3. Example Log File
After running `lux_logger.py`, a file like `logs/lux_data_example.log` will be generated with entries similar to:
```plaintext
75LUX
80LUX
85LUX
```

### 4. Running Tests
Use the `tests` folder to validate the functionality of your scripts:
```bash
python -m unittest discover tests
```

## Requirements

All dependencies are listed in `requirements.txt`. Install them using:
```bash
pip install -r requirements.txt
```

### Example `requirements.txt`:
```plaintext
bleak==0.20.2
```

## How It Works

1. **Connection**: The script connects to the UT383BT via BLE using its UUID or MAC address.
2. **Command to Enable Notifications**: A specific command (`0x5E`) is written to the `Data In` characteristic.
3. **Receiving Notifications**: LUX values are sent as notifications via the `Data Out` characteristic.
4. **Logging (Optional)**: The values are logged to a file for later use.

## Contributions

Contributions are welcome! Feel free to fork the repository, open an issue, or submit a pull request.
