"""Cross-platform tools for the UNI-T UT383BT Bluetooth lux meter."""

from ut383bt.client import (
    DEFAULT_COMMAND,
    DEFAULT_COMMAND_INTERVAL,
    DEFAULT_DATA_IN_UUID,
    DEFAULT_DATA_OUT_UUID,
    DiscoveredDevice,
    UT383BTClient,
    scan_devices,
)
from ut383bt.measurement import LuxMeasurement
from ut383bt.parser import MeasurementParseError, parse_notification

__all__ = [
    "DEFAULT_COMMAND",
    "DEFAULT_COMMAND_INTERVAL",
    "DEFAULT_DATA_IN_UUID",
    "DEFAULT_DATA_OUT_UUID",
    "DiscoveredDevice",
    "LuxMeasurement",
    "MeasurementParseError",
    "UT383BTClient",
    "parse_notification",
    "scan_devices",
]

__version__ = "1.0.0"
