"""Project-specific exception types."""


class UT383BTError(Exception):
    """Base exception for expected UT383BT failures."""


class DeviceConnectionError(UT383BTError):
    """Raised when the requested BLE peripheral cannot be connected."""


class ConfigurationError(UT383BTError):
    """Raised when runtime configuration is invalid."""
