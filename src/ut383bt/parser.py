"""Parse illuminance values from observed UT383BT BLE notifications."""

from __future__ import annotations

import re

from ut383bt.exceptions import UT383BTError
from ut383bt.measurement import LuxMeasurement

_LUX_PATTERN = re.compile(r"(?<!\d)(\d{1,9})\s*(?:LUX|LX)?(?![A-Z0-9])", re.IGNORECASE)


class MeasurementParseError(UT383BTError, ValueError):
    """Raised when a notification does not contain a usable lux value."""


def _extract_frame(payload: bytes) -> bytes:
    start = payload.find(b":")
    if start < 0:
        raise MeasurementParseError("notification does not contain ':'")
    end = payload.find(b";", start + 1)
    if end < 0:
        raise MeasurementParseError("notification does not contain ';' after ':'")
    if end == start + 1:
        raise MeasurementParseError("notification frame is empty")
    return payload[start + 1 : end]


def parse_lux(payload: bytes | bytearray) -> int:
    """Extract a non-negative integer lux value from one notification."""

    if not isinstance(payload, (bytes, bytearray)):
        raise TypeError("notification payload must be bytes-like")
    frame = _extract_frame(bytes(payload))
    try:
        text = frame.decode("ascii")
    except UnicodeDecodeError as error:
        raise MeasurementParseError("notification frame is not valid ASCII") from error

    match = _LUX_PATTERN.search(text.strip())
    if match is None:
        raise MeasurementParseError("notification frame does not contain a lux value")
    return int(match.group(1))


def parse_notification(payload: bytes | bytearray) -> LuxMeasurement:
    """Parse a BLE notification and timestamp it in UTC."""

    raw_payload = bytes(payload)
    return LuxMeasurement.now(parse_lux(raw_payload), raw_payload)
