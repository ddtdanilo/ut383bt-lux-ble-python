"""Asynchronous BLE client for the UNI-T UT383BT."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass
from typing import Any, Final, Protocol

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from ut383bt.exceptions import ConfigurationError, DeviceConnectionError
from ut383bt.measurement import LuxMeasurement
from ut383bt.parser import MeasurementParseError, parse_notification

DEFAULT_DATA_IN_UUID: Final = "0000ff01-0000-1000-8000-00805f9b34fb"
DEFAULT_DATA_OUT_UUID: Final = "0000ff02-0000-1000-8000-00805f9b34fb"
DEFAULT_COMMAND: Final = b"\x5e"
DEFAULT_COMMAND_INTERVAL: Final = 1.0

LOGGER = logging.getLogger(__name__)

MeasurementCallback = Callable[[LuxMeasurement], None]
RawCallback = Callable[[Any, bytes], None]


class ClientProtocol(Protocol):
    """The BleakClient surface used by this project."""

    is_connected: bool

    async def __aenter__(self) -> ClientProtocol: ...

    async def __aexit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None: ...

    async def start_notify(self, characteristic: str, callback: Callable[..., Any]) -> None: ...

    async def stop_notify(self, characteristic: str) -> None: ...

    async def write_gatt_char(
        self, characteristic: str, data: bytes, *, response: bool
    ) -> None: ...


ClientFactory = Callable[[str], ClientProtocol]
SleepFunction = Callable[[float], Awaitable[None]]


@dataclass(frozen=True, slots=True)
class DiscoveredDevice:
    """Stable public representation of a discovered BLE peripheral."""

    name: str | None
    address: str
    rssi: int | None = None


async def scan_devices(scan_duration: float = 5.0) -> list[DiscoveredDevice]:
    """Discover nearby BLE devices and return a stable, sorted representation."""

    if scan_duration <= 0:
        raise ConfigurationError("scan timeout must be greater than zero")
    devices = await BleakScanner.discover(timeout=scan_duration, return_adv=True)
    discovered = [
        DiscoveredDevice(
            name=device.name,
            address=device.address,
            rssi=getattr(advertisement, "rssi", None),
        )
        for device, advertisement in devices.values()
    ]
    return sorted(discovered, key=lambda item: ((item.name or "").casefold(), item.address))


class UT383BTClient:
    """Collect and parse notifications from one explicitly selected meter."""

    def __init__(  # noqa: PLR0913
        self,
        device: str,
        *,
        data_in_uuid: str = DEFAULT_DATA_IN_UUID,
        data_out_uuid: str = DEFAULT_DATA_OUT_UUID,
        command: bytes = DEFAULT_COMMAND,
        command_interval: float = DEFAULT_COMMAND_INTERVAL,
        write_with_response: bool = False,
        client_factory: ClientFactory = BleakClient,
        sleep: SleepFunction = asyncio.sleep,
    ) -> None:
        if not device.strip():
            raise ConfigurationError("device identifier must not be empty")
        if command_interval <= 0:
            raise ConfigurationError("command interval must be greater than zero")
        if not command:
            raise ConfigurationError("command must not be empty")

        self.device = device
        self.data_in_uuid = data_in_uuid
        self.data_out_uuid = data_out_uuid
        self.command = bytes(command)
        self.command_interval = command_interval
        self.write_with_response = write_with_response
        self._client_factory = client_factory
        self._sleep = sleep

    async def _send_commands(self, client: ClientProtocol) -> None:
        while True:
            await client.write_gatt_char(
                self.data_in_uuid,
                self.command,
                response=self.write_with_response,
            )
            await self._sleep(self.command_interval)

    async def collect(
        self,
        duration: float,
        callback: MeasurementCallback,
        *,
        raw_callback: RawCallback | None = None,
    ) -> int:
        """Collect measurements for ``duration`` seconds and return their count."""

        if duration <= 0:
            raise ConfigurationError("duration must be greater than zero")

        count = 0

        def notification_handler(sender: Any, data: bytearray) -> None:
            nonlocal count
            payload = bytes(data)
            if raw_callback is not None:
                raw_callback(sender, payload)
            try:
                measurement = parse_notification(payload)
            except MeasurementParseError as error:
                LOGGER.warning("Ignored malformed UT383BT notification: %s", error)
                return
            callback(measurement)
            count += 1

        try:
            async with self._client_factory(self.device) as client:
                if not client.is_connected:
                    raise DeviceConnectionError(f"unable to connect to BLE device {self.device!r}")

                await client.start_notify(self.data_out_uuid, notification_handler)
                command_task = asyncio.create_task(self._send_commands(client))
                try:
                    await self._sleep(duration)
                finally:
                    command_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await command_task
                    await client.stop_notify(self.data_out_uuid)
        except DeviceConnectionError:
            raise
        except BleakError as error:
            raise DeviceConnectionError(
                f"BLE operation failed for device {self.device!r}: {error}"
            ) from error

        return count


def format_discovered_devices(devices: Sequence[DiscoveredDevice]) -> str:
    """Format scan results as a deterministic text table."""

    if not devices:
        return "No BLE devices found."
    rows = ["NAME\tIDENTIFIER\tRSSI"]
    rows.extend(
        f"{device.name or '(unnamed)'}\t{device.address}\t"
        f"{device.rssi if device.rssi is not None else 'n/a'}"
        for device in devices
    )
    return "\n".join(rows)
