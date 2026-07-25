"""Unit tests for BLE orchestration with no physical hardware."""

from __future__ import annotations

import asyncio
import unittest
from dataclasses import dataclass
from unittest.mock import AsyncMock, patch

from bleak.exc import BleakError

from ut383bt.client import (
    DiscoveredDevice,
    UT383BTClient,
    format_discovered_devices,
    scan_devices,
)
from ut383bt.exceptions import ConfigurationError, DeviceConnectionError


class FakeClient:
    def __init__(
        self,
        device: str,
        *,
        connected: bool = True,
        enter_error: Exception | None = None,
    ) -> None:
        self.device = device
        self.is_connected = connected
        self.enter_error = enter_error
        self.notifications_started: list[str] = []
        self.notifications_stopped: list[str] = []
        self.writes: list[tuple[str, bytes, bool]] = []
        self.exited = False

    async def __aenter__(self) -> FakeClient:
        if self.enter_error is not None:
            raise self.enter_error
        return self

    async def __aexit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.exited = True

    async def start_notify(self, characteristic: str, callback: object) -> None:
        self.notifications_started.append(characteristic)
        callback("fake-characteristic", bytearray(b":123 LUX;"))  # type: ignore[operator]
        callback("fake-characteristic", bytearray(b"malformed"))  # type: ignore[operator]

    async def stop_notify(self, characteristic: str) -> None:
        self.notifications_stopped.append(characteristic)

    async def write_gatt_char(self, characteristic: str, data: bytes, *, response: bool) -> None:
        self.writes.append((characteristic, data, response))
        await asyncio.sleep(0)


async def yielding_sleep(delay: float) -> None:
    del delay
    await asyncio.sleep(0)


class UT383BTClientConfigurationTests(unittest.TestCase):
    def test_reject_empty_device(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "device"):
            UT383BTClient(" ")

    def test_reject_non_positive_interval(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "interval"):
            UT383BTClient("device", command_interval=0)

    def test_reject_empty_command(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "command"):
            UT383BTClient("device", command=b"")


class UT383BTClientAsyncTests(unittest.IsolatedAsyncioTestCase):
    async def test_collect_measurement_and_clean_up(self) -> None:
        fake = FakeClient("device")
        measurements = []
        raw = []
        client = UT383BTClient(
            "device",
            client_factory=lambda _device: fake,
            sleep=yielding_sleep,
        )

        count = await client.collect(
            0.01,
            measurements.append,
            raw_callback=lambda sender, payload: raw.append((sender, payload)),
        )

        self.assertEqual(count, 1)
        self.assertEqual(measurements[0].lux, 123)
        self.assertEqual(len(raw), 2)
        self.assertTrue(fake.exited)
        self.assertEqual(fake.notifications_started, [client.data_out_uuid])
        self.assertEqual(fake.notifications_stopped, [client.data_out_uuid])
        self.assertGreaterEqual(len(fake.writes), 1)
        self.assertEqual(fake.writes[0], (client.data_in_uuid, b"\x5e", False))

    async def test_write_with_response(self) -> None:
        fake = FakeClient("device")
        client = UT383BTClient(
            "device",
            write_with_response=True,
            client_factory=lambda _device: fake,
            sleep=yielding_sleep,
        )

        await client.collect(0.01, lambda _measurement: None)

        self.assertTrue(fake.writes[0][2])

    async def test_reject_non_positive_duration(self) -> None:
        client = UT383BTClient("device")
        with self.assertRaisesRegex(ConfigurationError, "duration"):
            await client.collect(0, lambda _measurement: None)

    async def test_reject_disconnected_client(self) -> None:
        fake = FakeClient("device", connected=False)
        client = UT383BTClient("device", client_factory=lambda _device: fake)

        with self.assertRaisesRegex(DeviceConnectionError, "unable to connect"):
            await client.collect(0.01, lambda _measurement: None)

    async def test_wrap_bleak_errors(self) -> None:
        fake = FakeClient("device", enter_error=BleakError("radio unavailable"))
        client = UT383BTClient("device", client_factory=lambda _device: fake)

        with self.assertRaisesRegex(DeviceConnectionError, "radio unavailable"):
            await client.collect(0.01, lambda _measurement: None)


@dataclass
class FakeDevice:
    name: str | None
    address: str


@dataclass
class FakeAdvertisement:
    rssi: int


class DiscoveryTests(unittest.IsolatedAsyncioTestCase):
    async def test_scan_and_sort_devices(self) -> None:
        results = {
            "b": (FakeDevice("Zeta", "B"), FakeAdvertisement(-60)),
            "a": (FakeDevice("Alpha", "A"), FakeAdvertisement(-40)),
        }
        with patch(
            "ut383bt.client.BleakScanner.discover",
            new=AsyncMock(return_value=results),
        ) as discover:
            devices = await scan_devices(1.5)

        discover.assert_awaited_once_with(timeout=1.5, return_adv=True)
        self.assertEqual(
            devices,
            [
                DiscoveredDevice(name="Alpha", address="A", rssi=-40),
                DiscoveredDevice(name="Zeta", address="B", rssi=-60),
            ],
        )

    async def test_scan_rejects_non_positive_timeout(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "timeout"):
            await scan_devices(0)

    def test_format_devices(self) -> None:
        value = format_discovered_devices(
            [
                DiscoveredDevice(name="Meter", address="ABC", rssi=-55),
                DiscoveredDevice(name=None, address="DEF", rssi=None),
            ]
        )
        self.assertEqual(
            value,
            "NAME\tIDENTIFIER\tRSSI\nMeter\tABC\t-55\n(unnamed)\tDEF\tn/a",
        )

    def test_format_empty_devices(self) -> None:
        self.assertEqual(format_discovered_devices([]), "No BLE devices found.")


if __name__ == "__main__":
    unittest.main()
