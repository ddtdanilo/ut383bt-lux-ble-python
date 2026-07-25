"""Unit tests for the command-line interface."""

from __future__ import annotations

import contextlib
import io
import tempfile
import unittest
from argparse import ArgumentTypeError, Namespace
from pathlib import Path
from unittest.mock import AsyncMock, patch

from ut383bt.cli import _hex_bytes, _positive_float, build_parser, main, run_async
from ut383bt.client import DiscoveredDevice
from ut383bt.exceptions import ConfigurationError
from ut383bt.measurement import LuxMeasurement


class ArgumentTests(unittest.TestCase):
    def test_positive_float(self) -> None:
        self.assertEqual(_positive_float("1.5"), 1.5)

    def test_reject_non_positive_float(self) -> None:
        for value in ("0", "-1"):
            with self.subTest(value=value), self.assertRaisesRegex(ArgumentTypeError, "greater"):
                _positive_float(value)

    def test_hex_bytes(self) -> None:
        self.assertEqual(_hex_bytes("5e"), b"\x5e")

    def test_reject_invalid_hex(self) -> None:
        with self.assertRaisesRegex(ArgumentTypeError, "hexadecimal"):
            _hex_bytes("zz")

    def test_reject_empty_hex(self) -> None:
        with self.assertRaisesRegex(ArgumentTypeError, "empty"):
            _hex_bytes("")

    def test_parser_defaults(self) -> None:
        args = build_parser().parse_args(["read", "--device", "ABC"])

        self.assertEqual(args.duration, 30.0)
        self.assertEqual(args.interval, 1.0)
        self.assertEqual(args.command, b"\x5e")
        self.assertFalse(args.write_with_response)


class AsyncCommandTests(unittest.IsolatedAsyncioTestCase):
    async def test_scan_command(self) -> None:
        args = Namespace(operation="scan", timeout=1.0)
        output = io.StringIO()
        with (
            patch(
                "ut383bt.cli.scan_devices",
                new=AsyncMock(
                    return_value=[DiscoveredDevice(name="Meter", address="ABC", rssi=-40)]
                ),
            ),
            contextlib.redirect_stdout(output),
        ):
            result = await run_async(args)

        self.assertEqual(result, 0)
        self.assertIn("Meter\tABC\t-40", output.getvalue())

    async def test_parse_command(self) -> None:
        args = Namespace(operation="parse", payload=b":88 LUX;")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = await run_async(args)

        self.assertEqual(result, 0)
        self.assertIn("\t88 lx", output.getvalue())

    async def test_read_command(self) -> None:
        args = self.connection_args(operation="read")
        measurement = LuxMeasurement.now(55, b":55 LUX;")
        output = io.StringIO()

        async def collect(duration: float, callback: object, *, raw_callback: object) -> int:
            self.assertEqual(duration, 2.0)
            self.assertIsNotNone(raw_callback)
            callback(measurement)  # type: ignore[operator]
            return 1

        with (
            patch("ut383bt.cli.UT383BTClient") as client_type,
            contextlib.redirect_stdout(output),
        ):
            client_type.return_value.collect = collect
            result = await run_async(args)

        self.assertEqual(result, 0)
        self.assertIn("55 lx", output.getvalue())
        client_type.assert_called_once()

    async def test_log_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "data.csv"
            args = self.connection_args(operation="log", output=output_path, raw=False)
            measurement = LuxMeasurement.now(77, b":77 LUX;")
            output = io.StringIO()

            async def collect(duration: float, callback: object, *, raw_callback: object) -> int:
                del duration, raw_callback
                callback(measurement)  # type: ignore[operator]
                return 1

            with (
                patch("ut383bt.cli.UT383BTClient") as client_type,
                contextlib.redirect_stdout(output),
            ):
                client_type.return_value.collect = collect
                result = await run_async(args)

            self.assertEqual(result, 0)
            self.assertIn("Wrote 1 measurement", output.getvalue())
            self.assertIn(",77\n", output_path.read_text(encoding="utf-8"))

    @staticmethod
    def connection_args(**overrides: object) -> Namespace:
        values: dict[str, object] = {
            "operation": "read",
            "device": "ABC",
            "duration": 2.0,
            "interval": 1.0,
            "command": b"\x5e",
            "data_in_uuid": "in",
            "data_out_uuid": "out",
            "write_with_response": False,
            "raw": True,
        }
        values.update(overrides)
        return Namespace(**values)


class MainTests(unittest.TestCase):
    def test_offline_parse_main(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = main(["parse", "3a3132204c55583b"])

        self.assertEqual(result, 0)
        self.assertIn("12 lx", output.getvalue())

    def test_expected_error_exits_with_two(self) -> None:
        with (
            patch("ut383bt.cli.run_async", new=AsyncMock(side_effect=ConfigurationError("bad"))),
            self.assertRaises(SystemExit) as error,
            contextlib.redirect_stderr(io.StringIO()),
        ):
            main(["scan"])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
