"""Command-line interface for discovery, reading, logging, and offline parsing."""

from __future__ import annotations

import argparse
import asyncio
import logging
from collections.abc import Sequence
from pathlib import Path

from ut383bt import __version__
from ut383bt.client import (
    DEFAULT_COMMAND_INTERVAL,
    DEFAULT_DATA_IN_UUID,
    DEFAULT_DATA_OUT_UUID,
    UT383BTClient,
    format_discovered_devices,
    scan_devices,
)
from ut383bt.exceptions import UT383BTError
from ut383bt.measurement import CsvMeasurementLogger, LuxMeasurement
from ut383bt.parser import parse_notification


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def _hex_bytes(value: str) -> bytes:
    try:
        parsed = bytes.fromhex(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("command must be valid hexadecimal") from error
    if not parsed:
        raise argparse.ArgumentTypeError("command must not be empty")
    return parsed


def _add_connection_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--device", required=True, help="BLE address or macOS UUID from `scan`")
    parser.add_argument("--duration", type=_positive_float, default=30.0, help="capture seconds")
    parser.add_argument(
        "--interval",
        type=_positive_float,
        default=DEFAULT_COMMAND_INTERVAL,
        help="seconds between measurement requests",
    )
    parser.add_argument(
        "--command", type=_hex_bytes, default=b"\x5e", help="request command as hex"
    )
    parser.add_argument("--data-in-uuid", default=DEFAULT_DATA_IN_UUID)
    parser.add_argument("--data-out-uuid", default=DEFAULT_DATA_OUT_UUID)
    parser.add_argument(
        "--write-with-response",
        action="store_true",
        help="request acknowledged GATT writes; off by default",
    )
    parser.add_argument("--raw", action="store_true", help="print raw notifications to stderr")


def build_parser() -> argparse.ArgumentParser:
    """Build the public argument parser."""

    parser = argparse.ArgumentParser(
        prog="ut383bt",
        description="Read and log UNI-T UT383BT illuminance measurements over BLE.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("-v", "--verbose", action="count", default=0)
    subparsers = parser.add_subparsers(dest="operation", required=True)

    scan = subparsers.add_parser("scan", help="discover nearby BLE peripherals")
    scan.add_argument("--timeout", type=_positive_float, default=5.0)

    read = subparsers.add_parser("read", help="print measurements")
    _add_connection_arguments(read)

    log = subparsers.add_parser("log", help="append measurements to CSV")
    _add_connection_arguments(log)
    log.add_argument("--output", type=Path, default=Path("lux_data.csv"))

    parse = subparsers.add_parser("parse", help="parse one captured notification offline")
    parse.add_argument("payload", type=_hex_bytes, help="notification as hexadecimal")
    return parser


def _client_from_args(args: argparse.Namespace) -> UT383BTClient:
    return UT383BTClient(
        args.device,
        data_in_uuid=args.data_in_uuid,
        data_out_uuid=args.data_out_uuid,
        command=args.command,
        command_interval=args.interval,
        write_with_response=args.write_with_response,
    )


def _print_measurement(measurement: LuxMeasurement) -> None:
    print(f"{measurement.captured_at.isoformat()}\t{measurement.lux} lx")


def _raw_printer(sender: object, payload: bytes) -> None:
    logging.getLogger(__name__).info("notification from %s: %s", sender, payload.hex())


async def run_async(args: argparse.Namespace) -> int:
    """Execute one parsed command."""

    if args.operation == "scan":
        print(format_discovered_devices(await scan_devices(args.timeout)))
        return 0
    if args.operation == "parse":
        _print_measurement(parse_notification(args.payload))
        return 0

    client = _client_from_args(args)
    raw_callback = _raw_printer if args.raw else None
    if args.operation == "read":
        count = await client.collect(args.duration, _print_measurement, raw_callback=raw_callback)
    else:
        with CsvMeasurementLogger(args.output) as csv_logger:
            count = await client.collect(args.duration, csv_logger.write, raw_callback=raw_callback)
        print(f"Wrote {count} measurement(s) to {args.output}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments and run the requested operation."""

    parser = build_parser()
    args = parser.parse_args(argv)
    level = logging.DEBUG if args.verbose > 1 else logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=level, format="%(levelname)s: %(message)s")
    try:
        return asyncio.run(run_async(args))
    except UT383BTError as error:
        parser.exit(2, f"error: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted.\n")
    return 2


def entrypoint() -> None:
    """Console-script wrapper."""

    raise SystemExit(main())
