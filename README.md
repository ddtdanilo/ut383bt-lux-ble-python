# UT383BT Lux over BLE

[![CI](https://github.com/ddtdanilo/ut383bt-lux-ble-python/actions/workflows/quality.yml/badge.svg)](https://github.com/ddtdanilo/ut383bt-lux-ble-python/actions/workflows/quality.yml)
[![CodeQL](https://github.com/ddtdanilo/ut383bt-lux-ble-python/actions/workflows/codeql.yml/badge.svg)](https://github.com/ddtdanilo/ut383bt-lux-ble-python/actions/workflows/codeql.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-3776ab.svg)](https://www.python.org/)
[![License: WTFPL](https://img.shields.io/badge/license-WTFPL-blue.svg)](LICENSE)

A tested Python library and command-line tool for discovering, reading, and
logging illuminance measurements from a **UNI-T UT383BT** over Bluetooth Low
Energy.

The project is hardware-independent in CI: parsing, CSV output, BLE lifecycle,
error handling, and the CLI are tested with deterministic fakes. A physical
meter is required only for real discovery and capture.

> [!NOTE]
> The BLE request and notification format are based on observation of one
> UT383BT. They are not documented in the public product manual and may vary
> across firmware revisions.

## Highlights

- Cross-platform BLE through [Bleak](https://bleak.readthedocs.io/)
- No hard-coded device address or macOS UUID
- Explicit `scan`, `read`, `log`, and offline `parse` commands
- Configurable characteristic UUIDs, command, interval, duration, and write mode
- UTC ISO 8601 and Unix timestamps in CSV output
- Malformed notifications are isolated without stopping a capture
- Guaranteed notification cleanup and BLE disconnection
- Python 3.11-3.14 CI, 97%+ branch coverage, Ruff, CodeQL, and dependency audits

## Requirements

- Python 3.11 or newer
- Bluetooth Low Energy hardware enabled on the host
- A UNI-T UT383BT meter with Bluetooth enabled
- OS permission for the terminal or Python process to use Bluetooth

Bleak 3 supports macOS, Windows 11, Linux with a compatible BlueZ stack, and
selected Android environments. This project was originally explored on macOS;
device identifiers differ by platform.

## Install

From the repository:

```bash
git clone https://github.com/ddtdanilo/ut383bt-lux-ble-python.git
cd ut383bt-lux-ble-python
python3 -m venv .venv
source .venv/bin/activate
python -m pip install .
```

For development:

```bash
python -m pip install --editable ".[dev]"
```

## Quick start

### 1. Discover the meter

```bash
ut383bt scan --timeout 8
```

Example:

```text
NAME        IDENTIFIER                            RSSI
UT383BT     11111111-2222-3333-4444-555555555555  -48
```

Use the reported identifier exactly. Linux and Windows commonly show a Bluetooth
address; macOS normally shows a UUID.

### 2. Read measurements

```bash
ut383bt read \
  --device "11111111-2222-3333-4444-555555555555" \
  --duration 30
```

### 3. Log to CSV

```bash
ut383bt log \
  --device "11111111-2222-3333-4444-555555555555" \
  --duration 60 \
  --output measurements/lux.csv
```

CSV format:

```csv
timestamp_utc,epoch_seconds,lux
2026-07-25T06:30:00.123456+00:00,1784961000.123456,250
```

### 4. Parse a captured notification offline

```bash
ut383bt parse "3a323530204c55583b"
```

That hexadecimal payload represents `:250 LUX;`.

All commands are also available through:

```bash
python -m ut383bt --help
```

## Observed protocol

| Purpose | UUID / value |
| --- | --- |
| Data In characteristic | `0000ff01-0000-1000-8000-00805f9b34fb` |
| Data Out characteristic | `0000ff02-0000-1000-8000-00805f9b34fb` |
| Periodic request | `0x5E` |
| Default request interval | 1 second |
| Observed payload shape | ASCII content between `:` and `;`, containing an integer lux value |

The CLI uses writes without response by default and specifies that choice
explicitly, as recommended for modern Bleak. If a firmware revision exposes only
acknowledged writes, add `--write-with-response`.

See [Protocol notes](docs/PROTOCOL.md) before changing UUIDs or parser behavior.

## Library usage

```python
import asyncio

from ut383bt import UT383BTClient


def display(measurement):
    print(measurement.captured_at.isoformat(), measurement.lux)


async def main():
    client = UT383BTClient("YOUR-BLE-IDENTIFIER")
    await client.collect(duration=30, callback=display)


asyncio.run(main())
```

The complete public surface is documented in [API reference](docs/API.md).

## Development

```bash
python -m pip install --editable ".[dev]"
ruff check .
ruff format --check .
pytest
python -m build
pip-audit
```

No test connects to a real Bluetooth peripheral. Hardware verification is an
explicit manual step described in [Contributing](CONTRIBUTING.md).

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [API reference](docs/API.md)
- [Protocol notes](docs/PROTOCOL.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Privacy](PRIVACY.md)
- [Security](SECURITY.md)
- [Contributing](CONTRIBUTING.md)
- [Support](SUPPORT.md)

## Privacy and safety

BLE scans expose nearby device names and identifiers to the local process. CSV
logs reveal when and where measurements were taken. Review
[`PRIVACY.md`](PRIVACY.md), avoid publishing identifiers or sensitive logs, and
never use this utility as the sole input to a safety-critical lighting system.

## License

Released under the [WTFPL, Version 2](LICENSE), preserving the license declared
by the original scripts.
