# Architecture

## Layers

```text
CLI (`cli.py`)
  ├── discovery and BLE lifecycle (`client.py`) ──► Bleak ──► host Bluetooth stack
  ├── pure notification parsing (`parser.py`)
  └── measurement model and CSV sink (`measurement.py`)
```

The parser has no Bluetooth dependency. The client accepts an injectable client
factory and sleep function, allowing async lifecycle behavior to be tested
without a radio. The CLI translates user input into those library interfaces.

## Capture lifecycle

1. The user supplies a device identifier from `scan`.
2. `UT383BTClient` opens `BleakClient` as an async context manager.
3. The client subscribes to Data Out notifications.
4. A background task writes the measurement request to Data In at the requested
   interval.
5. Notifications are copied to immutable bytes, optionally exposed for
   diagnostics, parsed, timestamped, and delivered to the callback.
6. Malformed notifications are logged and ignored.
7. On timeout, cancellation, or error, the request task is cancelled,
   notifications stop, and the async context disconnects.

## Design constraints

- Live hardware is never required in automated tests.
- Device identifiers are runtime inputs, not repository configuration.
- The GATT write response mode is always explicit.
- Physical units are integer lux because that is the observed payload.
- Raw payloads remain available on measurements for diagnostics but are not
  written to CSV.
