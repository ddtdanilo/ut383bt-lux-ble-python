# API reference

## Discovery

```python
from ut383bt import scan_devices

devices = await scan_devices(5.0)
```

Returns sorted `DiscoveredDevice` values with `name`, platform-specific
`address`, and optional `rssi`.

## Client

```python
from ut383bt import UT383BTClient

client = UT383BTClient(
    "DEVICE_IDENTIFIER",
    command_interval=1.0,
    write_with_response=False,
)
count = await client.collect(30.0, callback)
```

`collect()` requires a positive duration and synchronous callback accepting one
`LuxMeasurement`. It returns the count of valid parsed measurements. An
optional `raw_callback(sender, payload)` receives every raw notification before
parsing.

Expected BLE failures are raised as `DeviceConnectionError`. Invalid user
configuration raises `ConfigurationError`.

## Parsing

```python
from ut383bt import parse_notification
from ut383bt.parser import parse_lux

value = parse_lux(b":250 LUX;")
measurement = parse_notification(b":250 LUX;")
```

`parse_lux()` returns an integer. `parse_notification()` also records a UTC
timestamp and immutable raw payload. Invalid data raises
`MeasurementParseError`.

## CSV

```python
from ut383bt.measurement import CsvMeasurementLogger

with CsvMeasurementLogger("measurements/lux.csv") as logger:
    await client.collect(30.0, logger.write)
```

The logger creates parent directories, writes a header only for a new or empty
file, appends rows, and flushes each measurement.
