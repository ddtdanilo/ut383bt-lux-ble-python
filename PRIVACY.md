# Privacy

The project has no analytics, accounts, telemetry service, or network backend.
It communicates locally through the host Bluetooth stack and writes only the
CSV file explicitly requested by the user.

## Data processed

- nearby BLE device names and identifiers during `scan`;
- notifications from the selected meter;
- UTC timestamps and lux values when logging;
- raw payloads in memory, or in diagnostic output when `--raw` is enabled.

## User responsibilities

- Do not publish scans containing third-party devices.
- Treat Bluetooth addresses and macOS peripheral UUIDs as potentially
  identifying.
- Store measurement logs according to the sensitivity of the location and
  experiment.
- Use `--raw` only while troubleshooting and review output before sharing it.
- Delete local CSV files when they are no longer needed.

The project does not transmit measurement data beyond the selected local BLE
connection.
