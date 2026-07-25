# Troubleshooting

## The meter does not appear in `scan`

- Confirm Bluetooth is enabled on the meter and host.
- Close the vendor app and other BLE tools; many peripherals allow one
  connection at a time.
- Grant Bluetooth permission to the terminal or Python application.
- Increase `--timeout`.
- On Linux, verify BlueZ is running and your user can access the adapter.

## The connection fails

- Copy the identifier from a fresh scan.
- Remember that a macOS UUID is not a portable Bluetooth address.
- Move closer and replace low batteries.
- Disconnect the device from other phones or computers.
- Retry after toggling host Bluetooth.

## Notifications arrive but no values print

Run with verbose raw diagnostics:

```bash
ut383bt -v read --device "IDENTIFIER" --raw --duration 10
```

Review output locally before sharing it. If packets use a different shape, open
an issue with sanitized hexadecimal payloads.

## Writes fail

Try acknowledged GATT writes:

```bash
ut383bt read --device "IDENTIFIER" --write-with-response
```

Bleak documents that characteristic properties determine whether writes with or
without response are supported. This project defaults to the behavior observed
on the original meter and makes it configurable.

## Linux permissions

Exact configuration varies by distribution and BlueZ packaging. Prefer the
distribution's documented Bluetooth group, udev, D-Bus, or capability setup.
Do not run untrusted Python packages as root.
