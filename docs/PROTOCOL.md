# Observed BLE protocol

## Evidence boundary

The public UNI-T manual documents product operation and Bluetooth app support,
but not the GATT request or notification payload. The values here come from
exploration of one physical meter. Treat them as observed defaults rather than
a vendor guarantee.

## Characteristics

- Data In: `0000ff01-0000-1000-8000-00805f9b34fb`
- Data Out: `0000ff02-0000-1000-8000-00805f9b34fb`

The client subscribes to Data Out and periodically writes `0x5E` to Data In.
The default interval is one second. The CLI permits overrides for research and
firmware variants.

## Notification framing

The parser looks for the first complete ASCII segment between `:` and `;`.
Inside that segment it accepts a non-negative integer followed optionally by
`LUX` or `LX`, case-insensitively.

Examples:

```text
:250 LUX;
:0042lx;
:987;
```

Malformed, non-ASCII, empty, or truncated frames raise
`MeasurementParseError`. The live client logs and isolates that error so a bad
packet does not end the session.

## Unknowns

- maximum sensor reading and overload representation;
- firmware-to-firmware differences;
- whether all devices support writes without response;
- whether payloads include decimals or alternative units in other modes;
- service-level UUID and characteristic property variations.

Capture sanitized packets and open an issue before broadening the parser.
