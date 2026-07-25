# Security policy

## Supported versions

Security fixes are provided for the latest release and the current `main`
branch.

## Private reporting

Report vulnerabilities through GitHub's private vulnerability reporting
feature. Include the affected version, realistic impact, a minimal reproduction,
and a proposed mitigation when available.

Do not publish:

- Bluetooth addresses or stable macOS peripheral UUIDs;
- sensitive measurement logs;
- host diagnostics containing nearby devices;
- secrets or personal information.

## Trust boundaries

BLE advertisements and notifications are untrusted local-radio input. The
parser rejects malformed frames, but callers must not use this project as the
sole control or alarm input in safety-critical environments.

CSV output is written to the user-selected path and may disclose occupancy,
working hours, location, or experimental conditions. Protect it accordingly.

Dependency and CodeQL checks reduce risk but do not certify the Bluetooth stack,
meter firmware, radio environment, or measurement accuracy.
