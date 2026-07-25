# Contributing

Thank you for improving UT383BT Lux over BLE.

## Setup

```bash
git clone https://github.com/ddtdanilo/ut383bt-lux-ble-python.git
cd ut383bt-lux-ble-python
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --editable ".[dev]"
```

On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1`.

## Quality checks

```bash
ruff check .
ruff format --check .
pytest
python -m build
pip-audit
git diff --check
```

Tests must not scan for or connect to live BLE devices. Inject a fake client or
patch the scanner. Maintain at least 95% branch coverage.

## Hardware observations

Open an issue before changing the observed protocol. Provide:

- meter model and firmware information, if exposed;
- host OS and version;
- Bleak and Python versions;
- sanitized hexadecimal notifications;
- whether GATT writes require a response;
- steps that do not reveal a persistent device identifier.

Never commit Bluetooth addresses, macOS peripheral UUIDs, location-sensitive
logs, screenshots containing nearby devices, or system diagnostic archives.

## Code style

- Use type hints and concise docstrings on public interfaces.
- Raise project-specific errors for expected BLE and configuration failures.
- Avoid broad exception handling.
- Keep CLI output stable and useful in scripts.
- Add tests for malformed and boundary inputs.

## Commits and pull requests

Use Conventional Commit subjects:

```text
fix(parser): reject truncated notification frames
docs: clarify Linux Bluetooth permissions
```

Explain behavior, risk, validation, and manual hardware results in the pull
request. By participating, you agree to follow the
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).
