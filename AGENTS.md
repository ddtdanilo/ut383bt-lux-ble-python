# AGENTS.md

## Scope

This repository provides a maintained Python package and CLI for the UNI-T
UT383BT Bluetooth lux meter. Changes must remain safe to test without physical
hardware.

## Structure

- `src/ut383bt/`: maintained package.
- `tests/`: deterministic unit and async tests; never use a live radio.
- `docs/`: architecture, API, protocol, and troubleshooting references.
- `.github/`: repository automation and contribution templates.

## Engineering rules

1. Support Python 3.11 and newer.
2. Keep BLE operations asynchronous.
3. Use an explicit user-provided device identifier; never commit a personal
   address or macOS UUID.
4. Use Bleak's async context manager so disconnection always occurs.
5. Always pass the GATT `response` choice explicitly.
6. Keep parsing pure and independent from BLE so captured packets are testable
   offline.
7. Treat notification data as untrusted input. Bound, validate, and reject it
   without crashing a capture.
8. Store timestamps as timezone-aware UTC.
9. Do not add live-hardware tests to CI.
10. Update public documentation and `CHANGELOG.md` with user-visible behavior.

## Required checks

```bash
python -m pip install --editable ".[dev]"
ruff check .
ruff format --check .
pytest
python -m build
pip-audit
git diff --check
```

Coverage must remain at or above 95%.

## Pull requests

Use signed Conventional Commits and focused PRs. Include the affected operating
systems, sanitized device behavior, validation evidence, and any manual
hardware checks. Never include a real device identifier, sensitive measurement
log, or Bluetooth diagnostic containing personal data.
