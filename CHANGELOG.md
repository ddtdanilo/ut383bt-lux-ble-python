# Changelog

All notable changes to this project are documented here.

## [1.0.0] - 2026-07-25

### Added

- Installable `ut383bt` package and command-line interface.
- BLE discovery, explicit device selection, live reading, CSV logging, and
  offline packet parsing.
- Typed measurement model with timezone-aware UTC timestamps.
- Configurable request bytes, interval, characteristic UUIDs, and GATT write
  response behavior.
- Deterministic unit and async tests with more than 95% branch coverage.
- Complete architecture, API, protocol, troubleshooting, privacy, security,
  support, contribution, citation, agent, and release documentation.
- CI across Python 3.11-3.14, CodeQL, dependency review, packaging, release
  automation, Dependabot, and repository templates.
- The WTFPL Version 2 license file declared by the original scripts.

### Changed

- Updated the supported dependency line to Bleak 3.
- Removed the hard-coded personal device UUID.
- Replaced broad exception handling and direct script globals with reusable,
  tested components.
- CSV output now includes ISO 8601 UTC and epoch timestamps.

### Removed

- Empty placeholder modules and tests.
- Duplicate parsing, connection, and logging implementations.
