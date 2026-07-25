# CLAUDE.md

Follow [`AGENTS.md`](AGENTS.md) as the authoritative repository guide.

This is an async BLE project. Keep Bluetooth orchestration injectable and test
it with fakes; keep packet parsing pure; require users to select a device
explicitly; and guarantee notification and connection cleanup.

Run the complete quality, coverage, build, and dependency-audit commands from
`AGENTS.md` before proposing changes. Do not use a physical Bluetooth adapter in
automated tests or commit device identifiers and captured private data.
