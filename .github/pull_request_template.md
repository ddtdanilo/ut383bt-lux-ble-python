## Summary

<!-- What changed and why? -->

## Validation

- [ ] `ruff check .`
- [ ] `ruff format --check .`
- [ ] `pytest`
- [ ] `python -m build`
- [ ] `pip-audit`
- [ ] `git diff --check`

## BLE and privacy

- [ ] Automated tests use fakes and do not require a Bluetooth radio
- [ ] No persistent device identifiers or sensitive measurement logs are included
- [ ] Protocol changes include sanitized evidence and compatibility notes
- [ ] User-facing behavior and the changelog are updated
