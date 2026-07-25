"""Measurement models and CSV logging."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import TracebackType
from typing import TextIO


@dataclass(frozen=True, slots=True)
class LuxMeasurement:
    """One parsed illuminance measurement."""

    lux: int
    captured_at: datetime
    raw_payload: bytes

    @classmethod
    def now(cls, lux: int, raw_payload: bytes) -> LuxMeasurement:
        """Create a measurement with a timezone-aware UTC timestamp."""

        return cls(lux=lux, captured_at=datetime.now(UTC), raw_payload=raw_payload)

    @property
    def epoch_seconds(self) -> float:
        """Return the capture timestamp as Unix epoch seconds."""

        return self.captured_at.timestamp()


class CsvMeasurementLogger:
    """Append measurements to a UTF-8 CSV file and flush each row."""

    HEADER = ("timestamp_utc", "epoch_seconds", "lux")

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self._stream: TextIO | None = None
        self._writer: object | None = None

    def __enter__(self) -> CsvMeasurementLogger:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        is_empty = not self.path.exists() or self.path.stat().st_size == 0
        self._stream = self.path.open("a", encoding="utf-8", newline="")
        self._writer = csv.writer(self._stream)
        if is_empty:
            self._writer.writerow(self.HEADER)  # type: ignore[union-attr]
            self._stream.flush()
        return self

    def write(self, measurement: LuxMeasurement) -> None:
        """Append and immediately flush one measurement."""

        if self._stream is None or self._writer is None:
            raise RuntimeError("CSV logger must be used as a context manager")
        self._writer.writerow(  # type: ignore[union-attr]
            (
                measurement.captured_at.isoformat(),
                f"{measurement.epoch_seconds:.6f}",
                measurement.lux,
            )
        )
        self._stream.flush()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        if self._stream is not None:
            self._stream.close()
        self._stream = None
        self._writer = None
