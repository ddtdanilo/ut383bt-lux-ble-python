"""Unit tests for measurement models and CSV output."""

from __future__ import annotations

import csv
import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path

from ut383bt.measurement import CsvMeasurementLogger, LuxMeasurement


class LuxMeasurementTests(unittest.TestCase):
    def test_now_uses_utc(self) -> None:
        measurement = LuxMeasurement.now(17, b":17 LUX;")

        self.assertEqual(measurement.lux, 17)
        self.assertEqual(measurement.captured_at.tzinfo, UTC)

    def test_epoch_seconds(self) -> None:
        measurement = LuxMeasurement(
            lux=17,
            captured_at=datetime(2025, 1, 2, 3, 4, 5, tzinfo=UTC),
            raw_payload=b":17 LUX;",
        )
        self.assertEqual(measurement.epoch_seconds, 1735787045.0)


class CsvMeasurementLoggerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.path = Path(self.temporary_directory.name) / "nested" / "measurements.csv"
        self.measurement = LuxMeasurement(
            lux=250,
            captured_at=datetime(2025, 1, 2, 3, 4, 5, 123456, tzinfo=UTC),
            raw_payload=b":250 LUX;",
        )

    def read_rows(self) -> list[list[str]]:
        with self.path.open(encoding="utf-8", newline="") as stream:
            return list(csv.reader(stream))

    def test_create_parent_write_header_and_row(self) -> None:
        with CsvMeasurementLogger(self.path) as logger:
            logger.write(self.measurement)

        self.assertEqual(
            self.read_rows(),
            [
                ["timestamp_utc", "epoch_seconds", "lux"],
                ["2025-01-02T03:04:05.123456+00:00", "1735787045.123456", "250"],
            ],
        )

    def test_append_without_duplicate_header(self) -> None:
        with CsvMeasurementLogger(self.path) as logger:
            logger.write(self.measurement)
        with CsvMeasurementLogger(self.path) as logger:
            logger.write(self.measurement)

        self.assertEqual(len(self.read_rows()), 3)

    def test_write_requires_context_manager(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "context manager"):
            CsvMeasurementLogger(self.path).write(self.measurement)

    def test_close_even_when_context_raises(self) -> None:
        logger = CsvMeasurementLogger(self.path)
        with self.assertRaisesRegex(RuntimeError, "stop"), logger:
            raise RuntimeError("stop")

        self.assertIsNone(logger._stream)


if __name__ == "__main__":
    unittest.main()
